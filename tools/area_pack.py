#!/usr/bin/env python3
"""OSM bbox -> deterministic darter area pack (CLI, M1 start).

Parsing core ported from anisoptera (AGPL-3.0-or-later, same licence family):
training/osm_buildings.py (Overpass fetch/cache, height parsing, point_in_poly)
and ue/suburb_gen.py (raw ways+nodes -> buildings/roads/strips/grass/trees,
pitched-roof geometry, tree scatter, ground span). The source's OBJ+MTL
emission is REPLACED by a versioned pack:

    <out>/pack.json   structured scene, the source of truth (schema below)
    <out>/scene.obj   deterministic meshes derived from pack.json alone
                      (metre units, triangles, o-groups + material names,
                      no UVs — textures are a later task, CC0 per VISION)

Determinism contract: same input file + same --seed/--lat/--lon => byte
identical pack.json, scene.obj and terrain.bin. Mechanism: sorted JSON
keys, floats rounded to fixed precision before writing (3 decimals for
coordinates, 2-3 for heights and radii), OBJ verts formatted %.3f, one
seeded Mersenne Twister consumed in one fixed order, one fixed iteration
order over the input elements. pack.json records the input file's sha256
and the seed, so a pack is reproducible from its own provenance.

Elevation: "flat" packs stay at z=0 ("elevation": {"model": "flat"}).
"glo30" samples Copernicus GLO-30 GeoTIFFs (30 m) onto a node grid whose
z=0 datum is the DEM height at the grid origin, writes that grid as a
sidecar terrain.bin (the Rust core's ground-contact source), and drapes
the scene meshes onto the grid with the same surface offsets the flat
pack uses. pack.json's elevation block records model/datum/dimensions and
per-tile sha256 provenance for attribution. The GLO-30 COGs decode with
tifffile (tags only, no imagecodecs dependency) + stdlib zlib + numpy:
their float32 pages carry the deflate + floating-point predictor, which
system tifffile cannot decode alone. DEM files come from --dem-file
(offline/CI) or the tile cache under --dem-cache (default tools/
elevation_cache, populated by the live fetch when missing).

Frame: metres, x = east, y = north, z = up, origin at (--lat, --lon) — the
same convention as the anisoptera sources.

Usage:
  area_pack.py --osm CACHE.json --lat 50.8989 --lon -1.0586 [--seed 5] --out DIR
                     [--elevation flat|glo30 [--dem-file PATH] [--dem-cache DIR]]
  area_pack.py --fetch RADIUS_M --lat L --lon L [--osm CACHE.json] --out DIR
  area_pack.py DIR                      (validate a written pack dir)
"""
import argparse
import hashlib
import json
import math
import os
import random
import struct
import sys
import datetime
import urllib.error
import urllib.parse
import urllib.request

SCHEMA = "darter_area_pack"
# v2: counts.dashes + `o dash_` OBJ groups with material paint_white (v1 packs
# are rejected by the version check below — rebuild them).
# v3: elevation grows a glo30 model — sidecar terrain.bin (56-byte header +
# f64 payload, read by the Rust core), draped OBJ (ground becomes a per-cell
# triangle grid, sinks under buildings/strips, segmented ladder on roads) and
# the elevation provenance block (v2 packs are rejected — rebuild them).
VERSION = 3
M_PER_DEG_LAT = 111_320.0
OVERPASS_URL = "https://overpass-api.de/api/interpreter"
MIRROR_URL = "https://overpass.kumi.systems/api/interpreter"
ATTRIBUTION = "© OpenStreetMap contributors, ODbL 1.0"

# ---- scene parameters (ported from ue/suburb_gen.py) ------------------------
HOUSE_WALL = 5.4        # 2-storey eaves height, m
HOUSE_ROOF = 2.8        # ridge above eaves
GARAGE_WALL = 2.4
GARAGE_ROOF = 0.9
BIG_WALL = 6.5          # >200 m2 footprints (flats/shops), flat roof
ROOF_PITCH_LIMIT = 15.0  # deg opposite-edge misalignment for pitched roofs
ROAD_WIDTHS = {"primary": 8.0, "secondary": 7.0, "tertiary": 6.5,
               "residential": 5.5, "unclassified": 5.0, "service": 3.5,
               "footway": 1.2, "path": 1.2}
HEDGE_W, HEDGE_H = 0.7, 1.3
FENCE_W, FENCE_H = 0.12, 1.1
TREE_H = 2.3            # trunk top
SCATTER_PER_M2 = 1.0 / 120.0
MAX_SCATTER_TREES = 1500
GROUND_PAD = 200.0      # ground quad padding beyond the scene extent

GARAGE_TAGS = {"garage", "garages", "roof", "shed", "container", "carport",
               "hut"}
FLAT_TAGS = {"commercial", "retail", "industrial", "apartments", "hotel",
             "office", "church", "bakehouse", "public", "school",
             "warehouse"}
RESI_TAGS = {"house", "semidetached_house", "detached", "terrace",
             "residential", "bungalow", "yes", "", None}
WALL_MATS_PLAIN = ["brick_red_plain", "brick_buff_plain", "render_white_plain"]
WALL_MATS_WIN = ["brick_red", "brick_buff", "render_white",
                 "brick_red", "render_white"]
ROOF_MATS = ["tile_brown", "slate"]
ALL_WALL_MATS = WALL_MATS_PLAIN + WALL_MATS_WIN
FOLIAGE_VARIANTS = ["foliage_a", "foliage_b", "foliage_c", "foliage_d"]
SURFACES = {"asphalt", "concrete"}

# S3b lane-centre dashes: white paint on roads wide enough to carry two lanes
# (ROAD_WIDTHS values >= 5.0; footway/path at 1.2 and service at 3.5 fall out
# of the comparison — the width predicate is the whole exclusion rule).
DASH_MIN_WIDTH = 5.0
DASH_LEN = 3.0        # dash length along the centreline, m
DASH_GAP = 6.0        # gap, m (stride 9.0)
DASH_HALF_W = 0.09    # 0.18 m paint width
DASH_INSET = 2.0      # start inset from each polyline node, m
DASH_Z_CAP = 0.070    # never above: junction pads sit at 0.075, 5 mm clear

# ---- terrain (glo30) elevation ----------------------------------------------
# Ground contact plane for the physics core and the drawn ground surface: a
# node grid over the same equirectangular frame as the pack, sampled
# nearest-pixel from Copernicus GLO-30 GeoTIFFs, z datum = DEM height at the
# grid origin node (0,0) so z=0 is the ground the pack was centred on.
TERRAIN_STEP_M = 30.0   # node spacing; ~1 GLO-30 pixel, stable on slopes
ROAD_SUBDIV_M = 15.0    # max road sub-slice length when draping (half-pixel)
STRIP_SINK_M = 0.10     # hedges/fences buried 100 mm into sloped ground
BUILD_SINK_M = 0.75     # buildings translated down 750 mm (sunk on slopes)
GLO30_URL_TEMPLATE = (
    "https://copernicus-dem-30m.s3.amazonaws.com/"
    "Copernicus_DSM_COG_10_{lat}_00_{lon}_00_DEM/"
    "Copernicus_DSM_COG_10_{lat}_00_{lon}_00_DEM.tif")
GLO30_LICENCE = "© Copernicus DEM / ESA (GLO-30)"
TERRAIN_MAGIC = 0x31524E54          # b"TNR1" little-endian
TERRAIN_FMT_VERSION = 1             # terrain.bin internal format version
TERRAIN_HEADER_LEN = 56             # u32 magic + fmt + cols + rows, 5 x f64


# ---- small geometry helpers (ported) ----------------------------------------
def signed_area(ring):
    s = 0.0
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return 0.5 * s


def ensure_ccw(ring):
    return ring if signed_area(ring) > 0 else ring[::-1]


def edge_angle_deg(a1, a2, b1, b2):
    """Angle between edge (a1->a2) and (b1->b2) in degrees."""
    v1 = (a2[0] - a1[0], a2[1] - a1[1])
    v2 = (b2[0] - b1[0], b2[1] - b1[1])
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 180.0
    c = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _tri_sign(p, a, b):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def point_in_tri(p, a, b, c, eps=1e-9):
    d1, d2, d3 = _tri_sign(p, a, b), _tri_sign(p, b, c), _tri_sign(p, c, a)
    has_neg = d1 < -eps or d2 < -eps or d3 < -eps
    has_pos = d1 > eps or d2 > eps or d3 > eps
    return not (has_neg and has_pos)


def triangulate_ring(ring):
    """Ear-clip a simple CCW ring into index triples (ported verbatim)."""
    n = len(ring)
    if n == 3:
        return [(0, 1, 2)]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    idx = list(range(n))
    tris = []
    guard = 0
    while len(idx) > 3:
        guard += 1
        if guard > 4 * n * n:               # non-simple ring: fan as fallback
            for k in range(1, len(idx) - 1):
                tris.append((idx[0], idx[k], idx[k + 1]))
            return tris
        for m in range(len(idx)):
            i0, i1, i2 = idx[m - 1], idx[m], idx[(m + 1) % len(idx)]
            if cross(ring[i0], ring[i1], ring[i2]) <= 1e-12:
                continue                    # reflex or degenerate corner
            if any(point_in_tri(ring[j], ring[i0], ring[i1], ring[i2])
                   for j in idx if j not in (i0, i1, i2)):
                continue                    # other vertices inside the ear
            tris.append((i0, i1, i2))
            idx.pop(m)
            break
        else:                               # no ear found: fan the rest
            for k in range(1, len(idx) - 1):
                tris.append((idx[0], idx[k], idx[k + 1]))
            return tris
    tris.append(tuple(idx))
    return tris


def point_in_poly(x, y, ring):
    """Ray-casting test; ring is [(x, y), ...] open polygon (ported)."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


# ---- DEM (Copernicus GLO-30 GeoTIFF) ----------------------------------------
# Self-contained reader: tifffile supplies tag parsing only; the pixel decode
# is ours. This exists because the GLO-30 COGs store float32 with deflate +
# the floating-point predictor (3), which system tifffile cannot decode
# without the imagecodecs package, and we will not add a compiled dependency.

def _dem_import():
    try:
        import tifffile
        import numpy as np
    except ImportError as exc:
        raise ImportError(
            "the glo30 elevation path needs numpy + tifffile; install with "
            "'apt install python3-numpy python3-tifffile' (or pip install "
            "--user numpy tifffile)") from exc
    return tifffile, np


def _undo_fp_predictor(raw, rows, row_w, bps=4):
    """Undo TIFF predictor 3 on float32 pixel data, byte-exact.

    Layout (derived empirically against an imagecodecs ground-truth decode
    of the home GLO-30 tile, 0/4194304 mismatches): each row stores one
    continuous delta stream across the four byte planes in MSB-plane-first
    order, reset at the row start. Cum-sum the whole row, split to planes,
    reverse plane order so plane 0 (big-endian MSB) becomes the float's
    least-significant byte, then reassemble little-endian floats.
    """
    np = _dem_import()[1]
    stream = np.frombuffer(raw, dtype=np.uint8).reshape(rows, row_w * bps).astype(np.int64)
    vals = np.cumsum(stream, axis=1, dtype=np.int64) & 0xFF
    vals = vals.reshape(rows, bps, row_w)[:, ::-1, :]          # msb-first -> LE
    out = np.ascontiguousarray(np.transpose(vals, (0, 2, 1)).astype(np.uint8))
    return np.ascontiguousarray(out).view('<f4').reshape(rows, row_w)


def _segment_bytes(path, offset, count):
    with open(path, "rb") as f:
        f.seek(offset)
        return f.read(count)


def _decode_page(path, page, predictor):
    """Full-resolution pixel decode of one TIFF page (float32, planar 1)."""
    np = _dem_import()[1]
    import zlib
    w = int(page.tags[256].value)          # ImageWidth
    h = int(page.tags[257].value)          # ImageLength
    tiled = 322 in page.tags               # TileWidth
    if tiled:
        tw, th = int(page.tags[322].value), int(page.tags[323].value)
        offsets = page.tags[324].value
        counts = page.tags[325].value
        cols_t = (w + tw - 1) // tw
        rows_t = (h + th - 1) // th
        out = np.empty((rows_t * th, cols_t * tw), dtype='<f4')
        for k in range(len(offsets)):
            seg = _segment_bytes(path, offsets[k], counts[k])
            if comp_is_deflate(page):
                seg = zlib.decompress(seg)
            if predictor == 3:
                arr = _undo_fp_predictor(seg, th, tw)
            else:
                arr = np.frombuffer(seg, '<f4').reshape(th, tw)
            r, c = divmod(k, cols_t)
            out[r * th:r * th + th, c * tw:c * tw + tw] = arr
        return out[:h, :w], w, h
    else:
        offsets = page.tags[273].value     # StripOffsets
        counts = page.tags[279].value      # StripByteCounts
        try:
            rps = int(page.tags[278].value)    # RowsPerStrip
        except KeyError:
            rps = h
        out = np.empty((h, w), dtype='<f4')
        for k in range(len(offsets)):
            seg = _segment_bytes(path, offsets[k], counts[k])
            if comp_is_deflate(page):
                seg = zlib.decompress(seg)
            if predictor == 3:
                arr = _undo_fp_predictor(seg, rps, w)
            else:
                arr = np.frombuffer(seg, '<f4').reshape(rps, w)
            r0 = k * rps
            out[r0:min(r0 + rps, h)] = arr[:min(rps, h - r0)]
        return out, w, h


def comp_is_deflate(page):
    return int(page.tags[259].value) == 8   # Compression: deflate


def _tag_value(page, code, name):
    try:
        return page.tags[code].value
    except KeyError:
        raise ValueError(
            "%s: DEM GeoTIFF is missing tag %s (%s); only GeoTIFF DEMs are "
            "supported" % (name, code, _TAG_NAMES.get(code))) from None


_TAG_NAMES = {
    256: "ImageWidth", 257: "ImageLength", 258: "BitsPerSample",
    259: "Compression", 277: "SamplesPerPixel", 317: "Predictor",
    322: "TileWidth", 323: "TileLength", 33550: "ModelPixelScale",
    33922: "ModelTiepoint", 34735: "GeoKeyDirectory"}


def read_dem(path):
    """GeoTIFF heights + georeference -> dict, or a loud error.

    Returns {"sx","sy","tlx","tly","w","h","heights","sha256"}: pixel scale
    in degrees (x, y), the NW corner tiepoint, full-res float32 heights
    row-major, and the file's sha256 (provenance).
    """
    tifffile, np = _dem_import()
    name = os.path.basename(path)
    with tifffile.TiffFile(path) as tf:
        page = tf.pages[0]
        if page.is_mask or page.dtype.itemsize != 4 or page.dtype.kind != "f":
            raise ValueError(
                "%s: DEM pixels must be float32 (found %s)" % (name, page.dtype))
        if int(page.tags[277].value) != 1:          # SamplesPerPixel
            raise ValueError("%s: DEM must have one sample per pixel" % name)
        comp = int(page.tags[259].value)
        if comp not in (1, 8):
            raise ValueError(
                "%s: unsupported DEM compression %d (only raw/deflate)"
                % (name, comp))
        predictor = int(page.tags[317].value) if 317 in page.tags else 1
        if predictor not in (1, 3):
            raise ValueError(
                "%s: unsupported DEM predictor %d (only none/float)"
                % (name, predictor))
        sx, sy, _ = _tag_value(page, 33550, name)[:3]
        _, _, _, tl_x, tl_y, _ = _tag_value(page, 33922, name)[:6]
        geokeys = _tag_value(page, 34735, name)
        if sx <= 0.0 or sy <= 0.0:
            raise ValueError("%s: DEM pixel scale must be positive" % name)
        heights, w, h = _decode_page(path, page, predictor)
    if np.isnan(heights).any():
        raise ValueError("%s: DEM contains NaN heights" % name)
    with open(path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    return {"sx": float(sx), "sy": float(sy), "tlx": float(tl_x),
            "tly": float(tl_y), "w": int(w), "h": int(h),
            "heights": heights, "sha256": sha, "geokeys": geokeys}


def _dem_sample(dem, lat, lon):
    """Nearest-pixel DEM height at (lat, lon) (GDAL corner convention:
    the tiepoint is the top-left corner of pixel (0,0))."""
    row = int(math.floor((dem["tly"] - lat) / dem["sy"]))
    col = int(math.floor((lon - dem["tlx"]) / dem["sx"]))
    if not (0 <= row < dem["h"] and 0 <= col < dem["w"]):
        raise ValueError(
            "DEM %s: point lat %.6f lon %.6f is outside the tile "
            "(rows 0..%d cols 0..%d)" % (dem.get("label", dem["sha256"][:8]),
                                         lat, lon, dem["h"] - 1, dem["w"] - 1))
    return float(dem["heights"][row, col])


def glo30_tile_name(lat, lon):
    """GLO-30 tile id from a point inside it: the tile is named for its
    SW corner, so N50_00_W002_00 covers lat [50, 51), lon [-2, -1)."""
    lat_i = math.floor(lat)
    lon_i = math.floor(lon)
    lat_s = ("N%02d" % lat_i) if lat_i >= 0 else ("S%02d" % -lat_i)
    lon_s = ("E%03d" % lon_i) if lon_i >= 0 else ("W%03d" % -lon_i)
    return "%s_00_%s_00" % (lat_s, lon_s)


def _fetch_glo30(tile, cache_dir):
    """Cache-or-GET one GLO-30 tile into cache_dir; returns the path."""
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, tile + ".tif")
    if os.path.exists(path):
        return path
    parts = tile.split("_")
    url = GLO30_URL_TEMPLATE.format(lat=parts[0], lon=parts[2])
    request = urllib.request.Request(
        url, headers={"User-Agent": "darter-area-pack/%d" % VERSION})
    try:
        with urllib.request.urlopen(request, timeout=120) as resp:
            data = resp.read()
    except (urllib.error.URLError, OSError) as exc:
        raise IOError("GLO-30 fetch of tile %s failed: %s" % (tile, exc)) from exc
    if data[:4] not in (b"II*\x00", b"MM\x00*"):
        raise IOError(
            "GLO-30 fetch of tile %s did not return GeoTIFF bytes (first "
            "bytes %r, %d total)" % (tile, bytes(data[:8]), len(data)))
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return path


def build_terrain_grid(lat, lon, span, dem_file=None, cache_dir=None):
    """Node grid for the glo30 elevation model.

    Nodes step TERRAIN_STEP_M over a snapped span (ceil up to whole steps so
    node spacing stays exact), each node sampled nearest-pixel from the DEM;
    heights are stored relative to the DEM height at node (0,0) and rounded
    to 0.1 m as float64. Consumes no RNG (the scene's random streams pass
    untouched). With dem_file set, that single GeoTIFF must cover every
    node; otherwise tiles are fetched from the GLO-30 mirror on demand.

    Returns {"cols", "rows", "step", "origin_x", "origin_y", "z", "z_min",
    "z_max", "tiles", "h"} — h(x, y) is the linear interpolation the OBJ
    drape and (in terrain.rs form) the physics core both use.
    """
    cells = int(math.ceil(span / TERRAIN_STEP_M))
    if cells < 1:
        raise ValueError("terrain span %.1f m below one %d m step"
                         % (span, TERRAIN_STEP_M))
    step = TERRAIN_STEP_M
    span = cells * step
    cols = cells + 1
    rows = cols                     # square span
    origin_x = -span / 2.0
    origin_y = -span / 2.0
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat))

    tcache = {}                     # tile name -> read_dem dict
    tiles = {}                      # provenance name -> file sha256

    if dem_file is not None:
        fixed = read_dem(dem_file)
        fixed["label"] = os.path.basename(dem_file)
        tiles[fixed["label"]] = fixed["sha256"]

    def sample(nlat, nlon):
        if dem_file is not None:
            dem = fixed
        else:
            tn = glo30_tile_name(nlat, nlon)
            if tn in tcache:
                dem = tcache[tn]
            else:
                dem = read_dem(_fetch_glo30(tn, cache_dir))
                dem["label"] = tn + ".tif"
                tcache[tn] = dem
            tiles[dem["label"]] = dem["sha256"]
        return _dem_sample(dem, nlat, nlon)

    _, np = _dem_import()
    raw0 = None
    z = np.empty((rows, cols), dtype="<f8")
    for j in range(rows):
        y = origin_y + j * step
        node_lat = lat + y / M_PER_DEG_LAT
        for i in range(cols):
            x = origin_x + i * step
            node_lon = lon + x / m_per_deg_lon
            raw = sample(node_lat, node_lon)
            if raw0 is None:
                raw0 = raw
            z[j, i] = raw - raw0
    z = np.round(z, 1) + 0.0        # + 0.0 normalises -0.0 away
    grid = {
        "cols": cols, "rows": rows, "step": step,
        "origin_x": origin_x, "origin_y": origin_y,
        "z": z, "z_min": float(np.min(z)), "z_max": float(np.max(z)),
        "tiles": [{"name": n, "sha256": tiles[n]} for n in sorted(tiles)],
    }
    grid["h"] = lambda x, y: terrain_h_at(grid, x, y)
    return grid


def terrain_h_at(grid, x, y):
    """Terrain height by linear interpolation over the node grid.

    Same triangle split as ObjWriter.quad (quads split v00->v11): cells with
    u >= v interpolate the (1-uc, uc-vc, vc) triple, others the (1-vc,
    vc-uc, uc) triple — corners are exact, the shared diagonal is
    continuous, so the drawn surface is the contact surface.
    """
    u = (x - grid["origin_x"]) / grid["step"]
    v = (y - grid["origin_y"]) / grid["step"]
    ic = grid["cols"] - 1
    jc = grid["rows"] - 1
    u = min(max(u, 0.0), float(ic))
    v = min(max(v, 0.0), float(jc))
    i = min(ic - 1, int(u))
    j = min(jc - 1, int(v))
    uc = u - i
    vc = v - j
    z = grid["z"]
    z00 = z[j][i]
    z10 = z[j][i + 1]
    z01 = z[j + 1][i]
    z11 = z[j + 1][i + 1]
    if uc >= vc:
        return (1.0 - uc) * z00 + (uc - vc) * z10 + vc * z11
    return (1.0 - vc) * z00 + (vc - uc) * z01 + uc * z11


def write_terrain_bin(path, grid):
    """terrain.bin: 56-byte LE header, then rows*cols f64-LE row-major."""
    header = struct.pack("<IIII5d", TERRAIN_MAGIC, TERRAIN_FMT_VERSION,
                         grid["cols"], grid["rows"], grid["step"],
                         grid["origin_x"], grid["origin_y"],
                         grid["z_min"], grid["z_max"])
    with open(path, "wb") as f:
        f.write(header)
        grid["z"].tofile(f)


def parse_height_src(tags):
    """(height_m, source) from OSM tags, or (None, 'default')."""
    h = tags.get("height")
    if h:
        try:
            return max(2.5, float(str(h).strip().rstrip("m").strip())), "tag_height"
        except ValueError:
            pass
    lv = tags.get("building:levels")
    if lv:
        try:
            return max(2.5, float(lv) * 3.2), "levels"
        except ValueError:
            pass
    return None, "default"


def classify_building(ring, tags, rng):
    """(bld_class, wall_h, roof_h, roof_shape, wall_mat, roof_mat, h_src).

    Tag-aware classification ported from ue/suburb_gen.py (area only breaks
    ties; garages get windowless facades). roof_shape is normalised to
    gable/hip/flat: the source kept OSM's tagged name (e.g. pyramidal) while
    rendering it flat — the pack records the rendered truth.
    """
    area = abs(signed_area(ring))
    tagged = tags.get("roof:shape", "")
    bt = tags.get("building", "")
    h_src_tag, h_src = parse_height_src(tags)
    if bt in GARAGE_TAGS or area < 35.0:
        wall, roof_h = GARAGE_WALL, GARAGE_ROOF
        shape = tagged or ("gable" if len(ring) == 4 else "flat")
        mat = rng.choice(WALL_MATS_PLAIN)
        bld_class = "garage"
    elif bt in FLAT_TAGS or (bt not in RESI_TAGS and area >= 200.0):
        wall = max(BIG_WALL, h_src_tag if h_src_tag else BIG_WALL)
        roof_h = 0.0
        shape = tagged or "flat"
        mat = rng.choice(WALL_MATS_WIN)
        bld_class = "flat_block"
    else:
        # residential ('yes'/house/semidetached/...): pitched 2-storey
        # regardless of area — a semi pair mapped as one polygon is still
        # a home, not a block of flats (source comment kept).
        wall, roof_h = HOUSE_WALL, HOUSE_ROOF
        shape = tagged or ("gable" if len(ring) == 4 else "flat")
        mat = rng.choice(WALL_MATS_WIN)
        bld_class = "residential"
    if h_src_tag is not None and bld_class != "flat_block":
        wall = max(wall, h_src_tag)
    if len(ring) != 4:
        shape = "flat"
    if shape in ("gable", "hip") and len(ring) == 4:
        ok = (edge_angle_deg(ring[0], ring[1], ring[3], ring[2]) < ROOF_PITCH_LIMIT
              and edge_angle_deg(ring[1], ring[2], ring[0], ring[3]) < ROOF_PITCH_LIMIT)
        if not ok:
            shape = "flat"
    if shape not in ("gable", "hip"):
        shape = "flat"
    roof_mat = rng.choice(ROOF_MATS)
    return bld_class, wall, roof_h, shape, mat, roof_mat, h_src


# ---- scene parsing -----------------------------------------------------------
def parse_scene(data, lat, lon, seed):
    """Raw Overpass elements -> classified scene (ported from build_scene)."""
    rng = random.Random(seed)
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat))

    def to_xy(geom):
        return [((g["lon"] - lon) * m_per_deg_lon,
                 (g["lat"] - lat) * M_PER_DEG_LAT) for g in geom]

    buildings, roads, strips, grass, trees = [], [], [], [], []
    for el in data.get("elements", []):
        tags = el.get("tags", {})
        if el.get("type") == "way" and "building" in tags:
            geom = el.get("geometry") or []
            ring = to_xy(geom)
            if len(ring) > 2 and ring[0] == ring[-1]:
                ring = ring[:-1]
            if len(ring) >= 3:
                buildings.append((ensure_ccw(ring), tags))
        elif el.get("type") == "node" and tags.get("natural") == "tree":
            trees.append(((el["lon"] - lon) * m_per_deg_lon,
                          (el["lat"] - lat) * M_PER_DEG_LAT, "tagged"))
        elif el.get("type") == "way" and "highway" in tags:
            hw = tags["highway"]
            if hw in ROAD_WIDTHS:
                geom = el.get("geometry") or []
                if len(geom) >= 2:
                    roads.append((to_xy(geom), ROAD_WIDTHS[hw],
                                  "concrete" if hw in ("footway", "path")
                                  else "asphalt"))
        elif el.get("type") == "way" and tags.get("barrier") in ("hedge", "fence", "wall"):
            geom = el.get("geometry") or []
            if len(geom) >= 2:
                strips.append((to_xy(geom), tags["barrier"]))
        elif el.get("type") == "way" and (
                tags.get("landuse") in ("grass", "meadow")
                or tags.get("leisure") == "garden"):
            geom = el.get("geometry") or []
            ring = to_xy(geom)
            if len(ring) > 2 and ring[0] == ring[-1]:
                ring = ring[:-1]
            if len(ring) >= 3:
                grass.append(ensure_ccw(ring))

    # scatter extra trees into grass/garden polygons (OSM tagging is sparse)
    n_tagged = len(trees)
    for ring in grass:
        area = abs(signed_area(ring))
        want = int(area * SCATTER_PER_M2)
        for _ in range(want):
            xs = [p[0] for p in ring]
            ys = [p[1] for p in ring]
            for _try in range(20):
                x = rng.uniform(min(xs), max(xs))
                y = rng.uniform(min(ys), max(ys))
                if point_in_poly(x, y, ring):
                    trees.append((x, y, "scattered"))
                    break
    if len(trees) > n_tagged + MAX_SCATTER_TREES:
        extra = trees[n_tagged:]
        rng.shuffle(extra)
        trees = trees[:n_tagged] + extra[:MAX_SCATTER_TREES]

    # per-building classification consumes the rng in building order
    classified = []
    for ring, tags in buildings:
        cls, wall, roof_h, shape, mat, roof_mat, h_src = classify_building(
            ring, tags, rng)
        classified.append((ring, cls, wall, roof_h, shape, mat, roof_mat, h_src))

    # per-tree geometry params, consumed in tree order (draw order matches
    # the source's add_tree: trunk radius, trunk height, variant, crown radius)
    tree_params = []
    for _x, _y, _src in trees:
        trunk_r = rng.uniform(0.10, 0.18)
        trunk_h = TREE_H * rng.uniform(0.85, 1.15)
        variant = FOLIAGE_VARIANTS[rng.randrange(4)]
        crown_r = rng.uniform(1.4, 2.6)
        tree_params.append((trunk_r, trunk_h, variant, crown_r))

    return classified, roads, strips, grass, trees, tree_params, n_tagged


# ---- pack assembly -----------------------------------------------------------
def build_pack(classified, roads, strips, grass, trees, tree_params, n_tagged,
               lat, lon, seed, src_sha, fetched):
    """Assemble the pack dict. All floats are rounded here, once, so the JSON
    is byte-stable and self-consistent (bounds/counts recomputed from the
    rounded values validate exactly)."""
    buildings = []
    for i, (ring, cls, wall, roof_h, shape, mat, roof_mat, h_src) in enumerate(classified):
        buildings.append({
            "id": f"b{i}",
            "ring": [[round(x, 3), round(y, 3)] for x, y in ring],
            "bld_class": cls,
            "wall_h": round(wall, 2),
            "roof_h": round(roof_h, 2),
            "roof_shape": shape,
            "wall_mat": mat,
            "roof_mat": roof_mat,
            "height_source": h_src,
        })
    roads_out = []
    for i, (geom, width, surface) in enumerate(roads):
        roads_out.append({
            "id": f"r{i}",
            "polyline": [[round(x, 3), round(y, 3)] for x, y in geom],
            "width_m": round(width, 2),
            "surface": surface,
        })
    strips_out = []
    for i, (geom, kind) in enumerate(strips):
        strips_out.append({
            "id": f"s{i}",
            "polyline": [[round(x, 3), round(y, 3)] for x, y in geom],
            "kind": kind,
        })
    # S3b: roads wide enough to carry centre dashes. Counted from the rounded
    # widths (the same values the OBJ emission gates on), so the JSON count
    # and the geometry cannot disagree.
    dashes = sum(1 for r in roads_out if r["width_m"] >= DASH_MIN_WIDTH)
    grass_out = []
    for i, ring in enumerate(grass):
        grass_out.append({
            "id": f"g{i}",
            "ring": [[round(x, 3), round(y, 3)] for x, y in ring],
        })
    trees_out = []
    for i, ((x, y, src), (tr, th, variant, cr)) in enumerate(zip(trees, tree_params)):
        trees_out.append({
            "id": f"t{i}",
            "x": round(x, 3),
            "y": round(y, 3),
            "source": src,
            "trunk_r": round(tr, 3),
            "trunk_h": round(th, 2),
            "crown_r": round(cr, 2),
            "crown_variant": variant,
        })

    # bounds + ground span over the rounded scene points
    pts = [p for b in buildings for p in b["ring"]]
    pts += [p for r in roads_out for p in r["polyline"]]
    pts += [p for s in strips_out for p in s["polyline"]]
    pts += [p for g in grass_out for p in g["ring"]]
    pts += [(t["x"], t["y"]) for t in trees_out]
    min_x = min(p[0] for p in pts)
    max_x = max(p[0] for p in pts)
    min_y = min(p[1] for p in pts)
    max_y = max(p[1] for p in pts)
    span = 2 * max(abs(min_x), abs(max_x), abs(min_y), abs(max_y)) + GROUND_PAD
    origin_inside = any(point_in_poly(0.0, 0.0, b["ring"]) for b in buildings)

    return {
        "schema": SCHEMA,
        "version": VERSION,
        "attribution": ATTRIBUTION,
        "frame": "metres, x=east y=north z=up, origin at (source.origin_lat, source.origin_lon)",
        "elevation": {"model": "flat", "z_m": 0.0},
        "seed": seed,
        "source": {
            "osm_json_sha256": src_sha,
            "origin_lat": round(lat, 7),
            "origin_lon": round(lon, 7),
            "fetched": fetched,
        },
        "bounds": {"min_x": round(min_x, 3), "max_x": round(max_x, 3),
                   "min_y": round(min_y, 3), "max_y": round(max_y, 3)},
        "ground": {"span_m": round(span, 3)},
        "origin_inside_building": origin_inside,
        "counts": {
            "buildings": len(buildings),
            "roads": len(roads_out),
            "strips": len(strips_out),
            "grass": len(grass_out),
            "trees": len(trees_out),
            "trees_tagged": n_tagged,
            "dashes": dashes,
        },
        "buildings": buildings,
        "roads": roads_out,
        "strips": strips_out,
        "grass": grass_out,
        "trees": trees_out,
    }


# ---- OBJ emission: a pure function of the pack dict --------------------------
class ObjWriter:
    """Accumulates per-group triangle/quad faces; metre units, no UVs."""

    def __init__(self):
        self.chunks = []      # (name, material, [face])
        self._cur = None

    def begin(self, name, material):
        self._cur = [name, material, []]

    def add(self, pts):
        self._cur[2].append([(float(x), float(y), float(z)) for x, y, z in pts])

    def quad(self, pts):
        self.add(pts[:3])
        self.add([pts[0], pts[2], pts[3]])

    def end(self):
        self.chunks.append(tuple(self._cur))
        self._cur = None

    def write(self, path):
        with open(path, "w") as f:
            f.write("# darter area pack meshes (metres, triangles; derived"
                    " from pack.json)\n")
            vi = 1
            for name, mat, faces in self.chunks:
                f.write(f"o {name}\nusemtl {mat}\n")
                for face in faces:
                    for x, y, z in face:
                        f.write(f"v {x:.3f} {y:.3f} {z:.3f}\n")
                for face in faces:
                    f.write("f " + " ".join(str(vi + k)
                                            for k in range(len(face))) + "\n")
                    vi += len(face)


def _z_at(height, p, off):
    """Surface z = offset above the terrain sampler (or the plain offset for
    flat packs: height None returns off so flat bytes are unchanged)."""
    if height is None:
        return off
    return height(p[0], p[1]) + off


def write_obj(pack, path, height=None):
    """Emit scene.obj — a pure function of the pack dict, plus the terrain
    sampler height(x, y) for glo30 packs (None for flat). Flat packs keep
    the pre-terrain literal Z values, so their bytes are unchanged."""
    w = ObjWriter()
    half = pack["ground"]["span_m"] / 2.0
    w.begin("ground", "grass")
    if height is None:
        w.quad([(-half, -half, 0.0), (half, -half, 0.0),
                (half, half, 0.0), (-half, half, 0.0)])
    else:
        # the drawn ground IS the terrain grid: one cell quad per node cell,
        # sharing the v00->v11 diagonal ObjWriter.quad always splits, so the
        # drawn surface is the physics contact surface
        e = pack["elevation"]
        for j in range(e["rows"] - 1):
            for i in range(e["cols"] - 1):
                x0 = e["origin_x"] + i * e["step_m"]
                y0 = e["origin_y"] + j * e["step_m"]
                x1 = x0 + e["step_m"]
                y1 = y0 + e["step_m"]
                w.quad([(x0, y0, height(x0, y0)),
                        (x1, y0, height(x1, y0)),
                        (x1, y1, height(x1, y1)),
                        (x0, y1, height(x0, y1))])
    w.end()
    for g in pack["grass"]:
        ring = g["ring"]
        w.begin("grass_" + g["id"], "grass")
        for i in range(1, len(ring) - 1):
            w.add([(ring[0][0], ring[0][1], _z_at(height, ring[0], 0.005)),
                   (ring[i][0], ring[i][1], _z_at(height, ring[i], 0.005)),
                   (ring[i + 1][0], ring[i + 1][1],
                    _z_at(height, ring[i + 1], 0.005))])
        w.end()
    for r in pack["roads"]:
        geom, width, mat = r["polyline"], r["width_m"], r["surface"]
        w.begin("road_" + r["id"], mat)
        half_w = width / 2
        for k in range(len(geom) - 1):
            (x0, y0), (x1, y1) = geom[k], geom[k + 1]
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy)
            if L < 0.5:
                continue
            nx, ny = -dy / L * half_w, dx / L * half_w
            # 50 mm over the ground quad with 5 mm steps between segments:
            # depth precision is ~d^2/(near*2^24) (near=1.0 in the smoke
            # camera), so the old 10 mm base fought the ground beyond ~100 m
            # and the 1 mm inter-road steps fought at junctions (flicker seen
            # on-device, 2026-09-28).
            z = 0.05 + 0.005 * ((k * 7) % 5)
            if height is None:
                # winding flipped vs the naive order: A,B,C,D (the +n side
                # first) winds the quad DOWNWARD for a segment along +y
                # (verified in the source against backface culling from the
                # air); D,C,B,A faces up.
                w.quad([(x0 - nx, y0 - ny, z), (x1 - nx, y1 - ny, z),
                        (x1 + nx, y1 + ny, z), (x0 + nx, y0 + ny, z)])
            else:
                # draped road: sub-slices of at most ROAD_SUBDIV_M (half the
                # node step) so no slice spans more than one cell's relief;
                # the ladder keeps the original segment index k so the stagger
                # stays continuous across sub-slices
                subs = max(1, int(math.ceil(L / ROAD_SUBDIV_M)))
                prev_z = height(x0, y0) + z
                px, py = x0, y0
                for m_i in range(1, subs + 1):
                    t = m_i / subs
                    qx, qy = x0 + dx * t, y0 + dy * t
                    qz = height(qx, qy) + z
                    w.quad([(px - nx, py - ny, prev_z), (qx - nx, qy - ny, qz),
                            (qx + nx, qy + ny, qz), (px + nx, py + ny, prev_z)])
                    px, py, prev_z = qx, qy, qz
        for k in range(1, len(geom) - 1):
            cx, cy = geom[k]
            z = 0.075  # above every road-segment level (max 0.07)
            r_c = half_w / math.cos(math.pi / 8.0)
            ring = [(cx + r_c * math.cos(math.pi / 4.0 * i + math.pi / 8.0),
                     cy + r_c * math.sin(math.pi / 4.0 * i + math.pi / 8.0))
                    for i in range(8)]
            for i in range(8):
                a = ring[i]
                b = ring[(i + 1) % 8]
                w.add([(cx, cy, _z_at(height, (cx, cy), z)),
                       (a[0], a[1], _z_at(height, a, z)),
                       (b[0], b[1], _z_at(height, b, z))])
        w.end()
        # S3b lane-centre dashes: 3.0 m dash / 6.0 m gap along the
        # centreline. z = own segment's road z + 5 mm (the same 5 mm stagger
        # the road surfaces were measured good with) capped at 0.070 so the
        # dash is always >=5 mm under the 0.075 junction pads and never
        # coplanar with any road level (the 5 mm grid keeps every pair in
        # the depth-precision-good regime the road emission relies on).
        # Placement is length-driven, no rng: byte-identical rebuilds.
        if width >= DASH_MIN_WIDTH:
            w.begin("dash_" + r["id"], "paint_white")
            for k in range(len(geom) - 1):
                (x0, y0), (x1, y1) = geom[k], geom[k + 1]
                dx, dy = x1 - x0, y1 - y0
                L = math.hypot(dx, dy)
                if L < 2.0 * DASH_INSET + DASH_LEN:
                    continue
                ux, uy = dx / L, dy / L
                ndx, ndy = -uy * DASH_HALF_W, ux * DASH_HALF_W
                z = min(0.05 + 0.005 * ((k * 7) % 5) + 0.005, DASH_Z_CAP)
                s = DASH_INSET
                while s + DASH_LEN <= L - DASH_INSET:
                    ax, ay = x0 + ux * s, y0 + uy * s
                    bx, by = x0 + ux * (s + DASH_LEN), y0 + uy * (s + DASH_LEN)
                    za = _z_at(height, (ax, ay), z)
                    zb = _z_at(height, (bx, by), z)
                    # same winding as the road quad above (the -n side first
                    # faces up; +n-first winds downward)
                    w.quad([(ax - ndx, ay - ndy, za),
                            (bx - ndx, by - ndy, zb),
                            (bx + ndx, by + ndy, zb),
                            (ax + ndx, ay + ndy, za)])
                    s += DASH_LEN + DASH_GAP
            w.end()
    for s in pack["strips"]:
        kind = "hedge" if s["kind"] == "hedge" else "fence"
        width, top_h = (HEDGE_W, HEDGE_H) if kind == "hedge" else (FENCE_W, FENCE_H)
        geom = s["polyline"]
        w.begin(kind + "_" + s["id"], kind)
        half_w = width / 2
        for (x0, y0), (x1, y1) in zip(geom, geom[1:]):
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy)
            if L < 0.3:
                continue
            nx, ny = -dy / L * half_w, dx / L * half_w
            # draped: each endpoint's base sits STRIP_SINK_M into the ground
            # so the strip never floats free of sloped terrain; the top edge
            # tracks the base pair per endpoint (the cap is allowed to slope)
            z0 = (_z_at(height, (x0, y0), 0.0) - STRIP_SINK_M
                  if height is not None else 0.0)
            z1 = (_z_at(height, (x1, y1), 0.0) - STRIP_SINK_M
                  if height is not None else 0.0)
            b0 = (x0 + nx, y0 + ny, z0)
            b1 = (x1 + nx, y1 + ny, z1)
            b2 = (x1 - nx, y1 - ny, z1)
            b3 = (x0 - nx, y0 - ny, z0)
            t0 = (b0[0], b0[1], z0 + top_h)
            t1 = (b1[0], b1[1], z1 + top_h)
            t2 = (b2[0], b2[1], z1 + top_h)
            t3 = (b3[0], b3[1], z0 + top_h)
            w.quad([t0, t1, b1, b0])
            w.quad([b2, b3, t3, t2])
            w.quad([t1, t2, b2, b1])
            w.quad([b3, b0, t0, t3])
            w.quad([t3, t2, t1, t0])
        w.end()
    for b in pack["buildings"]:
        ring = b["ring"]
        wall_h, roof_h = b["wall_h"], b["roof_h"]
        # the whole building translates to the terrain: base = ground under
        # the ring mean, sunk BUILD_SINK_M so walls never float on slopes
        # (the roof code takes absolute wall heights, so a flat pack's
        # base = 0.0 reproduces today's bytes exactly)
        base = 0.0
        if height is not None:
            mx = sum(p[0] for p in ring) / len(ring)
            my = sum(p[1] for p in ring) / len(ring)
            base = height(mx, my) - BUILD_SINK_M
        wall_z = base + wall_h
        w.begin("bld_" + b["id"], b["wall_mat"])
        # bottom cap (never seen, closes the mesh); each triangle's winding is
        # reversed so the cap faces down (the ring itself is CCW, i.e. up)
        for i0, i1, i2 in triangulate_ring(ring):
            w.add([(ring[i2][0], ring[i2][1], base),
                   (ring[i1][0], ring[i1][1], base),
                   (ring[i0][0], ring[i0][1], base)])
        for i in range(len(ring)):
            j = (i + 1) % len(ring)
            w.quad([(ring[i][0], ring[i][1], base),
                    (ring[j][0], ring[j][1], base),
                    (ring[j][0], ring[j][1], wall_z),
                    (ring[i][0], ring[i][1], wall_z)])
        w.end()
        w.begin("bldroof_" + b["id"], b["roof_mat"])
        if b["roof_shape"] in ("gable", "hip") and len(ring) == 4 and roof_h > 0:
            _add_pitched_roof(w, ring, wall_z, roof_h, b["roof_shape"])
        else:
            for i0, i1, i2 in triangulate_ring(ring):
                w.add([(ring[i0][0], ring[i0][1], wall_z),
                       (ring[i1][0], ring[i1][1], wall_z),
                       (ring[i2][0], ring[i2][1], wall_z)])
        w.end()
    for t in pack["trees"]:
        _add_tree(w, t, _z_at(height, (t["x"], t["y"]), 0.0))
    w.write(path)


def _add_pitched_roof(w, ring, wall_h, roof_h, shape):
    """Closed gable or hip roof over a near-parallelogram CCW ring (ported)."""
    edges = [(ring[k], ring[(k + 1) % 4]) for k in range(4)]
    lens = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in edges]
    ends = (1, 3) if lens[0] >= lens[1] else (0, 2)     # short pair
    ma = ((edges[ends[0]][0][0] + edges[ends[0]][1][0]) / 2,
          (edges[ends[0]][0][1] + edges[ends[0]][1][1]) / 2)
    mb = ((edges[ends[1]][0][0] + edges[ends[1]][1][0]) / 2,
          (edges[ends[1]][0][1] + edges[ends[1]][1][1]) / 2)
    if shape == "hip":
        d = (mb[0] - ma[0], mb[1] - ma[1])
        L = math.hypot(*d)
        inset = min(min(lens[ends[0]], lens[ends[1]]) / 2.0, 0.45 * L)
        ra = (ma[0] + d[0] / L * inset, ma[1] + d[1] / L * inset)
        rb = (mb[0] - d[0] / L * inset, mb[1] - d[1] / L * inset)
    else:
        ra, rb = ma, mb
    ridge_top = wall_h + roof_h
    r = {ends[0]: (ra[0], ra[1], ridge_top), ends[1]: (rb[0], rb[1], ridge_top)}
    for k in ends:                       # gable/hip end triangles
        a, b = edges[k]
        rp = r[k]
        w.add([(a[0], a[1], wall_h), (b[0], b[1], wall_h),
               (rp[0], rp[1], rp[2])])
    for k in range(4):                   # slope quads over the eave edges
        if k in ends:
            continue
        j = (k + 1) % 4
        a, b = edges[k]
        rj, ri = r[j], r[(k + 3) % 4]
        w.quad([(a[0], a[1], wall_h), (b[0], b[1], wall_h),
                (rj[0], rj[1], rj[2]), (ri[0], ri[1], ri[2])])


def _add_tree(w, t, base=0.0):
    """Trunk (6-sided cylinder) + ellipsoid crown (ported, no UVs).

    base = terrain height under the tree (0.0 keeps flat packs identical)."""
    x, y = t["x"], t["y"]
    r_t, h_t = t["trunk_r"], t["trunk_h"]
    r_c, variant = t["crown_r"], t["crown_variant"]
    w.begin("tree_" + t["id"], "bark")
    ring0, ring1 = [], []
    for s in range(6):
        a = 2 * math.pi * s / 6
        px, py = x + r_t * math.cos(a), y + r_t * math.sin(a)
        ring0.append((px, py, base))
        ring1.append((px, py, base + h_t))
    for s in range(6):
        j = (s + 1) % 6
        w.quad([ring0[s], ring0[j], ring1[j], ring1[s]])
    w.end()
    w.begin("treec_" + t["id"], variant)
    cz = base + h_t + r_c * 0.8
    n_seg, n_band = 8, 4
    rings = []
    for b in range(1, n_band):        # interior bands
        phi = math.pi * b / n_band
        zr = cz + r_c * 1.15 * math.cos(phi)
        rr = r_c * math.sin(phi)
        rings.append([(x + rr * math.cos(2 * math.pi * s / n_seg),
                       y + rr * math.sin(2 * math.pi * s / n_seg), zr)
                      for s in range(n_seg)])
    top = (x, y, cz + r_c * 1.15)
    bot = (x, y, cz - r_c * 1.15)
    for s in range(n_seg):
        j = (s + 1) % n_seg
        w.add([top, rings[0][j], rings[0][s]])
        for b in range(len(rings) - 1):
            r_lo, r_hi = rings[b], rings[b + 1]
            w.quad([r_lo[s], r_lo[j], r_hi[j], r_hi[s]])
        w.add([bot, rings[-1][s], rings[-1][j]])
    w.end()


# ---- validator ----------------------------------------------------------------
def validate_pack(pack_dir):
    """Re-assert the schema on a written pack. Returns a list of errors."""
    errors = []

    def err(msg):
        errors.append(msg)

    path = os.path.join(pack_dir, "pack.json")
    if not os.path.exists(path):
        return [f"missing {path}"]
    with open(path) as f:
        pack = json.load(f)
    if pack.get("schema") != SCHEMA:
        err(f"schema {pack.get('schema')!r} != {SCHEMA!r}")
    if pack.get("version") != VERSION:
        err(f"version {pack.get('version')!r} != {VERSION}")
    attr = pack.get("attribution", "")
    if "OpenStreetMap" not in attr:
        err(f"attribution missing OpenStreetMap credit: {attr!r}")
    if not pack.get("frame"):
        err("frame missing")
    elev = pack.get("elevation", {})
    if elev.get("model") == "flat":
        if elev.get("z_m") != 0.0:
            err(f"elevation not a flat z=0 model: {elev!r}")
    elif elev.get("model") == "glo30":
        _validate_glo30_section(pack_dir, pack, elev, err)
    else:
        err(f"elevation model {elev.get('model')!r} unknown (flat | glo30)")
    src = pack.get("source", {})
    sha = src.get("osm_json_sha256", "")
    if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
        err(f"osm_json_sha256 not sha256 hex: {sha!r}")
    if not isinstance(src.get("origin_lat"), (int, float)):
        err("origin_lat missing")
    if not isinstance(src.get("origin_lon"), (int, float)):
        err("origin_lon missing")
    if not isinstance(pack.get("seed"), int):
        err("seed missing/not int")

    buildings = pack.get("buildings", [])
    roads = pack.get("roads", [])
    strips = pack.get("strips", [])
    grass = pack.get("grass", [])
    trees = pack.get("trees", [])
    counts = pack.get("counts", {})
    for key, arr in (("buildings", buildings), ("roads", roads),
                     ("strips", strips), ("grass", grass), ("trees", trees)):
        if counts.get(key) != len(arr):
            err(f"counts.{key} {counts.get(key)} != {len(arr)}")
    if counts.get("trees_tagged") != sum(
            1 for t in trees if t.get("source") == "tagged"):
        err("counts.trees_tagged != tagged tree count")
    # S3b: the dash count is derived from the same rounded widths the OBJ
    # emission gates on, so JSON and geometry are re-asserted independently.
    if counts.get("dashes") != sum(
            1 for r in roads if r.get("width_m") >= DASH_MIN_WIDTH):
        err("counts.dashes != width>=5.0 road count")

    # bounds and span, recomputed from the rounded coordinates
    pts = [p for b in buildings for p in b["ring"]]
    pts += [p for r in roads for p in r["polyline"]]
    pts += [p for s in strips for p in s["polyline"]]
    pts += [p for g in grass for p in g["ring"]]
    pts += [(t["x"], t["y"]) for t in trees]
    if not pts:
        err("scene has no points")
    else:
        bnd = pack.get("bounds", {})
        want = {"min_x": min(p[0] for p in pts), "max_x": max(p[0] for p in pts),
                "min_y": min(p[1] for p in pts), "max_y": max(p[1] for p in pts)}
        for k, v in want.items():
            if round(bnd.get(k, 0.0), 3) != round(v, 3):
                err(f"bounds.{k} {bnd.get(k)} != recomputed {round(v, 3)}")
        span = 2 * max(abs(want["min_x"]), abs(want["max_x"]),
                       abs(want["min_y"]), abs(want["max_y"])) + GROUND_PAD
        if elev.get("model") == "glo30":
            # the glo30 ground span snaps UP to whole node steps so the node
            # spacing stays exactly --step and the grid covers the scene
            span = math.ceil(span / TERRAIN_STEP_M) * TERRAIN_STEP_M
        if round(pack.get("ground", {}).get("span_m", 0.0), 3) != round(span, 3):
            err(f"ground.span_m {pack.get('ground', {}).get('span_m')} != {round(span, 3)}")

    ids = set()
    for b in buildings:
        ring = b.get("ring", [])
        if len(ring) < 3:
            err(f"{b.get('id')}: ring < 3 pts")
            continue
        if signed_area(ring) <= 0:
            err(f"{b.get('id')}: ring not CCW")
        if b.get("wall_h", 0) <= 0:
            err(f"{b.get('id')}: wall_h <= 0")
        if b.get("roof_h", -1) < 0:
            err(f"{b.get('id')}: roof_h < 0")
        if b.get("roof_shape") not in ("gable", "hip", "flat"):
            err(f"{b.get('id')}: roof_shape {b.get('roof_shape')!r}")
        if b.get("bld_class") not in ("garage", "residential", "flat_block"):
            err(f"{b.get('id')}: bld_class {b.get('bld_class')!r}")
        if b.get("height_source") not in ("tag_height", "levels", "default"):
            err(f"{b.get('id')}: height_source {b.get('height_source')!r}")
        if b.get("wall_mat") not in ALL_WALL_MATS:
            err(f"{b.get('id')}: wall_mat {b.get('wall_mat')!r}")
        if b.get("roof_mat") not in ROOF_MATS:
            err(f"{b.get('id')}: roof_mat {b.get('roof_mat')!r}")
        ids.add(b.get("id"))
    for r in roads:
        if len(r.get("polyline", [])) < 2:
            err(f"{r.get('id')}: polyline < 2 pts")
        if r.get("width_m") not in ROAD_WIDTHS.values():
            err(f"{r.get('id')}: width {r.get('width_m')} not a ROAD_WIDTHS value")
        if r.get("surface") not in SURFACES:
            err(f"{r.get('id')}: surface {r.get('surface')!r}")
        ids.add(r.get("id"))
    for s in strips:
        if len(s.get("polyline", [])) < 2:
            err(f"{s.get('id')}: polyline < 2 pts")
        if s.get("kind") not in ("hedge", "fence", "wall"):
            err(f"{s.get('id')}: kind {s.get('kind')!r}")
        ids.add(s.get("id"))
    for g in grass:
        if len(g.get("ring", [])) < 3:
            err(f"{g.get('id')}: ring < 3 pts")
        ids.add(g.get("id"))
    for t in trees:
        for k in ("x", "y", "trunk_r", "trunk_h", "crown_r"):
            if not isinstance(t.get(k), (int, float)) or not math.isfinite(t[k]):
                err(f"{t.get('id')}: {k} not finite")
        if not (0.05 <= t.get("trunk_r", 0) <= 0.25):
            err(f"{t.get('id')}: trunk_r out of range")
        if not (1.0 <= t.get("crown_r", 0) <= 3.5):
            err(f"{t.get('id')}: crown_r out of range")
        if t.get("crown_variant") not in FOLIAGE_VARIANTS:
            err(f"{t.get('id')}: crown_variant {t.get('crown_variant')!r}")
        if t.get("source") not in ("tagged", "scattered"):
            err(f"{t.get('id')}: source {t.get('source')!r}")
        ids.add(t.get("id"))
    if len(ids) != len(buildings) + len(roads) + len(strips) + len(grass) + len(trees):
        err("duplicate ids")

    # cross-check the OBJ group counts against the JSON
    obj_path = os.path.join(pack_dir, "scene.obj")
    if not os.path.exists(obj_path):
        err(f"missing {obj_path}")
    else:
        groups = {}
        vcount = {}
        cur = None
        bad_float = False
        with open(obj_path) as f:
            for line in f:
                p = line.split()
                if not p:
                    continue
                if p[0] == "o":
                    name = p[1].split("_")[0]
                    groups[name] = groups.get(name, 0) + 1
                    cur = p[1]
                    vcount[cur] = 0
                elif p[0] == "v":
                    for tok in p[1:4]:
                        if not _is_3dec(tok):
                            bad_float = True
                    if cur is not None:
                        vcount[cur] += 1
        if bad_float:
            err("scene.obj has vertices not formatted %.3f")
        want_groups = {
            "ground": 1,
            "grass": len(grass),
            "road": len(roads),
            "hedge": sum(1 for s in strips if s["kind"] == "hedge"),
            "fence": sum(1 for s in strips if s["kind"] != "hedge"),
            "bld": len(buildings),
            "bldroof": len(buildings),
            "tree": len(trees),
            "treec": len(trees),
            "dash": counts.get("dashes", 0),
        }
        for name, want in want_groups.items():
            if groups.get(name, 0) != want:
                err(f"scene.obj {name} groups {groups.get(name, 0)} != {want}")
        if elev.get("model") == "glo30":
            # the grid ground: two triangles (6 verts) per node cell
            want_v = 6 * (elev["cols"] - 1) * (elev["rows"] - 1)
            got_v = vcount.get("ground", 0)
            if got_v != want_v:
                err(f"scene.obj ground vertices {got_v} != {want_v} "
                    "(6 per grid cell, 2 tris)")
    return errors


def _validate_terrain_bin(path, elev, err):
    """Structural + self-consistency check of terrain.bin. Returns the
    parsed grid as h_at-able dict rows, or None (with errors added)."""
    cols, rows = elev.get("cols"), elev.get("rows")
    if not (isinstance(cols, int) and isinstance(rows, int)
            and cols >= 2 and rows >= 2):
        return None
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        err(f"missing {path}")
        return None
    want_size = TERRAIN_HEADER_LEN + 8 * rows * cols
    if len(data) != want_size:
        err(f"terrain.bin {len(data)} bytes != {want_size} (header "
            f"{TERRAIN_HEADER_LEN} + 8 * {rows} * {cols})")
        return None
    magic, fmt, hcols, hrows, step, ox, oy, hmin, hmax = struct.unpack(
        "<IIII5d", data[:TERRAIN_HEADER_LEN])
    if magic != TERRAIN_MAGIC:
        err(f"terrain.bin magic 0x{magic:08X} != 0x{TERRAIN_MAGIC:08X}")
    if fmt != TERRAIN_FMT_VERSION:
        err(f"terrain.bin format version {fmt} != {TERRAIN_FMT_VERSION}")
    if (hcols, hrows) != (cols, rows):
        err(f"terrain.bin dims ({hcols}, {hrows}) != pack.json {cols}x{rows}")
    for h_name, h_val, j_name, j_val in (("step", step, "step_m", elev.get("step_m")),
                                         ("origin_x", ox, "origin_x", elev.get("origin_x")),
                                         ("origin_y", oy, "origin_y", elev.get("origin_y")),
                                         ("z_min", hmin, "z_min", elev.get("z_min")),
                                         ("z_max", hmax, "z_max", elev.get("z_max"))):
        if h_val != j_val:
            err(f"terrain.bin {h_name} {h_val!r} != pack.json {j_name} {j_val!r}")
    vals = struct.unpack(f"<{rows * cols}d", data[TERRAIN_HEADER_LEN:])
    pmin, pmax = min(vals), max(vals)
    if pmin != hmin or pmax != hmax:
        err(f"terrain.bin payload range [{pmin!r}, {pmax!r}] != header "
            f"[{hmin!r}, {hmax!r}]")
    if vals[0] != 0.0:
        err(f"terrain.bin node (0,0) is {vals[0]!r}, not 0.0 (z datum)")
    # every node is round(raw - raw0, 1), so all values sit on the 0.1 m
    # lattice; an arbitrary corrupt byte lands off it with practical
    # certainty, which closes most of what the min/max + spot checks miss
    off_lattice = [v for v in vals if _round1(v) != v]
    if off_lattice:
        err(f"terrain.bin has {len(off_lattice)} node(s) off the 0.1 m "
            f"lattice (first {off_lattice[0]!r})")
    try:
        z = [list(vals[j * cols:(j + 1) * cols]) for j in range(rows)]
    except MemoryError:
        err("terrain.bin too large to parse")
        return None
    return {"cols": cols, "rows": rows, "step": step,
            "origin_x": ox, "origin_y": oy, "z": z}


def _validate_glo30_section(pack_dir, pack, elev, err):
    """glo30 elevation: block shape, provenance keys, terrain.bin consistency
    against pack.json, and grid h_at spot checks. DEM truth (heights vs the
    source geo) lives in the tests against the committed fixture."""
    if elev.get("datum") != "origin_ground":
        err(f"elevation datum {elev.get('datum')!r} != 'origin_ground'")
    if elev.get("step_m") != TERRAIN_STEP_M:
        err(f"elevation step_m {elev.get('step_m')!r} != {TERRAIN_STEP_M}")
    if elev.get("licence") != GLO30_LICENCE:
        err(f"elevation licence {elev.get('licence')!r} != {GLO30_LICENCE!r}")
    if "Copernicus" not in pack.get("attribution", ""):
        err("attribution missing the Copernicus DEM credit")
    for k in ("cols", "rows"):
        if not isinstance(elev.get(k), int) or elev[k] < 2:
            err(f"elevation {k} not an int >= 2")
    span = pack.get("ground", {}).get("span_m", 0.0)
    for k in ("origin_x", "origin_y"):
        want = -span / 2.0
        if elev.get(k) != want:
            err(f"elevation {k} {elev.get(k)!r} != {want!r}")
    for k in ("z_min", "z_max"):
        v = elev.get(k)
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            err(f"elevation {k} not finite: {v!r}")
    if isinstance(elev.get("z_min"), float) and isinstance(elev.get("z_max"), float) \
            and elev["z_min"] > elev["z_max"]:
        err("elevation z_min > z_max")
    sha = elev.get("sha256", "")
    if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
        err(f"elevation.sha256 not sha256 hex: {sha!r}")
    tile_list = elev.get("tiles")
    if not isinstance(tile_list, list) or not tile_list:
        err("elevation.tiles missing/empty")
    else:
        for t in tile_list:
            if not isinstance(t, dict) or not t.get("name") or not t.get("sha256"):
                err(f"elevation tile malformed: {t!r}")
                continue
            tsha = t["sha256"]
            if len(tsha) != 64 or any(c not in "0123456789abcdef" for c in tsha):
                err(f"elevation tile {t.get('name')!r} sha not sha256 hex")
            if not isinstance(t.get("name"), str) or not t["name"].endswith(".tif"):
                err(f"elevation tile name not a .tif: {t.get('name')!r}")
        if len(set(t.get("name") for t in tile_list)) != len(tile_list):
            err("elevation.tiles has duplicate names")
    if isinstance(elev.get("cols"), int) and isinstance(elev.get("step_m"), float) \
            and (elev["cols"] - 1) * elev["step_m"] != span:
        err(f"grid footprint (cols-1)*step != span_m {span!r}")
    grid = _validate_terrain_bin(os.path.join(pack_dir, "terrain.bin"), elev, err)
    if grid is None:
        return
    with open(os.path.join(pack_dir, "terrain.bin"), "rb") as f:
        terrain_sha = hashlib.sha256(f.read()).hexdigest()
    if terrain_sha != elev.get("sha256"):
        err(f"elevation.sha256 {elev.get('sha256')!r} != terrain.bin sha256")
    nodes = [(0, 0), (0, grid["rows"] - 1), (grid["cols"] - 1, 0),
             (grid["cols"] - 1, grid["rows"] - 1),
             (grid["cols"] // 2, grid["rows"] // 2),
             (0, grid["rows"] // 2), (grid["cols"] // 2, 0),
             (grid["cols"] - 1, grid["rows"] // 2),
             (grid["cols"] // 2, grid["rows"] - 1)]
    for i, j in nodes:
        x = grid["origin_x"] + i * grid["step"]
        y = grid["origin_y"] + j * grid["step"]
        if terrain_h_at(grid, x, y) != grid["z"][j][i]:
            err(f"h_at node ({i}, {j}) != grid z")
    ci, cj = grid["cols"] // 2, grid["rows"] // 2
    x = grid["origin_x"] + (ci + 0.5) * grid["step"]
    y = grid["origin_y"] + (cj + 0.5) * grid["step"]
    if not math.isfinite(terrain_h_at(grid, x, y)):
        err("h_at interior probe not finite")


def _round1(v):
    return round(v, 1)


def _is_3dec(tok):
    """Match the fixed %.3f vertex format (e.g. '-12.345', '0.000')."""
    body = tok[1:] if tok.startswith("-") else tok
    parts = body.split(".")
    return (len(parts) == 2 and parts[0].isdigit() and len(parts[1]) == 3
            and parts[1].isdigit())


# ---- live fetch (opt-in, network) ---------------------------------------------
def fetch_osm(lat, lon, radius_m, cache_path):
    """Fetch the broad element set around (lat, lon) from Overpass.

    Buildings + highways + barriers + grass/garden + tree nodes — the same
    breadth the cached fixture was probed with (the source's building-only
    query cannot build a scene). Returns (data, endpoint_used, date_iso).
    """
    r_deg = radius_m / M_PER_DEG_LAT
    bbox = (lat - r_deg, lon - r_deg * 1.6, lat + r_deg, lon + r_deg * 1.6)  # S,W,N,E
    query = (f"[out:json][timeout:120];"
             f'(way["building"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f'way["highway"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f'way["barrier"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f'way["landuse"~"^(grass|meadow)$"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f'way["leisure"="garden"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f'node["natural"="tree"]({bbox[0]:.7f},{bbox[1]:.7f},{bbox[2]:.7f},{bbox[3]:.7f});'
             f");out geom;")
    url = OVERPASS_URL + "?" + urllib.parse.urlencode({"data": query})
    print(f"osm: fetching {radius_m:.0f} m radius from overpass-api.de ...")
    try:
        with urllib.request.urlopen(url, timeout=120) as resp:
            raw = resp.read()
        endpoint = OVERPASS_URL
    except urllib.error.HTTPError as e:
        mirror = MIRROR_URL + "?" + urllib.parse.urlencode({"data": query})
        print(f"osm: primary endpoint failed ({e}), trying kumi mirror")
        with urllib.request.urlopen(mirror, timeout=120) as resp:
            raw = resp.read()
        endpoint = MIRROR_URL
    data = json.loads(raw)
    if cache_path:
        with open(cache_path, "w") as f:
            json.dump(data, f)
        print(f"osm: cache written to {cache_path}")
    sha = hashlib.sha256(raw).hexdigest()
    return data, endpoint, datetime.date.today().isoformat(), sha


# ---- main -----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--osm", help="raw Overpass cache (ways+nodes JSON)")
    ap.add_argument("--lat", type=float, default=50.8989)
    ap.add_argument("--lon", type=float, default=-1.0586)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--out", help="output pack directory")
    ap.add_argument("--fetch", type=float, metavar="RADIUS_M",
                    help="live Overpass fetch (network) instead of --osm")
    ap.add_argument("--elevation", default="flat", choices=("flat", "glo30"),
                    help="elevation model: flat (z=0) or glo30 (Copernicus DEM "
                         "30 m sampled onto a node grid + terrain.bin sidecar)")
    ap.add_argument("--dem-file", help="offline DEM GeoTIFF path (glo30): "
                    "build from this file instead of the tile mirror")
    ap.add_argument("--dem-cache", help="directory for GLO-30 tile cache "
                    "(default: tools/elevation_cache next to this script)")
    ap.add_argument("validate", nargs="?", help="validate a written pack dir")
    args = ap.parse_args(argv)

    if args.validate:
        errors = validate_pack(args.validate)
        if errors:
            for e in errors:
                print(f"INVALID: {e}", file=sys.stderr)
            return 1
        print(f"pack OK: {args.validate}")
        return 0

    if args.elevation != "glo30" and (args.dem_file or args.dem_cache):
        ap.error("--dem-file/--dem-cache only apply with --elevation glo30")

    fetched = None
    fetch_sha = ""
    if args.fetch:
        data, endpoint, date_iso, fetch_sha = fetch_osm(
            args.lat, args.lon, args.fetch, args.osm)
        fetched = {"date": date_iso, "endpoint": endpoint}
    elif args.osm:
        with open(args.osm) as f:
            data = json.load(f)
    else:
        ap.error("need --osm CACHE.json or --fetch RADIUS_M")
    if not args.out:
        ap.error("need --out DIR")

    if args.fetch:
        src_sha = fetch_sha
    elif args.osm and os.path.exists(args.osm):
        with open(args.osm, "rb") as f:
            src_sha = hashlib.sha256(f.read()).hexdigest()
    else:
        src_sha = ""

    scene = parse_scene(data, args.lat, args.lon, args.seed)
    classified, roads, strips, grass, trees, tree_params, n_tagged = scene
    pack = build_pack(classified, roads, strips, grass, trees, tree_params,
                      n_tagged, args.lat, args.lon, args.seed, src_sha, fetched)

    grid = None
    if args.elevation == "glo30":
        cache_dir = args.dem_cache or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "elevation_cache")
        grid = build_terrain_grid(args.lat, args.lon, pack["ground"]["span_m"],
                                  dem_file=args.dem_file, cache_dir=cache_dir)
        pack["ground"]["span_m"] = round((grid["cols"] - 1) * TERRAIN_STEP_M, 3)
        os.makedirs(args.out, exist_ok=True)
        terrain_path = os.path.join(args.out, "terrain.bin")
        write_terrain_bin(terrain_path, grid)
        with open(terrain_path, "rb") as f:
            terrain_sha = hashlib.sha256(f.read()).hexdigest()
        pack["elevation"] = {
            "model": "glo30",
            "datum": "origin_ground",
            "step_m": grid["step"],
            "cols": grid["cols"],
            "rows": grid["rows"],
            "origin_x": grid["origin_x"],
            "origin_y": grid["origin_y"],
            "z_min": grid["z_min"],
            "z_max": grid["z_max"],
            "sha256": terrain_sha,
            "tiles": grid["tiles"],
            "licence": GLO30_LICENCE,
        }
        pack["attribution"] = ATTRIBUTION + " | " + GLO30_LICENCE
        print(f"terrain: {grid['cols']}x{grid['rows']} nodes @ "
              f"{grid['step']:.0f} m, relief z {grid['z_min']:+.1f}.."
              f"{grid['z_max']:+.1f} m (datum = DEM height at the grid origin)")
        for t in grid["tiles"]:
            print(f"terrain tile: {t['name']} sha256 {t['sha256'][:12]}...")

    os.makedirs(args.out, exist_ok=True)
    pack_path = os.path.join(args.out, "pack.json")
    with open(pack_path, "w") as f:
        json.dump(pack, f, sort_keys=True, indent=1)
        f.write("\n")
    obj_path = os.path.join(args.out, "scene.obj")
    write_obj(pack, obj_path, height=(grid["h"] if grid is not None else None))

    c = pack["counts"]
    print(f"pack: {args.out} ({os.path.getsize(pack_path) / 1e6:.1f} MB json, "
          f"{os.path.getsize(obj_path) / 1e6:.1f} MB obj): "
          f"{c['buildings']} buildings, {c['trees']} trees "
          f"({n_tagged} tagged + {c['trees'] - n_tagged} scattered), "
          f"{c['roads']} roads, {c['strips']} hedges/fences, {c['grass']} grass, "
          f"span {pack['ground']['span_m']:.0f} m")
    if pack["origin_inside_building"]:
        print("WARNING: origin (0,0) inside a building footprint — pick "
              "home lat/lon on open ground", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())