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
identical pack.json and scene.obj. Mechanism: sorted JSON keys, floats
rounded to fixed precision before writing (3 decimals for coordinates, 2-3
for heights and radii), OBJ verts formatted %.3f, one seeded Mersenne Twister
consumed in one fixed order, one fixed iteration order over the input
elements. pack.json records the input file's sha256 and the seed, so a pack
is reproducible from its own provenance.

Elevation: the schema carries an elevation object; only the flat model is
implemented ("flat", z_m). The rasterio + Copernicus GLO-30 DEM path is a
stated gap — no rasterio in the test environment and the suite must build
offline. --elevation glo30 raises NotImplementedError rather than
pretending.

Frame: metres, x = east, y = north, z = up, origin at (--lat, --lon) — the
same convention as the anisoptera sources.

Usage:
  area_pack.py --osm CACHE.json --lat 50.8989 --lon -1.0586 [--seed 5] --out DIR
  area_pack.py --fetch RADIUS_M --lat L --lon L [--osm CACHE.json] --out DIR
  area_pack.py DIR                      (validate a written pack dir)
"""
import argparse
import hashlib
import json
import math
import os
import random
import sys
import datetime
import urllib.error
import urllib.parse
import urllib.request

SCHEMA = "darter_area_pack"
# v2: counts.dashes + `o dash_` OBJ groups with material paint_white (v1 packs
# are rejected by the version check below — rebuild them).
VERSION = 2
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


def write_obj(pack, path):
    w = ObjWriter()
    half = pack["ground"]["span_m"] / 2.0
    w.begin("ground", "grass")
    w.quad([(-half, -half, 0.0), (half, -half, 0.0),
            (half, half, 0.0), (-half, half, 0.0)])
    w.end()
    for g in pack["grass"]:
        ring = g["ring"]
        w.begin("grass_" + g["id"], "grass")
        for i in range(1, len(ring) - 1):
            w.add([(ring[0][0], ring[0][1], 0.005),
                   (ring[i][0], ring[i][1], 0.005),
                   (ring[i + 1][0], ring[i + 1][1], 0.005)])
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
            # winding flipped vs the naive order: A,B,C,D (the +n side first)
            # winds the quad DOWNWARD for a segment along +y (verified in the
            # source against backface culling from the air); D,C,B,A faces up.
            w.quad([(x0 - nx, y0 - ny, z), (x1 - nx, y1 - ny, z),
                    (x1 + nx, y1 + ny, z), (x0 + nx, y0 + ny, z)])
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
                w.add([(cx, cy, z), (a[0], a[1], z), (b[0], b[1], z)])
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
                    # same winding as the road quad above (the -n side first
                    # faces up; +n-first winds downward)
                    w.quad([(ax - ndx, ay - ndy, z),
                            (bx - ndx, by - ndy, z),
                            (bx + ndx, by + ndy, z),
                            (ax + ndx, ay + ndy, z)])
                    s += DASH_LEN + DASH_GAP
            w.end()
    for s in pack["strips"]:
        kind = "hedge" if s["kind"] == "hedge" else "fence"
        width, height = (HEDGE_W, HEDGE_H) if kind == "hedge" else (FENCE_W, FENCE_H)
        geom = s["polyline"]
        w.begin(kind + "_" + s["id"], kind)
        half_w = width / 2
        for (x0, y0), (x1, y1) in zip(geom, geom[1:]):
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy)
            if L < 0.3:
                continue
            nx, ny = -dy / L * half_w, dx / L * half_w
            b0 = (x0 + nx, y0 + ny, 0.0)
            b1 = (x1 + nx, y1 + ny, 0.0)
            b2 = (x1 - nx, y1 - ny, 0.0)
            b3 = (x0 - nx, y0 - ny, 0.0)
            t0 = (b0[0], b0[1], height)
            t1 = (b1[0], b1[1], height)
            t2 = (b2[0], b2[1], height)
            t3 = (b3[0], b3[1], height)
            w.quad([t0, t1, b1, b0])
            w.quad([b2, b3, t3, t2])
            w.quad([t1, t2, b2, b1])
            w.quad([b3, b0, t0, t3])
            w.quad([t3, t2, t1, t0])
        w.end()
    for b in pack["buildings"]:
        ring = b["ring"]
        wall_h, roof_h = b["wall_h"], b["roof_h"]
        w.begin("bld_" + b["id"], b["wall_mat"])
        # bottom cap (never seen, closes the mesh); each triangle's winding is
        # reversed so the cap faces down (the ring itself is CCW, i.e. up)
        for i0, i1, i2 in triangulate_ring(ring):
            w.add([(ring[i2][0], ring[i2][1], 0.0),
                   (ring[i1][0], ring[i1][1], 0.0),
                   (ring[i0][0], ring[i0][1], 0.0)])
        for i in range(len(ring)):
            j = (i + 1) % len(ring)
            w.quad([(ring[i][0], ring[i][1], 0.0),
                    (ring[j][0], ring[j][1], 0.0),
                    (ring[j][0], ring[j][1], wall_h),
                    (ring[i][0], ring[i][1], wall_h)])
        w.end()
        w.begin("bldroof_" + b["id"], b["roof_mat"])
        if b["roof_shape"] in ("gable", "hip") and len(ring) == 4 and roof_h > 0:
            _add_pitched_roof(w, ring, wall_h, roof_h, b["roof_shape"])
        else:
            for i0, i1, i2 in triangulate_ring(ring):
                w.add([(ring[i0][0], ring[i0][1], wall_h),
                       (ring[i1][0], ring[i1][1], wall_h),
                       (ring[i2][0], ring[i2][1], wall_h)])
        w.end()
    for t in pack["trees"]:
        _add_tree(w, t)
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


def _add_tree(w, t):
    """Trunk (6-sided cylinder) + ellipsoid crown (ported, no UVs)."""
    x, y = t["x"], t["y"]
    r_t, h_t = t["trunk_r"], t["trunk_h"]
    r_c, variant = t["crown_r"], t["crown_variant"]
    w.begin("tree_" + t["id"], "bark")
    ring0, ring1 = [], []
    for s in range(6):
        a = 2 * math.pi * s / 6
        px, py = x + r_t * math.cos(a), y + r_t * math.sin(a)
        ring0.append((px, py, 0.0))
        ring1.append((px, py, h_t))
    for s in range(6):
        j = (s + 1) % 6
        w.quad([ring0[s], ring0[j], ring1[j], ring1[s]])
    w.end()
    w.begin("treec_" + t["id"], variant)
    cz = h_t + r_c * 0.8
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
    if elev.get("model") != "flat" or not isinstance(elev.get("z_m"), (int, float)):
        err(f"elevation not a flat model: {elev!r}")
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
        bad_float = False
        with open(obj_path) as f:
            for line in f:
                p = line.split()
                if not p:
                    continue
                if p[0] == "o":
                    name = p[1].split("_")[0]
                    groups[name] = groups.get(name, 0) + 1
                elif p[0] == "v":
                    for tok in p[1:4]:
                        if not _is_3dec(tok):
                            bad_float = True
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
    return errors


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
    ap.add_argument("--elevation", default="flat",
                    help="elevation model: 'flat' only (glo30 is a stated gap)")
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

    if args.elevation != "flat":
        # not a silent fallback: the DEM path does not exist yet
        raise NotImplementedError(
            "elevation model %r is a stated gap (rasterio + GLO-30 not "
            "implemented); only 'flat' is available" % args.elevation)

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

    os.makedirs(args.out, exist_ok=True)
    pack_path = os.path.join(args.out, "pack.json")
    with open(pack_path, "w") as f:
        json.dump(pack, f, sort_keys=True, indent=1)
        f.write("\n")
    obj_path = os.path.join(args.out, "scene.obj")
    write_obj(pack, obj_path)

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