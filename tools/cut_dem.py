#! /usr/bin/env python3
"""Cut a committed GLO-30 fixture DEM (offline clip) out of a cached full tile.

The demo-area packs build from committed fixture DEMs (tests/fixtures/
dem_<area>.tif) so the bundle step needs no network and no full-tile mirror
access. This tool cuts that clip from a full tile already in the elevation
cache (put there by one prior `area_pack.py --fetch ... --dem-cache DIR`
run); it never uses the network itself.

The window is half-extent-m metres past the origin on both axes (+ a pixel
to round outward), snapped to whole 30 m pixels, and every pack node's
nearest-pixel DEM sample must land inside it — the byte-equality check
between the full-tile build and the clip build (S1 evidence, recorded in
the commit message) is what proves the coverage. The pack span is
data-shaped (2*max abs scene point + GROUND_PAD, area_pack.py:743), so pass
a half-extent that covers the worst case of the fetch bbox (1.6*radius east/
west) plus the ground pad, not a guessed span.

Provenance goes into the TIFF ImageDescription: source tile name + full-tile
sha256 + exact clip window + measured half-extents, then the tool that cut it.

  tools/cut_dem.py --lat 50.9060 --lon -1.4012 --half-extent-m 1500 \
      --cache tools/elevation_cache --out tests/fixtures/dem_city.tif
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from area_pack import glo30_tile_name, read_dem, _dem_import  # noqa: E402

TIFF_TAGS = {
    "ModelPixelScale": 33550,
    "ModelTiepoint": 33922,
    "GeoKeyDirectory": 34735,
}


def window_bounds(dem, lat, lon, half_lat, half_lon):
    """Pixel bounds (row0, row1, col0, col1, tl_x, tl_y) covering the box.

    row1/col1 are exclusive numpy-slice ends (the printed clip rows/cols are
    row0..row1-1 / col0..col1-1, matching the home fixture's inclusive style).
    """
    row0 = int((dem["tly"] - (lat + half_lat)) / sy_of(dem))
    row1 = int((dem["tly"] - (lat - half_lat)) / sy_of(dem)) + 2
    col0 = int((lon - half_lon - dem["tlx"]) / sx_of(dem))
    col1 = int((lon + half_lon - dem["tlx"]) / sx_of(dem)) + 2
    # clamp inside the tile
    row0, col0 = max(0, row0), max(0, col0)
    row1 = min(dem["h"], row1)
    col1 = min(dem["w"], col1)
    # tl_y goes DOWN as rows increase (north-up TIFF: lat decreases southward)
    return row0, row1, col0, col1, dem["tlx"] + col0 * sx_of(dem), \
        dem["tly"] - row0 * sy_of(dem)


def sx_of(dem):
    return dem["sx"]


def sy_of(dem):
    return dem["sy"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--half-extent-m", type=float, required=True,
                    help="metres past the origin the clip must cover, all axes")
    ap.add_argument("--cache", default="tools/elevation_cache")
    ap.add_argument("--out", required=True, help="output clip path")
    args = ap.parse_args(argv)

    tile = glo30_tile_name(args.lat, args.lon)
    tpath = os.path.join(args.cache, tile + ".tif")
    if not os.path.exists(tpath):
        sys.exit("cached tile %s missing in %s: run one area_pack --fetch "
                 "for this area first (network tool, one-time)" % (tile, tpath))

    dem = read_dem(tpath)
    tifffile, _np = _dem_import()
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(args.lat))
    row0, row1, col0, col1, tl_x, tl_y = window_bounds(
        dem, args.lat, args.lon,
        args.half_extent_m / 111_320.0,
        args.half_extent_m / m_per_deg_lon)

    heights = dem["heights"][row0:row1, col0:col1]
    if heights.size == 0 or heights.shape[0] < 2 or heights.shape[1] < 2:
        sys.exit("degenerate clip window at %s for %s"
                 % (tpath, args.out))
    row_c = (dem["tly"] - args.lat) / dem["sy"]
    col_c = (args.lon - dem["tlx"]) / dem["sx"]
    half_ns = max(row_c - row0, row1 - 1 - row_c) * dem["sy"] * 111_320.0
    half_ew = max(col_c - col0, col1 - 1 - col_c) * dem["sx"] * m_per_deg_lon

    desc = (
        "darter DEM clip (Copernicus GLO-30) for tests/fixtures. Source tile "
        "Copernicus_DSM_COG_10_%s_00_DEM.tif (full-tile sha256 %s, cached from "
        "the mirror). Clip = rows %d..%d cols %d..%d of the %dx%d tile; every "
        "pack node of a 30 m node grid centred on %s%.4f,%s%.4f "
        "nearest-samples inside it (half-extent %.0f m N-S / %.0f m E-W, "
        "%d m margin over the grid). GeoKeys copied unchanged. Cut by "
        "tools/cut_dem.py."
        % (tile, dem["sha256"], row0, row1 - 1, col0, col1 - 1,
           dem["w"], dem["h"],
           "+" if args.lat >= 0 else "", args.lat,
           "+" if args.lon >= 0 else "", args.lon,
           half_ns, half_ew,
           max(0, int(args.half_extent_m - max(half_ns, half_ew))))
    )

    extras = [
        (TIFF_TAGS["ModelPixelScale"], 12, 3, (dem["sx"], dem["sy"], 0.0), True),
        (TIFF_TAGS["ModelTiepoint"], 12, 6,
         (0.0, 0.0, 0.0, tl_x, tl_y, 0.0), True),
        (TIFF_TAGS["GeoKeyDirectory"], 3, len(dem["geokeys"]),
         tuple(int(v) for v in dem["geokeys"]), True),
    ]
    tifffile.imwrite(
        args.out,
        dem["heights"][row0:row1, col0:col1],
        photometric="minisblack",
        planarconfig="contig",
        compression=None,
        metadata=None,           # no tifffile-owned metadata dict
        description=desc,
        rowsperstrip=heights.shape[0],
        extratags=extras,
    )

    print("cut %s: %dx%d px of %s (%s)" % (args.out, heights.shape[1],
                                           heights.shape[0], tile, tpath))
    print("  half-extent %.1f m N-S / %.1f m E-W, rows %d..%d cols %d..%d"
          % (half_ns, half_ew, row0, row1 - 1, col0, col1 - 1))
    print("  z range %.1f..%.1f" % (heights.min(), heights.max()))
    print("  tiepoint %.7f %.7f" % (tl_x, tl_y))
    print("  description: %s" % desc)


if __name__ == "__main__":
    main()