#! /usr/bin/env python3
"""Flight-envelope check: a record against a pack's terrain and buildings.

The corridor rung (Godot smoke T7) validated its track sites with an ad-hoc
script; this is that check, reusable per area. Two gates over every row of
the flown window:

- clearance: pz is at least CLEAR_MIN metres above the terrain the pack's
  sidecar terrain.bin carries, sampled with the same linear interpolation
  the OBJ drape and the Rust physics both use (terrain_h_at, imported) —
  so "clear" here is the drawn and simulated ground, not a second estimate;
- building incursion: no row inside a generator building whose wall + roof
  top is at most BUILD_TOP_CAP (the generator's own height caps sit at
  ~8.2 m, so the cap covers every generated wall; OSM-tagged taller walls
  are outside the cap and unchecked). The ring bbox expands by MARGIN as a
  cheap prefilter, then the exact point-in-ring test decides.

  tools/envelope_check.py <pack_dir> --record <record>
  tools/envelope_check.py <pack_dir> --record <record> --sites -15,-40,-70

--sites prints per site the nearest on-line row (t, px, py, pz, terrain
height, clearance) — how gate sites and their transcribed z get read off a
candidate record. Both modes exit nonzero on a violated gate.
"""
import argparse
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from area_pack import TERRAIN_FMT_VERSION, TERRAIN_MAGIC, terrain_h_at  # noqa: E402

CLEAR_MIN = 8.0          # metres of pz above the terrain every row
BUILD_TOP_CAP = 10.0     # only buildings topping out at/below this are gates
RING_MARGIN = 1.0        # bbox expansion before the point-in-ring test


def read_terrain_grid(path):
    """terrain.bin -> {'cols','rows','step','origin_x','origin_y','z'}.

    The 56-byte header is <IIII5d> (magic, fmt, cols, rows, step, origin_x,
    origin_y, z_min, z_max); the payload is rows*cols f64-LE row-major —
    the exact array the writer (write_terrain_bin) emits, re-shaped here
    the way terrain_h_at indexes it.
    """
    with open(path, "rb") as f:
        header = f.read(56)
        magic, fmt, cols, rows = struct.unpack("<IIII", header[:16])
        step, ox, oy, zmin, zmax = struct.unpack("<5d", header[16:56])
        if magic != TERRAIN_MAGIC or fmt != TERRAIN_FMT_VERSION:
            raise ValueError(
                "terrain.bin magic/fmt mismatch in %s (read %d/%d, want "
                "%d/%d)" % (path, magic, fmt, TERRAIN_MAGIC,
                            TERRAIN_FMT_VERSION))
        payload = f.read()
    if len(payload) < rows * cols * 8:
        raise ValueError("terrain.bin short: %d bytes for %d f64"
                         % (len(payload), rows * cols))
    flat = struct.unpack(
        "<%dd" % (rows * cols), payload[: rows * cols * 8])
    z = [list(flat[j * cols:(j + 1) * cols]) for j in range(rows)]
    return {"cols": cols, "rows": rows, "step": step,
            "origin_x": ox, "origin_y": oy, "z": z, "z_min": zmin,
            "z_max": zmax}


def read_record(path):
    """""(t, px, py, pz) rows; schema header lines skipped."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("{\"schema"):
                continue
            row = json.loads(line)
            rows.append((row["t"], row["px"], row["py"], row["pz"]))
    return rows


def point_in_ring(px, py, ring):
    """Even-odd ray cast (ring points [x, y], closed implicitly)."""
    inside = False
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i][0], ring[i][1]
        x2, y2 = ring[(i + 1) % n][0], ring[(i + 1) % n][1]
        if (y1 > py) != (y2 > py):
            xin = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xin:
                inside = not inside
    return inside


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pack_dir")
    ap.add_argument("--record", required=True)
    ap.add_argument("--sites", default=None,
                    help="comma-separated inline x values: print the "
                         "nearest on-line row for site picking (still runs "
                         "the gates)")
    args = ap.parse_args(argv)

    pack_path = os.path.join(args.pack_dir, "pack.json")
    with open(pack_path) as f:
        pack = json.load(f)
    grid = read_terrain_grid(os.path.join(args.pack_dir, "terrain.bin"))
    rows = read_record(args.record)
    if not rows:
        sys.exit("no record rows in %s" % args.record)

    tall = short = 0
    rings = []           # (bbox_x0, bbox_x1, bbox_y0, bbox_y1, ring, top)
    for b in pack["buildings"]:
        top = b["wall_h"] + b["roof_h"]
        if top > BUILD_TOP_CAP:
            tall += 1
            continue
        short += 1
        xs = [p[0] for p in b["ring"]]
        ys = [p[1] for p in b["ring"]]
        rings.append((min(xs) - RING_MARGIN, max(xs) + RING_MARGIN,
                      min(ys) - RING_MARGIN, max(ys) + RING_MARGIN,
                      b["ring"], top))

    bad_clear = []
    bad_bld = []
    min_clear = float("inf")
    for k, (t, px, py, pz) in enumerate(rows):
        h = terrain_h_at(grid, px, py)
        clear = pz - h
        if clear < min_clear:
            min_clear = clear
        if clear < CLEAR_MIN:
            bad_clear.append((k, t, px, py, pz, h, clear))
        for x0, x1, y0, y1, ring, top in rings:
            if not (x0 <= px <= x1 and y0 <= py <= y1):
                continue
            if point_in_ring(px, py, ring) and pz < h + top:
                # `top` is metres above local ground: the world-space roof
                # line at this row is the draped terrain under the ring plus
                # wall+roof (buildings sit on terrain.bin's drape, unmodified).
                bad_bld.append((k, t, px, py, pz, h + top))
                break

    if args.sites:
        for site in [float(s) for s in args.sites.split(",")]:
            k = min(range(len(rows)), key=lambda i: abs(rows[i][1] - site))
            t, px, py, pz = rows[k]
            h = terrain_h_at(grid, px, py)
            print("site x_g %+9.1f -> row %-5d t %6.2f  px %9.2f  py %+7.2f  "
                  "pz %8.3f  h %7.2f  pz-h %7.3f"
                  % (site, k, t, px, py, pz, h, pz - h))
    print("envelope rows=%d min_clearance=%.3f  buildings gated %d (%d over "
          "cap %s m)" % (len(rows), min_clear, short, tall, BUILD_TOP_CAP))
    if bad_clear:
        print("CLEARANCE < %.1f m: %d rows, first %d:"
              % (CLEAR_MIN, len(bad_clear), min(5, len(bad_clear))))
        for k, t, px, py, pz, h, clear in bad_clear[:5]:
            print("  row %-5d t %6.2f px %9.2f py %+7.2f pz %8.3f h %7.2f "
                  "pz-h %7.3f" % (k, t, px, py, pz, h, clear))
    if bad_bld:
        print("BUILDING INCURSION (%d rows, wall+roof <= %.1f m, first %d):"
              % (len(bad_bld), BUILD_TOP_CAP, min(5, len(bad_bld))))
        for k, t, px, py, pz, world_top in bad_bld[:5]:
            print("  row %-5d t %6.2f px %9.2f py %+7.2f pz %8.3f under "
                  "top %.2f" % (k, t, px, py, pz, world_top))
    if bad_clear or bad_bld:
        sys.exit(1)
    print("envelope: %s PASSED" % args.record)


if __name__ == "__main__":
    main()