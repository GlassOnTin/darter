#!/bin/sh
# Build the regenerable demo-area bundle: tools/godot_smoke/areas/<name>/.
#
# Fully offline: every input is a committed fixture (tests/fixtures/), so
# this is the F-Droid build-path step — no network at build time. Each area
# dir is removed and regenerated from scratch, so stale payload cannot
# survive a run. pack.json / scene.packobj / record.jsonl / track.json land
# per area; areas/index.json (tracked) is not rewritten here.
#
# Usage: tools/godot_smoke/make_bundle.sh   (runs from the repo root)
# Requires: python3 (numpy+tifffile, already area_pack deps),
#           target/debug/sim_run  (cargo build --bin sim_run)
#
# Payload provenance per area: pack+scene = the committed fixtures through
# tools/area_pack.py at the area's seed; record = sim_run at the area's
# record seed / altitude flying the area's track; track = the source file
# copied verbatim (bundle-relative name track.json).
set -eu

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"
BUNDLE=tools/godot_smoke/areas
SCRATCH=$(mktemp -d "${TMPDIR:-/tmp}/darter-bundle.XXXXXX")
trap 'rm -rf "$SCRATCH"' EXIT

SIM=target/debug/sim_run
[ -x "$SIM" ] || { echo "make_bundle: $SIM missing (cargo build --bin sim_run)" >&2; exit 1; }
[ -f "$BUNDLE/index.json" ] || { echo "make_bundle: $BUNDLE/index.json missing" >&2; exit 1; }

# name lat lon seed recseed alt osm-fix dem-fix track-source
# (pins mirror tests/godot_pack.rs AREAS; the S3 discipline: measured, not
#  re-derived — a change here is a change there too)
while read -r name lat lon seed recseed alt osm dem trk; do
    [ -n "$name" ] || continue
    scratch="$SCRATCH/$name"

    echo "== $name: pack build"
    python3 tools/area_pack.py \
        --osm "$osm" --lat "$lat" --lon "$lon" --elevation glo30 \
        --dem-file "$dem" --seed "$seed" --out "$scratch"

    echo "== $name: validator"
    python3 tools/area_pack.py "$scratch"

    echo "== $name: record"
    "$SIM" --seed "$recseed" --duration 20 --alt "$alt" \
        --throttle 0.157 --vx -14 --determinism-check \
        --terrain "$scratch/terrain.bin" --track "$trk" --out "$scratch"

    rm -rf "$BUNDLE/$name"
    mkdir -p "$BUNDLE/$name"
    cp "$scratch/pack.json" "$BUNDLE/$name/pack.json"
    cp "$scratch/scene.obj" "$BUNDLE/$name/scene.packobj"
    cp "$scratch/flight.jsonl" "$BUNDLE/$name/record.jsonl"
    cp "$trk" "$BUNDLE/$name/track.json"
done <<'TABLE'
city 50.9060 -1.4012 11 21 30 tests/fixtures/osm_city.json tests/fixtures/dem_city.tif tools/godot_smoke/track/city.json
suburb 50.8989 -1.0586 5 7 35 tests/fixtures/osm_home_area.json tests/fixtures/dem_home_area.tif tools/godot_smoke/track/demo_track.json
coast 50.8130 -1.3070 13 23 30 tests/fixtures/osm_coast.json tests/fixtures/dem_coast.tif tools/godot_smoke/track/coast.json
hills 50.9761 -0.9457 17 29 60 tests/fixtures/osm_hills.json tests/fixtures/dem_hills.tif tools/godot_smoke/track/hills.json
TABLE

echo
printf '%-8s %-14s %11s %s\n' area file bytes sha256
find "$BUNDLE" -mindepth 2 -maxdepth 2 -type f | sort | while read -r f; do
    rel=${f#"$BUNDLE"/}
    name=${rel%%/*}
    file=${rel#*/}
    printf '%-8s %-14s %11s %s\n' "$name" "$file" "$(wc -c < "$f")" "$(sha256sum "$f" | cut -d' ' -f1)"
done
echo "BUNDLE_DONE areas=$(find "$BUNDLE" -mindepth 1 -maxdepth 1 -type d | wc -l)"