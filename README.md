# Darter

Darter is a quadcopter flight simulator that renders real places. A Rust core
(`darter-core`) runs the flight physics, f64 with 8 kHz substeps, motor and
battery models included; Godot 4 renders the scene. The model and its known
errors are published in [docs/physics.md](docs/physics.md): what is
implemented, where every number comes from, and what is not attempted. The world is built from
OpenStreetMap building, road and tree data plus Copernicus GLO-30 elevation
tiles, so the ground under the quad comes from real map data at real
coordinates. Where DEM data is unusable, the importer can instead build a
playground over seeded procedural relief (`tools/area_pack.py --elevation
procedural`), alone or layered under real OSM features.

The motivation is to make this a virtual hardware test platform. The real
flight controller is flown against the simulator, either as emulated firmware
(SITL) or as a model of flight-controller behaviour, so tuning and behaviour
checks happen in the sim before anything flies for real. Today only the
physics and renderer are in place; the firmware side is on the milestone list.

Licence: AGPL-3.0-or-later. `VISION.md` is the product authority: it states
the direction and the milestone list; this README describes the repo as it
stands today.

[![tests](https://github.com/GlassOnTin/darter/actions/workflows/test.yml/badge.svg)](https://github.com/GlassOnTin/darter/actions/workflows/test.yml)

## What runs today

- Four demo areas (city, suburb, coast, hills) built offline from committed
  map fixtures and bundled into the app; the coast's sea paints as water from
  the mapped coastline, islets staying land. A picker screen lists them with
  the OpenStreetMap and Copernicus attribution lines, and each flies a recorded
  20-second demo flight through six gate rings scored in the HUD.
- The same bundle runs on Linux and Android. Frame time on a recent Android
  phone (OPPO CPH2655, Adreno 830) averages 11.1 ms in the demo replay, about
  90 fps.
- The physics core produces byte-identical flight records on x86_64 and
  aarch64, and the render suites pin per-area pixel statistics with dated
  floors so renderer changes are caught, not averaged away.
- The core also runs inside Godot as a GDExtension class (`DarterQuad`,
  from `tools/gdext`). Its flight loop is parity-tested byte-for-byte
  against `sim_run` on flat and terrain-spawned flights — the live plant
  the later milestones (on-screen/radio flying, on-device SITL) build on.
  Not wired to any app screen yet.

## What is not built yet

Flying with a radio or on-screen controls, real-firmware SITL (Betaflight,
INAV, ArduPilot), weather, and multiplayer. The current app replays recorded
demo flights. These are milestones M2 onward of `VISION.md`.

## Try it

Release APKs are attached to
[Releases](https://github.com/GlassOnTin/darter/releases). They install as
"darter smoke" (package `org.darter.smoke`, arm64-v8a; debug-signed, so
sideload and launch from an installer that allows unknown sources). Tested on
an Android 16 phone with an Adreno 830. Launch, pick an area, and watch the
20-second flight; the replay exits by itself.

On a desktop, a demo flight renders through the same path the render tests
use, as 600 PNG frames at 30 fps:

```
cargo build --bin sim_run
python3 tools/godot_smoke/make_bundle.sh   # rebuilds areas/* from fixtures, offline
mkdir -p /tmp/darter-movie
DARTER_AREA=city tools/godot/bin/godot --path tools/godot_smoke \
  --rendering-method gl_compatibility --rendering-driver opengl3 \
  --write-movie /tmp/darter-movie/movie.png --fixed-fps 30 \
  --quit-after 610 res://pack_replay.tscn
```

`make_bundle.sh` needs python3 with numpy and tifffile, and `sim_run` (the
build above). The replay needs a Godot 4.7.2-stable editor binary at
`tools/godot/bin/godot`: the official Linux x86_64 editor from the Godot
releases page, unpacked there and verified against the checksum list this
repo commits at `tools/godot/SHA512-SUMS.txt`. Under an X display the command
above opens a window as well as writing frames; over SSH add `xvfb-run -a`
in front. Any of `city`, `suburb`, `coast`, `hills` works in `DARTER_AREA`.

## Building the Android APK

The complete recipe lives as the header comment of
`tools/godot_smoke/export_presets.cfg`: offline bundle build, throwaway
project copy with the picker as main scene, Godot android build template
install, gradle export. It needs the Android SDK (platform android-36,
build-tools 36.1.0), the pinned Godot editor, and cargo. Tagged commits get
the same build produced by CI and attached to the release.

## Tests

`cargo test` runs the offline suites: physics golden records, MSP framing,
track files, terrain, and pack regeneration and validation. Three renderer
or renderer-adjacent suites are `#[ignore]`d because they need the pinned
Godot binary and an X display:

```
cargo test --test godot -- --ignored        # T5: record-to-movie replay smoke
cargo test --test godot_pack -- --ignored   # T7/T8: pack replay + demo-areas gates
```

T7 replays the corridor demo through Godot and pins pixel statistics with
dated floors; T8 does the same for the four demo areas, checks every bundle
file byte-for-byte against a fresh offline regeneration, and exercises the
picker and the no-environment error path. Both run in CI on every push.

The third suite needs the extension built first (its parity flights drive
the DarterQuad GDExtension class and compare records byte-for-byte against
`sim_run --mode core` — flat and terrain-spawned, hash equality asserted
both ways):

```
cargo build --release --manifest-path tools/gdext/Cargo.toml
cargo test --test godot_live -- --ignored   # M2a: live plant vs sim_run parity
```

All three run in CI after the Godot and GDExtension build steps.

## Repo layout

| path | contents |
| --- | --- |
| `src/` | the `darter-core` crate; `src/bin/sim_run` is the flight recorder CLI |
| `docs/physics.md` | the applied-maths write-up: model equations, calibration status, known errors |
| `tools/area_pack.py` | OSM/DEM area importer and pack validator |
| `tools/godot_smoke/` | Godot 4 test project, demo bundle script, Android export preset |
| `tools/gdext/` | the standalone gdext crate: `DarterQuad`, the live plant (Godot bindings, never built by `cargo test`) |
| `tests/` | the cargo suites, including the Godot render gates |

## Data attribution

Map data: © OpenStreetMap contributors, ODbL 1.0. Elevation: © Copernicus DEM
/ ESA (GLO-30). Both lines are shown in the app's area picker, as the source
licences require.

## Licence

AGPL-3.0-or-later; see `LICENSE`.