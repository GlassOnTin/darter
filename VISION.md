# VISION — Anisoptera flight simulator

Working name: Darter (a dragonfly family; the repo is `darter` and renames cheaply
before an F-Droid listing if the name collides). Licence: AGPL-3.0-or-later.
Status: proposal. This document states intent and constraints, not a finished design.

## Statement

A free-software drone flight simulator that runs real flight controller firmware
(Betaflight, INAV, ArduPilot) against honest physics, in a world built from
OpenStreetMap, on Android and desktop, with no ads, no trackers, and no paid
unlocks.

Each ingredient exists somewhere on its own. Closed sims (Liftoff, Velocidrone,
Uncrashed) have polish but approximate the flight stack, ship baked scenery, and
several carry ads or subscriptions. Research projects (Flightmare,
gym-pybullet-drones, rotorpy) have good physics but are not products. The flight
stacks themselves ship SITL builds that nobody has wired into an end-user
simulator. No open, tracker-free drone simulator is packaged on F-Droid as of
September 2026. The combination of real firmware, real places, and real physics
in one open app is the product.

## The story we are building toward

A pilot planning to map a quarry on Saturday imports the site from OpenStreetMap
on Thursday, loads their real parameter file, sets the wind forecast, and flies
the mission in the simulator against the actual terrain, with the actual
firmware deciding what happens when the battery sags or the GPS drops. A
freestyle pilot tunes rates and RPM filters in the real Betaflight Configurator
against a simulated quad with a synthetic vibration spectrum, then copies the
parameter diff to the real one. Neither pays, neither sees an ad, and both can
read every line of code that produced what they saw.

## Who it is for

1. FPV pilots learning or practising, with an EdgeTX/ELRS radio (the project's
   reference input device is a RadioMaster Pocket over USB).
2. Multirotor pilots planning mapping or survey missions who want to rehearse
   against real terrain before flying it.
3. Firmware tinkerers who want the real stack plus synthetic IMU noise and
   vibration profiles, so filtering and tuning can be practised safely.
4. Our own navigation research. The anisoptera homing stack currently trains in
   Unreal via `sim/ue_bridge.py`. This simulator offers a second world with
   deterministic, seeded, headless runs and an OSM-backed reality that the UE
   world cannot match.

## Pillars

### 1. Real flight stacks, never reimplementations

- Run upstream SITL binaries as child processes: Betaflight SITL, INAV SITL,
  ArduPilot SITL. We do not emulate STM32 silicon and we do not write a parallel
  PID implementation that pretends to be the firmware.
- Protocols: MSP to Betaflight/INAV, MAVLink to ArduPilot, over emulated serial
  ports inside the app.
- Real parameters, real behaviours. Failsafe, GPS loss, and battery sag come
  from the firmware itself; the simulator only supplies honest inputs (IMU,
  GNSS, wind, battery state) and an RC link model.
- The real configurators and ground stations connect to the simulated vehicle:
  Betaflight Configurator, INAV Configurator, Mission Planner, QGroundControl.
  Parameter files move between sim and field unmodified.
- A built-in simple controller ships as a fallback for instant start and for
  platforms where SITL cannot run. It is clearly labelled and never presented as
  the real firmware.
- Aircraft presets are data files (5" freestyle 4S/6S, whoop, 7" long range, 10"
  mapping platform): mass, Kv, prop thrust data, battery model. Where a number
  is measured the preset says so; where it is estimated the UI says so.

### 2. Physics that earns trust

- 6-DOF rigid body with per-axis drag coefficients from frame geometry.
- Motors as first-order RPM dynamics with thrust and torque curves, coupled to
  the battery model (open-circuit voltage, internal resistance sag, capacity
  discharge from current integration). Air density from altitude and user-set
  temperature and pressure.
- Prop wash and ground effect, including the thrust efficiency loss and
  turbulence a quad meets descending through its own wash.
- Wind: a user-set mean vector plus gusts from a standard spectral model
  (Dryden or von Kármán, choice exposed). Wind shadowing around buildings is a
  stretch goal, not a launch claim.
- Physics substeps matched to the configured PID loop rate, 500 Hz to 8 kHz, so
  the firmware loop sees plausible gyro data. Configurable IMU noise, bias
  random walk, and a vibration spectrum keyed to simulated motor RPM, which is
  what makes RPM filtering tunable in-sim.
- Determinism: recorded inputs plus seeded wind reproduce a flight bit-for-bit.
  This serves CI regressions, bug reports, and training runs.
- Honesty over claims: the model and its known errors are published, starting
  with residuals against recorded flight logs (step response, thrust curves).
  "Realistic" is a measured property here or it is not claimed.

### 3. The world: fly real places

- OSM import: pick a bounding box, fetch buildings (footprints, roof shapes,
  levels), roads, waterways and water bodies, landuse, mapped trees, and
  landuse=forest scatter, over Copernicus GLO-30 DEM elevation. Output is a
  versioned, offline, shareable area pack.
- Procedural terrain generator for terrain without usable OSM data, and curated
  demo areas bundled with the app (city, suburb, coast, hills).
- Textures from CC0 sources (Poly Haven, ambientCG, Kenney) plus procedural
  facades and foliage. No AI-generated texture assets; F-Droid policy and
  provenance both get messy otherwise.
- Trees and buildings instanced with LoD and wind animation in the vertex
  shader. Water from OSM polygons with animated normals, depth colouring, and
  cheap reflections.
- Rendering budget is mid-range Android GPU from day one. Mobile perf targets
  shape the renderer early; a desktop-only engine that ports badly is the
  classic failure mode here.
- A track editor with checkpoints; tracks and area packs are plain files, easy
  to share without any server.

### 4. Radio link and latency realism

- Input paths: CRSF over USB (ELRS 3.x, lowest latency and highest fidelity),
  USB HID joystick (EdgeTX joystick mode, works with the Pocket today), USB
  serial with OTG, Bluetooth gamepads, keyboard on desktop.
- Configurable added latency, published as measured input-to-photon per
  platform rather than promised.
- RC link model with configurable packet loss feeding the firmware's real
  failsafe logic.
- Video feed models: clean FPV camera by default; analog static at range;
  blocky degradation for digital systems; configurable video latency and camera
  parameters (FOV, tilt, distortion, rolling shutter). No proprietary codec
  emulation, just the perceptible failure modes.

### 5. Open by construction

- Code AGPL-3.0-or-later. Map data under ODbL with visible attribution. Assets
  CC0 or CC-BY with sources recorded. Upstream firmware runs unmodified as
  separate GPLv3 processes, sources linked.
- No ads, no analytics, no accounts, no Google Play Services, no proprietary
  dependencies. Every network endpoint (Overpass, DEM, firmware downloads) is
  documented and only free data sources are used.
- Buildable from source on F-Droid, including the SITL binaries; packaging is a
  milestone in itself, not an afterthought.
- Flight logs stay on the device unless the user exports them, and export in
  the firmware's native formats (blackbox for Betaflight/INAV, dataflash for
  ArduPilot) so existing tools keep working.
- Headless mode and a small script API serve CI, physics regression tests, and
  the research training loop. MAVLink out means QGC and Mission Planner treat
  the simulator as just another vehicle.

## Architecture sketch

The physics core is a standalone library (Rust preferred for the headless
binary and research API), with Godot 4 as the renderer client for desktop and
Android. Flight stacks run as child processes talking MSP or MAVLink over
emulated serial. The world importer is a CLI first (scriptable, CI-able) with an
in-app front end later. Protocols between core, renderer, and stacks stay small
and documented so any component can be replaced. This is a sketch; the first
design document will revise it, and the UE bridge keeps serving the research
work until this world can take over.

## Milestones

- M0, desktop prototype. Rigid body, motor, battery, and prop wash models at 8
  kHz substeps; built-in fallback controller; Betaflight SITL over MSP; one
  hand-built demo scene; CRSF-over-USB and HID input; latency and frame-time
  instrumentation from the first commit.
- M1, the world. OSM importer producing area packs; curated demo areas;
  procedural fallback terrain; LoD vegetation, buildings, water; track files
  and checkpoints.
- M2, Android and F-Droid. Godot export, OTG serial and HID input, packaging
  and fastlane metadata, mid-range GPU perf pass, F-Droid submission.
- M3, more stacks. ArduPilot SITL with MAVLink and EKF, mission rehearsal from
  .plan files (coverage, overlap, battery against real terrain), INAV SITL,
  configurator and ground-station connectivity, native log export.
- M4, realism pass. Gust models, weather visuals, video feed models, measured
  motor and prop presets, IMU vibration profiles for filter tuning, published
  physics residuals.
- M5, together. Ghost racing, multiplayer rooms over plain UDP, community track
  sharing.

## Non-goals (v1)

- Emulating proprietary firmware (DJI and friends).
- AAA photorealism. Readable, pleasant, performant scenery is the bar.
- Fixed-wing and helicopter models at launch. Multirotor first, they are wanted
  later.
- Cloud accounts, online telemetry, monetisation of any kind.

## Success criteria

- A pilot connects a RadioMaster Pocket to an Android phone over USB and is
  flying Betaflight SITL in-sim within five minutes of install, with the real
  Betaflight Configurator connected for tuning.
- Importing a city block's OSM bbox yields a flyable, fully offline area in
  minutes, with attribution visible in the scene and the pack metadata.
- Input-to-photon latency is measured and published for desktop and a reference
  Android device.
- Replayed flights are bit-identical; CI runs physics regressions on every
  commit.
- The anisoptera homing stack trains in the headless sim and the results are
  comparable with the UE pipeline.
- F-Droid packaging is accepted, built from source including SITL binaries.

## Open questions

- Godot 4 confirmed as the renderer, and the exact F-Droid build path for
  Godot 4 Android exports; verify with a smoke-test app early (M0).
- SITL build maturity: Betaflight and INAV arm64 SITL targets are young; if
  they lag, we build them upstream rather than fork.
- How far to push digital video link emulation without touching proprietary
  codecs.
- APK size and F-Droid review when shipping three SITL binaries.
- Multiplayer infrastructure: plain UDP rooms are likely enough; anything
  hosted needs a licence-compatible story.
- Whether wind shadowing around OSM buildings is worth its complexity, and when.