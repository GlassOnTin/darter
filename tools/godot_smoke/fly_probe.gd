extends Node
# M2b closed-flyer probe (tests/godot_flyer.rs): drives the DarterFlyer
# extension class through one closed-flight run — spawn the Betaflight SITL
# child, apply + diff-verify the profile, fly the arming state machine, pump
# 250 Hz ticks, record — and prints the record hash plus run facts. Any
# failure prints "FLYER ERROR: ..." and exits 1 so the suite's assertion
# shows the full console log.
#
# Env contract (all set by the suite):
#   DARTER_FLYER_WORK      work dir; sitl_cwd, logs and flight.jsonl are
#                          created under it (record path == WORK/flight.jsonl)
#   DARTER_FLYER_BIN       Betaflight SITL binary path (required)
#   DARTER_FLYER_DURATION  flight duration seconds (default 13.0)
#   DARTER_FLYER_SEED      record seed
#   DARTER_FLYER_THROTTLE  Fly-phase throttle (default 0.16)
#   DARTER_FLYER_YAW / _YAW_UNTIL  yaw stick value and the sim time it releases
#   DARTER_FLYER_PROFILE   ';'-joined profile lines ("" = runner default)
#   DARTER_FLYER_SENSORS   set (nonempty, not "0") = sensor model on via
#                          flyer_set_sensors before flyer_start (sim_run's
#                          --sensors; sensor seed follows the run seed)
#   DARTER_FLYER_RADIO     set (nonempty, not "0") = the radio input path:
#                          flyer_set_radio_pocket(true) before flyer_start,
#                          then this probe plays a synthetic pilot through
#                          Godot's real Input pipeline — the same pipeline
#                          the physical radio feeds — per the measured
#                          RadioMaster Pocket HID map (guided captures,
#                          2026-10-04; raw Godot axis slots, not semantic):
#                            axis 3 = SA, -1/0/+1; +1 = arm end, aux3 -> 2000
#                            axis 4 = throttle, 0 at bottom .. +1 at top
#                            axis 0 = right-stick LR, +1 = right (aileron)
#                          Pilot schedule (sim time; armed_at from the state
#                          dict): SA held +1 from t=0.5 (the arming machine
#                          keeps the box down until FC grace clears); once
#                          armed_at is reported, throttle ramps 0 -> 0.30
#                          over 0.5 s then holds (a climb, not a hover — the
#                          setpoint and why it is not 0.16 sits on the
#                          PILOT_THROTTLE const below), and a short aileron
#                          burst +0.4 over armed+7.5..+8.0
#                          then releases (the test profile arms ANGLE on the
#                          same SA range, so the released stick self-levels;
#                          the burst sits late enough that the test's
#                          att-vs-truth gate window, armed+2..+7, sees only
#                          the hover). Injections are change-only and
#                          flushed so the Rust hook reads fresh values
#                          mid-pump.
# Terrain and wind stay off. Pumps 50 ticks (~0.2 s sim) per call,
# because flyer_pump wall-paces each tick internally: chunked pumping keeps
# the same wall-clock pacing, per-call state visible on the caller side.


func _ready() -> void:
	var work := OS.get_environment("DARTER_FLYER_WORK")
	if work.is_empty():
		_fail("DARTER_FLYER_WORK unset")
	var bin := OS.get_environment("DARTER_FLYER_BIN")
	if bin.is_empty():
		_fail("DARTER_FLYER_BIN unset")

	var flyer: DarterFlyer = DarterFlyer.new()
	var sensors := OS.get_environment("DARTER_FLYER_SENSORS")
	if sensors != "" and sensors != "0":
		flyer.flyer_set_sensors(true)
	var radio := OS.get_environment("DARTER_FLYER_RADIO")
	var radio_on := radio != "" and radio != "0"
	if radio_on:
		flyer.flyer_set_radio_pocket(true)
	var err: String = flyer.flyer_start(bin, work,
			_env_f("DURATION", 13.0), _env_i("SEED", 5), _env_f("THROTTLE", 0.16),
			_env_f("YAW", 0.0), _env_f("YAW_UNTIL", 0.0),
			OS.get_environment("DARTER_FLYER_PROFILE"))
	if err != "":
		_fail("flyer_start: %s" % err)

	var last_ax := {}
	while true:
		var e: String = flyer.flyer_pump(50)
		if e != "":
			_fail("flyer_pump: %s" % e)
		var pd: Dictionary = flyer.flyer_state_dict()
		if radio_on:
			_pilot(pd, last_ax)
		if int(pd["ticks_done"]) >= int(pd["ticks_total"]):
			break

	var st: Dictionary = flyer.flyer_state_dict()
	var hashv: String = flyer.flyer_finish()
	if hashv.length() != 16:
		_fail("flyer_finish returned %s" % hashv)
	print("FLYER hash=%s ticks=%d armed_at=%s max_alt=%s" % [
		hashv, st["ticks_done"], st["armed_at"], str(st["max_alt"])
	])
	get_tree().quit(0)


func _env_f(name: String, dflt: float) -> float:
	var v := OS.get_environment("DARTER_FLYER_" + name)
	return dflt if v.is_empty() else v.to_float()


func _env_i(name: String, dflt: int) -> int:
	var v := OS.get_environment("DARTER_FLYER_" + name)
	return dflt if v.is_empty() else v.to_int()


# The synthetic pilot (DARTER_FLYER_RADIO): one chunk of stick values,
# injected on change only (axis state persists; proved headless in the
# guided-capture scratch rig). armed_at < 0 = still arming.
#
# Throttle 0.30 sits well above this preset's hover point (~0.16-0.17:
# measured on both the CLI scratch — it never left the ground — and the
# first live run, which lifted off on a ~0.1% rpm hair and climbed at
# ~0.95 m/s; a razor edge no committed test may sit on). The climb is
# then drag-limited and deterministic in shape; the burst window and the
# settled att-vs-truth gates read the same before/after.
const PILOT_THROTTLE := 0.30

func _pilot(pd: Dictionary, last: Dictionary) -> void:
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	var t: float = int(pd["ticks_done"]) * 0.004
	_pilot_inj(last, 3, 1.0 if t >= 0.5 else 0.0)
	if armed_at < 0.0:
		return
	_pilot_inj(last, 4, PILOT_THROTTLE * clampf((t - armed_at) / 0.5, 0.0, 1.0))
	var dt_a: float = t - armed_at
	_pilot_inj(last, 0, 0.4 if (dt_a >= 7.5 and dt_a <= 8.0) else 0.0)


func _pilot_inj(last: Dictionary, axis: int, val: float) -> void:
	if last.has(axis) and abs(last[axis] - val) <= 0.001:
		return
	last[axis] = val
	var ev := InputEventJoypadMotion.new()
	ev.device = 0
	ev.axis = axis
	ev.axis_value = val
	Input.parse_input_event(ev)
	Input.flush_buffered_events()


func _fail(why: String) -> void:
	print("FLYER ERROR: ", why)
	get_tree().quit(1)
	# quit is deferred; a return here only skips further work in this stack.