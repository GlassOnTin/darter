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
# Terrain, sensor and wind stay off. Pumps 50 ticks (~0.2 s sim) per call,
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
	var err: String = flyer.flyer_start(bin, work,
			_env_f("DURATION", 13.0), _env_i("SEED", 5), _env_f("THROTTLE", 0.16),
			_env_f("YAW", 0.0), _env_f("YAW_UNTIL", 0.0),
			OS.get_environment("DARTER_FLYER_PROFILE"))
	if err != "":
		_fail("flyer_start: %s" % err)

	while true:
		var e: String = flyer.flyer_pump(50)
		if e != "":
			_fail("flyer_pump: %s" % e)
		var pd: Dictionary = flyer.flyer_state_dict()
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


func _fail(why: String) -> void:
	print("FLYER ERROR: ", why)
	get_tree().quit(1)
	# quit is deferred; a return here only skips further work in this stack.