extends Node
# M2a parity probe (tests/godot_live.rs): drives the DarterQuad extension
# class through one scripted core-mode flight — the same loop shape sim_run
# --mode core runs — and prints the record hash the suite compares against
# sim_run's. Any failure prints "PROBE ERROR: ..." and exits 1, so the
# suite's assertion shows the full renderer log.
#
# Env contract (all set by the suite):
#   DARTER_PROBE_RECORD   output path for flight.jsonl (required)
#   DARTER_PROBE_TICKS    250 Hz ticks to advance (required, > 0)
#   DARTER_PROBE_DURATION planned duration for the header (seconds)
#   DARTER_PROBE_SEED     record seed
#   DARTER_PROBE_THROTTLE scripted throttle, all four motors
#   DARTER_PROBE_TERRAIN  terrain.bin path ("" = flat ground)
# Spawn is fixed at (0, 0) with 0.5 m AGL, matching sim_run's defaults.


func _ready() -> void:
	var record := OS.get_environment("DARTER_PROBE_RECORD")
	if record.is_empty():
		_fail("DARTER_PROBE_RECORD unset")
	var ticks := OS.get_environment("DARTER_PROBE_TICKS").to_int()
	if ticks <= 0:
		_fail("bad DARTER_PROBE_TICKS %s" % OS.get_environment("DARTER_PROBE_TICKS"))

	var quad: DarterQuad = DarterQuad.new()
	if quad.tick_dt() != 0.004:
		_fail("tick_dt mismatch: %f" % quad.tick_dt())

	var thr: float = _env_f("THROTTLE", 0.0)
	if not quad.setup(OS.get_environment("DARTER_PROBE_TERRAIN"), 0.0, 0.0, 0.5):
		_fail("setup failed (godot error above)")
	quad.set_throttle(thr, thr, thr, thr)
	if not quad.record_open(record, _env_f("DURATION", 1.0), _env_i("SEED", 1)):
		_fail("record_open failed (godot error above)")

	var err: String = quad.advance_ticks(ticks)
	if err != "":
		_fail("advance_ticks: %s" % err)
	var hashv: String = quad.finish_record()
	if hashv.length() != 16:
		_fail("finish_record returned %s" % hashv)

	var st: Dictionary = quad.state_dict()
	print("PROBE hash=%s ticks=%d soc=%s vbus=%s pos=%s" % [
		hashv, st["ticks_done"], st["soc"], st["vbus"], str(st["pos"])
	])
	get_tree().quit(0)


func _env_f(name: String, dflt: float) -> float:
	var v := OS.get_environment("DARTER_PROBE_" + name)
	return dflt if v.is_empty() else v.to_float()


func _env_i(name: String, dflt: int) -> int:
	var v := OS.get_environment("DARTER_PROBE_" + name)
	return dflt if v.is_empty() else v.to_int()


func _fail(why: String) -> void:
	print("PROBE ERROR: ", why)
	get_tree().quit(1)
	# quit is deferred; a return here only skips further work in this stack.