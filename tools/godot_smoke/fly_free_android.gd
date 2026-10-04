extends Node
# C4 free-fly probe (fly_free_android.tscn): the instrument for VISION's
# success criterion "a pilot connects a RadioMaster Pocket to an Android
# phone over USB and is flying Betaflight SITL in-sim within five minutes
# of install". A REAL pilot flies the packaged SITL with the physical
# radio; nothing synthetic is injected. Differences from
# fly_probe_android.gd:
#
#   - no pilot script: the pump hook reads Input device 0, which the REAL
#     radio feeds (kernel event12). Whoever holds the sticks is the pilot.
#   - the pump is CHUNKED per physics frame instead of one blocking
#     _ready loop: each frame pumps the ticks its wall delta owes
#     (250 Hz x seconds elapsed, clamped 1..12), so the Android input
#     stack gets frame boundaries to flush real HID events through —
#     the open C4a question (injected parse_input_event reaches the
#     axis table without a frame; kernel event12 was never measured
#     this way).
#   - an on-screen status Label (2D UI only; the 3D scenery render is
#     M2e): pads, armed state, throttle, alt — the grey screen's first fix.
#   - wall-stamped [c4] phase lines in user://fly.log (Time epoch s):
#     boot, pads, input_seen (first raw-axis change vs boot rest), armed,
#     climbing (the five-minute stamp), ended. The driver records the
#     install epoch at the adb install return; five-minute flow =
#     climbing.minus(install).
#
# Choreography for the pilot: launch the app, wait past the boot grace
# (the runner holds the arm box down for ~5 s), raise SA to arm; if a
# disable flag latches the arm switch (raising SA during a standing
# disable), drop SA back down — the latch clears, raise again. Throttle
# bottom = 0 after the Android span remap. The record runs 180 sim s and
# finishes itself; land or disarm whenever. The device's achieved
# sim/wall rate is whatever the pump manages (measured ~0.5x on the C3
# legs) — a slow-motion flight is fine and logged.
#
# Gates pulled from fly.log + flight.jsonl after the run:
#   1. pads >= 1 and the EdgeTX Pocket's guid among them at boot.
#   2. input_seen stamps a raw-axis CHANGE while pump frames run (real
#      HID streams through a chunked pump).
#   3. armed_at >= 0 and sustained climb with sticks the only input; the
#      record's r/i rows show pilot variation (no flat synthetic trace).
#   4. climbing - install <= 300 wall s.

const PROFILE := "aux 0 0 2 1700 2100 0 0;aux 1 1 2 1700 2100 0 0"
const FLIGHT_S := 180.0
const SEED := 11

var _capture: FileAccess
var _flyer: DarterFlyer
var _label: Label
var _boot_raw: PackedFloat32Array
var _last_ms: int = -1
var _last_log_ms: int = 0
var _input_seen := false
var _armed_stamped := false
var _climb_stamped := false

func _ready() -> void:
	_capture = FileAccess.open("user://fly.log", FileAccess.WRITE_READ)
	if _capture:
		_capture.seek_end()
	_log(["[c4] boot epoch=", _epoch()])

	var layer := CanvasLayer.new()
	add_child(layer)
	_label = Label.new()
	_label.position = Vector2(48, 120)
	_label.add_theme_font_size_override("font_size", 44)
	layer.add_child(_label)
	_set_status("booting")

	var flyer: DarterFlyer = DarterFlyer.new()
	_flyer = flyer
	# The REAL radio is the pilot: no injections anywhere below.
	flyer.flyer_set_radio_pocket(true)
	var bin: String = flyer.flyer_sitl_path()
	if bin.is_empty():
		_fail("flyer_sitl_path empty - SITL jniLib not packaged next to the extension")
		return
	var work := ProjectSettings.globalize_path("user://") + "flyprobe"
	DirAccess.make_dir_recursive_absolute(work)
	_log(["[c4] sitl=", bin, " work=", work])

	var pads := Input.get_connected_joypads()
	_log(["[c4] pads=", pads.size()])
	for p in pads:
		_log(["[c4] pad=", p, " name=", Input.get_joy_name(p),
				" guid=", Input.get_joy_guid(p)])

	var err: String = flyer.flyer_start(bin, work, FLIGHT_S, SEED, 0.0, 0.0, 0.0, PROFILE)
	if err != "":
		_fail("flyer_start: %s" % err)
		return

	var pd: Dictionary = flyer.flyer_state_dict()
	_boot_raw = pd["pocket_raw"]
	_log(["[c4] start raw_rest=", str(_boot_raw), " epoch=", _epoch()])
	_set_status("pumping — rise SA to arm")

func _physics_process(_delta: float) -> void:
	if _flyer == null:
		return
	var now := Time.get_ticks_msec()
	var due := 4
	if _last_ms >= 0:
		due = clampi(int(float(now - _last_ms) * 0.25), 1, 12)
	_last_ms = now

	var e: String = _flyer.flyer_pump(due)
	if e != "":
		_fail("flyer_pump: %s" % e)
		return
	var pd: Dictionary = _flyer.flyer_state_dict()
	_stamps(pd, now)
	if int(pd["ticks_done"]) >= int(pd["ticks_total"]):
		_end(pd)
		return
	if now - _last_log_ms >= 500:
		_last_log_ms = now
		_diag(pd)
	_set_status(_status(pd))


# Phase stamps: the first raw-axis change vs boot rest (input_seen), the
# first armed row, the first sustained climb (the VISION stamp).
func _stamps(pd: Dictionary, now: int) -> void:
	var raw: PackedFloat32Array = pd["pocket_raw"]
	if not _input_seen and raw.size() == _boot_raw.size():
		for i in raw.size():
			if absf(raw[i] - _boot_raw[i]) > 0.05:
				_input_seen = true
				_log(["[c4] input_seen slot=", i, " rest=", str(_boot_raw[i]),
						" now=", str(raw[i]), " epoch=", _epoch(), " wall_ms=", now])
				break
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	if not _armed_stamped and armed_at >= 0.0:
		_armed_stamped = true
		_log(["[c4] armed epoch=", _epoch(), " sim=", "%.2f" % armed_at])
	if not _climb_stamped and armed_at >= 0.0 and float(pd["max_alt"]) > 1.5:
		_climb_stamped = true
		_log(["[c4] climbing epoch=", _epoch(), " sim=", "%.2f" % armed_at,
				" alt=", "%.1f" % float(pd["max_alt"])])


func _diag(pd: Dictionary) -> void:
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	_log(["[c4] t=", "%.2f" % (float(int(pd["ticks_done"])) * 0.004),
			" epoch=", _epoch(),
			" armed=", "%.2f" % armed_at,
			" alt=", "%.1f" % float(pd["max_alt"]),
			" thr=", "%.2f" % float(pd["pocket_throttle"]),
			" aux3=", int(pd["pocket_aux3_us"]),
			" servos=", int(pd["servo_packets"]),
			" samples=", int(pd["pocket_samples"]),
			" raw=", str(pd["pocket_raw"])])


func _status(pd: Dictionary) -> String:
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	if armed_at < 0.0:
		return "idle — arm: SA up (if it latches, SA down once)"
	return "FLYING alt=%.1f m thr=%d%%\nsim=%.0f/%.0f s  sticks live" % [
			float(pd["max_alt"]),
			int(round(float(pd["pocket_throttle"]) * 100.0)),
			float(int(pd["ticks_done"])) * 0.004,
			float(int(pd["ticks_total"])) * 0.004]


func _set_status(text: String) -> void:
	if _label:
		_label.text = text


func _end(pd: Dictionary) -> void:
	var hashv: String = _flyer.flyer_finish()
	if hashv.length() != 16:
		_fail("flyer_finish returned %s" % hashv)
		return
	_log(["[c4] ended epoch=", _epoch(), " hash=", hashv,
			" ticks=", int(pd["ticks_done"]), " max_alt=", str(pd["max_alt"]),
			" servos=", int(pd["servo_packets"]),
			" samples=", int(pd["pocket_samples"]),
			" input_seen=", _input_seen,
			" climb_stamped=", _climb_stamped])
	_set_status("flight over — pull fly.log")
	get_tree().quit(0)


func _epoch() -> String:
	return "%.3f" % Time.get_unix_time_from_system()


func _log(parts: Array) -> void:
	var msg := ""
	for p in parts:
		msg += str(p)
	print(msg)
	if _capture:
		_capture.store_line(msg)
		_capture.flush()


func _fail(why: String) -> void:
	print("C4 ERROR: ", why)
	_log(["[c4] FLYER ERROR: ", why])
	_set_status("error: " + why)
	get_tree().quit(1)
	# quit is deferred; a return here only skips further work in this stack.