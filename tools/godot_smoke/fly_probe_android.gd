extends Node
# On-device M2d fly probe (fly_probe_android.tscn): the closed-flyer probe
# of fly_probe.gd packaged into the darter-smoke APK and run from the app —
# on Android there is no env contract, so the desktop DARTER_FLYER_* vars
# become app-internal values plus the extension's own SITL resolver:
#
#   SITL  = flyer_sitl_path() — the jniLib libbetaflight_sitl.so in the
#           app's nativeLibraryDir; empty string = not packaged (and on
#           desktop, where ext/ holds no such sibling — the probe then
#           fails by design; the desktop probe passes its own path).
#   WORK  = globalized user://flyprobe — the SITL cwd, its logs, and
#           flight.jsonl land there; eeprom.bin is created in the cwd.
#   else  duration 16 s, seed 11, sensors off, default profile — the same
#           values as the desktop pocket run in tests/godot_flyer.rs,
#           including the ANGLE arm profile const below.
#
# The synthetic pilot is the desktop one retablotted onto the ANDROID
# Pocket slots (pocket_input's doc carries both tables): SA = axis 4
# (+1 = up = arm end), throttle = axis 2 spanning -1 (bottom) .. +1 (top),
# aileron = axis 0. The throttle stick scale therefore maps the
# desktop's frac-in-0..1 to frac*2 - 1. With the physical radio attached
# it must NOT be touched during the flight: at rest no axis events
# arrive, so injections win the merged axis state; a real stick move
# would override them.
#
# Output is file-first for the same reason as the hid capture probe
# (the OEM kills the app the moment it is backgrounded): every line is
# flushed to user://fly.log and also print()ed. Gated off-device by
# pulling both user://fly.log and user://flyprobe/flight.jsonl over adb:
# FLYER hash= 16 hex, armed_at >= 0, closed record present.

const PILOT_THROTTLE := 0.30
const PROFILE := "aux 0 0 2 1700 2100 0 0;aux 1 1 2 1700 2100 0 0"

var _capture: FileAccess

func _ready() -> void:
	_capture = FileAccess.open("user://fly.log", FileAccess.WRITE_READ)
	if _capture:
		_capture.seek_end()
	_log(["[flyprobe-android] start godot=", Engine.get_version_info().get("patch", "?")])

	var flyer: DarterFlyer = DarterFlyer.new()
	# Sensors stay off (isolating the input path; see fly_probe.gd).
	flyer.flyer_set_radio_pocket(true)
	var bin: String = flyer.flyer_sitl_path()
	if bin.is_empty():
		_fail("flyer_sitl_path empty - SITL jniLib not packaged next to the extension")
		return

	# Musend smoke check (the C3 no-flight hunt's exec/loopback-UDP
	# instrument, kept): run the sibling through the extension — Godot's
	# OS.execute hangs the Android main thread (the grey screen), the
	# extension spawns the way the SITL is spawned, so the exec context is
	# identical. Broken exec or loopback UDP names itself here before a
	# flight is attempted.
	var musend: PackedStringArray = flyer.flyer_musend(
			bin.get_base_dir().path_join("libmusend.so"))
	for line in musend:
		_log(["[musend] ", line])
	var work := ProjectSettings.globalize_path("user://") + "flyprobe"
	DirAccess.make_dir_recursive_absolute(work)
	_log(["[flyprobe-android] sitl=", bin, " work=", work])

	var err: String = flyer.flyer_start(bin, work, 16.0, 11, 0.16, 0.0, 0.0, PROFILE)
	if err != "":
		_fail("flyer_start: %s" % err)
		return

	var last_ax := {}
	while true:
		var e: String = flyer.flyer_pump(50)
		if e != "":
			_fail("flyer_pump: %s" % e)
			return
		var pd: Dictionary = flyer.flyer_state_dict()
		_pilot(pd, last_ax)
		if int(pd["ticks_done"]) % 500 == 0:
			_diag(pd)
		if int(pd["ticks_done"]) >= int(pd["ticks_total"]):
			break

	var st: Dictionary = flyer.flyer_state_dict()
	var hashv: String = flyer.flyer_finish()
	if hashv.length() != 16:
		_fail("flyer_finish returned %s" % hashv)
		return
	_log(["[flyprobe-android] FLYER hash=", hashv, " ticks=", int(st["ticks_done"]),
			" armed_at=", str(st["armed_at"]), " max_alt=", str(st["max_alt"]),
			" servo_packets=", int(st["servo_packets"]),
			" pocket_throttle=", str(st["pocket_throttle"]),
			" pocket_aux3_us=", int(st["pocket_aux3_us"]),
			" pocket_samples=", int(st["pocket_samples"]),
			" pocket_raw=", str(st["pocket_raw"])])
	get_tree().quit(0)


# One diagnostic row per 10th pump chunk (2 s of sim; trimmed from every
# chunk once the C3 leg was proven — full chunk history lives in
# flight.jsonl): the raw axis-table slots, the hook's mapped output, and
# the servo-packet drain count — together they read "injections reached
# the table", "the hook fed the FC", or "the servo leg never answered"
# directly from state.
func _diag(pd: Dictionary) -> void:
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	var raw: PackedFloat32Array = pd["pocket_raw"]
	_log(["[diag] t=", "%.2f" % (float(int(pd["ticks_done"])) * 0.004),
			" armed_at=", str(armed_at),
			" thr=", str(pd["pocket_throttle"]),
			" aux3=", int(pd["pocket_aux3_us"]),
			" raw=", str(raw),
			" servos=", int(pd["servo_packets"]),
			" alt=", str(pd["max_alt"])])


func _log(parts: Array) -> void:
	var msg := ""
	for p in parts:
		msg += str(p)
	print(msg)
	if _capture:
		_capture.store_line(msg)
		_capture.flush()


# The synthetic pilot: one chunk of stick values, injected on change only
# (axis state persists — the guided-capture rigs proved nothing clears it).
# armed_at < 0 = still arming. Android slot map: SA 4, throttle 2, aileron 0.
func _pilot(pd: Dictionary, last: Dictionary) -> void:
	var aa_v: Variant = pd["armed_at"]
	var armed_at: float = -1.0 if aa_v == null else float(aa_v)
	var t: float = int(pd["ticks_done"]) * 0.004
	_pilot_inj(last, 4, 1.0 if t >= 0.5 else 0.0)
	if armed_at < 0.0:
		return
	_pilot_inj(last, 2,
			(PILOT_THROTTLE * 2.0 - 1.0) * clampf((t - armed_at) / 0.5, 0.0, 1.0))
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
	_log(["[flyprobe-android] FLYER ERROR: ", why])
	get_tree().quit(1)
	# quit is deferred; a return here only skips further work in this stack.