# T5 smoke-test replay: reads a darter flight record (JSONL, env
# DARTER_RECORD), drives a camera-follow scene from it at the Movie Maker
# frame rate, and writes back per-frame sampled positions, brightness
# mean/variance and wall-clock frame deltas (env DARTER_REPLAY_OUT).
#
# Frames are counted in _process, which Movie Maker mode drives exactly once
# per --fixed-fps frame. The script quits itself after the last frame's
# capture completes, so every entry has a captured image.
#
# Coordinates: the record is ENU with z up (x east, y north). Godot is Y up,
# right-handed, so a record position (px, py, pz) is placed at
# (px, pz, -py). The JSON carries the RAW record coordinates; the Rust test
# compares those against the record directly.
extends Node3D

const MOVIE_FPS := 30.0
const SECONDS := 10.0

var rows: Array = []
var hz := 250.0
var total_frames := 0
var entries: Array = []
var last_usec := 0

var quad: MeshInstance3D
var cam: Camera3D
# Set when the last frame has been written. In Movie Maker mode get_tree
# .quit() takes effect only after the current iteration, and one extra
# _process call still runs; this flag keeps it from appending a 301st
# sample or rewriting the file.
var done := false


func _ready() -> void:
	_build_scene()
	var path := OS.get_environment("DARTER_RECORD")
	if path == "":
		push_error("DARTER_RECORD not set")
		get_tree().quit(1)
		return
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		push_error("cannot open record %s" % path)
		get_tree().quit(1)
		return
	while not f.eof_reached():
		var line := f.get_line()
		if line == "" or line.begins_with("{\"schema"):
			continue
		var j = JSON.parse_string(line)
		if j is Dictionary:
			rows.append(j)
	if rows.size() < 3:
		push_error("record too short")
		get_tree().quit(1)
		return
	# Measured from the record itself; rows[0].t is 0.000 and the tick is
	# uniform.
	hz = 1.0 / (float(rows[2]["t"]) - float(rows[1]["t"]))
	total_frames = int(SECONDS * MOVIE_FPS)
	last_usec = Time.get_ticks_usec()


func _build_scene() -> void:
	var env := Environment.new()
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var ground := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(2000, 2000)
	ground.mesh = pm
	var gm := StandardMaterial3D.new()
	gm.albedo_color = Color(0.35, 0.45, 0.3)
	ground.material_override = gm
	add_child(ground)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-45.0, 30.0, 0.0)
	sun.shadow_enabled = true
	add_child(sun)

	quad = MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.4, 0.12, 0.4)
	quad.mesh = bm
	var qm := StandardMaterial3D.new()
	qm.albedo_color = Color(0.85, 0.2, 0.1)
	quad.material_override = qm
	add_child(quad)

	cam = Camera3D.new()
	add_child(cam)
	cam.current = true


func _process(_delta: float) -> void:
	if done:
		return
	var fidx := entries.size()
	var now_usec := Time.get_ticks_usec()
	var dt_wall := now_usec - last_usec
	last_usec = now_usec
	var idx := clampi(int(round(fidx * hz / MOVIE_FPS)), 0, rows.size() - 1)
	var r: Dictionary = rows[idx]
	var px: float = r["px"]
	var py: float = r["py"]
	var pz: float = r["pz"]
	quad.position = Vector3(px, pz, -py)
	cam.position = Vector3(px, pz + 2.0, -py + 6.0)
	cam.look_at(quad.position, Vector3.UP)
	await RenderingServer.frame_post_draw
	var img: Image = get_viewport().get_texture().get_image()
	var bm_val := -1.0
	var bv_val := -1.0
	if img != null and not img.is_empty():
		var n := 0.0
		var s := 0.0
		var s2 := 0.0
		var h := img.get_height()
		var w := img.get_width()
		for y in range(0, h, 16):
			for x in range(0, w, 16):
				var l: float = img.get_pixel(x, y).get_luminance()
				s += l
				s2 += l * l
				n += 1.0
		var mean := s / n
		bm_val = mean
		bv_val = s2 / n - mean * mean
	entries.append(
		"{\"f\":%d,\"t\":%.3f,\"px\":%.6f,\"py\":%.6f,\"pz\":%.6f,\"bm\":%.9f,\"bv\":%.9f,\"du\":%d}" % [
			fidx, float(r["t"]), px, py, pz, bm_val, bv_val, dt_wall
		]
	)
	if entries.size() >= total_frames and not done:
		done = true
		var out := OS.get_environment("DARTER_REPLAY_OUT")
		var g := FileAccess.open(out, FileAccess.WRITE)
		if g == null:
			push_error("cannot write %s" % out)
			get_tree().quit(1)
			return
		var samples := ",".join(PackedStringArray(entries))
		g.store_string(
			"{\"movie_fps\":%d,\"frames\":%d,\"samples\":[%s]}" % [
				int(MOVIE_FPS), total_frames, samples
			]
		)
		g.close()
		print("REPLAY_DONE frames=%d hz=%.2f" % [entries.size(), hz])
		get_tree().quit()