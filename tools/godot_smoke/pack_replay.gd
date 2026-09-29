# T7 area-pack replay: builds the Godot scene from a darter area pack
# (env DARTER_PACK = pack directory with pack.json + scene.obj) and replays a
# darter flight record through it, same Movie Maker replay contract as
# replay.gd (env DARTER_RECORD, env DARTER_REPLAY_OUT).
#
# The pack loader is the renderer-side pack reader: it parses scene.obj
# (chunk-local vertices re-emitted per o group; face indices file-global
# 1-based; one material per chunk) into ONE ArrayMesh
# with a surface per material and asserts the o-group family counts against
# pack.json's counts — the same contract tools/area_pack.py's validator
# checks, now verified by the consumer side. A mismatch aborts the run.
#
# Coordinate frames: the pack is ENU with z up (x east, y north, z up), Godot
# is Y up, so a pack vertex (x, y, z) is placed at (x, z, -y). That is a
# proper rotation, so triangle orientation is preserved. The OBJ winds
# triangles CCW around the outward normal (verified against backface culling
# from the air, tools/area_pack.py); Godot's front faces are CLOCKWISE, so
# the loader reverses each face's index order and attaches per-vertex flat
# normals computed from the unflipped winding.
#
# Camera: a chase cam 10 m behind the quad along its recorded velocity and
# 4 m above, looking at it — a level transit shows the neighbourhood ahead
# and below. Per-frame the script records sampled positions, brightness
# mean/variance, the pixel-class fractions sky/veg/man-made sampled on a
# 16-px grid (every sample_every-th frame — the full-res readback cost ~25%
# of throughput on the Adreno 830 when run every frame), and wall-clock
# frame deltas for every frame; the Rust test
# asserts on all of these (positions vs the record, non-blank frames,
# man-made geometry actually filling part of the frame, and a second run
# reproducing the deterministic projection).

extends Node3D

const MOVIE_FPS := 30.0
const SECONDS := 20.0

# Fixed surface order (deterministic mesh build); a material present in the
# OBJ but missing here falls back to grey and is still emitted.
const MATERIAL_ORDER := [
	"grass", "asphalt", "concrete", "hedge", "fence",
	"brick_red", "brick_red_plain", "brick_buff", "brick_buff_plain",
	"render_white", "render_white_plain",
	"tile_brown", "slate", "bark",
	"foliage_a", "foliage_b", "foliage_c", "foliage_d",
]
const MATERIAL_COLORS := {
	"grass": Color(0.42, 0.52, 0.30),
	"asphalt": Color(0.16, 0.16, 0.17),
	"concrete": Color(0.60, 0.60, 0.58),
	"hedge": Color(0.22, 0.38, 0.18),
	"fence": Color(0.45, 0.42, 0.37),
	"brick_red": Color(0.52, 0.26, 0.19),
	"brick_red_plain": Color(0.56, 0.29, 0.21),
	"brick_buff": Color(0.70, 0.60, 0.46),
	"brick_buff_plain": Color(0.74, 0.64, 0.50),
	"render_white": Color(0.84, 0.82, 0.78),
	"render_white_plain": Color(0.88, 0.87, 0.83),
	"tile_brown": Color(0.42, 0.26, 0.17),
	"slate": Color(0.28, 0.30, 0.33),
	"bark": Color(0.33, 0.26, 0.19),
	"foliage_a": Color(0.28, 0.44, 0.19),
	"foliage_b": Color(0.24, 0.38, 0.16),
	"foliage_c": Color(0.33, 0.48, 0.22),
	"foliage_d": Color(0.27, 0.41, 0.24),
}

var rows: Array = []
var hz := 250.0
var total_frames := 0
var entries: Array = []
var last_usec := 0
var group_counts := {}

var quad: MeshInstance3D
var cam: Camera3D
# Set when the last frame has been written (see replay.gd: Movie Maker mode
# runs one extra _process after get_tree().quit()).
var done := false

# Pixel-class gate interval. 1 samples every frame (the desktop T7 contract);
# the Android replay carries no env and pays the readback tax, so it samples
# every 4th frame. DARTER_SAMPLE_EVERY overrides both (resolved in _ready).
var sample_every := 1

# Lighting preset (S1 art pass). Selected by env DARTER_LIGHTING; the tests
# never set it, so the default pins the canon run.
const LIGHTING_PRESETS := preload("lighting_presets.gd")


func _ready() -> void:
	sample_every = 4 if OS.has_feature("android") else 1
	var env_se := OS.get_environment("DARTER_SAMPLE_EVERY")
	if env_se != "":
		sample_every = maxi(1, int(env_se))
	_load_pack_and_build_scene()
	# Bundled record fallback, same reason as the pack (see above).
	var path := OS.get_environment("DARTER_RECORD")
	if path == "":
		path = "res://record/flight.jsonl"
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
	hz = 1.0 / (float(rows[2]["t"]) - float(rows[1]["t"]))
	total_frames = int(SECONDS * MOVIE_FPS)
	if rows.size() < total_frames * hz / MOVIE_FPS:
		push_error("record shorter than the replay window (%d rows)" % rows.size())
		get_tree().quit(1)
		return
	last_usec = Time.get_ticks_usec()


func _load_pack_and_build_scene() -> void:
	# Android launches carry no environment: fall back to the pack bundled in
	# the APK (tools/godot_smoke/pack, gitignored; local builds only).
	var dir := OS.get_environment("DARTER_PACK")
	if dir == "":
		dir = "res://pack"
	var t0 := Time.get_ticks_usec()

	var pf := FileAccess.open(dir + "/pack.json", FileAccess.READ)
	if pf == null:
		push_error("cannot open pack.json in %s" % dir)
		get_tree().quit(1)
		return
	var pack = JSON.parse_string(pf.get_as_text())
	if typeof(pack) != TYPE_DICTIONARY or pack.get("schema") != "darter_area_pack":
		push_error("pack.json is not a darter_area_pack")
		get_tree().quit(1)
		return

	# Bundled copies live under res:// where Godot would treat a bare .obj as
	# an importable model (and our per-chunk OBJ is not an importable model),
	# so the bundled file carries a non-importable extension.
	var obj_name := "scene.packobj" if dir.begins_with("res://") else "scene.obj"
	var of := FileAccess.open(dir + "/" + obj_name, FileAccess.READ)
	if of == null:
		push_error("cannot open %s in %s" % [obj_name, dir])
		get_tree().quit(1)
		return
	var text := of.get_as_text()
	var mesh := _obj_to_mesh(text, pack)
	if mesh == null:
		get_tree().quit(1)
		return
	var load_ms := (Time.get_ticks_usec() - t0) / 1000

	# The pack block of the replay JSON: the loader's own facts, asserted by
	# the Rust test against pack.json.
	var groups_json := ""
	for fam in ["ground", "grass", "road", "hedge", "fence", "bld", "bldroof", "tree", "treec"]:
		groups_json += "%s\"%s\":%d" % [
			"" if groups_json == "" else ",", fam, int(group_counts.get(fam, 0))
		]
	print("PACK_LOAD_DONE ms=%d groups={%s}" % [load_ms, groups_json])

	_build_world()
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)

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
	cam.far = 3000.0
	# Godot's 0.05 default near wastes most of the 24-bit depth range over a
	# 3 km far plane (~12 mm precision at 100 m): coplanar pack surfaces
	# (roads 50 mm over the ground) z-fight well inside the view. The chase
	# cam never approaches anything nearer than ~4 m, so 1.0 buys 20x.
	cam.near = 1.0

	# The summary is written after the last frame (in _process); keep the
	# pack block around until then.
	set_meta("pack_json",
		"\"pack\":{\"load_ms\":%d,\"materials\":%d,\"groups\":{%s}}" % [
			load_ms, MATERIAL_ORDER.size(), groups_json
		])


func _obj_to_mesh(text: String, pack: Dictionary) -> ArrayMesh:
	# Chunks are [mat, verts, norms, indices]. Face indices are resolved AT
	# PARSE TIME against a running committed-vertex base per material, so the
	# later concat needs no remap pass. Chunk-local arrays are appened to by
	# LOCAL vars (GDScript packed arrays are COW handles: dict[key].append()
	# would copy per append, O(n^2) over ~450k appends).
	var chunks := []
	var mat_base := {}
	var cur_mat := ""
	var mat_set := false
	var cur_verts := PackedVector3Array()
	var cur_norms := PackedVector3Array()
	var cur_idx := PackedInt32Array()
	var cur_base := -1  # committed verts of this material; set on first "v"
	var nv := 0  # global 1-based vertex cursor: face indices count ALL v lines
	var chunk_v0 := 0  # nv at the current chunk's start
	var lines := text.split("\n")
	for line in lines:
		if line.length() < 2:
			continue
		match line[0]:
			"o":
				# Commit the previous chunk. Must be inline: resetting packed
				# arrays inside a helper would not propagate back (by value).
				if cur_mat != "" and cur_verts.size() > 0:
					chunks.append([cur_mat, cur_verts, cur_norms, cur_idx])
					mat_base[cur_mat] = int(mat_base.get(cur_mat, 0)) + cur_verts.size()
				cur_mat = ""
				mat_set = false
				cur_verts = PackedVector3Array()
				cur_norms = PackedVector3Array()
				cur_idx = PackedInt32Array()
				cur_base = -1
				chunk_v0 = nv
				var fam: String = line.substr(2).split("_")[0]
				group_counts[fam] = int(group_counts.get(fam, 0)) + 1
			"u":
				if line.begins_with("usemtl "):
					cur_mat = line.substr(7)
					mat_set = true
			"v":
				if not mat_set or cur_mat == "":
					push_error("vertex before usemtl (pack format violation): %s" % line)
					return null
				if cur_base < 0:
					cur_base = int(mat_base.get(cur_mat, 0))
				var p: PackedFloat64Array = line.substr(2).split_floats(" ")
				if p.size() != 3:
					push_error("bad vertex line: %s" % line)
					return null
				# ENU (x east, y north, z up) -> Godot (x, z, -y), Y up.
				cur_verts.append(Vector3(p[0], p[2], -p[1]))
				nv += 1
			"f":
				var idx: PackedFloat64Array = line.substr(2).split_floats(" ")
				if idx.size() != 3 or cur_base < 0:
					push_error("bad face line: %s" % line)
					return null
				# Face indices are FILE-GLOBAL 1-based; each chunk re-emits
				# its own verts, so g - chunk_v0 - 1 is the chunk-local slot.
				var i0 := int(idx[0]) - chunk_v0 - 1
				var i1 := int(idx[1]) - chunk_v0 - 1
				var i2 := int(idx[2]) - chunk_v0 - 1
				if i0 < 0 or i1 < 0 or i2 < 0 or i0 >= cur_verts.size() \
						or i1 >= cur_verts.size() or i2 >= cur_verts.size():
					push_error("face references outside its chunk: %s" % line)
					return null
				var a: Vector3 = cur_verts[i0]
				var b: Vector3 = cur_verts[i1]
				var c: Vector3 = cur_verts[i2]
				var n := (b - a).cross(c - a)
				cur_norms.append(n)
				cur_norms.append(n)
				cur_norms.append(n)
				# Godot front faces are clockwise; the OBJ is CCW-outward.
				# The appended index is the material-global slot.
				cur_idx.append(cur_base + i2)
				cur_idx.append(cur_base + i1)
				cur_idx.append(cur_base + i0)
	if cur_mat != "" and cur_verts.size() > 0:
		chunks.append([cur_mat, cur_verts, cur_norms, cur_idx])
		mat_base[cur_mat] = int(mat_base.get(cur_mat, 0)) + cur_verts.size()

	# Family counts vs pack.json (the consumer-side contract check).
	var known := ["ground", "grass", "road", "hedge", "fence", "bld", "bldroof", "tree", "treec"]
	for fam in group_counts:
		if not known.has(fam):
			push_error("unknown o-group family %s" % fam)
			return null
	var want := {
		"ground": 1, "grass": int(pack["counts"]["grass"]),
		"road": int(pack["counts"]["roads"]),
		"bld": int(pack["counts"]["buildings"]),
		"bldroof": int(pack["counts"]["buildings"]),
		"tree": int(pack["counts"]["trees"]), "treec": int(pack["counts"]["trees"]),
	}
	for fam in want:
		if int(group_counts.get(fam, 0)) != int(want[fam]):
			push_error("pack scene.o groups: family %s %d != %d from pack.json" % [
				fam, int(group_counts.get(fam, 0)), int(want[fam])
			])
			return null
	var strips := int(group_counts.get("hedge", 0)) + int(group_counts.get("fence", 0))
	if strips != int(pack["counts"]["strips"]):
		push_error("pack scene.o groups: hedge+fence %d != %d strips" % [
			strips, int(pack["counts"]["strips"])
		])
		return null

	# One surface per material, fixed order.
	var surf := {}
	for ch in chunks:
		if not surf.has(ch[0]):
			surf[ch[0]] = [PackedVector3Array(), PackedVector3Array(), PackedInt32Array()]
		var s = surf[ch[0]]
		s[0].append_array(ch[1])
		s[1].append_array(ch[2])
		s[2].append_array(ch[3])

	var mesh := ArrayMesh.new()
	var used := 0
	for mat in MATERIAL_ORDER:
		if not surf.has(mat):
			continue
		var s = surf[mat]
		if s[0].size() == 0:
			continue
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = s[0]
		arrays[Mesh.ARRAY_NORMAL] = s[1]
		arrays[Mesh.ARRAY_INDEX] = s[2]
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		mesh.surface_set_material(used, _material_for(mat))
		used += 1
	for mat in surf:
		if not MATERIAL_ORDER.has(mat):
			push_error("material %s not in MATERIAL_ORDER" % mat)
			return null
	return mesh


func _material_for(name: String) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = MATERIAL_COLORS.get(name, Color(0.5, 0.5, 0.5))
	m.roughness = 1.0
	m.metallic = 0.0
	return m


func _build_world() -> void:
	var preset_name := OS.get_environment("DARTER_LIGHTING")
	if preset_name == "":
		preset_name = LIGHTING_PRESETS.DEFAULT_NAME
	var preset := LIGHTING_PRESETS.get_preset(preset_name)
	if preset.is_empty():
		get_tree().quit(1)
		return
	# Dev/metering-only knob: override the preset's frozen tonemap exposure
	# while solving it against the grey-card target (165/255) or eyeballing
	# presets. Tests never set this env, so the shipped determinism path
	# always uses the frozen value.
	var env_exposure := OS.get_environment("DARTER_TONEMAP_EXPOSURE")
	if env_exposure != "":
		preset["tonemap_exposure"] = float(env_exposure)

	var env := Environment.new()
	var sky := Sky.new()
	var sky_mat := ProceduralSkyMaterial.new()
	sky.sky_material = sky_mat
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	add_child(sun)
	# The preset carries the whole LB03 lighting model mapped to Godot
	# (sun, fog beta, ambient, tonemap) — see lighting_presets.gd.
	LIGHTING_PRESETS.apply(preset, env, sky_mat, sun)


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
	# Chase cam 10 m behind along the recorded velocity, 4 m above, looking
	# at the quad.
	var prev: Dictionary = rows[maxi(idx - 1, 0)]
	var dir := quad.position - Vector3(float(prev["px"]), float(prev["pz"]), -float(prev["py"]))
	if dir.length_squared() < 1e-12:
		dir = Vector3(0, 0, 1)
	dir = dir.normalized()
	cam.position = quad.position - dir * 10.0 + Vector3(0.0, 4.0, 0.0)
	cam.look_at(quad.position, Vector3.UP)
	# Pixel gates read the full framebuffer back from the GPU — on mobile that
	# cost ~25% of throughput per frame (Adreno 830, 2376x1080) — so they run
	# every sample_every-th frame. du is recorded every frame either way, so
	# the frame-pacing data is never gated.
	var bm_val := -1.0
	var bv_val := -1.0
	var sky_val := -1.0
	var veg_val := -1.0
	var mm_val := -1.0
	if fidx % sample_every == 0:
		await RenderingServer.frame_post_draw
		var img: Image = get_viewport().get_texture().get_image()
		if img != null and not img.is_empty():
			var n := 0.0
			var s := 0.0
			var s2 := 0.0
			var sky_n := 0
			var veg_n := 0
			var mm_n := 0
			var h := img.get_height()
			var w := img.get_width()
			for y in range(0, h, 16):
				for x in range(0, w, 16):
					var c := img.get_pixel(x, y)
					var l: float = c.get_luminance()
					s += l
					s2 += l * l
					n += 1.0
					# Sky is blue-dominant; vegetation green-dominant; man-made
					# surfaces (brick, render, roofs, asphalt, concrete) are red
					# or neutral.
					if c.b > c.r * 1.15 and c.b > c.g * 1.05:
						sky_n += 1
					elif c.g > c.r * 1.12 and c.g > c.b * 1.05:
						veg_n += 1
					else:
						mm_n += 1
			var mean := s / n
			bm_val = mean
			bv_val = s2 / n - mean * mean
			sky_val = float(sky_n) / n
			veg_val = float(veg_n) / n
			mm_val = float(mm_n) / n
	# Render-info counters are cheap reads (last completed frame) and proved
	# decisive for pass/culling questions, so every entry carries them.
	var pr_val := int(Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME))
	var dr_val := int(Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME))
	entries.append(
		"{\"f\":%d,\"t\":%.3f,\"px\":%.6f,\"py\":%.6f,\"pz\":%.6f,\"bm\":%.9f,\"bv\":%.9f,\"sky\":%.4f,\"veg\":%.4f,\"mm\":%.4f,\"du\":%d,\"pr\":%d,\"dr\":%d}" % [
			fidx, float(r["t"]), px, py, pz, bm_val, bv_val, sky_val, veg_val, mm_val, dt_wall, pr_val, dr_val
		]
	)
	if entries.size() >= total_frames and not done:
		done = true
		# App user data dir is the only writable path on Android.
		var out := OS.get_environment("DARTER_REPLAY_OUT")
		if out == "":
			out = OS.get_user_data_dir() + "/replay.json"
		var g := FileAccess.open(out, FileAccess.WRITE)
		if g == null:
			push_error("cannot write %s" % out)
			get_tree().quit(1)
			return
		var samples := ",".join(PackedStringArray(entries))
		g.store_string(
			"{\"movie_fps\":%d,\"frames\":%d,%s,\"samples\":[%s]}" % [
				int(MOVIE_FPS), total_frames, get_meta("pack_json", "\"pack\":{}"), samples
			]
		)
		g.close()
		print("REPLAY_DONE frames=%d hz=%.2f" % [entries.size(), hz])
		get_tree().quit()