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
	"grass", "asphalt", "concrete", "paint_white", "hedge", "fence",
	"brick_red", "brick_red_plain", "brick_buff", "brick_buff_plain",
	"render_white", "render_white_plain",
	"tile_brown", "slate", "bark",
	"foliage_a", "foliage_b", "foliage_c", "foliage_d",
]
const MATERIAL_COLORS := {
	"grass": Color(0.42, 0.52, 0.30),
	"asphalt": Color(0.16, 0.16, 0.17),
	"concrete": Color(0.60, 0.60, 0.58),
	# S3b lane paint: slightly warm off-white (road-marking paint reads
	# ~0.8 albedo); neutral-dark enough for the man-made class.
	"paint_white": Color(0.78, 0.77, 0.74),
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

# S2 shader instances. Facades (walls) and roofs get ShaderMaterials built on
# the S2 shader sources; everything else stays a StandardMaterial3D with the
# same MATERIAL_COLORS albedo. The per-building seed arrives through the
# mesh's vertex COLOR channel, not the material.
const SHADER_FACADE := preload("shaders/facade.gdshader")
const SHADER_ROOF := preload("shaders/roof.gdshader")
const FACADE_MATS := [
	"brick_red", "brick_red_plain", "brick_buff", "brick_buff_plain",
	"render_white", "render_white_plain",
]
const ROOF_MATS := ["tile_brown", "slate"]
# S3: the three world-horizontal surface materials get the CC0 photo textures.
const SHADER_GROUND := preload("shaders/ground.gdshader")
const GROUND_TEX := {
	"grass": preload("res://textures/grass.jpg"),
	"asphalt": preload("res://textures/asphalt.jpg"),
	"concrete": preload("res://textures/concrete.jpg"),
}
const GROUND_MATS := ["grass", "asphalt", "concrete"]
# Metres per texture repeat per material (asphalt grains read finer than lawn).
const GROUND_TILE := {"grass": 10.0, "asphalt": 6.0, "concrete": 6.0}

# S4: crowns/hedges tint per world cell; bark/fence get per-pixel noise.
const SHADER_FOLIAGE := preload("shaders/foliage.gdshader")
const FOLIAGE_MATS := ["hedge", "foliage_a", "foliage_b", "foliage_c", "foliage_d"]
const SHADER_SURFACE_NOISE := preload("shaders/surface_noise.gdshader")
const NOISE_MATS := ["bark", "fence"]

var rows: Array = []
var hz := 250.0
var total_frames := 0
var entries: Array = []
var last_usec := 0
var group_counts := {}
# The applied lighting preset (S1), read by the shader material factory for
# the facade shader's lit_fraction; set by _build_world (which runs before
# the pack loader).
var lighting: Dictionary = {}
var _shader_cache := {}  # material name -> Material

var quad: MeshInstance3D
var cam: Camera3D
# Set when the last frame has been written (see replay.gd: Movie Maker mode
# runs one extra _process after get_tree().quit()).
var done := false

# Pixel-class gate interval. 1 samples every frame (the desktop T7 contract);
# the Android replay carries no env and pays the readback tax, so it samples
# every 4th frame. DARTER_SAMPLE_EVERY overrides both (resolved in _ready).
var sample_every := 1

# Track (M1 rung): an optional darter_track JSON rendering gate rings + HUD,
# mirrored against src/bin/sim_run/track.rs's crossing math. All positions
# and normals are held as separate f64 scalars — Vector3 is f32 and the T7
# test gates the mirror against Rust to |dt| <= 0.01 s. Empty until
# _load_track() accepts a file; every use is guarded by track_on.
var track_on := false
var track_name := ""
var track_cps_n := 0
var track_is_loop := false
var track_x: PackedFloat64Array = PackedFloat64Array()
var track_y: PackedFloat64Array = PackedFloat64Array()
var track_z: PackedFloat64Array = PackedFloat64Array()
var track_nx: PackedFloat64Array = PackedFloat64Array()
var track_ny: PackedFloat64Array = PackedFloat64Array()
var track_nz: PackedFloat64Array = PackedFloat64Array()
var track_r2: PackedFloat64Array = PackedFloat64Array()
var track_kinds: Array = []  # "gate" | "start", per checkpoint
var track_events: Array = []  # [cp_index, t_cross] pairs, file order until sorted at done
var track_lap := 0            # live start-gate crossing count (HUD only)
var track_last_idx := -1      # last walked record row (mirror walk source)
var track_next_gate := -1     # currently pulsed gate node, -1 = none
var gate_nodes: Array = []
var gate_mats: Array = []
var track_hud: Label

# Lighting preset (S1 art pass). Selected by env DARTER_LIGHTING; the tests
# never set it, so the default pins the canon run.
const LIGHTING_PRESETS := preload("lighting_presets.gd")


func _ready() -> void:
	# World first: the shader materials built by the pack loader read the
	# lighting preset's windows_lit (S2 facades), so the preset must exist
	# before _obj_to_mesh runs.
	_build_world()
	sample_every = 4 if OS.has_feature("android") else 1
	sample_every = 4 if OS.has_feature("android") else 1
	var env_se := OS.get_environment("DARTER_SAMPLE_EVERY")
	if env_se != "":
		sample_every = maxi(1, int(env_se))
	_load_pack_and_build_scene()
	_load_track()
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

	# Relief (M1): grid z range from pack.json's elevation object, echoed
	# into the summary so the Rust test asserts the loader is driven by the
	# same hills pack.json promises. Flat packs carry {"model": "flat"} — no
	# grid, no relief member; the loader stays replay-ready for flat packs.
	var relief_json := ""
	var elev = pack.get("elevation")
	if elev is Dictionary and elev.get("model", "flat") != "flat":
		relief_json = "\"relief\":{\"z_min\":%.1f,\"z_max\":%.1f}," % [
			float(elev["z_min"]), float(elev["z_max"])
		]

	# The pack block of the replay JSON: the loader's own facts, asserted by
	# the Rust test against pack.json.
	var groups_json := ""
	for fam in ["ground", "grass", "road", "hedge", "fence", "bld", "bldroof", "tree", "treec", "dash"]:
		groups_json += "%s\"%s\":%d" % [
			"" if groups_json == "" else ",", fam, int(group_counts.get(fam, 0))
		]
	print("PACK_LOAD_DONE ms=%d groups={%s}" % [load_ms, groups_json])

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
	# 3 km far plane (~12 mm precision at 100 m): the 50 mm road-over-ground
	# offsets z-fight well inside the view (on a glo30 pack draped along the
	# terrain slope, offsets unchanged). With relief, ground can also sit
	# nearer to the cam than the flat pack's ~4 m margin when the quad flies
	# low over a rising slope — cam-through-terrain there is an eyeball
	# question, not a depth-precision one; near stays 1.0 and far stays 3000.
	cam.near = 1.0

	# The summary is written after the last frame (in _process); keep the
	# pack block around until then.
	set_meta("pack_json",
		"\"pack\":{\"load_ms\":%d,\"materials\":%d,%s\"groups\":{%s}}" % [
			load_ms, MATERIAL_ORDER.size(), relief_json, groups_json
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
	var cur_cols := PackedFloat64Array()
	var cur_idx := PackedInt32Array()
	var cur_base := -1  # committed verts of this material; set on first "v"
	var cur_seed := 0.0  # per-chunk (per-building) seed; set on "o"
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
					chunks.append([cur_mat, cur_verts, cur_norms, cur_cols, cur_idx])
					mat_base[cur_mat] = int(mat_base.get(cur_mat, 0)) + cur_verts.size()
				cur_mat = ""
				mat_set = false
				cur_verts = PackedVector3Array()
				cur_norms = PackedVector3Array()
				cur_cols = PackedFloat64Array()
				cur_idx = PackedInt32Array()
				cur_base = -1
				chunk_v0 = nv
				var fam: String = line.substr(2).split("_")[0]
				group_counts[fam] = int(group_counts.get(fam, 0)) + 1
				# Per-building seed (S2 shaders): one deterministic number
				# from the o-group id, constant over the chunk's vertices. It
				# travels as a vertex COLOR channel because shaders see
				# varyings only between vertices; identical values on all a
				# chunk's vertices keep it interpolation-stable.
				var parts: PackedStringArray = line.substr(2).split("_")
				var id_n := float(parts[1]) if parts.size() > 1 else 0.0
				cur_seed = fposmod(sin(id_n * 12.9898) * 43758.5453, 1.0)
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
				cur_cols.append(cur_seed)
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
		chunks.append([cur_mat, cur_verts, cur_norms, cur_cols, cur_idx])
		mat_base[cur_mat] = int(mat_base.get(cur_mat, 0)) + cur_verts.size()

	# Family counts vs pack.json (the consumer-side contract check).
	var known := ["ground", "grass", "road", "hedge", "fence", "bld", "bldroof", "tree", "treec", "dash"]
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
		"dash": int(pack["counts"]["dashes"]),
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
			surf[ch[0]] = [PackedVector3Array(), PackedVector3Array(), PackedFloat64Array(), PackedInt32Array()]
		var s = surf[ch[0]]
		s[0].append_array(ch[1])
		s[1].append_array(ch[2])
		s[2].append_array(ch[3])
		s[3].append_array(ch[4])

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
		# Per-building seed as float grey (S2 shaders read COLOR.r).
		var cols := PackedColorArray()
		cols.resize(s[0].size())
		for i in s[0].size():
			cols[i] = Color(s[2][i], s[2][i], s[2][i], 1.0)
		arrays[Mesh.ARRAY_COLOR] = cols
		arrays[Mesh.ARRAY_INDEX] = s[3]
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		mesh.surface_set_material(used, _shader_for(mat))
		used += 1
	for mat in surf:
		if not MATERIAL_ORDER.has(mat):
			push_error("material %s not in MATERIAL_ORDER" % mat)
			return null
	return mesh


func _shader_for(name: String) -> Material:
	# Cached per material name: one surface per material but the mesh's
	# surfaces each hold their own copy anyway; the cache just keeps wall/roof
	# variants consistent and the factory cheap to re-call.
	if _shader_cache.has(name):
		return _shader_cache[name]
	var col: Color = MATERIAL_COLORS.get(name, Color(0.5, 0.5, 0.5))
	var m: Material
	if name in FACADE_MATS:
		var fm := ShaderMaterial.new()
		fm.shader = SHADER_FACADE
		fm.set_shader_parameter("albedo", col)
		fm.set_shader_parameter("plain_mode", name.ends_with("_plain"))
		fm.set_shader_parameter("lit_fraction", float(lighting.get("windows_lit", 0.0)))
		m = fm
	elif name in ROOF_MATS:
		var rm := ShaderMaterial.new()
		rm.shader = SHADER_ROOF
		rm.set_shader_parameter("albedo", col)
		m = rm
	elif name in GROUND_MATS:
		var gm := ShaderMaterial.new()
		gm.shader = SHADER_GROUND
		gm.set_shader_parameter("albedo", col)
		gm.set_shader_parameter("tex", GROUND_TEX[name])
		gm.set_shader_parameter("tile_size", float(GROUND_TILE[name]))
		m = gm
	elif name in FOLIAGE_MATS:
		var fom := ShaderMaterial.new()
		fom.shader = SHADER_FOLIAGE
		fom.set_shader_parameter("albedo", col)
		m = fom
	elif name in NOISE_MATS:
		var nm := ShaderMaterial.new()
		nm.shader = SHADER_SURFACE_NOISE
		nm.set_shader_parameter("albedo", col)
		m = nm
	else:
		var sm := StandardMaterial3D.new()
		sm.albedo_color = col
		sm.roughness = 1.0
		sm.metallic = 0.0
		m = sm
	_shader_cache[name] = m
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
	lighting = preset
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


# Track precedence: DARTER_TRACK env when set; else the APK-bundled demo
# (res://track/demo_track.json) only on Android — the device plays the demo
# with no env plumbing; else disabled (desktop with no env keeps today's
# base path and byte-identical replay). A missing env-pointed file is a hard
# quit; a missing Android fallback is a soft skip (print, no track features).
# Validation mirrors parse() in src/bin/sim_run/track.rs — the renderer must
# accept exactly what the Rust CLI accepts, with hard errors too.
func _load_track() -> void:
	var env_path := OS.get_environment("DARTER_TRACK")
	var path := env_path
	if path == "":
		if not OS.has_feature("android"):
			return
		path = "res://track/demo_track.json"
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		if env_path == "" and OS.has_feature("android"):
			print("TRACK_SKIPPED missing %s" % path)
			return
		push_error("cannot open track %s" % path)
		get_tree().quit(1)
		return
	var j = JSON.parse_string(f.get_as_text())
	if typeof(j) != TYPE_DICTIONARY:
		push_error("track %s: not a JSON object" % path)
		get_tree().quit(1)
		return
	_track_validate(j)
	_build_gates()
	_build_hud()
	# Next expected gate pulses from the start (index 0).
	gate_mats[0].emission_energy_multiplier = 3.0
	track_next_gate = 0
	track_on = true
	print("TRACK_LOADED name=%s cps=%d loop=%s" % [track_name, track_cps_n, track_is_loop])


func _track_fail(msg: String) -> void:
	push_error("track: %s" % msg)
	get_tree().quit(1)


func _track_is_num(v) -> bool:
	return typeof(v) == TYPE_FLOAT and is_finite(v)


func _track_exact_keys(d: Dictionary, want: Array, what: String) -> void:
	for key in d:
		if not (key in want):
			_track_fail("%s has unknown key \"%s\" (allowed: %s)" % [what, key, ", ".join(want)])


func _track_validate(j: Dictionary) -> void:
	var top := ["schema", "version", "name", "spawn", "checkpoints"]
	_track_exact_keys(j, top, "track")
	if j.get("schema") != "darter_track":
		_track_fail('schema must be "darter_track"')
	if j.get("version") != 1.0:
		_track_fail("version must be 1")
	var name_v = j.get("name")
	if typeof(name_v) != TYPE_STRING or name_v == "":
		_track_fail("name must be a non-empty string")
	track_name = name_v
	if j.has("spawn"):
		var sp = j.get("spawn")
		if typeof(sp) != TYPE_DICTIONARY:
			_track_fail("spawn must be an object")
		_track_exact_keys(sp, ["x", "y", "z"], "spawn")
		for axis in ["x", "y", "z"]:
			if not _track_is_num(sp.get(axis)):
				_track_fail("spawn %s must be a finite number" % axis)
		# Advisory metadata only (mirrors the CLI): never read for anything.
	var cps = j.get("checkpoints")
	if typeof(cps) != TYPE_ARRAY or cps.size() < 2:
		_track_fail("checkpoints must be an array of at least 2")
	track_cps_n = cps.size()
	track_x.resize(track_cps_n)
	track_y.resize(track_cps_n)
	track_z.resize(track_cps_n)
	track_r2.resize(track_cps_n)
	track_kinds.resize(track_cps_n)
	var n_start := 0
	var first_start := -1
	for i in range(track_cps_n):
		var cp = cps[i]
		if typeof(cp) != TYPE_DICTIONARY:
			_track_fail("checkpoint %d must be an object" % i)
		_track_exact_keys(cp, ["kind", "x", "y", "z", "radius_m"], "checkpoint %d" % i)
		var kind = cp.get("kind")
		if kind != "gate" and kind != "start":
			_track_fail('checkpoint %d: kind must be "gate" or "start"' % i)
		if kind == "start":
			n_start += 1
			if first_start < 0:
				first_start = i
		for kv in [["x", 0], ["y", 1], ["z", 2]]:
			if not _track_is_num(cp.get(kv[0])):
				_track_fail("checkpoint %d: %s must be a finite number" % [i, kv[0]])
		var r = cp.get("radius_m")
		if not _track_is_num(r) or float(r) <= 0.0 or float(r) > 1000.0:
			_track_fail("checkpoint %d: radius_m must be in (0, 1000]" % i)
		track_x[i] = float(cp["x"])
		track_y[i] = float(cp["y"])
		track_z[i] = float(cp["z"])
		track_r2[i] = float(r) * float(r)
		track_kinds[i] = kind
	if n_start > 1:
		_track_fail("at most one start checkpoint")
	if n_start == 1 and first_start != 0:
		_track_fail("the start checkpoint must be the first entry")
	track_is_loop = n_start == 1
	if track_is_loop and track_cps_n < 3:
		_track_fail("a loop needs at least 3 checkpoints")
	# Distinctness: adjacent checkpoints (plus last-vs-first on a loop) may
	# not coincide, and every neighbour span may not be degenerate — same
	# rules and thresholds as the Rust parse.
	var n := track_cps_n
	for i in range(n):
		var nxt: int = (i + 1) % n if track_is_loop else mini(i + 1, n - 1)
		var prv: int = (i + n - 1) % n if track_is_loop else maxi(i - 1, 0)
		var dx := track_x[nxt] - track_x[prv]
		var dy := track_y[nxt] - track_y[prv]
		var dz := track_z[nxt] - track_z[prv]
		var span := sqrt(dx * dx + dy * dy + dz * dz)
		if span < 1e-6:
			_track_fail("checkpoint %d has a degenerate normal (neighbours coincide)" % i)
		# Adjacent positions distinct (loop adds the last-vs-first pair).
		var others: Array = [i + 1] if i < n - 1 else []
		if track_is_loop and i == n - 1:
			others.append(0)
		var dmin := INF
		var bad_j := -1
		for j2 in others:
			var ax := track_x[j2] - track_x[i]
			var ay := track_y[j2] - track_y[i]
			var az := track_z[j2] - track_z[i]
			var dist := sqrt(ax * ax + ay * ay + az * az)
			if dist < dmin:
				dmin = dist
				bad_j = j2
		if dmin < 1e-3:
			_track_fail("checkpoints %d and %d coincide" % [i, bad_j])
	# Unit plane normals, same neighbour rule as normals() in track.rs.
	track_nx.resize(track_cps_n)
	track_ny.resize(track_cps_n)
	track_nz.resize(track_cps_n)
	for i in range(track_cps_n):
		var nxt: int = (i + 1) % n if track_is_loop else mini(i + 1, n - 1)
		var prv: int = (i + n - 1) % n if track_is_loop else maxi(i - 1, 0)
		var dx := track_x[nxt] - track_x[prv]
		var dy := track_y[nxt] - track_y[prv]
		var dz := track_z[nxt] - track_z[prv]
		var l := sqrt(dx * dx + dy * dy + dz * dz)
		track_nx[i] = dx / l
		track_ny[i] = dy / l
		track_nz[i] = dz / l


# Gate rings as children of the world root: TorusMesh hole axis along the
# checkpoint normal (ENU -> Godot: (x, z, -y)); centreline radius is the
# checkpoint radius, ring thickness 0.30 m. Start ring orange, gates cyan;
# the next expected gate pulses brighter. Demo normals are near-horizontal,
# so the near-vertical fallback basis is a rarely-hit guard (eyeballed only).
func _build_gates() -> void:
	for i in range(track_cps_n):
		var r := float(sqrt(track_r2[i]))
		var torus := TorusMesh.new()
		torus.inner_radius = maxf(r - 0.15, 0.01)
		torus.outer_radius = r + 0.15
		var start_kind: bool = track_kinds[i] == "start"
		var col := Color(1.0, 0.55, 0.1) if start_kind else Color(0.1, 0.85, 1.0)
		var mat := StandardMaterial3D.new()
		mat.albedo_color = col
		mat.emission_enabled = true
		mat.emission = col
		mat.emission_energy_multiplier = 1.0
		var mi := MeshInstance3D.new()
		mi.mesh = torus
		mi.material_override = mat
		# ENU (x, y, z) -> Godot (X, Y, Z) = (x, z, -y), same rule as the mesh
		# verts and the quad.
		mi.position = Vector3(track_x[i], track_z[i], -track_y[i])
		var dir := Vector3(track_nx[i], track_nz[i], -track_ny[i])
		# Near-vertical normal fallback (the hole axis cannot use UP): stand
		# the ring on its edge via X instead. Rare guard; eyeballed only.
		if absf(dir.dot(Vector3.UP)) > 0.999:
			mi.basis = Basis(Quaternion(Vector3.RIGHT, dir))
		else:
			mi.basis = Basis(Quaternion(Vector3.UP, dir))
		add_child(mi)
		gate_nodes.append(mi)
		gate_mats.append(mat)


func _build_hud() -> void:
	var cl := CanvasLayer.new()
	add_child(cl)
	track_hud = Label.new()
	cl.add_child(track_hud)
	track_hud.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	track_hud.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	track_hud.offset_top = 12.0
	track_hud.add_theme_font_size_override("font_size", 44)
	# Sky at the HUD strip is near-white; black fill + white outline keeps
	# the text readable in every frame.
	track_hud.add_theme_color_override("font_color", Color.BLACK)
	track_hud.add_theme_color_override("font_outline_color", Color.WHITE)
	track_hud.add_theme_constant_override("outline_size", 8)
	track_hud.text = "Gate 1/%d" % track_cps_n


func _track_event_fired(i: int, t: float) -> void:
	track_events.append([i, t])
	if track_kinds[i] == "start":
		track_lap += 1
	var k := track_events.size()
	if k <= track_cps_n:
		if track_next_gate >= 0:
			var old: StandardMaterial3D = gate_mats[track_next_gate]
			old.emission_energy_multiplier = 1.0
		track_next_gate = k if k < track_cps_n else -1
		if track_next_gate >= 0:
			var nxt: StandardMaterial3D = gate_mats[track_next_gate]
			nxt.emission_energy_multiplier = 3.0
	if track_hud != null:
		if k >= track_cps_n:
			track_hud.text = "Done  %.2f s" % t
		else:
			var lap_txt := ("Lap %d  " % track_lap) if (track_is_loop and track_lap > 0) else ""
			track_hud.text = "%sGate %d/%d  %.2f s" % [lap_txt, k + 1, track_cps_n, t]


func _track_esc(s: String) -> String:
	# JSON string escaping: backslash first, then the quote.
	return s.replace("\\", "\\\\").replace("\"", "\\\"")


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
	if track_on:
		# Mirror of track.rs compute_events at display cadence: every record
		# row pair is walked exactly once (track_last_idx guards double
		# counting), scalar f64s throughout because Vector3 is f32 and the
		# comparison must match the Rust CLI's f64 math.
		for k in range(maxi(track_last_idx + 1, 1), idx + 1):
			var ak: Dictionary = rows[k - 1]
			var bk: Dictionary = rows[k]
			var tax: float = float(ak["px"])
			var tay: float = float(ak["py"])
			var taz: float = float(ak["pz"])
			var tbx: float = float(bk["px"])
			var tby: float = float(bk["py"])
			var tbz: float = float(bk["pz"])
			for i in range(track_cps_n):
				var d_a: float = (tax - track_x[i]) * track_nx[i] + (tay - track_y[i]) * track_ny[i] + (taz - track_z[i]) * track_nz[i]
				var d_b: float = (tbx - track_x[i]) * track_nx[i] + (tby - track_y[i]) * track_ny[i] + (tbz - track_z[i]) * track_nz[i]
				if d_b > 0.0 and d_a <= 0.0:
					var sfrac: float = d_a / (d_a - d_b)
					var hx: float = tax + (tbx - tax) * sfrac - track_x[i]
					var hy: float = tay + (tby - tay) * sfrac - track_y[i]
					var hz: float = taz + (tbz - taz) * sfrac - track_z[i]
					if hx * hx + hy * hy + hz * hz <= track_r2[i]:
						_track_event_fired(i, float(ak["t"]) + (float(bk["t"]) - float(ak["t"])) * sfrac)
		track_last_idx = idx
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
		# Track block written by the renderer's own recompute (decision 7).
		# Empty string when track is off: the emit stays byte-identical to
		# the pre-track format. Events sorted by (t, i) like the Rust CLI;
		# lap splits are consecutive start-crossing times, [] until a start
		# fires, null on an open track.
		var track_json := ""
		if track_on:
			track_events.sort_custom(func(a, b):
				return a[1] < b[1] or (a[1] == b[1] and a[0] < b[0]))
			var ev_parts: PackedStringArray = []
			for e in track_events:
				ev_parts.append("[%d,%.3f]" % [e[0], e[1]])
			var sp_str := "null"
			if track_is_loop:
				var start_ts: Array = []
				for e in track_events:
					if track_kinds[e[0]] == "start":
						start_ts.append(e[1])
				var sp_parts: PackedStringArray = []
				for w in range(1, start_ts.size()):
					sp_parts.append("%.3f" % (start_ts[w] - start_ts[w - 1]))
				sp_str = "[%s]" % ",".join(sp_parts)
			track_json = ",\"track\":{\"schema\":\"darter_track\",\"version\":1,\"name\":\"%s\",\"checkpoints\":%d,\"loop\":%s,\"events\":[%s],\"lap_splits\":%s}" % [
				_track_esc(track_name), track_cps_n, "true" if track_is_loop else "false",
				",".join(ev_parts), sp_str
			]
		g.store_string(
			"{\"movie_fps\":%d,\"frames\":%d,%s%s,\"samples\":[%s]}" % [
				int(MOVIE_FPS), total_frames, get_meta("pack_json", "\"pack\":{}"), track_json, samples
			]
		)
		g.close()
		print("REPLAY_DONE frames=%d hz=%.2f" % [entries.size(), hz])
		get_tree().quit()