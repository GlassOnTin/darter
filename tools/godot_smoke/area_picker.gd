extends Control
# S6 area picker: the export main scene. Lists the demo areas actually
# bundled in res://areas/index.json, shows the OSM/DEM attribution
# (VISION.md requires it visible, verbatim), and hands the chosen area to
# pack_replay.tscn through DARTER_AREA. A missing or malformed index is a
# hard quit: a bundle that cannot even state what it ships fails loudly
# instead of showing an empty menu.

const AREA_INDEX := "res://areas/index.json"
const REPLAY_SCENE := "res://pack_replay.tscn"


func _ready() -> void:
	var index := _read_index()
	var areas = index.get("areas", [])
	var attribution = index.get("attribution", "")
	if typeof(areas) != TYPE_ARRAY or typeof(attribution) != TYPE_STRING:
		push_error("area picker: %s missing or malformed (needs an areas array and an attribution string)" % AREA_INDEX)
		get_tree().quit(1)
		return
	var rows: Array = []
	for row in areas:
		if typeof(row) == TYPE_DICTIONARY and typeof(row.get("name")) == TYPE_STRING:
			rows.append(row)
	if rows.is_empty():
		push_error("area picker: %s carries no area names" % AREA_INDEX)
		get_tree().quit(1)
		return
	print("PICKER_READY areas=%s attribution=%s" % [
		",".join(rows.map(func(r): return r["name"])), attribution
	])
	_build_ui(rows, attribution)


func _read_index() -> Dictionary:
	# FileAccess (not DirAccess) so the export PCK reads it; {} on failure.
	var f := FileAccess.open(AREA_INDEX, FileAccess.READ)
	if f == null:
		return {}
	var parsed = JSON.parse_string(f.get_as_text())
	return parsed if typeof(parsed) == TYPE_DICTIONARY else {}


func _build_ui(rows: Array, attribution: String) -> void:
	var bg := ColorRect.new()
	bg.color = Color(0.05, 0.06, 0.08)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(bg)

	var title := Label.new()
	title.text = "Fly a demo area"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	title.add_theme_font_size_override("font_size", 44)
	title.set_anchors_preset(Control.PRESET_TOP_WIDE)
	title.offset_bottom = 64.0
	add_child(title)

	# Centered band at 80% height: four buttons of ~54 px apiece at the
	# 640x360 test viewport (separation 24), proportionally taller on the
	# device's native-resolution window.
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 24)
	box.anchor_left = 0.0
	box.anchor_right = 1.0
	box.anchor_top = 0.10
	box.anchor_bottom = 0.90
	add_child(box)

	for row in rows:
		var button := Button.new()
		button.text = str(row["name"])
		button.add_theme_font_size_override("font_size", 44)
		button.size_flags_horizontal = Control.SIZE_FILL
		button.size_flags_vertical = Control.SIZE_EXPAND_FILL
		button.pressed.connect(_open_area.bind(row["name"]))
		box.add_child(button)

	var footer := Label.new()
	footer.text = attribution
	footer.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	footer.vertical_alignment = VERTICAL_ALIGNMENT_BOTTOM
	footer.add_theme_font_size_override("font_size", 24)
	footer.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	footer.offset_top = -48.0
	add_child(footer)


func _open_area(area_name: String) -> void:
	OS.set_environment("DARTER_AREA", area_name)
	get_tree().change_scene_to_file(REPLAY_SCENE)