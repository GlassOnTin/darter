@tool
extends EditorPlugin
# Export shim for the hand-rolled refresh-vote Godot Android plugin (v2).
# There is no runtime GDScript API: the AAR's manifest meta-data
# org.godotengine.plugin.v2.refresh_vote -> org.darter.fro.RefreshVotePlugin
# is what the merged app manifest registers with the engine at startup.
# Plugin scripts must extend EditorPlugin (an EditorExportPlugin alone is
# rejected by the addon loader); the export plugin registers as an inner class.

var export_plugin: RefreshVoteExportPlugin


func _enter_tree() -> void:
	export_plugin = RefreshVoteExportPlugin.new()
	add_export_plugin(export_plugin)


func _exit_tree() -> void:
	remove_export_plugin(export_plugin)
	export_plugin = null


class RefreshVoteExportPlugin extends EditorExportPlugin:
	func _get_name() -> String:
		return "refresh_vote"

	func _supports_platform(platform: EditorExportPlatform) -> bool:
		return platform is EditorExportPlatformAndroid

	# AAR paths are relative to the 'addons' directory (see the plugin docs).
	func _get_android_libraries(_platform: EditorExportPlatform, _debug: bool) -> PackedStringArray:
		return PackedStringArray(["refresh_vote/refresh_vote.aar"])