#!/bin/sh
# Rebuild refresh_vote.aar from RefreshVotePlugin.java (hand-rolled Godot
# Android plugin v2; no gradle in the plugin itself - the app's gradle build
# just merges the AAR, see export_presets.cfg).
#
# Requires:
#   * a JDK's javac/jar and unzip/zip on PATH
#   * an Android SDK platform jar (framework stubs: Display.Mode,
#     Surface.setFrameRate, WindowManager.LayoutParams) - android-36 here
#   * the Godot engine's Java classes, shipped as godot-lib.template_*.aar in
#     the android build template's libs/ (--install-android-build-template)
#
# Env overrides:
#   GODOT_LIB_AAR  path to godot-lib.template_*.aar (default: the android
#                  build template's libs/release copy, then debug)
#   PLATFORM_JAR   path to android.jar (default: $ANDROID_HOME/platforms/
#                  android-36/android.jar, else ~/Android/Sdk/...)
#
# Output: refresh_vote.aar next to this script. The built AAR is committed, so
# this script only needs a rerun when RefreshVotePlugin.java changes.

set -eu
cd "$(dirname "$0")"

if [ -z "${GODOT_LIB_AAR:-}" ] || [ ! -f "$GODOT_LIB_AAR" ]; then
	# The template ships the engine classes under the project's android build
	# tree (created by --install-android-build-template; on this workstation
	# the template lives in the throwaway export copy, see export_presets.cfg).
	for candidate in ../android/build/libs/release/godot-lib.template_release.aar \
		../android/build/libs/debug/godot-lib.template_debug.aar; do
		if [ -f "$candidate" ]; then
			GODOT_LIB_AAR="$candidate"
			break
		fi
	done
fi
if [ -z "${GODOT_LIB_AAR:-}" ] || [ ! -f "$GODOT_LIB_AAR" ]; then
	echo "cannot find godot-lib.template_*.aar (set GODOT_LIB_AAR)" >&2
	exit 1
fi
if [ -n "${ANDROID_HOME:-}" ] && [ ! -f "${PLATFORM_JAR:-/dev/null}" ]; then
	PLATFORM_JAR="$ANDROID_HOME/platforms/android-36/android.jar"
fi
PLATFORM_JAR=${PLATFORM_JAR:-$HOME/Android/Sdk/platforms/android-36/android.jar}
if [ ! -f "$PLATFORM_JAR" ]; then
	echo "cannot find android platform jar (set PLATFORM_JAR)" >&2
	exit 1
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
unzip -o -q "$GODOT_LIB_AAR" classes.jar -d "$WORK/godotlib"
unzip -o -q "$WORK/godotlib/classes.jar" -d "$WORK/godotlib/classes"

mkdir -p "$WORK/out"
javac --release 11 \
	-cp "$WORK/godotlib/classes:$PLATFORM_JAR" \
	-d "$WORK/out" \
	RefreshVotePlugin.java

mkdir -p "$WORK/aar"
jar cf "$WORK/aar/classes.jar" -C "$WORK/out" .
cp AndroidManifest.xml "$WORK/aar/AndroidManifest.xml"
(cd "$WORK/aar" && zip -q -X ../refresh_vote.aar AndroidManifest.xml classes.jar)
cp "$WORK/refresh_vote.aar" refresh_vote.aar
echo "built refresh_vote.aar ($(wc -c < refresh_vote.aar) bytes) against: $(basename "$(dirname "$(dirname "$GODOT_LIB_AAR")")")"