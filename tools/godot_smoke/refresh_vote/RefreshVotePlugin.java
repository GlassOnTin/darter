package org.darter.fro;

import android.app.Activity;
import android.os.Build;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;

import org.godotengine.godot.Godot;
import org.godotengine.godot.gl.GLSurfaceView;
import org.godotengine.godot.plugin.GodotPlugin;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Godot Android plugin v2 (hand-rolled; no runtime GDScript surface).
 *
 * Godot 4.7.2's Android DisplayServer never votes for a refresh rate, so every
 * Godot app lands in the panel's "normal" frame-rate category and vsync paces
 * the render loop there (60 Hz on the CPH2655 panel, which supports
 * 120/90/60). This plugin votes from Java at the two points the engine gives
 * plugins:
 *
 *  1. window vote  - WindowManager.LayoutParams.preferredDisplayModeId +
 *     preferredRefreshRate, set on the lowest supported mode >= 89 Hz whose
 *     physical size matches the currently active mode, re-applied
 *     idempotently on every resume. Always pinned (not only when the panel
 *     reads low) because panel mode reads are racy across app transitions on
 *     ColorOS: measured, create-time Display.getMode() reported the launcher's
 *     90 Hz while the system was still holding it, then the panel settled to
 *     60 Hz for this app and nothing lifted it again. The pin is scoped to
 *     this app's window - WMS applies it only while the window is on top and
 *     releases it otherwise - so the user's own panel setting is untouched; a
 *     surface vote alone (Surface.setFrameRate) registers on the SF layer but
 *     ColorOS's mode decision did not follow it.
 *  2. surface vote - Surface.setFrameRate(value, CHANGE_FRAME_RATE_ALWAYS) on
 *     the GL render surface, with the active or chosen high-tier rate.
 *
 * 90 Hz is the phone's own high tier: dumpsys display on this panel reports
 * FrameRateCategoryRate {normal=60.0, high=90.0}, and 120 Hz is the top tier
 * (battery-costly). Panels without a >= 89 Hz mode are untouched.
 * Every path is guarded: failure to vote degrades to stock Godot behaviour.
 */
public class RefreshVotePlugin extends GodotPlugin {

	private static final String TAG = "GodotRefreshVote";
	/** Lowest window refresh rate we will vote for (inclusive threshold). */
	private static final float MIN_VOTE_HZ = 89.0f;
	/** Chosen (or already-active) high-tier rate, cached for the surface vote.
	 *  0 = no high tier found on this panel. */
	private volatile float mTargetRate = 0f;

	public RefreshVotePlugin(Godot godot) {
		super(godot);
	}

	@Override
	public String getPluginName() {
		return "refresh_vote";
	}

	@Override
	public View onMainCreate(Activity activity) {
		voteWindow(activity);
		// Delayed polls show whether the panel honours the vote once the
		// window is attached (the switch can be deferred or overridden).
		pollLater(3000);
		pollLater(8000);
		return super.onMainCreate(activity);
	}

	@Override
	public void onMainResume() {
		report("resume");
		voteWindow(getActivity());
		voteSurface();
		super.onMainResume();
	}

	@Override
	public void onMainPause() {
		report("pause");
		super.onMainPause();
	}

	@Override
	public void onGLSurfaceCreated(GL10 gl, EGLConfig config) {
		report("gl surface created");
		voteSurface();
		super.onGLSurfaceCreated(gl, config);
	}

	/** Window-level vote: pin preferredDisplayModeId + preferredRefreshRate. */
	private void voteWindow(Activity activity) {
		if (activity == null) {
			return;
		}
		try {
			Display display = ((WindowManager) activity.getSystemService(Activity.WINDOW_SERVICE))
					.getDefaultDisplay();
			Display.Mode current = display.getMode();
			if (current == null) {
				report("window: no current mode");
				return;
			}
			String modes = modeList(display);
			Display.Mode target = pickHigherMode(display, current);
			if (target == null) {
				String msg = "window: no mode >= " + MIN_VOTE_HZ + " Hz at "
						+ current.getPhysicalWidth() + "x"
						+ current.getPhysicalHeight()
						+ "; staying at " + current.getRefreshRate()
						+ " Hz; modes: " + modes;
				Log.i(TAG, msg);
				report(msg);
				return;
			}
			mTargetRate = target.getRefreshRate();
			android.view.Window window = activity.getWindow();
			WindowManager.LayoutParams lp = window.getAttributes();
			if (lp.preferredDisplayModeId != 0 && lp.preferredDisplayModeId != target.getModeId()) {
				String msg = "window: another mode pinned ("
						+ lp.preferredDisplayModeId + "); leaving it";
				Log.i(TAG, msg);
				report(msg);
				return;
			}
			lp.preferredDisplayModeId = target.getModeId();
			lp.preferredRefreshRate = target.getRefreshRate();
			window.setAttributes(lp);
			String msg = "window vote: mode " + target.getModeId() + " @ "
					+ target.getRefreshRate() + " Hz (panel reads "
					+ current.getRefreshRate() + " Hz, mode "
					+ current.getModeId() + "); modes: " + modes
					+ "; displayState=" + display.getState();
			Log.i(TAG, msg);
			report(msg);
		} catch (Throwable t) {
			Log.w(TAG, "window vote failed", t);
			report("window vote FAILED: " + t);
		}
	}

	/** Lowest refresh >= MIN_VOTE_HZ among the current mode's same-size modes:
	 *  the panel's high tier. 90 is the phone's own high category (measured:
	 *  dumpsys display FrameRateCategoryRate normal=60.0 high=90.0); voting the
	 *  topmost tier would double the render load for +33% rate. */
	private Display.Mode pickHigherMode(Display display, Display.Mode current) {
		Display.Mode best = null;
		for (Display.Mode mode : display.getSupportedModes()) {
			if (mode.getPhysicalWidth() != current.getPhysicalWidth()
					|| mode.getPhysicalHeight() != current.getPhysicalHeight()) {
				continue;
			}
			if (mode.getRefreshRate() < MIN_VOTE_HZ) {
				continue;
			}
			if (best == null || mode.getRefreshRate() < best.getRefreshRate()) {
				best = mode;
			}
		}
		return best;
	}

	/** Compact listing of the panel's modes, e.g. "1@120 2@60 3@90 ...". */
	private String modeList(Display display) {
		StringBuilder b = new StringBuilder();
		for (Display.Mode mode : display.getSupportedModes()) {
			if (b.length() > 0) {
				b.append(' ');
			}
			b.append(mode.getModeId()).append('@').append(mode.getRefreshRate());
		}
		return b.toString();
	}

	/** Surface-level vote on the GL render surface (belt and braces). */
	private void voteSurface() {
		if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S || mTargetRate <= MIN_VOTE_HZ) {
			return;
		}
		try {
			Activity activity = getActivity();
			if (activity == null) {
				return;
			}
			View root = activity.getWindow().getDecorView();
			GLSurfaceView glView = findGlSurfaceView(root);
			if (glView == null) {
				return;
			}
			Surface surface = glView.getHolder().getSurface();
			if (surface == null || !surface.isValid()) {
				return;
			}
			surface.setFrameRate(mTargetRate, Surface.CHANGE_FRAME_RATE_ALWAYS);
			String msg = "surface vote: " + mTargetRate + " Hz CHANGE_FRAME_RATE_ALWAYS";
			Log.i(TAG, msg);
			report(msg);
		} catch (Throwable t) {
			Log.w(TAG, "surface vote failed", t);
			report("surface vote FAILED: " + t);
		}
	}

	private GLSurfaceView findGlSurfaceView(View view) {
		if (view instanceof GLSurfaceView) {
			return (GLSurfaceView) view;
		}
		if (view instanceof ViewGroup) {
			ViewGroup group = (ViewGroup) view;
			for (int i = 0; i < group.getChildCount(); i++) {
				GLSurfaceView found = findGlSurfaceView(group.getChildAt(i));
				if (found != null) {
					return found;
				}
			}
		}
		return null;
	}

	/** One line appended to the app's files dir. ColorOS swallows this app's
	 *  logcat, so vote decisions are only readable back through adb with
	 *  run-as (see the class doc). Failures are swallowed deliberately: the
	 *  vote path must never crash the app. */
	private void report(String line) {
		try {
			Activity activity = getActivity();
			if (activity == null) {
				return;
			}
			java.io.File file = new java.io.File(activity.getFilesDir(), "vote-java.txt");
			java.io.FileWriter writer = new java.io.FileWriter(file, true);
			writer.write(System.currentTimeMillis() + "| " + line + "\n");
			writer.close();
		} catch (Throwable ignored) {
		}
	}

	/** Schedule a delayed active-mode poll on the main thread. */
	private void pollLater(long delayMs) {
		try {
			android.os.Handler handler = new android.os.Handler(android.os.Looper.getMainLooper());
			handler.postDelayed(() -> pollActiveMode(), delayMs);
		} catch (Throwable ignored) {
		}
	}

	private void pollActiveMode() {
		try {
			Activity activity = getActivity();
			if (activity == null) {
				return;
			}
			Display display = ((WindowManager) activity.getSystemService(Activity.WINDOW_SERVICE))
					.getDefaultDisplay();
			Display.Mode mode = display.getMode();
			report("poll: state=" + display.getState() + " activeMode="
					+ mode.getModeId() + "@" + mode.getRefreshRate());
			voteSurface();
		} catch (Throwable t) {
			report("poll failed: " + t);
		}
	}
}