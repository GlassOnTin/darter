// S2 facade shader: floor bands, window cells, hash-lit windows, grime.
// UVs are computed in vertex() from world METRES, so every wall shares the
// same physical scale with no importer/loader format change.
//
// Fragment-space classifier constraints (T7 pixel gates, pack_replay.gd:
// sky = b > r*1.15 && b > g*1.05; veg = g > r*1.12 && g > b*1.05):
// - window glass is kept WARM (r >= g >= b). The S1 ambient is the blue sky
//   colour itself, so shadow-side walls read cool; a dark glass tinted the
//   planned (0.09,0.11,0.13) would itself be blue-dominant and classify
//   shadow windows as sky. (0.13,0.122,0.112) cannot pass either test even
//   under the blue ambient.
// - lit windows emit warm (r >> b) and never leave the man-made class.
// - bands/plinth/per-building tint are scalar multiplications of already-
//   classified albedos: the red/neutral dominance of brick, render and tile
//   albedos survives them.
//
// World-anchored grid: window/band/grime coordinates come from world
// position, NOT per-building registration (the pack format carries no UVs),
// so window columns do not align with building edges and can be cut by an
// edge. Accepted for S2 (2026-09-29): reads as scattered houses, not a
// texture seam.
shader_type spatial;

uniform vec4 albedo : source_color = vec4(0.5, 0.5, 0.5, 1.0);
uniform float band_period = 3.0;    // storey pitch, m
uniform float band_width = 0.16;    // darkened stripe width, m
uniform float band_darken = 0.90;   // stripe multiplier
uniform float cell_pitch = 2.2;     // window cell pitch, m
uniform float win_wfrac = 0.62;     // window aperture as fraction of the
uniform float win_hfrac = 0.55;     // cell (1.36 x 1.21 m at 2.2 m pitch)
uniform float lit_fraction = 0.0;   // hash-lit windows (per lighting preset)
uniform bool plain_mode = false;    // windowless walls: plinth, no windows
uniform float grime_amp = 0.08;     // +- grime noise, mean-preserving

varying vec2 wall_uv;   // (metres along-wall from the world tangent line, height)
varying vec3 wpos;      // world position, m
varying float vdist;    // camera distance, m (view-space VERTEX in fragment)


float h21(vec2 p) {
	// Fract-based hash: stable in fp32 (no sin of large arguments).
	vec3 p3 = fract(vec3(p.xyx) * 0.1031);
	p3 += dot(p3, p3.yzx + 33.33);
	return fract((p3.x + p3.y) * p3.z);
}


float vnoise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(h21(i), h21(i + vec2(1.0, 0.0)), u.x),
			mix(h21(i + vec2(0.0, 1.0)), h21(i + vec2(1.0, 1.0)), u.x), u.y);
}


void vertex() {
	NORMAL = normalize(NORMAL);  // loader normals are unnormalized plane crosses
	vec3 wp = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	vec3 wn = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
	// Along-wall tangent: UP x normal. Degenerate on a (nonexistent here)
	// horizontal face inside a bld chunk; +X fallback instead of a NaN UV.
	vec3 t = cross(vec3(0.0, 1.0, 0.0), wn);
	t = (dot(t, t) < 1e-6) ? vec3(1.0, 0.0, 0.0) : normalize(t);
	wall_uv = vec2(dot(wp, t), wp.y);
	wpos = wp;
	// View space (camera at origin): metres to the fragment's vertex.
	vdist = length((MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz);
}


void fragment() {
	vec3 c = albedo.rgb;
	// Per-building seed (loader: one o group per building, same value on all
	// its vertices -> interpolation-constant across the chunk).
	float seed = COLOR.r;

	// Per-building tint: +-6%.
	c *= 0.94 + 0.12 * seed;

	// Grime: anisotropic two-octave value noise — stretched vertically so the
	// streaks run down the wall; broad patches plus fine grain. Multiplied by
	// (amp*2*(v*0.5 - 0.5)) with amp=0.08 gives +-8%, mean-preserving.
	float nv = 0.65 * vnoise(vec2(wpos.x * 0.42, wpos.y * 0.21))
			+ 0.35 * vnoise(vec2(wpos.x * 1.9, wpos.y * 0.35));
	c *= 1.0 + grime_amp * 2.0 * (nv * 0.5 - 0.5);

	// Storey bands: darkened stripe on every band_period boundary.
	float d2band = abs(fract(wall_uv.y / band_period + 0.5) - 0.5) * band_period;
	float stripe = 1.0 - smoothstep(band_width * 0.6, band_width * 1.1, d2band);
	c *= mix(1.0, band_darken, stripe);

	if (plain_mode) {
		// Windowless walls (garages and the like): darker plinth band only.
		c *= mix(1.0, 0.72, smoothstep(0.42, 0.18, wall_uv.y));
	} else {
		// Window cells on a world-anchored 2.2 m grid; aperture centred in
		// the cell, size as a fraction of the cell.
		vec2 f = fract(wall_uv / cell_pitch);
		vec2 cell_id = floor(wall_uv / cell_pitch);
		vec2 half_a = 0.5 * vec2(win_wfrac, win_hfrac);
		// Edge AA: a couple of screen pixels, floored so the aperture stays
		// sharp at close range.
		vec2 aa = max(fwidth(wall_uv), vec2(0.006)) / cell_pitch;
		float inside = smoothstep(0.0, aa.x, half_a.x - abs(f.x - 0.5))
				* smoothstep(0.0, aa.y, half_a.y - abs(f.y - 0.5));
		// Fade the pattern with distance (shimmer guard; the aerial-
		// perspective fog has already washed the far walls by ~80 m).
		float fade = 1.0 - smoothstep(45.0, 80.0, vdist);
		inside *= fade;

		// Per-window hash streams per window id, offset by the building seed
		// so adjacent houses never share the same lit pattern.
		float wh = h21(cell_id + vec2(seed * 91.7, -seed * 57.3));
		float is_lit = step(1.0 - lit_fraction, wh);

		c = mix(c, vec3(0.13, 0.122, 0.112), inside);
		EMISSION += is_lit * inside * vec3(0.85, 0.52, 0.28) * 0.9;
	}

	ALBEDO = c;
	ROUGHNESS = 0.95;
	METALLIC = 0.0;
}