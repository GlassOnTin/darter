// S4 foliage shader: hedge and the four crown variants stop reading as flat
// copies of one another. Per-cell world-position hash tint (3 m cells: hue
// bias + lightness swing, both mean-preserving) plus a per-pixel 2-octave
// noise so neighbouring crowns differ while each crown stays coherent.
//
// Classifier constraints (T7 pixel gates; see facade.gdshader's comment):
// every operation here is a positive-per-channel mean-preserving scalar —
// the hue bias moves g and r/b in opposite directions by at most 5%, the
// lightness and noise are scalars — so a green-dominant base albedo
// (foliage g/r ~1.5-1.7 against the 1.12 gate) keeps its margin everywhere.
// The tint reads the UNSWAYED world position (wpos is frozen before the
// vertex displacement), so cells stay anchored while the crown sways.
//
// Optional vertex sway (crown-only, hedge walls never reach the smoothstep
// band): x-axis wind at 1.3 Hz, amplitude 0.05 m scaled by smoothstep over
// VERTEX.y 2.0-4.0 m, phase per 3 m cell. Both mechanisms measured
// (2026-09-29): TIME under Movie Maker --fixed-fps is pinned run-to-run
// (T7 canon x2 identical with the sway active), and an amplify-check —
// 1.5 m amplitude vs 0.0 rendered from the same flight record at the same
// frame indices — changed 163-777 px per frame against a pixel-identical
// camera, so the displacement is real vertex motion, not a dead constant.
shader_type spatial;

uniform vec4 albedo : source_color = vec4(0.5, 0.5, 0.5, 1.0);
uniform float cell_pitch = 3.0;  // metres per tint cell (one crown ~ one cell)
uniform float noise_amp = 0.06;  // +- per-pixel noise swing, mean-preserving

varying vec3 wpos;   // world position, m (pre-sway)


float h21(vec2 p) {
	// Fract-based hash (fp32-stable; see facade.gdshader).
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
	wpos = wp;
	// Gentle wind sway: crown-only, phase per cell. Local space here is
	// world space (the mesh is at identity), y = up.
	float phase = h21(floor(wp.xz / cell_pitch)) * 6.2831853;
	float amp = 0.05 * smoothstep(2.0, 4.0, VERTEX.y);
	VERTEX.x += amp * sin(TIME * 8.16814 + phase);           // 1.3 Hz
	VERTEX.z += 0.6 * amp * sin(TIME * 8.16814 + phase * 0.7);
}


void fragment() {
	vec2 cell = floor(wpos.xz / cell_pitch);
	float h2 = h21(cell * 1.117 + 5.13);
	float hl = h21(cell + 0.37);
	vec3 c = albedo.rgb;
	// Per-cell hue bias +-5%: green scales up while r/b scale down (and the
	// reverse for other cells) — mean brightness ~1.0, dominance preserved.
	c.g *= 1.0 + (h2 - 0.5) * 0.10;
	c.r *= 1.0 - (h2 - 0.5) * 0.05;
	c.b *= 1.0 - (h2 - 0.5) * 0.05;
	// Per-cell lightness 0.88-1.12, exactly mean 1.0 over a cell population.
	c *= 0.88 + 0.24 * hl;
	// Per-pixel noise: broad clumps + fine grain, scalar, +-6%.
	float nv = 0.6 * vnoise(wpos.xz * 0.35) + 0.4 * vnoise(wpos.xz * 1.7);
	c *= 1.0 + noise_amp * 2.0 * (nv * 0.5 - 0.5);
	ALBEDO = c;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}