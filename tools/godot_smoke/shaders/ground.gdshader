// S3 ground shader: CC0 photo texture x material tint x macro noise, for the
// three world-horizontal surface materials (grass, asphalt, concrete — the
// base quad, roads and parking pads, paths and paved areas all sit within
// z <= 0.075 m, so one world-XZ planar projection serves all three).
//
// UV is world XZ in metres divided by tile_size (10 m per tile): every
// surface shares the physical texture scale, and hardware repeat tiles the
// 4.7 km base quad. Tiling seams on the big quad are broken by the macro
// noise below (2-octave value noise +-10%, mean-preserving, wavelengths
// ~24 m and ~4.4 m).
//
// Classifier constraints (T7 pixel gates; see facade.gdshader's comment):
// - albedo = tint * texture_sample: measured texture g/r ratios (grass 1.34,
//   asphalt 1.00, concrete 0.96) multiply with the tint ratios (grass 1.24,
//   asphalt 1.00, concrete 1.00), so the tinted grass stays green-dominant
//   (product g/r ~1.66 > the 1.12 gate with margin) and asphalt/concrete
//   stay man-made. The noise is a SCALAR mean-preserving multiplier — it
//   cannot flip a channel dominance relation anywhere.
// - fp32: world positions reach ~2400 m; the value noise input is metres
//   (not tiles), hashed with the fp32-safe fract hash (no sin of large
//   arguments); the tile coordinates are handled by hardware repeat.
shader_type spatial;

uniform sampler2D tex : source_color, filter_linear_mipmap, repeat_enable;
uniform vec4 albedo : source_color = vec4(0.5, 0.5, 0.5, 1.0);
uniform float tile_size = 10.0;   // metres per texture repeat
uniform float noise_amp = 0.10;   // +- macro-noise swing, mean-preserving

varying vec3 wpos;   // world position, m


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
}


void fragment() {
	vec3 c = albedo.rgb * texture(tex, wpos.xz / tile_size).rgb;
	// Macro noise: broad patches, medium, fine grain on top.
	float nv = 0.55 * vnoise(wpos.xz * 0.041)
			+ 0.30 * vnoise(wpos.xz * 0.23)
			+ 0.15 * vnoise(wpos.xz * 1.1);
	c *= 1.0 + noise_amp * 2.0 * (nv * 0.5 - 0.5);
	ALBEDO = c;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}