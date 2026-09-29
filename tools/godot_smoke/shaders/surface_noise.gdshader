// S4 surface-noise shader: bark and fence get per-pixel noise over the
// material tint, so the vertical flat panels stop reading as one colour
// with lighting only. Scalar, mean-preserving +-8% — the man-made class
// ratio products cannot flip (see facade.gdshader's classifier comment).
shader_type spatial;

uniform vec4 albedo : source_color = vec4(0.5, 0.5, 0.5, 1.0);
uniform float noise_amp = 0.08;  // +- per-pixel noise swing, mean-preserving

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
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}


void fragment() {
	// Noise plane picks the face's dominant axis: bark ridges vary along
	// the trunk's height (xy/zy for radial faces), fence weathering varies
	// along the run AND its height. Without this, a thin panel's XZ span is
	// near-constant and the noise collapses to vertical stripes.
	vec2 np = abs(NORMAL.x) > 0.5 ? wpos.zy : (abs(NORMAL.z) > 0.5 ? wpos.xy : wpos.xz);
	// Bark/fence grain: medium streaks + fine grain (fence planks and bark
	// ridging both live at these scales).
	float nv = 0.6 * vnoise(np * 1.4) + 0.4 * vnoise(np * 6.0);
	vec3 c = albedo.rgb * (1.0 + noise_amp * 2.0 * (nv * 0.5 - 0.5));
	ALBEDO = c;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}