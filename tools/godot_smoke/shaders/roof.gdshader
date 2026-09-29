// S2 roof shader: per-building lightness variation on the planar XZ roofs.
// The variation source is the loader's per-building seed (vertex COLOR.r,
// one o group = one building roof), not a normal hash: neighbouring
// triangles of the same roof plane carry near-equal but not identical
// normals, which a normal-based hash would resolve into visibly split
// shades. CLASSIFIER NOTE (T7): the multiplier is a scalar product of an
// already-classified roof albedo (tile_brown, slate — red/neutral), so
// dominance relations survive it by construction.
shader_type spatial;

uniform vec4 albedo : source_color = vec4(0.5, 0.5, 0.5, 1.0);
uniform float shade_amp = 0.16;   // per-building lightness +-16%


void fragment() {
	vec3 c = albedo.rgb;
	float seed = COLOR.r;
	c *= (1.0 - shade_amp) + 2.0 * shade_amp * seed;
	ALBEDO = c;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}