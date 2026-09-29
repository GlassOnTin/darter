# Five frozen lighting presets for the pack scene, derived offline from
# anisoptera's ue/render_synth.py :: draw_lighting() (the LB01-LB03 lighting
# model: sun elev/az draw, cloud/haze weather ranges, warm->blue sky lerp
# greying toward lum*1.05, sun_energy U(1.5,3.5)*(1-0.85*cloud), Koschmieder
# visibility bands). Each preset quotes its derivation draw so every number
# is re-derivable; the derivation script re-draws the seed in the same order
# and asserts field-by-field equality with the real draw_lighting() before
# its numbers are trusted (all five verified 2026-09-29 against the anisoptera
# checkout).
#
# Godot mapping (compatibility renderer):
# - fog_density = beta = 3.0/V directly: compat fog is exact exponential
#   extinction 1-exp(-dist*density), no anisoptera _FOG_CAL Blender bridge;
# - shadow_sun energy = sun_energy*(1-0.97*cloud)*10 and ambient energy =
#   sky_intensity*(1+1.5*cloud): the UE plugin's cloud split
#   (ue/AnisopteraBridgeActor.cpp ServiceLighting) against one directional
#   light + ambient colour. The UE x10 sun scale (10-lux directional vs 1-lux
#   skylight defaults) is RELATIVE structure, not a unit: first attempt
#   dropped it and the sun:ambient ratio measured 1:1 — flat/washed, no form
#   (2026-09-29 frame). Corrected to port the ratio; the EV solve then
#   handles absolute exposure;
# - sun beam = travel dir (dx, dz, -dy) from the dataset frame, placed with
#   rotation_degrees (-elev, az, 0) — verified algebraically against Ry*Rx
#   on the beam axis;
# - windows_lit carried for the S2 facade shader; exposure for the S1
#   grey-card solve (target mean brightness 165/255 = 0.647, anisoptera
#   TARGET_MEAN).
#
# SKY: this pinned compatibility build bypasses custom sky shaders entirely
# (probe 2026-09-29: a solid-red shader_type sky renders the dark default
# clear colour, unchanged). The sky is a ProceduralSkyMaterial whose colours
# are scalar products of the preset zenith, so b/g/r dominance travels with
# it. Blue-dominance constraint (T7 pixel-class gate): for the default
# preset (clear_noon) sky pixels the 16-px sampler sees must keep
# b > r*1.15 and b > g*1.05 after the Filmic tonemap; scalar products and
# convex mixes of blue-dominant colours both preserve that. If the default
# preset ever fails the sky gate, bias the zenith colour here first (one
# line), not the gate bound. The procedural sky draws its sun disc from the
# actual DirectionalLight3D, so disc and shadows cannot disagree, and the
# cloud-quenched energy dims it on the overcast/fog presets.
#
# Preset selection: tests never set DARTER_LIGHTING, so the default pins the
# canon run. clear_noon is the default and the only preset whose sky stays
# blue-dominant — the other four are dev/eyeball presets (warm skies
# classify as man-made by the sampler) and are NOT expected to pass T7.
extends RefCounted

const DEFAULT_NAME := "clear_noon"

const PRESETS := {
	"clear_noon": {
		"elev_deg": 61.7857, "az_deg": 201.5181, "cloud": 0.323474,
		"zenith": Color(0.409816, 0.458865, 0.542323),
		"sun_rgb": Color(1.0, 0.978518, 0.961332),
		"sun_energy": 12.516770,      # shadow-sun: sun_energy*(1-0.97*cloud)
		"ambient_energy": 1.364555,  # sky_intensity*(1+1.5*cloud)
		"beta": 9.449504e-05,        # V = 31747.7 m (clear band 20-40 km)
		"fog_sky_affect": 0.5,
		"shadow_max_distance": 200.0,
		"windows_lit": 0.0,
		# Grey-card solve (anisoptera meter_episode, TARGET 165/255 = 0.647),
		# re-solved three times in S1: (1) dark-default sky (void — the sky
		# was unlit), (2) lit ProceduralSky at sun:ambient 1:1, then (3) after
		# porting the UE sun:x10 ratio AND lifting the sky radiance
		# (sky_energy_multiplier 5.0 — with the palette as radiance the sun
		# bleached the ground next to a slate-dark sky). Final solve: x10 + x5
		# structure, 18 metered rounds, landing mean_bm 0.6477 at the frozen
		# value (2026-09-29).
		"tonemap_exposure": 0.054452,
	},
	"clear_low": {
		"elev_deg": 26.8643, "az_deg": 236.6901, "cloud": 0.233244,
		"zenith": Color(0.598158, 0.499043, 0.443760),
		"sun_rgb": Color(1.0, 0.820887, 0.677596),
		"sun_energy": 9.440020,
		"ambient_energy": 1.215277,
		"beta": 1.091104e-04,        # V = 27495.1 m
		"fog_sky_affect": 0.7,
		"shadow_max_distance": 200.0,
		"windows_lit": 0.05,
		"tonemap_exposure": 1.0,
	},
	"clear_dawn": {
		"elev_deg": 4.0721, "az_deg": 246.6929, "cloud": 0.239429,
		"zenith": Color(0.685479, 0.518153, 0.396462),
		"sun_rgb": Color(1.0, 0.60, 0.35),        # dawn tweak (-0.15 g, -0.20 b)
		"sun_energy": 11.444000,
		"ambient_energy": 1.143284,
		"beta": 1.218959e-04,        # V = 24611.2 m
		"fog_sky_affect": 0.7,
		# A 4-degree sun casts ~77 m house shadows; the shadow split must
		# reach them.
		"shadow_max_distance": 350.0,
		"windows_lit": 0.15,
		"tonemap_exposure": 1.0,
	},
	"overcast_noon": {
		"elev_deg": 56.6024, "az_deg": 252.9308, "cloud": 0.860817,
		"zenith": Color(0.466747, 0.473809, 0.488044),
		"sun_rgb": Color(1.0, 0.961512, 0.930722),
		"sun_energy": 0.904150,      # nearly quenched: (1-0.97*cloud) at 0.86
		"ambient_energy": 3.596310,
		"beta": 4.417983e-04,        # V = 6790.4 m (overcast band 5-12 km)
		"fog_sky_affect": 1.0,
		"shadow_max_distance": 200.0,
		"windows_lit": 0.10,
		"tonemap_exposure": 1.0,
	},
	"fog_noon": {
		"elev_deg": 69.4786, "az_deg": 4.1957, "cloud": 0.920797,
		"zenith": Color(0.450382, 0.458185, 0.469951),
		"sun_rgb": Color(1.0, 0.998843, 0.997917),
		"sun_energy": 0.806210,
		"ambient_energy": 4.010658,
		"beta": 2.855397e-02,        # V = 105.1 m (fog band 0.1-0.4 km)
		"fog_sky_affect": 1.0,
		"shadow_max_distance": 200.0,
		"windows_lit": 0.20,
		"tonemap_exposure": 1.0,
	},
}


static func get_preset(name: String) -> Dictionary:
	if not PRESETS.has(name):
		push_error("unknown lighting preset %s (have %s)" % [name, str(PRESETS.keys())])
		return {}
	var p: Dictionary = PRESETS[name].duplicate()
	p["name"] = name
	return p


## Apply one preset to the WorldEnvironment's Environment, the sky's
## ProceduralSkyMaterial, and the sun DirectionalLight3D.
static func apply(p: Dictionary, env: Environment, sky_mat: ProceduralSkyMaterial, sun: DirectionalLight3D) -> void:
	env.background_mode = Environment.BG_SKY
	env.sky = Sky.new()
	env.sky.sky_material = sky_mat
	# Filmic tonemap + the grey-card-solved exposure (see the preset's
	# tonemap_exposure provenance comment).
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.tonemap_exposure = p["tonemap_exposure"]
	# AMBIENT_SOURCE_COLOR with the preset zenith: the UE Skylight analogue,
	# direct control of colour and energy (AMBIENT_SOURCE_BG radiance from
	# the procedural sky is untested in compat).
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = p["zenith"]
	env.ambient_light_energy = p["ambient_energy"]
	env.fog_enabled = true
	env.fog_density = p["beta"]
	# Fog inscatter = the horizon-band radiance (the light the fogged ground
	# sits against): zenith * horizon_lift * sky_radiance_scale. Without the
	# scale the inscatter mixed in DARKER than the ground it covers — dark
	# haze (2026-09-29 frame).
	env.fog_light_color = p["zenith"] * 1.55 * 5.0
	env.fog_sky_affect = p["fog_sky_affect"]
	env.fog_height = 0.0
	env.fog_height_density = 0.0

	# Sky colours: all scalar products of the zenith (see the header). The
	# zenith palette is a tone-mapped-look colour, not radiance — 5.0 lifts
	# the sky to a radiance that reads bright next to the x10 sun's ground
	# (dark-slate-sky-vs-bleached-ground measured 2026-09-29 without it).
	sky_mat.sky_top_color = p["zenith"]
	sky_mat.sky_horizon_color = p["zenith"] * 1.55
	sky_mat.sky_curve = 0.12
	sky_mat.sky_energy_multiplier = 5.0
	sky_mat.ground_bottom_color = p["zenith"] * 0.55
	sky_mat.ground_horizon_color = p["zenith"] * 1.3
	sky_mat.ground_curve = 0.02
	sky_mat.ground_energy_multiplier = 1.0
	# Tight styled sun disc drawn from the DirectionalLight3D itself.
	sky_mat.sun_angle_max = 4.0
	sky_mat.sun_curve = 0.12

	sun.rotation_degrees = Vector3(-p["elev_deg"], p["az_deg"], 0.0)
	sun.light_color = p["sun_rgb"]
	sun.light_energy = p["sun_energy"]
	sun.shadow_enabled = true
	# PSSM-2 over the preset distance: the compatibility default PSSM-4
	# renders the pack 5x per frame; two splits cut it to 3x (measured at
	# 200 m, commit 4f0639d's baseline) with unchanged near-field shadows.
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_2_SPLITS
	sun.directional_shadow_max_distance = p["shadow_max_distance"]