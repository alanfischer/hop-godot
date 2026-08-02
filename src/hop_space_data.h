#pragma once

#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

#include <hop/hop.h>
#include <hop/bvh_manager.h>
#include <memory>
#include <vector>
#include "hop_conversions.h"

using namespace godot;

class HopDirectSpaceState;

struct HopSpaceData {
	RID self_rid;
	bool active = false;

	std::unique_ptr<hop::simulator<hop_scalar>> simulator;
	hop::bvh_manager<hop_scalar> bvh_manager;

	// Dedicated broadphase for area sensor solids — a pure spatial index, NOT
	// driven by the simulator (areas never step or block).  Accelerates area
	// overlap queries and area-area monitoring from O(areas) to O(log areas).
	// Holds only areas, so broadphase results never need body/area disambiguation.
	hop::bvh_manager<hop_scalar> area_bvh;

	// Space params (Godot defaults)
	float contact_recycle_radius = 0.01f;
	float contact_max_separation = 0.05f;
	float contact_max_allowed_penetration = 0.01f;
	float contact_default_bias = 0.8f;
	float body_linear_velocity_sleep_threshold = 0.01f;
	float body_angular_velocity_sleep_threshold = 0.05f;
	float body_time_to_sleep = 0.5f;
	float solver_iterations = 8.0f;

	// Debug contacts
	int max_debug_contacts = 0;
	PackedVector3Array debug_contacts;

	// Cached direct state (created lazily, owned by this space)
	HopDirectSpaceState *direct_state = nullptr;

	// The space's DEFAULT area, which Godot addresses by the space RID rather than an
	// area RID — that is how physics/3d/default_gravity reaches a PhysicsServer3D.
	// Magnitude and direction arrive as separate calls, so both are kept and the
	// simulator is re-pushed whenever either changes.
	float default_gravity = 9.81f;
	Vector3 default_gravity_direction = Vector3(0.0f, -1.0f, 0.0f);

	void apply_default_gravity() {
		Vector3 g = default_gravity_direction.normalized() * default_gravity;
		simulator->set_gravity(hop::vec3<hop_scalar>(
			to_hop_scalar(g.x), to_hop_scalar(g.y), to_hop_scalar(g.z)));
	}

	HopSpaceData() {
		simulator = std::make_unique<hop::simulator<hop_scalar>>();
		simulator->set_manager(&bvh_manager);
		apply_default_gravity();
		simulator->set_integrator(hop::integrator_type::improved);
		// Default every body to the speculative solve (matches the old global
		// set_speculative_contacts(true); per-body overridable via set_contact_mode).
		simulator->set_default_contact_mode(hop::contact_mode::speculative);
		// Narrowphase accuracy for rounded-vs-polytope pairs: accurate GJK
		// (default) vs the cheap inflate/AABB path. Pick via the project setting
		// "physics/hop/accurate_narrowphase" (bool); absent => true.
		bool accurate = true;
		if (ProjectSettings *ps = ProjectSettings::get_singleton())
			accurate = ps->get_setting("physics/hop/accurate_narrowphase", true);
		simulator->set_accurate_narrowphase(accurate);
		// Angular substepping (opt-in continuous collision for fast spinners): cap the
		// number of fixed-orientation sub-traces per frame. "physics/hop/angular_substeps_max"
		// (int); absent / 1 => off (single snapshot, bit-identical). Raise it for a scene
		// with fast, thin spinners (a blade trap) that must not be stepped over angularly.
		int substeps = 1;
		if (ProjectSettings *ps = ProjectSettings::get_singleton())
			substeps = (int)ps->get_setting("physics/hop/angular_substeps_max", 1);
		simulator->set_angular_substeps_max(substeps);
	}
};
