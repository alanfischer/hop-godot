#pragma once

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

	HopSpaceData() {
		simulator = std::make_unique<hop::simulator<hop_scalar>>();
		simulator->set_manager(&bvh_manager);
		simulator->set_gravity(hop::vec3<hop_scalar>(
			scalar_from_int<hop_scalar>(0),
			scalar_from_milli<hop_scalar>(-9810),
			scalar_from_int<hop_scalar>(0)));
		simulator->set_integrator(hop::integrator_type::improved);
		// Default every body to the speculative solve (matches the old global
		// set_speculative_contacts(true); per-body overridable via set_contact_mode).
		simulator->set_default_contact_mode(hop::contact_mode::speculative);
	}
};
