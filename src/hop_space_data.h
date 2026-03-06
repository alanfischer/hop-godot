#pragma once

#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

#include <hop/hop.h>
#include <memory>
#include <vector>

using namespace godot;

class HopDirectSpaceState;

struct HopSpaceData {
	RID self_rid;
	bool active = false;

	std::unique_ptr<hop::simulator<float>> simulator;

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
		simulator = std::make_unique<hop::simulator<float>>();
		simulator->set_gravity(hop::vec3<float>(0.0f, -9.8f, 0.0f));
		simulator->set_integrator(hop::integrator_type::improved);
	}
};
