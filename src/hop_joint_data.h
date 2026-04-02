#pragma once

#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <hop/hop.h>
#include <memory>
#include "hop_conversions.h"

using namespace godot;

struct HopJointData {
	RID self_rid;
	PhysicsServer3D::JointType type = PhysicsServer3D::JOINT_TYPE_MAX;
	int solver_priority = 1;
	bool disable_collisions = false;

	// Pin joint data
	RID body_a;
	RID body_b;
	Vector3 local_a;
	Vector3 local_b;
	float pin_bias = 0.3f;
	float pin_damping = 1.0f;
	float pin_impulse_clamp = 0.0f;

	// 6DOF spring data
	bool linear_spring_enabled = false;
	float linear_spring_stiffness = 0.0f;
	float linear_spring_damping = 0.0f;
	float linear_spring_equilibrium = 0.0f;

	// hop backing
	std::shared_ptr<hop::constraint<hop_scalar>> hop_constraint;
};
