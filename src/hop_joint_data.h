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

	// Cone-twist data. Godot's own defaults, which it will also set explicitly right after
	// _joint_make_cone_twist — the spans arrive in RADIANS even though PhysicalBone3D's
	// inspector shows degrees.
	float cone_swing_span = 0.7853982f;  // 45 degrees
	float cone_twist_span = 3.1415927f;  // 180 degrees
	float cone_bias = 0.3f;
	float cone_softness = 0.8f;
	float cone_relaxation = 1.0f;

	// 6DOF spring data
	bool linear_spring_enabled = false;
	float linear_spring_stiffness = 0.0f;
	float linear_spring_damping = 0.0f;
	float linear_spring_equilibrium = 0.0f;

	// hop backing
	std::shared_ptr<hop::constraint<hop_scalar>> hop_constraint;
};
