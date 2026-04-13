#pragma once

#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <vector>
#include <map>

using namespace godot;

struct HopAreaShapeEntry {
	RID shape_rid;
	Transform3D local_xform;
	bool disabled = false;
};

struct HopAreaData {
	RID self_rid;
	RID space_rid;
	uint64_t object_instance_id = 0;

	Transform3D transform;

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	bool monitorable = false;
	bool ray_pickable = false;

	// Gravity / damp overrides
	float gravity = 9.8f;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	bool gravity_is_point = false;
	float gravity_point_distance = 0.0f;
	PhysicsServer3D::AreaSpaceOverrideMode space_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	float linear_damp = 0.1f;
	float angular_damp = 0.1f;
	PhysicsServer3D::AreaSpaceOverrideMode linear_damp_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	PhysicsServer3D::AreaSpaceOverrideMode angular_damp_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	int priority = 0;

	// Fluid / wind: combined with linear_damp, they make a current or wind zone.
	// Fluid velocity = wind_direction * wind_force_magnitude.
	// Bodies inside converge toward this velocity via drag: F = fluid_vel * drag_coeff.
	float wind_force_magnitude = 0.0f;
	Vector3 wind_direction = Vector3(0, 0, -1); // Godot default
	float wind_attenuation_factor = 0.0f;

	Callable monitor_callback;
	Callable area_monitor_callback;

	std::vector<HopAreaShapeEntry> shapes;

	// Overlap tracking for monitor callbacks
	// Maps object_instance_id -> body RID for currently overlapping bodies
	std::map<uint64_t, RID> overlapping_bodies;
	// Maps object_instance_id -> area RID for currently overlapping areas
	std::map<uint64_t, RID> overlapping_areas;
};
