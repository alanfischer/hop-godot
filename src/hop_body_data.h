#pragma once

#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <hop/hop.h>
#include <memory>
#include <vector>
#include <set>

using namespace godot;

class HopDirectBodyState;

struct HopBodyShapeEntry {
	RID shape_rid;
	Transform3D local_xform;
	bool disabled = false;
	std::shared_ptr<hop::shape<float>> hop_shape;
};

struct HopBodyData {
	RID self_rid;
	RID space_rid;
	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;
	uint64_t object_instance_id = 0;

	Transform3D transform;
	Vector3 linear_velocity;
	Vector3 angular_velocity; // stored but not used by hop
	Vector3 constant_force;
	Vector3 constant_torque; // stored but not used by hop

	float mass = 1.0f;
	float bounce = 0.0f;
	float friction = 1.0f;
	float gravity_scale = 1.0f;
	float linear_damp = 0.0f;
	float angular_damp = 0.0f;

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	float collision_priority = 1.0f;

	bool ccd_enabled = false;
	bool ray_pickable = true;
	bool sleeping = false;
	bool omit_force_integration = false;
	bool can_sleep = true;

	int max_contacts_reported = 0;
	float contacts_depth_threshold = 0.0f;

	uint32_t user_flags = 0;

	Callable state_sync_callback;
	Callable force_integration_callback;
	Variant force_integration_userdata;

	std::vector<HopBodyShapeEntry> shapes;
	std::vector<RID> collision_exceptions;

	// hop backing
	std::shared_ptr<hop::solid<float>> hop_solid;

	// Contact info from last step
	struct ContactInfo {
		Vector3 local_pos;
		Vector3 local_normal;
		Vector3 impulse;
		int local_shape = 0;
		Vector3 local_velocity;
		RID collider_rid;
		Vector3 collider_pos;
		uint64_t collider_id = 0;
		int collider_shape = 0;
		Vector3 collider_velocity;
	};
	std::vector<ContactInfo> contacts;

	// Cached direct state (created lazily, owned by this body)
	HopDirectBodyState *direct_state = nullptr;

	void create_hop_solid();
	void sync_to_hop();
	void sync_from_hop();
	void on_collision(const hop::collision<float> &c);
	bool is_static_or_kinematic() const {
		return mode == PhysicsServer3D::BODY_MODE_STATIC ||
			   mode == PhysicsServer3D::BODY_MODE_KINEMATIC;
	}
};
