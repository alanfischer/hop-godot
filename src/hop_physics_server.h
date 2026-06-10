#pragma once

#include <godot_cpp/classes/physics_server3d_extension.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/physics_server3d_rendering_server_handler.hpp>
#include <godot_cpp/classes/physics_direct_body_state3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>

#include "hop_rid.h"
#include "hop_shape_data.h"
#include "hop_body_data.h"
#include "hop_area_data.h"
#include "hop_space_data.h"
#include "hop_joint_data.h"

using namespace godot;

class HopDirectBodyState;
class HopDirectSpaceState;

class HopPhysicsServer : public PhysicsServer3DExtension {
	GDCLASS(HopPhysicsServer, PhysicsServer3DExtension);

protected:
	static void _bind_methods() {}

public:
	mutable HopRIDOwner<HopShapeData> shape_owner;
	mutable HopRIDOwner<HopBodyData> body_owner;
	mutable HopRIDOwner<HopAreaData> area_owner;
	mutable HopRIDOwner<HopSpaceData> space_owner;
	mutable HopRIDOwner<HopJointData> joint_owner;

	bool active = true;
	bool flushing_queries = false;
	float last_step = 1.0f / 60.0f;

	// Helpers
	void rebuild_body_shapes(HopBodyData *body);
	void add_body_to_space(HopBodyData *body, HopSpaceData *space);
	void remove_body_from_space(HopBodyData *body);

	// Areas are modeled as persistent hop sensor solids: shapes built once and
	// reused for narrow-phase overlap tests / world-bound checks (rebuilt only when
	// the area's shapes change), and indexed in the space's dedicated area_bvh
	// broadphase so overlap queries and area-area monitoring are O(log areas).
	void ensure_area_solid(HopAreaData *area);
	void rebuild_area_shapes(HopAreaData *area);
	void add_area_to_space(HopAreaData *area, HopSpaceData *space);
	void remove_area_from_space(HopAreaData *area);
	void mark_area_bvh_dirty(HopAreaData *area);

	// === Shape API ===
	RID _world_boundary_shape_create() override;
	RID _separation_ray_shape_create() override;
	RID _sphere_shape_create() override;
	RID _box_shape_create() override;
	RID _capsule_shape_create() override;
	RID _cylinder_shape_create() override;
	RID _convex_polygon_shape_create() override;
	RID _concave_polygon_shape_create() override;
	RID _heightmap_shape_create() override;
	RID _custom_shape_create() override;

	void _shape_set_data(const RID &p_shape, const Variant &p_data) override;
	void _shape_set_custom_solver_bias(const RID &p_shape, float p_bias) override;
	void _shape_set_margin(const RID &p_shape, float p_margin) override;
	float _shape_get_margin(const RID &p_shape) const override;
	PhysicsServer3D::ShapeType _shape_get_type(const RID &p_shape) const override;
	Variant _shape_get_data(const RID &p_shape) const override;
	float _shape_get_custom_solver_bias(const RID &p_shape) const override;

	// === Space API ===
	RID _space_create() override;
	void _space_set_active(const RID &p_space, bool p_active) override;
	bool _space_is_active(const RID &p_space) const override;
	void _space_set_param(const RID &p_space, PhysicsServer3D::SpaceParameter p_param, float p_value) override;
	float _space_get_param(const RID &p_space, PhysicsServer3D::SpaceParameter p_param) const override;
	PhysicsDirectSpaceState3D *_space_get_direct_state(const RID &p_space) override;
	void _space_set_debug_contacts(const RID &p_space, int32_t p_max_contacts) override;
	PackedVector3Array _space_get_contacts(const RID &p_space) const override;
	int32_t _space_get_contact_count(const RID &p_space) const override;

	// === Area API ===
	RID _area_create() override;
	void _area_set_space(const RID &p_area, const RID &p_space) override;
	RID _area_get_space(const RID &p_area) const override;
	void _area_add_shape(const RID &p_area, const RID &p_shape, const Transform3D &p_transform, bool p_disabled) override;
	void _area_set_shape(const RID &p_area, int32_t p_shape_idx, const RID &p_shape) override;
	void _area_set_shape_transform(const RID &p_area, int32_t p_shape_idx, const Transform3D &p_transform) override;
	void _area_set_shape_disabled(const RID &p_area, int32_t p_shape_idx, bool p_disabled) override;
	int32_t _area_get_shape_count(const RID &p_area) const override;
	RID _area_get_shape(const RID &p_area, int32_t p_shape_idx) const override;
	Transform3D _area_get_shape_transform(const RID &p_area, int32_t p_shape_idx) const override;
	void _area_remove_shape(const RID &p_area, int32_t p_shape_idx) override;
	void _area_clear_shapes(const RID &p_area) override;
	void _area_attach_object_instance_id(const RID &p_area, uint64_t p_id) override;
	uint64_t _area_get_object_instance_id(const RID &p_area) const override;
	void _area_set_param(const RID &p_area, PhysicsServer3D::AreaParameter p_param, const Variant &p_value) override;
	void _area_set_transform(const RID &p_area, const Transform3D &p_transform) override;
	Variant _area_get_param(const RID &p_area, PhysicsServer3D::AreaParameter p_param) const override;
	Transform3D _area_get_transform(const RID &p_area) const override;
	void _area_set_collision_layer(const RID &p_area, uint32_t p_layer) override;
	uint32_t _area_get_collision_layer(const RID &p_area) const override;
	void _area_set_collision_mask(const RID &p_area, uint32_t p_mask) override;
	uint32_t _area_get_collision_mask(const RID &p_area) const override;
	void _area_set_monitorable(const RID &p_area, bool p_monitorable) override;
	void _area_set_ray_pickable(const RID &p_area, bool p_enable) override;
	void _area_set_monitor_callback(const RID &p_area, const Callable &p_callback) override;
	void _area_set_area_monitor_callback(const RID &p_area, const Callable &p_callback) override;

	// === Body API ===
	RID _body_create() override;
	void _body_set_space(const RID &p_body, const RID &p_space) override;
	RID _body_get_space(const RID &p_body) const override;
	void _body_set_mode(const RID &p_body, PhysicsServer3D::BodyMode p_mode) override;
	PhysicsServer3D::BodyMode _body_get_mode(const RID &p_body) const override;
	void _body_add_shape(const RID &p_body, const RID &p_shape, const Transform3D &p_transform, bool p_disabled) override;
	void _body_set_shape(const RID &p_body, int32_t p_shape_idx, const RID &p_shape) override;
	void _body_set_shape_transform(const RID &p_body, int32_t p_shape_idx, const Transform3D &p_transform) override;
	void _body_set_shape_disabled(const RID &p_body, int32_t p_shape_idx, bool p_disabled) override;
	int32_t _body_get_shape_count(const RID &p_body) const override;
	RID _body_get_shape(const RID &p_body, int32_t p_shape_idx) const override;
	Transform3D _body_get_shape_transform(const RID &p_body, int32_t p_shape_idx) const override;
	void _body_remove_shape(const RID &p_body, int32_t p_shape_idx) override;
	void _body_clear_shapes(const RID &p_body) override;
	void _body_attach_object_instance_id(const RID &p_body, uint64_t p_id) override;
	uint64_t _body_get_object_instance_id(const RID &p_body) const override;
	void _body_set_enable_continuous_collision_detection(const RID &p_body, bool p_enable) override;
	bool _body_is_continuous_collision_detection_enabled(const RID &p_body) const override;
	void _body_set_collision_layer(const RID &p_body, uint32_t p_layer) override;
	uint32_t _body_get_collision_layer(const RID &p_body) const override;
	void _body_set_collision_mask(const RID &p_body, uint32_t p_mask) override;
	uint32_t _body_get_collision_mask(const RID &p_body) const override;
	void _body_set_collision_priority(const RID &p_body, float p_priority) override;
	float _body_get_collision_priority(const RID &p_body) const override;
	void _body_set_user_flags(const RID &p_body, uint32_t p_flags) override;
	uint32_t _body_get_user_flags(const RID &p_body) const override;
	void _body_set_param(const RID &p_body, PhysicsServer3D::BodyParameter p_param, const Variant &p_value) override;
	Variant _body_get_param(const RID &p_body, PhysicsServer3D::BodyParameter p_param) const override;
	void _body_reset_mass_properties(const RID &p_body) override;
	void _body_set_state(const RID &p_body, PhysicsServer3D::BodyState p_state, const Variant &p_value) override;
	Variant _body_get_state(const RID &p_body, PhysicsServer3D::BodyState p_state) const override;
	void _body_apply_central_impulse(const RID &p_body, const Vector3 &p_impulse) override;
	void _body_apply_impulse(const RID &p_body, const Vector3 &p_impulse, const Vector3 &p_position) override;
	void _body_apply_torque_impulse(const RID &p_body, const Vector3 &p_impulse) override;
	void _body_apply_central_force(const RID &p_body, const Vector3 &p_force) override;
	void _body_apply_force(const RID &p_body, const Vector3 &p_force, const Vector3 &p_position) override;
	void _body_apply_torque(const RID &p_body, const Vector3 &p_torque) override;
	void _body_add_constant_central_force(const RID &p_body, const Vector3 &p_force) override;
	void _body_add_constant_force(const RID &p_body, const Vector3 &p_force, const Vector3 &p_position) override;
	void _body_add_constant_torque(const RID &p_body, const Vector3 &p_torque) override;
	void _body_set_constant_force(const RID &p_body, const Vector3 &p_force) override;
	Vector3 _body_get_constant_force(const RID &p_body) const override;
	void _body_set_constant_torque(const RID &p_body, const Vector3 &p_torque) override;
	Vector3 _body_get_constant_torque(const RID &p_body) const override;
	void _body_set_axis_velocity(const RID &p_body, const Vector3 &p_axis_velocity) override;
	void _body_set_axis_lock(const RID &p_body, PhysicsServer3D::BodyAxis p_axis, bool p_lock) override;
	bool _body_is_axis_locked(const RID &p_body, PhysicsServer3D::BodyAxis p_axis) const override;
	void _body_add_collision_exception(const RID &p_body, const RID &p_excepted_body) override;
	void _body_remove_collision_exception(const RID &p_body, const RID &p_excepted_body) override;
	TypedArray<RID> _body_get_collision_exceptions(const RID &p_body) const override;
	void _body_set_max_contacts_reported(const RID &p_body, int32_t p_amount) override;
	int32_t _body_get_max_contacts_reported(const RID &p_body) const override;
	void _body_set_contacts_reported_depth_threshold(const RID &p_body, float p_threshold) override;
	float _body_get_contacts_reported_depth_threshold(const RID &p_body) const override;
	void _body_set_omit_force_integration(const RID &p_body, bool p_enable) override;
	bool _body_is_omitting_force_integration(const RID &p_body) const override;
	void _body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) override;
	void _body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_userdata) override;
	void _body_set_ray_pickable(const RID &p_body, bool p_enable) override;
	bool _body_test_motion(const RID &p_body, const Transform3D &p_from, const Vector3 &p_motion, float p_margin, int32_t p_max_collisions, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer3DExtensionMotionResult *p_result) const override;
	PhysicsDirectBodyState3D *_body_get_direct_state(const RID &p_body) override;

	// === Soft Body API (stubs) ===
	RID _soft_body_create() override;
	void _soft_body_update_rendering_server(const RID &p_body, PhysicsServer3DRenderingServerHandler *p_rendering_server_handler) override;
	void _soft_body_set_space(const RID &p_body, const RID &p_space) override;
	RID _soft_body_get_space(const RID &p_body) const override;
	void _soft_body_set_ray_pickable(const RID &p_body, bool p_enable) override;
	void _soft_body_set_collision_layer(const RID &p_body, uint32_t p_layer) override;
	uint32_t _soft_body_get_collision_layer(const RID &p_body) const override;
	void _soft_body_set_collision_mask(const RID &p_body, uint32_t p_mask) override;
	uint32_t _soft_body_get_collision_mask(const RID &p_body) const override;
	void _soft_body_add_collision_exception(const RID &p_body, const RID &p_body_b) override;
	void _soft_body_remove_collision_exception(const RID &p_body, const RID &p_body_b) override;
	TypedArray<RID> _soft_body_get_collision_exceptions(const RID &p_body) const override;
	void _soft_body_set_state(const RID &p_body, PhysicsServer3D::BodyState p_state, const Variant &p_variant) override;
	Variant _soft_body_get_state(const RID &p_body, PhysicsServer3D::BodyState p_state) const override;
	void _soft_body_set_transform(const RID &p_body, const Transform3D &p_transform) override;
	void _soft_body_set_simulation_precision(const RID &p_body, int32_t p_simulation_precision) override;
	int32_t _soft_body_get_simulation_precision(const RID &p_body) const override;
	void _soft_body_set_total_mass(const RID &p_body, float p_total_mass) override;
	float _soft_body_get_total_mass(const RID &p_body) const override;
	void _soft_body_set_linear_stiffness(const RID &p_body, float p_linear_stiffness) override;
	float _soft_body_get_linear_stiffness(const RID &p_body) const override;
	void _soft_body_set_pressure_coefficient(const RID &p_body, float p_pressure_coefficient) override;
	float _soft_body_get_pressure_coefficient(const RID &p_body) const override;
	void _soft_body_set_damping_coefficient(const RID &p_body, float p_damping_coefficient) override;
	float _soft_body_get_damping_coefficient(const RID &p_body) const override;
	void _soft_body_set_drag_coefficient(const RID &p_body, float p_drag_coefficient) override;
	float _soft_body_get_drag_coefficient(const RID &p_body) const override;
	void _soft_body_set_mesh(const RID &p_body, const RID &p_mesh) override;
	AABB _soft_body_get_bounds(const RID &p_body) const override;
	void _soft_body_move_point(const RID &p_body, int32_t p_point_index, const Vector3 &p_global_position) override;
	Vector3 _soft_body_get_point_global_position(const RID &p_body, int32_t p_point_index) const override;
	void _soft_body_remove_all_pinned_points(const RID &p_body) override;
	void _soft_body_pin_point(const RID &p_body, int32_t p_point_index, bool p_pin) override;
	bool _soft_body_is_point_pinned(const RID &p_body, int32_t p_point_index) const override;

	// === Joint API ===
	RID _joint_create() override;
	void _joint_clear(const RID &p_joint) override;
	void _joint_make_pin(const RID &p_joint, const RID &p_body_A, const Vector3 &p_local_A, const RID &p_body_B, const Vector3 &p_local_B) override;
	void _pin_joint_set_param(const RID &p_joint, PhysicsServer3D::PinJointParam p_param, float p_value) override;
	float _pin_joint_get_param(const RID &p_joint, PhysicsServer3D::PinJointParam p_param) const override;
	void _pin_joint_set_local_a(const RID &p_joint, const Vector3 &p_local_A) override;
	Vector3 _pin_joint_get_local_a(const RID &p_joint) const override;
	void _pin_joint_set_local_b(const RID &p_joint, const Vector3 &p_local_B) override;
	Vector3 _pin_joint_get_local_b(const RID &p_joint) const override;
	void _joint_make_hinge(const RID &p_joint, const RID &p_body_A, const Transform3D &p_hinge_A, const RID &p_body_B, const Transform3D &p_hinge_B) override;
	void _joint_make_hinge_simple(const RID &p_joint, const RID &p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, const RID &p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) override;
	void _hinge_joint_set_param(const RID &p_joint, PhysicsServer3D::HingeJointParam p_param, float p_value) override;
	float _hinge_joint_get_param(const RID &p_joint, PhysicsServer3D::HingeJointParam p_param) const override;
	void _hinge_joint_set_flag(const RID &p_joint, PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) override;
	bool _hinge_joint_get_flag(const RID &p_joint, PhysicsServer3D::HingeJointFlag p_flag) const override;
	void _joint_make_slider(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) override;
	void _slider_joint_set_param(const RID &p_joint, PhysicsServer3D::SliderJointParam p_param, float p_value) override;
	float _slider_joint_get_param(const RID &p_joint, PhysicsServer3D::SliderJointParam p_param) const override;
	void _joint_make_cone_twist(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) override;
	void _cone_twist_joint_set_param(const RID &p_joint, PhysicsServer3D::ConeTwistJointParam p_param, float p_value) override;
	float _cone_twist_joint_get_param(const RID &p_joint, PhysicsServer3D::ConeTwistJointParam p_param) const override;
	void _joint_make_generic_6dof(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) override;
	void _generic_6dof_joint_set_param(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, float p_value) override;
	float _generic_6dof_joint_get_param(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const override;
	void _generic_6dof_joint_set_flag(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable) override;
	bool _generic_6dof_joint_get_flag(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const override;
	PhysicsServer3D::JointType _joint_get_type(const RID &p_joint) const override;
	void _joint_set_solver_priority(const RID &p_joint, int32_t p_priority) override;
	int32_t _joint_get_solver_priority(const RID &p_joint) const override;
	void _joint_disable_collisions_between_bodies(const RID &p_joint, bool p_disable) override;
	bool _joint_is_disabled_collisions_between_bodies(const RID &p_joint) const override;

	// === General ===
	void _free_rid(const RID &p_rid) override;
	void _set_active(bool p_active) override;
	void _init() override;
	void _step(float p_step) override;
	void _sync() override;
	void _flush_queries() override;
	void _end_sync() override;
	void _finish() override;
	bool _is_flushing_queries() const override;
	int32_t _get_process_info(PhysicsServer3D::ProcessInfo p_process_info) override;
};
