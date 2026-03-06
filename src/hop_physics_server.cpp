#include "hop_physics_server.h"
#include "hop_body_state.h"
#include "hop_space_state.h"
#include "hop_conversions.h"

#include <algorithm>

// ============================================================
// Shape API
// ============================================================

static RID make_shape(HopRIDOwner<HopShapeData> &owner, PhysicsServer3D::ShapeType type) {
	auto *s = new HopShapeData();
	s->type = type;
	return owner.make_rid(s);
}

RID HopPhysicsServer::_world_boundary_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_WORLD_BOUNDARY); }
RID HopPhysicsServer::_separation_ray_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_SEPARATION_RAY); }
RID HopPhysicsServer::_sphere_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_SPHERE); }
RID HopPhysicsServer::_box_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_BOX); }
RID HopPhysicsServer::_capsule_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_CAPSULE); }
RID HopPhysicsServer::_cylinder_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_CYLINDER); }
RID HopPhysicsServer::_convex_polygon_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_CONVEX_POLYGON); }
RID HopPhysicsServer::_concave_polygon_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_CONCAVE_POLYGON); }
RID HopPhysicsServer::_heightmap_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_HEIGHTMAP); }
RID HopPhysicsServer::_custom_shape_create() { return make_shape(shape_owner, PhysicsServer3D::SHAPE_CUSTOM); }

void HopPhysicsServer::_shape_set_data(const RID &p_shape, const Variant &p_data) {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	if (!s) return;
	s->set_data(s->type, p_data);
}

void HopPhysicsServer::_shape_set_custom_solver_bias(const RID &p_shape, float p_bias) {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	if (s) s->custom_solver_bias = p_bias;
}

void HopPhysicsServer::_shape_set_margin(const RID &p_shape, float p_margin) {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	if (s) s->margin = p_margin;
}

float HopPhysicsServer::_shape_get_margin(const RID &p_shape) const {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	return s ? s->margin : 0.0f;
}

PhysicsServer3D::ShapeType HopPhysicsServer::_shape_get_type(const RID &p_shape) const {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	return s ? s->type : PhysicsServer3D::SHAPE_CUSTOM;
}

Variant HopPhysicsServer::_shape_get_data(const RID &p_shape) const {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	return s ? s->data : Variant();
}

float HopPhysicsServer::_shape_get_custom_solver_bias(const RID &p_shape) const {
	HopShapeData *s = shape_owner.get_or_null(p_shape);
	return s ? s->custom_solver_bias : 0.0f;
}

// ============================================================
// Space API
// ============================================================

RID HopPhysicsServer::_space_create() {
	auto *space = new HopSpaceData();
	RID rid = space_owner.make_rid(space);
	space->self_rid = rid;
	return rid;
}

void HopPhysicsServer::_space_set_active(const RID &p_space, bool p_active) {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	if (space) space->active = p_active;
}

bool HopPhysicsServer::_space_is_active(const RID &p_space) const {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	return space ? space->active : false;
}

void HopPhysicsServer::_space_set_param(const RID &p_space, PhysicsServer3D::SpaceParameter p_param, float p_value) {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	if (!space) return;
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: space->contact_recycle_radius = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: space->contact_max_separation = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: space->contact_max_allowed_penetration = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: space->contact_default_bias = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: space->body_linear_velocity_sleep_threshold = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: space->body_angular_velocity_sleep_threshold = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: space->body_time_to_sleep = p_value; break;
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: space->solver_iterations = p_value; break;
		default: break;
	}
}

float HopPhysicsServer::_space_get_param(const RID &p_space, PhysicsServer3D::SpaceParameter p_param) const {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	if (!space) return 0.0f;
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: return space->contact_recycle_radius;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: return space->contact_max_separation;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: return space->contact_max_allowed_penetration;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: return space->contact_default_bias;
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: return space->body_linear_velocity_sleep_threshold;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: return space->body_angular_velocity_sleep_threshold;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: return space->body_time_to_sleep;
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: return space->solver_iterations;
		default: return 0.0f;
	}
}

PhysicsDirectSpaceState3D *HopPhysicsServer::_space_get_direct_state(const RID &p_space) {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	if (!space) return nullptr;
	if (!space->direct_state) {
		space->direct_state = memnew(HopDirectSpaceState);
		space->direct_state->space = space;
		space->direct_state->server = this;
	}
	return space->direct_state;
}

void HopPhysicsServer::_space_set_debug_contacts(const RID &p_space, int32_t p_max_contacts) {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	if (space) space->max_debug_contacts = p_max_contacts;
}

PackedVector3Array HopPhysicsServer::_space_get_contacts(const RID &p_space) const {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	return space ? space->debug_contacts : PackedVector3Array();
}

int32_t HopPhysicsServer::_space_get_contact_count(const RID &p_space) const {
	HopSpaceData *space = space_owner.get_or_null(p_space);
	return space ? space->debug_contacts.size() : 0;
}

// ============================================================
// Area API
// ============================================================

RID HopPhysicsServer::_area_create() {
	auto *area = new HopAreaData();
	RID rid = area_owner.make_rid(area);
	area->self_rid = rid;
	return rid;
}

void HopPhysicsServer::_area_set_space(const RID &p_area, const RID &p_space) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->space_rid = p_space;
}

RID HopPhysicsServer::_area_get_space(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->space_rid : RID();
}

void HopPhysicsServer::_area_add_shape(const RID &p_area, const RID &p_shape, const Transform3D &p_transform, bool p_disabled) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	area->shapes.push_back({ p_shape, p_transform, p_disabled });
}

void HopPhysicsServer::_area_set_shape(const RID &p_area, int32_t p_shape_idx, const RID &p_shape) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].shape_rid = p_shape;
}

void HopPhysicsServer::_area_set_shape_transform(const RID &p_area, int32_t p_shape_idx, const Transform3D &p_transform) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].local_xform = p_transform;
}

void HopPhysicsServer::_area_set_shape_disabled(const RID &p_area, int32_t p_shape_idx, bool p_disabled) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].disabled = p_disabled;
}

int32_t HopPhysicsServer::_area_get_shape_count(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? (int32_t)area->shapes.size() : 0;
}

RID HopPhysicsServer::_area_get_shape(const RID &p_area, int32_t p_shape_idx) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return RID();
	return area->shapes[p_shape_idx].shape_rid;
}

Transform3D HopPhysicsServer::_area_get_shape_transform(const RID &p_area, int32_t p_shape_idx) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return Transform3D();
	return area->shapes[p_shape_idx].local_xform;
}

void HopPhysicsServer::_area_remove_shape(const RID &p_area, int32_t p_shape_idx) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes.erase(area->shapes.begin() + p_shape_idx);
}

void HopPhysicsServer::_area_clear_shapes(const RID &p_area) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->shapes.clear();
}

void HopPhysicsServer::_area_attach_object_instance_id(const RID &p_area, uint64_t p_id) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->object_instance_id = p_id;
}

uint64_t HopPhysicsServer::_area_get_object_instance_id(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->object_instance_id : 0;
}

void HopPhysicsServer::_area_set_param(const RID &p_area, PhysicsServer3D::AreaParameter p_param, const Variant &p_value) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: area->gravity = p_value; break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: area->gravity_direction = p_value; break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT: area->gravity_is_point = p_value; break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: area->gravity_point_distance = p_value; break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: area->linear_damp_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value; break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: area->linear_damp = p_value; break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: area->angular_damp_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value; break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: area->angular_damp = p_value; break;
		case PhysicsServer3D::AREA_PARAM_PRIORITY: area->priority = p_value; break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE: area->space_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value; break;
		default: break;
	}
}

void HopPhysicsServer::_area_set_transform(const RID &p_area, const Transform3D &p_transform) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->transform = p_transform;
}

Variant HopPhysicsServer::_area_get_param(const RID &p_area, PhysicsServer3D::AreaParameter p_param) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return Variant();
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: return area->gravity;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: return area->gravity_direction;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT: return area->gravity_is_point;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: return area->gravity_point_distance;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: return (int)area->linear_damp_override_mode;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: return area->linear_damp;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: return (int)area->angular_damp_override_mode;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: return area->angular_damp;
		case PhysicsServer3D::AREA_PARAM_PRIORITY: return area->priority;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE: return (int)area->space_override_mode;
		default: return Variant();
	}
}

Transform3D HopPhysicsServer::_area_get_transform(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->transform : Transform3D();
}

void HopPhysicsServer::_area_set_collision_layer(const RID &p_area, uint32_t p_layer) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->collision_layer = p_layer;
}

uint32_t HopPhysicsServer::_area_get_collision_layer(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->collision_layer : 0;
}

void HopPhysicsServer::_area_set_collision_mask(const RID &p_area, uint32_t p_mask) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->collision_mask = p_mask;
}

uint32_t HopPhysicsServer::_area_get_collision_mask(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->collision_mask : 0;
}

void HopPhysicsServer::_area_set_monitorable(const RID &p_area, bool p_monitorable) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->monitorable = p_monitorable;
}

void HopPhysicsServer::_area_set_ray_pickable(const RID &p_area, bool p_enable) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->ray_pickable = p_enable;
}

void HopPhysicsServer::_area_set_monitor_callback(const RID &p_area, const Callable &p_callback) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->monitor_callback = p_callback;
}

void HopPhysicsServer::_area_set_area_monitor_callback(const RID &p_area, const Callable &p_callback) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (area) area->area_monitor_callback = p_callback;
}

// ============================================================
// Body API
// ============================================================

RID HopPhysicsServer::_body_create() {
	auto *body = new HopBodyData();
	RID rid = body_owner.make_rid(body);
	body->self_rid = rid;
	body->create_hop_solid();
	return rid;
}

void HopPhysicsServer::rebuild_body_shapes(HopBodyData *body) {
	if (!body || !body->hop_solid) return;
	body->hop_solid->remove_all_shapes();

	for (auto &entry : body->shapes) {
		if (entry.disabled) continue;

		HopShapeData *sd = shape_owner.get_or_null(entry.shape_rid);
		if (!sd) continue;

		auto hs = sd->make_hop_shape(entry.local_xform);
		if (hs) {
			entry.hop_shape = hs;
			body->hop_solid->add_shape(hs);
		}
	}

	// add_shape calls activate() — re-deactivate static bodies
	if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) {
		body->hop_solid->deactivate();
	}
}

void HopPhysicsServer::add_body_to_space(HopBodyData *body, HopSpaceData *space) {
	if (!body->hop_solid) {
		body->create_hop_solid();
	}
	body->sync_to_hop();
	rebuild_body_shapes(body);
	space->simulator->add_solid(body->hop_solid);

	// Deactivate static bodies AFTER shapes/position are set
	// (add_shape and set_position both call activate())
	if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) {
		body->hop_solid->deactivate();
	}
}

void HopPhysicsServer::remove_body_from_space(HopBodyData *body) {
	if (!body->hop_solid) return;
	HopSpaceData *space = space_owner.get_or_null(body->space_rid);
	if (space) {
		space->simulator->remove_solid(body->hop_solid);
	}
}

void HopPhysicsServer::_body_set_space(const RID &p_body, const RID &p_space) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;

	// Remove from old space
	if (body->space_rid.is_valid()) {
		remove_body_from_space(body);
	}

	body->space_rid = p_space;

	// Add to new space
	if (p_space.is_valid()) {
		HopSpaceData *space = space_owner.get_or_null(p_space);
		if (space) {
			add_body_to_space(body, space);
		}
	}
}

RID HopPhysicsServer::_body_get_space(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->space_rid : RID();
}

void HopPhysicsServer::_body_set_mode(const RID &p_body, PhysicsServer3D::BodyMode p_mode) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->mode = p_mode;

	if (body->hop_solid) {
		if (body->is_static_or_kinematic()) {
			body->hop_solid->set_infinite_mass();
			body->hop_solid->set_coefficient_of_gravity(0.0f);
			body->hop_solid->set_velocity(hop::vec3<float>(0, 0, 0));
			body->hop_solid->deactivate();
		} else {
			body->hop_solid->set_mass(body->mass);
			body->hop_solid->set_coefficient_of_gravity(body->gravity_scale);
			body->hop_solid->activate();
		}
	}
}

PhysicsServer3D::BodyMode HopPhysicsServer::_body_get_mode(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->mode : PhysicsServer3D::BODY_MODE_STATIC;
}

void HopPhysicsServer::_body_add_shape(const RID &p_body, const RID &p_shape, const Transform3D &p_transform, bool p_disabled) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->shapes.push_back({ p_shape, p_transform, p_disabled, nullptr });
	if (body->space_rid.is_valid()) {
		rebuild_body_shapes(body);
	}
}

void HopPhysicsServer::_body_set_shape(const RID &p_body, int32_t p_shape_idx, const RID &p_shape) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return;
	body->shapes[p_shape_idx].shape_rid = p_shape;
	if (body->space_rid.is_valid()) rebuild_body_shapes(body);
}

void HopPhysicsServer::_body_set_shape_transform(const RID &p_body, int32_t p_shape_idx, const Transform3D &p_transform) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return;
	body->shapes[p_shape_idx].local_xform = p_transform;
	if (body->space_rid.is_valid()) rebuild_body_shapes(body);
}

void HopPhysicsServer::_body_set_shape_disabled(const RID &p_body, int32_t p_shape_idx, bool p_disabled) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return;
	body->shapes[p_shape_idx].disabled = p_disabled;
	if (body->space_rid.is_valid()) rebuild_body_shapes(body);
}

int32_t HopPhysicsServer::_body_get_shape_count(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? (int32_t)body->shapes.size() : 0;
}

RID HopPhysicsServer::_body_get_shape(const RID &p_body, int32_t p_shape_idx) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return RID();
	return body->shapes[p_shape_idx].shape_rid;
}

Transform3D HopPhysicsServer::_body_get_shape_transform(const RID &p_body, int32_t p_shape_idx) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return Transform3D();
	return body->shapes[p_shape_idx].local_xform;
}

void HopPhysicsServer::_body_remove_shape(const RID &p_body, int32_t p_shape_idx) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || p_shape_idx < 0 || p_shape_idx >= (int32_t)body->shapes.size()) return;
	body->shapes.erase(body->shapes.begin() + p_shape_idx);
	if (body->space_rid.is_valid()) rebuild_body_shapes(body);
}

void HopPhysicsServer::_body_clear_shapes(const RID &p_body) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->shapes.clear();
	if (body->hop_solid) body->hop_solid->remove_all_shapes();
}

void HopPhysicsServer::_body_attach_object_instance_id(const RID &p_body, uint64_t p_id) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->object_instance_id = p_id;
}

uint64_t HopPhysicsServer::_body_get_object_instance_id(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->object_instance_id : 0;
}

void HopPhysicsServer::_body_set_enable_continuous_collision_detection(const RID &p_body, bool p_enable) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->ccd_enabled = p_enable;
}

bool HopPhysicsServer::_body_is_continuous_collision_detection_enabled(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->ccd_enabled : false;
}

void HopPhysicsServer::_body_set_collision_layer(const RID &p_body, uint32_t p_layer) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->collision_layer = p_layer;
	if (body->hop_solid) body->hop_solid->set_collision_scope(p_layer);
}

uint32_t HopPhysicsServer::_body_get_collision_layer(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->collision_layer : 0;
}

void HopPhysicsServer::_body_set_collision_mask(const RID &p_body, uint32_t p_mask) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->collision_mask = p_mask;
	if (body->hop_solid) body->hop_solid->set_collide_with_scope(p_mask);
}

uint32_t HopPhysicsServer::_body_get_collision_mask(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->collision_mask : 0;
}

void HopPhysicsServer::_body_set_collision_priority(const RID &p_body, float p_priority) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->collision_priority = p_priority;
}

float HopPhysicsServer::_body_get_collision_priority(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->collision_priority : 1.0f;
}

void HopPhysicsServer::_body_set_user_flags(const RID &p_body, uint32_t p_flags) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->user_flags = p_flags;
}

uint32_t HopPhysicsServer::_body_get_user_flags(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->user_flags : 0;
}

void HopPhysicsServer::_body_set_param(const RID &p_body, PhysicsServer3D::BodyParameter p_param, const Variant &p_value) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE: {
			body->bounce = p_value;
			if (body->hop_solid) body->hop_solid->set_coefficient_of_restitution(body->bounce);
		} break;
		case PhysicsServer3D::BODY_PARAM_FRICTION: {
			body->friction = p_value;
			if (body->hop_solid) {
				body->hop_solid->set_coefficient_of_static_friction(body->friction);
				body->hop_solid->set_coefficient_of_dynamic_friction(body->friction);
			}
		} break;
		case PhysicsServer3D::BODY_PARAM_MASS: {
			body->mass = p_value;
			if (body->hop_solid && !body->is_static_or_kinematic()) {
				body->hop_solid->set_mass(body->mass);
			}
		} break;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: {
			body->gravity_scale = p_value;
			if (body->hop_solid) body->hop_solid->set_coefficient_of_gravity(body->gravity_scale);
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP_MODE: break; // stored but not used differently
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			body->linear_damp = p_value;
			if (body->hop_solid) body->hop_solid->set_coefficient_of_effective_drag(body->linear_damp);
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: body->angular_damp = p_value; break;
		default: break;
	}
}

Variant HopPhysicsServer::_body_get_param(const RID &p_body, PhysicsServer3D::BodyParameter p_param) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return Variant();
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE: return body->bounce;
		case PhysicsServer3D::BODY_PARAM_FRICTION: return body->friction;
		case PhysicsServer3D::BODY_PARAM_MASS: return body->mass;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: return body->gravity_scale;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: return body->linear_damp;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: return body->angular_damp;
		case PhysicsServer3D::BODY_PARAM_INERTIA: return Vector3(); // no rotation
		default: return Variant();
	}
}

void HopPhysicsServer::_body_reset_mass_properties(const RID &p_body) {
	// No-op: hop handles mass directly
}

void HopPhysicsServer::_body_set_state(const RID &p_body, PhysicsServer3D::BodyState p_state, const Variant &p_value) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: {
			body->transform = p_value;
			if (body->hop_solid) {
				body->hop_solid->set_position(to_hop(body->transform.origin));
				// set_position calls activate() — re-deactivate static bodies
				if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) {
					body->hop_solid->deactivate();
				}
			}
		} break;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: {
			body->linear_velocity = p_value;
			if (body->hop_solid) body->hop_solid->set_velocity(to_hop(body->linear_velocity));
		} break;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: {
			body->angular_velocity = p_value; // stored but no-op in hop
		} break;
		case PhysicsServer3D::BODY_STATE_SLEEPING: {
			body->sleeping = p_value;
			if (body->hop_solid) {
				if (body->sleeping) body->hop_solid->deactivate();
				else body->hop_solid->activate();
			}
		} break;
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP: {
			body->can_sleep = p_value;
		} break;
		default: break;
	}
}

Variant HopPhysicsServer::_body_get_state(const RID &p_body, PhysicsServer3D::BodyState p_state) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return Variant();
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: return body->transform;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: return body->linear_velocity;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: return body->angular_velocity;
		case PhysicsServer3D::BODY_STATE_SLEEPING: return body->sleeping;
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP: return body->can_sleep;
		default: return Variant();
	}
}

void HopPhysicsServer::_body_apply_central_impulse(const RID &p_body, const Vector3 &p_impulse) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || !body->hop_solid || body->is_static_or_kinematic()) return;
	// impulse = mass * delta_v => delta_v = impulse / mass
	float m = body->mass;
	if (m > 0.0f) {
		Vector3 dv = p_impulse / m;
		body->linear_velocity += dv;
		body->hop_solid->set_velocity(to_hop(body->linear_velocity));
	}
}

void HopPhysicsServer::_body_apply_impulse(const RID &p_body, const Vector3 &p_impulse, const Vector3 &p_position) {
	// hop has no rotation, so position is ignored — just apply central impulse
	_body_apply_central_impulse(p_body, p_impulse);
}

void HopPhysicsServer::_body_apply_torque_impulse(const RID &p_body, const Vector3 &p_impulse) {
	// No-op: hop has no rotation
}

void HopPhysicsServer::_body_apply_central_force(const RID &p_body, const Vector3 &p_force) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || !body->hop_solid || body->is_static_or_kinematic()) return;
	body->hop_solid->add_force(to_hop(p_force));
}

void HopPhysicsServer::_body_apply_force(const RID &p_body, const Vector3 &p_force, const Vector3 &p_position) {
	_body_apply_central_force(p_body, p_force);
}

void HopPhysicsServer::_body_apply_torque(const RID &p_body, const Vector3 &p_torque) {
	// No-op: hop has no rotation
}

void HopPhysicsServer::_body_add_constant_central_force(const RID &p_body, const Vector3 &p_force) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->constant_force += p_force;
}

void HopPhysicsServer::_body_add_constant_force(const RID &p_body, const Vector3 &p_force, const Vector3 &p_position) {
	_body_add_constant_central_force(p_body, p_force);
}

void HopPhysicsServer::_body_add_constant_torque(const RID &p_body, const Vector3 &p_torque) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->constant_torque += p_torque;
}

void HopPhysicsServer::_body_set_constant_force(const RID &p_body, const Vector3 &p_force) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->constant_force = p_force;
}

Vector3 HopPhysicsServer::_body_get_constant_force(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->constant_force : Vector3();
}

void HopPhysicsServer::_body_set_constant_torque(const RID &p_body, const Vector3 &p_torque) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->constant_torque = p_torque;
}

Vector3 HopPhysicsServer::_body_get_constant_torque(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->constant_torque : Vector3();
}

void HopPhysicsServer::_body_set_axis_velocity(const RID &p_body, const Vector3 &p_axis_velocity) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	Vector3 axis = p_axis_velocity.normalized();
	float proj = body->linear_velocity.dot(axis);
	body->linear_velocity += (p_axis_velocity.length() - proj) * axis;
	if (body->hop_solid) body->hop_solid->set_velocity(to_hop(body->linear_velocity));
}

void HopPhysicsServer::_body_set_axis_lock(const RID &p_body, PhysicsServer3D::BodyAxis p_axis, bool p_lock) {
	// No-op: hop doesn't support axis locking
}

bool HopPhysicsServer::_body_is_axis_locked(const RID &p_body, PhysicsServer3D::BodyAxis p_axis) const {
	return false;
}

void HopPhysicsServer::_body_add_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->collision_exceptions.push_back(p_excepted_body);
}

void HopPhysicsServer::_body_remove_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	auto &vec = body->collision_exceptions;
	vec.erase(std::remove(vec.begin(), vec.end(), p_excepted_body), vec.end());
}

TypedArray<RID> HopPhysicsServer::_body_get_collision_exceptions(const RID &p_body) const {
	TypedArray<RID> result;
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) {
		for (const RID &r : body->collision_exceptions) {
			result.push_back(r);
		}
	}
	return result;
}

void HopPhysicsServer::_body_set_max_contacts_reported(const RID &p_body, int32_t p_amount) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->max_contacts_reported = p_amount;
}

int32_t HopPhysicsServer::_body_get_max_contacts_reported(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->max_contacts_reported : 0;
}

void HopPhysicsServer::_body_set_contacts_reported_depth_threshold(const RID &p_body, float p_threshold) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->contacts_depth_threshold = p_threshold;
}

float HopPhysicsServer::_body_get_contacts_reported_depth_threshold(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->contacts_depth_threshold : 0.0f;
}

void HopPhysicsServer::_body_set_omit_force_integration(const RID &p_body, bool p_enable) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->omit_force_integration = p_enable;
}

bool HopPhysicsServer::_body_is_omitting_force_integration(const RID &p_body) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	return body ? body->omit_force_integration : false;
}

void HopPhysicsServer::_body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->state_sync_callback = p_callable;
}

void HopPhysicsServer::_body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_userdata) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->force_integration_callback = p_callable;
	body->force_integration_userdata = p_userdata;
}

void HopPhysicsServer::_body_set_ray_pickable(const RID &p_body, bool p_enable) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (body) body->ray_pickable = p_enable;
}

bool HopPhysicsServer::_body_test_motion(const RID &p_body, const Transform3D &p_from, const Vector3 &p_motion, float p_margin, int32_t p_max_collisions, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer3DExtensionMotionResult *p_result) const {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || !body->hop_solid) return false;

	HopSpaceData *space = space_owner.get_or_null(body->space_rid);
	if (!space) return false;

	// Save and set position for the test
	hop::vec3<float> orig_pos = body->hop_solid->get_position();
	body->hop_solid->set_position_direct(to_hop(p_from.origin));

	hop::segment<float> seg;
	seg.set_start_end(to_hop(p_from.origin), to_hop(p_from.origin + p_motion));

	hop::collision<float> result;
	space->simulator->trace_solid(result, body->hop_solid.get(), seg, body->collision_mask);

	// Restore position
	body->hop_solid->set_position_direct(orig_pos);

	if (result.time >= 1.0f) {
		// No collision
		if (p_result) {
			p_result->travel = p_motion;
			p_result->remainder = Vector3();
			p_result->collision_safe_fraction = 1.0f;
			p_result->collision_unsafe_fraction = 1.0f;
			p_result->collision_count = 0;
			p_result->collision_depth = 0.0f;
		}
		return false;
	}

	// Collision occurred
	if (p_result) {
		p_result->travel = p_motion * result.time;
		p_result->remainder = p_motion * (1.0f - result.time);
		p_result->collision_safe_fraction = result.time;
		p_result->collision_unsafe_fraction = result.time;
		p_result->collision_depth = 0.0f;
		p_result->collision_count = 1;

		auto &col = p_result->collisions[0];
		col.position = to_godot(result.point);
		col.normal = to_godot(result.normal);
		col.depth = 0.0f;
		col.local_shape = 0;

		if (result.collidee) {
			HopBodyData *other = static_cast<HopBodyData *>(result.collidee->get_user_data());
			if (other) {
				col.collider = other->self_rid;
				col.collider_id = ObjectID(other->object_instance_id);
				col.collider_shape = 0;
				col.collider_velocity = other->linear_velocity;
			}
		}
	}
	return true;
}

PhysicsDirectBodyState3D *HopPhysicsServer::_body_get_direct_state(const RID &p_body) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return nullptr;
	if (!body->direct_state) {
		body->direct_state = memnew(HopDirectBodyState);
		body->direct_state->body = body;
		body->direct_state->server = this;
	}
	return body->direct_state;
}

// ============================================================
// Soft Body API (all stubs)
// ============================================================

RID HopPhysicsServer::_soft_body_create() { return RID(); }
void HopPhysicsServer::_soft_body_update_rendering_server(const RID &p_body, PhysicsServer3DRenderingServerHandler *p_rendering_server_handler) {}
void HopPhysicsServer::_soft_body_set_space(const RID &p_body, const RID &p_space) {}
RID HopPhysicsServer::_soft_body_get_space(const RID &p_body) const { return RID(); }
void HopPhysicsServer::_soft_body_set_ray_pickable(const RID &p_body, bool p_enable) {}
void HopPhysicsServer::_soft_body_set_collision_layer(const RID &p_body, uint32_t p_layer) {}
uint32_t HopPhysicsServer::_soft_body_get_collision_layer(const RID &p_body) const { return 0; }
void HopPhysicsServer::_soft_body_set_collision_mask(const RID &p_body, uint32_t p_mask) {}
uint32_t HopPhysicsServer::_soft_body_get_collision_mask(const RID &p_body) const { return 0; }
void HopPhysicsServer::_soft_body_add_collision_exception(const RID &p_body, const RID &p_body_b) {}
void HopPhysicsServer::_soft_body_remove_collision_exception(const RID &p_body, const RID &p_body_b) {}
TypedArray<RID> HopPhysicsServer::_soft_body_get_collision_exceptions(const RID &p_body) const { return TypedArray<RID>(); }
void HopPhysicsServer::_soft_body_set_state(const RID &p_body, PhysicsServer3D::BodyState p_state, const Variant &p_variant) {}
Variant HopPhysicsServer::_soft_body_get_state(const RID &p_body, PhysicsServer3D::BodyState p_state) const { return Variant(); }
void HopPhysicsServer::_soft_body_set_transform(const RID &p_body, const Transform3D &p_transform) {}
void HopPhysicsServer::_soft_body_set_simulation_precision(const RID &p_body, int32_t p_simulation_precision) {}
int32_t HopPhysicsServer::_soft_body_get_simulation_precision(const RID &p_body) const { return 0; }
void HopPhysicsServer::_soft_body_set_total_mass(const RID &p_body, float p_total_mass) {}
float HopPhysicsServer::_soft_body_get_total_mass(const RID &p_body) const { return 0.0f; }
void HopPhysicsServer::_soft_body_set_linear_stiffness(const RID &p_body, float p_linear_stiffness) {}
float HopPhysicsServer::_soft_body_get_linear_stiffness(const RID &p_body) const { return 0.0f; }
void HopPhysicsServer::_soft_body_set_pressure_coefficient(const RID &p_body, float p_pressure_coefficient) {}
float HopPhysicsServer::_soft_body_get_pressure_coefficient(const RID &p_body) const { return 0.0f; }
void HopPhysicsServer::_soft_body_set_damping_coefficient(const RID &p_body, float p_damping_coefficient) {}
float HopPhysicsServer::_soft_body_get_damping_coefficient(const RID &p_body) const { return 0.0f; }
void HopPhysicsServer::_soft_body_set_drag_coefficient(const RID &p_body, float p_drag_coefficient) {}
float HopPhysicsServer::_soft_body_get_drag_coefficient(const RID &p_body) const { return 0.0f; }
void HopPhysicsServer::_soft_body_set_mesh(const RID &p_body, const RID &p_mesh) {}
AABB HopPhysicsServer::_soft_body_get_bounds(const RID &p_body) const { return AABB(); }
void HopPhysicsServer::_soft_body_move_point(const RID &p_body, int32_t p_point_index, const Vector3 &p_global_position) {}
Vector3 HopPhysicsServer::_soft_body_get_point_global_position(const RID &p_body, int32_t p_point_index) const { return Vector3(); }
void HopPhysicsServer::_soft_body_remove_all_pinned_points(const RID &p_body) {}
void HopPhysicsServer::_soft_body_pin_point(const RID &p_body, int32_t p_point_index, bool p_pin) {}
bool HopPhysicsServer::_soft_body_is_point_pinned(const RID &p_body, int32_t p_point_index) const { return false; }

// ============================================================
// Joint API
// ============================================================

RID HopPhysicsServer::_joint_create() {
	auto *j = new HopJointData();
	RID rid = joint_owner.make_rid(j);
	j->self_rid = rid;
	return rid;
}

void HopPhysicsServer::_joint_clear(const RID &p_joint) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;
	if (j->hop_constraint) {
		j->hop_constraint->destroy();
		j->hop_constraint.reset();
	}
	j->type = PhysicsServer3D::JOINT_TYPE_MAX;
}

void HopPhysicsServer::_joint_make_pin(const RID &p_joint, const RID &p_body_A, const Vector3 &p_local_A, const RID &p_body_B, const Vector3 &p_local_B) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;

	_joint_clear(p_joint);

	j->type = PhysicsServer3D::JOINT_TYPE_PIN;
	j->body_a = p_body_A;
	j->body_b = p_body_B;
	j->local_a = p_local_A;
	j->local_b = p_local_B;

	HopBodyData *ba = body_owner.get_or_null(p_body_A);
	HopBodyData *bb = body_owner.get_or_null(p_body_B);
	if (!ba || !bb || !ba->hop_solid || !bb->hop_solid) return;

	j->hop_constraint = std::make_shared<hop::constraint<float>>(ba->hop_solid, bb->hop_solid);
	j->hop_constraint->set_spring_constant(100.0f);
	j->hop_constraint->set_damping_constant(10.0f);
	j->hop_constraint->set_distance_threshold(0.0f);

	HopSpaceData *space = space_owner.get_or_null(ba->space_rid);
	if (space) {
		space->simulator->add_constraint(j->hop_constraint);
	}
}

void HopPhysicsServer::_pin_joint_set_param(const RID &p_joint, PhysicsServer3D::PinJointParam p_param, float p_value) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: j->pin_bias = p_value; break;
		case PhysicsServer3D::PIN_JOINT_DAMPING: j->pin_damping = p_value; break;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: j->pin_impulse_clamp = p_value; break;
		default: break;
	}
}

float HopPhysicsServer::_pin_joint_get_param(const RID &p_joint, PhysicsServer3D::PinJointParam p_param) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return 0.0f;
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: return j->pin_bias;
		case PhysicsServer3D::PIN_JOINT_DAMPING: return j->pin_damping;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: return j->pin_impulse_clamp;
		default: return 0.0f;
	}
}

void HopPhysicsServer::_pin_joint_set_local_a(const RID &p_joint, const Vector3 &p_local_A) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (j) j->local_a = p_local_A;
}

Vector3 HopPhysicsServer::_pin_joint_get_local_a(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->local_a : Vector3();
}

void HopPhysicsServer::_pin_joint_set_local_b(const RID &p_joint, const Vector3 &p_local_B) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (j) j->local_b = p_local_B;
}

Vector3 HopPhysicsServer::_pin_joint_get_local_b(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->local_b : Vector3();
}

// Hinge, slider, cone twist, 6DOF — all stubs
void HopPhysicsServer::_joint_make_hinge(const RID &p_joint, const RID &p_body_A, const Transform3D &p_hinge_A, const RID &p_body_B, const Transform3D &p_hinge_B) {}
void HopPhysicsServer::_joint_make_hinge_simple(const RID &p_joint, const RID &p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, const RID &p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) {}
void HopPhysicsServer::_hinge_joint_set_param(const RID &p_joint, PhysicsServer3D::HingeJointParam p_param, float p_value) {}
float HopPhysicsServer::_hinge_joint_get_param(const RID &p_joint, PhysicsServer3D::HingeJointParam p_param) const { return 0.0f; }
void HopPhysicsServer::_hinge_joint_set_flag(const RID &p_joint, PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) {}
bool HopPhysicsServer::_hinge_joint_get_flag(const RID &p_joint, PhysicsServer3D::HingeJointFlag p_flag) const { return false; }
void HopPhysicsServer::_joint_make_slider(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) {}
void HopPhysicsServer::_slider_joint_set_param(const RID &p_joint, PhysicsServer3D::SliderJointParam p_param, float p_value) {}
float HopPhysicsServer::_slider_joint_get_param(const RID &p_joint, PhysicsServer3D::SliderJointParam p_param) const { return 0.0f; }
void HopPhysicsServer::_joint_make_cone_twist(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) {}
void HopPhysicsServer::_cone_twist_joint_set_param(const RID &p_joint, PhysicsServer3D::ConeTwistJointParam p_param, float p_value) {}
float HopPhysicsServer::_cone_twist_joint_get_param(const RID &p_joint, PhysicsServer3D::ConeTwistJointParam p_param) const { return 0.0f; }
void HopPhysicsServer::_joint_make_generic_6dof(const RID &p_joint, const RID &p_body_A, const Transform3D &p_local_ref_A, const RID &p_body_B, const Transform3D &p_local_ref_B) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;

	_joint_clear(p_joint);

	j->type = PhysicsServer3D::JOINT_TYPE_6DOF;
	j->body_a = p_body_A;
	j->body_b = p_body_B;

	HopBodyData *ba = body_owner.get_or_null(p_body_A);
	HopBodyData *bb = body_owner.get_or_null(p_body_B);
	if (!ba || !bb || !ba->hop_solid || !bb->hop_solid) return;

	j->hop_constraint = std::make_shared<hop::constraint<float>>(ba->hop_solid, bb->hop_solid);
	j->hop_constraint->set_spring_constant(j->linear_spring_stiffness);
	j->hop_constraint->set_damping_constant(j->linear_spring_damping);
	j->hop_constraint->set_distance_threshold(j->linear_spring_equilibrium);

	HopSpaceData *space = space_owner.get_or_null(ba->space_rid);
	if (space) {
		space->simulator->add_constraint(j->hop_constraint);
	}
}

void HopPhysicsServer::_generic_6dof_joint_set_param(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, float p_value) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;

	// Map linear spring params (any axis) to hop constraint
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			j->linear_spring_stiffness = p_value;
			if (j->hop_constraint) j->hop_constraint->set_spring_constant(p_value);
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			j->linear_spring_damping = p_value;
			if (j->hop_constraint) j->hop_constraint->set_damping_constant(p_value);
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			j->linear_spring_equilibrium = p_value;
			if (j->hop_constraint) j->hop_constraint->set_distance_threshold(p_value);
			break;
		default:
			break;
	}
}

float HopPhysicsServer::_generic_6dof_joint_get_param(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return 0.0f;
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS: return j->linear_spring_stiffness;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING: return j->linear_spring_damping;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT: return j->linear_spring_equilibrium;
		default: return 0.0f;
	}
}

void HopPhysicsServer::_generic_6dof_joint_set_flag(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;
	if (p_flag == PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING) {
		j->linear_spring_enabled = p_enable;
	}
}

bool HopPhysicsServer::_generic_6dof_joint_get_flag(const RID &p_joint, Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return false;
	if (p_flag == PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING) return j->linear_spring_enabled;
	return false;
}

PhysicsServer3D::JointType HopPhysicsServer::_joint_get_type(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->type : PhysicsServer3D::JOINT_TYPE_MAX;
}

void HopPhysicsServer::_joint_set_solver_priority(const RID &p_joint, int32_t p_priority) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (j) j->solver_priority = p_priority;
}

int32_t HopPhysicsServer::_joint_get_solver_priority(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->solver_priority : 1;
}

void HopPhysicsServer::_joint_disable_collisions_between_bodies(const RID &p_joint, bool p_disable) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (j) j->disable_collisions = p_disable;
}

bool HopPhysicsServer::_joint_is_disabled_collisions_between_bodies(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->disable_collisions : false;
}

// ============================================================
// General / Stepping
// ============================================================

void HopPhysicsServer::_free_rid(const RID &p_rid) {
	if (shape_owner.owns(p_rid)) {
		HopShapeData *s = shape_owner.get_or_null(p_rid);
		shape_owner.free(p_rid);
		delete s;
	} else if (body_owner.owns(p_rid)) {
		HopBodyData *body = body_owner.get_or_null(p_rid);
		remove_body_from_space(body);
		if (body->direct_state) {
			body->direct_state->body = nullptr;
			body->direct_state->server = nullptr;
			memdelete(body->direct_state);
			body->direct_state = nullptr;
		}
		body_owner.free(p_rid);
		delete body;
	} else if (area_owner.owns(p_rid)) {
		HopAreaData *area = area_owner.get_or_null(p_rid);
		area_owner.free(p_rid);
		delete area;
	} else if (space_owner.owns(p_rid)) {
		HopSpaceData *space = space_owner.get_or_null(p_rid);
		if (space->direct_state) {
			space->direct_state->space = nullptr;
			space->direct_state->server = nullptr;
			memdelete(space->direct_state);
			space->direct_state = nullptr;
		}
		space_owner.free(p_rid);
		delete space;
	} else if (joint_owner.owns(p_rid)) {
		HopJointData *j = joint_owner.get_or_null(p_rid);
		if (j->hop_constraint) j->hop_constraint->destroy();
		joint_owner.free(p_rid);
		delete j;
	}
}

void HopPhysicsServer::_set_active(bool p_active) {
	active = p_active;
}

void HopPhysicsServer::_init() {
}

void HopPhysicsServer::_step(float p_step) {
	if (!active) return;
	last_step = p_step;

	int dt_ms = (int)(p_step * 1000.0f);
	if (dt_ms < 1) dt_ms = 1;

	// Clear contacts from previous step
	body_owner.for_each([](HopBodyData *body) {
		body->contacts.clear();
	});

	// Apply constant forces to all dynamic bodies before stepping
	body_owner.for_each([&](HopBodyData *body) {
		if (!body->hop_solid || body->is_static_or_kinematic()) return;
		if (body->constant_force.length_squared() > 0.0f) {
			body->hop_solid->add_force(to_hop(body->constant_force));
		}

		// Call force integration callback if set
		if (body->force_integration_callback.is_valid() && !body->omit_force_integration) {
			if (!body->direct_state) {
				body->direct_state = memnew(HopDirectBodyState);
				body->direct_state->body = body;
				body->direct_state->server = this;
			}
			body->force_integration_callback.call(body->direct_state, body->force_integration_userdata);
		}
	});

	// Step all active spaces
	space_owner.for_each([&](HopSpaceData *space) {
		if (!space->active) return;
		space->simulator->update(dt_ms, 0x3FFFFFFF | hop::simulator<float>::scope_report_collisions);
	});

	// Sync positions from hop to Godot
	body_owner.for_each([&](HopBodyData *body) {
		body->sync_from_hop();
	});
}

void HopPhysicsServer::_sync() {
}

void HopPhysicsServer::_flush_queries() {
	flushing_queries = true;

	// Call state_sync_callback for all bodies that have one
	body_owner.for_each([&](HopBodyData *body) {
		if (!body->state_sync_callback.is_valid()) return;
		if (body->is_static_or_kinematic() && body->mode != PhysicsServer3D::BODY_MODE_KINEMATIC) return;

		if (!body->direct_state) {
			body->direct_state = memnew(HopDirectBodyState);
			body->direct_state->body = body;
			body->direct_state->server = this;
		}
		body->state_sync_callback.call(body->direct_state);
	});

	flushing_queries = false;
}

void HopPhysicsServer::_end_sync() {
}

void HopPhysicsServer::_finish() {
	// Null out back-pointers on any remaining direct_state objects so
	// they are safe if Godot's ObjectDB cleans them up after us.
	body_owner.for_each([](HopBodyData *body) {
		if (body->direct_state) {
			body->direct_state->body = nullptr;
			body->direct_state->server = nullptr;
		}
	});
	space_owner.for_each([](HopSpaceData *space) {
		if (space->direct_state) {
			space->direct_state->space = nullptr;
			space->direct_state->server = nullptr;
		}
	});
}

bool HopPhysicsServer::_is_flushing_queries() const {
	return flushing_queries;
}

int32_t HopPhysicsServer::_get_process_info(PhysicsServer3D::ProcessInfo p_process_info) {
	return 0;
}
