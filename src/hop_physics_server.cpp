#include "hop_physics_server.h"
#include "hop_body_state.h"
#include "hop_space_state.h"
#include "hop_conversions.h"
#include <hop/math/mat3.h>
#include <hop/math/quat.h>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cstdlib>
#include <unordered_map>

// Phase 8: principal-axis inertia of the body's collision AABB treated as a solid
// box — I = mass/12 · (eᵧ²+e_z², e_x²+e_z², e_x²+eᵧ²) for full extents e. A sensible
// default for any shape so a dynamic RigidBody3D tumbles; the game can override via
// BODY_PARAM_INERTIA. A degenerate (zero-extent) bound yields zero inertia → no spin.
static hop::vec3<hop_scalar> aabb_box_inertia(const hop::aa_box<hop_scalar> &b, float mass) {
	hop::vec3<hop_scalar> e;
	hop::sub(e, b.maxs, b.mins);
	float ex = to_godot_float(e.x), ey = to_godot_float(e.y), ez = to_godot_float(e.z);
	float c = mass / 12.0f;
	return hop::vec3<hop_scalar>(to_hop_scalar(c * (ey * ey + ez * ez)),
	                             to_hop_scalar(c * (ex * ex + ez * ez)),
	                             to_hop_scalar(c * (ex * ex + ey * ey)));
}

// KINEMATIC bodies (the CharacterBody3D player) resolve contacts via sweep-and-slide
// so they slide along surfaces; dynamic and static bodies use the speculative solve
// (the space default, set in HopSpaceData). hop-godot is a generic backend with no
// "player" concept, so body mode is the only signal available — moving platforms are
// kinematic too, but contact_mode is moot for infinite-mass bodies, so this is safe.
static void apply_contact_mode(HopBodyData *body) {
	if (!body->hop_solid) return;
	body->hop_solid->set_contact_mode(
		body->mode == PhysicsServer3D::BODY_MODE_KINEMATIC
			? hop::contact_mode::sweep_slide
			: hop::contact_mode::speculative);
}

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
	if (!area) return;
	if (area->space_rid.is_valid()) remove_area_from_space(area);
	area->space_rid = p_space;
	if (p_space.is_valid()) {
		HopSpaceData *space = space_owner.get_or_null(p_space);
		if (space) add_area_to_space(area, space);
	}
}

RID HopPhysicsServer::_area_get_space(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->space_rid : RID();
}

void HopPhysicsServer::_area_add_shape(const RID &p_area, const RID &p_shape, const Transform3D &p_transform, bool p_disabled) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	area->shapes.push_back({ p_shape, p_transform, p_disabled });
	rebuild_area_shapes(area);
}

void HopPhysicsServer::_area_set_shape(const RID &p_area, int32_t p_shape_idx, const RID &p_shape) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].shape_rid = p_shape;
	rebuild_area_shapes(area);
}

void HopPhysicsServer::_area_set_shape_transform(const RID &p_area, int32_t p_shape_idx, const Transform3D &p_transform) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].local_xform = p_transform;
	rebuild_area_shapes(area);
}

void HopPhysicsServer::_area_set_shape_disabled(const RID &p_area, int32_t p_shape_idx, bool p_disabled) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area || p_shape_idx < 0 || p_shape_idx >= (int32_t)area->shapes.size()) return;
	area->shapes[p_shape_idx].disabled = p_disabled;
	rebuild_area_shapes(area);
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
	rebuild_area_shapes(area);
}

void HopPhysicsServer::_area_clear_shapes(const RID &p_area) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	area->shapes.clear();
	if (area->hop_solid) area->hop_solid->remove_all_shapes();
	mark_area_bvh_dirty(area);
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
	if (!area) {
		// Not an area RID — Godot addresses a space's DEFAULT area by the SPACE rid
		// (World3D pushes physics/3d/default_gravity through here at startup, and
		// again whenever the setting changes). Dropping it left the simulator on its
		// construction-time 9.81 no matter what the project asked for, so every
		// dynamic body fell at the wrong rate while CharacterBody3D movement — which
		// integrates its own gravity in script — looked correct.
		if (HopSpaceData *space = space_owner.get_or_null(p_area)) {
			switch (p_param) {
				case PhysicsServer3D::AREA_PARAM_GRAVITY:
					space->default_gravity = p_value;
					space->apply_default_gravity();
					break;
				case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
					space->default_gravity_direction = p_value;
					space->apply_default_gravity();
					break;
				default: break;  // the rest have no space-wide analogue in hop
			}
		}
		return;
	}
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
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE: area->wind_force_magnitude = p_value; break;
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION: area->wind_direction = p_value; break;
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR: area->wind_attenuation_factor = p_value; break;
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE: break; // node-path; not applicable
		default: break;
	}
}

void HopPhysicsServer::_area_set_transform(const RID &p_area, const Transform3D &p_transform) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	Transform3D old_transform = area->transform;
	area->transform = p_transform;
	// Position moves the persistent solid; a static rotation just updates its
	// orientation (no rebuild). Scale is baked into the geometry, so a scale
	// change requires a shape rebuild. The world bound shifts either way, so the
	// area broadphase must rebuild before its next query.
	if (area->hop_solid) {
		area->hop_solid->set_position(to_hop(area->transform.origin));
		if (!area->transform.basis.is_equal_approx(old_transform.basis)) {
			if (!area->transform.basis.get_scale().is_equal_approx(
			        old_transform.basis.get_scale())) {
				rebuild_area_shapes(area);
			}
			area->hop_solid->set_orientation(
			    to_hop_orientation(area->transform.basis));
		}
		mark_area_bvh_dirty(area);
	}
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
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE: return area->wind_force_magnitude;
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION: return area->wind_direction;
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR: return area->wind_attenuation_factor;
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE: return Variant();
		default: return Variant();
	}
}

Transform3D HopPhysicsServer::_area_get_transform(const RID &p_area) const {
	HopAreaData *area = area_owner.get_or_null(p_area);
	return area ? area->transform : Transform3D();
}

void HopPhysicsServer::_area_set_collision_layer(const RID &p_area, uint32_t p_layer) {
	HopAreaData *area = area_owner.get_or_null(p_area);
	if (!area) return;
	area->collision_layer = p_layer;
	if (area->hop_solid) {
		area->hop_solid->set_collision_scope(p_layer);
		area->hop_solid->set_trigger_scope(p_layer);
	}
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

// Flip BSP hull collision for every body already in the world, not just ones
// built from here on. Toggling has to be immediate and symmetric — the game
// replicates this flag, and a client whose collision lags the server's by even a
// map's worth of bodies mispredicts every step against the wrong geometry.
//
// Bodies are re-resolved from scratch (bsp_checked = 0) rather than remembered,
// so turning it back on picks up maps loaded while it was off.
void HopPhysicsServer::set_bsp_hulls_enabled(bool p_enabled) {
	if (bsp_hulls_enabled == p_enabled) return;
	bsp_hulls_enabled = p_enabled;

	int rebuilt = 0;
	int released = 0;
	body_owner.for_each([&](HopBodyData *body) {
		const bool was_carrier = body->bsp_checked == 2;
		if (!was_carrier && body->bsp_checked == 1 && !p_enabled) return;
		body->bsp_checked = 0;  // re-resolve; a map may have loaded while off
		if (!p_enabled) body->bsp_map.reset();  // stop pinning the tree while off
		if (!body->hop_solid || !body->space_rid.is_valid()) return;

		rebuild_body_shapes(body);
		// The solid's bound changes wholesale (a map-sized hull vs. per-brush
		// shapes), so re-register it with the broadphase as _body_set_mode does.
		HopSpaceData *space = space_owner.get_or_null(body->space_rid);
		if (space) {
			space->bvh_manager.remove_solid(body->hop_solid.get());
			space->bvh_manager.add_solid(body->hop_solid.get(),
				body->mode == PhysicsServer3D::BODY_MODE_STATIC);
		}
		if (body->bsp_checked == 2) {
			// Now tracing the hull, so this body's carrier shapes are dead weight.
			// Dropping the entry frees the shape and with it the built geometry
			// (verts, normals and BVH for a trimesh) — a hop::shape owns its
			// traceable, so there is nothing to work out about who else uses it.
			for (auto &entry : body->shapes) {
				if (entry.hop_shape) released++;
				entry.hop_shape.reset();
			}
		}
		if (was_carrier || body->bsp_checked == 2) rebuilt++;
	});

	// bsp_maps is a weak cache — entries expire on their own once the last body
	// holding the map is gone, so there is nothing to clear here.
	UtilityFunctions::print("[Hop] BSP hull collision ", p_enabled ? "ON" : "OFF",
		" — rebuilt ", rebuilt, " carrier body/bodies, released ", released,
		" unused shape geometr", released == 1 ? "y" : "ies");
}

// A body whose node carries goldsrc-godot's "bsp_model" metadata stands in for a
// real GoldSrc BSP hull: the trimesh/convex shapes on it exist so default Godot
// physics has something to collide with, but we can trace the compiler's own
// clipnode trees instead and get exact plane normals and true 18-unit steps.
//
// The blob itself lives once on the scene root under "bsp_data" (it runs to
// megabytes and a map has hundreds of brush entities), so walk up to find it and
// share one copy across every body of that map.
//
// Returns false whenever there is no hull to be had, leaving the carrier shapes
// to be built exactly as before — which is the entire fallback story.
std::unique_ptr<HopBspTraceable<hop_scalar>> HopPhysicsServer::try_build_bsp_hull(HopBodyData *body) {
	if (!bsp_hulls_enabled) return nullptr;
	if (body->bsp_checked == 1) return nullptr;  // known not to be a carrier

	body->bsp_checked = 1;
	if (body->object_instance_id == 0) return nullptr;
	Node *node = Object::cast_to<Node>(
		UtilityFunctions::instance_from_id((int64_t)body->object_instance_id));
	if (!node || !node->has_meta("bsp_model")) return nullptr;

	Node *root = node;
	while (root && !root->has_meta("bsp_data")) root = root->get_parent();
	if (!root) return nullptr;

	const uint64_t key = root->get_instance_id();
	std::shared_ptr<hopbsp::map_data> map;
	auto it = bsp_maps.find(key);
	if (it != bsp_maps.end()) map = it->second.lock();
	if (!map) {
		PackedByteArray blob = root->get_meta("bsp_data");
		if (blob.is_empty()) return nullptr;
		map = std::make_shared<hopbsp::map_data>();
		if (!map->load(blob.ptr(), (size_t)blob.size())) return nullptr;
		bsp_maps[key] = map;
		// Once per map: the tree is loaded once and shared by every body on it.
		UtilityFunctions::print("[Hop] BSP hull collision active for ", root->get_name(),
			" (", (int64_t)blob.size(), " byte tree, shared by all its bodies)");
	}

	auto traceable = std::make_unique<HopBspTraceable<hop_scalar>>();
	const int model = (int)node->get_meta("bsp_model");
	// No default: the scale is half the contract (it converts every query into the
	// map's units). A missing one means the tagging side and this side disagree,
	// and guessing WizardWars' 0.025 would silently trace a map at the wrong size.
	if (!node->has_meta("bsp_scale")) return nullptr;
	const float scale = (float)node->get_meta("bsp_scale");
	const int blocking = (int)node->get_meta("bsp_blocking", hopbsp::BLOCK_SOLID);
	if (!traceable->build(map, model, scalar_from_float<hop_scalar>(scale), blocking)) return nullptr;

	body->bsp_map = map;    // pin the shared tree for as long as this body lives
	body->bsp_checked = 2;  // a carrier; the traceable itself is the shape's business
	return traceable;
}

void HopPhysicsServer::rebuild_body_shapes(HopBodyData *body) {
	if (!body || !body->hop_solid) return;
	body->hop_solid->remove_all_shapes();

	// The game turns a brush entity intangible by disabling its CollisionShape3Ds
	// (EntityBase._apply_active_state does this for toggle-walls, opened doors and
	// anything out of PVS on a client). The hull stands in for those shapes, so it
	// has to answer to the same switch — otherwise a door that opened by going
	// non-solid stays solid under hulls.
	bool any_enabled = false;
	for (const auto &entry : body->shapes)
		if (!entry.disabled) { any_enabled = true; break; }

	// The hull replaces the carrier shapes outright — one traceable for the whole
	// model, at the body's own origin (BSP geometry is authored in exactly the
	// space the importer gives the node).
	if (any_enabled) {
		if (auto hull = try_build_bsp_hull(body)) {
			body->hop_solid->add_shape(
				std::make_shared<hop::shape<hop_scalar>>(std::move(hull)));
			if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) body->hop_solid->deactivate();
			return;
		}
	}

	// Static rotation: the body's ROTATION is carried as the solid's orientation
	// (set in sync_to_hop / _body_set_state), not baked. hop has no scale, so the
	// body's SCALE is still baked into the shape geometry here — composed in front
	// of each shape's local transform (world = body_rot · body_scale · local · p,
	// and body_rot is applied by the narrowphase). make_hop_shape further splits
	// the resulting local transform into rotation (→ shape.local_rotation) + scale
	// (baked), so a rotated CollisionShape within the body is honored too.
	Transform3D body_scale_only = scale_only_xform(body->transform.basis);

	for (auto &entry : body->shapes) {
		if (entry.disabled) continue;

		HopShapeData *sd = shape_owner.get_or_null(entry.shape_rid);
		if (!sd) continue;

		auto hs = sd->make_hop_shape(body_scale_only * entry.local_xform);
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
	// add_solid stamped the space default (speculative). The player (KINEMATIC
	// CharacterBody3D) resolves via sweep-and-slide instead; dynamic/static bodies
	// keep speculative. Re-applied here in case mode was set before the solid existed.
	apply_contact_mode(body);
	space->bvh_manager.add_solid(body->hop_solid.get(),
		body->mode == PhysicsServer3D::BODY_MODE_STATIC);

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
		space->bvh_manager.remove_solid(body->hop_solid.get());
		space->simulator->remove_solid(body->hop_solid);
	}
}

void HopPhysicsServer::ensure_area_solid(HopAreaData *area) {
	if (area->hop_solid) return;
	area->hop_solid = std::make_shared<hop::solid<hop_scalar>>();
	area->hop_solid->set_user_data(area);
	area->hop_solid->set_infinite_mass();
	// Sensor semantics: broadcasts on its layer (for future broad-phase use) and
	// tags that layer as a trigger, but listens to nothing and is never added to
	// the simulator/broadphase — so it can never block or be stepped.
	area->hop_solid->set_collision_scope(area->collision_layer);
	area->hop_solid->set_collide_with_scope(0);
	area->hop_solid->set_trigger_scope(area->collision_layer);
	area->hop_solid->set_position(to_hop(area->transform.origin));
	area->hop_solid->deactivate();
}

void HopPhysicsServer::rebuild_area_shapes(HopAreaData *area) {
	ensure_area_solid(area);
	area->hop_solid->remove_all_shapes();
	// Static rotation: bake the area's SCALE into the geometry (hop has no scale);
	// its ROTATION is carried as the solid's orientation below, like a body.
	Transform3D area_scale_only = scale_only_xform(area->transform.basis);
	for (auto &entry : area->shapes) {
		if (entry.disabled) {
			entry.hop_shape.reset();
			continue;
		}
		HopShapeData *sd = shape_owner.get_or_null(entry.shape_rid);
		if (!sd) continue;
		auto hs = sd->make_hop_shape(area_scale_only * entry.local_xform);
		if (hs) {
			entry.hop_shape = hs;
			area->hop_solid->add_shape(hs);
		}
	}
	// add_shape activates and set_position is unnecessary churn otherwise; keep the
	// position/orientation in sync and leave the solid inactive (it is never stepped).
	area->hop_solid->set_position(to_hop(area->transform.origin));
	area->hop_solid->set_orientation(to_hop_orientation(area->transform.basis));
	area->hop_solid->deactivate();
	// World bound changed — the area broadphase must rebuild before its next query.
	mark_area_bvh_dirty(area);
}

void HopPhysicsServer::mark_area_bvh_dirty(HopAreaData *area) {
	HopSpaceData *space = space_owner.get_or_null(area->space_rid);
	if (space) space->area_bvh.mark_dirty();
}

void HopPhysicsServer::add_area_to_space(HopAreaData *area, HopSpaceData *space) {
	rebuild_area_shapes(area); // ensures the solid exists and its shapes are built
	space->area_bvh.add_solid(area->hop_solid.get(), /*is_static=*/true);
}

void HopPhysicsServer::remove_area_from_space(HopAreaData *area) {
	if (!area->hop_solid) return;
	HopSpaceData *space = space_owner.get_or_null(area->space_rid);
	if (space) space->area_bvh.remove_solid(area->hop_solid.get());
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
		if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) {
			body->hop_solid->set_infinite_mass();
			body->hop_solid->set_coefficient_of_gravity(scalar_from_int<hop_scalar>(0));
			body->hop_solid->set_velocity(hop::vec3<hop_scalar>(scalar_from_int<hop_scalar>(0), scalar_from_int<hop_scalar>(0), scalar_from_int<hop_scalar>(0)));
			body->hop_solid->deactivate();
		} else if (body->mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
			// Kinematic bodies: infinite mass, no gravity, no self-propulsion in hop.
			// Stay active so dynamic bodies collide with them; position is driven by
			// Godot each frame via _body_set_state(TRANSFORM). hop must never integrate
			// their position, so we keep their hop velocity at zero (the _step loop
			// prescribes a per-frame velocity to sweep them, then snaps it back).
			body->hop_solid->set_infinite_mass();
			body->hop_solid->set_coefficient_of_gravity(scalar_from_int<hop_scalar>(0));
			body->hop_solid->set_velocity(hop::vec3<hop_scalar>(scalar_from_int<hop_scalar>(0), scalar_from_int<hop_scalar>(0), scalar_from_int<hop_scalar>(0)));
			body->hop_solid->activate();
		} else {
			body->hop_solid->set_mass(to_hop_scalar(body->mass));
			body->hop_solid->set_coefficient_of_gravity(to_hop_scalar(body->gravity_scale));
			body->hop_solid->activate();
			// Re-derive inertia for the new mode now, so toggling lock_rotation
			// (RIGID <-> RIGID_LINEAR) takes effect without waiting on a mass re-send.
			update_body_inertia(body);
		}

		// KINEMATIC (player) → sweep-and-slide; STATIC/RIGID → speculative solve.
		apply_contact_mode(body);

		// Re-register with the BVH manager — static/dynamic classification may have changed.
		if (body->space_rid.is_valid()) {
			HopSpaceData *space = space_owner.get_or_null(body->space_rid);
			if (space) {
				space->bvh_manager.remove_solid(body->hop_solid.get());
				space->bvh_manager.add_solid(body->hop_solid.get(),
					body->mode == PhysicsServer3D::BODY_MODE_STATIC);
			}
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
			if (body->hop_solid) body->hop_solid->set_coefficient_of_restitution(to_hop_scalar(body->bounce));
		} break;
		case PhysicsServer3D::BODY_PARAM_FRICTION: {
			body->friction = p_value;
			if (body->hop_solid) {
				body->hop_solid->set_coefficient_of_static_friction(to_hop_scalar(body->friction));
				body->hop_solid->set_coefficient_of_dynamic_friction(to_hop_scalar(body->friction));
			}
		} break;
		case PhysicsServer3D::BODY_PARAM_MASS: {
			body->mass = p_value;
			if (body->hop_solid && !body->is_static_or_kinematic()) {
				body->hop_solid->set_mass(to_hop_scalar(body->mass));
				update_body_inertia(body); // Phase 8: keep auto inertia in step with mass
			}
		} break;
		case PhysicsServer3D::BODY_PARAM_INERTIA: {
			// Explicit inertia from the game (Vector3 principal diagonal). A zero
			// vector means "let the engine compute it" (Godot's convention), so we
			// fall back to auto-compute; any non-zero value pins it (custom_inertia).
			Vector3 I = p_value;
			body->custom_inertia = I.length_squared() > 0.0f;
			if (body->hop_solid && !body->is_static_or_kinematic()) {
				if (body->custom_inertia)
					body->hop_solid->set_inertia(to_hop(I));
				else
					update_body_inertia(body);
			}
		} break;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: {
			body->gravity_scale = p_value;
			if (body->hop_solid) body->hop_solid->set_coefficient_of_gravity(to_hop_scalar(body->gravity_scale));
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP_MODE: break; // stored but not used differently
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			body->linear_damp = p_value;
			if (body->hop_solid) body->hop_solid->set_coefficient_of_effective_drag(to_hop_scalar(body->linear_damp));
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
		case PhysicsServer3D::BODY_PARAM_INERTIA:
			return body->hop_solid ? to_godot(body->hop_solid->get_inertia()) : Vector3();
		default: return Variant();
	}
}

void HopPhysicsServer::_body_reset_mass_properties(const RID &p_body) {
	// Godot's "recompute inertia from shapes+mass". Drop any custom inertia and
	// re-derive (Phase 8); mass itself is handled directly via BODY_PARAM_MASS.
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	body->custom_inertia = false;
	update_body_inertia(body);
}

void HopPhysicsServer::update_body_inertia(HopBodyData *body) {
	if (!body || !body->hop_solid || body->is_static_or_kinematic())
		return;
	// lock_rotation: Godot maps RigidBody3D.lock_rotation onto BODY_MODE_RIGID_LINEAR
	// (rigid translation, rotation locked). Zero the inertia so inv_inertia==0 and
	// rotates_dynamically() stays false — the body is pushed/integrated linearly but
	// never tumbles, matching GodotPhysics. Overrides custom inertia, as lock_rotation
	// does in Godot (a locked body never spins regardless of its inertia tensor).
	if (body->mode == PhysicsServer3D::BODY_MODE_RIGID_LINEAR) {
		body->hop_solid->set_inertia(hop::vec3<hop_scalar>{}); // zero ⇒ inv_inertia 0 ⇒ never spins
		return;
	}
	if (body->custom_inertia)
		return;
	if (body->mass <= 0.0f)
		return;
	// local_bound_ encloses the (scale-baked) collision shapes; treat it as a solid
	// box. Zero extent (no shapes yet) → zero inertia → no spin until shapes exist;
	// the per-step lazy init in _step retries once they do.
	body->hop_solid->set_inertia(aabb_box_inertia(body->hop_solid->get_local_bound(), body->mass));
}

void HopPhysicsServer::_body_set_state(const RID &p_body, PhysicsServer3D::BodyState p_state, const Variant &p_value) {
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body) return;
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: {
			Transform3D old_transform = body->transform;
			body->transform = p_value;
			if (body->hop_solid) {
				if (body->mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
					// Do NOT teleport kinematic bodies in hop here.  The pre-step
					// loop in _step computes velocity = delta / dt so the body
					// sweeps through space and pushes dynamic bodies in its path.
				} else {
					body->hop_solid->set_position(to_hop(body->transform.origin));

					// Static rotation: a rotation change just updates the solid's
					// orientation (cheap — no geometry rebake; the narrowphase honors
					// it directly). Scale is still baked into the shapes, so a scale
					// change requires a rebuild.
					if (!body->transform.basis.is_equal_approx(old_transform.basis)) {
						if (!body->transform.basis.get_scale().is_equal_approx(
						        old_transform.basis.get_scale())) {
							rebuild_body_shapes(body);
						}
						body->hop_solid->set_orientation(
						    to_hop_orientation(body->transform.basis));
					}

					// set_position calls activate() — re-deactivate static bodies
					if (body->mode == PhysicsServer3D::BODY_MODE_STATIC) {
						body->hop_solid->deactivate();
					}
				}
			}
		} break;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: {
			body->linear_velocity = p_value;
			// Kinematic bodies are position-controlled by game script; hop must not
			// integrate their velocity, so only push to hop_solid for dynamic bodies.
			if (body->hop_solid && body->mode != PhysicsServer3D::BODY_MODE_KINEMATIC)
				body->hop_solid->set_velocity(to_hop(body->linear_velocity));
		} break;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: {
			body->angular_velocity = p_value;
			// Phase 8: a directly-set ω on a dynamic body seeds its integrated spin.
			// Kinematic carry ω is still derived from the per-frame orientation delta
			// in _step (not this setter), so only push for dynamic bodies.
			if (body->hop_solid && body->mode != PhysicsServer3D::BODY_MODE_KINEMATIC)
				body->hop_solid->set_angular_velocity(to_hop(body->angular_velocity));
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
	// Phase 8: an angular impulse J changes angular momentum by J, so Δω = I⁻¹·J.
	// I⁻¹ is diagonal in the body frame, so rotate J in by Rᵀ, divide, rotate back.
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || !body->hop_solid || body->is_static_or_kinematic() || !body->hop_solid->rotates_dynamically()) return;
	hop::vec3<hop_scalar> dw; // Δω = I⁻¹·J (world), via the shared body-frame round-trip
	hop::apply_inv_inertia_world(body->hop_solid.get(), to_hop(p_impulse), dw);
	hop::vec3<hop_scalar> w = body->hop_solid->get_angular_velocity();
	hop::add(w, dw);
	body->hop_solid->set_angular_velocity(w);
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
	// Phase 8: a one-step world-frame torque; hop integrates ω += I⁻¹·τ·dt and
	// clears it each step (body-frame conversion handled in integrate_angular).
	HopBodyData *body = body_owner.get_or_null(p_body);
	if (!body || !body->hop_solid || body->is_static_or_kinematic()) return;
	body->hop_solid->add_torque(to_hop(p_torque));
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

	// This drives CharacterBody3D.move_and_slide().  For clean wall sliding it
	// must, like Godot's own physics: (1) depenetrate the body from anything it
	// currently overlaps (recovery), (2) leave a `margin`-sized gap between the
	// body and the wall it stops against (so the next slide step doesn't start
	// flush and immediately re-collide at t=0), and (3) report distinct
	// safe/unsafe fractions plus the recovery depth.
	hop::vec3<hop_scalar> orig_pos = body->hop_solid->get_position();

	// p_from is the full transform to test from, not just a position: move_and_slide
	// can probe a rotation the body's committed transform doesn't have yet. Pose the
	// solid to match for the duration of the query (it is restored with the position
	// below, since this is a const query). Scale is deliberately not applied — that
	// lives in the shape geometry and would mean a rebuild.
	// The quaternion, not the mat3, is what gets restored: set_orientation re-derives
	// orientation_q_ from the matrix, and under fixed-point that mat3→quat round trip
	// loses precision off a spinning body's integrated orientation — once per
	// move_and_slide, which accumulates.
	hop::quat<hop_scalar> orig_orient_q = body->hop_solid->get_orientation_quat();
	hop::mat3<hop_scalar> from_orient = to_hop_orientation(p_from.basis);
	const bool repose = from_orient != body->hop_solid->get_orientation();
	if (repose)
		body->hop_solid->set_orientation(from_orient);

	float margin = std::max(p_margin, 0.0f);
	// How far past a contact surface we shove the body during recovery: enough to
	// re-establish the margin gap, but small enough not to pop through thin walls.
	float skin = margin > 1e-4f ? margin : 1e-3f;

	// --- (1) Recovery: iteratively push out of any static overlap ---
	Vector3 recover;
	Vector3 recover_normal;
	float recover_depth = 0.0f;
	HopBodyData *recover_body = nullptr;  // the body we depenetrated from (for its contact velocity)
	const int MAX_RECOVER_ITERS = 4;
	for (int i = 0; i < MAX_RECOVER_ITERS; ++i) {
		Vector3 test_pos = p_from.origin + recover;
		body->hop_solid->set_position(to_hop(test_pos));

		hop::segment<hop_scalar> probe;
		probe.set_start_end(to_hop(test_pos), to_hop(test_pos));

		hop::collision<hop_scalar> overlap;
		space->simulator->trace_solid(overlap, body->hop_solid.get(), probe, body->collision_mask);

		// time >= 1 means no static overlap at this position.
		if (to_godot_float(overlap.time) > 0.0f) break;

		float depth = to_godot_float(overlap.depth);
		Vector3 n = to_godot(overlap.normal);
		if (depth <= 0.0f || n == Vector3()) break; // touching, but not penetrating

		recover += n * (depth + skin);
		recover_normal = n;
		recover_body = body_of(overlap.collider, overlap.collidee, body->hop_solid.get());
		if (depth > recover_depth) recover_depth = depth;
	}

	Vector3 from_pos = p_from.origin + recover;

	// --- (2) Cast the motion from the recovered position ---
	// Extend the probe a `margin` beyond the requested motion so we also detect a
	// surface the body is already resting within `margin` of.  Step (3) leaves the
	// body sitting `margin` above whatever it stops against (the floor, after a
	// landing).  A standing body's per-frame motion is just the gravity sliver,
	// which is smaller than that gap, so an exact-length cast misses the floor and
	// is_on_floor() flickers as the body drifts in and out of the gap.  The
	// extension is detection-only: hit distances are re-expressed against the real
	// motion below, so mid-motion wall hits and sliding are unaffected.  (Walking
	// grounding doesn't rely on this; the movement code does its own down-cast.)
	float motion_len = p_motion.length();
	Vector3 probe_motion = p_motion;
	float probe_len = motion_len;
	if (motion_len > 1e-6f) {
		probe_motion += (p_motion / motion_len) * margin;
		probe_len = motion_len + margin;
	}

	body->hop_solid->set_position(to_hop(from_pos));

	hop::segment<hop_scalar> seg;
	seg.set_start_end(to_hop(from_pos), to_hop(from_pos + probe_motion));

	// Don't blend normals across triangles for this query: a beveled normal at a
	// triangle seam makes velocity.slide() over-cancel and the body catch on flat
	// walls.  We want the crisp face normal so sliding stays smooth.
	bool saved_avg = space->simulator->get_average_normals();
	space->simulator->set_average_normals(false);

	hop::collision<hop_scalar> result;
	space->simulator->trace_solid(result, body->hop_solid.get(), seg, body->collision_mask);

	space->simulator->set_average_normals(saved_avg);

	// --- (2b) Ground probe: detect the floor the body is resting on, independent
	// of the motion direction.  The motion-aligned cast above only sees the floor
	// when the motion points at it; it misses it whenever the motion is tangential
	// (running/strafing along a wall) or near-zero (just landed) — which is exactly
	// when is_on_floor(), and therefore the ability to jump, would wrongly drop
	// out.  Cast a short sweep straight down (along gravity) from the recovered
	// position; a hit becomes an extra resting contact reported below.  Skipped
	// while moving against gravity so a jump reads as airborne immediately.
	bool ground_hit = false;
	Vector3 ground_point;
	Vector3 ground_normal;
	HopBodyData *ground_body = nullptr;  // the floor body (for moving-platform carry velocity)
	if (p_recovery_as_collision) {
		Vector3 gvec = to_godot(space->simulator->get_gravity());
		float glen = gvec.length();
		if (glen > 0.0f && p_motion.dot(gvec / glen) >= -1e-4f) {
			Vector3 down = gvec / glen;
			// Cover the `margin` resting gap (step 3) with slack for numerics.
			float probe_dist = std::max(margin * 2.0f, 0.02f);
			body->hop_solid->set_position(to_hop(from_pos));
			hop::segment<hop_scalar> gseg;
			gseg.set_start_end(to_hop(from_pos), to_hop(from_pos + down * probe_dist));
			space->simulator->set_average_normals(false);
			hop::collision<hop_scalar> gres;
			space->simulator->trace_solid(gres, body->hop_solid.get(), gseg, body->collision_mask);
			space->simulator->set_average_normals(saved_avg);
			if (to_godot_float(gres.time) < 1.0f) {
				ground_hit = true;
				ground_point = to_godot(gres.point);
				ground_normal = to_godot(gres.normal);
				ground_body = body_of(gres.collider, gres.collidee, body->hop_solid.get());
			}
		}
	}

	// Restore the authoritative pose (this is a const query).
	body->hop_solid->set_position(orig_pos);
	if (repose)
		body->hop_solid->set_orientation_from_quat(orig_orient_q);

	// result.time is a fraction of the (possibly extended) probe.  Convert to a
	// hit distance, then back to a fraction of the *real* motion for Godot's
	// safe/unsafe semantics — a contact found only in the margin extension lands
	// at unsafe == 1 (body doesn't advance) but is still reported below.
	float probe_time = std::min(to_godot_float(result.time), 1.0f);
	float hit_dist = probe_time * probe_len;
	float unsafe = motion_len > 1e-6f ? std::min(hit_dist / motion_len, 1.0f)
	                                  : (probe_time < 1.0f ? 0.0f : 1.0f);

	// A t==0 swept contact (the body is already flush — within the speculative
	// margin of a surface) must not starve a move the body isn't really driving
	// into. Depenetration is handled separately by the recovery step above. Two
	// cases get freed (probe_time/unsafe forced to "no swept hit"):
	//
	//  1. WALKABLE surface (normal within 45° of up): the floor/ramp you stand on.
	//     It must never block horizontal motion — but a walk carries a small
	//     downward gravity sliver, so mdir·n is slightly negative and a pure
	//     moving-into test wrongly classifies the floor as an obstruction, zeroing
	//     the safe fraction every tick the capsule settles within the (8mm) spec
	//     band of the floor. That is the "random stuck on flat floor" bug.
	//  2. WALL the body is only flush against, not moving into (tangential or
	//     separating) — so it can slide along a wall instead of sticking in the
	//     spec band (recovery, measured against the bare radius, never pushes it
	//     out of that band).
	//
	// Genuine into-a-wall motion (low n.y, mdir·n < 0) still blocks normally.
	if (probe_time <= 1e-4f && motion_len > 1e-6f) {
		Vector3 n = to_godot(result.normal);
		Vector3 up(0.0f, 1.0f, 0.0f);
		Vector3 g = to_godot(space->simulator->get_gravity());
		if (g.length() > 1e-6f) up = -g.normalized();
		bool walkable = n.dot(up) >= 0.707f;             // floor/ramp — never blocks the walk
		bool not_into = (p_motion / motion_len).dot(n) >= -1e-3f; // wall: only blocks motion into it
		if (walkable || not_into) {
			probe_time = 1.0f;
			unsafe = 1.0f;
		}
	}

	bool recovered = recover != Vector3();
	bool collided = probe_time < 1.0f;

	if (!collided && !recovered && !ground_hit) {
		// Free motion, no overlap.
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

	// --- (3) Back the stopping point off by `margin` so a gap remains ---
	float safe = unsafe;
	if (collided && motion_len > 0.0f) {
		safe = (unsafe * motion_len - margin) / motion_len;
		if (safe < 0.0f) safe = 0.0f;
		if (safe > unsafe) safe = unsafe;
	}

	if (p_result) {
		Vector3 motion_travel = p_motion * safe;
		// Recovery displacement is folded into travel so the body ends up
		// depenetrated; the leftover motion is what move_and_slide will slide.
		p_result->travel = recover + motion_travel;
		p_result->remainder = p_motion - motion_travel;
		p_result->collision_safe_fraction = safe;
		p_result->collision_unsafe_fraction = unsafe;
		p_result->collision_depth = recover_depth;

		int count = 0;
		int max_collisions = p_max_collisions > 0 ? p_max_collisions : 1;

		// Fill one contact entry (other/velocity default to none for static hits).
		auto add_collision = [&](const Vector3 &position, const Vector3 &normal, float depth, HopBodyData *other) {
			auto &col = p_result->collisions[count];
			col.position = position;
			col.normal = normal;
			col.depth = depth;
			col.local_shape = 0;
			col.collider_shape = 0;
			col.collider = other ? other->self_rid : RID();
			col.collider_id = other ? ObjectID(other->object_instance_id) : ObjectID();
			// Velocity at the contact point (v_linear + ω × r), so a CharacterBody3D
			// rider is carried by a moving/rotating platform.
			col.collider_velocity = other
			    ? other->velocity_at_local(position - other->transform.origin)
			    : Vector3();
			count++;
		};

		// Primary contact: the wall the swept motion stops against. test_solid
		// records the hit partner in `collider` (it sets col.collider = the other
		// solid and leaves collidee null on the swept-query path), so read that —
		// falling back to collidee for any path using the opposite convention, and
		// excluding self. Without this, move_and_collide().get_collider() came back
		// null/RID(0) for every static hit (worldspawn trimesh AND brush-entity
		// convex), since the old code read the always-null collidee.
		if (collided) {
			HopBodyData *other = body_of(result.collider, result.collidee, body->hop_solid.get());
			add_collision(to_godot(result.point), to_godot(result.normal), 0.0f, other);
		}

		// Recovery contact: report the surface we pushed out of so callers that
		// asked for it (is_on_floor / wall stability) see a stable contact.
		if (p_recovery_as_collision && recovered && recover_normal != Vector3() && count < max_collisions) {
			add_collision(from_pos, recover_normal, recover_depth, recover_body);
		}

		// Ground-rest contact: the floor found by the gravity-down probe, reported
		// as a zero-depth touch (it never pushes the body) so is_on_floor() stays
		// true while the swept motion is sliding along a wall or has just stopped
		// after a landing.  CharacterBody3D still classifies floor-vs-wall by angle.
		if (ground_hit && ground_normal != Vector3() && count < max_collisions) {
			add_collision(ground_point, ground_normal, 0.0f, ground_body);
		}

		p_result->collision_count = count;
	}

	return collided || (p_recovery_as_collision && recovered) || ground_hit;
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

	j->hop_constraint = std::make_shared<hop::constraint<hop_scalar>>(ba->hop_solid, bb->hop_solid);
	j->hop_constraint->set_spring_constant(to_hop_scalar(100.0f));
	j->hop_constraint->set_damping_constant(to_hop_scalar(10.0f));
	j->hop_constraint->set_rest_length(to_hop_scalar(0.0f));
	// Pin at the joint's anchor points, not the body centers. Godot's local_A/local_B
	// are offsets in each body's local frame — exactly hop's local anchors. Off-center
	// anchors now also torque a dynamic body via their lever arm (hop Phase 10).
	j->hop_constraint->set_local_anchor_a(to_hop(j->local_a));
	j->hop_constraint->set_local_anchor_b(to_hop(j->local_b));

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
	if (!j) return;
	j->local_a = p_local_A;
	if (j->hop_constraint) j->hop_constraint->set_local_anchor_a(to_hop(p_local_A));
}

Vector3 HopPhysicsServer::_pin_joint_get_local_a(const RID &p_joint) const {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	return j ? j->local_a : Vector3();
}

void HopPhysicsServer::_pin_joint_set_local_b(const RID &p_joint, const Vector3 &p_local_B) {
	HopJointData *j = joint_owner.get_or_null(p_joint);
	if (!j) return;
	j->local_b = p_local_B;
	if (j->hop_constraint) j->hop_constraint->set_local_anchor_b(to_hop(p_local_B));
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

	j->hop_constraint = std::make_shared<hop::constraint<hop_scalar>>(ba->hop_solid, bb->hop_solid);
	j->hop_constraint->set_spring_constant(to_hop_scalar(j->linear_spring_stiffness));
	j->hop_constraint->set_damping_constant(to_hop_scalar(j->linear_spring_damping));
	j->hop_constraint->set_rest_length(to_hop_scalar(j->linear_spring_equilibrium));

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
			if (j->hop_constraint) j->hop_constraint->set_spring_constant(to_hop_scalar(p_value));
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			j->linear_spring_damping = p_value;
			if (j->hop_constraint) j->hop_constraint->set_damping_constant(to_hop_scalar(p_value));
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			j->linear_spring_equilibrium = p_value;
			if (j->hop_constraint) j->hop_constraint->set_rest_length(to_hop_scalar(p_value));
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
		remove_area_from_space(area);
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

	// hop integrates in seconds (gravity is m/s²), so feed it the frame time
	// directly. Passing milliseconds here over-integrates by 1000x and blows
	// the scene apart on the first frame.
	hop_scalar dt = to_hop_scalar(p_step > 0.0f ? p_step : (1.0f / 60.0f));

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
		// Phase 8: per-step constant torque, and lazy auto-inertia (covers the
		// shapes-set-after-mass ordering — update_body_inertia is a no-op once set or
		// when the game pinned a custom inertia).
		if (body->constant_torque.length_squared() > 0.0f) {
			body->hop_solid->add_torque(to_hop(body->constant_torque));
		}
		if (!body->custom_inertia && body->mass > 0.0f && !body->hop_solid->rotates_dynamically()
				&& body->mode != PhysicsServer3D::BODY_MODE_RIGID_LINEAR) {
			update_body_inertia(body);
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

	// Apply area space overrides (gravity, linear damp) to dynamic bodies.
	// hop applies global gravity uniformly, so we must compensate here each step.
	body_owner.for_each([&](HopBodyData *body) {
		if (!body->hop_solid || body->is_static_or_kinematic()) return;
		HopSpaceData *space = space_owner.get_or_null(body->space_rid);
		if (!space) return;

		struct AreaMatch {
			int priority;
			PhysicsServer3D::AreaSpaceOverrideMode grav_mode;
			Vector3 gravity;
			PhysicsServer3D::AreaSpaceOverrideMode damp_mode;
			float lin_damp;
			Vector3 fluid_vel; // wind_direction * wind_force_magnitude
		};
		std::vector<AreaMatch> matched;
		auto body_wb = body->hop_solid->get_world_bound();

		area_owner.for_each([&](HopAreaData *area) {
			bool has_grav = area->space_override_mode != PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
			bool has_damp = area->linear_damp_override_mode != PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
			bool has_wind = area->wind_force_magnitude != 0.0f;
			if (!has_grav && !has_damp && !has_wind) return;
			if (!area->space_rid.is_valid() || area->space_rid != body->space_rid) return;
			if (!(area->collision_mask & body->hop_solid->get_collision_scope())) return;

			if (!area->hop_solid || area->hop_solid->get_shapes().empty()) return;
			hop::aa_box<hop_scalar> area_wb = area->hop_solid->get_world_bound();
			if (!hop::test_intersection(body_wb, area_wb)) return;

			Vector3 ag = area->gravity_direction.normalized() * area->gravity;
			Vector3 fv = area->wind_direction.normalized() * area->wind_force_magnitude;
			matched.push_back({ area->priority, area->space_override_mode, ag,
				area->linear_damp_override_mode, area->linear_damp, fv });
		});

		// Linear damp: always update so leaving an area resets the coefficient.
		float resolved_drag = body->linear_damp;
		{
			bool any_damp = false;
			if (!matched.empty()) {
				std::sort(matched.begin(), matched.end(), [](const AreaMatch &a, const AreaMatch &b) {
					return a.priority > b.priority;
				});
				resolved_drag = 0.0f;
				bool done = false;
				for (auto &e : matched) {
					if (e.damp_mode == PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED) continue;
					any_damp = true;
					switch (e.damp_mode) {
						case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:          resolved_drag += e.lin_damp; break;
						case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:  resolved_drag += e.lin_damp; done = true; break;
						case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:          resolved_drag  = e.lin_damp; done = true; break;
						case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:  resolved_drag  = e.lin_damp; break;
						default: break;
					}
					if (done) break;
				}
				if (!done) resolved_drag += body->linear_damp;
			}
			if (!any_damp) resolved_drag = body->linear_damp;
			body->hop_solid->set_coefficient_of_effective_drag(to_hop_scalar(resolved_drag));
		}

		if (matched.empty()) return;

		// Gravity override: apply a force that corrects for area gravity vs global gravity.
		{
			bool any_grav = false;
			Vector3 result;
			bool done = false;
			for (auto &e : matched) {
				if (e.grav_mode == PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED) continue;
				any_grav = true;
				switch (e.grav_mode) {
					case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:          result += e.gravity; break;
					case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:  result += e.gravity; done = true; break;
					case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:          result  = e.gravity; done = true; break;
					case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:  result  = e.gravity; break;
					default: break;
				}
				if (done) break;
			}
			if (!any_grav) return;
			Vector3 global_gravity = to_godot(space->simulator->get_gravity());
			if (!done) result += global_gravity;
			result *= body->gravity_scale;
			// hop already applies global_gravity * gravity_scale; add the difference.
			Vector3 correction = result - global_gravity * body->gravity_scale;
			if (correction.length_squared() > 0.0f) {
				body->hop_solid->add_force(to_hop(correction * body->mass));
			}
		}

		// Fluid / wind velocity: sum all area fluid velocities and apply a force
		// that drives the body toward that velocity via drag.
		// hop computes: a_drag = (fluid_vel_global - v) * drag_coeff / mass
		// With fluid_vel_global = 0, we add F = area_fluid_vel * drag_coeff to get
		// a_net = (area_fluid_vel - v) * drag_coeff / mass — without touching hop.
		{
			Vector3 fluid_vel_sum;
			for (auto &e : matched) {
				fluid_vel_sum += e.fluid_vel;
			}
			if (fluid_vel_sum.length_squared() > 0.0f && resolved_drag > 0.0f) {
				body->hop_solid->add_force(to_hop(fluid_vel_sum * resolved_drag));
			}
		}
	});

	// Drive kinematic bodies by computing velocity = (target - current) / dt.
	// This makes them sweep through space so they can push dynamic bodies in
	// their path, rather than teleporting and only passively blocking.
	{
		float fdt = p_step > 0.0f ? p_step : (1.0f / 60.0f);
		hop_scalar inv_dt = scalar_from_float<hop_scalar>(1.0f / fdt);
		// Teleport threshold: if the requested delta exceeds 10 m treat as a
		// discontinuous jump (respawn, level load, etc.) and snap instead.
		const float teleport_dist2 = 10.0f * 10.0f;

		body_owner.for_each([&](HopBodyData *body) {
			if (!body->hop_solid || body->mode != PhysicsServer3D::BODY_MODE_KINEMATIC) return;
			if (!body->space_rid.is_valid()) return;

			hop::vec3<hop_scalar> old_pos = body->hop_solid->get_position();
			hop::vec3<hop_scalar> new_pos = to_hop(body->transform.origin);
			hop::vec3<hop_scalar> delta;
			hop::sub(delta, new_pos, old_pos);

			float dx = to_godot_float(delta.x);
			float dy = to_godot_float(delta.y);
			float dz = to_godot_float(delta.z);

			// Angular analog of the linear sweep: derive ω from the body's
			// per-frame orientation delta so a rotating kinematic platform
			// (func_rotating) carries the riders touching it. ΔR = R_new·R_oldᵀ
			// (world-frame delta) → axis-angle → ω = axis·θ/dt. hop snapshots
			// orientation per frame rather than integrating ω, so we also commit
			// the new orientation here; the post-step snap-back zeroes ω again.
			hop::mat3<hop_scalar> old_R = body->hop_solid->get_orientation();
			hop::mat3<hop_scalar> new_R = to_hop_orientation(body->transform.basis);
			hop::vec3<hop_scalar> omega;  // zero unless the body actually rotated
			hop_scalar angle {};
			if (new_R != old_R) {
				// Most kinematic bodies only translate, so skip the quat / axis-angle
				// (two sqrts) whenever the orientation is unchanged frame-to-frame.
				hop::mat3<hop_scalar> old_RT;
				hop::transpose(old_RT, old_R);
				hop::mat3<hop_scalar> dR;
				hop::mul(dR, new_R, old_RT);
				hop::quat<hop_scalar> dq;
				hop::set_quat_from_mat3(dq, dR);
				hop::vec3<hop_scalar> axis;
				angle = hop::get_axis_angle_from_quat(
				    axis, dq, hop::scalar_traits<hop_scalar>::default_epsilon());
				hop::mul(omega, axis, angle * inv_dt);
			}

			// A rotation past this per-frame angle is a placement snap, not a smooth
			// spin (0.5 rad/frame ≈ 1800°/s — far above any real platform), so it is
			// treated as discontinuous like a large translation: it would otherwise
			// impart a one-frame ω spike on the frame a body is placed pre-rotated.
			const float angular_teleport = 0.5f;
			bool jump = dx * dx + dy * dy + dz * dz > teleport_dist2 ||
			            to_godot_float(angle) > angular_teleport;

			// Orientation is committed every frame regardless (hop snapshots it and
			// never integrates it); only position / velocity / ω differ by branch.
			body->hop_solid->set_orientation(new_R);
			if (jump) {
				// Discontinuous jump (placement/respawn): snap, carry no motion.
				body->hop_solid->set_position(new_pos);
				body->hop_solid->set_velocity(hop::vec3<hop_scalar>{});
				body->hop_solid->set_angular_velocity(hop::vec3<hop_scalar>{});
				body->linear_velocity = Vector3();
				body->angular_velocity = Vector3();
			} else {
				// Set velocity so hop sweeps the body to new_pos (pushing dynamics and
				// feeding the contact solver's dynamic-rider carry via ω).
				hop::vec3<hop_scalar> vel;
				vel.x = delta.x * inv_dt;
				vel.y = delta.y * inv_dt;
				vel.z = delta.z * inv_dt;
				body->hop_solid->set_velocity(vel);
				body->hop_solid->set_angular_velocity(omega);
				body->hop_solid->activate();
				// Publish the per-frame motion as this body's velocity (as GodotPhysics3D
				// tracks a kinematic body), so a CharacterBody3D rider's platform carry
				// reads v + ω×r via get_velocity_at_local_position / collider_velocity —
				// the rider carries itself; it is not a hop dynamic body.
				body->linear_velocity = to_godot(vel);
				body->angular_velocity = to_godot(omega);
			}
		});
	}

	// Step all active spaces
	space_owner.for_each([&](HopSpaceData *space) {
		if (!space->active) return;
		space->simulator->update(dt, 0x3FFFFFFF);
	});

	// After stepping, snap kinematic bodies back to the Godot-authoritative
	// position and zero their hop velocity. hop may have stopped the swept body
	// short at a contact; Godot's transform is the target. (The platform-carry
	// velocity above lives on body->linear/angular, so it survives this.)
	body_owner.for_each([&](HopBodyData *body) {
		if (!body->hop_solid || body->mode != PhysicsServer3D::BODY_MODE_KINEMATIC) return;
		body->hop_solid->set_position(to_hop(body->transform.origin));
		body->hop_solid->set_velocity(hop::vec3<hop_scalar>{});
		body->hop_solid->set_angular_velocity(hop::vec3<hop_scalar>{});
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

	// Area monitoring: detect body enter/exit for areas with monitor callbacks
	area_owner.for_each([&](HopAreaData *area) {
		if (!area->monitor_callback.is_valid()) return;
		if (!area->space_rid.is_valid()) return;
		HopSpaceData *space = space_owner.get_or_null(area->space_rid);
		if (!space) return;

		// The area's world AABB comes straight off its persistent solid.
		if (!area->hop_solid || area->hop_solid->get_shapes().empty()) return;
		hop::aa_box<hop_scalar> area_aabb = area->hop_solid->get_world_bound();

		// Find bodies overlapping the area's AABB. The buffer holds every solid in the
		// space, so a body can never be crowded out of the result (see HopSpaceData).
		auto &found = space->size_for_bodies(space->monitor_body_buffer);
		int count = space->simulator->find_solids_in_aa_box(area_aabb, found.data(), (int)found.size(),
			(int)area->collision_mask);

		// Zero-length sweep of the area's own sensor solid, for the narrow-phase
		// confirm below.
		hop::segment<hop_scalar> zseg;
		zseg.set_start_end(area->hop_solid->get_position(), area->hop_solid->get_position());

		std::map<uint64_t, RID> current_overlaps;
		for (int i = 0; i < count; i++) {
			hop::solid<hop_scalar> *s = found[i];
			if (!s) continue;

			// Check collision mask: area's mask vs body's layer
			if (!(area->collision_mask & s->get_collision_scope())) continue;

			HopBodyData *body = static_cast<HopBodyData *>(s->get_user_data());
			if (!body) continue;

			// Narrow-phase confirm — the broadphase only compared bounding boxes, and
			// the shapes inside them usually don't touch.  A CharacterBody3D resting
			// against a wall sits exactly its safe_margin (1mm) away, and a capsule
			// standing clear of a box's corner still has crossing AABBs: both read as
			// "inside" without this, so e.g. a func_door's blocked-detector treats a
			// player merely standing against the door as a rider and carries them up
			// with it.  Matches _intersect_shape, which confirms the same way.
			hop::collision<hop_scalar> col;
			space->simulator->test_solid(col, area->hop_solid.get(), zseg, s);
			if (to_godot_float(col.time) >= 1.0f) continue;

			current_overlaps[body->object_instance_id] = body->self_rid;
		}

		// Fire ADDED callbacks for newly overlapping bodies
		for (auto &[id, rid] : current_overlaps) {
			if (area->overlapping_bodies.find(id) == area->overlapping_bodies.end()) {
				area->monitor_callback.call(
					PhysicsServer3D::AREA_BODY_ADDED,
					rid, ObjectID(id), 0, 0);
			}
		}

		// Fire REMOVED callbacks for bodies that left
		for (auto &[id, rid] : area->overlapping_bodies) {
			if (current_overlaps.find(id) == current_overlaps.end()) {
				area->monitor_callback.call(
					PhysicsServer3D::AREA_BODY_REMOVED,
					rid, ObjectID(id), 0, 0);
			}
		}

		area->overlapping_bodies = current_overlaps;
	});

	// Area-to-area monitoring: detect when areas (monitoring=true) overlap monitorable areas
	area_owner.for_each([&](HopAreaData *detector) {
		if (!detector->area_monitor_callback.is_valid()) return;
		if (!detector->space_rid.is_valid()) return;
		HopSpaceData *space = space_owner.get_or_null(detector->space_rid);
		if (!space) return;

		// Detector AABB from its persistent solid.
		if (!detector->hop_solid || detector->hop_solid->get_shapes().empty()) return;
		hop::aa_box<hop_scalar> det_aabb = detector->hop_solid->get_world_bound();

		// Broadphase the area index instead of scanning every area: the candidates
		// it returns already passed the AABB-overlap test against det_aabb, and are
		// all in this space (area_bvh is per-space).
		auto &cand = space->size_for_areas(space->monitor_area_buffer);
		int n = space->area_bvh.find_solids_in_aa_box(det_aabb, cand.data(), (int)cand.size(),
			(int)detector->collision_mask);

		hop::segment<hop_scalar> zseg;
		zseg.set_start_end(detector->hop_solid->get_position(), detector->hop_solid->get_position());

		std::map<uint64_t, RID> current_area_overlaps;
		for (int i = 0; i < n; i++) {
			HopAreaData *target = static_cast<HopAreaData *>(cand[i]->get_user_data());
			if (!target || target == detector) continue;
			if (!target->monitorable) continue;
			if (!(detector->collision_mask & target->collision_layer)) continue;
			// Narrow-phase confirm, as in the body pass above.
			hop::collision<hop_scalar> col;
			space->simulator->test_solid(col, detector->hop_solid.get(), zseg, cand[i]);
			if (to_godot_float(col.time) >= 1.0f) continue;
			current_area_overlaps[target->object_instance_id] = target->self_rid;
		}

		// Fire ADDED callbacks for newly overlapping areas
		for (auto &[id, rid] : current_area_overlaps) {
			if (detector->overlapping_areas.find(id) == detector->overlapping_areas.end()) {
				detector->area_monitor_callback.call(
					PhysicsServer3D::AREA_BODY_ADDED, rid, ObjectID(id), 0, 0);
			}
		}
		// Fire REMOVED callbacks for areas that left
		for (auto &[id, rid] : detector->overlapping_areas) {
			if (current_area_overlaps.find(id) == current_area_overlaps.end()) {
				detector->area_monitor_callback.call(
					PhysicsServer3D::AREA_BODY_REMOVED, rid, ObjectID(id), 0, 0);
			}
		}
		detector->overlapping_areas = current_area_overlaps;
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
