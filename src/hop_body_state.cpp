#include "hop_body_state.h"
#include "hop_physics_server.h"
#include "hop_space_state.h"
#include "hop_conversions.h"

#include <godot_cpp/classes/object.hpp>
#include <hop/math/intersect.h>
#include <algorithm>
#include <vector>

Vector3 HopDirectBodyState::_get_total_gravity() const {
	if (!body || !body->hop_solid) return Vector3(0, -9.8f, 0);
	HopSpaceData *space = server->space_owner.get_or_null(body->space_rid);
	if (!space) return Vector3(0, -9.8f, 0);

	Vector3 global_gravity = to_godot(space->simulator->get_gravity());
	auto body_wb = body->hop_solid->get_world_bound();

	// Collect areas overlapping this body that have a gravity override
	struct AreaEntry { int priority; PhysicsServer3D::AreaSpaceOverrideMode mode; Vector3 gravity; };
	std::vector<AreaEntry> matched;

	server->area_owner.for_each([&](HopAreaData *area) {
		if (area->space_override_mode == PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED) return;
		if (!area->space_rid.is_valid() || area->space_rid != body->space_rid) return;

		// The area's solid already maintains this — orientation, per-shape rotation and
		// per-shape offset all folded in (solid::update_local_bound). Rebuilding every
		// shape here to bound them by hand re-derived it wrong the moment a shape carried
		// a local_position or rotation, and cost a shape build per area per query besides.
		if (!area->hop_solid || area->hop_solid->get_shapes().empty()) return;
		hop::aa_box<hop_scalar> area_wb = area->hop_solid->get_world_bound();
		if (!hop::test_intersection(body_wb, area_wb)) return;
		if (!(area->collision_mask & body->hop_solid->get_collision_scope())) return;

		Vector3 ag = area->gravity_direction.normalized() * area->gravity;
		matched.push_back({ area->priority, area->space_override_mode, ag });
	});

	if (matched.empty()) return global_gravity * body->gravity_scale;

	// Higher priority areas are processed first
	std::sort(matched.begin(), matched.end(), [](const AreaEntry &a, const AreaEntry &b) {
		return a.priority > b.priority;
	});

	Vector3 result;
	bool done = false;
	for (auto &e : matched) {
		switch (e.mode) {
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:
				result += e.gravity; break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:
				result += e.gravity; done = true; break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:
				result = e.gravity; done = true; break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:
				result = e.gravity; break;
			default: break;
		}
		if (done) break;
	}
	if (!done) result += global_gravity;

	return result * body->gravity_scale;
}

float HopDirectBodyState::_get_total_linear_damp() const {
	return body ? body->linear_damp : 0.0f;
}

float HopDirectBodyState::_get_total_angular_damp() const {
	return body ? body->angular_damp : 0.0f;
}

Vector3 HopDirectBodyState::_get_center_of_mass() const {
	return body ? body->transform.origin : Vector3();
}

// Zero is the right answer, not a stub. hop has no centre-of-mass concept at all: a
// solid's position IS its centre of mass, which is what lets _get_center_of_mass above
// hand back the body origin directly. So the offset from origin to COM really is zero.
//
// It does diverge from Godot, where the auto mode puts the COM at the shapes' centroid
// and an offset collider therefore has a non-zero local COM. Matching that is not a
// matter of filling this in — it needs a COM offset on the solid, threaded through
// integration and every lever arm — and it would move any body whose collider is offset
// from its origin, the feet-origin player collider included. Left alone deliberately.
Vector3 HopDirectBodyState::_get_center_of_mass_local() const {
	return Vector3();
}

// hop keeps inertia as a DIAGONAL in the body frame, so the principal axes are just the
// body's orientation. Exact for everything hop can represent — and that is also the
// limit: a tensor with off-diagonal terms has no home here, so a shape whose true
// principal axes are not its local ones was already approximated upstream, where
// update_body_inertia treats the whole collision AABB as a solid box. This reports what
// hop holds, which is the question the caller is really asking.
Basis HopDirectBodyState::_get_principal_inertia_axes() const {
	if (!body || !body->hop_solid) return Basis();
	return to_godot_basis(body->hop_solid->get_orientation());
}

float HopDirectBodyState::_get_inverse_mass() const {
	if (!body || body->mass <= 0.0f) return 0.0f;
	return 1.0f / body->mass;
}

// Body-frame reciprocal diagonal, straight off the solid — the same value
// rotates_dynamically() reads, so a zero here means exactly "this body never spins"
// (no shapes yet, or lock_rotation, which zeroes the inertia on purpose).
//
// These two used to return zero with a "no rotation" comment, left over from before hop
// integrated angular velocity at all. That stopped being true and the comment outlived
// it, so Godot's API answered confidently and wrongly for every dynamic body — which
// costs whoever next probes a body's inertia an hour, as it did during the
// test_bsp_contact_point investigation.
Vector3 HopDirectBodyState::_get_inverse_inertia() const {
	if (!body || !body->hop_solid) return Vector3();
	return to_godot(body->hop_solid->get_inv_inertia());
}

// World space, which is what Godot means by the tensor: R·diag(I⁻¹)·Rᵀ. hop already
// keeps exactly that matrix on the solid — the contact solver needs it per angular pair
// per iteration — and the orientation writers rebuild it, so this is a read, not a
// recomputation.
Basis HopDirectBodyState::_get_inverse_inertia_tensor() const {
	if (!body || !body->hop_solid) return Basis();
	return to_godot_basis(body->hop_solid->get_inv_inertia_world());
}

void HopDirectBodyState::_set_linear_velocity(const Vector3 &p_velocity) {
	if (!body) return;
	body->linear_velocity = p_velocity;
	if (body->hop_solid) body->hop_solid->set_velocity(to_hop(p_velocity));
}

Vector3 HopDirectBodyState::_get_linear_velocity() const {
	return body ? body->linear_velocity : Vector3();
}

void HopDirectBodyState::_set_angular_velocity(const Vector3 &p_velocity) {
	if (body) body->angular_velocity = p_velocity; // stored but no-op
}

Vector3 HopDirectBodyState::_get_angular_velocity() const {
	return body ? body->angular_velocity : Vector3();
}

void HopDirectBodyState::_set_transform(const Transform3D &p_transform) {
	if (!body) return;
	body->transform = p_transform;
	if (body->hop_solid) body->hop_solid->set_position(to_hop(p_transform.origin));
}

Transform3D HopDirectBodyState::_get_transform() const {
	return body ? body->transform : Transform3D();
}

Vector3 HopDirectBodyState::_get_velocity_at_local_position(const Vector3 &p_local_position) const {
	if (!body) return Vector3();
	// v_linear + ω × r, same as GodotPhysicsDirectBodyState3D. For a kinematic mover
	// linear/angular velocity are the per-frame motion published in _step, so this is
	// what carries a CharacterBody3D rider on a moving / rotating platform.
	return body->velocity_at_local(p_local_position);
}

void HopDirectBodyState::_apply_central_impulse(const Vector3 &p_impulse) {
	if (!body || body->is_static_or_kinematic()) return;
	float m = body->mass;
	if (m > 0.0f) {
		body->linear_velocity += p_impulse / m;
		if (body->hop_solid) body->hop_solid->set_velocity(to_hop(body->linear_velocity));
	}
}

void HopDirectBodyState::_apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) {
	_apply_central_impulse(p_impulse);
}

void HopDirectBodyState::_apply_torque_impulse(const Vector3 &p_impulse) {
	// no-op
}

void HopDirectBodyState::_apply_central_force(const Vector3 &p_force) {
	if (!body || !body->hop_solid || body->is_static_or_kinematic()) return;
	body->hop_solid->add_force(to_hop(p_force));
}

void HopDirectBodyState::_apply_force(const Vector3 &p_force, const Vector3 &p_position) {
	_apply_central_force(p_force);
}

void HopDirectBodyState::_apply_torque(const Vector3 &p_torque) {
	// no-op
}

void HopDirectBodyState::_add_constant_central_force(const Vector3 &p_force) {
	if (body) body->constant_force += p_force;
}

void HopDirectBodyState::_add_constant_force(const Vector3 &p_force, const Vector3 &p_position) {
	_add_constant_central_force(p_force);
}

void HopDirectBodyState::_add_constant_torque(const Vector3 &p_torque) {
	if (body) body->constant_torque += p_torque;
}

void HopDirectBodyState::_set_constant_force(const Vector3 &p_force) {
	if (body) body->constant_force = p_force;
}

Vector3 HopDirectBodyState::_get_constant_force() const {
	return body ? body->constant_force : Vector3();
}

void HopDirectBodyState::_set_constant_torque(const Vector3 &p_torque) {
	if (body) body->constant_torque = p_torque;
}

Vector3 HopDirectBodyState::_get_constant_torque() const {
	return body ? body->constant_torque : Vector3();
}

void HopDirectBodyState::_set_sleep_state(bool p_enabled) {
	if (!body) return;
	body->sleeping = p_enabled;
	if (body->hop_solid) {
		if (p_enabled) body->hop_solid->deactivate();
		else body->hop_solid->activate();
	}
}

bool HopDirectBodyState::_is_sleeping() const {
	return body ? body->sleeping : false;
}

int32_t HopDirectBodyState::_get_contact_count() const {
	return body ? (int32_t)body->contacts.size() : 0;
}

Vector3 HopDirectBodyState::_get_contact_local_position(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].local_pos;
}

Vector3 HopDirectBodyState::_get_contact_local_normal(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].local_normal;
}

Vector3 HopDirectBodyState::_get_contact_impulse(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].impulse;
}

int32_t HopDirectBodyState::_get_contact_local_shape(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return 0;
	return body->contacts[p_contact_idx].local_shape;
}

Vector3 HopDirectBodyState::_get_contact_local_velocity_at_position(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].local_velocity;
}

RID HopDirectBodyState::_get_contact_collider(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return RID();
	return body->contacts[p_contact_idx].collider_rid;
}

Vector3 HopDirectBodyState::_get_contact_collider_position(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].collider_pos;
}

uint64_t HopDirectBodyState::_get_contact_collider_id(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return 0;
	return body->contacts[p_contact_idx].collider_id;
}

Object *HopDirectBodyState::_get_contact_collider_object(int32_t p_contact_idx) const {
	uint64_t id = _get_contact_collider_id(p_contact_idx);
	if (!id) return nullptr;
	// A binding wrapper, NOT the raw engine pointer get_collider_safe() returns: godot-cpp
	// marshals a virtual's Object* return through ret->_owner, so a raw pointer reads garbage
	// out of the engine object and the caller sees null. (The raw form is right where the
	// engine reads our pointer directly — the query result structs in hop_space_state.)
	return ObjectDB::get_instance(id);
}

int32_t HopDirectBodyState::_get_contact_collider_shape(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return 0;
	return body->contacts[p_contact_idx].collider_shape;
}

Vector3 HopDirectBodyState::_get_contact_collider_velocity_at_position(int32_t p_contact_idx) const {
	if (!body || p_contact_idx < 0 || p_contact_idx >= (int32_t)body->contacts.size()) return Vector3();
	return body->contacts[p_contact_idx].collider_velocity;
}

float HopDirectBodyState::_get_step() const {
	return server ? server->last_step : (1.0f / 60.0f);
}

void HopDirectBodyState::_integrate_forces() {
	// hop handles integration internally
}

PhysicsDirectSpaceState3D *HopDirectBodyState::_get_space_state() {
	if (!body || !server || !body->space_rid.is_valid()) return nullptr;
	HopSpaceData *space = server->space_owner.get_or_null(body->space_rid);
	if (!space) return nullptr;
	if (!space->direct_state) {
		space->direct_state = memnew(HopDirectSpaceState);
		space->direct_state->space = space;
		space->direct_state->server = server;
	}
	return space->direct_state;
}
