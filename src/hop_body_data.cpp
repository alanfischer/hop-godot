#include "hop_body_data.h"
#include "hop_conversions.h"

void HopBodyData::create_hop_solid() {
	hop_solid = std::make_shared<hop::solid<hop_scalar>>();
	hop_solid->set_user_data(this);

	if (is_static_or_kinematic()) {
		hop_solid->set_infinite_mass();
	} else {
		hop_solid->set_mass(to_hop_scalar(mass));
	}

	hop_solid->set_coefficient_of_restitution(to_hop_scalar(bounce));
	// Godot combines a contact's two bounce values with max, not hop's default
	// average. Level geometry carries no PhysicsMaterial (bounce 0), so averaging
	// halved every bouncy body against the world: a 0.75 ball bounced at 0.375 and
	// was flat after one hop.
	hop_solid->set_restitution_combine(hop::restitution_combine::maximum);
	hop_solid->set_coefficient_of_static_friction(to_hop_scalar(friction));
	hop_solid->set_coefficient_of_dynamic_friction(to_hop_scalar(friction));
	if (is_static_or_kinematic()) {
		hop_solid->set_coefficient_of_gravity(scalar_from_int<hop_scalar>(0));
	} else {
		hop_solid->set_coefficient_of_gravity(to_hop_scalar(gravity_scale));
	}
	hop_solid->set_coefficient_of_effective_drag(to_hop_scalar(linear_damp));

	hop_solid->set_collision_scope(collision_layer);
	hop_solid->set_collide_with_scope(collision_mask);

	hop_solid->set_position(to_hop(transform.origin));
	hop_solid->set_velocity(to_hop(linear_velocity));
	hop_solid->set_collision_callback([this](const hop::collision<hop_scalar> &c) { on_collision(c); });
	hop_solid->set_collision_filter([this](hop::solid<hop_scalar> *other) -> bool {
		if (collision_exceptions.empty()) return true;
		auto *other_body = static_cast<HopBodyData *>(other->get_user_data());
		if (!other_body) return true;
		for (const RID &exc : collision_exceptions) {
			if (exc == other_body->self_rid) return false;
		}
		return true;
	});
}

void HopBodyData::sync_to_hop() {
	if (!hop_solid) return;

	hop_solid->set_position(to_hop(transform.origin));
	// Static rotation: the body's world orientation (rotation only — scale is
	// baked into the shapes by rebuild_body_shapes). hop has no angular dynamics;
	// this is a fixed pose the narrowphase respects.
	hop_solid->set_orientation(to_hop_orientation(transform.basis));

	if (mode == PhysicsServer3D::BODY_MODE_STATIC) {
		hop_solid->set_infinite_mass();
		hop_solid->set_velocity(hop::vec3<hop_scalar>(hop_scalar{}, hop_scalar{}, hop_scalar{}));
	} else if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
		hop_solid->set_infinite_mass();
		// Do not push linear_velocity into hop_solid — kinematic bodies are
		// position-controlled; hop must not integrate their velocity.
	} else {
		hop_solid->set_mass(to_hop_scalar(mass));
		hop_solid->set_velocity(to_hop(linear_velocity));
	}

	hop_solid->set_coefficient_of_restitution(to_hop_scalar(bounce));
	hop_solid->set_coefficient_of_static_friction(to_hop_scalar(friction));
	hop_solid->set_coefficient_of_dynamic_friction(to_hop_scalar(friction));
	if (is_static_or_kinematic()) {
		hop_solid->set_coefficient_of_gravity(scalar_from_int<hop_scalar>(0));
	} else {
		hop_solid->set_coefficient_of_gravity(to_hop_scalar(gravity_scale));
	}
	hop_solid->set_coefficient_of_effective_drag(to_hop_scalar(linear_damp));

	hop_solid->set_collision_scope(collision_layer);
	hop_solid->set_collide_with_scope(collision_mask);
}

void HopBodyData::sync_from_hop() {
	if (!hop_solid) return;
	if (is_static_or_kinematic()) return;

	transform.origin = to_godot(hop_solid->get_position());
	linear_velocity = to_godot(hop_solid->get_velocity());
	// Phase 8: a body that spins dynamically (inv_inertia != 0) has hop integrate
	// its orientation — write it back (rotation only; re-apply the existing scale,
	// which hop bakes into geometry) along with ω. Non-spinning dynamic bodies keep
	// their Godot-authoritative basis untouched, so this is opt-in and adds no churn.
	if (hop_solid->rotates_dynamically()) {
		Vector3 scale = transform.basis.get_scale();
		Basis rot = to_godot_basis(hop_solid->get_orientation());
		rot.scale(scale);
		transform.basis = rot;
		angular_velocity = to_godot(hop_solid->get_angular_velocity());
	}
	sleeping = !hop_solid->active();
}

void HopBodyData::on_collision(const hop::collision<hop_scalar> &c) {
	if (max_contacts_reported <= 0) return;
	if ((int)contacts.size() >= max_contacts_reported) return;

	ContactInfo ci;
	ci.local_pos = to_godot(c.impact);
	ci.local_normal = to_godot(c.normal);
	ci.local_velocity = linear_velocity;
	ci.local_shape = 0;

	// In a *delivered* collision hop names the receiver `collidee` and the thing it hit
	// `collider` (simulator::report_collisions inverts the pair for the second listener so
	// this holds for both). Reading collidee here reported every body as having collided
	// with itself: a RigidBody3D's get_contact_collider_object() came back as its own node,
	// so contact_monitor logic that asks "what did I hit?" — a projectile dealing its damage,
	// say — never found a target and the body just bounced. Take collider, and fall back to
	// collidee for any path that hands us the opposite convention.
	const hop::solid<hop_scalar> *other_solid = c.collider != hop_solid.get() ? c.collider : c.collidee;
	if (other_solid && other_solid != hop_solid.get()) {
		HopBodyData *other = static_cast<HopBodyData *>(other_solid->get_user_data());
		if (other) {
			ci.collider_rid = other->self_rid;
			ci.collider_id = other->object_instance_id;
			ci.collider_pos = to_godot(c.impact);
			ci.collider_shape = 0;
			// Surface velocity at the contact (v + ω×r), matching the _body_test_motion
			// path — so a RigidBody3D rider on a rotating platform reads the full carry
			// velocity, not just the platform's linear motion (Phase 7 rider carry).
			ci.collider_velocity = other->velocity_at_local(to_godot(c.impact) - other->transform.origin);
		}
	}

	contacts.push_back(ci);
}
