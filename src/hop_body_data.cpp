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
	hop_solid->set_orientation(to_hop_mat3(Basis(transform.basis.get_rotation_quaternion())));

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

	if (c.collidee) {
		HopBodyData *other = static_cast<HopBodyData *>(c.collidee->get_user_data());
		if (other) {
			ci.collider_rid = other->self_rid;
			ci.collider_id = other->object_instance_id;
			ci.collider_pos = to_godot(c.impact);
			ci.collider_shape = 0;
			ci.collider_velocity = other->linear_velocity;
		}
	}

	contacts.push_back(ci);
}
