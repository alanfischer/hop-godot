#include "hop_body_data.h"
#include "hop_conversions.h"

void HopBodyData::create_hop_solid() {
	hop_solid = std::make_shared<hop::solid<float>>();
	hop_solid->set_user_data(this);

	if (is_static_or_kinematic()) {
		hop_solid->set_infinite_mass();
	} else {
		hop_solid->set_mass(mass);
	}

	hop_solid->set_coefficient_of_restitution(bounce);
	hop_solid->set_coefficient_of_static_friction(friction);
	hop_solid->set_coefficient_of_dynamic_friction(friction);
	if (is_static_or_kinematic()) {
		hop_solid->set_coefficient_of_gravity(0.0f);
	} else {
		hop_solid->set_coefficient_of_gravity(gravity_scale);
	}
	hop_solid->set_coefficient_of_effective_drag(linear_damp);

	hop_solid->set_collision_scope(collision_layer);
	hop_solid->set_collide_with_scope(collision_mask);

	hop_solid->set_position(to_hop(transform.origin));
	hop_solid->set_velocity(to_hop(linear_velocity));
	hop_solid->set_collision_listener(this);
}

void HopBodyData::sync_to_hop() {
	if (!hop_solid) return;

	hop_solid->set_position(to_hop(transform.origin));

	if (is_static_or_kinematic()) {
		hop_solid->set_infinite_mass();
		hop_solid->set_velocity(to_hop(linear_velocity));
	} else {
		hop_solid->set_mass(mass);
		hop_solid->set_velocity(to_hop(linear_velocity));
	}

	hop_solid->set_coefficient_of_restitution(bounce);
	hop_solid->set_coefficient_of_static_friction(friction);
	hop_solid->set_coefficient_of_dynamic_friction(friction);
	if (is_static_or_kinematic()) {
		hop_solid->set_coefficient_of_gravity(0.0f);
	} else {
		hop_solid->set_coefficient_of_gravity(gravity_scale);
	}
	hop_solid->set_coefficient_of_effective_drag(linear_damp);

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

void HopBodyData::on_collision(const hop::collision<float> &c) {
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

