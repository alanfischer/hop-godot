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
	hop_solid->set_coefficient_of_gravity(gravity_scale);
	hop_solid->set_coefficient_of_effective_drag(linear_damp);

	hop_solid->set_collision_scope(collision_layer);
	hop_solid->set_collide_with_scope(collision_mask);

	hop_solid->set_position(to_hop(transform.origin));
	hop_solid->set_velocity(to_hop(linear_velocity));

	if (is_static_or_kinematic()) {
		hop_solid->set_stay_active(false);
		if (mode == PhysicsServer3D::BODY_MODE_STATIC) {
			hop_solid->deactivate();
		}
	}
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
	hop_solid->set_coefficient_of_gravity(gravity_scale);
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

void HopBodyData::rebuild_hop_shapes() {
	if (!hop_solid) return;

	hop_solid->remove_all_shapes();

	for (auto &entry : shapes) {
		if (entry.disabled) continue;
		entry.hop_shape = nullptr;
		// Need shape data to build - caller must provide via HopShapeData lookup
	}
}
