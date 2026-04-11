#include "hop_space_state.h"
#include "hop_physics_server.h"
#include "hop_body_data.h"
#include "hop_conversions.h"

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/godot.hpp>


bool HopDirectSpaceState::_intersect_ray(const Vector3 &p_from, const Vector3 &p_to, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_hit_from_inside, bool p_hit_back_faces, bool p_pick_ray, PhysicsServer3DExtensionRayResult *p_result) {
	if (!space || !p_collide_with_bodies) return false;

	hop::segment<hop_scalar> seg;
	seg.set_start_end(to_hop(p_from), to_hop(p_to));

	hop::collision<hop_scalar> result;
	space->simulator->trace_segment(result, seg, p_collision_mask);

	if (to_godot_float(result.time) >= 1.0f) return false;

	if (p_result) {
		p_result->position = to_godot(result.point);
		p_result->normal = to_godot(result.normal);
		p_result->face_index = -1;
		p_result->shape = 0;

		if (result.collidee) {
			HopBodyData *hit_body = static_cast<HopBodyData *>(result.collidee->get_user_data());
			if (hit_body) {
				p_result->rid = hit_body->self_rid;
				p_result->collider_id = ObjectID(hit_body->object_instance_id);
				p_result->collider = get_collider_safe(hit_body->object_instance_id);
			}
		}
	}
	return true;
}

int32_t HopDirectSpaceState::_intersect_point(const Vector3 &p_position, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeResult *p_results, int32_t p_max_results) {
	if (!space || !p_collide_with_bodies || p_max_results <= 0) return 0;

	auto hp = to_hop(p_position);
	auto eps = to_hop_scalar(0.001f);
	hop::aa_box<hop_scalar> box(
		hop::vec3<hop_scalar>(hp.x - eps, hp.y - eps, hp.z - eps),
		hop::vec3<hop_scalar>(hp.x + eps, hp.y + eps, hp.z + eps)
	);

	std::vector<hop::solid<hop_scalar> *> found(p_max_results * 2);
	int count = space->simulator->find_solids_in_aa_box(box, found.data(), (int)found.size());

	hop::segment<hop_scalar> seg;
	seg.set_start_end(hp, hp);

	int result_count = 0;
	for (int i = 0; i < count && result_count < p_max_results; i++) {
		hop::solid<hop_scalar> *s = found[i];
		if (!s || !(s->get_collision_scope() & p_collision_mask)) continue;

		HopBodyData *body = static_cast<HopBodyData *>(s->get_user_data());
		if (!body) continue;

		hop::collision<hop_scalar> col;
		space->simulator->trace_segment(col, seg, s->get_collision_scope());
		if (to_godot_float(col.time) >= 1.0f) continue;

		if (p_results) {
			p_results[result_count].rid = body->self_rid;
			p_results[result_count].collider_id = ObjectID(body->object_instance_id);
			p_results[result_count].collider = get_collider_safe(body->object_instance_id);
			p_results[result_count].shape = 0;
		}
		result_count++;
	}
	return result_count;
}

int32_t HopDirectSpaceState::_intersect_shape(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeResult *p_results, int32_t p_max_results) {
	if (!space || !server || !p_collide_with_bodies || p_max_results <= 0) return 0;

	HopShapeData *sd = server->shape_owner.get_or_null(p_shape_rid);
	if (!sd) return 0;

	auto hs = sd->make_hop_shape(Transform3D());
	if (!hs) return 0;

	hop::aa_box<hop_scalar> shape_box;
	hs->get_bound(shape_box);
	auto pos = to_hop(p_transform.origin);
	add(shape_box.mins, pos);
	add(shape_box.maxs, pos);

	if (p_motion.length_squared() > 0) {
		hop::aa_box<hop_scalar> end_box = shape_box;
		auto motion = to_hop(p_motion);
		add(end_box.mins, motion);
		add(end_box.maxs, motion);
		shape_box.merge(end_box.mins);
		shape_box.merge(end_box.maxs);
	}

	std::vector<hop::solid<hop_scalar> *> found(p_max_results * 2);
	int count = space->simulator->find_solids_in_aa_box(shape_box, found.data(), (int)found.size());

	int result_count = 0;
	for (int i = 0; i < count && result_count < p_max_results; i++) {
		hop::solid<hop_scalar> *s = found[i];
		if (!s || !(s->get_collision_scope() & p_collision_mask)) continue;

		HopBodyData *body = static_cast<HopBodyData *>(s->get_user_data());
		if (!body) continue;

		if (p_results) {
			p_results[result_count].rid = body->self_rid;
			p_results[result_count].collider_id = ObjectID(body->object_instance_id);
			p_results[result_count].collider = get_collider_safe(body->object_instance_id);
			p_results[result_count].shape = 0;
		}
		result_count++;
	}
	return result_count;
}

bool HopDirectSpaceState::_cast_motion(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, float *p_closest_safe, float *p_closest_unsafe, PhysicsServer3DExtensionShapeRestInfo *p_info) {
	if (!space || !server) return false;

	HopShapeData *sd = server->shape_owner.get_or_null(p_shape_rid);
	if (!sd) return false;

	auto temp_solid = std::make_shared<hop::solid<hop_scalar>>();
	temp_solid->set_infinite_mass();
	temp_solid->set_position(to_hop(p_transform.origin));
	temp_solid->set_collision_scope(0);
	temp_solid->set_collide_with_scope(p_collision_mask);

	auto hs = sd->make_hop_shape(Transform3D());
	if (!hs) return false;
	temp_solid->add_shape(hs);

	space->simulator->add_solid(temp_solid);

	hop::segment<hop_scalar> seg;
	seg.set_start_end(to_hop(p_transform.origin), to_hop(p_transform.origin + p_motion));

	hop::collision<hop_scalar> result;
	space->simulator->trace_solid(result, temp_solid.get(), seg, p_collision_mask);

	space->simulator->remove_solid(temp_solid);

	float time_f = to_godot_float(result.time);
	if (p_closest_safe) *p_closest_safe = time_f;
	if (p_closest_unsafe) *p_closest_unsafe = time_f;

	if (time_f < 1.0f && p_info) {
		p_info->point = to_godot(result.point);
		p_info->normal = to_godot(result.normal);
		p_info->shape = 0;
		if (result.collidee) {
			HopBodyData *hit = static_cast<HopBodyData *>(result.collidee->get_user_data());
			if (hit) {
				p_info->rid = hit->self_rid;
				p_info->collider_id = ObjectID(hit->object_instance_id);
				p_info->linear_velocity = hit->linear_velocity;
			}
		}
		return true;
	}

	return false;
}

bool HopDirectSpaceState::_collide_shape(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, void *p_results, int32_t p_max_results, int32_t *p_result_count) {
	if (p_result_count) *p_result_count = 0;
	return false;
}

bool HopDirectSpaceState::_rest_info(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeRestInfo *p_rest_info) {
	return false;
}

Vector3 HopDirectSpaceState::_get_closest_point_to_object_volume(const RID &p_object, const Vector3 &p_point) const {
	return p_point;
}
