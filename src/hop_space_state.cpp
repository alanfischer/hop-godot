#include "hop_space_state.h"
#include "hop_physics_server.h"
#include "hop_body_data.h"
#include "hop_area_data.h"
#include "hop_shape_data.h"
#include "hop_conversions.h"

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/godot.hpp>


// Find areas in `space` overlapping `query_box` (broadphase) whose collision_layer
// matches `mask`, confirm with overlaps(area_solid), and record one result per
// overlapping area (capped at max_results); returns the updated count.  Shared by
// _intersect_point and _intersect_shape.  Uses the space's dedicated area_bvh, so
// far-away areas are pruned in O(log n) and there are no per-query shape rebuilds
// (area solids are built once; see HopPhysicsServer::rebuild_area_shapes).
template <typename Overlaps>
static int collect_overlapping_areas(HopSpaceData *space, const hop::aa_box<hop_scalar> &query_box,
                                     uint32_t mask, PhysicsServer3DExtensionShapeResult *results,
                                     int count, int max_results, Overlaps overlaps) {
	hop::solid<hop_scalar> *cand[256];
	int n = space->area_bvh.find_solids_in_aa_box(query_box, cand, 256);
	for (int i = 0; i < n && count < max_results; i++) {
		HopAreaData *area = static_cast<HopAreaData *>(cand[i]->get_user_data());
		if (!area) continue;
		if (!(mask & area->collision_layer)) continue;

		// Narrow-phase confirm; only then resolve the collider (two engine calls).
		if (!overlaps(cand[i])) continue;

		// Skip the area if its backing node is gone, so a query never reports a hit
		// whose collider is dead (callers deref result.collider, e.g. ladder
		// detection's hits[0].collider.get_parent()).  get_collider_safe returns
		// the raw engine object pointer the result struct requires.
		uint64_t oid = area->object_instance_id;
		if (!UtilityFunctions::is_instance_id_valid((int64_t)oid)) continue;
		Object *collider = get_collider_safe(oid);
		if (!collider) continue;

		if (results) {
			results[count].rid = area->self_rid;
			results[count].collider_id = ObjectID(oid);
			results[count].collider = collider;
			results[count].shape = 0;
		}
		count++;
	}
	return count;
}

bool HopDirectSpaceState::_intersect_ray(const Vector3 &p_from, const Vector3 &p_to, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_hit_from_inside, bool p_hit_back_faces, bool p_pick_ray, PhysicsServer3DExtensionRayResult *p_result) {
	if (!space || !p_collide_with_bodies) return false;

	hop::segment<hop_scalar> seg;
	seg.set_start_end(to_hop(p_from), to_hop(p_to));

	hop::collision<hop_scalar> result;
	space->simulator->trace_segment(result, seg, p_collision_mask);

	if (to_godot_float(result.time) >= 1.0f) return false;
	if (!p_hit_from_inside && to_godot_float(result.time) <= 0.0f) return false;

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
	if (!space || p_max_results <= 0) return 0;
	if (!p_collide_with_bodies && !p_collide_with_areas) return 0;

	auto hp = to_hop(p_position);
	auto eps = to_hop_scalar(0.001f);
	hop::aa_box<hop_scalar> box(
		hop::vec3<hop_scalar>(hp.x - eps, hp.y - eps, hp.z - eps),
		hop::vec3<hop_scalar>(hp.x + eps, hp.y + eps, hp.z + eps)
	);

	int result_count = 0;

	if (p_collide_with_bodies) {
		std::vector<hop::solid<hop_scalar> *> found(p_max_results * 2);
		int count = space->simulator->find_solids_in_aa_box(box, found.data(), (int)found.size());

		hop::segment<hop_scalar> seg;
		seg.set_start_end(hp, hp);

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
	}

	if (p_collide_with_areas && server && result_count < p_max_results) {
		// Narrow-phase confirm using hop's native point-query form: a zero-length
		// segment at the point.  test_segment dispatches per shape type (box/sphere/
		// capsule/convex_solid/traceable) and reports t==0 when the point is inside —
		// no AABB assumptions, unlike a bounding-box containment test which false-
		// positives on thin or concave brushes (e.g. shallow water, whose bounding
		// box towers over the actual surface).
		hop::segment<hop_scalar> zseg;
		zseg.set_start_end(hp, hp);

		result_count = collect_overlapping_areas(space, box, p_collision_mask, p_results, result_count, p_max_results,
			[&](hop::solid<hop_scalar> *area_solid) -> bool {
				hop::collision<hop_scalar> col;
				space->simulator->test_segment(col, zseg, area_solid);
				return to_godot_float(col.time) < 1.0f;
			});
	}

	return result_count;
}

int32_t HopDirectSpaceState::_intersect_shape(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeResult *p_results, int32_t p_max_results) {
	if (!space || !server || p_max_results <= 0) return 0;
	if (!p_collide_with_bodies && !p_collide_with_areas) return 0;

	HopShapeData *sd = server->shape_owner.get_or_null(p_shape_rid);
	if (!sd) return 0;

	// Separation ray: fire a segment rather than sweeping a solid shape.
	// The ray fires along p_motion if given, otherwise straight down by the ray's length.
	if (sd->type == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
		Dictionary ray_data = sd->data;
		float length = ray_data.get("length", 1.0f);
		Vector3 dir = p_motion.length_squared() > 0.0f ? p_motion : Vector3(0.0f, -length, 0.0f);

		hop::segment<hop_scalar> seg;
		seg.set_start_end(to_hop(p_transform.origin), to_hop(p_transform.origin + dir));

		hop::collision<hop_scalar> result;
		space->simulator->trace_segment(result, seg, p_collision_mask);

		if (to_godot_float(result.time) >= 1.0f) return 0;

		if (p_results) {
			if (result.collidee) {
				HopBodyData *hit = static_cast<HopBodyData *>(result.collidee->get_user_data());
				if (hit) {
					p_results[0].rid = hit->self_rid;
					p_results[0].collider_id = ObjectID(hit->object_instance_id);
					p_results[0].collider = get_collider_safe(hit->object_instance_id);
					p_results[0].shape = 0;
				}
			}
		}
		return 1;
	}

	auto pos = to_hop(p_transform.origin);
	int result_count = 0;

	// --- Bodies ---
	if (p_collide_with_bodies) {
		auto hs = sd->make_hop_shape(Transform3D());
		if (hs) {
			// Build a temporary solid carrying the test shape.
			// Only collide with static bodies — kinematic and dynamic bodies (e.g. the
			// crouched player capsule) must not be counted as blockers.  _intersect_shape
			// does not receive the caller's exclude list, so without this filter the
			// player's own body would always register as an overlap and can_uncrouch()
			// would permanently return false.
			auto temp_solid = std::make_shared<hop::solid<hop_scalar>>();
			temp_solid->set_infinite_mass();
			temp_solid->set_position(pos);
			temp_solid->set_collision_scope(0);
			temp_solid->set_collide_with_scope(p_collision_mask);
			temp_solid->add_shape(hs);
			temp_solid->set_collision_filter([](hop::solid<hop_scalar> *other) -> bool {
				auto *body = static_cast<HopBodyData *>(other->get_user_data());
				return body && body->mode == PhysicsServer3D::BODY_MODE_STATIC;
			});

			space->simulator->add_solid(temp_solid);

			// Perform a narrow-phase overlap test via a zero-direction sweep.
			// All hop primitives return t=0 when the solid's origin is already inside
			// (aa_box/sphere/capsule via test_inside; trimesh via the static overlap
			// path in HopTrimeshTraceable::trace_solid).
			hop::segment<hop_scalar> seg;
			seg.set_start_end(pos, pos);

			hop::collision<hop_scalar> result;
			space->simulator->trace_solid(result, temp_solid.get(), seg, p_collision_mask);

			space->simulator->remove_solid(temp_solid);

			if (to_godot_float(result.time) < 1.0f && result_count < p_max_results) {
				// At least one static solid overlaps the test shape.
				if (p_results) {
					p_results[result_count].rid = RID();
					p_results[result_count].collider_id = ObjectID();
					p_results[result_count].collider = nullptr;
					p_results[result_count].shape = 0;
					// result.collidee is null for trimesh hits — that's fine; the
					// only current body caller (can_uncrouch) checks is_empty().
					if (result.collidee) {
						HopBodyData *hit = static_cast<HopBodyData *>(result.collidee->get_user_data());
						if (hit) {
							p_results[result_count].rid = hit->self_rid;
							p_results[result_count].collider_id = ObjectID(hit->object_instance_id);
							p_results[result_count].collider = get_collider_safe(hit->object_instance_id);
						}
					}
				}
				result_count++;
			}
		}
	}

	// --- Areas ---
	// Water / ladder volumes are Area3D, each backed by a persistent sensor solid
	// (built once).  Test the query shape against each candidate area's solid with
	// the narrow phase — no per-query geometry rebuilds.  The player probes these
	// with collide_with_areas = true, collide_with_bodies = false.
	if (p_collide_with_areas && result_count < p_max_results) {
		auto query_shape = sd->make_hop_shape(Transform3D());
		if (query_shape) {
			auto query_solid = std::make_shared<hop::solid<hop_scalar>>();
			query_solid->set_infinite_mass();
			query_solid->set_position(pos);
			query_solid->add_shape(query_shape);

			hop::segment<hop_scalar> zseg;
			zseg.set_start_end(pos, pos);

			hop::aa_box<hop_scalar> query_box = query_solid->get_world_bound();
			result_count = collect_overlapping_areas(space, query_box, p_collision_mask, p_results, result_count, p_max_results,
				[&](hop::solid<hop_scalar> *area_solid) -> bool {
					hop::collision<hop_scalar> col;
					space->simulator->test_solid(col, query_solid.get(), zseg, area_solid);
					return to_godot_float(col.time) < 1.0f;
				});
		}
	}

	return result_count;
}

bool HopDirectSpaceState::_cast_motion(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, float *p_closest_safe, float *p_closest_unsafe, PhysicsServer3DExtensionShapeRestInfo *p_info) {
	if (!space || !server) return false;

	HopShapeData *sd = server->shape_owner.get_or_null(p_shape_rid);
	if (!sd) return false;

	// Separation ray: fire a segment rather than sweeping a solid shape.
	// The ray fires along p_motion if given, otherwise straight down by the ray's length.
	if (sd->type == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
		Dictionary ray_data = sd->data;
		float length = ray_data.get("length", 1.0f);
		Vector3 dir = p_motion.length_squared() > 0.0f ? p_motion : Vector3(0.0f, -length, 0.0f);

		hop::segment<hop_scalar> seg;
		seg.set_start_end(to_hop(p_transform.origin), to_hop(p_transform.origin + dir));

		hop::collision<hop_scalar> result;
		space->simulator->trace_segment(result, seg, p_collision_mask);

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
		}
		return time_f < 1.0f;
	}

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
