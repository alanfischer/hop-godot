#pragma once

#include <godot_cpp/classes/physics_direct_space_state3d_extension.hpp>

#include "hop_space_data.h"

using namespace godot;

class HopPhysicsServer;

class HopDirectSpaceState : public PhysicsDirectSpaceState3DExtension {
	GDCLASS(HopDirectSpaceState, PhysicsDirectSpaceState3DExtension);

protected:
	static void _bind_methods() {}

public:
	HopSpaceData *space = nullptr;
	HopPhysicsServer *server = nullptr;

	bool _intersect_ray(const Vector3 &p_from, const Vector3 &p_to, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_hit_from_inside, bool p_hit_back_faces, bool p_pick_ray, PhysicsServer3DExtensionRayResult *p_result) override;
	int32_t _intersect_point(const Vector3 &p_position, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeResult *p_results, int32_t p_max_results) override;
	int32_t _intersect_shape(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeResult *p_result_count, int32_t p_max_results) override;
	bool _cast_motion(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, float *p_closest_safe, float *p_closest_unsafe, PhysicsServer3DExtensionShapeRestInfo *p_info) override;
	bool _collide_shape(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, void *p_results, int32_t p_max_results, int32_t *p_result_count) override;
	bool _rest_info(const RID &p_shape_rid, const Transform3D &p_transform, const Vector3 &p_motion, float p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, PhysicsServer3DExtensionShapeRestInfo *p_rest_info) override;
	Vector3 _get_closest_point_to_object_volume(const RID &p_object, const Vector3 &p_point) const override;
};
