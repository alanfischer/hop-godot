#pragma once

#include <hop/hop.h>
#include <type_traits>
#include <vector>

#include "hop_triangle_collision.h"

// A traceable implementation for triangle meshes.  Stores triangles as
// vertices + index triplets and accelerates queries with a BVH over
// per-triangle AABBs.

template <typename T>
class HopTrimeshTraceable : public hop::traceable<T> {
	using tr = hop::scalar_traits<T>;

public:
	struct triangle {
		int i0, i1, i2;
	};

	void build(const hop::vec3<T> * verts, int vert_count,
	           const triangle * tris, int tri_count,
	           bool backface_collision = false) {
		backface_collision_ = backface_collision;
		verts_.assign(verts, verts + vert_count);
		tris_.assign(tris, tris + tri_count);

		// Precompute per-triangle normals
		normals_.resize(tri_count);
		for (int i = 0; i < tri_count; ++i) {
			auto & t = tris_[i];
			hop::vec3<T> e1, e2;
			hop::sub(e1, verts_[t.i1], verts_[t.i0]);
			hop::sub(e2, verts_[t.i2], verts_[t.i0]);
			hop::cross(normals_[i], e1, e2);
			hop::normalize_carefully(normals_[i], T {});
		}

		// Build BVH over triangle AABBs
		std::vector<std::pair<hop::aa_box<T>, int>> entries;
		entries.reserve(tri_count);
		for (int i = 0; i < tri_count; ++i) {
			hop::aa_box<T> box;
			tri_aabb(i, box);
			entries.push_back({ box, i });
		}
		bvh_.build(entries);

		// Cache total bound
		if (!bvh_.empty()) {
			total_bound_ = bvh_.get_nodes()[0].box;
		}
	}

	void get_bound(hop::aa_box<T> & result) override {
		result = total_bound_;
	}

	void trace_segment(hop::collision<T> & result,
	                   const hop::vec3<T> & position,
	                   const hop::mat3<T> & orientation,
	                   const hop::segment<T> & seg) override {
		// Transform the ray into mesh-local space (mesh verts never move). For a
		// rotated mesh that's a rotate by Rᵀ about `position`, with the hit mapped
		// back by R; the common identity case is just the translate, with no matrix
		// math on the hot raycast path.
		static const hop::mat3<T> identity;
		const bool rotated = (orientation != identity);
		hop::mat3<T> Rt;
		hop::segment<T> local_seg;
		if (rotated) {
			hop::transpose(Rt, orientation);
			hop::vec3<T> rel;
			hop::sub(rel, seg.origin, position);
			hop::mul(local_seg.origin, Rt, rel);
			hop::mul(local_seg.direction, Rt, seg.direction);
		} else {
			hop::sub(local_seg.origin, seg.origin, position);
			local_seg.direction = seg.direction;
		}

		bvh_.query_ray(local_seg.origin, local_seg.direction, [&](int tri_idx, T & best_t) {
			const auto & t_ = tris_[tri_idx];
			hop::vec3<T> point, normal;
			T t = hoptri::ray_triangle(local_seg, verts_[t_.i0], verts_[t_.i1], verts_[t_.i2],
			                           normals_[tri_idx], point, normal);
			if (t < best_t && t < result.time) {
				best_t = t;
				result.time = t;
				if (rotated) {
					hop::mul(result.point, orientation, point);
					hop::add(result.point, position);
					hop::mul(result.normal, orientation, normal);
				} else {
					hop::add(result.point, point, position);
					result.normal = normal;
				}
			}
		});
	}

	void trace_solid(hop::collision<T> & result,
	                 hop::solid<T> * s,
	                 const hop::vec3<T> & position,
	                 const hop::mat3<T> & orientation,
	                 const hop::segment<T> & seg, T margin) override {
		static const hop::mat3<T> identity;
		if (orientation != identity) {
			// Rotated mesh: the shared driver handles the frame transform + mover
			// spine reduction; we supply only the BVH broadphase over the local AABB.
			hoptri::trace_solid_rotated(result, s, position, orientation, seg, margin, seam_tol_,
			    [&](const hop::aa_box<T> & q, auto && visit) {
				    bvh_.query_aabb(q, [&](int tri_idx) {
					    const auto & tri = tris_[tri_idx];
					    visit(verts_[tri.i0], verts_[tri.i1], verts_[tri.i2], normals_[tri_idx]);
				    });
			    });
			return;
		}
		// seg.origin = other solid's position, seg.direction = its movement
		// position = traceable's solid position
		//
		// The player is a CAPSULE; for capsule shapes we do real capsule-segment-
		// vs-triangle CLOSEST-POINT contact (Godot's approach), which fixes the two
		// bugs the old center+plane approximation couldn't:
		//   * Recovery pushes out along the triangle's FRONT face, by the true
		//     penetration, and never flips downward when the body sinks below a
		//     floor (the basement fall-through). One-sided front gating rejects the
		//     coincident back-wound twin on the BSP's mixed-winding ramps.
		//   * The swept cast is distance-based (two-sided by nature), so it stops
		//     the capsule at the surface from whichever side it approaches — no
		//     holes on the mixed-winding ramp — keeping penetration shallow so the
		//     recovery only ever sees the clean case.
		// Non-capsule shapes (boxes/spheres, e.g. area queries) keep the older
		// winding-agnostic support-plane approximation.

		// Compute swept AABB of the other solid along the segment for BVH query
		hop::aa_box<T> swept_box;
		const auto & shapes = s->get_shapes();
		shapes[0]->get_bound(swept_box);
		for (size_t i = 1; i < shapes.size(); ++i) {
			hop::aa_box<T> sb;
			shapes[i]->get_bound(sb);
			swept_box.merge(sb);
		}

		// swept_box is in solid-local space; expand to cover the full sweep path
		// Work in mesh-local space (subtract traceable position)
		hop::vec3<T> local_origin;
		hop::sub(local_origin, seg.origin, position);

		hop::aa_box<T> query_box;
		query_box.mins.x = local_origin.x + swept_box.mins.x;
		query_box.mins.y = local_origin.y + swept_box.mins.y;
		query_box.mins.z = local_origin.z + swept_box.mins.z;
		query_box.maxs.x = local_origin.x + swept_box.maxs.x;
		query_box.maxs.y = local_origin.y + swept_box.maxs.y;
		query_box.maxs.z = local_origin.z + swept_box.maxs.z;

		// Zero-direction static-overlap (recovery) path. The swept cast's
		// "moving-into" guard skips everything when the direction is zero, so the
		// depenetration query is handled separately here.
		if (hop::dot(seg.direction, seg.direction) == T{}) {
			bool found = false;
			T best_depth = T{};
			bvh_.query_aabb(query_box, [&](int tri_idx) {
				const auto & tri = tris_[tri_idx];
				for (const auto & sh_ptr : s->get_shapes()) {
					hoptri::recover_shape_vs_triangle(sh_ptr.get(), local_origin, seg, margin, seam_tol_,
					    verts_[tri.i0], verts_[tri.i1], verts_[tri.i2], normals_[tri_idx],
					    result, found, best_depth);
				}
			});
			return;
		}

		// Expand by endpoint
		hop::vec3<T> end;
		end.x = local_origin.x + seg.direction.x;
		end.y = local_origin.y + seg.direction.y;
		end.z = local_origin.z + seg.direction.z;
		hop::aa_box<T> end_box;
		end_box.mins.x = end.x + swept_box.mins.x;
		end_box.mins.y = end.y + swept_box.mins.y;
		end_box.mins.z = end.z + swept_box.mins.z;
		end_box.maxs.x = end.x + swept_box.maxs.x;
		end_box.maxs.y = end.y + swept_box.maxs.y;
		end_box.maxs.z = end.z + swept_box.maxs.z;
		query_box.merge(end_box);

		bvh_.query_aabb(query_box, [&](int tri_idx) {
			const auto & tri = tris_[tri_idx];
			for (const auto & sh_ptr : s->get_shapes()) {
				hoptri::sweep_shape_vs_triangle(sh_ptr.get(), local_origin, seg, margin, seam_tol_,
				    verts_[tri.i0], verts_[tri.i1], verts_[tri.i2], normals_[tri_idx], result);
			}
		});
	}

private:
	std::vector<hop::vec3<T>> verts_;
	std::vector<triangle> tris_;
	std::vector<hop::vec3<T>> normals_;
	hop::bvh<T, int> bvh_;
	hop::aa_box<T> total_bound_;
	bool backface_collision_ = false;
	// World-space tolerance for accepting a contact just off a triangle, so
	// hairline seam / T-junction gaps can't tunnel a swept step. Used only by the
	// non-capsule (box/sphere) support-plane fallback; the capsule closest-point
	// path is distance-based and bridges seams up to the capsule radius for free.
	// ~1cm at the game's scale: bridges sub-cm mesh seams without papering over
	// real holes.
	T seam_tol_ = tr::from_int(1) / tr::from_int(100);

	void tri_aabb(int idx, hop::aa_box<T> & box) const {
		auto & t = tris_[idx];
		box.mins = verts_[t.i0];
		box.maxs = verts_[t.i0];
		box.merge(verts_[t.i1]);
		box.merge(verts_[t.i2]);
		// Small epsilon expansion so zero-thickness (axis-aligned) triangles
		// are reliably found by the BVH even when the query box just grazes them.
		const T eps = tr::from_milli(1);
		box.mins.x -= eps; box.mins.y -= eps; box.mins.z -= eps;
		box.maxs.x += eps; box.maxs.y += eps; box.maxs.z += eps;
	}

};
