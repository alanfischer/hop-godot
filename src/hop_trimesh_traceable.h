#pragma once

#include <hop/hop.h>
#include <vector>
#include "hop_conversions.h"

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
	                   const hop::segment<T> & seg) override {
		// Transform segment into mesh-local space (subtract traceable position)
		hop::segment<T> local_seg;
		hop::sub(local_seg.origin, seg.origin, position);
		local_seg.direction = seg.direction;

		bvh_.query_ray(local_seg.origin, local_seg.direction, [&](int tri_idx, T & best_t) {
			hop::vec3<T> point, normal;
			T t = ray_triangle(local_seg, tri_idx, point, normal);
			if (t < best_t && t < result.time) {
				best_t = t;
				result.time = t;
				// Convert hit point back to world space
				hop::add(result.point, point, position);
				result.normal = normal;
			}
		});
	}

	void trace_solid(hop::collision<T> & result,
	                 hop::solid<T> * s,
	                 const hop::vec3<T> & position,
	                 const hop::segment<T> & seg) override {
		// seg.origin = other solid's position, seg.direction = its movement
		// position = traceable's solid position
		// We need to find when the swept solid first contacts the mesh.
		//
		// For each triangle, we expand the triangle along its inward normal
		// by the other solid's support extent in that direction (Minkowski sum).
		// Then we trace the segment origin against the expanded plane.

		// Compute swept AABB of the other solid along the segment for BVH query
		hop::aa_box<T> swept_box;
		s->get_shape(0)->get_bound(swept_box);
		for (int i = 1; i < s->get_num_shapes(); ++i) {
			hop::aa_box<T> sb;
			s->get_shape(i)->get_bound(sb);
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

		// Zero-direction static overlap path.
		// The sweep loop's `denom >= 0` guard skips every triangle when direction
		// is zero (denom = dot(face_n, 0) == 0).  Handle this case separately:
		// a solid overlaps a triangle when its Minkowski-expanded plane is already
		// crossed, i.e. dot(face_n, local_origin) <= plane_d + expand.
		if (hop::dot(seg.direction, seg.direction) == T{}) {
			bool found = false;
			bvh_.query_aabb(query_box, [&](int tri_idx) {
				if (found) return;
				auto & tri = tris_[tri_idx];
				auto & n   = normals_[tri_idx];

				int num_sides = backface_collision_ ? 2 : 1;
				for (int side = 0; side < num_sides && !found; ++side) {
					hop::vec3<T> face_n = n;
					if (side == 1) hop::neg(face_n, n);

					for (int si = 0; si < s->get_num_shapes() && !found; ++si) {
						auto * sh = s->get_shape(si);

						hop::vec3<T> neg_fn;
						hop::neg(neg_fn, face_n);
						hop::vec3<T> sup;
						hop::support(sup, *sh, neg_fn);
						T expand = hop::dot(sup, neg_fn);

						T plane_d    = hop::dot(face_n, verts_[tri.i0]);
						T expanded_d = plane_d + expand;

						// Not penetrating this face — skip.
						if (hop::dot(face_n, local_origin) > expanded_d) continue;

						// Project local_origin onto the original triangle plane and
						// check if it lies inside the triangle.
						T dist = hop::dot(face_n, local_origin) - plane_d;
						hop::vec3<T> proj, n_scaled;
						hop::mul(n_scaled, face_n, dist);
						hop::sub(proj, local_origin, n_scaled);

						if (point_in_triangle(proj, tri_idx)) {
							result.time   = T{};
							result.normal = face_n;
							result.point  = seg.origin;
							found = true;
						}
					}
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
			auto & tri = tris_[tri_idx];
			auto & n = normals_[tri_idx];

			// Test front face, and optionally the back face
			int num_sides = backface_collision_ ? 2 : 1;
			for (int side = 0; side < num_sides; ++side) {
				hop::vec3<T> face_n;
				if (side == 0) {
					face_n = n;
				} else {
					hop::neg(face_n, n);
				}

				T denom = hop::dot(face_n, seg.direction);
				if (denom >= T {})
					continue; // Moving away or parallel

				// For each shape on the solid, compute how far to expand
				// the triangle plane along its normal using support()
				for (int si = 0; si < s->get_num_shapes(); ++si) {
					auto * sh = s->get_shape(si);

					// support() in the negative-normal direction gives the point on
					// the shape furthest into the triangle plane
					hop::vec3<T> neg_fn;
					hop::neg(neg_fn, face_n);
					hop::vec3<T> sup;
					hop::support(sup, *sh, neg_fn);

					// The expansion distance: how far the support point extends
					// along the negative normal from the shape's local origin
					T expand = hop::dot(sup, neg_fn);

					// Build the expanded plane in mesh-local space
					auto & v0 = verts_[tri.i0];
					T plane_d = hop::dot(face_n, v0);
					// Expand the plane outward by the shape's extent
					T expanded_d = plane_d + expand;

					T t = (expanded_d - hop::dot(face_n, local_origin)) / denom;
					if (t > tr::one() || t >= result.time)
						continue;
					// If t < 0 the body center is already inside the expanded plane
					// (penetrating). Clamp to 0 so the simulator's depenetration
					// logic can push the body back out; skipping it entirely lets
					// the body sink further through the mesh each frame.
					if (t < T {})
						t = T {};

					// Compute hit point on the expanded plane
					hop::vec3<T> hit;
					hop::mul(hit, seg.direction, t);
					hop::add(hit, local_origin);

					// Project hit point onto the original triangle plane and
					// check if it's inside the triangle (barycentric test)
					// The hit is on the expanded plane, offset by 'expand' from
					// the original. Project back to the original plane along normal.
					hop::vec3<T> proj;
					T offset = hop::dot(face_n, hit) - plane_d;
					hop::vec3<T> n_scaled;
					hop::mul(n_scaled, face_n, offset);
					hop::sub(proj, hit, n_scaled);

					if (point_in_triangle(proj, tri_idx)) {
						result.time = t;
						hop::mul(result.point, seg.direction, t);
						hop::add(result.point, seg.origin);
						result.normal = face_n;
					}
				}
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


	void tri_aabb(int idx, hop::aa_box<T> & box) const {
		auto & t = tris_[idx];
		box.mins = verts_[t.i0];
		box.maxs = verts_[t.i0];
		box.merge(verts_[t.i1]);
		box.merge(verts_[t.i2]);
		// Small epsilon expansion so zero-thickness (axis-aligned) triangles
		// are reliably found by the BVH even when the query box just grazes them.
		const T eps = T(1e-3f);
		box.mins.x -= eps; box.mins.y -= eps; box.mins.z -= eps;
		box.maxs.x += eps; box.maxs.y += eps; box.maxs.z += eps;
	}

	// Ray-triangle intersection (Möller–Trumbore)
	T ray_triangle(const hop::segment<T> & seg, int tri_idx,
	               hop::vec3<T> & point, hop::vec3<T> & normal) const {
		auto & t = tris_[tri_idx];
		auto & v0 = verts_[t.i0];
		auto & v1 = verts_[t.i1];
		auto & v2 = verts_[t.i2];

		hop::vec3<T> e1, e2;
		hop::sub(e1, v1, v0);
		hop::sub(e2, v2, v0);

		hop::vec3<T> h;
		hop::cross(h, seg.direction, e2);
		T a = hop::dot(e1, h);

		T zero_val {};
		T eps;
		if constexpr (std::is_same_v<T, hop::fixed16>)
			eps = hop::fixed16::from_raw(1);
		else
			eps = T(1e-7);

		if (a > -eps && a < eps)
			return tr::one();

		T inv_a = tr::one() / a;
		hop::vec3<T> s_vec;
		hop::sub(s_vec, seg.origin, v0);

		T u = hop::dot(s_vec, h) * inv_a;
		if (u < zero_val || u > tr::one())
			return tr::one();

		hop::vec3<T> q;
		hop::cross(q, s_vec, e1);

		T v = hop::dot(seg.direction, q) * inv_a;
		if (v < zero_val || (u + v) > tr::one())
			return tr::one();

		T time = hop::dot(e2, q) * inv_a;
		if (time < zero_val || time > tr::one())
			return tr::one();

		hop::mul(point, seg.direction, time);
		hop::add(point, seg.origin);
		normal = normals_[tri_idx];

		// Ensure normal faces against ray direction
		if (hop::dot(normal, seg.direction) > zero_val)
			hop::neg(normal);

		return time;
	}

	// Barycentric point-in-triangle test
	bool point_in_triangle(const hop::vec3<T> & p, int tri_idx) const {
		auto & t = tris_[tri_idx];
		auto & v0 = verts_[t.i0];
		auto & v1 = verts_[t.i1];
		auto & v2 = verts_[t.i2];

		hop::vec3<T> e0, e1, ep;
		hop::sub(e0, v1, v0);
		hop::sub(e1, v2, v0);
		hop::sub(ep, p, v0);

		T d00 = hop::dot(e0, e0);
		T d01 = hop::dot(e0, e1);
		T d11 = hop::dot(e1, e1);
		T d20 = hop::dot(ep, e0);
		T d21 = hop::dot(ep, e1);

		T denom = d00 * d11 - d01 * d01;
		T zero_val {};
		if (denom == zero_val)
			return false;

		T inv_denom = tr::one() / denom;
		T u = (d11 * d20 - d01 * d21) * inv_denom;
		T v = (d00 * d21 - d01 * d20) * inv_denom;

		T eps;
		if constexpr (std::is_same_v<T, hop::fixed16>)
			eps = -hop::fixed16::from_raw(1 << 4);
		else
			eps = T(-1e-4);

		return u >= eps && v >= eps && (u + v) <= (tr::one() - eps);
	}
};
