#pragma once

#include <hop/hop.h>
#include <vector>

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
	                 const hop::segment<T> & seg, T margin) override {
		// seg.origin = other solid's position, seg.direction = its movement
		// position = traceable's solid position
		// We need to find when the swept solid first contacts the mesh.
		//
		// For each triangle, we expand the triangle along its inward normal
		// by the other solid's support extent in that direction (Minkowski sum).
		// Then we trace the segment origin against the expanded plane.

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

		// Zero-direction static overlap path.
		// The sweep loop's `denom >= 0` guard skips every triangle when direction
		// is zero (denom = dot(face_n, 0) == 0).  Handle this case separately:
		// a solid overlaps a triangle when its Minkowski-expanded plane is already
		// crossed, i.e. dot(face_n, local_origin) <= plane_d + expand.
		if (hop::dot(seg.direction, seg.direction) == T{}) {
			// Find the NEAREST overlapping surface to push out of (minimum-translation
			// depenetration). Triangle normals are oriented toward the body, so this
			// is winding-agnostic — matching Godot's concave collision, which is
			// double-sided. The BSP emits coincident inverted (back-facing) triangles
			// on walkable surfaces; the old "deepest face, raw winding" logic latched
			// the inverted one and reported a bogus ~2*extent penetration with an
			// inward normal, shoving the body straight through ramps/floors.
			bool found = false;
			T best_depth = T{};
			bvh_.query_aabb(query_box, [&](int tri_idx) {
				auto & tri = tris_[tri_idx];
				auto & n   = normals_[tri_idx];

				// Orient this triangle's normal toward the body's reference point so
				// the contact normal points the way the body must move to separate,
				// regardless of triangle winding.
				hop::vec3<T> face_n = n;
				if (hop::dot(n, local_origin) - hop::dot(n, verts_[tri.i0]) < T{})
					hop::neg(face_n, n);

				for (const auto & sh_ptr : s->get_shapes()) {
					auto * sh = sh_ptr.get();

					hop::vec3<T> neg_fn;
					hop::neg(neg_fn, face_n);
					hop::vec3<T> sup;
					hop::support(sup, *sh, neg_fn);
					T expand = hop::dot(sup, neg_fn);

					T plane_d   = hop::dot(face_n, verts_[tri.i0]);
					// The Minkowski-expanded contact surface for this shape.
					T surface_d = plane_d + expand;
					// Inflate outward by the speculative margin so near-resting
					// solids within `margin` register as overlapping (t=0).
					T expanded_d = surface_d + margin;

					T along = hop::dot(face_n, local_origin);
					// Not within the inflated surface for this face — skip.
					if (along > expanded_d) continue;

					// Project local_origin onto the original triangle plane and
					// check if it lies inside the triangle. dist >= 0 by orientation.
					T dist = along - plane_d;
					hop::vec3<T> proj, n_scaled;
					hop::mul(n_scaled, face_n, dist);
					hop::sub(proj, local_origin, n_scaled);

					if (point_near_triangle(proj, tri_idx, seam_tol_)) {
						// Positive when the solid is buried past the contact
						// surface; clamped so a grazing touch reports zero.
						T depth = surface_d - along;
						if (depth < T{}) depth = T{};

						// Push out of the NEAREST surface (minimum depth), not the
						// deepest — picking the deepest on a thin/coincident surface
						// ejects the body the wrong way (through the floor).
						if (!found || depth < best_depth) {
							best_depth    = depth;
							result.time   = T{};
							result.depth  = depth;
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

			{
				// Winding-agnostic, like Godot's concave collision: orient this
				// triangle's normal toward the body's reference point so the contact
				// normal is STABLE regardless of winding. The BSP emits coincident
				// opposite-wound triangles on walkable surfaces; testing raw front/back
				// faces makes the swept normal flip ± between sub-steps, so
				// velocity.slide() over-cancels and the body catches/stops on the ramp.
				hop::vec3<T> face_n = n;
				auto & v0o = verts_[tri.i0];
				if (hop::dot(n, local_origin) - hop::dot(n, v0o) < T {})
					hop::neg(face_n, n);

				T denom = hop::dot(face_n, seg.direction);
				if (denom >= T {})
					return; // Body not moving into this (body-facing) surface — slide freely

				// For each shape on the solid, compute how far to expand
				// the triangle plane along its normal using support()
				for (const auto & sh_ptr : s->get_shapes()) {
					auto * sh = sh_ptr.get();

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

					if (point_near_triangle(proj, tri_idx, seam_tol_)) {
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
	// World-space tolerance for accepting a contact just off a triangle, so
	// hairline seam / T-junction gaps between triangles can't tunnel a swept
	// step (the rare "fall through the floor into the void" case). ~1cm at the
	// game's scale: bridges sub-cm mesh seams without papering over real holes.
	T seam_tol_ = T(1) / T(100);


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

	// True if p is inside the triangle, OR within `tol` world units of it.
	// Bridges hairline seam / T-junction gaps in the mesh so a swept step can't
	// slip between two triangles and tunnel through the surface. The exact
	// closest-point test (Ericson, Real-Time Collision Detection) runs only on
	// the fail path, so the common in-triangle case stays cheap.
	bool point_near_triangle(const hop::vec3<T> & p, int tri_idx, T tol) const {
		if (point_in_triangle(p, tri_idx))
			return true;
		auto & t = tris_[tri_idx];
		const auto & a = verts_[t.i0];
		const auto & b = verts_[t.i1];
		const auto & c = verts_[t.i2];
		const T zero {};
		hop::vec3<T> ab, ac, ap;
		hop::sub(ab, b, a); hop::sub(ac, c, a); hop::sub(ap, p, a);
		T d1 = hop::dot(ab, ap), d2 = hop::dot(ac, ap);
		hop::vec3<T> closest, tmp;
		if (d1 <= zero && d2 <= zero) {
			closest = a;                                  // vertex region A
		} else {
			hop::vec3<T> bp; hop::sub(bp, p, b);
			T d3 = hop::dot(ab, bp), d4 = hop::dot(ac, bp);
			if (d3 >= zero && d4 <= d3) {
				closest = b;                              // vertex region B
			} else {
				T vc = d1 * d4 - d3 * d2;
				if (vc <= zero && d1 >= zero && d3 <= zero) {
					T v = d1 / (d1 - d3);                 // edge AB
					hop::mul(tmp, ab, v); hop::add(closest, a, tmp);
				} else {
					hop::vec3<T> cp; hop::sub(cp, p, c);
					T d5 = hop::dot(ab, cp), d6 = hop::dot(ac, cp);
					if (d6 >= zero && d5 <= d6) {
						closest = c;                      // vertex region C
					} else {
						T vb = d5 * d2 - d1 * d6;
						if (vb <= zero && d2 >= zero && d6 <= zero) {
							T w = d2 / (d2 - d6);         // edge AC
							hop::mul(tmp, ac, w); hop::add(closest, a, tmp);
						} else {
							T va = d3 * d6 - d5 * d4;
							if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero) {
								T w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); // edge BC
								hop::vec3<T> bc; hop::sub(bc, c, b);
								hop::mul(tmp, bc, w); hop::add(closest, b, tmp);
							} else {
								T denom = tr::one() / (va + vb + vc); // face interior
								T v = vb * denom, w = vc * denom;
								hop::mul(tmp, ab, v); hop::add(closest, a, tmp);
								hop::mul(tmp, ac, w); hop::add(closest, tmp);
							}
						}
					}
				}
			}
		}
		hop::vec3<T> diff; hop::sub(diff, p, closest);
		return hop::length_squared(diff) <= tol * tol;
	}
};
