#pragma once

#include <hop/hop.h>
#include <type_traits>
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
				for (const auto & sh_ptr : s->get_shapes()) {
					auto * sh = sh_ptr.get();
					if (sh->get_type() == hop::shape_type::capsule) {
						hop::vec3<T> base;
						hop::add(base, local_origin, sh->get_local_position());
						const auto & cap = sh->get_capsule();
						T depth;
						hop::vec3<T> n;
						if (recover_capsule_triangle(tri_idx, cap, base, margin, depth, n)) {
							// One-sided front gating already rejects the inverted twin,
							// so escaping the DEEPEST front contact first is safe (and
							// resolves corners better than nearest-first). A resting
							// touch (depth 0) still registers t=0 with the up normal.
							if (!found || depth > best_depth) {
								best_depth    = depth;
								result.time   = T{};
								result.depth  = depth;
								result.normal = n;
								result.point  = seg.origin;
								found = true;
							}
						}
					} else {
						recover_support_plane(tri_idx, sh, local_origin, seg,
						                      margin, result, found, best_depth);
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
			for (const auto & sh_ptr : s->get_shapes()) {
				auto * sh = sh_ptr.get();
				if (sh->get_type() == hop::shape_type::capsule) {
					hop::vec3<T> base, p1, p2;
					hop::add(base, local_origin, sh->get_local_position());
					const auto & cap = sh->get_capsule();
					hop::add(p1, base, cap.origin);
					hop::add(p2, p1, cap.direction);
					T toi;
					hop::vec3<T> n;
					if (sweep_capsule_triangle(tri_idx, p1, p2, seg.direction,
					                           cap.radius, margin, result.time, toi, n)) {
						result.time = toi;
						hop::mul(result.point, seg.direction, toi);
						hop::add(result.point, seg.origin);
						result.normal = n;
					}
				} else {
					sweep_support_plane(tri_idx, sh, local_origin, seg, result);
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
	// hairline seam / T-junction gaps can't tunnel a swept step. Used only by the
	// non-capsule (box/sphere) support-plane fallback; the capsule closest-point
	// path is distance-based and bridges seams up to the capsule radius for free.
	// ~1cm at the game's scale: bridges sub-cm mesh seams without papering over
	// real holes.
	T seam_tol_ = T(1) / T(100);


	// A small distance below which we treat two points as coincident (avoids
	// dividing by a ~zero length when forming a contact normal).
	static T contact_eps() {
		if constexpr (std::is_same_v<T, hop::fixed16>)
			return hop::fixed16::from_raw(16);
		else if constexpr (std::is_same_v<T, hop::fixed32>)
			return hop::fixed32::from_raw(int64_t(1) << 8);
		else
			return T(1e-6);
	}

	static T clamp01(T v) {
		if (v < T{}) return T{};
		if (v > tr::one()) return tr::one();
		return v;
	}

	// Closest point on triangle (a,b,c) to point p. Ericson, Real-Time Collision
	// Detection §5.1.5 (Voronoi-region test).
	static hop::vec3<T> closest_pt_point_triangle(const hop::vec3<T> & p,
	        const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c) {
		const T zero{};
		hop::vec3<T> ab, ac, ap, tmp, closest;
		hop::sub(ab, b, a); hop::sub(ac, c, a); hop::sub(ap, p, a);
		T d1 = hop::dot(ab, ap), d2 = hop::dot(ac, ap);
		if (d1 <= zero && d2 <= zero) return a;                 // vertex A
		hop::vec3<T> bp; hop::sub(bp, p, b);
		T d3 = hop::dot(ab, bp), d4 = hop::dot(ac, bp);
		if (d3 >= zero && d4 <= d3) return b;                   // vertex B
		T vc = d1 * d4 - d3 * d2;
		if (vc <= zero && d1 >= zero && d3 <= zero) {           // edge AB
			T v = d1 / (d1 - d3);
			hop::mul(tmp, ab, v); hop::add(closest, a, tmp); return closest;
		}
		hop::vec3<T> cp; hop::sub(cp, p, c);
		T d5 = hop::dot(ab, cp), d6 = hop::dot(ac, cp);
		if (d6 >= zero && d5 <= d6) return c;                   // vertex C
		T vb = d5 * d2 - d1 * d6;
		if (vb <= zero && d2 >= zero && d6 <= zero) {           // edge AC
			T w = d2 / (d2 - d6);
			hop::mul(tmp, ac, w); hop::add(closest, a, tmp); return closest;
		}
		T va = d3 * d6 - d5 * d4;
		if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero) { // edge BC
			T w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			hop::vec3<T> bc; hop::sub(bc, c, b);
			hop::mul(tmp, bc, w); hop::add(closest, b, tmp); return closest;
		}
		T denom = tr::one() / (va + vb + vc);                  // face interior
		T v = vb * denom, w = vc * denom;
		hop::mul(tmp, ab, v); hop::add(closest, a, tmp);
		hop::mul(tmp, ac, w); hop::add(closest, tmp);
		return closest;
	}

	// Closest points c1,c2 between segments [p1,q1] and [p2,q2]. Ericson §5.1.9.
	static void closest_pt_segment_segment(const hop::vec3<T> & p1, const hop::vec3<T> & q1,
	        const hop::vec3<T> & p2, const hop::vec3<T> & q2,
	        hop::vec3<T> & c1, hop::vec3<T> & c2) {
		const T zero{};
		const T eps = contact_eps();
		hop::vec3<T> d1, d2, r;
		hop::sub(d1, q1, p1);
		hop::sub(d2, q2, p2);
		hop::sub(r, p1, p2);
		T a = hop::dot(d1, d1);
		T e = hop::dot(d2, d2);
		T f = hop::dot(d2, r);
		T s, t;
		if (a <= eps && e <= eps) {
			s = zero; t = zero;
		} else if (a <= eps) {
			s = zero; t = clamp01(f / e);
		} else {
			T c = hop::dot(d1, r);
			if (e <= eps) {
				t = zero; s = clamp01(-c / a);
			} else {
				T b = hop::dot(d1, d2);
				T denom = a * e - b * b;
				s = (denom != zero) ? clamp01((b * f - c * e) / denom) : zero;
				t = (b * s + f) / e;
				if (t < zero) { t = zero; s = clamp01(-c / a); }
				else if (t > tr::one()) { t = tr::one(); s = clamp01((b - c) / a); }
			}
		}
		hop::vec3<T> tmp;
		hop::mul(tmp, d1, s); hop::add(c1, p1, tmp);
		hop::mul(tmp, d2, t); hop::add(c2, p2, tmp);
	}

	// Squared closest distance between segment (p,q) and triangle tri_idx, with
	// the closest point on the segment (cs) and on the triangle (ct). Considers
	// the spine-pierces-triangle case (distance 0), both spine endpoints vs the
	// triangle, and the spine vs each of the three edges.
	T closest_seg_triangle(const hop::vec3<T> & p, const hop::vec3<T> & q, int tri_idx,
	        hop::vec3<T> & cs, hop::vec3<T> & ct) const {
		auto & tri = tris_[tri_idx];
		const auto & a = verts_[tri.i0];
		const auto & b = verts_[tri.i1];
		const auto & c = verts_[tri.i2];

		// Spine pierces the triangle interior → distance 0.
		const auto & fn = normals_[tri_idx];
		hop::vec3<T> pq; hop::sub(pq, q, p);
		T denom = hop::dot(fn, pq);
		if (denom > contact_eps() || denom < -contact_eps()) {
			hop::vec3<T> ap; hop::sub(ap, a, p);
			T tt = hop::dot(fn, ap) / denom;
			if (tt >= T{} && tt <= tr::one()) {
				hop::vec3<T> x; hop::mul(x, pq, tt); hop::add(x, p);
				if (point_in_triangle(x, tri_idx)) { cs = x; ct = x; return T{}; }
			}
		}

		auto consider = [&](const hop::vec3<T> & sp, const hop::vec3<T> & tp,
		                    T & best, bool & have) {
			hop::vec3<T> diff; hop::sub(diff, sp, tp);
			T d2 = hop::length_squared(diff);
			if (!have || d2 < best) { best = d2; cs = sp; ct = tp; have = true; }
		};

		T best{}; bool have = false;
		hop::vec3<T> tp, sc, tc;
		tp = closest_pt_point_triangle(p, a, b, c); consider(p, tp, best, have);
		tp = closest_pt_point_triangle(q, a, b, c); consider(q, tp, best, have);
		closest_pt_segment_segment(p, q, a, b, sc, tc); consider(sc, tc, best, have);
		closest_pt_segment_segment(p, q, b, c, sc, tc); consider(sc, tc, best, have);
		closest_pt_segment_segment(p, q, c, a, sc, tc); consider(sc, tc, best, have);
		return best;
	}

	// Static-overlap recovery of a capsule against one triangle, one-sided to the
	// triangle's FRONT face (Godot's rule). `base` is the body origin in mesh-
	// local space (plus the shape's local offset). Returns true and fills the
	// push-out depth + normal when the capsule overlaps the front of the face
	// within `radius + margin`.
	bool recover_capsule_triangle(int tri_idx, const hop::capsule<T> & cap,
	        const hop::vec3<T> & base, T margin, T & out_depth,
	        hop::vec3<T> & out_normal) const {
		hop::vec3<T> p1, p2;
		hop::add(p1, base, cap.origin);
		hop::add(p2, p1, cap.direction);

		hop::vec3<T> cs, ct;
		T d2 = closest_seg_triangle(p1, p2, tri_idx, cs, ct);
		T thresh = cap.radius + margin;
		// Strict: a capsule resting exactly at d == radius still registers as a
		// (zero-depth) touch, matching the speculative-margin behaviour callers rely
		// on for ground detection.
		if (d2 > thresh * thresh) return false;

		T d = tr::sqrt(d2);
		const hop::vec3<T> & face_n = normals_[tri_idx];

		// Contact normal: from the triangle toward the capsule spine. Degenerate
		// when the spine pierces the face (d≈0); fall back to the stored face
		// normal (consistently up on flat floors), which keeps the basement push
		// pointing UP no matter how deep the body has sunk.
		hop::vec3<T> n;
		if (d > contact_eps()) {
			hop::sub(n, cs, ct);
			hop::mul(n, tr::one() / d);
		} else {
			n = face_n;
		}

		// One-sided front gate: only push out of the FRONT face. Rejects the
		// coincident back-wound twin on mixed-winding BSP surfaces and never flips
		// downward when the center sinks below a floor.
		if (hop::dot(face_n, n) < T{}) return false;

		T depth = cap.radius - d;
		if (depth < T{}) depth = T{};
		out_depth = depth;
		out_normal = n;
		return true;
	}

	// Conservative-advancement swept contact of a capsule (spine p1..p2 at t=0,
	// radius) moving by `dir` against one triangle. Distance-based, hence two-
	// sided: it stops the capsule at the surface from whichever side it
	// approaches, so the BSP's mixed-winding ramp has no holes. Returns true and
	// fills the time-of-impact (in [0,1]) + contact normal if it hits before
	// `result_time` while moving INTO the surface.
	bool sweep_capsule_triangle(int tri_idx, const hop::vec3<T> & p1_0,
	        const hop::vec3<T> & p2_0, const hop::vec3<T> & dir, T radius, T margin,
	        T result_time, T & out_t, hop::vec3<T> & out_normal) const {
		T speed = tr::sqrt(hop::dot(dir, dir));
		if (speed <= contact_eps()) return false;
		T thresh = radius + margin; // stop this far from the true surface

		T t = T{};
		const int MAX_CA = 32;
		hop::vec3<T> off, p1, p2, cs, ct;
		for (int it = 0; it < MAX_CA; ++it) {
			if (t >= result_time) return false; // a nearer contact already wins
			hop::mul(off, dir, t);
			hop::add(p1, p1_0, off);
			hop::add(p2, p2_0, off);

			T d2 = closest_seg_triangle(p1, p2, tri_idx, cs, ct);
			T d = tr::sqrt(d2);
			T gap = d - thresh;
			if (gap <= contact_eps()) {
				// Contact at parameter t. Normal points from the surface toward the
				// capsule spine (so it opposes the approach for clean sliding).
				hop::vec3<T> n;
				if (d > contact_eps()) {
					hop::sub(n, cs, ct);
					hop::mul(n, tr::one() / d);
				} else {
					n = normals_[tri_idx];
					if (hop::dot(n, dir) > T{}) hop::neg(n);
				}
				// Only block if actually moving INTO the surface — a body resting on
				// or grazing along the face (dot≈0) or separating slides freely.
				if (hop::dot(n, dir) >= -contact_eps()) return false;
				out_t = t;
				out_normal = n;
				return true;
			}
			t += gap / speed; // conservative: the gap can't close faster than `speed`
		}
		return false;
	}

	// --- Non-capsule (box/sphere) fallback: winding-agnostic support-plane ---
	// Recovery of a single non-capsule shape against one triangle. Orients the
	// face normal toward the body (double-sided, like Godot's concave collision)
	// and pushes out of the nearest (minimum-depth) surface.
	void recover_support_plane(int tri_idx, hop::shape<T> * sh,
	        const hop::vec3<T> & local_origin, const hop::segment<T> & seg, T margin,
	        hop::collision<T> & result, bool & found, T & best_depth) const {
		auto & tri = tris_[tri_idx];
		auto & n = normals_[tri_idx];

		hop::vec3<T> face_n = n;
		if (hop::dot(n, local_origin) - hop::dot(n, verts_[tri.i0]) < T{})
			hop::neg(face_n, n);

		hop::vec3<T> neg_fn; hop::neg(neg_fn, face_n);
		hop::vec3<T> sup; hop::support(sup, *sh, neg_fn);
		T expand = hop::dot(sup, neg_fn);

		T plane_d = hop::dot(face_n, verts_[tri.i0]);
		T surface_d = plane_d + expand;
		T expanded_d = surface_d + margin;

		T along = hop::dot(face_n, local_origin);
		if (along > expanded_d) return;

		T dist = along - plane_d;
		hop::vec3<T> proj, n_scaled;
		hop::mul(n_scaled, face_n, dist);
		hop::sub(proj, local_origin, n_scaled);

		if (point_near_triangle(proj, tri_idx, seam_tol_)) {
			T depth = surface_d - along;
			if (depth < T{}) depth = T{};
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

	// Swept cast of a single non-capsule shape against one triangle.
	void sweep_support_plane(int tri_idx, hop::shape<T> * sh,
	        const hop::vec3<T> & local_origin, const hop::segment<T> & seg,
	        hop::collision<T> & result) const {
		auto & tri = tris_[tri_idx];
		auto & n = normals_[tri_idx];

		hop::vec3<T> face_n = n;
		if (hop::dot(n, local_origin) - hop::dot(n, verts_[tri.i0]) < T{})
			hop::neg(face_n, n);

		T denom = hop::dot(face_n, seg.direction);
		if (denom >= T{}) return; // not moving into this (body-facing) surface

		hop::vec3<T> neg_fn; hop::neg(neg_fn, face_n);
		hop::vec3<T> sup; hop::support(sup, *sh, neg_fn);
		T expand = hop::dot(sup, neg_fn);

		auto & v0 = verts_[tri.i0];
		T plane_d = hop::dot(face_n, v0);
		T expanded_d = plane_d + expand;

		T t = (expanded_d - hop::dot(face_n, local_origin)) / denom;
		if (t > tr::one() || t >= result.time) return;
		if (t < T{}) t = T{};

		hop::vec3<T> hit;
		hop::mul(hit, seg.direction, t);
		hop::add(hit, local_origin);

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
