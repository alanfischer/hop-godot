#pragma once

#include <hop/hop.h>
#include <type_traits>

// Per-triangle collision shared by the trimesh (BVH-fed) and heightfield
// (grid-fed) traceables. Each routine takes the triangle's three verts + its
// precomputed face normal directly, so the candidate-triangle source (a vertex
// array vs an on-the-fly heightmap cell) is the caller's concern. The actual
// closest-point / swept-contact math is hop core (math/triangle.h, math/gjk.h);
// this layer is the shape-type dispatch + the BSP-winding / seam handling.

namespace hoptri {

// Small distance below which two points are coincident (avoids dividing by a
// ~zero length when forming a contact normal). Per scalar type.
template <typename T> inline T contact_eps() {
	if constexpr (std::is_same_v<T, hop::fixed16>)
		return hop::fixed16::from_raw(16);
	else if constexpr (std::is_same_v<T, hop::fixed32>)
		return hop::fixed32::from_raw(int64_t(1) << 8);
	else
		return T(1e-6);
}

// Barycentric point-in-triangle test (cheap fast path).
template <typename T>
inline bool point_in_triangle(const hop::vec3<T> & p, const hop::vec3<T> & a,
                              const hop::vec3<T> & b, const hop::vec3<T> & c) {
	using tr = hop::scalar_traits<T>;
	hop::vec3<T> e0, e1, ep;
	hop::sub(e0, b, a);
	hop::sub(e1, c, a);
	hop::sub(ep, p, a);
	T d00 = hop::dot(e0, e0), d01 = hop::dot(e0, e1), d11 = hop::dot(e1, e1);
	T d20 = hop::dot(ep, e0), d21 = hop::dot(ep, e1);
	T denom = d00 * d11 - d01 * d01;
	if (denom == T {})
		return false;
	T inv = tr::one() / denom;
	T u = (d11 * d20 - d01 * d21) * inv;
	T v = (d00 * d21 - d01 * d20) * inv;
	T eps;
	if constexpr (std::is_same_v<T, hop::fixed16>)
		eps = -hop::fixed16::from_raw(1 << 4);
	else
		eps = T(-1e-4);
	return u >= eps && v >= eps && (u + v) <= (tr::one() - eps);
}

// True if p is inside the triangle, OR within `tol` of it. Bridges hairline
// seam / T-junction gaps so a swept step can't slip between two triangles and
// tunnel. The exact closest-point (core math/triangle.h) runs only on the fail
// path, so the common in-triangle case stays cheap.
template <typename T>
inline bool point_near_triangle(const hop::vec3<T> & p, const hop::vec3<T> & a,
                                const hop::vec3<T> & b, const hop::vec3<T> & c, T tol) {
	if (point_in_triangle(p, a, b, c))
		return true;
	hop::vec3<T> closest;
	hop::closest_point_triangle(closest, p, a, b, c);
	hop::vec3<T> diff;
	hop::sub(diff, p, closest);
	return hop::length_squared(diff) <= tol * tol;
}

// Ray-triangle intersection (Möller–Trumbore). Returns the hit time in [0,1] (or
// 1 on miss); on hit fills `point` and `normal` (oriented against the ray).
template <typename T>
inline T ray_triangle(const hop::segment<T> & seg, const hop::vec3<T> & v0,
                      const hop::vec3<T> & v1, const hop::vec3<T> & v2,
                      const hop::vec3<T> & face_normal, hop::vec3<T> & point, hop::vec3<T> & normal) {
	using tr = hop::scalar_traits<T>;
	const T zero {};
	hop::vec3<T> e1, e2;
	hop::sub(e1, v1, v0);
	hop::sub(e2, v2, v0);
	hop::vec3<T> h;
	hop::cross(h, seg.direction, e2);
	T a = hop::dot(e1, h);
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
	if (u < zero || u > tr::one())
		return tr::one();
	hop::vec3<T> q;
	hop::cross(q, s_vec, e1);
	T v = hop::dot(seg.direction, q) * inv_a;
	if (v < zero || (u + v) > tr::one())
		return tr::one();
	T time = hop::dot(e2, q) * inv_a;
	if (time < zero || time > tr::one())
		return tr::one();
	hop::mul(point, seg.direction, time);
	hop::add(point, seg.origin);
	normal = face_normal;
	if (hop::dot(normal, seg.direction) > zero)
		hop::neg(normal);
	return time;
}

// --- capsule (player) vs triangle: analytic closest-point ---

// One-sided (FRONT face) static-overlap recovery of a capsule spine (explicit
// endpoints p1,p2 + radius) against a triangle, all in the same frame. Returns
// true + push-out depth/normal when within radius + margin of the front face.
// The base/cap.* overload below is the common (axis-aligned) caller; the rotated
// traceable path supplies endpoints transformed into the target's local frame.
template <typename T>
inline bool recover_capsule_triangle(const hop::vec3<T> & p1, const hop::vec3<T> & p2, T radius,
                                     const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                     const hop::vec3<T> & face_n, T margin, T & out_depth,
                                     hop::vec3<T> & out_normal) {
	using tr = hop::scalar_traits<T>;
	hop::vec3<T> cs, ct;
	T d2 = hop::closest_segment_triangle(p1, p2, a, b, c, cs, ct, contact_eps<T>());
	T thresh = radius + margin;
	if (d2 > thresh * thresh)
		return false;
	T d = tr::sqrt(d2);
	// Normal from triangle toward the spine; degenerate when the spine pierces
	// the face (d≈0) — fall back to the stored face normal (keeps the push up).
	hop::vec3<T> n;
	if (d > contact_eps<T>()) {
		hop::sub(n, cs, ct);
		hop::mul(n, tr::one() / d);
	} else {
		n = face_n;
	}
	// Front gate: reject the back-wound twin on mixed-winding BSP surfaces.
	if (hop::dot(face_n, n) < T {})
		return false;
	T depth = radius - d;
	if (depth < T {})
		depth = T {};
	out_depth = depth;
	out_normal = n;
	return true;
}

// `base` is the capsule's body origin in mesh-local space (incl. the shape's
// local offset); the spine is base+cap.origin .. +cap.direction.
template <typename T>
inline bool recover_capsule_triangle(const hop::capsule<T> & cap, const hop::vec3<T> & base,
                                     const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                     const hop::vec3<T> & face_n, T margin, T & out_depth,
                                     hop::vec3<T> & out_normal) {
	hop::vec3<T> p1, p2;
	hop::add(p1, base, cap.origin);
	hop::add(p2, p1, cap.direction);
	return recover_capsule_triangle(p1, p2, cap.radius, a, b, c, face_n, margin, out_depth, out_normal);
}

// Swept capsule-vs-triangle via the shared conservative-advancement loop with an
// analytic segment-vs-triangle closest-point. Distance-based (two-sided).
template <typename T>
inline hop::gjk_sweep_result<T> sweep_capsule_triangle(const hop::vec3<T> & p1_0, const hop::vec3<T> & p2_0,
                                                       const hop::vec3<T> & dir, T radius,
                                                       const hop::vec3<T> & a, const hop::vec3<T> & b,
                                                       const hop::vec3<T> & c, const hop::vec3<T> & face_n, T margin) {
	using tr = hop::scalar_traits<T>;
	hop::gjk_sweep_result<T> res;
	hop::conservative_advance<T>(res, dir, radius + margin, contact_eps<T>(),
	    [&](const hop::vec3<T> & xA, const hop::vec3<T> & /*seed*/,
	        T & dist, hop::vec3<T> & n, bool & deep) {
		    deep = false;
		    hop::vec3<T> p1, p2, cs, ct;
		    hop::add(p1, p1_0, xA);
		    hop::add(p2, p2_0, xA);
		    dist = tr::sqrt(hop::closest_segment_triangle(p1, p2, a, b, c, cs, ct, contact_eps<T>()));
		    if (dist > contact_eps<T>()) {
			    hop::sub(n, cs, ct);
			    hop::mul(n, tr::one() / dist);
		    } else {
			    n = face_n;
			    if (hop::dot(n, dir) > T {})
				    hop::neg(n);
		    }
	    });
	return res;
}

// --- non-capsule (box/sphere) vs triangle: winding-agnostic support-plane ---

// Static-overlap recovery. Orients the face normal toward the body (double-
// sided, like Godot's concave collision) and pushes out of the nearest surface.
template <typename T>
inline void recover_support_plane(hop::shape<T> * sh, const hop::vec3<T> & local_origin,
                                  const hop::segment<T> & seg, const hop::vec3<T> & a, const hop::vec3<T> & b,
                                  const hop::vec3<T> & c, const hop::vec3<T> & normal, T margin, T seam_tol,
                                  hop::collision<T> & result, bool & found, T & best_depth) {
	hop::vec3<T> face_n = normal;
	if (hop::dot(normal, local_origin) - hop::dot(normal, a) < T {})
		hop::neg(face_n, normal);
	hop::vec3<T> neg_fn;
	hop::neg(neg_fn, face_n);
	hop::vec3<T> sup;
	hop::support(sup, *sh, neg_fn);
	T expand = hop::dot(sup, neg_fn);
	T plane_d = hop::dot(face_n, a);
	T surface_d = plane_d + expand;
	T expanded_d = surface_d + margin;
	T along = hop::dot(face_n, local_origin);
	if (along > expanded_d)
		return;
	T dist = along - plane_d;
	hop::vec3<T> proj, n_scaled;
	hop::mul(n_scaled, face_n, dist);
	hop::sub(proj, local_origin, n_scaled);
	if (point_near_triangle(proj, a, b, c, seam_tol)) {
		T depth = surface_d - along;
		if (depth < T {})
			depth = T {};
		if (!found || depth < best_depth) {
			best_depth = depth;
			result.time = T {};
			result.depth = depth;
			result.normal = face_n;
			result.point = seg.origin;
			found = true;
		}
	}
}

// Swept cast of a non-capsule shape against a triangle.
template <typename T>
inline void sweep_support_plane(hop::shape<T> * sh, const hop::vec3<T> & local_origin,
                                const hop::segment<T> & seg, const hop::vec3<T> & a, const hop::vec3<T> & b,
                                const hop::vec3<T> & c, const hop::vec3<T> & normal, T seam_tol,
                                hop::collision<T> & result) {
	using tr = hop::scalar_traits<T>;
	hop::vec3<T> face_n = normal;
	if (hop::dot(normal, local_origin) - hop::dot(normal, a) < T {})
		hop::neg(face_n, normal);
	T denom = hop::dot(face_n, seg.direction);
	if (denom >= T {})
		return; // not moving into this (body-facing) surface
	hop::vec3<T> neg_fn;
	hop::neg(neg_fn, face_n);
	hop::vec3<T> sup;
	hop::support(sup, *sh, neg_fn);
	T expand = hop::dot(sup, neg_fn);
	T plane_d = hop::dot(face_n, a);
	T expanded_d = plane_d + expand;
	T t = (expanded_d - hop::dot(face_n, local_origin)) / denom;
	if (t > tr::one() || t >= result.time)
		return;
	if (t < T {})
		t = T {};
	hop::vec3<T> hit;
	hop::mul(hit, seg.direction, t);
	hop::add(hit, local_origin);
	hop::vec3<T> proj;
	T offset = hop::dot(face_n, hit) - plane_d;
	hop::vec3<T> n_scaled;
	hop::mul(n_scaled, face_n, offset);
	hop::sub(proj, hit, n_scaled);
	if (point_near_triangle(proj, a, b, c, seam_tol)) {
		result.time = t;
		hop::mul(result.point, seg.direction, t);
		hop::add(result.point, seg.origin);
		result.normal = face_n;
	}
}

// --- per-(shape, triangle) dispatch used by both traceables ---

// Static-overlap recovery: capsule (one-sided front, deepest-first) vs others
// (support-plane, nearest). Updates `result`/`found`/`best_depth` in place.
template <typename T>
inline void recover_shape_vs_triangle(hop::shape<T> * sh, const hop::vec3<T> & local_origin,
                                      const hop::segment<T> & seg, T margin, T seam_tol,
                                      const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                      const hop::vec3<T> & face_n, hop::collision<T> & result,
                                      bool & found, T & best_depth) {
	if (sh->get_type() == hop::shape_type::capsule) {
		hop::vec3<T> base;
		hop::add(base, local_origin, sh->get_local_position());
		const auto & cap = sh->get_capsule();
		T depth;
		hop::vec3<T> n;
		if (recover_capsule_triangle(cap, base, a, b, c, face_n, margin, depth, n)) {
			// Deepest front contact first (resolves corners better than nearest).
			if (!found || depth > best_depth) {
				best_depth = depth;
				result.time = T {};
				result.depth = depth;
				result.normal = n;
				result.point = seg.origin;
				found = true;
			}
		}
	} else {
		recover_support_plane(sh, local_origin, seg, a, b, c, face_n, margin, seam_tol, result, found, best_depth);
	}
}

// Swept cast: capsule (closest-point CA) vs others (support-plane). Keeps the
// earliest impact; among equal-time overlaps keeps the deepest (and propagates
// depth so the speculative solver can depenetrate). Updates `result` in place.
template <typename T>
inline void sweep_shape_vs_triangle(hop::shape<T> * sh, const hop::vec3<T> & local_origin,
                                    const hop::segment<T> & seg, T margin, T seam_tol,
                                    const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                    const hop::vec3<T> & face_n, hop::collision<T> & result) {
	if (sh->get_type() == hop::shape_type::capsule) {
		hop::vec3<T> base, p1, p2;
		hop::add(base, local_origin, sh->get_local_position());
		const auto & cap = sh->get_capsule();
		hop::add(p1, base, cap.origin);
		hop::add(p2, p1, cap.direction);
		auto sweep = sweep_capsule_triangle(p1, p2, seg.direction, cap.radius, a, b, c, face_n, margin);
		if (sweep.hit && (sweep.time < result.time ||
		                  (sweep.time == result.time && sweep.depth > result.depth))) {
			result.time = sweep.time;
			result.depth = sweep.depth;
			hop::mul(result.point, seg.direction, sweep.time);
			hop::add(result.point, seg.origin);
			result.normal = sweep.normal;
		}
	} else {
		sweep_support_plane(sh, local_origin, seg, a, b, c, face_n, seam_tol, result);
	}
}

// --- static-rotation support: rotated traceable frame ---
//
// When a trimesh/heightfield target carries a non-identity world rotation, the
// query is transformed into the target's local frame (where the geometry lives)
// and the result mapped back. The mover's spine can no longer be reconstructed
// from the shape alone — once tilted into the target frame a capsule is not
// axis-aligned — so the mover is reduced to an explicit world-space spine
// (capsule) or point (sphere) here, transformed by the caller, and fed to the
// explicit-endpoint routines above. Box movers have no rotation-invariant spine
// and are out of scope for static rotation (the caller skips them).

// World-space capsule spine (p1,p2) + radius for the mover shape, honoring the
// mover solid's own orientation and the shape's local pose. A sphere collapses
// to a zero-length spine (p1==p2). Returns false for shapes with no spine (box).
template <typename T>
inline bool mover_world_capsule(hop::solid<T> * s, hop::shape<T> * sh, const hop::vec3<T> & mover_pos,
                                hop::vec3<T> & p1, hop::vec3<T> & p2, T & radius) {
	hop::mat3<T> Rm;
	hop::mul(Rm, s->get_orientation(), sh->get_local_rotation());
	hop::vec3<T> base, off;
	hop::mul(base, s->get_orientation(), sh->get_local_position());
	hop::add(base, mover_pos);
	if (sh->get_type() == hop::shape_type::capsule) {
		const auto & cap = sh->get_capsule();
		hop::mul(off, Rm, cap.origin);
		hop::add(p1, base, off);
		hop::mul(off, Rm, cap.direction);
		hop::add(p2, p1, off);
		radius = cap.radius;
		return true;
	}
	if (sh->get_type() == hop::shape_type::sphere) {
		const auto & sph = sh->get_sphere();
		hop::mul(off, Rm, sph.origin);
		hop::add(p1, base, off);
		p2 = p1;
		radius = sph.radius;
		return true;
	}
	return false;
}

// Per-(spine, triangle) swept/recover, all expressed in the target's LOCAL
// frame: `p1`/`p2` are the local spine, `local_seg` the local mover segment
// (origin = mover position, direction = displacement). Results are written in
// local space — the caller maps point/normal back to world by the target's
// rotation. Mirrors {sweep,recover}_shape_vs_triangle for the capsule case.
template <typename T>
inline void capsule_local_vs_triangle(const hop::vec3<T> & p1, const hop::vec3<T> & p2, T radius,
                                      const hop::segment<T> & local_seg, bool is_static, T margin,
                                      const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                      const hop::vec3<T> & face_n, hop::collision<T> & result,
                                      bool & found, T & best_depth) {
	if (is_static) {
		T depth;
		hop::vec3<T> n;
		if (recover_capsule_triangle(p1, p2, radius, a, b, c, face_n, margin, depth, n)) {
			if (!found || depth > best_depth) {
				best_depth = depth;
				result.time = T {};
				result.depth = depth;
				result.normal = n;
				result.point = local_seg.origin;
				found = true;
			}
		}
	} else {
		auto sweep = sweep_capsule_triangle(p1, p2, local_seg.direction, radius, a, b, c, face_n, margin);
		if (sweep.hit && (sweep.time < result.time ||
		                  (sweep.time == result.time && sweep.depth > result.depth))) {
			result.time = sweep.time;
			result.depth = sweep.depth;
			hop::mul(result.point, local_seg.direction, sweep.time);
			hop::add(result.point, local_seg.origin);
			result.normal = sweep.normal;
		}
	}
}

// AABB enclosing the capsule spine p1..p2 grown by `radius`.
template <typename T>
inline void spine_aabb(hop::aa_box<T> & box, const hop::vec3<T> & p1,
                       const hop::vec3<T> & p2, T radius) {
	box.mins = p1;
	box.maxs = p1;
	box.merge(p2);
	box.mins.x -= radius; box.mins.y -= radius; box.mins.z -= radius;
	box.maxs.x += radius; box.maxs.y += radius; box.maxs.z += radius;
}

// Drives a traceable's rotated-frame solid trace, shared by the trimesh (BVH)
// and heightfield (grid) traceables: transform the world query into the target's
// local frame by Rᵀ, reduce each mover shape to a capsule/sphere spine in that
// frame, dispatch per candidate triangle, then map the contact back to world by
// R. The only per-traceable difference — how candidate triangles are found — is
// the `enumerate` callback: it receives the local query AABB and a visitor
// `visit(a, b, c, face_n)` to call for each candidate triangle (already in local
// space). Results are written into `result` in WORLD space.
template <typename T, typename Enumerate>
inline void trace_solid_rotated(hop::collision<T> & result, hop::solid<T> * s,
                                const hop::vec3<T> & position, const hop::mat3<T> & orientation,
                                const hop::segment<T> & seg, T margin, Enumerate && enumerate) {
	hop::mat3<T> Rt;
	hop::transpose(Rt, orientation);
	hop::segment<T> local_seg;
	hop::vec3<T> rel;
	hop::sub(rel, seg.origin, position);
	hop::mul(local_seg.origin, Rt, rel);
	hop::mul(local_seg.direction, Rt, seg.direction);

	const bool is_static = (hop::dot(seg.direction, seg.direction) == T {});
	hop::collision<T> local;
	local.time = result.time;
	local.depth = result.depth;
	bool found = false;
	T best_depth = T {};

	for (const auto & sh_ptr : s->get_shapes()) {
		hop::vec3<T> p1w, p2w;
		T radius;
		if (!mover_world_capsule(s, sh_ptr.get(), seg.origin, p1w, p2w, radius))
			continue; // box movers have no rotation-invariant spine (out of scope)
		hop::vec3<T> p1, p2, t;
		hop::sub(t, p1w, position); hop::mul(p1, Rt, t);
		hop::sub(t, p2w, position); hop::mul(p2, Rt, t);

		hop::aa_box<T> q;
		spine_aabb(q, p1, p2, radius);
		if (!is_static) {
			hop::vec3<T> e1, e2;
			hop::add(e1, p1, local_seg.direction);
			hop::add(e2, p2, local_seg.direction);
			hop::aa_box<T> qe;
			spine_aabb(qe, e1, e2, radius);
			q.merge(qe);
		}

		enumerate(q, [&](const hop::vec3<T> & a, const hop::vec3<T> & b,
		                 const hop::vec3<T> & c, const hop::vec3<T> & face_n) {
			capsule_local_vs_triangle(p1, p2, radius, local_seg, is_static, margin,
			    a, b, c, face_n, local, found, best_depth);
		});
	}

	if (found || local.time < result.time) {
		result.time = local.time;
		result.depth = local.depth;
		hop::mul(result.normal, orientation, local.normal);
		hop::vec3<T> wp;
		hop::mul(wp, orientation, local.point);
		hop::add(result.point, wp, position);
	}
}

} // namespace hoptri
