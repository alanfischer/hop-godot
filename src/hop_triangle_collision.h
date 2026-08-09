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
	else if constexpr (std::is_same_v<T, hop::fixed32>)
		eps = -hop::fixed32::from_raw(1LL << 20); // ~ -2.4e-4, matching the fixed16 tolerance
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
	else if constexpr (std::is_same_v<T, hop::fixed32>)
		eps = hop::fixed32::from_raw(1LL << 9); // ~1.2e-7, matching the float threshold
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
	// support_in_solid, not support: the intrinsic support ignores the shape's
	// local_position/local_rotation, and this extent is measured from the SOLID's
	// origin. (The capsule arms below already compose local_position by hand.)
	hop::support_in_solid(sup, *sh, neg_fn);
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
			// Real contact point on the triangle (proj = local_origin projected onto
			// the face); map local→world by +position (seg.origin - local_origin).
			hop::vec3<T> off;
			hop::sub(off, seg.origin, local_origin);
			hop::add(result.impact, proj, off);
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
	// support_in_solid, not support: the intrinsic support ignores the shape's
	// local_position/local_rotation, and this extent is measured from the SOLID's
	// origin. (The capsule arms below already compose local_position by hand.)
	hop::support_in_solid(sup, *sh, neg_fn);
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
		// Real contact point on the triangle at TOI; map local→world by +position.
		hop::vec3<T> off;
		hop::sub(off, seg.origin, local_origin);
		hop::add(result.impact, proj, off);
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
				// Real contact point on the triangle (a resting rider's footprint),
				// for lever arms; map local→world by +position (seg.origin-local_origin).
				hop::vec3<T> p1s, p2s, cs, ct, off;
				hop::add(p1s, base, cap.origin);
				hop::add(p2s, p1s, cap.direction);
				hop::closest_segment_triangle(p1s, p2s, a, b, c, cs, ct, contact_eps<T>());
				hop::sub(off, seg.origin, local_origin);
				hop::add(result.impact, ct, off);
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
			// Real contact point on the TRIANGLE (collidee surface) at TOI, for lever
			// arms (kinematic carry / angular response). result.point is the MOVER's
			// origin at impact; this is the witness point on the target. The spine's
			// closest-point on the triangle (ct) is what we want; recompute it at the
			// impact position, then map the local point to world by +position
			// (= seg.origin - local_origin for this identity-oriented traceable).
			hop::vec3<T> p1t, p2t, cs, ct, off;
			hop::mul(off, seg.direction, sweep.time);
			hop::add(p1t, p1, off);
			hop::add(p2t, p2, off);
			hop::closest_segment_triangle(p1t, p2t, a, b, c, cs, ct, contact_eps<T>());
			hop::sub(off, seg.origin, local_origin); // traceable world position
			hop::add(result.impact, ct, off);
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
				// Local contact point on the triangle; trace_solid_rotated maps it to
				// world by R alongside point/normal.
				hop::vec3<T> cs, ct;
				hop::closest_segment_triangle(p1, p2, a, b, c, cs, ct, contact_eps<T>());
				result.impact = ct;
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
			// Local contact point on the triangle at TOI (mapped to world by caller).
			hop::vec3<T> p1t, p2t, cs, ct, off;
			hop::mul(off, local_seg.direction, sweep.time);
			hop::add(p1t, p1, off);
			hop::add(p2t, p2, off);
			hop::closest_segment_triangle(p1t, p2t, a, b, c, cs, ct, contact_eps<T>());
			result.impact = ct;
		}
	}
}

// --- oriented box (OBB) mover vs triangle, in the target's local frame --------
//
// A box mover has no rotation-invariant spine, so the capsule reduction above
// can't carry it into a rotated traceable's frame. Instead, once transformed into
// the target's local frame the box is an OBB (center `center_l`, axes = columns of
// `Q`, half-extents `half`); we run the same winding-agnostic support-plane test
// the non-rotated box path uses (sweep_support_plane / recover_support_plane), but
// expressed locally and writing local results that trace_solid_rotated maps back
// to world — exactly like capsule_local_vs_triangle. Only the triangle face normal
// is used as the separating axis (same SAT altitude as the AA box path: edge
// contacts are approximate, by parity, not regression).

// OBB support corner offset from center in direction d: Σ sign(d·qᵢ)·hᵢ·qᵢ.
template <typename T>
inline void obb_support(hop::vec3<T> & out, const hop::vec3<T> & q0, const hop::vec3<T> & q1,
                        const hop::vec3<T> & q2, const hop::vec3<T> & half, const hop::vec3<T> & d) {
	out.reset();
	hop::vec3<T> t;
	hop::mul(t, q0, (hop::dot(d, q0) >= T {} ? half.x : -half.x));
	hop::add(out, t);
	hop::mul(t, q1, (hop::dot(d, q1) >= T {} ? half.y : -half.y));
	hop::add(out, t);
	hop::mul(t, q2, (hop::dot(d, q2) >= T {} ? half.z : -half.z));
	hop::add(out, t);
}

// Per-(OBB, triangle) static-recover + swept test, all in the target's local
// frame. `local_seg.origin` is the mover solid origin in local space (for
// result.point), `center_l` the OBB center. Mirrors capsule_local_vs_triangle.
template <typename T>
inline void obb_local_vs_triangle(const hop::vec3<T> & center_l, const hop::vec3<T> & q0,
                                  const hop::vec3<T> & q1, const hop::vec3<T> & q2, const hop::vec3<T> & half,
                                  const hop::segment<T> & local_seg, bool is_static, T margin, T seam_tol,
                                  const hop::vec3<T> & a, const hop::vec3<T> & b, const hop::vec3<T> & c,
                                  const hop::vec3<T> & normal, hop::collision<T> & result,
                                  bool & found, T & best_depth) {
	using tr = hop::scalar_traits<T>;
	// Orient the face normal toward the body (double-sided, like Godot concave).
	hop::vec3<T> face_n = normal;
	if (hop::dot(normal, center_l) - hop::dot(normal, a) < T {})
		hop::neg(face_n, normal);
	hop::vec3<T> neg_fn;
	hop::neg(neg_fn, face_n);
	hop::vec3<T> sup;
	obb_support(sup, q0, q1, q2, half, neg_fn);
	T expand = hop::dot(sup, neg_fn);
	T plane_d = hop::dot(face_n, a);

	if (is_static) {
		T surface_d = plane_d + expand;
		T expanded_d = surface_d + margin;
		T along = hop::dot(face_n, center_l);
		if (along > expanded_d)
			return;
		T dist = along - plane_d;
		hop::vec3<T> proj, n_scaled;
		hop::mul(n_scaled, face_n, dist);
		hop::sub(proj, center_l, n_scaled);
		if (!point_near_triangle(proj, a, b, c, seam_tol))
			return;
		T depth = surface_d - along;
		if (depth < T {})
			depth = T {};
		if (!found || depth < best_depth) {
			best_depth = depth;
			result.time = T {};
			result.depth = depth;
			result.normal = face_n;
			result.point = local_seg.origin;
			result.impact = proj;
			found = true;
		}
		return;
	}

	T denom = hop::dot(face_n, local_seg.direction);
	if (denom >= T {})
		return; // not moving into this (body-facing) surface
	T expanded_d = plane_d + expand;
	T t = (expanded_d - hop::dot(face_n, center_l)) / denom;
	if (t > tr::one() || t >= result.time)
		return;
	if (t < T {})
		t = T {};
	hop::vec3<T> hit_center, off;
	hop::mul(off, local_seg.direction, t);
	hop::add(hit_center, center_l, off);
	T offset = hop::dot(face_n, hit_center) - plane_d;
	hop::vec3<T> proj, n_scaled;
	hop::mul(n_scaled, face_n, offset);
	hop::sub(proj, hit_center, n_scaled);
	if (!point_near_triangle(proj, a, b, c, seam_tol))
		return;
	result.time = t;
	hop::mul(result.point, local_seg.direction, t);
	hop::add(result.point, local_seg.origin);
	result.normal = face_n;
	result.impact = proj;
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
                                const hop::segment<T> & seg, T margin, T seam_tol, Enumerate && enumerate) {
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
		hop::shape<T> * sh = sh_ptr.get();
		hop::vec3<T> p1w, p2w;
		T radius;
		if (mover_world_capsule(s, sh, seg.origin, p1w, p2w, radius)) {
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
			continue;
		}
		if (sh->get_type() != hop::shape_type::box)
			continue; // only box/capsule/sphere movers carry into a rotated traceable

		// Box mover: reduce to an OBB in the target's local frame. Mover world
		// rotation Rm = solid_orientation·shape_local_rotation; box axes Q = Rᵀ·Rm;
		// box center mapped into local by Rᵀ·(world_center − position).
		const hop::aa_box<T> & box = sh->get_box();
		hop::vec3<T> bc, half;
		hop::add(bc, box.mins, box.maxs); hop::mul(bc, hop::scalar_traits<T>::half());
		hop::sub(half, box.maxs, box.mins); hop::mul(half, hop::scalar_traits<T>::half());
		hop::mat3<T> Rm, Q;
		hop::mul(Rm, s->get_orientation(), sh->get_local_rotation());
		hop::mul(Q, Rt, Rm);
		hop::vec3<T> q0, q1, q2;
		hop::mul(q0, Q, hop::constants<T>::x_unit_vec3());
		hop::mul(q1, Q, hop::constants<T>::y_unit_vec3());
		hop::mul(q2, Q, hop::constants<T>::z_unit_vec3());
		// world box center = seg.origin + solid_orientation·local_position + Rm·bc
		hop::vec3<T> wc, off;
		hop::mul(off, s->get_orientation(), sh->get_local_position());
		hop::add(wc, seg.origin, off);
		hop::mul(off, Rm, bc);
		hop::add(wc, off);
		hop::vec3<T> center_l, rel;
		hop::sub(rel, wc, position); hop::mul(center_l, Rt, rel);

		// Local query AABB: the OBB's local-axis extent (Σ|qᵢ·axis|·hᵢ) around the
		// center, merged with the swept endpoint.
		hop::vec3<T> ext(
		    hop::scalar_traits<T>::abs(q0.x) * half.x + hop::scalar_traits<T>::abs(q1.x) * half.y + hop::scalar_traits<T>::abs(q2.x) * half.z,
		    hop::scalar_traits<T>::abs(q0.y) * half.x + hop::scalar_traits<T>::abs(q1.y) * half.y + hop::scalar_traits<T>::abs(q2.y) * half.z,
		    hop::scalar_traits<T>::abs(q0.z) * half.x + hop::scalar_traits<T>::abs(q1.z) * half.y + hop::scalar_traits<T>::abs(q2.z) * half.z);
		hop::aa_box<T> q;
		hop::sub(q.mins, center_l, ext);
		hop::add(q.maxs, center_l, ext);
		if (!is_static) {
			hop::vec3<T> ec;
			hop::add(ec, center_l, local_seg.direction);
			hop::aa_box<T> qe;
			hop::sub(qe.mins, ec, ext);
			hop::add(qe.maxs, ec, ext);
			q.merge(qe);
		}

		enumerate(q, [&](const hop::vec3<T> & a, const hop::vec3<T> & b,
		                 const hop::vec3<T> & c, const hop::vec3<T> & face_n) {
			obb_local_vs_triangle(center_l, q0, q1, q2, half, local_seg, is_static, margin,
			    seam_tol, a, b, c, face_n, local, found, best_depth);
		});
	}

	if (found || local.time < result.time) {
		result.time = local.time;
		result.depth = local.depth;
		hop::mul(result.normal, orientation, local.normal);
		hop::vec3<T> wp;
		hop::mul(wp, orientation, local.point);
		hop::add(result.point, wp, position);
		// Map the local contact point on the triangle back to world (R·local + pos).
		hop::vec3<T> wi;
		hop::mul(wi, orientation, local.impact);
		hop::add(result.impact, wi, position);
	}
}

} // namespace hoptri
