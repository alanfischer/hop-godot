// Unit tests for HopTrimeshTraceable's capsule-vs-triangle closest-point contact.
//
// Background: the player is a CAPSULE moving against GoldSrc BSP triangle soup
// (via build_hull_collision). That soup's per-triangle winding is NOT globally
// consistent — flat floors are wound one way, but walkable ramp faces carry
// coincident, oppositely-wound triangles. The old center+plane approximation
// couldn't satisfy both the flat-floor and the mixed-winding-ramp cases at once,
// producing two bugs: (a) a deep "basement" sink where depenetration flipped and
// shoved the body DOWN through a floor, and (b) holes/sticking on ramps.
//
// The rewrite does real capsule-segment-vs-triangle closest-point contact, like
// Godot's concave collision:
//   * Recovery is ONE-SIDED to each triangle's FRONT face: it pushes out along
//     the face the capsule is on, by the true penetration, and never flips
//     downward when the body sinks below the floor. The front gate rejects the
//     coincident back-wound twin, so a mixed-winding surface is still solid.
//   * The swept cast is distance-based, hence TWO-SIDED: it stops the capsule at
//     the surface from whichever side it approaches (no winding holes), so
//     penetration stays shallow and recovery only sees the clean case.
//
// These tests pin those invariants. Note the winding of `flat_quad`: reversed=
// true gives an UP face normal (the "solid-from-above" floor), reversed=false an
// up-side-down (DOWN) normal.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <hop/hop.h>
#include "hop_trimesh_traceable.h"

using T = double;
using V = hop::vec3<T>;
using Tri = HopTrimeshTraceable<T>::triangle;

static V vec(T x, T y, T z) { V v; v.x = x; v.y = y; v.z = z; return v; }
static bool approx(T a, T b, T tol = 1e-3) { return std::fabs(a - b) < tol; }

// A capsule "player": spine (0,-half,0)..(0,half,0), radius r. Up axis = +Y.
static std::shared_ptr<hop::solid<T>> make_capsule(T radius, T half_spine) {
	hop::capsule<T> cap(vec(0, -half_spine, 0), vec(0, half_spine * 2, 0), radius);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(cap));
	return s;
}

// An axis-aligned box mover with half-extents (hx,hy,hz) — exercises the
// support-plane (non-capsule) contact path.
static std::shared_ptr<hop::solid<T>> make_box(T hx, T hy, T hz) {
	hop::aa_box<T> b;
	b.mins = vec(-hx, -hy, -hz);
	b.maxs = vec(hx, hy, hz);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(b));
	return s;
}

static std::unique_ptr<HopTrimeshTraceable<T>> make_mesh(const std::vector<V> & verts,
                                                         const std::vector<Tri> & tris) {
	auto m = std::make_unique<HopTrimeshTraceable<T>>();
	m->build(verts.data(), (int)verts.size(), tris.data(), (int)tris.size());
	return m;
}

// Zero-direction (static overlap) trace of a capsule centered at `center`.
static hop::collision<T> probe_static(HopTrimeshTraceable<T> & mesh,
                                      hop::solid<T> * body, const V & center) {
	body->set_position(center);
	hop::segment<T> seg;
	seg.set_start_end(center, center); // zero direction → static-overlap path
	hop::collision<T> c; c.reset();
	static const hop::mat3<T> identity;
	mesh.trace_solid(c, body, vec(0, 0, 0), identity, seg, T {});
	return c;
}

// Swept trace of a capsule moving from `center` by `motion`.
static hop::collision<T> probe_swept(HopTrimeshTraceable<T> & mesh,
                                     hop::solid<T> * body, const V & center, const V & motion) {
	body->set_position(center);
	hop::segment<T> seg;
	V end = vec(center.x + motion.x, center.y + motion.y, center.z + motion.z);
	seg.set_start_end(center, end);
	hop::collision<T> c; c.reset();
	static const hop::mat3<T> identity;
	mesh.trace_solid(c, body, vec(0, 0, 0), identity, seg, T {});
	return c;
}

// One horizontal quad at y=0 (a flat floor), with explicit winding.
//   reversed=true  → UP face normal (solid from above)
//   reversed=false → DOWN face normal
static void flat_quad(std::vector<V> & v, std::vector<Tri> & t, bool reversed) {
	int i = (int)v.size();
	v.push_back(vec(-5, 0, -5));
	v.push_back(vec(5, 0, -5));
	v.push_back(vec(5, 0, 5));
	v.push_back(vec(-5, 0, 5));
	if (!reversed) { t.push_back({ i, i + 1, i + 2 }); t.push_back({ i, i + 2, i + 3 }); }
	else           { t.push_back({ i, i + 2, i + 1 }); t.push_back({ i, i + 3, i + 2 }); }
}

// --- Test 1: one-sided recovery off an UP floor reports the TRUE depth ---
static void test_recovery_front_face() {
	const T r = 0.4, half = 0.5, extent = r + half; // lowest point = center.y - extent
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true); // UP-facing floor
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	// Resting exactly on the floor: lowest point at y=0 → center at y=extent.
	hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, extent, 0));
	assert(c.time == T {});       // static overlap detected
	assert(c.normal.y > 0.9);     // pushes straight UP
	assert(c.depth < 0.05);       // ~0, never a bogus ~2*extent shove

	// Sunk 0.1 into the floor → depth ~0.1, still up.
	hop::collision<T> s = probe_static(*mesh, body.get(), vec(0, extent - 0.1, 0));
	assert(s.normal.y > 0.9);
	assert(approx(s.depth, 0.1, 0.03));
	printf("  recovery front face: normal.y=%.2f depth=%.3f\n", c.normal.y, c.depth);
}

// --- Test 2: mixed-winding surface (the real BSP ramp/coincident case) ---
// A walkable face carrying BOTH an up- and a down-wound triangle must still push
// the resting capsule UP — the front gate rejects the inverted twin instead of
// latching it for a downward shove.
static void test_recovery_mixed_winding() {
	const T r = 0.4, half = 0.5, extent = r + half;
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true);  // up-wound
	flat_quad(verts, tris, false); // coincident down-wound twin
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, extent - 0.05, 0));
	assert(c.time == T {});
	assert(c.normal.y > 0.9);     // up, despite the inverted twin
	assert(c.depth < 0.25);       // not the ~1.8 bogus depth
	printf("  recovery mixed winding: normal.y=%.2f depth=%.3f\n", c.normal.y, c.depth);
}

// --- Test 3: the basement regression — deep sink must push UP, never down ---
// When the capsule sinks far enough that its spine straddles the floor plane, the
// old code's "orient toward body center" flipped the push DOWN (the captured
// recover.y = -1.81). The one-sided closest-point recovery must keep pushing UP
// for every sink depth, converging the body back above the floor over the
// simulator's iterated recovery.
static void test_recovery_deep_sink_pushes_up() {
	const T r = 0.4, half = 0.5;
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true); // UP floor at y=0
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	// Walk the center from on-surface down to where the spine still straddles the
	// floor plane (center y >= -half, so the capsule's top stays above it — the
	// captured basement state). Recovery must always be upward, never the -1.81
	// downward flip. (Once the WHOLE capsule is below an up-facing floor it is, by
	// design, on the back side and not solid — exactly like Godot's one-sided
	// concave collision; the two-sided swept cast is what keeps the body from ever
	// getting there.)
	for (T cy = 0.3; cy >= -0.4; cy -= 0.1) {
		hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, cy, 0));
		assert(c.time == T {});       // overlapping
		assert(c.normal.y > 0.9);     // UP — never the -1.81 downward flip
		assert(c.depth > T {});       // a real positive push-out
	}

	// Emulate the simulator's iterated recovery: a few up pushes must lift the
	// deeply-sunk body back to resting on the floor (lowest point ~ y=0).
	V pos = vec(0, -0.4, 0);
	for (int i = 0; i < 6; ++i) {
		hop::collision<T> c = probe_static(*mesh, body.get(), pos);
		if (c.time != T {} || c.depth <= 0) break;
		pos.y += c.normal.y * c.depth; // push along the (upward) normal
	}
	assert(approx(pos.y, r + half, 0.05)); // converged to resting (lowest point ~0)
	printf("  deep sink pushes up: converged center.y=%.3f\n", pos.y);
}

// --- Test 4: swept cast is two-sided (catches either winding) ---
// A capsule dropped onto a single floor triangle must register a hit with an
// upward-facing normal regardless of that triangle's stored winding, because the
// cast is distance-based.
static void test_swept_two_sided() {
	const T r = 0.4, half = 0.5, extent = r + half;
	for (bool reversed : { false, true }) {
		std::vector<V> verts; std::vector<Tri> tris;
		flat_quad(verts, tris, reversed);
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);

		// Drop from above the floor; lowest point starts at y=1, floor at y=0.
		hop::collision<T> c = probe_swept(*mesh, body.get(), vec(0, extent + 1.0, 0), vec(0, -2.0, 0));
		assert(c.time < T(1));        // hit the floor
		assert(c.time > T {});        // not already penetrating
		assert(c.normal.y > 0.5);     // upward (body-facing) normal regardless of winding
		printf("  swept two-sided (down-winding=%d): time=%.3f normal.y=%.2f\n",
		       (int)reversed, c.time, c.normal.y);
	}
}

// --- Test 5: a body resting on the floor still moves horizontally ---
// The swept cast must not block a capsule that's resting on (grazing) the floor
// from sliding along it — only motion INTO a surface blocks. Guards the "can't
// walk" regression from the moving-into gate.
static void test_resting_allows_horizontal_motion() {
	const T r = 0.4, half = 0.5, extent = r + half;
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true);
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	// Resting exactly on the floor, moving horizontally: no blocking contact.
	hop::collision<T> c = probe_swept(*mesh, body.get(), vec(0, extent, 0), vec(1.0, 0, 0));
	assert(c.time == T(1));
	printf("  resting allows horizontal motion: time=%.3f\n", c.time);
}

// --- Test 6: a finite-radius capsule can't fall through a too-narrow gap ---
// Distance-based contact naturally bridges gaps narrower than the capsule
// (it catches on the edge), while a hole wider than the capsule lets it through.
static void test_capsule_gap_fit() {
	const T r = 0.4, half = 0.5, extent = r + half;
	auto run = [&](T gap) {
		std::vector<V> verts; std::vector<Tri> tris;
		// two floor halves split along x=0 with a `gap`-wide slit (UP-wound)
		int i = (int)verts.size();
		verts.push_back(vec(-5, 0, -5)); verts.push_back(vec(-gap / 2, 0, -5));
		verts.push_back(vec(-gap / 2, 0, 5)); verts.push_back(vec(-5, 0, 5));
		tris.push_back({ i, i + 2, i + 1 }); tris.push_back({ i, i + 3, i + 2 });
		i = (int)verts.size();
		verts.push_back(vec(gap / 2, 0, -5)); verts.push_back(vec(5, 0, -5));
		verts.push_back(vec(5, 0, 5)); verts.push_back(vec(gap / 2, 0, 5));
		tris.push_back({ i, i + 2, i + 1 }); tris.push_back({ i, i + 3, i + 2 });
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);
		// drop centered exactly over the slit
		hop::collision<T> c = probe_swept(*mesh, body.get(), vec(0, extent + 1.0, 0), vec(0, -2.0, 0));
		return c.time < T(1);
	};
	assert(run(0.10));   // 10cm slit, capsule r=0.4 can't fit → caught on the edge
	assert(!run(1.20));  // 1.2m hole > capsule diameter (0.8) → falls through
	printf("  capsule gap fit: narrow slit caught, wide hole not\n");
}

// --- Test 7: a body clear of the mesh reports no contact ---
static void test_no_false_contact() {
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true);
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(0.4, 0.5);
	hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, 5.0, 0)); // well above
	assert(c.time != T {}); // no static overlap
	printf("  no false contact when clear\n");
}

// --- Test 8: climbing a 45° ramp does not fall through or stick ---
// A coincident mixed-winding ramp (both windings, like the BSP) must stop a
// capsule swept toward it, with an outward-facing normal, all the way up.
static void test_ramp_climb() {
	const T r = 0.4, half = 0.5;
	// 45° ramp in the x-y plane spanning x in [-5,5], y = x+5 (so y from 0..10),
	// extruded in z. Build both windings (the BSP's coincident-twin case).
	std::vector<V> verts; std::vector<Tri> tris;
	auto add_quad = [&](V a, V b, V c, V d) {
		int i = (int)verts.size();
		verts.push_back(a); verts.push_back(b); verts.push_back(c); verts.push_back(d);
		tris.push_back({ i, i + 1, i + 2 }); tris.push_back({ i, i + 2, i + 3 });
		tris.push_back({ i, i + 2, i + 1 }); tris.push_back({ i, i + 3, i + 2 }); // twin
	};
	add_quad(vec(-5, 0, -5), vec(5, 10, -5), vec(5, 10, 5), vec(-5, 0, 5));
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	// Sample a few points along the ramp; from just above the surface, a downward
	// probe must catch the ramp (never tunnel) with an upward-ish normal.
	const T inv_sqrt2 = 0.70710678;
	int hits = 0;
	for (T x = -4; x <= 4; x += 1.0) {
		T surf_y = x + 5;                       // ramp surface height at this x
		// capsule centered so its lowest point sits ~0.3 above the surface
		V center = vec(x, surf_y + r + 0.3, 0);
		hop::collision<T> c = probe_swept(*mesh, body.get(), center, vec(0, -1.0, 0));
		assert(c.time < T(1));                  // caught the ramp, no fall-through
		assert(c.normal.y > 0.4);               // outward (up-ish) on a 45° face
		(void)inv_sqrt2;
		++hits;
	}
	assert(hits == 9);
	printf("  ramp climb: %d/9 ramp samples caught with up-ish normal\n", hits);
}

// --- Test 9: degenerate triangle must not yield a non-normalized normal ---
// build() runs normalize_carefully with a zero fallback, so a sliver/degenerate
// triangle (common in BSP soup) stores a ZERO face normal. The swept path must
// keep the unit radial normal — otherwise move_and_slide
// gets a (0,0,0) normal, velocity.slide() asserts "must be normalized", and the
// body's velocity is zeroed (the reported console spam + a hard catch).
static void test_degenerate_tri_normal() {
	const T r = 0.4;
	// A real triangle whose closest feature to the capsule is its vertical x=0.5
	// edge, but passed with a ZERO face normal to emulate the degenerate case.
	V a = vec(0.5, -1, 0), b = vec(0.5, 1, 0), c = vec(0.7, 1, 0);
	V p1 = vec(0.2, -0.5, -0.6), p2 = vec(0.2, 0.5, -0.6); // spine offset toward the edge
	V dir = vec(0, 0, 1.2);
	V zero_fn = vec(0, 0, 0);
	auto res = hoptri::sweep_capsule_triangle<T>(p1, p2, dir, r, a, b, c, zero_fn, T {});
	assert(res.hit); // it does graze the edge
	T len = std::sqrt(res.normal.x * res.normal.x + res.normal.y * res.normal.y +
	                  res.normal.z * res.normal.z);
	assert(std::fabs(len - 1.0) < 1e-3); // unit radial kept; never the zero face_n
	printf("  degenerate-tri normal: hit, normal stays unit (len=%.4f)\n", len);
}

// --- Test 10: a capsule dropping onto a step top is NOT rejected at the edge ---
// walk_move climbs a step by lift → move forward → drop, and accepts the step only
// if the drop lands on a floor (down normal·UP >= 0.5). When the capsule drops near
// the step's front edge, the contact must report an up-ish step-TOP normal, not the
// vertical RISER face — otherwise landed_on_floor is false and the step can't be
// climbed. The radial closest-point normal is up-ish here, which is what we want.
static void test_step_drop_not_rejected() {
	const T r = 0.4, half = 0.5, H = 0.27; // 27cm step (well under step_height)
	std::vector<V> verts; std::vector<Tri> tris;
	auto quad = [&](V a, V b, V c, V d) {
		int i = (int)verts.size();
		verts.push_back(a); verts.push_back(b); verts.push_back(c); verts.push_back(d);
		tris.push_back({ i, i + 1, i + 2 }); tris.push_back({ i, i + 2, i + 3 });
		tris.push_back({ i, i + 2, i + 1 }); tris.push_back({ i, i + 3, i + 2 });
	};
	quad(vec(-5, 0, 0), vec(5, 0, 0), vec(5, 0, 5), vec(-5, 0, 5));       // lower floor y=0
	quad(vec(-5, 0, 0), vec(-5, H, 0), vec(5, H, 0), vec(5, 0, 0));       // riser z=0 (+z)
	quad(vec(-5, H, -5), vec(5, H, -5), vec(5, H, 0), vec(-5, H, 0));     // step top y=H
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	// Lifted capsule dropping onto the step top, from just past the edge (z<0) to
	// just short of it (z>0, overhanging the edge) — every offset must land "floor".
	int landed = 0;
	for (T cz = -0.20; cz <= 0.10 + 1e-9; cz += 0.05) {
		V start = vec(0, H + r + half + 0.30, cz);
		hop::collision<T> c = probe_swept(*mesh, body.get(), start, vec(0, -0.46, 0));
		assert(c.time < T(1));        // the drop lands
		assert(c.normal.y >= 0.5);    // up-ish → walk_move accepts (never the riser)
		++landed;
	}
	assert(landed == 7);
	printf("  step drop not rejected: %d/7 drops land on floor (n.y>=0.5)\n", landed);
}

// --- Test 11: col.impact is the real contact point ON the collidee surface ---
// Needed for lever arms (rotating-platform carry, angular response): col.point is
// the MOVER's origin at impact (~the capsule center); col.impact must be the
// witness point on the triangle, ~one radius away along the normal.
static void test_contact_point_on_surface() {
	const T r = 0.4, half = 0.5;

	// (a) Swept into a vertical wall at z=0 (normal +z). Contact point sits on the
	//     wall (z≈0); the mover origin stops ~radius behind it.
	{
		std::vector<V> verts; std::vector<Tri> tris;
		verts.push_back(vec(-5, -5, 0)); verts.push_back(vec(5, -5, 0));
		verts.push_back(vec(5, 5, 0)); verts.push_back(vec(-5, 5, 0));
		tris.push_back({ 0, 1, 2 }); tris.push_back({ 0, 2, 3 });
		tris.push_back({ 0, 2, 1 }); tris.push_back({ 0, 3, 2 }); // BSP twin
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);
		hop::collision<T> c = probe_swept(*mesh, body.get(), vec(0, 0, -1.0), vec(0, 0, 1.2));
		assert(c.time < T(1));                       // hit the wall
		assert(std::fabs(c.impact.z) < 0.02);        // contact ON the wall plane
		assert(c.point.z < -0.2);                    // mover origin is behind the wall
		assert(std::fabs((c.impact.z - c.point.z) - r) < 0.05); // ~one radius apart
		printf("  contact point (swept wall): impact.z=%.3f point.z=%.3f\n",
		       (double)c.impact.z, (double)c.point.z);
	}

	// (b) Resting on a floor at y=0 (static overlap): contact is on the floor below,
	//     not at the capsule center.
	{
		std::vector<V> verts; std::vector<Tri> tris;
		flat_quad(verts, tris, true); // UP-facing floor at y=0
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);
		hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, r + half, 0));
		assert(c.time == T {});                      // resting contact
		assert(std::fabs(c.impact.y) < 0.02);        // contact ON the floor
		assert(c.point.y > 0.5);                     // mover origin is up at the center
		printf("  contact point (resting floor): impact.y=%.3f point.y=%.3f\n",
		       (double)c.impact.y, (double)c.point.y);
	}

	// (c) BOX mover (support-plane path) swept into the same wall: contact on the
	//     wall (z≈0), box origin ~one half-extent behind.
	{
		const T hb = 0.4;
		std::vector<V> verts; std::vector<Tri> tris;
		verts.push_back(vec(-5, -5, 0)); verts.push_back(vec(5, -5, 0));
		verts.push_back(vec(5, 5, 0)); verts.push_back(vec(-5, 5, 0));
		tris.push_back({ 0, 1, 2 }); tris.push_back({ 0, 2, 3 });
		tris.push_back({ 0, 2, 1 }); tris.push_back({ 0, 3, 2 });
		auto mesh = make_mesh(verts, tris);
		auto box = make_box(hb, hb, hb);
		hop::collision<T> c = probe_swept(*mesh, box.get(), vec(0, 0, -1.0), vec(0, 0, 1.2));
		assert(c.time < T(1));
		assert(std::fabs(c.impact.z) < 0.02);        // contact ON the wall, not the box center
		assert(c.point.z < -0.2);
		printf("  contact point (swept box): impact.z=%.3f point.z=%.3f\n",
		       (double)c.impact.z, (double)c.point.z);
	}
}

// --- Test 12: a BOX mover is honored against a ROTATED trimesh ---------------
// Before Phase 5 the rotated-traceable driver skipped box movers (no rotation-
// invariant spine), so a box collided as if the mesh were unrotated. The OBB-vs-
// triangle path now handles it. Setup: a flat floor (xz-plane, +y normal) rotated
// 90° about Z becomes a vertical wall at world x=0 facing -x. An axis-aligned box
// swept in +x must stop at the wall with a world normal pointing -x and the
// contact point on the wall (x≈0).
static void test_box_mover_vs_rotated_mesh() {
	const T hb = 0.4;
	// Rz(90°): rows (0,-1,0),(1,0,0),(0,0,1) → maps local +y normal to world -x.
	hop::mat3<T> Rz90(0, -1, 0, 1, 0, 0, 0, 0, 1);

	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, true); // UP-facing floor at local y=0
	auto mesh = make_mesh(verts, tris);
	auto box = make_box(hb, hb, hb);

	auto probe = [&](const V & center, const V & motion) {
		box->set_position(center);
		hop::segment<T> seg;
		seg.set_start_end(center, vec(center.x + motion.x, center.y + motion.y, center.z + motion.z));
		hop::collision<T> c; c.reset();
		mesh->trace_solid(c, box.get(), vec(0, 0, 0), Rz90, seg, T {});
		return c;
	};

	// Swept from x=-1 toward +x into the wall at x=0; +x box face hits at center_x=-hb.
	hop::collision<T> sw = probe(vec(-1, 0, 0), vec(1.2, 0, 0));
	assert(sw.time < T(1));                          // box honored against the rotated mesh
	assert(approx(sw.time, (1.0 - hb) / 1.2, 0.03)); // center_x reaches -0.4
	assert(sw.normal.x < -0.9);                      // world normal faces -x (toward the box)
	assert(std::fabs(sw.impact.x) < 0.03);           // contact ON the wall plane (x≈0)
	printf("  box vs rotated mesh (swept): time=%.3f normal.x=%.2f impact.x=%.3f\n",
	       sw.time, sw.normal.x, sw.impact.x);

	// Static overlap: box straddling the wall reports an -x push-out.
	hop::collision<T> st = probe(vec(-0.2, 0, 0), vec(0, 0, 0));
	assert(st.time == T {});
	assert(st.normal.x < -0.9);
	assert(st.depth > T {});
	printf("  box vs rotated mesh (static): normal.x=%.2f depth=%.3f\n", st.normal.x, st.depth);
}

int main() {
	printf("test_trimesh_traceable:\n");
	test_recovery_front_face();
	test_recovery_mixed_winding();
	test_recovery_deep_sink_pushes_up();
	test_swept_two_sided();
	test_resting_allows_horizontal_motion();
	test_capsule_gap_fit();
	test_no_false_contact();
	test_ramp_climb();
	test_degenerate_tri_normal();
	test_step_drop_not_rejected();
	test_contact_point_on_surface();
	test_box_mover_vs_rotated_mesh();
	printf("ALL PASSED\n");
	return 0;
}
