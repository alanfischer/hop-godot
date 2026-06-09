// Unit tests for HopTrimeshTraceable's winding-agnostic collision.
//
// Background: GoldSrc BSP geometry (via build_hull_collision) produces triangle
// soup whose per-triangle winding is NOT globally consistent — walkable surfaces
// can carry coincident, oppositely-wound triangles. The trimesh trace must behave
// like Godot's concave collision (winding-agnostic / double-sided): a body is
// pushed out of, and stopped by, the surface FACING it, regardless of winding.
//
// These tests pin the invariants that fix the "fall through ramps/floors" and
// "stuck" bugs:
//   1. Zero-direction recovery returns a body-facing normal + the true (small)
//      penetration depth — never the inward normal with a ~2*extent bogus depth
//      that shoved the body through the surface.
//   2. That holds for either winding AND for coincident opposite-wound pairs.
//   3. The swept cast registers a hit with a body-facing normal even when a
//      triangle's stored winding is inverted.
//   4. The seam tolerance bridges a hairline gap between triangles but does not
//      paper over a genuine hole.

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
	mesh.trace_solid(c, body, vec(0, 0, 0), seg, T {});
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
	mesh.trace_solid(c, body, vec(0, 0, 0), seg, T {});
	return c;
}

// One horizontal quad at y=0 (a flat floor), with explicit winding.
static void flat_quad(std::vector<V> & v, std::vector<Tri> & t, bool reversed) {
	int i = (int)v.size();
	v.push_back(vec(-5, 0, -5));
	v.push_back(vec(5, 0, -5));
	v.push_back(vec(5, 0, 5));
	v.push_back(vec(-5, 0, 5));
	if (!reversed) { t.push_back({ i, i + 1, i + 2 }); t.push_back({ i, i + 2, i + 3 }); }
	else           { t.push_back({ i, i + 2, i + 1 }); t.push_back({ i, i + 3, i + 2 }); }
}

// --- Test 1: resting recovery is winding-agnostic and reports the TRUE depth ---
// A capsule resting on a flat floor must depenetrate UP with ~0 depth, whether
// the floor triangles are wound up or down. The old deepest-face/raw-winding
// logic reported ~2*extent (=1.8 here) with a downward normal for the reversed
// winding — which shoved the body through the floor.
static void test_recovery_winding_agnostic() {
	const T r = 0.4, half = 0.5, extent = r + half; // capsule lowest point = center.y - extent
	for (bool reversed : { false, true }) {
		std::vector<V> verts; std::vector<Tri> tris;
		flat_quad(verts, tris, reversed);
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);

		// Resting exactly on the floor: lowest point at y=0 → center at y=extent.
		hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, extent, 0));
		assert(c.time == T {});                 // static overlap detected
		assert(c.normal.y > 0.5);               // pushes UP (toward the body), not down
		assert(c.depth < 0.25);                 // ~0, never the ~2*extent (1.8) bogus depth
		assert(c.depth < extent);               // strictly less than one body extent

		// Sunk 0.1 into the floor → depth ~0.1, still up.
		hop::collision<T> s = probe_static(*mesh, body.get(), vec(0, extent - 0.1, 0));
		assert(s.normal.y > 0.5);
		assert(approx(s.depth, 0.1, 0.05));
		printf("  recovery winding-agnostic (reversed=%d): normal.y=%.2f depth=%.3f\n",
		       (int)reversed, c.normal.y, c.depth);
	}
}

// --- Test 2: coincident opposite-wound triangles (the BSP case) ---
// The exact failure: a walkable surface carries BOTH an up- and a down-wound
// triangle. A capsule resting on top must still get a small upward depenetration,
// not a ~2*extent downward shove from the inverted twin.
static void test_coincident_opposite_faces() {
	const T r = 0.4, half = 0.5, extent = r + half;
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, false); // up-wound
	flat_quad(verts, tris, true);  // coincident down-wound twin
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(r, half);

	hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, extent, 0));
	assert(c.time == T {});
	assert(c.normal.y > 0.5);     // up, despite the inverted twin
	assert(c.depth < 0.25);       // not the ~1.8 bogus depth
	printf("  coincident opposite faces: normal.y=%.2f depth=%.3f\n", c.normal.y, c.depth);
}

// --- Test 3: swept cast is winding-agnostic ---
// A capsule dropped onto a single floor triangle must register a hit with an
// upward-facing normal even when that triangle's stored winding points DOWN.
static void test_swept_winding_agnostic() {
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
		printf("  swept winding-agnostic (reversed=%d): time=%.3f normal.y=%.2f\n",
		       (int)reversed, c.time, c.normal.y);
	}
}

// --- Test 4: seam tolerance bridges a hairline gap, not a real hole ---
static void test_seam_tolerance() {
	const T r = 0.4, half = 0.5, extent = r + half;
	auto run = [&](T gap) {
		std::vector<V> verts; std::vector<Tri> tris;
		// two floor halves split along x=0 with a `gap`-wide slit
		int i = (int)verts.size();
		verts.push_back(vec(-5, 0, -5)); verts.push_back(vec(-gap / 2, 0, -5));
		verts.push_back(vec(-gap / 2, 0, 5)); verts.push_back(vec(-5, 0, 5));
		tris.push_back({ i, i + 1, i + 2 }); tris.push_back({ i, i + 2, i + 3 });
		i = (int)verts.size();
		verts.push_back(vec(gap / 2, 0, -5)); verts.push_back(vec(5, 0, -5));
		verts.push_back(vec(5, 0, 5)); verts.push_back(vec(gap / 2, 0, 5));
		tris.push_back({ i, i + 1, i + 2 }); tris.push_back({ i, i + 2, i + 3 });
		auto mesh = make_mesh(verts, tris);
		auto body = make_capsule(r, half);
		// drop centered exactly over the slit
		hop::collision<T> c = probe_swept(*mesh, body.get(), vec(0, extent + 1.0, 0), vec(0, -2.0, 0));
		return c.time < T(1);
	};
	assert(run(0.02));   // 2cm hairline seam → bridged (caught)
	assert(!run(0.30));  // 30cm real hole → correctly NOT bridged
	printf("  seam tolerance: 2cm gap caught, 30cm hole not\n");
}

// --- Test 5: a body clear of the mesh reports no contact ---
static void test_no_false_contact() {
	std::vector<V> verts; std::vector<Tri> tris;
	flat_quad(verts, tris, false);
	auto mesh = make_mesh(verts, tris);
	auto body = make_capsule(0.4, 0.5);
	hop::collision<T> c = probe_static(*mesh, body.get(), vec(0, 5.0, 0)); // well above
	assert(c.time != T {}); // no static overlap
	printf("  no false contact when clear\n");
}

int main() {
	printf("test_trimesh_traceable:\n");
	test_recovery_winding_agnostic();
	test_coincident_opposite_faces();
	test_swept_winding_agnostic();
	test_seam_tolerance();
	test_no_false_contact();
	printf("ALL PASSED\n");
	return 0;
}
