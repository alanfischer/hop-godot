// Unit tests for HopBspTraceable — the GoldSrc BSP hull trace.
//
// The trace descends planes in NATIVE GoldSrc space (Z-up, inches) while the
// query arrives in Godot space (Y-up, metres). That in/out transform is the
// highest-risk detail in the whole feature: get it wrong and floors come back
// facing sideways. So every test here states its expectation in Godot space and
// asserts on Godot-space normals.
//
// All fixtures are hand-authored stripped BSP30 blobs — a header with a lump
// directory and only the lumps the trace reads — so the byte layout in
// hop_bsp_format.h is round-trip-checked against known-good bytes at the same
// time.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include <hop/hop.h>
#include "hop_bsp_traceable.h"

using T = double;
using V = hop::vec3<T>;
using namespace hop_bsp;

static const double SCALE = 0.025;  // WizardWars' GameConsts.SCALE_FACTOR

static V vec(T x, T y, T z) { V v; v.set(x, y, z); return v; }
static bool approx(T a, T b, T tol = 1e-3) { return std::fabs(a - b) < tol; }
static bool approx_v(const V &v, T x, T y, T z, T tol = 1e-3) {
	return approx(v.x, x, tol) && approx(v.y, y, tol) && approx(v.z, z, tol);
}

// --- synthetic blob authoring ---------------------------------------------

struct BlobBuilder {
	std::vector<BSPPlane> planes;
	std::vector<BSPNode> nodes;
	std::vector<BSPClipNode> clipnodes;
	std::vector<BSPLeaf> leafs;
	std::vector<BSPModel> models;

	std::vector<uint8_t> build() const {
		std::vector<uint8_t> out(sizeof(BSPHeader), 0);
		BSPHeader hdr {};
		hdr.version = HLBSP_VERSION;

		auto put = [&](int idx, const void *data, size_t sz) {
			if (sz == 0) return;
			hdr.lumps[idx].fileofs = (int32_t)out.size();
			hdr.lumps[idx].filelen = (int32_t)sz;
			const uint8_t *p = (const uint8_t *)data;
			out.insert(out.end(), p, p + sz);
		};
		put(LUMP_PLANES, planes.data(), planes.size() * sizeof(BSPPlane));
		put(LUMP_NODES, nodes.data(), nodes.size() * sizeof(BSPNode));
		put(LUMP_CLIPNODES, clipnodes.data(), clipnodes.size() * sizeof(BSPClipNode));
		put(LUMP_LEAFS, leafs.data(), leafs.size() * sizeof(BSPLeaf));
		put(LUMP_MODELS, models.data(), models.size() * sizeof(BSPModel));

		memcpy(out.data(), &hdr, sizeof(BSPHeader));
		return out;
	}
};

// Six axis planes bounding [mins, maxs], appended to `planes`. Returns the index
// of the first. Plane k*2 is the lower bound on axis k, k*2+1 the upper.
static int add_box_planes(BlobBuilder &b, const double mins[3], const double maxs[3]) {
	int first = (int)b.planes.size();
	for (int axis = 0; axis < 3; ++axis) {
		for (int hi = 0; hi < 2; ++hi) {
			BSPPlane p {};
			p.normal[axis] = 1.0f;
			p.dist = (float)(hi ? maxs[axis] : mins[axis]);
			p.type = axis;  // axial — exercises the fast `p[type] - dist` path
			b.planes.push_back(p);
		}
	}
	return first;
}

// A solid axis-aligned box brush as a 6-deep tree. Descending "inward" on every
// plane lands in SOLID; stepping outside any one of them lands in EMPTY.
//
// Leaf convention (matching a real BSP): leaf 0 is the solid leaf.
//
// `outside` overrides where "not in this box" goes, so brushes can be chained into a
// union: give brush A the root of brush B and the tree reads "solid if inside A, else
// test B" — which is what lets a fixture have a leaf bounded by another brush's face.
// Returns the root node/clipnode index of the brush just added.
static int add_box_brush(BlobBuilder &b, const double mins[3], const double maxs[3],
                         bool as_nodes, int contents = CONTENTS_SOLID,
                         int outside = 1) {  // 1 = "use the default empty child"
	const int p0 = add_box_planes(b, mins, maxs);
	const int base = as_nodes ? (int)b.nodes.size() : (int)b.clipnodes.size();

	if (as_nodes && b.leafs.empty()) {
		BSPLeaf solid {}; solid.contents = contents;
		BSPLeaf empty {}; empty.contents = CONTENTS_EMPTY;
		b.leafs.push_back(solid);
		b.leafs.push_back(empty);
	}
	// hull 0 addresses leafs as -(leaf+1); hulls 1..3 store the contents directly.
	const int SOLID_CHILD = as_nodes ? -1 : contents;
	const int EMPTY_CHILD = outside != 1 ? outside : (as_nodes ? -2 : CONTENTS_EMPTY);

	for (int i = 0; i < 6; ++i) {
		const bool upper = (i % 2) == 1;
		const int inward = (i == 5) ? SOLID_CHILD : (base + i + 1);
		int child0, child1;
		if (upper) { child0 = EMPTY_CHILD; child1 = inward; }
		else       { child0 = inward;      child1 = EMPTY_CHILD; }
		if (as_nodes) {
			BSPNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.nodes.push_back(n);
		} else {
			BSPClipNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.clipnodes.push_back(n);
		}
	}
	return base;
}

// One model whose hull 0 is a box brush and whose hulls 1..3 are that same brush
// expanded by each engine hull size — exactly what the map compiler bakes.
static std::vector<uint8_t> make_box_map(const double mins[3], const double maxs[3],
                                         int contents = CONTENTS_SOLID) {
	BlobBuilder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) { m.mins[i] = (float)mins[i]; m.maxs[i] = (float)maxs[i]; }

	m.headnode[0] = 0;
	add_box_brush(b, mins, maxs, /*as_nodes=*/true, contents);

	for (int h = 1; h < 4; ++h) {
		double emins[3], emaxs[3];
		for (int i = 0; i < 3; ++i) {
			emins[i] = mins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			emaxs[i] = maxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		m.headnode[h] = (int32_t)b.clipnodes.size();
		add_box_brush(b, emins, emaxs, /*as_nodes=*/false, contents);
	}
	b.models.push_back(m);
	return b.build();
}

// A floor with a wall standing on it, as a union of two brushes, sized so that in
// hull 1 the wall's solid begins exactly where the floor's ends (z = 36 — the floor
// top at 0, raised by the hull's 36-unit half-height).
//
// That seam is not contrived: it is where every standing player's trace point lives.
// A hull-1 mover is traced as the point at feet + 36, so a player resting on a floor
// sits EXACTLY on that floor's expanded top plane, and if they are pushed into a wall
// rising off that floor, the leaf they land in is bounded by that plane at distance
// zero. ww_golem's cockpit is this shape (deck at z=-112, seam at -76).
static std::vector<uint8_t> make_wall_on_floor_map() {
	// Floor: everything below z = 0. Wall: everything at x <= -8 above z = 72.
	// Expanded for hull h the floor tops out at -HULL.mins.z and the wall starts at
	// 72 - -HULL.mins.z, which meet for hull 1 (36) — the case under test. For the
	// other hulls they simply overlap, which is just as solid.
	const double fmins[3] = { -4096, -4096, -4096 }, fmaxs[3] = { 4096, 4096, 0 };
	const double wmins[3] = { -4096, -4096, 72 }, wmaxs[3] = { -8, 4096, 4096 };

	BlobBuilder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) { m.mins[i] = (float)fmins[i]; m.maxs[i] = (float)wmaxs[i]; }
	m.maxs[0] = (float)fmaxs[0];
	m.maxs[1] = (float)fmaxs[1];

	// Floor first so the wall can point its "outside" children at it.
	const int f0 = add_box_brush(b, fmins, fmaxs, /*as_nodes=*/true);
	m.headnode[0] = add_box_brush(b, wmins, wmaxs, /*as_nodes=*/true, CONTENTS_SOLID, f0);

	for (int h = 1; h < 4; ++h) {
		double efmins[3], efmaxs[3], ewmins[3], ewmaxs[3];
		for (int i = 0; i < 3; ++i) {
			efmins[i] = fmins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			efmaxs[i] = fmaxs[i] - hopbsp::HULL_SIZES[h].mins[i];
			ewmins[i] = wmins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			ewmaxs[i] = wmaxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		const int ef = add_box_brush(b, efmins, efmaxs, /*as_nodes=*/false);
		m.headnode[h] = add_box_brush(b, ewmins, ewmaxs, /*as_nodes=*/false, CONTENTS_SOLID, ef);
	}
	b.models.push_back(m);
	return b.build();
}

// A wide, thin slab centred on the GoldSrc origin: a floor whose top face sits at
// z = 0, i.e. Godot y = 0.
static std::vector<uint8_t> make_floor_map() {
	const double mins[3] = { -512, -512, -64 };
	const double maxs[3] = { 512, 512, 0 };
	return make_box_map(mins, maxs);
}

static std::unique_ptr<HopBspTraceable<T>> load(const std::vector<uint8_t> &blob,
                                                int blocking = hopbsp::BLOCK_SOLID) {
	auto t = std::make_unique<HopBspTraceable<T>>();
	bool ok = t->build(blob.data(), blob.size(), 0, (T)SCALE, blocking);
	assert(ok && "blob failed to parse");
	(void)ok;
	return t;
}

static std::shared_ptr<hop::solid<T>> make_box_solid(T hx, T hy, T hz) {
	hop::aa_box<T> b;
	b.mins = vec(-hx, -hy, -hz);
	b.maxs = vec(hx, hy, hz);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(b));
	return s;
}

// Sweep `s` from `from` along `motion` and return what it hit. Every solid test
// below is one of these plus its assertions; a zero `motion` is the static
// overlap query.
static hop::collision<T> sweep(HopBspTraceable<T> &t, std::shared_ptr<hop::solid<T>> s,
                               V from, V motion, T margin = T {}, V at = V {}) {
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(from, motion);
	t.trace_solid(c, s.get(), at, hop::mat3<T>(), seg, margin);
	return c;
}

// Same, for a point ray against hull 0.
static hop::collision<T> ray(HopBspTraceable<T> &t, V from, V dir, V at = V {},
                             hop::mat3<T> rot = hop::mat3<T>()) {
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(from, dir);
	t.trace_segment(c, at, rot, seg);
	return c;
}

// --- blob / format --------------------------------------------------------

static void test_blob_roundtrip() {
	auto blob = make_floor_map();
	BlobView v;
	assert(parse_blob(blob.data(), blob.size(), v));
	assert(v.plane_count == 6 * 4);      // hull 0 + three expanded hulls
	assert(v.node_count == 6);
	assert(v.clipnode_count == 18);
	assert(v.leaf_count == 2);
	assert(v.model_count == 1);
	assert(v.models[0].headnode[0] == 0);
	assert(v.models[0].headnode[1] == 0);   // first clipnode tree
	assert(v.models[0].headnode[3] == 12);

	// Truncation and a bad version must be rejected, not read past the end.
	BlobView bad;
	assert(!parse_blob(blob.data(), sizeof(BSPHeader) / 2, bad));
	auto wrong = blob;
	wrong[0] = 7;
	assert(!parse_blob(wrong.data(), wrong.size(), bad));
	printf("  blob_roundtrip ok\n");
}

// --- contents / point queries ---------------------------------------------

static void test_point_contents() {
	auto t = load(make_floor_map());
	// GoldSrc z = -32 (inside the slab) → Godot y = -32 * 0.025 = -0.8
	assert(t->contents_at_godot(vec(0, -0.8, 0)) == CONTENTS_SOLID);
	assert(t->contents_at_godot(vec(0, 0.8, 0)) == CONTENTS_EMPTY);
	// Well outside the slab's x extent (GoldSrc ±512 → Godot ∓12.8).
	assert(t->contents_at_godot(vec(20, -0.8, 0)) == CONTENTS_EMPTY);
	printf("  point_contents ok\n");
}

// --- trace_segment (hull 0, point ray) ------------------------------------

static void test_ray_hits_floor() {
	auto t = load(make_floor_map());
	hop::collision<T> c = ray(*t, vec(0, 2, 0), vec(0, -4, 0));  // straight down through y=0

	assert(c.time < 1.0);
	// Fraction 0.5 puts the hit at y=0; the epsilon backoff keeps it a hair above.
	assert(approx(c.time, 0.5, 0.01));
	assert(approx_v(c.point, 0, 0, 0, 0.01));
	assert(approx_v(c.normal, 0, 1, 0));  // the floor faces UP in Godot space
	printf("  ray_hits_floor ok\n");
}

static void test_ray_hits_wall() {
	// A slab standing in the x-z plane: solid for GoldSrc x in [0, 64].
	const double mins[3] = { 0, -512, -512 };
	const double maxs[3] = { 64, 512, 512 };
	auto t = load(make_box_map(mins, maxs));

	// GoldSrc +x is Godot -x, so approach from Godot -x moving +x and we hit the
	// GoldSrc x=64 face, whose Godot-space normal is -x.
	hop::collision<T> c = ray(*t, vec(-4, 0, 0), vec(8, 0, 0));
	assert(c.time < 1.0);
	assert(approx(c.point.x, -1.6, 0.01));  // GoldSrc x=64 → Godot x=-1.6
	assert(approx_v(c.normal, -1, 0, 0));
	printf("  ray_hits_wall ok\n");
}

static void test_ray_misses() {
	auto t = load(make_floor_map());
	hop::collision<T> c = ray(*t, vec(0, 2, 0), vec(0, 1, 0));  // upward, away from the floor
	assert(c.time == 1.0);
	printf("  ray_misses ok\n");
}

static void test_ray_from_inside_solid() {
	auto t = load(make_floor_map());
	hop::collision<T> c = ray(*t, vec(0, -0.8, 0), vec(0, 4, 0));  // starts inside the slab
	// Quake semantics: a trace that starts solid reports no impact plane — it must
	// not fabricate one, or a stuck body gets shoved by a garbage normal.
	assert(c.time == 1.0);
	printf("  ray_from_inside_solid ok\n");
}

static void test_ray_respects_body_position() {
	auto t = load(make_floor_map());
	// Same floor, but the body sits 10m up: the hit must ride with it.
	hop::collision<T> c = ray(*t, vec(0, 12, 0), vec(0, -4, 0), vec(0, 10, 0), hop::mat3<T>());
	assert(c.time < 1.0);
	assert(approx(c.point.y, 10.0, 0.01));
	assert(approx_v(c.normal, 0, 1, 0));
	printf("  ray_respects_body_position ok\n");
}

static void test_ray_respects_orientation() {
	// Yaw the body 90° about Godot +y: a wall in GoldSrc x becomes a wall in z.
	const double mins[3] = { 0, -512, -512 };
	const double maxs[3] = { 64, 512, 512 };
	auto t = load(make_box_map(mins, maxs));

	hop::mat3<T> rot;
	hop::set_mat3_from_axis_angle(rot, vec(0, 1, 0), (T)(M_PI / 2));

	// Un-rotated this ray (along -x, from +x) misses the solid side entirely;
	// rotated, the same wall now blocks a ray travelling along z.
	hop::collision<T> c = ray(*t, vec(0, 0, 4), vec(0, 0, -8), vec(0, 0, 0), rot);
	assert(c.time < 1.0);
	assert(approx(hop::length(c.normal), 1.0));
	// The face normal was Godot -x before the yaw; a +90° yaw about y sends it to +z.
	assert(approx_v(c.normal, 0, 0, 1));
	printf("  ray_respects_orientation ok\n");
}

// --- hull selection -------------------------------------------------------

static void test_hull_selection() {
	assert(hopbsp::hull_for_size(0, 0) == 0);          // point
	assert(hopbsp::hull_for_size(32, 72) == 1);        // standing player
	assert(hopbsp::hull_for_size(32, 36) == 3);        // crouched player
	assert(hopbsp::hull_for_size(64, 64) == 2);        // large monster
	assert(hopbsp::hull_for_size(8, 8) == 0);          // small projectile
	printf("  hull_selection ok\n");
}

// --- trace_solid (box hulls) ----------------------------------------------

static void test_solid_lands_on_floor() {
	auto t = load(make_floor_map());
	// A standing-player box: 32x32x72 GoldSrc = 0.8 x 1.8 x 0.8 Godot, so
	// half-extents (0.4, 0.9, 0.4). It must pick hull 1 and stop with its CENTRE
	// 36 units (0.9m) above the floor — not with its centre on the floor.
	auto s = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 3, 0), vec(0, -4, 0));

	assert(c.time < 1.0);
	assert(approx(c.point.y, 0.9, 0.02));
	assert(approx_v(c.normal, 0, 1, 0));
	printf("  solid_lands_on_floor ok\n");
}

// A mover whose origin is NOT the centre of its own box — Godot's convention for a
// character body, and WizardWars' actual player: the collider is offset upward so
// the origin sits at the feet.
static std::shared_ptr<hop::solid<T>> make_feet_box_solid(T hx, T height, T hz) {
	hop::aa_box<T> b;
	b.mins = vec(-hx, 0, -hz);
	b.maxs = vec(hx, height, hz);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(b));
	return s;
}

static void test_feet_origin_mover_lands_on_the_floor() {
	auto t = load(make_floor_map());
	// Same 32x32x72 player, but with its origin at the feet. It must come to rest
	// with its ORIGIN on the floor (y=0), not its centre.
	auto s = make_feet_box_solid(0.4, 1.8, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 3, 0), vec(0, -4, 0));
	assert(c.time < 1.0);
	assert(approx(c.point.y, 0.0, 0.02));
	assert(approx_v(c.normal, 0, 1, 0));
	printf("  feet_origin_mover_lands_on_the_floor ok\n");
}

// The regression that dropped bots through ww_2fort: with the hull offset applied
// backwards, a feet-origin mover standing ON the floor traces from half a body
// BELOW its feet, reads as deeply embedded, and gets a downward push-out that
// shoves it through the world. A body resting on the floor must be in the clear.
static void test_feet_origin_mover_resting_is_not_stuck() {
	auto t = load(make_floor_map());
	auto s = make_feet_box_solid(0.4, 1.8, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 0.001, 0), vec(0, 0, 0));  // standing on the floor
	assert(c.time == 1.0 && "a body resting on the floor must not report as overlapping");

	// And a downward sweep from there must not be waved through.
	hop::collision<T> down = sweep(*t, s, vec(0, 0.001, 0), vec(0, -2, 0));
	assert(down.time < 1.0);
	assert(down.normal.y > 0.5 && "the floor must push UP, never down");
	printf("  feet_origin_mover_resting_is_not_stuck ok\n");
}

static void test_solid_crouched_uses_short_hull() {
	auto t = load(make_floor_map());
	// 32x32x36 GoldSrc → half-extents (0.4, 0.45, 0.4). Hull 3 stops the centre
	// 18 units (0.45m) up.
	auto s = make_box_solid(0.4, 0.45, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 3, 0), vec(0, -4, 0));
	assert(c.time < 1.0);
	assert(approx(c.point.y, 0.45, 0.02));
	printf("  solid_crouched_uses_short_hull ok\n");
}

static void test_solid_impact_is_on_the_surface() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 3, 0), vec(0, -4, 0));
	// `point` is the mover's origin at impact; `impact` is the witness point on the
	// hull surface, which the lever-arm math needs. They must not be the same.
	assert(approx(c.point.y, 0.9, 0.02));
	assert(approx(c.impact.y, 0.0, 0.02));
	printf("  solid_impact_is_on_the_surface ok\n");
}

static void test_solid_misses() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> c = sweep(*t, s, vec(0, 3, 0), vec(0, 1, 0));
	assert(c.time == 1.0);
	printf("  solid_misses ok\n");
}

static void test_solid_overlap_pushes_out() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Zero direction = static overlap query, with the box centre 0.4m up: hull 1
	// wants 0.9m, so it is 0.5m deep in the expanded hull and must be pushed UP.
	hop::collision<T> c = sweep(*t, s, vec(0, 0.4, 0), vec(0, 0, 0));

	assert(c.time == 0.0);
	assert(approx_v(c.normal, 0, 1, 0));
	assert(approx(c.depth, 0.5, 0.02));
	printf("  solid_overlap_pushes_out ok\n");
}

static void test_solid_margin_finds_resting_contact() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Resting exactly on the floor with a 4cm speculative margin: not penetrating,
	// but the contact must still be reported, with depth measured against the
	// INFLATED surface (depth = margin - true_gap; here the gap is ~0).
	hop::collision<T> c = sweep(*t, s, vec(0, 0.92, 0), vec(0, 0, 0), (T)0.04);

	assert(c.time == 0.0);
	assert(approx_v(c.normal, 0, 1, 0));
	assert(c.depth > 0.0 && c.depth < 0.04);
	printf("  solid_margin_finds_resting_contact ok\n");
}

static void test_solid_starting_stuck_reports_overlap() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Mappers routinely sink a spawn a hair into the floor (ww_countryside puts
	// ten of them at z=351.9 where the hull surface is 352). A SWEPT trace from
	// inside solid must report the overlap, not a clear path.
	//
	// Quake reports startsolid and leaves the trace clear, because its callers
	// check that flag and refuse the move. hop's caller has no such flag, so a
	// clear trace reads as "the whole path is free" — and a body standing a hair
	// inside the floor then sweeps freely downward under gravity, sinking further
	// every frame until it drops out of the world. That is exactly what dropped
	// bots through ww_2fort's ramps to y=-103.
	//
	// Depth is measured against the hull shrunk by STUCK_SLOP, so the reported
	// penetration is the real 0.1 less that band — the mover is deliberately left a
	// hair inside rather than resolved to exactly zero overlap. That is the point of
	// the band, and the recovery adds a margin-sized skin on top anyway, so nothing
	// ends up embedded. Anything shallower than the band is not called stuck at all.
	const double slop = hopbsp::STUCK_SLOP;
	//
	// A swept query from inside solid is answered by stepping off the surface and
	// sweeping from there, so it comes back as an ordinary surface hit at ~zero
	// distance rather than as an overlap: the mover is stopped by the floor, which is
	// the property that matters. (It must not come back CLEAR — that is the bug this
	// test guards.) The overlap itself is the zero-direction query's job, below, and
	// that is the call the recovery step actually makes.
	hop::collision<T> swept = sweep(*t, s, vec(0, 0.8975, 0), vec(0, -4, 0));  // 0.1 GoldSrc units low
	assert(swept.time < 0.01 && "a sweep from inside solid must not report a clear path");
	assert(approx_v(swept.normal, 0, 1, 0));

	// The zero-direction query reports the same thing: push straight up, by
	// the 0.1 units it is buried less the slop band.
	hop::collision<T> rec = sweep(*t, s, vec(0, 0.8975, 0), vec(0, 0, 0));
	assert(rec.time == 0.0);
	assert(approx_v(rec.normal, 0, 1, 0));
	assert(approx(rec.depth / SCALE, 0.1 - slop, 0.005));  // in GoldSrc units

	// A mover resting exactly ON the surface is not stuck, and neither is one within
	// the slop band. This is the case that matters most in play: a standing player
	// traces as the point at feet+36, which IS the floor's expanded top plane.
	hop::collision<T> resting = sweep(*t, s, vec(0, 0.9, 0), vec(0, 0, 0));
	assert(resting.depth == 0.0 && "a mover resting on a floor must not read as stuck");
	printf("  solid_starting_stuck_reports_overlap ok\n");
}

// Depenetration must nudge, never launch. A body buried deeper than its own size has
// no escape within reach, so hull_push_out falls back to the nearest plane of the
// LEAF it landed in — and a leaf can be enormous, so without a cap the body would be
// ejected metres in one query, with hop iterating recovery on top. Being buried is
// reachable in practice wherever the clip hull disagrees with the render geometry,
// i.e. any CLIP brush, since those exist only in hulls 1..3.
// A mover flush against a wall — the pose depenetration always leaves it in — must
// still be stopped by that wall.
//
// The hull walk calls a start exactly ON a plane solid and gives up, reporting no
// surface at all, which reads as "nothing in the way" and hands back the whole step.
// It is not an exotic pose: recovery pushes a body out of a wall to precisely its
// face, so the very next sweep that tick is taken from there. On ww_golem that let a
// rider in the cockpit corner walk out through 15 cm of wall, one step per tick.
// A hull a hair INSIDE the floor it stands on must still be stopped by a wall.
//
// This is the ordinary state of standing on a moving platform: the deck bobs, and for
// a tick the mover is a fraction of a unit below the surface it is resting on. If a
// start inside the floor lets the horizontal sweep report a clear path, the mover
// walks straight through whatever is beside it — and 0.07 GoldSrc units is under two
// millimetres, which is noise, not a pose anyone can avoid.
//
// Measured on ww_golem: a rider whose feet were 0.07 units into the cockpit deck swept
// the full 8 units backwards through a rear wall standing 2.6 units behind them.
static void test_sunk_into_the_floor_still_hits_the_wall() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Resting height is Godot y = 0.9 (the box centre IS the hull-1 trace point, and
	// the floor's expanded top is GoldSrc z = 36). Drop it 0.07 units below that.
	const T sunk = (T)(0.9 - 0.07 * SCALE);
	// Start 12 GoldSrc units clear of the wall face (GoldSrc x = 8) and drive into it.
	// Godot +x is GoldSrc -x, so the wall is 12 units along a 16-unit sweep.
	hop::collision<T> c = sweep(*t, s, vec((T)(-20 * SCALE), sunk, (T)0),
	                            vec((T)(16 * SCALE), 0, 0));
	// The trace must not hand back a clear path. It answers with the floor it is in
	// rather than the wall ahead — that is this layer's contract, and _body_test_motion
	// is what turns it into a fraction — but "you are inside something" and "the way is
	// clear" must never be the same answer.
	assert(c.time < 0.01 && "a mover sunk into the floor must not get a clear sweep");
	assert(c.depth > T {} && "being inside the floor must be reported as an overlap");
}

static void test_flush_against_a_wall_is_still_blocked() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// In hull 1 the wall's solid is GoldSrc x <= 8 (its face at -8 expanded by the
	// hull's 16). Sit half of STUCK_SLOP inside it: the stuck test measures against a
	// hull shrunk by that band, so this is a pose it calls free and a pose a push-out
	// is content to leave a body in — while the sweep, which uses no such band, still
	// begins in solid. Godot +x is GoldSrc -x, so +x drives into the wall, -x leaves.
	const V flush = vec((T)(-(8 - hopbsp::STUCK_SLOP * 0.5) * SCALE), (T)1.5, (T)0);

	hop::collision<T> into = sweep(*t, s, flush, vec((T)0.15, 0, 0));
	assert(into.time < 0.01 && "a mover flush on a wall face must be stopped by that wall");
	assert(approx_v(into.normal, -1, 0, 0));

	// The same pose moving AWAY is free — the fix must block the wall, not pin the
	// body to it. Backing the start off along the motion puts it inside the solid it
	// is leaving, the walk gives up again, and the move stays clear.
	hop::collision<T> away = sweep(*t, s, flush, vec((T)-0.15, 0, 0));
	assert(away.time > 0.99 && "a mover flush on a wall must stay free to leave it");
}

// Sliding DOWN a wall face, a mover must still land on the floor that wall stands on.
//
// The seam reached sideways. Where the wall face meets the floor the two expanded solids
// share a corner point, and contents being a strict d < 0 puts a mover arriving exactly
// there inside NEITHER. The resting-gap invariant only lifts along gravity, so it does
// not cover this approach — the traceable itself has to report the floor.
static void test_sliding_down_a_wall_face_still_lands_on_the_floor() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Flush on the wall face (the pose a slide leaves a body in), well above the floor,
	// sweeping straight down past the floor's expanded top at Godot y = 0.9.
	const V flush = vec((T)(-8 * SCALE), (T)1.5, (T)0);
	hop::collision<T> down = sweep(*t, s, flush, vec((T)0, (T)-1.2, (T)0));
	assert(down.time < (T)0.99 && "a mover sliding down a wall must not fall through the floor");
	assert(down.normal.y > (T)0.5 && "the surface that stops it must be the floor");

	// The same, half a slop band inside the wall — the pose a push-out is content to
	// leave a body in, and the one a real slide actually produces.
	const V inside = vec((T)(-(8 - hopbsp::STUCK_SLOP * 0.5) * SCALE), (T)1.5, (T)0);
	hop::collision<T> in_down = sweep(*t, s, inside, vec((T)0, (T)-1.2, (T)0));
	assert(in_down.time < (T)0.99 &&
	       "a mover a hair inside a wall must still be stopped by the floor below it");

	// The motion a falling rider actually has: diagonal, driving into the wall while
	// dropping past the floor. Down and sideways are both answered correctly on their
	// own; the composite must not be assumed to follow from them.
	const V clear_of_face = vec((T)(-8.4 * SCALE), (T)(0.9 + 2 * SCALE), (T)0);
	hop::collision<T> diag = sweep(*t, s, clear_of_face,
	                               vec((T)(1.99 * SCALE), (T)(-4.49 * SCALE), (T)0));
	assert(diag.time < (T)0.99 &&
	       "a mover driving into a wall while falling must still be stopped");
}

// A mover a HAIR inside a wall face must still be reported as inside it.
//
// Three hundredths of a unit is under the DIST_EPSILON a trace backs its endpoints off
// by, so it is where traces naturally leave a body, not an unusual pose. Read as clear,
// recovery has nothing to push out of and the next sweep begins inside solid unnoticed.
static void test_a_hair_inside_a_wall_is_still_inside_it() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// The wall's expanded solid is GoldSrc x <= 8; sit 0.03 units inside it. Godot +x
	// is GoldSrc -x, so a more negative Godot x is a larger GoldSrc x.
	const V hair = vec((T)(-(8 - 0.03) * SCALE), (T)1.5, (T)0);

	hop::collision<T> still = sweep(*t, s, hair, vec((T)0, (T)0, (T)0));
	assert(still.depth > T {} && "a mover 0.03 units into a wall must report an overlap");
	assert(still.normal.x < (T)-0.5 && "and the way out must point out of that wall");

	// And the consequence: falling from that pose must still be stopped.
	hop::collision<T> fall = sweep(*t, s, hair, vec((T)0, (T)-1.2, (T)0));
	assert(fall.time < (T)0.99 &&
	       "a mover a hair inside a wall must not fall straight through the floor");
}

// Already buried in a wall, a mover must not be free to travel deeper into it.
//
// Sweeping from inside solid finds no entry plane — the next surface along the path
// is the wall's FAR face — so an unguarded trace reports a clear path all the way
// through and out the other side. Measured on ww_golem: a rider 0.1 units into the
// cockpit's side wall was handed the full step and came out the back of it.
static void test_inside_a_wall_cannot_travel_deeper() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// One GoldSrc unit past the face, i.e. inside the wall.
	const V buried = vec((T)(-7 * SCALE), (T)1.5, (T)0);

	hop::collision<T> deeper = sweep(*t, s, buried, vec((T)0.15, 0, 0));
	assert(deeper.time < 0.01 && "a buried mover must not sweep freely deeper into the solid");
	assert(deeper.depth > T {} && "being inside solid must be reported as an overlap");
}

static void test_deep_overlap_is_nudged_not_launched() {
	// A thick slab: 512 units deep, so its interior leaf is far from any face.
	const double mins[3] = { -512, -512, -512 };
	const double maxs[3] = { 512, 512, 0 };
	auto t = load(make_box_map(mins, maxs));
	auto s = make_feet_box_solid(0.4, 1.8, 0.4);

	// Sitting 200 units (5m) inside it — the ejection must stay bounded by the
	// hull's own half-width, not the distance to the far side of the leaf.
	hop::collision<T> c = sweep(*t, s, vec(0, -5.0, 0), vec(0, 0, 0));
	assert(c.time == 0.0);
	// Pushed straight up, so the bound is the hull's height: 72 units, not the
	// 200 units to the nearest face of the leaf it is sitting in.
	const double hull_height_m = 72.0 * SCALE;
	assert(c.depth > 0.0);
	assert(c.depth <= hull_height_m + 1e-6 && "one recovery step must not teleport the body");
	assert(c.depth < 5.0 && "the raw leaf-plane distance would be metres");
	printf("  deep_overlap_is_nudged_not_launched ok\n");
}

// Depenetration must aim somewhere that actually leaves the solid. The nearest plane
// bounding a mover's LEAF is not that: the compiler cuts a solid region into many
// convex cells, so most of a leaf's faces open onto the next cell of the same brush.
//
// The case that bites is a player standing on a floor and squeezed into a wall rising
// off it — a moving brush entity sweeping into them will do it. Their trace point sits
// exactly on the floor's expanded top plane, so that plane wins at distance ZERO over
// the wall face they actually came through, and the push-out comes back pointing into
// the floor they are standing on, with no depth. Nothing then ejects them; worse,
// _body_test_motion reads the vertical normal as "flush, not moving into it" and hands
// back the whole motion, so the player simply walks out through the wall. That is the
// golem cockpit bug: riders shot out of the back of ww_golem while it walked.
static void test_push_out_picks_a_direction_that_exits() {
	auto t = load(make_wall_on_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);  // 32x32x72, centre origin → hull 1, no offset

	// Trace point at GoldSrc (-20, 0, 36): 28 units inside the wall's expanded face
	// (x = 8) and exactly ON the floor's expanded top (z = 36). Godot (0.5, 0.9, 0).
	hop::collision<T> c = sweep(*t, s, vec(0.5, 0.9, 0.0), vec(0, 0, 0));
	assert(c.time == 0.0 && "a point inside solid must report an overlap");
	// The nearest leaf face is the seam, at distance 0, pushing DOWN into the floor —
	// the answer that used to come back, and a dead end. The only way out is sideways.
	assert(approx_v(c.normal, -1, 0, 0) && "push-out must aim out of the wall, not into the floor");
	assert(approx(c.depth / SCALE, 28.0, 0.1));

	// Control: the same penetration higher up the wall, where the nearest face was
	// already the right one. Unchanged, and the seam costs nothing when it isn't near.
	hop::collision<T> high = sweep(*t, s, vec(0.5, 5.0, 0.0), vec(0, 0, 0));
	assert(high.time == 0.0);
	assert(approx_v(high.normal, -1, 0, 0));
	assert(approx(high.depth / SCALE, 28.0, 0.1));
	printf("  push_out_picks_a_direction_that_exits ok\n");
}

// --- contents filtering ---------------------------------------------------

static void test_sky_is_passable_but_maskable() {
	const double mins[3] = { -512, -512, -64 };
	const double maxs[3] = { 512, 512, 0 };
	auto blob = make_box_map(mins, maxs, CONTENTS_SKY);

	// Default (block solid only): a ray flies straight through a sky brush — which
	// is exactly how a projectile leaves the map in GoldSrc.
	{
		auto t = load(blob);
		assert(ray(*t, vec(0, 2, 0), vec(0, -4, 0)).time == 1.0);
	}
	// And with the VOID behind the sky, as a real map has: outside the world is
	// CONTENTS_SOLID, so a naive trace stops just past the sky brush and the
	// projectile detonates in mid-air. It has to pass through both.
	{
		BlobBuilder b;
		BSPModel m {};
		const double smins[3] = { -512, -512, -64 }, smaxs[3] = { 512, 512, 0 };
		for (int i = 0; i < 3; ++i) { m.mins[i] = (float)smins[i]; m.maxs[i] = (float)smaxs[i]; }
		m.headnode[0] = 0;
		add_box_brush(b, smins, smaxs, /*as_nodes=*/true, CONTENTS_SKY);
		// The solid void immediately below the sky slab.
		const double vmins[3] = { -512, -512, -256 }, vmaxs[3] = { 512, 512, -64 };
		BSPLeaf solid_void {}; solid_void.contents = CONTENTS_SOLID;
		b.leafs.push_back(solid_void);                 // leaf 2
		const int p0 = add_box_planes(b, vmins, vmaxs);
		const int base = (int)b.nodes.size();
		for (int i = 0; i < 6; ++i) {
			const bool upper = (i % 2) == 1;
			const int inward = (i == 5) ? -3 : (base + i + 1);   // -3 => leaf 2, solid
			BSPNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)(upper ? -2 : inward);
			n.children[1] = (int16_t)(upper ? inward : -2);
			b.nodes.push_back(n);
		}
		// Chain the sky tree's "outside" exits into the void tree.
		for (int i = 0; i < 6; ++i)
			for (int c = 0; c < 2; ++c)
				if (b.nodes[i].children[c] == -2) b.nodes[i].children[c] = (int16_t)base;
		for (int h = 1; h < 4; ++h) m.headnode[h] = 0;
		b.models.push_back(m);
		auto t = load(b.build());
		assert(t->contents_at_godot(vec(0, -0.8, 0)) == CONTENTS_SKY);
		assert(t->contents_at_godot(vec(0, -3.0, 0)) == CONTENTS_SOLID);
		assert(ray(*t, vec(0, 2, 0), vec(0, -8, 0)).time == 1.0
			&& "a ray through sky must not stop on the void behind it");
	}
	// The same map viewed with sky blocking — how player movement sees it.
	{
		auto t = load(blob, hopbsp::blocking_bit(CONTENTS_SKY));
		hop::collision<T> c = ray(*t, vec(0, 2, 0), vec(0, -4, 0));
		assert(c.time < 1.0);
		assert(approx_v(c.normal, 0, 1, 0));
	}
	printf("  sky_is_passable_but_maskable ok\n");
}

// hop ships with hop_scalar = float, so the whole thing has to instantiate and
// stay accurate at that width. The descent itself is always double internally;
// only the interface narrows.
static void test_float_instantiation() {
	auto blob = make_floor_map();
	HopBspTraceable<float> t;
	assert(t.build(blob.data(), blob.size(), 0, (float)SCALE));

	hop::collision<float> c;
	hop::segment<float> seg;
	hop::vec3<float> o, d;
	o.set(0.0f, 2.0f, 0.0f);
	d.set(0.0f, -4.0f, 0.0f);
	seg.set_start_dir(o, d);
	hop::vec3<float> zero;
	t.trace_segment(c, zero, hop::mat3<float>(), seg);
	assert(std::fabs(c.time - 0.5f) < 0.01f);
	assert(std::fabs(c.normal.y - 1.0f) < 1e-3f);

	hop::aa_box<float> box;
	box.mins.set(-0.4f, -0.9f, -0.4f);
	box.maxs.set(0.4f, 0.9f, 0.4f);
	auto s = std::make_shared<hop::solid<float>>();
	s->add_shape(std::make_shared<hop::shape<float>>(box));
	hop::collision<float> c2;
	o.set(0.0f, 3.0f, 0.0f);
	seg.set_start_dir(o, d);
	t.trace_solid(c2, s.get(), zero, hop::mat3<float>(), seg, 0.0f);
	assert(std::fabs(c2.point.y - 0.9f) < 0.02f);
	printf("  float_instantiation ok\n");
}

int main() {
	printf("test_bsp_traceable\n");
	test_blob_roundtrip();
	test_point_contents();
	test_ray_hits_floor();
	test_ray_hits_wall();
	test_ray_misses();
	test_ray_from_inside_solid();
	test_ray_respects_body_position();
	test_ray_respects_orientation();
	test_hull_selection();
	test_solid_lands_on_floor();
	test_feet_origin_mover_lands_on_the_floor();
	test_feet_origin_mover_resting_is_not_stuck();
	test_solid_crouched_uses_short_hull();
	test_solid_impact_is_on_the_surface();
	test_solid_misses();
	test_solid_overlap_pushes_out();
	test_solid_margin_finds_resting_contact();
	test_solid_starting_stuck_reports_overlap();
	test_sunk_into_the_floor_still_hits_the_wall();
	test_flush_against_a_wall_is_still_blocked();
	test_sliding_down_a_wall_face_still_lands_on_the_floor();
	test_a_hair_inside_a_wall_is_still_inside_it();
	test_inside_a_wall_cannot_travel_deeper();
	test_deep_overlap_is_nudged_not_launched();
	test_push_out_picks_a_direction_that_exits();
	test_sky_is_passable_but_maskable();
	test_float_instantiation();
	printf("all bsp traceable tests passed\n");
	return 0;
}
