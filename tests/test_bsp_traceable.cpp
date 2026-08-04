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
static void add_box_brush(BlobBuilder &b, const double mins[3], const double maxs[3],
                          bool as_nodes, int contents = CONTENTS_SOLID) {
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
	const int EMPTY_CHILD = as_nodes ? -2 : CONTENTS_EMPTY;

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
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 2, 0), vec(0, -4, 0));  // straight down through y=0
	t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);

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
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(-4, 0, 0), vec(8, 0, 0));
	t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);
	assert(c.time < 1.0);
	assert(approx(c.point.x, -1.6, 0.01));  // GoldSrc x=64 → Godot x=-1.6
	assert(approx_v(c.normal, -1, 0, 0));
	printf("  ray_hits_wall ok\n");
}

static void test_ray_misses() {
	auto t = load(make_floor_map());
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 2, 0), vec(0, 1, 0));  // upward, away from the floor
	t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);
	assert(c.time == 1.0);
	printf("  ray_misses ok\n");
}

static void test_ray_from_inside_solid() {
	auto t = load(make_floor_map());
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, -0.8, 0), vec(0, 4, 0));  // starts inside the slab
	t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);
	// Quake semantics: a trace that starts solid reports no impact plane — it must
	// not fabricate one, or a stuck body gets shoved by a garbage normal.
	assert(c.time == 1.0);
	printf("  ray_from_inside_solid ok\n");
}

static void test_ray_respects_body_position() {
	auto t = load(make_floor_map());
	// Same floor, but the body sits 10m up: the hit must ride with it.
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 12, 0), vec(0, -4, 0));
	t->trace_segment(c, vec(0, 10, 0), hop::mat3<T>(), seg);
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

	hop::collision<T> c;
	hop::segment<T> seg;
	// Un-rotated this ray (along -x, from +x) misses the solid side entirely;
	// rotated, the same wall now blocks a ray travelling along z.
	seg.set_start_dir(vec(0, 0, 4), vec(0, 0, -8));
	t->trace_segment(c, vec(0, 0, 0), rot, seg);
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
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 3, 0), vec(0, -4, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});

	assert(c.time < 1.0);
	assert(approx(c.point.y, 0.9, 0.02));
	assert(approx_v(c.normal, 0, 1, 0));
	printf("  solid_lands_on_floor ok\n");
}

static void test_solid_crouched_uses_short_hull() {
	auto t = load(make_floor_map());
	// 32x32x36 GoldSrc → half-extents (0.4, 0.45, 0.4). Hull 3 stops the centre
	// 18 units (0.45m) up.
	auto s = make_box_solid(0.4, 0.45, 0.4);
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 3, 0), vec(0, -4, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});
	assert(c.time < 1.0);
	assert(approx(c.point.y, 0.45, 0.02));
	printf("  solid_crouched_uses_short_hull ok\n");
}

static void test_solid_impact_is_on_the_surface() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 3, 0), vec(0, -4, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});
	// `point` is the mover's origin at impact; `impact` is the witness point on the
	// hull surface, which the lever-arm math needs. They must not be the same.
	assert(approx(c.point.y, 0.9, 0.02));
	assert(approx(c.impact.y, 0.0, 0.02));
	printf("  solid_impact_is_on_the_surface ok\n");
}

static void test_solid_misses() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 3, 0), vec(0, 1, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});
	assert(c.time == 1.0);
	printf("  solid_misses ok\n");
}

static void test_solid_overlap_pushes_out() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Zero direction = static overlap query, with the box centre 0.4m up: hull 1
	// wants 0.9m, so it is 0.5m deep in the expanded hull and must be pushed UP.
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.4, 0), vec(0, 0, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});

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
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.92, 0), vec(0, 0, 0));
	t->trace_solid(c, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, (T)0.04);

	assert(c.time == 0.0);
	assert(approx_v(c.normal, 0, 1, 0));
	assert(c.depth > 0.0 && c.depth < 0.04);
	printf("  solid_margin_finds_resting_contact ok\n");
}

static void test_solid_starting_stuck_defers_to_recovery() {
	auto t = load(make_floor_map());
	auto s = make_box_solid(0.4, 0.9, 0.4);
	// Mappers routinely sink a spawn a hair into the floor (ww_countryside puts
	// ten of them at z=351.9 where the hull surface is 352). A sweep from inside
	// solid must NOT fabricate an impact plane — Quake reports startsolid and no
	// normal, because a made-up normal shoves a stuck body in a garbage direction.
	hop::collision<T> swept;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.8975, 0), vec(0, -4, 0));  // 0.1 GoldSrc units low
	t->trace_solid(swept, s.get(), vec(0, 0, 0), hop::mat3<T>(), seg, T {});
	assert(swept.time == 1.0);

	// The zero-direction query is what resolves it: push straight up, by exactly
	// the 0.1 units it is buried.
	hop::collision<T> rec;
	hop::segment<T> still;
	still.set_start_dir(vec(0, 0.8975, 0), vec(0, 0, 0));
	t->trace_solid(rec, s.get(), vec(0, 0, 0), hop::mat3<T>(), still, T {});
	assert(rec.time == 0.0);
	assert(approx_v(rec.normal, 0, 1, 0));
	assert(approx(rec.depth / SCALE, 0.1, 0.01));  // in GoldSrc units
	printf("  solid_starting_stuck_defers_to_recovery ok\n");
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
		hop::collision<T> c;
		hop::segment<T> seg;
		seg.set_start_dir(vec(0, 2, 0), vec(0, -4, 0));
		t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);
		assert(c.time == 1.0);
	}
	// The same map viewed with sky blocking — how player movement sees it.
	{
		auto t = load(blob, hopbsp::blocking_bit(CONTENTS_SKY));
		hop::collision<T> c;
		hop::segment<T> seg;
		seg.set_start_dir(vec(0, 2, 0), vec(0, -4, 0));
		t->trace_segment(c, vec(0, 0, 0), hop::mat3<T>(), seg);
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
	test_solid_crouched_uses_short_hull();
	test_solid_impact_is_on_the_surface();
	test_solid_misses();
	test_solid_overlap_pushes_out();
	test_solid_margin_finds_resting_contact();
	test_solid_starting_stuck_defers_to_recovery();
	test_sky_is_passable_but_maskable();
	test_float_instantiation();
	printf("all bsp traceable tests passed\n");
	return 0;
}
