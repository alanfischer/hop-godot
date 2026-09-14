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
#include <algorithm>
#include <memory>
#include <vector>

#include <hop/hop.h>
#include "hop_bsp_traceable.h"

using T = double;
using V = hop::vec3<T>;
using namespace hop_bsp;

#include "bsp_fixtures.h"

// Sweep `s` from `from` along `motion` and return what it hit. Every solid test
// below is one of these plus its assertions; a zero `motion` is the static
// overlap query.
static hop::collision<T> sweep(HopBspTraceable<T> &t, std::shared_ptr<hop::solid<T>> s,
                               V from, V motion, T margin = T {}, V at = V {},
                               hop::mat3<T> rot = hop::mat3<T>()) {
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(from, motion);
	t.trace_solid(c, s.get(), at, rot, seg, margin);
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

// A brush entity authored lying down and rotated upright to close: ww_monoliths'
// spawn gates. Rays already cover orientation, but a ray is a point mover with no hull
// offset — so the one thing rotation can break, they cannot see. Applied unrotated,
// a standing player's 36-unit "raise the point to my centre" went sideways THROUGH the
// plate: 72 units off one face, eye inside the other.
static void test_feet_origin_mover_against_a_tipped_plate() {
	// 16 units thin on GoldSrc z — a floor slab, until it is stood on its edge.
	const double mins[3] = { -512, -512, -8 };
	const double maxs[3] = { 512, 512, 8 };
	auto t = load(make_box_map(mins, maxs));

	// +90° about Godot z sends the plate's thin axis (Godot y) to Godot x: a wall
	// whose faces sit at x = ±0.2.
	hop::mat3<T> rot;
	hop::set_mat3_from_axis_angle(rot, vec(0, 0, 1), (T)(M_PI / 2));

	// The tree is expanded in the MODEL's frame, so the thin axis carries hull 1's
	// 36-unit half-HEIGHT rather than its 16-unit half-width: 0.9 m of standoff off
	// each face. Fat — and fat in GoldSrc too, for the same reason. What this asserts
	// is that both faces agree, which is what the offset decides.
	auto standing = make_feet_box_solid(0.4, 1.8, 0.4);
	hop::collision<T> plus = sweep(*t, standing, vec(3, 0, 0), vec(-6, 0, 0), T {}, V {}, rot);
	assert(plus.time < 1.0);
	assert(approx(plus.point.x, 1.1, 0.02));
	assert(approx_v(plus.normal, 1, 0, 0));

	hop::collision<T> minus = sweep(*t, standing, vec(-3, 0, 0), vec(6, 0, 0), T {}, V {}, rot);
	assert(minus.time < 1.0);
	assert(approx(minus.point.x, -1.1, 0.02));
	assert(approx_v(minus.normal, -1, 0, 0));

	// Crouched picks hull 3, whose offset is 18 rather than 36 — the same test with a
	// different number, and the pair is what pins the offset to the mover's own box.
	auto crouched = make_feet_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> cplus = sweep(*t, crouched, vec(3, 0, 0), vec(-6, 0, 0), T {}, V {}, rot);
	hop::collision<T> cminus = sweep(*t, crouched, vec(-3, 0, 0), vec(6, 0, 0), T {}, V {}, rot);
	assert(approx(cplus.point.x, 0.65, 0.02));
	assert(approx(cminus.point.x, -0.65, 0.02));

	// A centre-origin mover has no offset at all, so it was symmetric even while this
	// was broken: it holds the fixture itself honest.
	auto centred = make_box_solid(0.4, 0.9, 0.4);
	hop::collision<T> nplus = sweep(*t, centred, vec(3, 0, 0), vec(-6, 0, 0), T {}, V {}, rot);
	hop::collision<T> nminus = sweep(*t, centred, vec(-3, 0, 0), vec(6, 0, 0), T {}, V {}, rot);
	assert(approx(nplus.point.x, 1.1, 0.02));
	assert(approx(nminus.point.x, -1.1, 0.02));
	printf("  feet_origin_mover_against_a_tipped_plate ok\n");
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

// --- hull-0 movers with a box of their own --------------------------------

// A hull-0 mover is traced as its box CENTRE with the tree expanded by its own
// half-extents. WizardWars' corpse is the awkward shape that exposed whether that
// expansion is real: a capsule r=0.3 h=1.0 whose collider sits +0.5 above the body
// origin, so in GoldSrc units a 24x24x40 box with the origin on its bottom face.
static std::shared_ptr<hop::solid<T>> make_corpse_solid() {
	hop::aa_box<T> b;
	b.mins = vec(-0.3, 0, -0.3);
	b.maxs = vec(0.3, 1.0, 0.3);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(b));
	return s;
}

// It must come to rest ON the floor. The expansion has to be applied while the tree
// is DESCENDED, not only at the crosspoint: a walk that reads both segment ends as
// "in front of the floor plane" never splits, so the mover was waved through until
// its centre was genuinely inside the brush — 0.5m of floor, for this body.
static void test_hull0_box_lands_on_the_floor() {
	auto t = load(make_floor_map());
	auto s = make_corpse_solid();
	hop::collision<T> c = sweep(*t, s, vec(0, 1.5, 0), vec(0, -2.0, 0));
	assert(c.time < 1.0 && "the box must be stopped by the floor");
	const double rest = 1.5 - 2.0 * (double)c.time;
	assert(approx(rest, 0.0, 0.02) && "it rests with its bottom face on the floor");
	assert(approx_v(c.normal, 0, 1, 0));
	printf("  hull0_box_lands_on_the_floor ok (rest y=%.4f)\n", rest);
}

// Sideways, against a wall: it stops its own half-WIDTH short (0.3), not its
// half-height and not centred on the surface. One scalar radius cannot do both.
static void test_hull0_box_stops_at_its_own_width() {
	// A wall block occupying x >= 4 (GoldSrc x is -godot x, so this is the -x side).
	const double mins[3] = { -512, -512, -512 };
	const double maxs[3] = { -160, 512, 512 };  // godot x >= 4.0
	auto t = load(make_box_map(mins, maxs));
	auto s = make_corpse_solid();
	hop::collision<T> c = sweep(*t, s, vec(0, 1.0, 0), vec(5.0, 0, 0));
	assert(c.time < 1.0);
	const double stop = 5.0 * (double)c.time;
	assert(approx(stop, 4.0 - 0.3, 0.02) && "stops half its width short of the wall");
	printf("  hull0_box_stops_at_its_own_width ok (stop x=%.4f)\n", stop);
}

// The bug this file gained a section for: a corpse that bounced on the ground,
// forever. Sweep and overlap are two answers to the same question, and they used
// different geometry — the sweep let the body sink half its height into the floor,
// the overlap probe shoved it back out to the surface, and the next frame it fell
// again. Run the loop the physics server runs and require it to settle.
static void test_hull0_box_settles_instead_of_bouncing() {
	auto t = load(make_floor_map());
	auto s = make_corpse_solid();
	const double dt = 1.0 / 60.0, gravity = 20.0;
	double y = 1.0, vy = 0.0, lo = 1e30, hi = -1e30;

	for (int step = 0; step < 240; ++step) {
		vy -= gravity * dt;
		hop::collision<T> c = sweep(*t, s, vec(0, (T)y, 0), vec(0, (T)(vy * dt), 0));
		if (c.time < 1.0) { y += vy * dt * (double)c.time; vy = 0.0; }
		else { y += vy * dt; }
		// Depenetration, as _body_test_motion's recovery does it.
		hop::collision<T> o = sweep(*t, s, vec(0, (T)y, 0), vec(0, 0, 0));
		if (o.time == 0.0 && o.depth > 0.0) y += (double)o.normal.y * (double)o.depth;
		if (step >= 120) { lo = std::min(lo, y); hi = std::max(hi, y); }
	}
	printf("  hull0_box_settles_instead_of_bouncing ok (y=%.4f, band=%.4f)\n", y, hi - lo);
	assert(hi - lo < 0.01 && "a settled body must not oscillate");
	assert(approx(y, 0.0, 0.02) && "and it settles on the floor, not inside it");
}

// hop's speculative solver reads a time-0 contact's `depth` as how far into the
// margin shell the body sits: separation = margin - depth. A sweep that yields no
// motion has to answer in those terms, or the solver is told the body has a whole
// margin of room and lets it approach that far EVERY tick — a resting body then
// creeps into the floor at margin-per-tick forever.
static void test_touching_sweep_reports_the_margin_gap() {
	auto t = load(make_floor_map());
	const T margin = 0.007;

	auto corpse = make_corpse_solid();  // hull 0, resting on the floor
	hop::collision<T> c = sweep(*t, corpse, vec(0, 0, 0), vec(0, -0.01, 0), margin);
	assert(c.time == 0.0 && "a body on the floor cannot move down");
	assert(approx(c.depth, margin, 0.002) && "gap 0 must read back as separation 0");

	// A sized-hull mover (the player) resting on the floor: same contract.
	auto player = make_feet_box_solid(0.4, 1.8, 0.4);
	hop::collision<T> p = sweep(*t, player, vec(0, 0, 0), vec(0, -0.01, 0), margin);
	assert(p.time == 0.0);
	assert(approx(p.depth, margin, 0.002));

	// With no margin (the public _body_test_motion query) depth stays exact: a body
	// that is merely touching is not inside anything.
	hop::collision<T> bare = sweep(*t, corpse, vec(0, 0, 0), vec(0, -0.01, 0));
	assert(bare.time == 0.0 && bare.depth == 0.0);
	printf("  touching_sweep_reports_the_margin_gap ok\n");
}


// --- oriented hull expansion ----------------------------------------------
//
// The hull is expanded by the mover's reach along each plane normal. Taking that
// reach from the LOCAL AABB means a box sweeps the same volume however it is turned,
// so a plate stood on its edge stops as if it were still lying flat and sinks in to
// its own half-thickness. These check it stops where the ROTATED box actually reaches.

static hop::mat3<T> rot_about(V axis, double degrees) {
	hop::mat3<T> m;
	hop::set_mat3_from_axis_angle(m, axis, (T)(degrees * 3.14159265358979323846 / 180.0));
	return m;
}

static void test_rotated_box_stops_at_its_rotated_reach() {
	auto t = load(make_floor_map());   // floor top at Godot y = 0
	// A plate: wide and thin. Turned 45 degrees about Z its downward reach becomes
	// (hx + hy)/sqrt(2) — much more than the hy it reaches lying flat.
	// A body that can TURN, i.e. one with inertia: rotates_dynamically() is the gate on
	// the oriented path. A body that cannot turn has an attitude that never changes, so
	// tracing it as its axis-aligned bound costs nothing and keeps it off hull 0, which
	// an oriented mover is forced onto — that is what keeps a yawing player on the
	// sized hull it is built for.
	const double hx = 0.2, hy = 0.05;
	auto box = make_spinning_box((T)hx, (T)hy, (T)0.2);
	box->set_orientation(rot_about(vec(0, 0, 1), 45.0));

	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.9, 0), vec(0, -1.2, 0));
	c.reset();
	t->trace_solid(c, box.get(), V {}, hop::mat3<T>(), seg, T {});
	assert(c.time < (T)1);

	const double stop_y = 0.9 - 1.2 * (double)c.time;
	const double flat_reach = hy;
	const double turned_reach = (hx + hy) * 0.70710678;
	assert(std::fabs(stop_y - turned_reach) < 0.01 &&
	       "a turned box must stop at the reach it actually has");
	assert(stop_y > flat_reach + 0.05 && "and that is well clear of its flat reach");
	printf("  rotated_box_stops_at_its_rotated_reach ok (y=%.4f, flat would be %.4f)\n",
	       stop_y, flat_reach);
}

static void test_unrotated_box_is_unchanged_by_the_oriented_path() {
	// Identity orientation must take the pre-rotation path exactly — no drift, no
	// "nearly the same". This is the guard on every body in the game today.
	auto t = load(make_floor_map());
	auto spun = make_spinning_box(0.2, 0.05, 0.2);  // eligible for the oriented path
	spun->set_orientation(hop::mat3<T>());          // explicitly identity
	auto plain = make_box_solid(0.2, 0.05, 0.2);    // never touched

	hop::collision<T> a, b;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.4, 0), vec(0, -0.5, 0));
	a.reset(); t->trace_solid(a, spun.get(), V {}, hop::mat3<T>(), seg, T {});
	b.reset(); t->trace_solid(b, plain.get(), V {}, hop::mat3<T>(), seg, T {});
	assert(a.time == b.time && "identity orientation must be bit-identical");
	assert(a.impact.y == b.impact.y && a.normal.y == b.normal.y);
	printf("  unrotated_box_is_unchanged_by_the_oriented_path ok\n");
}

static void test_rotated_box_reach_is_symmetric_about_the_turn() {
	// +45 and -45 about the same axis reach equally far down: the support function is
	// an absolute value, so sign must not matter. Catches a dropped fabs.
	auto t = load(make_floor_map());
	double stops[2];
	for (int k = 0; k < 2; ++k) {
		auto box = make_spinning_box(0.2, 0.05, 0.2);
		box->set_orientation(rot_about(vec(0, 0, 1), k ? -45.0 : 45.0));
		hop::collision<T> c;
		hop::segment<T> seg;
		seg.set_start_dir(vec(0, 0.9, 0), vec(0, -1.2, 0));
		c.reset();
		t->trace_solid(c, box.get(), V {}, hop::mat3<T>(), seg, T {});
		stops[k] = 0.9 - 1.2 * (double)c.time;
	}
	assert(std::fabs(stops[0] - stops[1]) < 1e-6 && "turn direction must not change reach");
	printf("  rotated_box_reach_is_symmetric_about_the_turn ok\n");
}

// --- end to end ------------------------------------------------------------
//
// Everything above calls trace_solid directly, which verifies the trace and nothing
// else. These run the simulator, wired as hop-godot wires it: the traceable is a shape
// on a static solid, reached through collide.h's test_solid — not through the manager,
// whose trace_solid is a no-op here.

static std::shared_ptr<hop::simulator<T>> world_with_floor(
    std::shared_ptr<hop::solid<T>> &world_out, std::vector<uint8_t> &blob) {
	auto sim = std::make_shared<hop::simulator<T>>();
	sim->set_gravity(vec(0, -20, 0));
	// hop's own default is sweep_slide, which hop-godot gives KINEMATIC bodies only;
	// anything dynamic resolves speculatively.
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	auto tr = std::make_unique<HopBspTraceable<T>>();
	assert(tr->build(blob.data(), blob.size(), 0, (T)SCALE, hopbsp::BLOCK_SOLID));
	auto world = std::make_shared<hop::solid<T>>();
	world->set_infinite_mass();
	world->set_coefficient_of_gravity(T {});
	world->add_shape(std::make_shared<hop::shape<T>>(std::move(tr)));
	sim->add_solid(world);
	world_out = world;
	return sim;
}

// A gib: an 18cm rod, 2cm across, free to turn. `axis` is which local axis the spine
// runs along — Godot authors capsules along Y, this file's own fixtures along X, and the
// trace must not care which.
static std::shared_ptr<hop::solid<T>> make_rod_on(int axis, T half_len, T radius) {
	V from = vec(0, 0, 0), dir = vec(0, 0, 0);
	if (axis == 0) { from = vec(-half_len, 0, 0); dir = vec(2 * half_len, 0, 0); }
	else if (axis == 1) { from = vec(0, -half_len, 0); dir = vec(0, 2 * half_len, 0); }
	else { from = vec(0, 0, -half_len); dir = vec(0, 0, 2 * half_len); }
	hop::capsule<T> c(from, dir, radius);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(c));
	s->set_mass((T)1);
	// About its OWN spine a rod barely resists turning; across it, much more. Handing
	// every rod the X-spine tensor gives a Y- or Z-authored one its small moment about
	// the wrong axis, so it tumbles in a way no rod of that shape would.
	V I = vec((T)0.004, (T)0.004, (T)0.004);
	if (axis == 0) I.x = (T)0.002;
	else if (axis == 1) I.y = (T)0.002;
	else I.z = (T)0.002;
	s->set_inertia(I);
	return s;
}

static std::shared_ptr<hop::solid<T>> make_rod(T half_len, T radius) {
	return make_rod_on(0, half_len, radius);
}

static double settle(std::shared_ptr<hop::simulator<T>> sim, std::shared_ptr<hop::solid<T>> body) {
	for (int i = 0; i < 240; ++i) sim->update((T)(1.0 / 60.0));
	V ax;
	hop::mul(ax, body->get_orientation(), vec(1, 0, 0));
	return std::fabs((double)ax.y);
}

// THE behaviour this change exists for. Before it, every mover was traced as its local
// AABB and contacted under its own centre, so the lever arm was vertical whatever the
// attitude and gravity had nothing to turn: a rod stood at 60 degrees for four seconds
// with w == 0.
static void test_a_tilted_rod_falls_over() {
	auto blob = make_floor_map();
	std::shared_ptr<hop::solid<T>> world;
	auto sim = world_with_floor(world, blob);
	auto rod = make_rod(0.09, 0.01);
	hop::mat3<T> m;
	hop::set_mat3_from_axis_angle(m, vec(0, 0, 1), (T)(60.0 * 3.14159265358979323846 / 180.0));
	rod->set_orientation(m);
	rod->set_position(vec(0, 0.35, 0));
	sim->add_solid(rod);

	const double tilt = settle(sim, rod);
	assert(tilt < 0.3 && "a rod dropped on its end falls over");
	const double y = (double)rod->get_position().y;
	assert(y > 0.0 && y < 0.03 && "and comes to rest on its own radius");
	printf("  a_tilted_rod_falls_over ok (|axis.y|=%.3f, y=%.4f)\n", tilt, y);
}

// And one already lying down STAYS still. Its contact runs along the spine there, so a
// support point taken as a bare sign(n.axis) picks an arbitrary END and hands a level
// rod a lever arm it does not have — it spins up and sinks, which is the box-corner
// failure in capsule form. Scaling the spine support by tilt is what keeps this at rest.
static void test_a_level_rod_lies_still() {
	auto blob = make_floor_map();
	std::shared_ptr<hop::solid<T>> world;
	auto sim = world_with_floor(world, blob);
	auto rod = make_rod(0.09, 0.01);
	rod->set_position(vec(0, 0.05, 0));
	sim->add_solid(rod);

	const double tilt = settle(sim, rod);
	assert(tilt < 0.05 && "a level rod stays level");
	const double w = (double)hop::length(rod->get_angular_velocity());
	assert(w < 0.5 && "and does not spin itself up");
	const double y = (double)rod->get_position().y;
	assert(y > 0.004 && y < 0.02 && "resting on its radius, not sunk into the floor");
	printf("  a_level_rod_lies_still ok (w=%.3f, y=%.4f)\n", w, y);
	// What this does NOT yet cover, so nobody reads it as more than it is. A rod that
	// arrives ALREADY level settles and sleeps; one that has to topple from near
	// vertical does not. It cartwheels, because one contact point cannot hold a rod
	// down along its length — a lying rod touches along a LINE, and only a line can
	// resist turning about the surface normal or about the spine. Same limit as a flat
	// box being held at a single point in the middle of its face; a capsule feels it
	// sooner because it has no face to be held by. Wants trace_solid to report a
	// contact SET, which it cannot yet do.
}

// The spine may be authored along ANY local axis. Godot builds capsules along Y and
// this file's fixtures along X, and reading the spine out of a fixed local slot gives
// one of them a zero-length spine — the contact then lands under the centre, the lever
// arm vanishes, and a gib in the game stands exactly as it had before while an
// X-authored test capsule passes.
//
// Asked of the TRACE rather than of a settled simulation, deliberately. Three rods with
// the same geometry pointed the same way in the world are the same object, so they must
// stop at the same height, exactly — a claim with one answer. Dropping them and looking
// at where they end up cannot make that claim: a rod toppling from near-vertical does
// not settle here at all (see the note on test_a_level_rod_lies_still), so the run would
// be reporting a tumble rather than the thing under test.
static void test_a_rod_reaches_the_same_whichever_axis_its_spine_runs_along() {
	auto t = load(make_floor_map());
	const double half_len = 0.09, radius = 0.01;
	// One world direction for the spine, 60 degrees off vertical, so the reach is a
	// genuine mix of length and roundness rather than either one alone.
	const double ang = 60.0 * 3.14159265358979323846 / 180.0;
	const V want = vec((T)std::sin(ang), (T)std::cos(ang), (T)0);
	const double expect = half_len * std::cos(ang) + radius;

	double stops[3];
	for (int axis = 0; axis < 3; ++axis) {
		auto rod = make_rod_on(axis, (T)half_len, (T)radius);
		// Turn the rod's own spine axis onto `want`: rotate about their cross product
		// by the angle between them.
		V unit = vec(axis == 0 ? 1 : 0, axis == 1 ? 1 : 0, axis == 2 ? 1 : 0);
		V cross = vec(unit.y * want.z - unit.z * want.y,
		              unit.z * want.x - unit.x * want.z,
		              unit.x * want.y - unit.y * want.x);
		hop::mat3<T> m;
		if (hop::length(cross) < (T)1e-9) {
			m = hop::mat3<T>();
		} else {
			hop::mul(cross, (T)(1.0 / hop::length(cross)));
			const double dot = (double)(unit.x * want.x + unit.y * want.y + unit.z * want.z);
			hop::set_mat3_from_axis_angle(m, cross, (T)std::acos(dot < -1 ? -1 : dot > 1 ? 1 : dot));
		}
		rod->set_orientation(m);

		hop::collision<T> c;
		hop::segment<T> seg;
		seg.set_start_dir(vec(0, 0.6, 0), vec(0, -0.8, 0));
		c.reset();
		t->trace_solid(c, rod.get(), V {}, hop::mat3<T>(), seg, T {});
		assert(c.time < (T)1 && "a tilted rod must reach the floor");
		stops[axis] = 0.6 - 0.8 * (double)c.time;
		assert(std::fabs(stops[axis] - expect) < 0.004 &&
		       "and stop at |n.spine| * half + radius, whatever axis it was authored on");
	}
	assert(std::fabs(stops[0] - stops[1]) < 1e-6 && std::fabs(stops[0] - stops[2]) < 1e-6 &&
	       "the same rod pointed the same way is the same rod");
	printf("  a_rod_reaches_the_same_whichever_axis_its_spine_runs_along ok "
	       "(%.4f / %.4f / %.4f, want %.4f)\n", stops[0], stops[1], stops[2], expect);
}

// The expansion itself: a capsule is pushed out by |n.axis| * half + radius, so lying
// along X its downward reach is the radius alone, not half its length.
static void test_a_capsule_stops_at_its_own_reach() {
	auto t = load(make_floor_map());
	auto rod = make_rod(0.09, 0.01);
	hop::collision<T> c;
	hop::segment<T> seg;
	seg.set_start_dir(vec(0, 0.6, 0), vec(0, -0.8, 0));
	c.reset();
	t->trace_solid(c, rod.get(), V {}, hop::mat3<T>(), seg, T {});
	assert(c.time < (T)1);
	const double stop_y = 0.6 - 0.8 * (double)c.time;
	assert(std::fabs(stop_y - 0.01) < 0.005 && "a level capsule stops on its radius");
	printf("  a_capsule_stops_at_its_own_reach ok (y=%.4f)\n", stop_y);
}

// --- contact manifolds ------------------------------------------------------
//
// The reason the phase touches the BSP path at all: a corpse lies on LEVEL GEOMETRY,
// and level geometry is a traceable shape on a solid. Fixing solid-vs-solid alone would
// not move the game a millimetre.
//
// A traceable has no face polygon to clip against, so each corner of the mover's
// contacting face is probed straight down the contact normal onto the real hull. That
// is what makes the ledge case below work: a corner over thin air finds nothing and is
// dropped, where clipping to an infinite contact plane would hold the box up on nothing.

static void test_a_tilted_box_on_a_bsp_floor_gets_a_patch_and_sleeps() {
	auto blob = make_floor_map();
	std::shared_ptr<hop::solid<T>> world;
	auto sim = world_with_floor(world, blob);
	auto box = make_spinning_box(0.08, 0.08, 0.08);
	hop::mat3<T> m;
	hop::set_mat3_from_axis_angle(m, vec(0.5774, 0.5774, 0.5774), (T)0.05);
	box->set_orientation(m);
	box->set_position(vec(0, 0.5, 0));
	sim->add_solid(box);

	int slept_at = -1;
	for (int i = 0; i < 400; ++i) {
		sim->update((T)(1.0 / 60.0));
		if (slept_at < 0 && !box->active())
			slept_at = i;
	}
	int points = 0;
	for (int k = 0; k < box->get_touch_count(); ++k)
		points += box->get_touch(k).point_count;
	if (hop::max_manifold_points > 1)
		assert(points == 4 && "four corners of the box's bottom face are on the floor");
	assert(slept_at >= 0 && slept_at < 300 && "and it comes to rest");
	assert(hop::length(box->get_angular_velocity()) == T {} && "dead still, not frozen mid-turn");
	printf("  a_tilted_box_on_a_bsp_floor_gets_a_patch_and_sleeps ok (%d points, asleep at %d)\n",
	       points, slept_at);
}

// Hang the box off the edge of the floor brush and the corners over thin air must NOT be
// reported. This is the whole reason each corner is PROBED against the real hull rather
// than clipped to the contact plane: an infinite plane would hold an overhanging body up
// on nothing and it would never tip.
static void test_corners_over_a_ledge_are_not_reported() {
	auto blob = make_floor_map();
	std::shared_ptr<hop::solid<T>> world;
	auto sim = world_with_floor(world, blob);
	// make_floor_map's slab spans +-512 GoldSrc units about the origin, which at
	// SCALE (0.025) is +-12.8 m. Sit the box astride the +x edge, half on and half off.
	const T edge = (T)(512.0 * SCALE);
	auto box = make_spinning_box(0.08, 0.08, 0.08);
	box->set_position(vec(edge - (T)0.04, (T)0.081, 0));
	sim->add_solid(box);
	for (int i = 0; i < 20; ++i)
		sim->update((T)(1.0 / 60.0));
	int points = 0;
	for (int k = 0; k < box->get_touch_count(); ++k)
		points += box->get_touch(k).point_count;
	assert(points >= 1 && "the supported half is still in contact");
	if (hop::max_manifold_points > 1)
		assert(points <= 2 && "but the corners hanging over the edge are not");
	printf("  test_corners_over_a_ledge_are_not_reported ok (%d points at the edge)\n", points);
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
	test_feet_origin_mover_against_a_tipped_plate();
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
	test_hull0_box_lands_on_the_floor();
	test_hull0_box_stops_at_its_own_width();
	test_hull0_box_settles_instead_of_bouncing();
	test_touching_sweep_reports_the_margin_gap();
	test_sky_is_passable_but_maskable();
	test_float_instantiation();
	test_rotated_box_stops_at_its_rotated_reach();
	test_unrotated_box_is_unchanged_by_the_oriented_path();
	test_rotated_box_reach_is_symmetric_about_the_turn();
	test_a_capsule_stops_at_its_own_reach();
	test_a_tilted_rod_falls_over();
	test_a_level_rod_lies_still();
	test_a_rod_reaches_the_same_whichever_axis_its_spine_runs_along();
	test_a_tilted_box_on_a_bsp_floor_gets_a_patch_and_sleeps();
	test_corners_over_a_ledge_are_not_reported();
	printf("all bsp traceable tests passed\n");
	return 0;
}
