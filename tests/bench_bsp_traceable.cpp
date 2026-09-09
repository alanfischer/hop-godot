// Where the GoldSrc BSP trace spends its time.
//
// Written to answer one question before changing anything: a body RESTING on
// world geometry takes the zero-direction overlap path, and that path calls
// nearest_surface, which probes six axes with a full hull_sweep each. A swept
// body takes one descent. So the resting case should cost several times what
// flight costs, and that ratio is the budget any manifold work has to fit in.
//
// Each scenario is one trace_solid/trace_segment call, timed in isolation, at a
// few tree depths — a shallow fixture flatters the descent and hides how much of
// the cost is per-node rather than fixed overhead.
//
// Build & run:
//   cmake -S extern/hop-godot/tests -B <build> -DCMAKE_BUILD_TYPE=Release
//   cmake --build <build> --target bench_bsp_traceable
//   <build>/bench_bsp_traceable

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include <hop/hop.h>
#include <bench/bench.h>
#include "hop_bsp_traceable.h"

using T = double;
using V = hop::vec3<T>;
using namespace hop_bsp;

#include "bsp_fixtures.h"

// A floor plus `decoys` disjoint slabs stacked in the tree ABOVE it, chained
// through each other's `outside` child. A query near the floor therefore descends
// past every decoy first, which is how tree depth is dialled: 6 planes per brush,
// so `decoys` = 4 is a 30-node walk, about what a real map's balanced tree costs
// for a query deep in the world.
static std::vector<uint8_t> make_layered_map(int decoys, bool oblique = false) {
	const double fmins[3] = { -4096, -4096, -4096 }, fmaxs[3] = { 4096, 4096, 0 };
	BlobBuilder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) { m.mins[i] = (float)fmins[i]; m.maxs[i] = (float)fmaxs[i]; }
	m.maxs[2] = 4096.0f;

	int prev = add_box_brush(b, fmins, fmaxs, /*as_nodes=*/true, CONTENTS_SOLID, 1, oblique);
	for (int d = 0; d < decoys; ++d) {
		// Far overhead and out of the way: they are tree depth, not geometry the
		// query touches.
		const double z = 1024.0 + d * 128.0;
		const double dmins[3] = { -4096, -4096, z }, dmaxs[3] = { 4096, 4096, z + 32.0 };
		prev = add_box_brush(b, dmins, dmaxs, /*as_nodes=*/true, CONTENTS_SOLID, prev, oblique);
	}
	m.headnode[0] = prev;

	// Hulls 1..3: the floor alone, expanded. Nothing here traces them (every mover
	// below is under 32 units wide, so hull_for_size sends it to hull 0), but a
	// model with no clipnodes at all is not a shape the parser expects.
	for (int h = 1; h < 4; ++h) {
		double emins[3], emaxs[3];
		for (int i = 0; i < 3; ++i) {
			emins[i] = fmins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			emaxs[i] = fmaxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		m.headnode[h] = add_box_brush(b, emins, emaxs, /*as_nodes=*/false);
	}
	b.models.push_back(m);
	return b.build();
}


// --- scenarios -------------------------------------------------------------

// A gib: 2x2x20cm, the shape whose contact patch the trace cannot currently
// resolve. Half-extents in Godot metres.
static std::shared_ptr<hop::solid<T>> make_gib() { return make_box_solid(0.10, 0.01, 0.01); }



static void run_depth(int decoys, bool oblique) {
	char suffix[64];
	snprintf(suffix, sizeof(suffix), "%d brush%s%s", decoys + 1, decoys ? "es" : "",
	         oblique ? ", oblique" : ", axial");
	printf("\n[tree: %s — %d nodes]\n", suffix, (decoys + 1) * 6);

	auto blob = make_layered_map(decoys, oblique);
	auto t = load(blob);
	auto gib = make_gib();
	const hop::mat3<T> ident;

	// Floor top is GoldSrc z=0 → Godot y=0. A gib at rest sits its half-height up.
	const T rest_y = (T)0.01;
	const V resting = vec(0, rest_y, 0);
	const V airborne = vec(0, 5.0, 0);
	const V fall = vec(0, -0.5, 0);   // one tick of gravity, roughly
	const V slide = vec(0.1, 0, 0);   // travelling along the floor it rests on

	char name[128];
	hop::collision<T> c;
	hop::segment<T> seg;

	// The unit of cost: one point descent, no box, no expansion.
	snprintf(name, sizeof(name), "trace_segment (point ray, 1 descent)");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(airborne, vec(0, -10, 0));
		t->trace_segment(c, V {}, ident, seg);
	});

	// Swept, clear air: descends and finds nothing. The cheap common case.
	snprintf(name, sizeof(name), "trace_solid swept, no hit");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(airborne, fall);
		t->trace_solid(c, gib.get(), V {}, ident, seg, T {});
	});

	// Swept into the floor: descends, splits, stops, walks the witness back.
	snprintf(name, sizeof(name), "trace_solid swept, lands on floor");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(vec(0, 0.3, 0), fall);
		t->trace_solid(c, gib.get(), V {}, ident, seg, T {});
	});

	// Resting and sliding: starts touching, so it takes the sweep path but with a
	// start that is already on the expanded surface.
	snprintf(name, sizeof(name), "trace_solid swept, sliding on floor");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(resting, slide);
		t->trace_solid(c, gib.get(), V {}, ident, seg, T {});
	});

	// THE ONE THAT MATTERS: zero direction, at rest on the floor. This is the
	// static-overlap query, and the path that runs nearest_surface's six probes.
	snprintf(name, sizeof(name), "trace_solid ZERO-DIR, resting on floor");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(resting, V {});
		t->trace_solid(c, gib.get(), V {}, ident, seg, T {});
	});

	// Zero direction in clear air: the same path, but nothing near enough to probe
	// for. Isolates how much of the above is the six probes rather than the entry.
	snprintf(name, sizeof(name), "trace_solid ZERO-DIR, clear air");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(airborne, V {});
		t->trace_solid(c, gib.get(), V {}, ident, seg, T {});
	});

	// --- oriented expansion -----------------------------------------------
	// The axial fast path is preserved by construction (an axial plane's support is
	// precomputed per trace), so the delta is the oblique planes and the setup only.
	auto plate = make_spinning_box(0.2, 0.05, 0.2);
	auto turned = make_spinning_box(0.2, 0.05, 0.2);
	{
		hop::mat3<T> m;
		hop::set_mat3_from_axis_angle(m, vec(0, 0, 1), (T)0.7853981634);
		turned->set_orientation(m);
	}

	snprintf(name, sizeof(name), "  rotated box: swept (oriented expansion)");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(vec(0, 0.9, 0), vec(0, -1.2, 0));
		t->trace_solid(c, turned.get(), V {}, ident, seg, T {});
	});
	snprintf(name, sizeof(name), "  axis-aligned box: same sweep (AABB expansion)");
	hop::bench::go(name, 200000, [&] {
		c.reset(); c.time = hop::scalar_traits<T>::one();
		seg.set_start_dir(vec(0, 0.9, 0), vec(0, -1.2, 0));
		t->trace_solid(c, plate.get(), V {}, ident, seg, T {});
	});

}

// --- primitive micro-costs -------------------------------------------------
//
// The scenarios above are whole trace_solid calls. These price the two tree walks
// a manifold would trade between: hull_sweep (what nearest_surface spends six of)
// against hull_point_contents (what a face-clip keep-test would spend four of).
// Same tree, same start point, so the ratio is the thing to read.
static void run_primitives(int decoys) {
	printf("\n[primitives: %d brushes — %d nodes]\n", decoys + 1, (decoys + 1) * 6);
	auto blob = make_layered_map(decoys, false);
	BlobView view;
	bool ok = parse_blob(blob.data(), blob.size(), view);
	assert(ok); (void)ok;

	hopbsp::hull h;
	h.planes = view.planes;
	h.nodes = view.nodes;
	h.leafs = view.leafs;
	h.leaf_count = view.leaf_count;
	h.node_count = view.node_count;
	h.root = view.models[0].headnode[0];

	// A gib at rest: GoldSrc z = its half-height above the floor top at 0.
	const double at[3] = { 0, 0, 0.4 };
	const double down[3] = { 0, 0, -1 };
	const double probe_end[3] = { 0, 0, 0.4 - 4.0 };

	hop::bench::go("hull_point_contents (point descent)", 500000, [&] {
		volatile int c = hopbsp::hull_point_contents(h, h.root, at);
		(void)c;
	});
	hop::bench::go("hull_point_contents_biased (stuck band)", 500000, [&] {
		volatile int c = hopbsp::hull_point_contents_biased(h, h.root, at, -hopbsp::STUCK_SLOP);
		(void)c;
	});
	hop::bench::go("hull_sweep (one nearest_surface probe)", 500000, [&] {
		auto t = hopbsp::hull_sweep(h, at, probe_end, hopbsp::BLOCK_SOLID);
		volatile double f = t.fraction; (void)f;
	});
	hop::bench::go("hull_inside_distance (one push_out probe)", 500000, [&] {
		volatile double d = hopbsp::hull_inside_distance(h, h.root, at, down, 4.0,
		                                                 hopbsp::BLOCK_SOLID);
		(void)d;
	});
}

int main() {
	printf("hop BSP traceable — baseline cost by scenario and tree depth\n");
	printf("gib mover: 20x2x2cm box, hull 0 (hull_for_size sends anything <32 units there)\n");

	for (int decoys : { 0, 4, 12 }) run_depth(decoys, /*oblique=*/false);
	run_depth(4, /*oblique=*/true);
	run_primitives(4);
	run_primitives(12);

	printf("\nRead: ZERO-DIR/resting against swept is the manifold budget. Any\n");
	printf("manifold that costs less than the six probes it replaces is free.\n");
	return 0;
}
