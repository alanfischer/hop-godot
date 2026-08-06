// Regression tests for broad-phase result-buffer capacity.
//
// find_solids_in_aa_box fills at most max_solids and CLAMPS its return value to it
// (simulator.h), so an undersized buffer truncates with no signal to the caller —
// count == max_solids is indistinguishable from "exactly that many overlapped".
//
// That would be a mere capacity nuisance if the dropped hits were arbitrary, but
// they aren't: bvh_manager::find_solids_in_aa_box drains the STATIC bucket before
// it looks at the dynamic one. So the hits a short buffer discards are precisely
// the moving bodies a caller cares about. An Area3D sitting on enough static
// geometry would stop reporting players entirely — silently, and only on maps
// dense enough to cross the threshold.
//
// hop-godot therefore sizes every broad-phase buffer to the live solid count
// (HopSpaceData::size_for_bodies / size_for_areas), mirroring what hop's own
// simulator does internally with spacial_collection_. These tests pin the hazard
// so nobody reintroduces a fixed-size array.

#include <cassert>
#include <cstdio>
#include <memory>
#include <vector>

#include <hop/hop.h>

using namespace hop;

using T = float;
using tr = scalar_traits<T>;

namespace {

// Layers used by the mask tests: the statics broadcast on one channel, the body on
// another, so a mask for the body's channel must reject every static.
constexpr int layer_world = 1 << 0;
constexpr int layer_body = 1 << 1;

// A unit-box solid at the origin, so everything built here mutually overlaps and
// the broad-phase must report all of it.
std::shared_ptr<solid<T>> make_solid_at_origin(bool is_static, int scope) {
	auto s = std::make_shared<solid<T>>();
	if (is_static)
		s->set_infinite_mass();
	else
		s->set_mass(tr::one());
	s->set_collision_scope(scope);
	s->set_position(vec3<T>{});
	s->add_shape(std::make_shared<shape<T>>(aa_box<T>(tr::one())));
	return s;
}

aa_box<T> query_box() {
	return aa_box<T>(vec3<T>(-tr::two(), -tr::two(), -tr::two()),
	                 vec3<T>(tr::two(), tr::two(), tr::two()));
}

// The dynamic body is added LAST so it is the one at risk: statics are scanned first.
struct Scene {
	std::shared_ptr<simulator<T>> sim = std::make_shared<simulator<T>>();
	bvh_manager<T> mgr;
	std::vector<std::shared_ptr<solid<T>>> statics;
	std::shared_ptr<solid<T>> dynamic;

	explicit Scene(int static_count) {
		sim->set_gravity(vec3<T>{});
		sim->set_manager(&mgr);
		for (int i = 0; i < static_count; i++) {
			auto s = make_solid_at_origin(true, layer_world);
			sim->add_solid(s);
			mgr.add_solid(s.get(), true);
			statics.push_back(s);
		}
		dynamic = make_solid_at_origin(false, layer_body);
		sim->add_solid(dynamic);
		mgr.add_solid(dynamic.get(), false);
	}

	// bits == -1 runs the unfiltered query.
	bool query_finds_dynamic(int buffer_size, int *out_count = nullptr, int bits = -1) {
		std::vector<solid<T> *> found(buffer_size);
		int count = sim->find_solids_in_aa_box(query_box(), found.data(), buffer_size, bits);
		if (out_count) *out_count = count;
		for (int i = 0; i < count; i++)
			if (found[i] == dynamic.get()) return true;
		return false;
	}
};

// The hazard itself: with more statics than slots, the dynamic body is dropped and
// the caller cannot tell. This is what a fixed-size array does on a dense map.
void test_undersized_buffer_hides_dynamic_body() {
	Scene scene(100);

	int count = 0;
	const bool found = scene.query_finds_dynamic(64, &count);
	assert(count == 64);   // saturated — and clamped, so indistinguishable from exact
	assert(!found);        // ...while the body we actually wanted is gone

	printf("  undersized buffer hides dynamic body: OK\n");
}

// The fix: size to the solid count and the body is always reported.
void test_buffer_sized_to_solid_count_finds_dynamic_body() {
	Scene scene(100);

	const int capacity = (int)scene.sim->get_solids().size();
	assert(capacity == 101);

	int count = 0;
	const bool found = scene.query_finds_dynamic(capacity, &count);
	assert(count == 101);
	assert(found);

	printf("  buffer sized to solid count finds dynamic body: OK\n");
}

// get_solids() is the sizing input hop-godot uses, so pin that it counts both
// buckets — a static-only count would reintroduce the truncation for dynamics.
void test_solid_count_covers_both_buckets() {
	Scene scene(7);

	assert(scene.mgr.get_static_count() == 7);
	assert(scene.mgr.get_dynamic_count() == 1);
	assert(scene.sim->get_solids().size() == 8);

	printf("  solid count covers both buckets: OK\n");
}

// The prefilter is the reason a caller doesn't have to choose between truncating and
// iterating the whole world: with a mask, the statics never occupy a slot at all, so
// even the undersized buffer that failed above now finds the body.
void test_mask_prefilter_keeps_dynamic_body_in_a_small_buffer() {
	Scene scene(100);

	int count = 0;
	const bool found = scene.query_finds_dynamic(4, &count, layer_body);
	assert(count == 1);   // 100 statics rejected inside the traversal
	assert(found);

	printf("  mask prefilter keeps dynamic body in a small buffer: OK\n");
}

// A mask that matches nothing must report nothing — the filter rejects, it doesn't
// fall back to reporting everything.
void test_mask_matching_nothing_reports_nothing() {
	Scene scene(5);

	std::vector<solid<T> *> found(16);
	const int count = scene.sim->find_solids_in_aa_box(query_box(), found.data(), 16, 1 << 7);
	assert(count == 0);

	printf("  mask matching nothing reports nothing: OK\n");
}

// -1 means "no filter", not "all bits set": a scope-0 solid has no bit in common with
// anything, so a bitwise test would drop it, but the unfiltered query must still
// report it (that is the historical behavior hop's own broad-phase calls rely on).
void test_unfiltered_query_reports_scope_zero_solids() {
	simulator<T> sim;
	bvh_manager<T> mgr;
	sim.set_gravity(vec3<T>{});
	sim.set_manager(&mgr);

	auto s = make_solid_at_origin(false, 0);
	sim.add_solid(s);
	mgr.add_solid(s.get(), false);

	std::vector<solid<T> *> found(4);
	const int count = sim.find_solids_in_aa_box(query_box(), found.data(), 4);
	assert(count == 1);
	assert(found[0] == s.get());

	printf("  unfiltered query reports scope-0 solids: OK\n");
}

// Exactly-sized is enough: capacity == overlap count must not truncate.
void test_exact_capacity_is_sufficient() {
	Scene scene(0);  // just the dynamic body

	int count = 0;
	assert(scene.query_finds_dynamic(1, &count));
	assert(count == 1);

	printf("  exact capacity is sufficient: OK\n");
}

} // namespace

int main() {
	printf("Broad-phase capacity tests\n");
	test_undersized_buffer_hides_dynamic_body();
	test_buffer_sized_to_solid_count_finds_dynamic_body();
	test_solid_count_covers_both_buckets();
	test_mask_prefilter_keeps_dynamic_body_in_a_small_buffer();
	test_mask_matching_nothing_reports_nothing();
	test_unfiltered_query_reports_scope_zero_solids();
	test_exact_capacity_is_sufficient();
	printf("All broad-phase capacity tests passed\n");
	return 0;
}
