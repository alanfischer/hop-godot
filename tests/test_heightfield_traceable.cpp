// Bodies dropped on a HeightMapShape3D-style heightfield rest on it.
//
// A box sweep used to report an overlap as a zero-depth touch, so once a box sank past
// the contact margin nothing pushed it back and the manifold's corner probes, starting
// inside the ground, found nothing: the box sank a little more every tick and fell
// through. And against an unturned grid a turned box was sized as an axis-aligned one.
// Sloped ground showed both; a flat grid hid them. Each case here drops a body on a
// plane-shaped grid, turned 180° about Y or not, and checks it never sinks into it.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <hop/hop.h>
#include "hop_heightfield_traceable.h"

using T = double;
using V = hop::vec3<T>;

static const int N = 65;
static const T CELL = 1.6;

static V vec(T x, T y, T z) { V v; v.x = x; v.y = y; v.z = z; return v; }

// World ground height: a plane through the origin.
struct Slope { T sx, sz; T at(T x, T z) const { return sx * x + sz * z; } };

static std::unique_ptr<HopHeightfieldTraceable<T>> make_field(const Slope & s, bool turned) {
	std::vector<float> h(N * N);
	const T half = (N - 1) * 0.5;
	for (int j = 0; j < N; ++j)
		for (int i = 0; i < N; ++i) {
			T x = (i - half) * CELL, z = (j - half) * CELL;
			// Turned 180° about Y, grid-local (x, z) lies at world (-x, -z).
			h[j * N + i] = (float)(turned ? s.at(-x, -z) : s.at(x, z));
		}
	auto f = std::make_unique<HopHeightfieldTraceable<T>>();
	f->build(N, N, h.data(), vec(CELL, 1, CELL), V());
	return f;
}

struct Drop { T lowest; T final_height; };

// Drops a body (a sphere of radius hy, or a box of half-extents hx,hy,hz turned by
// `yaw`) from 1 above the ground at the origin; returns its centre's lowest and final
// height over the ground below it, while it is still over the grid.
static Drop drop(const Slope & s, bool turned, bool box, T hx, T hy, T hz, T yaw) {
	hop::simulator<T> sim;
	sim.set_gravity(vec(0, -20, 0));
	sim.set_default_contact_mode(hop::contact_mode::speculative);
	auto world = std::make_shared<hop::solid<T>>();
	world->set_infinite_mass();
	world->set_coefficient_of_gravity(T {});
	world->add_shape(std::make_shared<hop::shape<T>>(make_field(s, turned)));
	if (turned)
		world->set_orientation(hop::mat3<T>(-1, 0, 0, 0, 1, 0, 0, 0, -1));
	sim.add_solid(world);
	world->set_collision_scope(1);
	world->set_collide_with_scope(0);

	auto body = std::make_shared<hop::solid<T>>();
	if (box) {
		hop::aa_box<T> b;
		b.mins = vec(-hx, -hy, -hz);
		b.maxs = vec(hx, hy, hz);
		body->add_shape(std::make_shared<hop::shape<T>>(b));
	} else {
		body->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(V(), hy)));
	}
	// Unit mass, and the solid's own inertia: hop-godot derives it from the shape the same way.
	body->set_mass((T)1);
	if (box)
		body->set_inertia(vec((hy * hy + hz * hz) / 3, (hx * hx + hz * hz) / 3, (hx * hx + hy * hy) / 3));
	else
		body->set_inertia(vec(hy * hy * 0.4, hy * hy * 0.4, hy * hy * 0.4));
	const T c = std::cos(yaw), sn = std::sin(yaw);
	body->set_orientation(hop::mat3<T>(c, 0, sn, 0, 1, 0, -sn, 0, c));
	body->set_position(vec(0, s.at(0, 0) + 1, 0));
	sim.add_solid(body);
	body->set_contact_mode(hop::contact_mode::speculative);
	body->set_collision_scope(0);
	body->set_collide_with_scope(1);

	Drop d {1e9, 0};
	const T edge = (N - 1) * 0.5 * CELL - 3;
	for (int i = 0; i < 300; ++i) {
		sim.update((T)(1.0 / 60.0));
		V p = body->get_position();
		if (std::fabs(p.x) > edge || std::fabs(p.z) > edge)
			break;  // slid off the grid's edge: past here there is no ground to check
		d.final_height = p.y - s.at(p.x, p.z);
		if (d.final_height < d.lowest)
			d.lowest = d.final_height;
	}
	return d;
}

int main() {
	printf("test_heightfield_traceable:\n");
	const Slope slopes[] = {{0, 0}, {0.36, 0}, {0, 0.36}, {0.2, 0.3}};
	const char * names[] = {"sphere", "cube", "plank"};
	int cases = 0;
	for (bool turned : {false, true})
		for (const Slope & s : slopes)
			for (T yaw : {0.0, 0.5, M_PI / 2})
				for (int shape = 0; shape < 3; ++shape) {
					// A plank is longer than a cell: 5 x 0.5 x 1.5.
					const T hx = shape == 2 ? 2.5 : 0.5, hy = shape == 2 ? 0.25 : 0.5,
					        hz = shape == 2 ? 0.75 : 0.5;
					Drop d = drop(s, turned, shape > 0, hx, hy, hz, yaw);
					if (d.lowest < hy * 0.5 || d.final_height < hy * 0.9 || d.final_height > hy * 2)
						printf("  FAIL turned=%d slope=(%.2f,%.2f) yaw=%.2f %s: lowest %.3f final %.3f\n", turned,
						       s.sx, s.sz, yaw, names[shape], d.lowest, d.final_height);
					// Never more than half its depth into the ground, even landing on a corner...
					assert(d.lowest >= hy * 0.5);
					// ...and it comes to rest on it, not in it or held above it.
					assert(d.final_height >= hy * 0.9 && d.final_height <= hy * 2);
					++cases;
				}
	printf("  %d drops rest on the grid\n", cases);
	printf("ALL PASSED\n");
	return 0;
}
