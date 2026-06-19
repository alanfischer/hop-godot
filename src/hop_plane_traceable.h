#pragma once

#include <hop/hop.h>
#include "hop_conversions.h"

// A traceable implementation for an infinite plane (world boundary).

template <typename T>
class HopPlaneTraceable : public hop::traceable<T> {
	using tr = hop::scalar_traits<T>;

public:
	void set_plane(const hop::vec3<T> &p_normal, T p_distance) {
		normal_ = p_normal;
		distance_ = p_distance;
	}

	void get_bound(hop::aa_box<T> &result) override {
		// Infinite plane — use a large bounding box
		T big = tr::from_int(10000);
		result.mins = { -big, -big, -big };
		result.maxs = { big, big, big };
	}

	void trace_segment(hop::collision<T> &result,
	                   const hop::vec3<T> &position,
	                   const hop::mat3<T> &orientation,
	                   const hop::segment<T> &seg) override {
		// The plane normal is authored in local space; a static rotation just turns
		// it. Working with the world-rotated plane (normal n = R·normal_ through
		// `position`) needs no query transform and is identity-equivalent — and the
		// common identity case skips the matrix multiply entirely.
		hop::vec3<T> normal = world_normal(orientation);

		T denom = hop::dot(normal, seg.direction);
		if (denom >= T {})
			return; // Moving away or parallel

		T t = (distance_ + hop::dot(normal, position) - hop::dot(normal, seg.origin)) / denom;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			hop::mul(result.point, seg.direction, t);
			hop::add(result.point, seg.origin);
			result.normal = normal;
		}
	}

	void trace_solid(hop::collision<T> &result,
	                 hop::solid<T> *s,
	                 const hop::vec3<T> &position,
	                 const hop::mat3<T> &orientation,
	                 const hop::segment<T> &seg, T margin) override {
		hop::vec3<T> normal = world_normal(orientation);

		// Find the solid's extent in the negative-normal direction (into the plane)
		hop::vec3<T> neg_n;
		hop::neg(neg_n, normal);

		T deepest = T {};
		for (const auto &shape : s->get_shapes()) {
			hop::vec3<T> sup;
			hop::support(sup, *shape, neg_n);
			T extent = hop::dot(sup, neg_n);
			if (extent > deepest)
				deepest = extent;
		}

		// Expand the plane outward by the shape's extent, plus the speculative
		// margin so near-resting solids within `margin` register as contacts.
		T expanded_d = distance_ + deepest + margin;

		T origin_dot = hop::dot(normal, seg.origin) - hop::dot(normal, position);
		T denom = hop::dot(normal, seg.direction);

		// Zero-direction (static overlap / proximity) query: the swept solve
		// below divides by `denom`, so handle a stationary solid separately.
		if (denom == T {}) {
			if (origin_dot <= expanded_d && result.time > T {}) {
				result.time = T {};
				result.normal = normal;
				result.point = seg.origin;
				// depth measured against the inflated surface (= margin − true_gap)
				result.depth = expanded_d - origin_dot;
				project_to_plane(result.impact, result.point, normal, position);
			}
			return;
		}
		if (denom > T {})
			return; // moving away

		T t = (expanded_d - origin_dot) / denom;
		if (t <= tr::one() && t < result.time) {
			if (t < T {}) {
				// Already inside the inflated surface — clamp so the solver can
				// push back out, and report how far past the surface we are.
				result.depth = expanded_d - origin_dot;
				t = T {};
			}
			result.time = t;
			hop::mul(result.point, seg.direction, t);
			hop::add(result.point, seg.origin);
			result.normal = normal;
			project_to_plane(result.impact, result.point, normal, position);
		}
	}

private:
	// Real contact point on the plane = the mover origin projected onto the world
	// plane (dot(n, x − position) = distance_). For lever arms (carry / angular).
	void project_to_plane(hop::vec3<T> &out, const hop::vec3<T> &p,
	                      const hop::vec3<T> &normal, const hop::vec3<T> &position) const {
		T gap = hop::dot(normal, p) - hop::dot(normal, position) - distance_;
		hop::vec3<T> ns;
		hop::mul(ns, normal, gap);
		hop::sub(out, p, ns);
	}

	// World plane normal under a static rotation: R·normal_. The common identity
	// case returns the stored normal directly, skipping the matrix multiply.
	hop::vec3<T> world_normal(const hop::mat3<T> &orientation) const {
		static const hop::mat3<T> identity;
		if (orientation == identity)
			return normal_;
		hop::vec3<T> n;
		hop::mul(n, orientation, normal_);
		return n;
	}

	hop::vec3<T> normal_;
	T distance_ {};
};
