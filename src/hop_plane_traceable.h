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
	                   const hop::segment<T> &seg) override {
		T denom = hop::dot(normal_, seg.direction);
		if (denom >= T {})
			return; // Moving away or parallel

		T t = (distance_ + hop::dot(normal_, position) - hop::dot(normal_, seg.origin)) / denom;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			hop::mul(result.point, seg.direction, t);
			hop::add(result.point, seg.origin);
			result.normal = normal_;
		}
	}

	void trace_solid(hop::collision<T> &result,
	                 hop::solid<T> *s,
	                 const hop::vec3<T> &position,
	                 const hop::segment<T> &seg, T margin) override {
		// Find the solid's extent in the negative-normal direction (into the plane)
		hop::vec3<T> neg_n;
		hop::neg(neg_n, normal_);

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

		T origin_dot = hop::dot(normal_, seg.origin) - hop::dot(normal_, position);
		T denom = hop::dot(normal_, seg.direction);

		// Zero-direction (static overlap / proximity) query: the swept solve
		// below divides by `denom`, so handle a stationary solid separately.
		if (denom == T {}) {
			if (origin_dot <= expanded_d && result.time > T {}) {
				result.time = T {};
				result.normal = normal_;
				result.point = seg.origin;
				// depth measured against the inflated surface (= margin − true_gap)
				result.depth = expanded_d - origin_dot;
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
			result.normal = normal_;
		}
	}

private:
	hop::vec3<T> normal_;
	T distance_ {};
};
