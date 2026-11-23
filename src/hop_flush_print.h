#pragma once

#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <hop/hop.h>
#include <cstdint>
#include <cstring>
#include "hop_conversions.h"

using namespace godot;

// A RID's id, read straight out of it (0 for an invalid one). godot-cpp copies, compares and
// validates a RID through the engine interface, which the flush would pay hundreds of times.
inline uint64_t rid_key(const RID &r) {
	static_assert(sizeof(RID) == sizeof(uint64_t), "a RID is its id");
	uint64_t k;
	std::memcpy(&k, &r, sizeof k);
	return k;
}

// What an area's overlap test reads off a body or an area, as of the last _flush_queries. An area
// whose own print is unchanged, and that no changed print touches, cannot have gained or lost an
// overlap, so the flush skips its broadphase query. Read off the live state rather than kept up
// by the setters (the callbacks aside), so a setter that forgets to say it changed something
// cannot strand a trigger.
struct HopFlushPrint {
	bool valid = false;
	uint64_t space = 0;  // rid_key of the space
	Transform3D transform;
	hop::aa_box<hop_scalar> bound;
	uint64_t bits = 0;       // layer and mask
	uint32_t flags = 0;      // whatever else the owner's overlap test reads
	static constexpr uint32_t AREA_MONITORABLE = 1;
	uintptr_t shapes = 0;    // the hop shapes in use, folded together

	bool operator==(const HopFlushPrint &o) const {
		return valid == o.valid && space == o.space && transform == o.transform
			&& bound.mins == o.bound.mins && bound.maxs == o.bound.maxs
			&& bits == o.bits && flags == o.flags && shapes == o.shapes;
	}
	bool operator!=(const HopFlushPrint &o) const { return !(*this == o); }

	static uintptr_t fold_shapes(const hop::solid<hop_scalar> *s) {
		if (!s) return 0;
		uintptr_t h = s->get_shapes().size();
		for (const auto &sh : s->get_shapes())
			h = h * 31 + reinterpret_cast<uintptr_t>(sh.get());
		return h;
	}
};
