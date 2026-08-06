#pragma once

#include <godot_cpp/templates/rid_owner.hpp>

#include <vector>

using namespace godot;

// Wrapper around godot-cpp's RID_PtrOwner with for_each support.
template <typename T>
class HopRIDOwner : public RID_PtrOwner<T> {
	// Snapshot buffer reused across calls. for_each runs several times per physics
	// tick over every body and every area, so the obvious List<RID> snapshot cost a
	// heap node per entry per call — with a few hundred areas that was the single
	// largest allocator in the physics frame. A vector that keeps its capacity makes
	// steady-state iteration allocation-free.
	std::vector<RID> iter_buffer;
	bool iterating = false;

	// Snapshot semantics matter: func() may free the entry it is handed (an area
	// monitor callback that frees the node, say), so the RID list is captured up
	// front and each entry re-resolved through get_or_null before use.
	template <typename F>
	void iterate(const RID *rids, uint32_t count, F &&func) {
		for (uint32_t i = 0; i < count; i++) {
			T *ptr = this->get_or_null(rids[i]);
			if (ptr) {
				func(ptr);
			}
		}
	}

public:
	// Iterate all owned entries
	template <typename F>
	void for_each(F &&func) {
		const uint32_t count = this->get_rid_count();
		if (count == 0) return;
		if (iterating) {
			// Nested for_each on the same owner (a callback that iterates again):
			// the shared buffer is live, so take a private snapshot for this level.
			std::vector<RID> nested(count);
			this->fill_owned_buffer(nested.data());
			iterate(nested.data(), count, func);
			return;
		}
		struct Guard {
			bool &flag;
			explicit Guard(bool &f) : flag(f) { flag = true; }
			~Guard() { flag = false; }
		} guard(iterating);
		iter_buffer.resize(count);
		this->fill_owned_buffer(iter_buffer.data());
		iterate(iter_buffer.data(), count, func);
	}
};
