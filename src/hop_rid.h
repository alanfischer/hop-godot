#pragma once

#include <godot_cpp/templates/rid_owner.hpp>

#include <cstdint>
#include <cstring>
#include <vector>

using namespace godot;

// Wrapper around godot-cpp's RID_PtrOwner with for_each support.
//
// for_each runs several times per flush over every body and every area. Resolving each entry
// through RIDs costs engine-interface calls apiece in godot-cpp (rid_from_int64, RID compare,
// get_id), which across a few hundred entries was most of the flush. So the owner mirrors its own
// entries in a table indexed by the RID's slot, walked in the allocator's own slot order.
template <typename T>
class HopRIDOwner : public RID_PtrOwner<T> {
	struct Slot {
		T *ptr = nullptr;
		uint64_t made = 0;  // when the entry was made, against `made_count`
	};
	std::vector<Slot> slots;
	uint64_t made_count = 0;

	static uint32_t slot_of(const RID &r) {
		static_assert(sizeof(RID) == sizeof(uint64_t), "a RID is its id");
		uint64_t id;
		std::memcpy(&id, &r, sizeof id);
		return uint32_t(id & 0xFFFFFFFF);
	}

public:
	RID make_rid(T *p_ptr) {
		RID rid = RID_PtrOwner<T>::make_rid(p_ptr);
		const uint32_t slot = slot_of(rid);
		if (slot >= slots.size()) slots.resize(slot + 1);
		slots[slot] = { p_ptr, ++made_count };
		return rid;
	}

	void free(const RID &p_rid) {
		const uint32_t slot = slot_of(p_rid);
		if (slot < slots.size() && slots[slot].ptr == this->get_or_null(p_rid)) slots[slot] = {};
		RID_PtrOwner<T>::free(p_rid);
	}

	// Snapshot semantics, as when the RIDs were captured up front: func() may free entries (an
	// area monitor callback that frees the node, say), which are then skipped, and entries made
	// during the walk are not visited, even in a slot just freed.
	template <typename F>
	void for_each(F &&func) {
		const uint64_t start = made_count;
		const size_t count = slots.size();
		for (size_t i = 0; i < count; i++) {
			const Slot s = slots[i];
			if (s.ptr && s.made <= start) func(s.ptr);
		}
	}
};
