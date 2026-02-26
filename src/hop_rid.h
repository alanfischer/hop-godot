#pragma once

#include <godot_cpp/templates/rid_owner.hpp>
#include <godot_cpp/templates/list.hpp>

using namespace godot;

// Wrapper around godot-cpp's RID_PtrOwner with for_each support.
template <typename T>
class HopRIDOwner : public RID_PtrOwner<T> {
public:
	// Iterate all owned entries
	template <typename F>
	void for_each(F &&func) {
		List<RID> owned;
		this->get_owned_list(&owned);
		for (const RID &rid : owned) {
			T *ptr = this->get_or_null(rid);
			if (ptr) {
				func(ptr);
			}
		}
	}
};
