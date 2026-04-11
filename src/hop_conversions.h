#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/godot.hpp>
#include <hop/fixed16.h>
#include <hop/math/vec3.h>
#include <type_traits>

using namespace godot;

// Safely get an Object* for a known instance id.
// Only returns non-null if the GDExtension binding already exists for this object.
// Avoids crash in get_parent_class when the object's class is a GDScript type.
inline godot::Object *get_collider_safe(uint64_t p_id) {
	if (!p_id) return nullptr;
	GDExtensionObjectPtr obj = godot::internal::gdextension_interface_object_get_instance_from_id(p_id);
	if (!obj) return nullptr;
	return reinterpret_cast<godot::Object *>(
		godot::internal::gdextension_interface_object_get_instance_binding(obj, godot::internal::token, nullptr));
}

// The scalar type used throughout the GDExtension.
// Switch between float and hop::fixed16 here.
using hop_scalar = float;

// --- Scalar conversions ---

template <typename T>
inline T scalar_from_float(float f) {
	if constexpr (std::is_same_v<T, float>) {
		return f;
	} else {
		return T::from_float(f);
	}
}

template <typename T>
inline float scalar_to_float(T s) {
	if constexpr (std::is_same_v<T, float>) {
		return s;
	} else {
		return s.to_float();
	}
}

template <typename T>
inline T scalar_from_int(int i) {
	if constexpr (std::is_same_v<T, float>) {
		return static_cast<float>(i);
	} else {
		return T::from_int(i);
	}
}

template <typename T>
inline T scalar_from_milli(int m) {
	if constexpr (std::is_same_v<T, float>) {
		return static_cast<float>(m) / 1000.0f;
	} else {
		return T::from_milli(m);
	}
}

// --- Convenience wrappers using hop_scalar ---

inline hop_scalar to_hop_scalar(float f) {
	return scalar_from_float<hop_scalar>(f);
}

inline float to_godot_float(hop_scalar s) {
	return scalar_to_float(s);
}

inline hop::vec3<hop_scalar> to_hop(const Vector3 &v) {
	return hop::vec3<hop_scalar>(
		to_hop_scalar(v.x),
		to_hop_scalar(v.y),
		to_hop_scalar(v.z));
}

inline Vector3 to_godot(const hop::vec3<hop_scalar> &v) {
	return Vector3(
		scalar_to_float(v.x),
		scalar_to_float(v.y),
		scalar_to_float(v.z));
}
