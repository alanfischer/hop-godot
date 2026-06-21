#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/godot.hpp>
#include <hop/fixed16.h>
#include <hop/fixed32.h>
#include <hop/math/vec3.h>
#include <type_traits>

using namespace godot;

// Resolve the engine Object pointer for an instance id, for use in physics
// result structs (RayResult/ShapeResult/MotionCollision `.collider`).
//
// These fields are read DIRECTLY by the engine, which expects the raw engine
// object pointer (GDExtensionObjectPtr) — exactly what object_get_instance_from_id
// returns. It must NOT be the godot-cpp binding *wrapper*
// (object_get_instance_binding / ObjectDB::get_instance): the wrapper is a
// different address, and the engine misreads it as a freed object, so GDScript
// sees a "previously freed" instance when it dereferences result.collider.
inline godot::Object *get_collider_safe(uint64_t p_id) {
	if (!p_id) return nullptr;
	return reinterpret_cast<godot::Object *>(
		godot::gdextension_interface::object_get_instance_from_id(p_id));
}

// The scalar type used throughout the GDExtension.
// Switch between float and hop::fixed16 here.
using hop_scalar = hop::fixed32;

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

// Godot Basis → hop mat3 (rotation only; pass an orthonormal basis). Both are
// applied as r[i] = Σⱼ m(i,j)·v[j] and to_hop maps godot XYZ straight through, so
// element (i,j) copies directly. The hop mat3 ctor is row-major.
inline hop::mat3<hop_scalar> to_hop_mat3(const Basis &b) {
	return hop::mat3<hop_scalar>(
		to_hop_scalar(b[0][0]), to_hop_scalar(b[0][1]), to_hop_scalar(b[0][2]),
		to_hop_scalar(b[1][0]), to_hop_scalar(b[1][1]), to_hop_scalar(b[1][2]),
		to_hop_scalar(b[2][0]), to_hop_scalar(b[2][1]), to_hop_scalar(b[2][2]));
}

// hop solid orientation (rotation only) from a Godot transform basis: drops the
// basis's scale (which hop bakes into geometry) and keeps the pure rotation.
inline hop::mat3<hop_scalar> to_hop_orientation(const Basis &b) {
	return to_hop_mat3(Basis(b.get_rotation_quaternion()));
}
