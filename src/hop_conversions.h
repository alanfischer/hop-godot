#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <hop/math/vec3.h>

using namespace godot;

inline hop::vec3<float> to_hop(const Vector3 &v) {
	return hop::vec3<float>(v.x, v.y, v.z);
}

inline Vector3 to_godot(const hop::vec3<float> &v) {
	return Vector3(v.x, v.y, v.z);
}
