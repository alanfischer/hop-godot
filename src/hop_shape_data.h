#pragma once

#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <hop/hop.h>
#include <memory>

using namespace godot;

struct HopShapeData {
	PhysicsServer3D::ShapeType type = PhysicsServer3D::SHAPE_CUSTOM;
	Variant data;
	float margin = 0.0f;
	float custom_solver_bias = 0.0f;

	// Cached hop shape — rebuilt when data changes
	std::shared_ptr<hop::shape<float>> hop_shape;

	void set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data);
	std::shared_ptr<hop::shape<float>> make_hop_shape(const Transform3D &p_local_xform) const;
};
