#pragma once

#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <hop/hop.h>
#include <memory>
#include "hop_conversions.h"
#include "hop_heightfield_traceable.h"
#include "hop_plane_traceable.h"
#include "hop_trimesh_traceable.h"

using namespace godot;

struct HopShapeData {
	PhysicsServer3D::ShapeType type = PhysicsServer3D::SHAPE_CUSTOM;
	Variant data;
	float margin = 0.0f;
	float custom_solver_bias = 0.0f;

	// No traceables are cached here. Each make_hop_shape builds its own and hands
	// ownership to the hop::shape it returns, so the geometry lives exactly as long
	// as the shape pointing at it — even when several collision objects share this
	// RID, and even though they may bake different local transforms into it.
	void set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data);
	std::shared_ptr<hop::shape<hop_scalar>> make_hop_shape(const Transform3D &p_local_xform);

	// Builds the shape geometry from a rotation-stripped (scale + translation)
	// transform. make_hop_shape splits the local transform into rotation (carried
	// as the shape's static local_rotation, honored by the narrowphase) and the
	// scale/origin baked here, since hop geometry has no orientation of its own.
	std::shared_ptr<hop::shape<hop_scalar>> build_shape_geometry(const Basis &p_scale);
};
