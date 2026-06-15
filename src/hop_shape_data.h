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

	// Cached hop shape — rebuilt when data changes
	std::shared_ptr<hop::shape<hop_scalar>> hop_shape;

	// Owned traceables (raw ptr passed to hop, so we must outlive the shape)
	std::shared_ptr<HopTrimeshTraceable<hop_scalar>> trimesh_traceable;
	std::shared_ptr<HopPlaneTraceable<hop_scalar>> plane_traceable;
	std::shared_ptr<HopHeightfieldTraceable<hop_scalar>> heightfield_traceable;

	void set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data);
	std::shared_ptr<hop::shape<hop_scalar>> make_hop_shape(const Transform3D &p_local_xform);
};
