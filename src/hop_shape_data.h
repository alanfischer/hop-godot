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

	// Drop the built geometry, keeping `data` — the next make_hop_shape rebuilds it
	// from scratch, which is what that function does on every call anyway.
	//
	// For a trimesh that geometry is verts + per-triangle normals + a BVH, several
	// times the size of the source triangle soup. A shape whose only users have
	// switched to BSP hull collision is holding all of it for nothing, so the
	// hull toggle releases it (see HopPhysicsServer::set_bsp_hulls_enabled).
	//
	// Only safe once no live hop::shape points here: hop::shape holds the traceable
	// as a RAW pointer, so freeing one out from under a shape still in a solid
	// dangles it. The caller is responsible for establishing that.
	void release_derived() {
		hop_shape.reset();
		trimesh_traceable.reset();
		plane_traceable.reset();
		heightfield_traceable.reset();
	}

	// Builds the shape geometry from a rotation-stripped (scale + translation)
	// transform. make_hop_shape splits the local transform into rotation (carried
	// as the shape's static local_rotation, honored by the narrowphase) and the
	// scale/origin baked here, since hop geometry has no orientation of its own.
	std::shared_ptr<hop::shape<hop_scalar>> build_shape_geometry(const Transform3D &p_local_xform);
};
