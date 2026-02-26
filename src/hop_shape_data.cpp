#include "hop_shape_data.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/array.hpp>

#include "hop_conversions.h"

void HopShapeData::set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data) {
	type = p_type;
	data = p_data;
	hop_shape.reset();
}

std::shared_ptr<hop::shape<float>> HopShapeData::make_hop_shape(const Transform3D &p_local_xform) const {
	Vector3 origin = p_local_xform.origin;

	switch (type) {
		case PhysicsServer3D::SHAPE_SPHERE: {
			float radius = (float)data;
			hop::sphere<float> s(to_hop(origin), radius);
			return std::make_shared<hop::shape<float>>(s);
		}

		case PhysicsServer3D::SHAPE_BOX: {
			Vector3 half_extents = data;
			hop::vec3<float> ho = to_hop(origin);
			hop::aa_box<float> box(
				hop::vec3<float>(ho.x - half_extents.x, ho.y - half_extents.y, ho.z - half_extents.z),
				hop::vec3<float>(ho.x + half_extents.x, ho.y + half_extents.y, ho.z + half_extents.z)
			);
			return std::make_shared<hop::shape<float>>(box);
		}

		case PhysicsServer3D::SHAPE_CAPSULE: {
			Dictionary d = data;
			float radius = d.get("radius", 0.5f);
			float height = d.get("height", 2.0f);
			float half_height = height * 0.5f - radius;
			hop::vec3<float> ho = to_hop(origin);
			// Godot capsules are Y-aligned
			hop::capsule<float> c(
				hop::vec3<float>(ho.x, ho.y - half_height, ho.z),
				hop::vec3<float>(0.0f, half_height * 2.0f, 0.0f),
				radius
			);
			return std::make_shared<hop::shape<float>>(c);
		}

		case PhysicsServer3D::SHAPE_CONVEX_POLYGON: {
			PackedVector3Array points = data;
			if (points.size() < 4) {
				return nullptr;
			}
			// Build convex solid from planes using face normals
			// For now, use aa_box bounding the points as approximation
			// TODO: proper convex hull -> plane extraction
			Vector3 mn = points[0] + origin;
			Vector3 mx = mn;
			for (int i = 1; i < points.size(); i++) {
				Vector3 p = points[i] + origin;
				mn = mn.min(p);
				mx = mx.max(p);
			}
			hop::aa_box<float> box(to_hop(mn), to_hop(mx));
			return std::make_shared<hop::shape<float>>(box);
		}

		default:
			return nullptr;
	}
}
