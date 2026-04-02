#include "hop_shape_data.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/array.hpp>

#include "hop_conversions.h"

#include <algorithm>
#include <cmath>

// Build a hop::convex_solid from a set of convex hull vertices by extracting
// the unique bounding planes.  For each triplet of non-collinear vertices we
// compute the plane and check that every other vertex is on the inside (or on
// the plane).  Duplicate / near-duplicate planes are merged.
static bool build_convex_solid_from_points(
		const Vector3 *points, int count, const Vector3 &origin,
		hop::convex_solid<hop_scalar> &out) {

	if (count < 4) return false;

	const float PLANE_EPS = 1e-4f;
	const float NORMAL_EPS = 0.999f; // dot threshold for "same normal"

	// Work in Godot floats for geometry, convert planes at the end
	struct PlaneF { Vector3 normal; float distance; };
	std::vector<PlaneF> planes;

	for (int i = 0; i < count - 2; ++i) {
		for (int j = i + 1; j < count - 1; ++j) {
			for (int k = j + 1; k < count; ++k) {
				Vector3 a = points[i] + origin;
				Vector3 b = points[j] + origin;
				Vector3 c = points[k] + origin;

				Vector3 edge1 = b - a;
				Vector3 edge2 = c - a;
				Vector3 n = edge1.cross(edge2);
				float len = n.length();
				if (len < 1e-6f) continue;
				n /= len;

				float d = n.dot(a);

				bool valid = true;
				bool has_inside = false;
				for (int l = 0; l < count; ++l) {
					if (l == i || l == j || l == k) continue;
					Vector3 p = points[l] + origin;
					float side = n.dot(p) - d;
					if (side > PLANE_EPS) {
						valid = false;
						break;
					}
					if (side < -PLANE_EPS) {
						has_inside = true;
					}
				}

				if (!valid) {
					n = -n;
					d = -d;
					valid = true;
					has_inside = false;
					for (int l = 0; l < count; ++l) {
						if (l == i || l == j || l == k) continue;
						Vector3 p = points[l] + origin;
						float side = n.dot(p) - d;
						if (side > PLANE_EPS) {
							valid = false;
							break;
						}
						if (side < -PLANE_EPS) {
							has_inside = true;
						}
					}
				}

				if (!valid || !has_inside) continue;

				bool duplicate = false;
				for (auto &existing : planes) {
					float ndot = existing.normal.dot(n);
					if (ndot > NORMAL_EPS &&
						std::abs(existing.distance - d) < PLANE_EPS) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					planes.push_back({n, d});
				}
			}
		}
	}

	if (planes.size() < 4) return false;

	for (auto &pf : planes) {
		out.planes.push_back(hop::plane<hop_scalar>(
			to_hop(pf.normal), to_hop_scalar(pf.distance)));
	}
	return true;
}

void HopShapeData::set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data) {
	type = p_type;
	data = p_data;
	hop_shape.reset();
}

std::shared_ptr<hop::shape<hop_scalar>> HopShapeData::make_hop_shape(const Transform3D &p_local_xform) const {
	Vector3 origin = p_local_xform.origin;

	switch (type) {
		case PhysicsServer3D::SHAPE_SPHERE: {
			float radius = (float)data;
			hop::sphere<hop_scalar> s(to_hop(origin), to_hop_scalar(radius));
			return std::make_shared<hop::shape<hop_scalar>>(s);
		}

		case PhysicsServer3D::SHAPE_BOX: {
			Vector3 half_extents = data;
			Vector3 mn = origin - half_extents;
			Vector3 mx = origin + half_extents;
			hop::aa_box<hop_scalar> box(to_hop(mn), to_hop(mx));
			return std::make_shared<hop::shape<hop_scalar>>(box);
		}

		case PhysicsServer3D::SHAPE_CAPSULE: {
			Dictionary d = data;
			float radius = d.get("radius", 0.5f);
			float height = d.get("height", 2.0f);
			float half_height = height * 0.5f - radius;
			Vector3 cap_bottom = origin + Vector3(0, -half_height, 0);
			Vector3 cap_dir = Vector3(0, half_height * 2.0f, 0);
			hop::capsule<hop_scalar> c(
				to_hop(cap_bottom),
				to_hop(cap_dir),
				to_hop_scalar(radius)
			);
			return std::make_shared<hop::shape<hop_scalar>>(c);
		}

		case PhysicsServer3D::SHAPE_CONVEX_POLYGON: {
			PackedVector3Array points = data;
			if (points.size() < 4) {
				return nullptr;
			}
			hop::convex_solid<hop_scalar> cs;
			if (build_convex_solid_from_points(points.ptr(), points.size(), origin, cs)) {
				return std::make_shared<hop::shape<hop_scalar>>(cs);
			}
			// Fallback to AABB if plane extraction fails
			Vector3 mn = points[0] + origin;
			Vector3 mx = mn;
			for (int i = 1; i < points.size(); i++) {
				Vector3 p = points[i] + origin;
				mn = mn.min(p);
				mx = mx.max(p);
			}
			hop::aa_box<hop_scalar> box(to_hop(mn), to_hop(mx));
			return std::make_shared<hop::shape<hop_scalar>>(box);
		}

		case PhysicsServer3D::SHAPE_CYLINDER: {
			Dictionary d = data;
			float radius = d.get("radius", 0.5f);
			float height = d.get("height", 2.0f);
			float half_h = height * 0.5f;
			Vector3 ho_v = origin;

			// Approximate cylinder as convex solid: N side planes + top/bottom caps
			const int SIDES = 12;
			hop::convex_solid<hop_scalar> cs;
			// Top and bottom cap planes
			cs.planes.push_back(hop::plane<hop_scalar>(
				to_hop(Vector3(0, 1, 0)), to_hop_scalar(ho_v.y + half_h)));
			cs.planes.push_back(hop::plane<hop_scalar>(
				to_hop(Vector3(0, -1, 0)), to_hop_scalar(-(ho_v.y - half_h))));
			// Side planes
			for (int i = 0; i < SIDES; ++i) {
				float angle = (float)i / (float)SIDES * 2.0f * 3.14159265f;
				float nx = std::cos(angle);
				float nz = std::sin(angle);
				float d_val = nx * (ho_v.x + radius * nx) + nz * (ho_v.z + radius * nz);
				cs.planes.push_back(hop::plane<hop_scalar>(
					to_hop(Vector3(nx, 0, nz)), to_hop_scalar(d_val)));
			}
			return std::make_shared<hop::shape<hop_scalar>>(cs);
		}

		default:
			return nullptr;
	}
}
