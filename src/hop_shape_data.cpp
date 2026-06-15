#include "hop_shape_data.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

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

	// Always clamp to the point-cloud AABB. A hop::convex_solid is the intersection
	// of its half-spaces and is UNBOUNDED unless the extracted planes fully enclose
	// it. The triplet/epsilon face extraction above misses a cap whenever a face's
	// vertices aren't coplanar — e.g. ww_2fort's elevator guide-post, whose bottom
	// 4 verts are skewed ~5u, leaving the -Y face open so the solid extended to
	// infinity and walled off the lift approach. The 6 axis-aligned bounding planes
	// can never clip the true hull (every vertex is inside its own AABB), so they
	// only remove the spurious unbounded extension. (Slivers are dropped earlier in
	// make_hop_shape; this clamp is what keeps legitimately-kept thin cells bounded.)
	Vector3 mn = points[0] + origin;
	Vector3 mx = mn;
	for (int i = 1; i < count; ++i) {
		Vector3 p = points[i] + origin;
		mn = mn.min(p);
		mx = mx.max(p);
	}
	planes.push_back({ Vector3( 1, 0, 0),  mx.x });
	planes.push_back({ Vector3(-1, 0, 0), -mn.x });
	planes.push_back({ Vector3( 0, 1, 0),  mx.y });
	planes.push_back({ Vector3( 0,-1, 0), -mn.y });
	planes.push_back({ Vector3( 0, 0, 1),  mx.z });
	planes.push_back({ Vector3( 0, 0,-1), -mn.z });

	for (auto &pf : planes) {
		out.planes.push_back(hop::plane<hop_scalar>(
			to_hop(pf.normal), to_hop_scalar(pf.distance)));
	}

	// Pre-populate the vertex cache with the ACTUAL input hull points. hop's
	// convex_solid normally derives its vertices lazily from the planes (plane
	// triple-intersections), but that enumeration produces wrong/far vertices when
	// the extracted plane set is ill-conditioned or fails to fully enclose the hull
	// (a thin/skewed cell's missing cap) — and support()/get_bound() run off those
	// vertices, so the solid collides where no surface is (ww_2fort's elevator
	// guide-post phantom-walled the lift approach). Seeding vertices with the real
	// points makes collision use the true, bounded hull regardless of plane health.
	for (int i = 0; i < count; ++i) {
		out.vertices.push_back(to_hop(points[i] + origin));
	}
	return true;
}

void HopShapeData::set_data(PhysicsServer3D::ShapeType p_type, const Variant &p_data) {
	type = p_type;
	data = p_data;
	hop_shape.reset();
}

std::shared_ptr<hop::shape<hop_scalar>> HopShapeData::make_hop_shape(const Transform3D &p_local_xform) {
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

			// AABB of the hull (in the shape's local+offset frame).
			Vector3 mn = points[0] + origin;
			Vector3 mx = mn;
			for (int i = 1; i < points.size(); i++) {
				Vector3 p = points[i] + origin;
				mn = mn.min(p);
				mx = mx.max(p);
			}
			Vector3 ext = mx - mn;
			const float MIN_DIM = 0.2f; // 8 game units
			int thin = int(ext.x < MIN_DIM) + int(ext.y < MIN_DIM) + int(ext.z < MIN_DIM);

			// "Rod" cells — thin on 2+ axes (e.g. ww_2fort's 886u elevator guide-
			// post). The triplet plane extraction leaves these unbounded whenever a
			// cap face's verts aren't coplanar, and the plane-based capsule sweep
			// (trace_convex_solid) then treats the solid as extending to infinity —
			// phantom-walling nearby geometry (this was the lift's 52u wall). A
			// rod's AABB is a faithful box (it is ~axis-aligned) and aa_box
			// collision is exact and bounded, so emit a box instead of a convex.
			if (thin >= 2) {
				hop::aa_box<hop_scalar> box(to_hop(mn), to_hop(mx));
				return std::make_shared<hop::shape<hop_scalar>>(box);
			}

			// Degenerate skewed "slivers" — fat AABB (thin on no axis) yet vertices
			// at only <=2 opposite corners: near-zero volume, no real surface. The
			// BSP-leaf decomposition emits several of these; they have no business
			// colliding (Godot's hull builder discards them too). Drop them.
			if (thin == 0) {
				const float tol = 0.15f * ext.length();
				int occupied = 0;
				for (int cx = 0; cx < 2; cx++)
					for (int cy = 0; cy < 2; cy++)
						for (int cz = 0; cz < 2; cz++) {
							Vector3 corner(cx ? mx.x : mn.x, cy ? mx.y : mn.y, cz ? mx.z : mn.z);
							for (int i = 0; i < points.size(); i++) {
								if ((points[i] + origin).distance_to(corner) < tol) {
									occupied++;
									break;
								}
							}
						}
				if (occupied <= 2) {
					return nullptr;
				}
			}

			hop::convex_solid<hop_scalar> cs;
			if (build_convex_solid_from_points(points.ptr(), points.size(), origin, cs)) {
				return std::make_shared<hop::shape<hop_scalar>>(cs);
			}
			// Fallback to AABB if plane extraction fails.
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

		case PhysicsServer3D::SHAPE_WORLD_BOUNDARY: {
			Plane p = data;
			plane_traceable = std::make_shared<HopPlaneTraceable<hop_scalar>>();
			plane_traceable->set_plane(to_hop(p.normal), to_hop_scalar(p.d));
			return std::make_shared<hop::shape<hop_scalar>>(plane_traceable.get());
		}

		case PhysicsServer3D::SHAPE_CONCAVE_POLYGON: {
			Dictionary d = data;
			PackedVector3Array faces = d["faces"];
			bool backface_collision = d.get("backface_collision", false);
			int face_count = faces.size();
			// Godot passes concave polygon data as a flat array of face vertices
			// (3 vertices per triangle)
			if (face_count < 3 || (face_count % 3) != 0)
				return nullptr;

			int tri_count = face_count / 3;
			const Vector3 *pts = faces.ptr();

			// Deduplicate vertices and build index list
			std::vector<hop::vec3<hop_scalar>> verts;
			std::vector<HopTrimeshTraceable<hop_scalar>::triangle> tris;
			verts.reserve(face_count);
			tris.reserve(tri_count);

			auto find_or_add = [&](const Vector3 &p) -> int {
				hop::vec3<hop_scalar> hp = to_hop(p + origin);
				for (int i = 0; i < (int)verts.size(); ++i) {
					hop::vec3<hop_scalar> diff;
					hop::sub(diff, verts[i], hp);
					if (hop::length_squared(diff) < to_hop_scalar(1e-8f))
						return i;
				}
				verts.push_back(hp);
				return (int)verts.size() - 1;
			};

			for (int i = 0; i < tri_count; ++i) {
				int i0 = find_or_add(pts[i * 3 + 0]);
				int i1 = find_or_add(pts[i * 3 + 1]);
				int i2 = find_or_add(pts[i * 3 + 2]);
				if (i0 == i1 || i1 == i2 || i0 == i2)
					continue; // degenerate
				tris.push_back({ i0, i2, i1 }); // GoldSrc BSP uses CW winding from outside → reverse to get outward normals
			}

			if (tris.empty())
				return nullptr;

			trimesh_traceable = std::make_shared<HopTrimeshTraceable<hop_scalar>>();
			trimesh_traceable->build(verts.data(), (int)verts.size(),
			                         tris.data(), (int)tris.size(),
			                         backface_collision);
			return std::make_shared<hop::shape<hop_scalar>>(trimesh_traceable.get());
		}

		case PhysicsServer3D::SHAPE_HEIGHTMAP: {
			// Godot passes heightmap data as {width, depth, heights (row-major
			// PackedFloat32Array, depth rows of width), min_height, max_height}.
			Dictionary d = data;
			int w = (int)d.get("width", 0);
			int dp = (int)d.get("depth", 0);
			PackedFloat32Array heights = d.get("heights", PackedFloat32Array());
			if (w < 2 || dp < 2 || heights.size() != w * dp)
				return nullptr;

			// The grid is 1-unit-spaced in shape-local X/Z with heights along Y;
			// real spacing/scale lives in the transform basis (hop solids carry no
			// scale, so bake it into the traceable). to_hop is identity, so godot
			// X/Z/Y map straight through.
			Vector3 gscale = p_local_xform.basis.get_scale();
			heightfield_traceable = std::make_shared<HopHeightfieldTraceable<hop_scalar>>();
			heightfield_traceable->build(w, dp, heights.ptr(), to_hop(gscale), to_hop(origin));
			return std::make_shared<hop::shape<hop_scalar>>(heightfield_traceable.get());
		}

		default:
			return nullptr;
	}
}
