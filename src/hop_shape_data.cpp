#include "hop_shape_data.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "hop_conversions.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

// Spatial-hash key for trimesh vertex dedup: a vertex position quantized to a
// grid cell. Coincident / bit-identical verts (the common case from Godot's
// flat face array) land in the same cell and share an index, turning dedup from
// an O(N) linear scan per vertex into an O(1) lookup. The cell size matches the
// old 1e-4 merge tolerance; quantizing only ever under-merges (two distinct
// verts never collapse below the grid resolution), so it can't distort geometry.
namespace {
struct VertKey {
	int64_t x, y, z;
	bool operator==(const VertKey &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VertKeyHash {
	size_t operator()(const VertKey &k) const {
		// Cheap spatial hash (large primes, à la Teschner et al.).
		return (size_t)(k.x * 73856093) ^ (size_t)(k.y * 19349663) ^ (size_t)(k.z * 83492791);
	}
};
inline VertKey quantize_vert(const Vector3 &p) {
	// ≈ the old length_squared < 1e-8 merge tolerance. Done in double so binning
	// stays exact at large terrain coordinates (float loses integer precision
	// past ~1.6e7, i.e. ~1.6km at this 1e-4 step).
	const double q = 1e-4;
	return VertKey{ (int64_t)std::llround((double)p.x / q),
	                (int64_t)std::llround((double)p.y / q),
	                (int64_t)std::llround((double)p.z / q) };
}
} // namespace

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
}

std::shared_ptr<hop::shape<hop_scalar>> HopShapeData::make_hop_shape(const Transform3D &p_local_xform) {
	// Split the local transform into a pure rotation (carried as the shape's static
	// local_rotation, which the narrowphase composes with the solid's orientation)
	// and the scale + translation, which is baked into the geometry (hop shapes
	// have no orientation of their own). Identity rotation is an exact no-op, so
	// existing axis-aligned content is unchanged.
	Basis rot_basis = Basis(p_local_xform.basis.get_rotation_quaternion());
	Transform3D geom_xform;
	// rot_basis is orthonormal, so its inverse is its transpose (exact + cheaper).
	geom_xform.basis = rot_basis.transposed() * p_local_xform.basis; // scale, rotation stripped
	geom_xform.origin = p_local_xform.origin;

	auto shape = build_shape_geometry(geom_xform);
	if (shape)
		shape->set_local_rotation(to_hop_mat3(rot_basis));
	return shape;
}

std::shared_ptr<hop::shape<hop_scalar>> HopShapeData::build_shape_geometry(const Transform3D &p_local_xform) {
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

			// Rotation-invariant thickness check (L3) to drop true coplanar 2D slivers
			// without misclassifying rotated 3D brush solids.
			if (thin == 0 && points.size() >= 4) {
				float max_d1_sq = 0.0f;
				int p0_idx = 0, pa_idx = 0;
				for (int i = 0; i < points.size(); ++i) {
					for (int j = i + 1; j < points.size(); ++j) {
						float d_sq = (points[i] - points[j]).length_squared();
						if (d_sq > max_d1_sq) {
							max_d1_sq = d_sq;
							p0_idx = i;
							pa_idx = j;
						}
					}
				}
				Vector3 p0 = points[p0_idx] + origin;
				Vector3 pa = points[pa_idx] + origin;
				Vector3 v1 = pa - p0;
				float l1 = std::sqrt(max_d1_sq);

				if (l1 > 1e-4f) {
					Vector3 u1 = v1 / l1;
					float max_d2_sq = 0.0f;
					int pb_idx = p0_idx;
					for (int i = 0; i < points.size(); ++i) {
						Vector3 w = (points[i] + origin) - p0;
						Vector3 perp = w - u1 * w.dot(u1);
						float d2_sq = perp.length_squared();
						if (d2_sq > max_d2_sq) {
							max_d2_sq = d2_sq;
							pb_idx = i;
						}
					}
					Vector3 pb = points[pb_idx] + origin;
					Vector3 v2 = pb - p0;
					Vector3 cross = v1.cross(v2);
					float c_len = cross.length();

					if (c_len > 1e-4f) {
						Vector3 norm = cross / c_len;
						float d3_min = 0.0f, d3_max = 0.0f;
						for (int i = 0; i < points.size(); ++i) {
							float dist = ((points[i] + origin) - p0).dot(norm);
							d3_min = std::min(d3_min, dist);
							d3_max = std::max(d3_max, dist);
						}
						float l3 = d3_max - d3_min;

						// Only drop if 3D thickness is below threshold (< 0.1m)
						if (l3 < MIN_DIM) {
							return nullptr;
						}
					}
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
			auto plane = std::make_unique<HopPlaneTraceable<hop_scalar>>();
			plane->set_plane(to_hop(p.normal), to_hop_scalar(p.d));
			return std::make_shared<hop::shape<hop_scalar>>(std::move(plane));
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

			// Deduplicate vertices and build index list. A spatial hash keyed on
			// the quantized position makes this O(N) — the old linear rescan was
			// O(N²) and took tens of seconds on large terrain/BSP meshes.
			std::vector<hop::vec3<hop_scalar>> verts;
			std::vector<HopTrimeshTraceable<hop_scalar>::triangle> tris;
			verts.reserve(face_count);
			tris.reserve(tri_count);
			std::unordered_map<VertKey, int, VertKeyHash> vert_index;
			vert_index.reserve(face_count);

			auto find_or_add = [&](const Vector3 &p) -> int {
				// Apply full transform (basis + origin) to vertices
				Vector3 wp = p_local_xform.xform(p);
				VertKey key = quantize_vert(wp);
				auto it = vert_index.find(key);
				if (it != vert_index.end())
					return it->second;
				int idx = (int)verts.size();
				verts.push_back(to_hop(wp));
				vert_index.emplace(key, idx);
				return idx;
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

			auto trimesh = std::make_unique<HopTrimeshTraceable<hop_scalar>>();
			trimesh->build(verts.data(), (int)verts.size(),
			                         tris.data(), (int)tris.size(),
			                         backface_collision);
			return std::make_shared<hop::shape<hop_scalar>>(std::move(trimesh));
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
			auto heightfield = std::make_unique<HopHeightfieldTraceable<hop_scalar>>();
			heightfield->build(w, dp, heights.ptr(), to_hop(gscale), to_hop(origin));
			return std::make_shared<hop::shape<hop_scalar>>(std::move(heightfield));
		}

		default:
			return nullptr;
	}
}
