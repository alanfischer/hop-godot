#pragma once

#include <hop/hop.h>
#include <vector>

#include "hop_triangle_collision.h"

// A traceable for a Godot HeightMapShape3D: a regular width×depth grid of
// heights. Unlike the trimesh traceable it stores no triangles and no BVH — the
// grid is implicit, so the broadphase is direct arithmetic (the cell range under
// a query's AABB) and each cell's two triangles are generated on the fly. This
// keeps a 1024² terrain at ~the height array's size instead of ~millions of
// triangles + a BVH. Per-triangle collision is the shared hoptri:: logic.
//
// Frame: heights/spacing are baked with the shape's transform scale at build
// time (hop solids carry no scale); the grid is centred on the shape origin,
// matching HeightMapShape3D. Queries arrive in solid-local space (caller passes
// the solid's world position); everything here works in that local frame.

template <typename T>
class HopHeightfieldTraceable : public hop::traceable<T> {
	using tr = hop::scalar_traits<T>;

public:
	// `heights` is row-major (depth rows of width), Godot's map_data order.
	// `scale` is the shape transform's per-axis scale; `origin` the shape's local
	// offset. Spacing is 1 unit per cell before scale.
	void build(int width, int depth, const float * heights,
	           const hop::vec3<T> & scale, const hop::vec3<T> & origin) {
		width_ = width;
		depth_ = depth;
		origin_ = origin;
		cell_x_ = scale.x;
		cell_z_ = scale.z;
		half_x_ = tr::from_int(width_ - 1) * tr::half();
		half_z_ = tr::from_int(depth_ - 1) * tr::half();

		heights_.resize(width_ * depth_);
		min_h_ = max_h_ = T {};
		for (int k = 0; k < width_ * depth_; ++k) {
			// Raw map_data height (along Y), scaled — exactly as Godot's
			// HeightMapShape3D interprets it. Any "ground at gray 0.5" convention
			// belongs in the terrain data / shape transform, not here.
			T h = to_hop_scalar(heights[k]) * scale.y;
			heights_[k] = h;
			if (k == 0 || h < min_h_) min_h_ = h;
			if (k == 0 || h > max_h_) max_h_ = h;
		}
	}

	void get_bound(hop::aa_box<T> & result) override {
		result.mins.x = origin_.x - half_x_ * cell_x_;
		result.maxs.x = origin_.x + half_x_ * cell_x_;
		result.mins.z = origin_.z - half_z_ * cell_z_;
		result.maxs.z = origin_.z + half_z_ * cell_z_;
		result.mins.y = origin_.y + min_h_;
		result.maxs.y = origin_.y + max_h_;
	}

	void trace_segment(hop::collision<T> & result,
	                   const hop::vec3<T> & position,
	                   const hop::mat3<T> & orientation,
	                   const hop::segment<T> & seg) override {
		// Transform the ray into grid-local space (the grid never moves). For a
		// rotated grid that's a rotate by Rᵀ about `position`, with the hit mapped
		// back by R; the common identity case is just the translate, with no matrix
		// math on the hot raycast path.
		static const hop::mat3<T> identity;
		const bool rotated = (orientation != identity);
		hop::segment<T> local;
		if (rotated) {
			hop::mat3<T> Rt;
			hop::transpose(Rt, orientation);
			hop::vec3<T> rel;
			hop::sub(rel, seg.origin, position);
			hop::mul(local.origin, Rt, rel);
			hop::mul(local.direction, Rt, seg.direction);
		} else {
			hop::sub(local.origin, seg.origin, position);
			local.direction = seg.direction;
		}

		// Walk only the cells the ray crosses (Amanatides–Woo over the XZ grid).
		dda_cells(local, [&](int i, int j) -> bool {
			hop::vec3<T> a, b, c, n, point, normal;
			for (int t = 0; t < 2; ++t) {
				cell_triangle(i, j, t, a, b, c, n);
				T th = hoptri::ray_triangle(local, a, b, c, n, point, normal);
				if (th < result.time) {
					result.time = th;
					if (rotated) {
						hop::mul(result.point, orientation, point);
						hop::add(result.point, position);
						hop::mul(result.normal, orientation, normal);
					} else {
						hop::add(result.point, point, position);
						result.normal = normal;
					}
				}
			}
			// DDA visits cells in increasing t, so the first cell with a hit holds
			// the nearest one — stop once we have it; keep walking while still clear.
			return result.time >= tr::one();
		});
	}

	void trace_solid(hop::collision<T> & result,
	                 hop::solid<T> * s,
	                 const hop::vec3<T> & position,
	                 const hop::mat3<T> & orientation,
	                 const hop::segment<T> & seg, T margin) override {
		static const hop::mat3<T> identity;
		if (orientation != identity) {
			// Rotated grid: the shared driver handles the frame transform + mover
			// spine reduction; we supply only the cell broadphase over the local AABB.
			hoptri::trace_solid_rotated(result, s, position, orientation, seg, margin, seam_tol_,
			    [&](const hop::aa_box<T> & q, auto && visit) {
				    int i0, i1, j0, j1;
				    if (!cell_range(q, i0, i1, j0, j1))
					    return;
				    for (int j = j0; j <= j1; ++j)
					    for (int i = i0; i <= i1; ++i) {
						    hop::vec3<T> a, b, c, n;
						    for (int k = 0; k < 2; ++k) {
							    cell_triangle(i, j, k, a, b, c, n);
							    visit(a, b, c, n);
						    }
					    }
			    });
			return;
		}
		// Swept query AABB in heightfield-local space (mirrors the trimesh path — see
		// there for why this is the solid's oriented bound and not a merge of raw shape
		// bounds, which are intrinsic and omit local_position/local_rotation).
		hop::aa_box<T> swept_box;
		s->get_bound_about_position(swept_box);

		hop::vec3<T> local_origin;
		hop::sub(local_origin, seg.origin, position);

		hop::aa_box<T> q;
		add_box(q, local_origin, swept_box);
		hop::vec3<T> end;
		hop::add(end, local_origin, seg.direction);
		hop::aa_box<T> end_box;
		add_box(end_box, end, swept_box);
		q.merge(end_box);

		int i0, i1, j0, j1;
		if (!cell_range(q, i0, i1, j0, j1))
			return;

		const bool is_static = (hop::dot(seg.direction, seg.direction) == T {});
		bool found = false;
		T best_depth = T {};

		for (int j = j0; j <= j1; ++j) {
			for (int i = i0; i <= i1; ++i) {
				hop::vec3<T> a, b, c, n;
				for (int t = 0; t < 2; ++t) {
					cell_triangle(i, j, t, a, b, c, n);
					for (const auto & sh_ptr : s->get_shapes()) {
						if (is_static)
							hoptri::recover_shape_vs_triangle(sh_ptr.get(), local_origin, seg, margin,
							    seam_tol_, a, b, c, n, result, found, best_depth);
						else
							hoptri::sweep_shape_vs_triangle(sh_ptr.get(), local_origin, seg, margin,
							    seam_tol_, a, b, c, n, result);
					}
				}
			}
		}
	}

private:
	int width_ = 0, depth_ = 0;
	std::vector<T> heights_;
	hop::vec3<T> origin_;
	T cell_x_ = tr::one(), cell_z_ = tr::one();
	T half_x_ {}, half_z_ {};
	T min_h_ {}, max_h_ {};
	// Seam tolerance for the non-capsule support-plane path (matches the trimesh).
	T seam_tol_ = tr::from_int(1) / tr::from_int(100);

	T height_at(int i, int j) const { return heights_[j * width_ + i]; }

	// Local-space position of grid point (i, j).
	hop::vec3<T> point_at(int i, int j) const {
		hop::vec3<T> p;
		p.x = origin_.x + (tr::from_int(i) - half_x_) * cell_x_;
		p.y = origin_.y + height_at(i, j);
		p.z = origin_.z + (tr::from_int(j) - half_z_) * cell_z_;
		return p;
	}

	// The `t`-th (0 or 1) triangle of cell (i, j), with an upward-oriented face
	// normal. Godot diagonal runs (i,j)→(i+1,j+1); winding gives +Y on flat ground.
	void cell_triangle(int i, int j, int t, hop::vec3<T> & a, hop::vec3<T> & b,
	                   hop::vec3<T> & c, hop::vec3<T> & n) const {
		if (t == 0) {
			a = point_at(i, j);
			b = point_at(i, j + 1);
			c = point_at(i + 1, j + 1);
		} else {
			a = point_at(i, j);
			b = point_at(i + 1, j + 1);
			c = point_at(i + 1, j);
		}
		hop::vec3<T> e1, e2;
		hop::sub(e1, b, a);
		hop::sub(e2, c, a);
		hop::cross(n, e1, e2);
		hop::normalize_carefully(n, T {});
	}

	static void add_box(hop::aa_box<T> & out, const hop::vec3<T> & o, const hop::aa_box<T> & b) {
		out.mins.x = o.x + b.mins.x; out.mins.y = o.y + b.mins.y; out.mins.z = o.z + b.mins.z;
		out.maxs.x = o.x + b.maxs.x; out.maxs.y = o.y + b.maxs.y; out.maxs.z = o.z + b.maxs.z;
	}

	// Continuous grid coords of a local x/z.
	T grid_i(T x) const { return (x - origin_.x) / cell_x_ + half_x_; }
	T grid_j(T z) const { return (z - origin_.z) / cell_z_ + half_z_; }

	static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

	// Cell index range [i0..i1]×[j0..j1] covered by a local-space AABB. Returns
	// false if the AABB is entirely off the grid.
	bool cell_range(const hop::aa_box<T> & box, int & i0, int & i1, int & j0, int & j1) const {
		if (width_ < 2 || depth_ < 2)
			return false;
		i0 = clampi(tr::to_int(grid_i(box.mins.x)), 0, width_ - 2);
		i1 = clampi(tr::to_int(grid_i(box.maxs.x)), 0, width_ - 2);
		j0 = clampi(tr::to_int(grid_j(box.mins.z)), 0, depth_ - 2);
		j1 = clampi(tr::to_int(grid_j(box.maxs.z)), 0, depth_ - 2);
		// Off-grid rejection: if the box is wholly left/right/front/back of the grid
		// the clamped range is still valid (a 1-cell edge); accept — the per-cell
		// triangle test will simply not hit. (Cheap and avoids a separate overlap test.)
		return true;
	}

	// Amanatides–Woo DDA over the XZ grid; calls cb(i, j) per crossed cell while it
	// returns true. Operates in continuous grid coordinates.
	template <typename CB>
	void dda_cells(const hop::segment<T> & local, CB && cb) const {
		if (width_ < 2 || depth_ < 2)
			return;
		T gi = grid_i(local.origin.x);
		T gj = grid_j(local.origin.z);
		T di = local.direction.x / cell_x_; // grid units per unit t
		T dj = local.direction.z / cell_z_;

		const T zero {};
		const T gmax_i = tr::from_int(width_ - 1);
		const T gmax_j = tr::from_int(depth_ - 1);

		// Clip the ray's t-range to the grid rectangle [0,width-1]×[0,depth-1].
		T t_enter = zero, t_exit = tr::one();
		auto clip = [&](T p, T d, T lo, T hi) -> bool {
			if (d > -tr::from_milli(1) && d < tr::from_milli(1)) // ~parallel
				return p >= lo && p <= hi;
			T t1 = (lo - p) / d, t2 = (hi - p) / d;
			if (t1 > t2) { T tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > t_enter) t_enter = t1;
			if (t2 < t_exit) t_exit = t2;
			return t_enter <= t_exit;
		};
		if (!clip(gi, di, zero, gmax_i)) return;
		if (!clip(gj, dj, zero, gmax_j)) return;
		if (t_enter < zero) t_enter = zero;

		// Entry point in grid coords.
		T ci = gi + di * t_enter;
		T cj = gj + dj * t_enter;
		int i = clampi(tr::to_int(ci), 0, width_ - 2);
		int j = clampi(tr::to_int(cj), 0, depth_ - 2);

		int step_i = di > zero ? 1 : -1;
		int step_j = dj > zero ? 1 : -1;

		// t to first boundary, and t per cell crossing, for each axis.
		auto axis = [&](T c, T d, int idx, int step, T & t_next, T & t_delta) {
			if (d > -tr::from_milli(1) && d < tr::from_milli(1)) {
				t_next = tr::one() * tr::from_int(1000); // effectively never
				t_delta = t_next;
				return;
			}
			T next_boundary = (step > 0) ? tr::from_int(idx + 1) : tr::from_int(idx);
			t_next = t_enter + (next_boundary - (c + d * t_enter)) / d;
			T ad = d < zero ? -d : d; // 1 grid cell per |d| units of t
			t_delta = tr::one() / ad;
		};
		T t_next_i, t_delta_i, t_next_j, t_delta_j;
		axis(gi, di, i, step_i, t_next_i, t_delta_i);
		axis(gj, dj, j, step_j, t_next_j, t_delta_j);

		const int max_steps = (width_ + depth_) * 2;
		for (int s = 0; s < max_steps; ++s) {
			if (i < 0 || i >= width_ - 1 || j < 0 || j >= depth_ - 1)
				return;
			if (!cb(i, j))
				return;
			if (t_next_i < t_next_j) {
				if (t_next_i > t_exit) return;
				i += step_i;
				t_next_i += t_delta_i;
			} else {
				if (t_next_j > t_exit) return;
				j += step_j;
				t_next_j += t_delta_j;
			}
		}
	}
};
