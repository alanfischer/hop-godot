#pragma once

#include <hop/hop.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "hop_bsp_format.h"

// A traceable implementation for GoldSrc BSP hulls.
//
// GoldSrc precomputes four collision trees per model: hull 0 (the point hull,
// stored as nodes + leafs) and hulls 1..3 (clipnodes), each already Minkowski-
// expanded for one fixed box size. A mover is therefore reduced to a POINT and
// swept through whichever hull matches its bounding box — that is what gives
// GoldSrc its exact plane normals and true 18-unit steps, with no trimesh seam
// jitter.
//
// The descent runs in native GoldSrc space (Z-up, inches) against the planes
// exactly as they sit in the file. The Godot-space query is transformed in and
// the result transformed back out; no plane is ever pre-baked, so no float error
// accumulates across a map's thousands of planes.

namespace hopbsp {

// Trace crosspoints are placed this far on the near side of the plane, so a
// trace that stops on a surface never ends up microscopically inside it. Quake's
// DIST_EPSILON, in GoldSrc units.
inline constexpr double DIST_EPSILON = 0.03125;

// Engine-baked hull box sizes (Half-Life). These live in the engine, not the
// file: the compiler expanded the clipnode trees for exactly these boxes, so a
// consumer has to know them to pick a hull and offset the traced point.
struct hull_size {
	double mins[3];
	double maxs[3];
};
inline constexpr hull_size HULL_SIZES[4] = {
	{ {   0,   0,   0 }, {  0,  0,  0 } },  // 0 — point hull
	{ { -16, -16, -36 }, { 16, 16, 36 } },  // 1 — standing player (32x32x72)
	{ { -32, -32, -32 }, { 32, 32, 32 } },  // 2 — large monster (64x64x64)
	{ { -16, -16, -18 }, { 16, 16, 18 } },  // 3 — crouched player (32x32x36)
};

// GoldSrc's SV_HullForEntity sizing rule, on a mover's GoldSrc-space box size.
inline int hull_for_size(double sx, double sz) {
	if (sx <= 8.0) return 0;
	if (sx <= 36.0) return (sz <= 36.0) ? 3 : 1;
	return 2;
}

// One hull's tree. hull 0 descends `nodes` and reads contents out of `leafs`;
// hulls 1..3 descend `clipnodes`, whose negative children ARE the contents.
// Both are addressed through child(), so the trace is written once.
struct hull {
	const hop_bsp::BSPPlane *planes = nullptr;
	const hop_bsp::BSPNode *nodes = nullptr;
	const hop_bsp::BSPLeaf *leafs = nullptr;
	const hop_bsp::BSPClipNode *clipnodes = nullptr;
	int node_count = 0;   // entries in whichever tree array is live
	int leaf_count = 0;
	int plane_count = 0;
	int root = 0;
	int index = 0;        // 0..3, selects HULL_SIZES

	bool valid() const { return planes != nullptr && (nodes != nullptr || clipnodes != nullptr); }

	int planenum(int num) const {
		return nodes ? nodes[num].planenum : clipnodes[num].planenum;
	}

	// Child `side` of node `num`. >= 0 is another node; < 0 is a contents value.
	int child(int num, int side) const {
		if (clipnodes) return clipnodes[num].children[side];
		int c = nodes[num].children[side];
		if (c >= 0) return c;
		int leaf = -1 - c;
		if (leaf < 0 || leaf >= leaf_count) return hop_bsp::CONTENTS_SOLID;
		return (int)leafs[leaf].contents;
	}
};

// Which contents values stop a trace, as a bitmask over -contents (SOLID = -2 →
// bit 2). GoldSrc blocks on CONTENTS_SOLID alone; sky, water, slime and lava are
// passable, which is exactly why a bullet flies through a sky brush on hull 0
// while a player still collides with it on hull 1 (the compiler writes sky as
// solid in the clipnode trees). Exposed so a caller can build, say, a
// sky-blocking or water-blocking view of the same map.
inline constexpr int blocking_bit(int contents) {
	// Contents are small negatives (-1..-15). Anything outside that is corruption;
	// bit 0 is unused by any real contents value, so it never matches a mask.
	return (contents >= 0 || contents < -30) ? 1 : (1 << (-contents));
}
inline constexpr int BLOCK_SOLID = blocking_bit(hop_bsp::CONTENTS_SOLID);

struct hull_trace {
	double fraction = 1.0;
	double endpos[3] = { 0, 0, 0 };
	double normal[3] = { 0, 0, 0 };
	double dist = 0.0;
	bool allsolid = true;    // every point along the trace was inside blocking contents
	bool startsolid = false; // the trace started inside blocking contents
	bool hit = false;        // fraction < 1 and `normal` is meaningful
	int start_contents = hop_bsp::CONTENTS_EMPTY;
};

// Signed distance of `p` from a plane, with the whole hull inflated outward by
// `margin`. Solid is the back side (d < 0), so pushing `dist` out by the margin
// grows the solid region — exact on faces, rounding the convex corners off. That
// bias is applied at read time rather than by copying the plane array: a map has
// tens of thousands of planes and hundreds of brush entities.
inline double plane_dist(const hop_bsp::BSPPlane &pl, const double p[3], double margin) {
	double d = (pl.type < 3)
		? p[pl.type] - pl.dist
		: pl.normal[0] * p[0] + pl.normal[1] * p[1] + pl.normal[2] * p[2] - pl.dist;
	return d - margin;
}

// Contents at point `p` (GoldSrc space), descending from node `num`.
inline int hull_point_contents(const hull &h, int num, const double p[3], double margin = 0) {
	while (num >= 0) {
		if (num >= h.node_count) return hop_bsp::CONTENTS_SOLID;
		double d = plane_dist(h.planes[h.planenum(num)], p, margin);
		num = h.child(num, d < 0 ? 1 : 0);
	}
	return num;
}

// The nearest bounding plane of the leaf containing `p`, as an outward push-out
// (`normal`, `depth`). BSP leaves are convex, so the splitting plane crossed with
// the smallest margin during the descent IS the closest surface. Returns false if
// `p` isn't inside blocking contents.
inline bool hull_push_out(const hull &h, int num, const double p[3], int blocking,
                          double normal[3], double &depth, double margin = 0) {
	double best = 1e30;
	double best_n[3] = { 0, 0, 1 };
	while (num >= 0) {
		if (num >= h.node_count) return false;
		const hop_bsp::BSPPlane &pl = h.planes[h.planenum(num)];
		double d = plane_dist(pl, p, margin);
		// best_n is the way OUT through this plane: descending to the back means
		// the plane bounds the cell from above, so out is +normal; to the front
		// means it bounds from below, so out is -normal.
		double mag = d < 0 ? -d : d;
		if (mag < best) {
			best = mag;
			double s = d < 0 ? 1.0 : -1.0;
			best_n[0] = pl.normal[0] * s;
			best_n[1] = pl.normal[1] * s;
			best_n[2] = pl.normal[2] * s;
		}
		num = h.child(num, d < 0 ? 1 : 0);
	}
	if ((blocking & blocking_bit(num)) == 0) return false;
	normal[0] = best_n[0];
	normal[1] = best_n[1];
	normal[2] = best_n[2];
	depth = best;
	return true;
}

// Quake's SV_RecursiveHullCheck. Sweeps the point p1→p2 (GoldSrc space) through
// the hull, splitting the segment at every plane crossing. Returns false once the
// trace has been stopped.
inline bool recursive_hull_check(const hull &h, int num, double p1f, double p2f,
                                 const double p1[3], const double p2[3],
                                 int blocking, hull_trace &tr, double margin = 0) {
	if (num < 0) {
		if ((blocking & blocking_bit(num)) == 0) {
			tr.allsolid = false;
		} else {
			tr.startsolid = true;
		}
		return true;
	}
	if (num >= h.node_count) return false;

	const hop_bsp::BSPPlane &pl = h.planes[h.planenum(num)];
	const double t1 = plane_dist(pl, p1, margin);
	const double t2 = plane_dist(pl, p2, margin);

	if (t1 >= 0 && t2 >= 0) return recursive_hull_check(h, h.child(num, 0), p1f, p2f, p1, p2, blocking, tr, margin);
	if (t1 < 0 && t2 < 0)   return recursive_hull_check(h, h.child(num, 1), p1f, p2f, p1, p2, blocking, tr, margin);

	// Split, keeping the crosspoint DIST_EPSILON on the near side of the plane.
	double frac = (t1 < 0) ? (t1 + DIST_EPSILON) / (t1 - t2)
	                       : (t1 - DIST_EPSILON) / (t1 - t2);
	if (frac < 0) frac = 0;
	if (frac > 1) frac = 1;

	double midf = p1f + (p2f - p1f) * frac;
	double mid[3];
	for (int i = 0; i < 3; ++i) mid[i] = p1[i] + frac * (p2[i] - p1[i]);

	int side = (t1 < 0) ? 1 : 0;

	if (!recursive_hull_check(h, h.child(num, side), p1f, midf, p1, mid, blocking, tr, margin))
		return false;

	int far_contents = hull_point_contents(h, h.child(num, side ^ 1), mid, margin);
	if ((blocking & blocking_bit(far_contents)) == 0)
		return recursive_hull_check(h, h.child(num, side ^ 1), midf, p2f, mid, p2, blocking, tr, margin);

	if (tr.allsolid) return false;  // never got out of the solid area

	// The far side is blocking: this is the impact. The normal faces back along
	// the side we approached from.
	if (side == 0) {
		tr.normal[0] = pl.normal[0]; tr.normal[1] = pl.normal[1]; tr.normal[2] = pl.normal[2];
		tr.dist = pl.dist;
	} else {
		tr.normal[0] = -pl.normal[0]; tr.normal[1] = -pl.normal[1]; tr.normal[2] = -pl.normal[2];
		tr.dist = -pl.dist;
	}
	tr.hit = true;

	// Back up if the crosspoint still landed inside solid — rare, but the epsilon
	// nudge can't save every degenerate plane pair.
	while ((blocking & blocking_bit(hull_point_contents(h, h.root, mid, margin))) != 0) {
		frac -= 0.1;
		if (frac < 0) {
			tr.fraction = midf;
			for (int i = 0; i < 3; ++i) tr.endpos[i] = mid[i];
			return false;
		}
		midf = p1f + (p2f - p1f) * frac;
		for (int i = 0; i < 3; ++i) mid[i] = p1[i] + frac * (p2[i] - p1[i]);
	}

	tr.fraction = midf;
	for (int i = 0; i < 3; ++i) tr.endpos[i] = mid[i];
	return false;
}

// Full point sweep through one hull, GoldSrc space in and out.
inline hull_trace hull_sweep(const hull &h, const double start[3], const double end[3],
                             int blocking = BLOCK_SOLID, double margin = 0) {
	hull_trace tr;
	for (int i = 0; i < 3; ++i) tr.endpos[i] = end[i];
	if (!h.valid()) { tr.allsolid = false; return tr; }
	tr.start_contents = hull_point_contents(h, h.root, start, margin);
	recursive_hull_check(h, h.root, 0.0, 1.0, start, end, blocking, tr, margin);
	if (tr.fraction == 1.0) for (int i = 0; i < 3; ++i) tr.endpos[i] = end[i];
	return tr;
}

// One map's stripped BSP, owned once and shared by every traceable built from it.
// A map has hundreds of brush entities and the blob runs to megabytes, so this is
// never copied per body. hop owns the bytes — it never points into another
// extension's parse, which could be freed underneath it.
struct map_data {
	std::vector<uint8_t> bytes;
	hop_bsp::BlobView view;

	bool load(const uint8_t *blob, size_t size) {
		bytes.assign(blob, blob + size);
		if (!hop_bsp::parse_blob(bytes.data(), bytes.size(), view)) { bytes.clear(); return false; }
		return true;
	}
	bool valid() const { return view.valid(); }
};

} // namespace hopbsp

// ---------------------------------------------------------------------------

template <typename T>
class HopBspTraceable : public hop::traceable<T> {
	using tr = hop::scalar_traits<T>;

public:
	// `scale` is the importer's GoldSrc-units → metres factor.
	bool build(std::shared_ptr<hopbsp::map_data> map, int model_index, T scale,
	           int blocking = hopbsp::BLOCK_SOLID) {
		map_ = std::move(map);
		blocking_ = blocking;
		scale_ = (double)scale;
		inv_scale_ = scale_ != 0.0 ? 1.0 / scale_ : 0.0;
		if (!map_ || !map_->valid()) { map_.reset(); return false; }
		view_ = map_->view;
		if (model_index < 0 || model_index >= view_.model_count) { map_.reset(); return false; }
		model_ = &view_.models[model_index];

		for (int i = 0; i < 4; ++i) {
			hulls_[i].planes = view_.planes;
			hulls_[i].plane_count = view_.plane_count;
			hulls_[i].leafs = view_.leafs;
			hulls_[i].leaf_count = view_.leaf_count;
			hulls_[i].root = model_->headnode[i];
			hulls_[i].index = i;
			if (i == 0) {
				hulls_[i].nodes = view_.nodes;
				hulls_[i].node_count = view_.node_count;
			} else {
				hulls_[i].clipnodes = view_.clipnodes;
				hulls_[i].node_count = view_.clipnode_count;
			}
		}

		// Godot-space bound from the model's GoldSrc box, grown by the largest hull
		// so a mover reduced to a point is still inside the bound where it matters.
		hop::vec3<T> a = gs_to_godot(model_->mins[0] - 32, model_->mins[1] - 32, model_->mins[2] - 36);
		hop::vec3<T> b = gs_to_godot(model_->maxs[0] + 32, model_->maxs[1] + 32, model_->maxs[2] + 36);
		bound_.mins.set(tr::min_val(a.x, b.x), tr::min_val(a.y, b.y), tr::min_val(a.z, b.z));
		bound_.maxs.set(tr::max_val(a.x, b.x), tr::max_val(a.y, b.y), tr::max_val(a.z, b.z));
		return true;
	}

	// Convenience for callers holding raw bytes (tests): loads a private copy.
	bool build(const uint8_t *blob, size_t size, int model_index, T scale,
	           int blocking = hopbsp::BLOCK_SOLID) {
		auto map = std::make_shared<hopbsp::map_data>();
		if (!map->load(blob, size)) return false;
		return build(std::move(map), model_index, scale, blocking);
	}

	bool is_built() const { return model_ != nullptr; }

	void get_bound(hop::aa_box<T> &result) override { result = bound_; }

	void trace_segment(hop::collision<T> &result,
	                   const hop::vec3<T> &position,
	                   const hop::mat3<T> &orientation,
	                   const hop::segment<T> &seg) override {
		if (model_ == nullptr) return;
		hop::vec3<T> lo, ld;
		to_local(seg.origin, seg.direction, position, orientation, lo, ld);

		double s[3], e[3];
		godot_to_gs(lo, s);
		hop::vec3<T> lend;
		hop::add(lend, lo, ld);
		godot_to_gs(lend, e);

		// A ray is a point mover: hull 0, no offset.
		hopbsp::hull_trace ht = hopbsp::hull_sweep(hulls_[0], s, e, blocking_);
		if (!ht.hit || (T)ht.fraction >= result.time) return;

		result.time = (T)ht.fraction;
		hop::vec3<T> hit_local = gs_to_godot(ht.endpos[0], ht.endpos[1], ht.endpos[2]);
		hop::vec3<T> n_local = gs_dir_to_godot(ht.normal[0], ht.normal[1], ht.normal[2]);
		to_world(hit_local, n_local, position, orientation, result.point, result.normal);
		result.impact = result.point;
	}

	void trace_solid(hop::collision<T> &result,
	                 hop::solid<T> *s,
	                 const hop::vec3<T> &position,
	                 const hop::mat3<T> &orientation,
	                 const hop::segment<T> &seg, T margin) override {
		if (model_ == nullptr) return;

		// The mover's own box, in GoldSrc units, picks the hull. Its offset is what
		// turns the mover into a point: GoldSrc traces (origin + clip_mins - mins).
		double mins[3], maxs[3];
		solid_box_gs(s, mins, maxs);
		const int hi = hopbsp::hull_for_size(maxs[0] - mins[0], maxs[2] - mins[2]);
		const hopbsp::hull &h = hulls_[hi];
		if (!h.valid()) return;
		// Reduce the mover to the point the hull was expanded around. GoldSrc traces
		// (origin + mins - clip_mins): the hull is built for a box whose origin sits
		// at clip_mins from its low corner, so a mover whose origin sits elsewhere in
		// its own box has to be shifted by the difference.
		//
		// This is ZERO for a centre-origin box, which is what makes it easy to get
		// backwards and never notice — GoldSrc's own player is centre-origin. Godot
		// bodies are not: WizardWars offsets the player's collider upward so the
		// origin is at the feet, and with the sign flipped the trace point lands a
		// full half-height BELOW the feet, i.e. inside the floor, every frame.
		double offset[3];
		for (int i = 0; i < 3; ++i) offset[i] = mins[i] - hopbsp::HULL_SIZES[hi].mins[i];

		hop::vec3<T> lo, ld;
		to_local(seg.origin, seg.direction, position, orientation, lo, ld);

		double start[3], end[3];
		godot_to_gs(lo, start);
		hop::vec3<T> lend;
		hop::add(lend, lo, ld);
		godot_to_gs(lend, end);
		for (int i = 0; i < 3; ++i) { start[i] += offset[i]; end[i] += offset[i]; }

		// `margin` inflates the hull outward so a near-resting mover registers as a
		// contact. Pushing every plane out by it is exact on faces and rounds the
		// convex corners off — the accepted approximation for speculative contacts.
		const double margin_gs = (double)margin * inv_scale_;

		const bool zero_dir = hop::length_squared(ld) == T {};

		// A mover that STARTS inside solid is reported as an overlap, whatever its
		// direction. Quake instead reports startsolid and leaves the trace clear,
		// because its callers check that flag and refuse the move — hop's caller has
		// no such flag, so "clear" reads as "the whole path is free" and the mover
		// sails through the wall it is embedded in. Standing on a floor and sinking
		// a hair into the expanded hull is enough to start it: each frame gravity
		// sweeps freely, sinking it deeper, until it drops out of the world. That is
		// exactly what happened to bots on ww_2fort's ramps.
		//
		// Resting exactly ON the surface is not inside it (contents test is d < 0),
		// so this costs the normal standing case nothing.
		double n[3], depth = 0;
		double probe[3] = { start[0], start[1], start[2] };
		// Margin 0 on purpose: only REAL penetration counts as stuck. Testing
		// against the inflated hull would call a mover merely within `margin` of a
		// wall stuck and stall it dead.
		const bool inside = hopbsp::hull_push_out(h, h.root, probe, blocking_, n, depth);
		if (inside || zero_dir) {
			if (inside) {
				depth += margin_gs;
			} else {
				if (margin_gs <= 0) return;
				// Not inside, but a face within `margin` still counts as a contact:
				// sweeping a probe along every candidate normal is overkill, so use
				// the six axes and take the nearest surface.
				if (!nearest_surface(h, probe, margin_gs, n, depth)) return;
				depth = margin_gs - depth;
				if (depth <= 0) return;
			}
			if (T {} >= result.time) return;
			result.time = T {};
			result.depth = (T)(depth * scale_);
			hop::vec3<T> n_local = gs_dir_to_godot(n[0], n[1], n[2]);
			hop::vec3<T> p_local = gs_to_godot(probe[0] - offset[0], probe[1] - offset[1], probe[2] - offset[2]);
			to_world(p_local, n_local, position, orientation, result.point, result.normal);
			result.impact = result.point;
			return;
		}

		hopbsp::hull_trace ht = hopbsp::hull_sweep(h, start, end, blocking_, margin_gs);
		if (!ht.hit || (T)ht.fraction >= result.time) return;

		result.time = (T)ht.fraction;
		result.depth = T {};
		hop::vec3<T> n_local = gs_dir_to_godot(ht.normal[0], ht.normal[1], ht.normal[2]);
		// endpos is where the POINT stopped; undo the hull offset to get the mover's
		// origin, and place the witness point on the surface it stopped against.
		hop::vec3<T> p_local = gs_to_godot(ht.endpos[0] - offset[0], ht.endpos[1] - offset[1], ht.endpos[2] - offset[2]);
		to_world(p_local, n_local, position, orientation, result.point, result.normal);

		// The witness point: endpos sits on the EXPANDED surface, so walk back by
		// the hull box's own extent along the normal to land on the real geometry.
		// Exact on a face, off by the box corner on a bevel — the same
		// approximation the expanded hulls are built on.
		double w[3];
		for (int i = 0; i < 3; ++i) {
			w[i] = ht.endpos[i] + (ht.normal[i] > 0 ? hopbsp::HULL_SIZES[hi].mins[i]
			                                        : hopbsp::HULL_SIZES[hi].maxs[i]);
		}
		hop::vec3<T> impact_local = gs_to_godot(w[0], w[1], w[2]);
		hop::vec3<T> ignored;
		to_world(impact_local, n_local, position, orientation, result.impact, ignored);
	}

	// --- helpers exposed for tests -----------------------------------------

	const hopbsp::hull &get_hull(int i) const { return hulls_[i]; }
	int contents_at_godot(const hop::vec3<T> &p) const {
		double gs[3];
		godot_to_gs(p, gs);
		return hopbsp::hull_point_contents(hulls_[0], hulls_[0].root, gs);
	}

private:
	// GoldSrc (Z-up, inches) ↔ Godot (Y-up, metres). The axis swap is its own
	// inverse and has determinant +1, so directions use the same map both ways and
	// normals need no special handling.
	hop::vec3<T> gs_to_godot(double x, double y, double z) const {
		hop::vec3<T> v;
		v.set((T)(-x * scale_), (T)(z * scale_), (T)(y * scale_));
		return v;
	}
	hop::vec3<T> gs_dir_to_godot(double x, double y, double z) const {
		hop::vec3<T> v;
		v.set((T)(-x), (T)z, (T)y);
		return v;
	}
	void godot_to_gs(const hop::vec3<T> &v, double out[3]) const {
		out[0] = -(double)v.x * inv_scale_;
		out[1] = (double)v.z * inv_scale_;
		out[2] = (double)v.y * inv_scale_;
	}

	void to_local(const hop::vec3<T> &origin, const hop::vec3<T> &dir,
	              const hop::vec3<T> &position, const hop::mat3<T> &orientation,
	              hop::vec3<T> &lo, hop::vec3<T> &ld) const {
		static const hop::mat3<T> identity;
		hop::vec3<T> rel;
		hop::sub(rel, origin, position);
		if (orientation != identity) {
			hop::mat3<T> Rt;
			hop::transpose(Rt, orientation);
			hop::mul(lo, Rt, rel);
			hop::mul(ld, Rt, dir);
		} else {
			lo = rel;
			ld = dir;
		}
	}

	void to_world(const hop::vec3<T> &p_local, const hop::vec3<T> &n_local,
	              const hop::vec3<T> &position, const hop::mat3<T> &orientation,
	              hop::vec3<T> &p_out, hop::vec3<T> &n_out) const {
		static const hop::mat3<T> identity;
		if (orientation != identity) {
			hop::mul(p_out, orientation, p_local);
			hop::add(p_out, position);
			hop::mul(n_out, orientation, n_local);
		} else {
			hop::add(p_out, p_local, position);
			n_out = n_local;
		}
	}

	// Union of the mover's shape bounds, converted to a GoldSrc-space box.
	void solid_box_gs(hop::solid<T> *s, double mins[3], double maxs[3]) const {
		hop::aa_box<T> box;
		box.mins.set(T {}, T {}, T {});
		box.maxs.set(T {}, T {}, T {});
		bool first = true;
		if (s != nullptr) {
			for (const auto &sh : s->get_shapes()) {
				hop::aa_box<T> b;
				sh->get_bound(b);
				if (first) { box = b; first = false; }
				else box.merge(b);
			}
		}
		// Axis swap maps godot X to -gs X, so the box ends swap on that axis.
		double lo[3], hi[3];
		lo[0] = -(double)box.maxs.x * inv_scale_; hi[0] = -(double)box.mins.x * inv_scale_;
		lo[1] = (double)box.mins.z * inv_scale_;  hi[1] = (double)box.maxs.z * inv_scale_;
		lo[2] = (double)box.mins.y * inv_scale_;  hi[2] = (double)box.maxs.y * inv_scale_;
		for (int i = 0; i < 3; ++i) { mins[i] = lo[i]; maxs[i] = hi[i]; }
	}

	// Nearest blocking surface within `limit` of `p`, by six axis probes. Only used
	// for the speculative (margin, not penetrating) contact case.
	bool nearest_surface(const hopbsp::hull &h, const double p[3], double limit,
	                     double n[3], double &gap) const {
		static const double axes[6][3] = {
			{ 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
		};
		bool found = false;
		gap = limit;
		for (const auto &a : axes) {
			double e[3] = { p[0] + a[0] * limit, p[1] + a[1] * limit, p[2] + a[2] * limit };
			hopbsp::hull_trace t = hopbsp::hull_sweep(h, p, e, blocking_);
			if (!t.hit) continue;
			double d = t.fraction * limit;
			if (d < gap) {
				gap = d;
				n[0] = t.normal[0]; n[1] = t.normal[1]; n[2] = t.normal[2];
				found = true;
			}
		}
		return found;
	}

	std::shared_ptr<hopbsp::map_data> map_;  // keeps the shared bytes alive
	hop_bsp::BlobView view_;
	const hop_bsp::BSPModel *model_ = nullptr;
	hopbsp::hull hulls_[4];
	hop::aa_box<T> bound_;
	int blocking_ = hopbsp::BLOCK_SOLID;
	double scale_ = 1.0;
	double inv_scale_ = 1.0;
};
