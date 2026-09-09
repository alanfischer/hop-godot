#pragma once
// Hand-authored stripped BSP30 blobs, shared by the traceable tests and the bench.
//
// The byte layout here is round-trip-checked against hop_bsp_format.h every time a
// test runs, so it lives in one place: a second copy drifts silently the first time
// the format changes.
//
// Expects the includer to have defined `T` (the scalar) and `V` (hop::vec3<T>), and
// to have included <hop/hop.h> and "hop_bsp_traceable.h".

inline const double SCALE = 0.025;  // WizardWars' GameConsts.SCALE_FACTOR

inline V vec(T x, T y, T z) { V v; v.set(x, y, z); return v; }

inline bool approx(T a, T b, T tol = 1e-3) { return std::fabs(a - b) < tol; }
inline bool approx_v(const V &v, T x, T y, T z, T tol = 1e-3) {
	return approx(v.x, x, tol) && approx(v.y, y, tol) && approx(v.z, z, tol);
}


// --- synthetic blob authoring ---------------------------------------------

struct BlobBuilder {
	std::vector<BSPPlane> planes;
	std::vector<BSPNode> nodes;
	std::vector<BSPClipNode> clipnodes;
	std::vector<BSPLeaf> leafs;
	std::vector<BSPModel> models;

	std::vector<uint8_t> build() const {
		std::vector<uint8_t> out(sizeof(BSPHeader), 0);
		BSPHeader hdr {};
		hdr.version = HLBSP_VERSION;
		auto put = [&](int idx, const void *data, size_t sz) {
			if (sz == 0) return;
			hdr.lumps[idx].fileofs = (int32_t)out.size();
			hdr.lumps[idx].filelen = (int32_t)sz;
			const uint8_t *p = (const uint8_t *)data;
			out.insert(out.end(), p, p + sz);
		};
		put(LUMP_PLANES, planes.data(), planes.size() * sizeof(BSPPlane));
		put(LUMP_NODES, nodes.data(), nodes.size() * sizeof(BSPNode));
		put(LUMP_CLIPNODES, clipnodes.data(), clipnodes.size() * sizeof(BSPClipNode));
		put(LUMP_LEAFS, leafs.data(), leafs.size() * sizeof(BSPLeaf));
		put(LUMP_MODELS, models.data(), models.size() * sizeof(BSPModel));
		memcpy(out.data(), &hdr, sizeof(BSPHeader));
		return out;
	}
};

// `oblique` tilts the box's side planes off-axis so they miss plane_offset's
// `type < 3` fast path — the two together bracket a real map, whose planes are
// mostly but not entirely axial.
inline int add_box_planes(BlobBuilder &b, const double mins[3], const double maxs[3],
                          bool oblique = false) {
	int first = (int)b.planes.size();
	for (int axis = 0; axis < 3; ++axis) {
		for (int hi = 0; hi < 2; ++hi) {
			BSPPlane p {};
			if (oblique && axis != 2) {
				// Tilt within the horizontal plane, normalized, so the brush stays
				// convex and the distance stays meaningful. Floors/ceilings stay axial:
				// a map's Z planes overwhelmingly are.
				const double a = 0.9689, c = 0.2474;  // ~14 degrees, unit length
				p.normal[axis] = (float)a;
				p.normal[1 - axis] = (float)c;
				p.dist = (float)((hi ? maxs[axis] : mins[axis]) * a);
				p.type = 3;  // non-axial: forces the dot-product path
			} else {
				p.normal[axis] = 1.0f;
				p.dist = (float)(hi ? maxs[axis] : mins[axis]);
				p.type = axis;
			}
			b.planes.push_back(p);
		}
	}
	return first;
}

inline int add_box_brush(BlobBuilder &b, const double mins[3], const double maxs[3],
                         bool as_nodes, int contents = CONTENTS_SOLID,
                         int outside = 1, bool oblique = false) {
	const int p0 = add_box_planes(b, mins, maxs, oblique);
	const int base = as_nodes ? (int)b.nodes.size() : (int)b.clipnodes.size();
	if (as_nodes && b.leafs.empty()) {
		BSPLeaf solid {}; solid.contents = contents;
		BSPLeaf empty {}; empty.contents = CONTENTS_EMPTY;
		b.leafs.push_back(solid);
		b.leafs.push_back(empty);
	}
	const int SOLID_CHILD = as_nodes ? -1 : contents;
	const int EMPTY_CHILD = outside != 1 ? outside : (as_nodes ? -2 : CONTENTS_EMPTY);
	for (int i = 0; i < 6; ++i) {
		const bool upper = (i % 2) == 1;
		const int inward = (i == 5) ? SOLID_CHILD : (base + i + 1);
		int child0, child1;
		if (upper) { child0 = EMPTY_CHILD; child1 = inward; }
		else       { child0 = inward;      child1 = EMPTY_CHILD; }
		if (as_nodes) {
			BSPNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.nodes.push_back(n);
		} else {
			BSPClipNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.clipnodes.push_back(n);
		}
	}
	return base;
}

// One model whose hull 0 is a box brush and whose hulls 1..3 are that same brush
// expanded by each engine hull size — exactly what the map compiler bakes.
inline std::vector<uint8_t> make_box_map(const double mins[3], const double maxs[3],
                                         int contents = CONTENTS_SOLID) {
	BlobBuilder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) { m.mins[i] = (float)mins[i]; m.maxs[i] = (float)maxs[i]; }

	m.headnode[0] = 0;
	add_box_brush(b, mins, maxs, /*as_nodes=*/true, contents);

	for (int h = 1; h < 4; ++h) {
		double emins[3], emaxs[3];
		for (int i = 0; i < 3; ++i) {
			emins[i] = mins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			emaxs[i] = maxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		m.headnode[h] = (int32_t)b.clipnodes.size();
		add_box_brush(b, emins, emaxs, /*as_nodes=*/false, contents);
	}
	b.models.push_back(m);
	return b.build();
}

// A floor with a wall standing on it, as a union of two brushes, sized so that in
// hull 1 the wall's solid begins exactly where the floor's ends (z = 36 — the floor
// top at 0, raised by the hull's 36-unit half-height).
//
// That seam is not contrived: it is where every standing player's trace point lives.
// A hull-1 mover is traced as the point at feet + 36, so a player resting on a floor
// sits EXACTLY on that floor's expanded top plane, and if they are pushed into a wall
// rising off that floor, the leaf they land in is bounded by that plane at distance
// zero. ww_golem's cockpit is this shape (deck at z=-112, seam at -76).
inline std::vector<uint8_t> make_wall_on_floor_map() {
	// Floor: everything below z = 0. Wall: everything at x <= -8 above z = 72.
	// Expanded for hull h the floor tops out at -HULL.mins.z and the wall starts at
	// 72 - -HULL.mins.z, which meet for hull 1 (36) — the case under test. For the
	// other hulls they simply overlap, which is just as solid.
	const double fmins[3] = { -4096, -4096, -4096 }, fmaxs[3] = { 4096, 4096, 0 };
	const double wmins[3] = { -4096, -4096, 72 }, wmaxs[3] = { -8, 4096, 4096 };

	BlobBuilder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) { m.mins[i] = (float)fmins[i]; m.maxs[i] = (float)wmaxs[i]; }
	m.maxs[0] = (float)fmaxs[0];
	m.maxs[1] = (float)fmaxs[1];

	// Floor first so the wall can point its "outside" children at it.
	const int f0 = add_box_brush(b, fmins, fmaxs, /*as_nodes=*/true);
	m.headnode[0] = add_box_brush(b, wmins, wmaxs, /*as_nodes=*/true, CONTENTS_SOLID, f0);

	for (int h = 1; h < 4; ++h) {
		double efmins[3], efmaxs[3], ewmins[3], ewmaxs[3];
		for (int i = 0; i < 3; ++i) {
			efmins[i] = fmins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			efmaxs[i] = fmaxs[i] - hopbsp::HULL_SIZES[h].mins[i];
			ewmins[i] = wmins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			ewmaxs[i] = wmaxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		const int ef = add_box_brush(b, efmins, efmaxs, /*as_nodes=*/false);
		m.headnode[h] = add_box_brush(b, ewmins, ewmaxs, /*as_nodes=*/false, CONTENTS_SOLID, ef);
	}
	b.models.push_back(m);
	return b.build();
}

// A wide, thin slab centred on the GoldSrc origin: a floor whose top face sits at
// z = 0, i.e. Godot y = 0.
inline std::vector<uint8_t> make_floor_map() {
	const double mins[3] = { -512, -512, -64 };
	const double maxs[3] = { 512, 512, 0 };
	return make_box_map(mins, maxs);
}

inline std::unique_ptr<HopBspTraceable<T>> load(const std::vector<uint8_t> &blob,
                                                int blocking = hopbsp::BLOCK_SOLID) {
	auto t = std::make_unique<HopBspTraceable<T>>();
	bool ok = t->build(blob.data(), blob.size(), 0, (T)SCALE, blocking);
	assert(ok && "blob failed to parse");
	(void)ok;
	return t;
}

inline std::shared_ptr<hop::solid<T>> make_box_solid(T hx, T hy, T hz) {
	hop::aa_box<T> b;
	b.mins = vec(-hx, -hy, -hz);
	b.maxs = vec(hx, hy, hz);
	auto s = std::make_shared<hop::solid<T>>();
	s->add_shape(std::make_shared<hop::shape<T>>(b));
	return s;
}


// A box that can spin. Inertia is what arms rotates_dynamically(), which is the gate on
// contact patches — so this, not make_box_solid, is what a patch test wants.
inline std::shared_ptr<hop::solid<T>> make_spinning_box(T hx, T hy, T hz) {
	auto s = make_box_solid(hx, hy, hz);
	s->set_mass((T)1);
	s->set_inertia(vec((T)1, (T)1, (T)1));
	return s;
}
