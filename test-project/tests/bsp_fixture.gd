extends RefCounted
## Builds a stripped BSP30 blob in memory, so a test can exercise the GoldSrc hull trace
## without shipping a .bsp.
##
## Only the lumps the trace reads are written (PLANES / NODES / CLIPNODES / LEAFS /
## MODELS); everything else is a zero-length lump, which the loader treats as absent.
## That is the same shape a host engine's blob exporter produces and parks on the scene
## root as "bsp_data".
##
## Mirrors the C++ fixture in tests/test_bsp_traceable.cpp. The byte layout is asserted
## on the C++ side by static_assert, so a format change breaks a build rather than
## silently mistracing here.

const HLBSP_VERSION := 30
const MAX_BSP_LUMPS := 15
const LUMP_PLANES := 1
const LUMP_NODES := 5
const LUMP_CLIPNODES := 9
const LUMP_LEAFS := 10
const LUMP_MODELS := 14

const CONTENTS_EMPTY := -1
const CONTENTS_SOLID := -2

## Engine-baked hull box sizes (Half-Life). The compiler expanded the clipnode trees for
## exactly these, so a fixture has to expand its brushes the same way.
const HULL_SIZES := [
	{"mins": Vector3(0, 0, 0), "maxs": Vector3(0, 0, 0)},           # 0 — point hull
	{"mins": Vector3(-16, -16, -36), "maxs": Vector3(16, 16, 36)},  # 1 — standing
	{"mins": Vector3(-32, -32, -32), "maxs": Vector3(32, 32, 32)},  # 2 — large
	{"mins": Vector3(-16, -16, -18), "maxs": Vector3(16, 16, 18)},  # 3 — crouched
]

var _planes := PackedByteArray()
var _plane_count := 0
var _nodes := PackedByteArray()
var _node_count := 0
var _clipnodes := PackedByteArray()
var _clipnode_count := 0
var _leafs := PackedByteArray()
var _leaf_count := 0
var _models := PackedByteArray()


static func _f32(a: PackedByteArray, v: float) -> void:
	var b := PackedByteArray(); b.resize(4); b.encode_float(0, v); a.append_array(b)


static func _i32(a: PackedByteArray, v: int) -> void:
	var b := PackedByteArray(); b.resize(4); b.encode_s32(0, v); a.append_array(b)


static func _i16(a: PackedByteArray, v: int) -> void:
	var b := PackedByteArray(); b.resize(2); b.encode_s16(0, v); a.append_array(b)


static func _u16(a: PackedByteArray, v: int) -> void:
	var b := PackedByteArray(); b.resize(2); b.encode_u16(0, v); a.append_array(b)


## Six axis planes bounding [mins, maxs]. Returns the index of the first; plane k*2 is
## the lower bound on axis k, k*2+1 the upper.
func _add_box_planes(mins: Vector3, maxs: Vector3) -> int:
	var first := _plane_count
	for axis in 3:
		for hi in 2:
			var n := Vector3.ZERO
			n[axis] = 1.0
			_f32(_planes, n.x); _f32(_planes, n.y); _f32(_planes, n.z)
			_f32(_planes, maxs[axis] if hi == 1 else mins[axis])
			_i32(_planes, axis)  # axial — exercises the fast `p[type] - dist` path
			_plane_count += 1
	return first


## Leaf 0 is the solid leaf, leaf 1 empty — the convention a real BSP uses, and what
## hull 0's negative children address as -(leaf + 1).
func _ensure_leafs(contents: int) -> void:
	if _leaf_count != 0:
		return
	for c in [contents, CONTENTS_EMPTY]:
		_i32(_leafs, c); _i32(_leafs, -1)          # contents, visofs
		for i in 6: _i16(_leafs, 0)                # mins/maxs
		_u16(_leafs, 0); _u16(_leafs, 0)           # firstmarksurface, nummarksurfaces
		for i in 4: _leafs.append(0)               # ambient_level
		_leaf_count += 1


## A solid axis-aligned box brush as a 6-deep tree: descending "inward" on every plane
## lands in SOLID, stepping outside any one of them lands in EMPTY.
##
## `outside` chains brushes into a union — give brush A the root of brush B and the tree
## reads "solid if inside A, else test B", which is what lets a fixture have a leaf
## bounded by another brush's face. Pass NO_OUTSIDE for the plain empty child.
## Returns the root node/clipnode index of the brush just added.
const NO_OUTSIDE := 0x7FFFFFFF

func _add_box_brush(mins: Vector3, maxs: Vector3, as_nodes: bool,
		contents := CONTENTS_SOLID, outside := NO_OUTSIDE) -> int:
	var p0 := _add_box_planes(mins, maxs)
	var base := _node_count if as_nodes else _clipnode_count
	if as_nodes:
		_ensure_leafs(contents)

	# hull 0 addresses leafs as -(leaf+1); hulls 1..3 store the contents directly.
	var solid_child := -1 if as_nodes else contents
	var empty_child := outside if outside != NO_OUTSIDE else (-2 if as_nodes else CONTENTS_EMPTY)

	for i in 6:
		var upper := (i % 2) == 1
		var inward := solid_child if i == 5 else (base + i + 1)
		var child0 := empty_child if upper else inward
		var child1 := inward if upper else empty_child
		if as_nodes:
			_i32(_nodes, p0 + i)
			_i16(_nodes, child0); _i16(_nodes, child1)
			for k in 6: _i16(_nodes, 0)        # mins/maxs
			_u16(_nodes, 0); _u16(_nodes, 0)   # firstface, numfaces
			_node_count += 1
		else:
			_i32(_clipnodes, p0 + i)
			_i16(_clipnodes, child0); _i16(_clipnodes, child1)
			_clipnode_count += 1
	return base


## A union of box brushes, built into all four hulls at once. Returns the four
## headnodes, ready for _add_model.
##
## `brushes` is an array of {"mins": Vector3, "maxs": Vector3}, tested in order: the
## tree reads "solid if inside the first, else the second, …, else empty". It is built
## back to front so each brush can point its "outside" child at the next one.
##
## This exists to state the hull expansion ONCE. It is the line in this file easiest to
## get wrong — the low bound grows by the hull's MAXS and the high bound shrinks by its
## MINS, which reads backwards until you remember the hull box is swept around the
## brush rather than added to it. Hull 0 falls out of the same loop for free, since its
## box is a point and the arithmetic is a no-op there.
func _add_expanded_union(brushes: Array) -> Array:
	var headnodes := [0, 0, 0, 0]
	for h in 4:
		var hs: Dictionary = HULL_SIZES[h]
		var outside := NO_OUTSIDE
		for i in range(brushes.size() - 1, -1, -1):
			var br: Dictionary = brushes[i]
			outside = _add_box_brush(br["mins"] - hs["maxs"], br["maxs"] - hs["mins"],
				h == 0, CONTENTS_SOLID, outside)
		headnodes[h] = outside
	return headnodes


func _add_model(mins: Vector3, maxs: Vector3, headnodes: Array) -> void:
	_f32(_models, mins.x); _f32(_models, mins.y); _f32(_models, mins.z)
	_f32(_models, maxs.x); _f32(_models, maxs.y); _f32(_models, maxs.z)
	_f32(_models, 0.0); _f32(_models, 0.0); _f32(_models, 0.0)  # origin
	for h in headnodes: _i32(_models, h)
	_i32(_models, 0)                    # visleafs
	_i32(_models, 0); _i32(_models, 0)  # firstface, numfaces


func build() -> PackedByteArray:
	var out := PackedByteArray()
	out.resize(4 + MAX_BSP_LUMPS * 8)
	out.encode_s32(0, HLBSP_VERSION)
	var put := func(idx: int, lump: PackedByteArray) -> void:
		if lump.is_empty():
			return
		out.encode_s32(4 + idx * 8, out.size())
		out.encode_s32(4 + idx * 8 + 4, lump.size())
		out.append_array(lump)
	put.call(LUMP_PLANES, _planes)
	put.call(LUMP_NODES, _nodes)
	put.call(LUMP_CLIPNODES, _clipnodes)
	put.call(LUMP_LEAFS, _leafs)
	put.call(LUMP_MODELS, _models)
	return out


## A floor with a wall standing on it, as a union of two brushes, sized so that in hull 1
## the wall's solid begins exactly where the floor's ends.
##
## That seam is where every standing player's trace point lives: a hull-1 mover is traced
## as the point at feet + 36, so a mover resting on a floor sits EXACTLY on that floor's
## expanded top plane, and a wall rising off the floor is bounded by the same plane at
## distance zero. Contents is a strict d < 0, so a point there is inside NEITHER — the
## whole class of bug these tests exist for.
##
## GoldSrc: floor is everything below z = 0; wall is everything at x <= -8 above z = 72.
static func wall_on_floor() -> PackedByteArray:
	var b = new()
	var fmins := Vector3(-4096, -4096, -4096)
	var fmaxs := Vector3(4096, 4096, 0)
	var wmins := Vector3(-4096, -4096, 72)
	var wmaxs := Vector3(-8, 4096, 4096)

	var headnodes := b._add_expanded_union([
		{"mins": wmins, "maxs": wmaxs},   # the wall, falling through to
		{"mins": fmins, "maxs": fmaxs},   # the floor
	])
	b._add_model(fmins, Vector3(fmaxs.x, fmaxs.y, wmaxs.z), headnodes)
	return b.build()


## A free-standing slab, reachable from BOTH sides, and nothing else.
##
## Every other fixture here is a wall a mover can only get at from one direction, which
## is exactly the shape that hid the hull-offset bug for so long: a one-sided wall is
## always wrong by the same amount, and looks like a deliberate standoff. The defect only
## states itself as a difference between the two faces of the same slab.
##
## GoldSrc: solid x in [-8, +8], above z = 72.
static func pillar() -> PackedByteArray:
	var b = new()
	var mins := Vector3(-8, -4096, 72)
	var maxs := Vector3(8, 4096, 4096)
	b._add_model(mins, maxs, b._add_expanded_union([{"mins": mins, "maxs": maxs}]))
	return b.build()


## A floor with TWO perpendicular walls standing on it — a corner.
##
## One wall cannot reproduce a mover being poisoned by the solid it is *beside* while
## sweeping into the solid *ahead*: recovery pushes it out of a single wall before the
## sweep ever runs. A corner separates the two roles, which is the pose that mattered —
## sub-STUCK_SLOP inside the side wall, travelling parallel to it, into the far wall.
##
## GoldSrc: floor z <= 0; wall X at x <= -8; wall Y at y <= -8, both above z = 72.
## Expanded for hull 1 those faces are x = 8 and y = 8.
static func corner_on_floor() -> PackedByteArray:
	var b = new()
	var fmins := Vector3(-4096, -4096, -4096)
	var fmaxs := Vector3(4096, 4096, 0)
	var xmins := Vector3(-4096, -4096, 72)
	var xmaxs := Vector3(-8, 4096, 4096)
	var ymins := Vector3(-4096, -4096, 72)
	var ymaxs := Vector3(4096, -8, 4096)

	# "solid if in X, else if in Y, else if in the floor, else empty".
	var headnodes := b._add_expanded_union([
		{"mins": xmins, "maxs": xmaxs},
		{"mins": ymins, "maxs": ymaxs},
		{"mins": fmins, "maxs": fmaxs},
	])
	b._add_model(fmins, Vector3(fmaxs.x, fmaxs.y, xmaxs.z), headnodes)
	return b.build()
