extends RefCounted
## body_test_motion against a GoldSrc BSP hull — the server-level query, not the trace.
##
## tests/test_bsp_traceable.cpp covers the layer below this, where the answer is a
## hull_trace. What _body_test_motion adds on top is recovery, the resting-gap lift, the
## escape clause for a sweep that begins inside solid, and the safe/unsafe fractions
## CharacterBody3D actually moves on. Those need a live PhysicsServer3D, so they cannot
## be reached from the C++ suite — and every one of them has been wrong at some point
## while the traceable underneath was fine.
##
## The fixture is a real BSP blob (bsp_fixture.gd) on a carrier body, exactly the
## contract a host engine's map converter writes.
##
## COVERAGE, measured by reintroducing each fix's bug and re-running rather than assumed:
##   caught   the STUCK_SLOP disagreement between the stuck test and the sweep
##            (a_hair_inside_a_SIDE_wall_does_not_free_the_path_ahead)
##   caught   recovery reading overlap.depth as margin-inflated
##            (sunk_into_the_floor_still_hits_the_wall)
##   NOT caught, and worth knowing before trusting this file:
##            the touching branch's clear-by-(depth + touch_eps) sign, recovery
##            continuing past a touching contact rather than breaking, and the escape
##            clause taking its push-out from a swept trace. All three self-correct in a
##            static fixture; they only diverge against a mover being pushed by moving
##            geometry, which needs a simulated ride, not a query. The rest of the tests
##            here assert real invariants but are not pinned to a specific past bug.

const BspFixture = preload("res://tests/bsp_fixture.gd")

const S := 0.025      # a host engine's metres-per-GoldSrc-unit; matches WizardWars
const MARGIN := 0.01  # a typical CharacterBody3D safe_margin

var _map: Node3D
var _mover: CharacterBody3D
var _tree: SceneTree


func _init(tree: SceneTree) -> void:
	_tree = tree


## GoldSrc (x, y, z) -> Godot. Godot +x is GoldSrc -x; Godot y is GoldSrc z.
static func gs(x: float, y: float, z: float) -> Vector3:
	return Vector3(-x * S, z * S, y * S)


func setup() -> void:
	_build(BspFixture.wall_on_floor())


## Tear down and rebuild with a different blob. The hull is built when a carrier body
## enters the tree, so the blob has to be on the ancestor BEFORE that — assigning
## "bsp_data" afterwards changes nothing, which is a silent no-op worth knowing about.
func swap_fixture(blob: PackedByteArray) -> void:
	teardown()
	_build(blob)


func _build(blob: PackedByteArray) -> void:
	# The blob lives once on an ancestor under "bsp_data"; each carrier body carries the
	# model index and the scale. Scale has no default on purpose — a missing one means
	# the two sides disagree about the map's units.
	_map = Node3D.new()
	_map.set_meta("bsp_data", blob)
	_tree.get_root().add_child(_map)

	var world := StaticBody3D.new()
	world.collision_layer = 1
	world.set_meta("bsp_model", 0)
	world.set_meta("bsp_scale", S)
	# A carrier still needs one enabled shape: the hull stands in for it, but a body with
	# no enabled shape is intangible by design (that is how a door goes non-solid).
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(8, 8, 8)
	cs.shape = box
	world.add_child(cs)
	_map.add_child(world)

	# A standing player: 32x32x72 GoldSrc, feet-origin, which selects hull 1.
	_mover = CharacterBody3D.new()
	_mover.collision_layer = 0
	_mover.collision_mask = 1
	_mover.safe_margin = MARGIN
	var mcs := CollisionShape3D.new()
	var cap := CapsuleShape3D.new()
	cap.radius = 16 * S
	cap.height = 72 * S
	mcs.shape = cap
	mcs.position = Vector3(0, 36 * S, 0)
	_mover.add_child(mcs)
	_map.add_child(_mover)


func teardown() -> void:
	if is_instance_valid(_map):
		_map.free()
	_map = null
	_mover = null


## One body_test_motion from a GoldSrc-space feet position with a GoldSrc-space motion.
## `travel_gs` comes back in GoldSrc units so assertions read in the same numbers the
## geometry is stated in.
func probe(feet: Vector3, motion: Vector3) -> Dictionary:
	var xform := Transform3D(Basis.IDENTITY, gs(feet.x, feet.y, feet.z))
	_mover.global_transform = xform
	var p := PhysicsTestMotionParameters3D.new()
	p.from = xform
	p.motion = gs(motion.x, motion.y, motion.z)
	p.margin = MARGIN
	p.recovery_as_collision = true
	p.max_collisions = 4
	var r := PhysicsTestMotionResult3D.new()
	PhysicsServer3D.body_test_motion(_mover.get_rid(), p, r)
	var t := r.get_travel()
	return {
		"unsafe": r.get_collision_unsafe_fraction(),
		"travel_gs": Vector3(-t.x / S, t.z / S, t.y / S),
		"contacts": r.get_collision_count(),
	}


# The fixture in GoldSrc units, and the two numbers every pose below is stated against.
# The brushes are floor z <= 0 and wall x <= -8 above z = 72, but a hull-1 mover is
# traced as a POINT against the EXPANDED tree, so those are not the faces it meets:
#
#   expanded floor top   trace point z = 36, i.e. feet z = 0
#   expanded wall face   GoldSrc x = 8   (the brush's -8, grown by the hull's 16)
#
# Both of which are the same plane where they meet: a mover resting on the floor has its
# trace point exactly on the floor's expanded top, which is exactly where the wall's
# expanded underside begins. Contents is a strict d < 0, so a point there is inside
# NEITHER — the whole class of bug these tests exist for.
const WALL_FACE := 8.0    # expanded; inside the wall is x < this
const FLOOR_FEET := 0.0   # feet height at which the trace point sits on the floor


func tests() -> Array:
	return [
		"resting_on_the_floor_is_not_inside_it",
		"resting_on_the_seam_still_hits_the_wall",
		"sunk_into_the_floor_still_hits_the_wall",
		"a_hair_inside_a_wall_does_not_free_the_fall",
		"a_hair_inside_a_SIDE_wall_does_not_free_the_path_ahead",
		"flush_against_a_wall_can_still_leave_it",
		"open_air_is_unobstructed",
	]


func resting_on_the_floor_is_not_inside_it(t) -> void:
	# Standing on open floor, well clear of the wall, asked to hold still. Recovery must
	# not shove a resting body: a lift of a hair is the invariant, a lift of a margin is
	# the bug that made a standing player hover.
	var r := probe(Vector3(60, 0, 0), Vector3.ZERO)
	t.lt(r["travel_gs"].length(), 0.2,
		"a mover resting on a floor is nudged by a hair at most, not a margin")


func resting_on_the_seam_still_hits_the_wall(t) -> void:
	# The original ejection, reduced. Feet exactly on the floor puts the trace point
	# exactly on the plane the floor and the wall share, where a strict d < 0 leaves the
	# point inside NEITHER. Walking into the wall from 12 units out must still stop.
	var r := probe(Vector3(20, 0, 0), Vector3(-16, 0, 0))
	t.lt(r["unsafe"], 0.9, "the wall must stop a walk that starts on the seam")
	t.lt(absf(r["travel_gs"].x), 13.0, "and stop it before it passes through")


func sunk_into_the_floor_still_hits_the_wall(t) -> void:
	# The few thousandths of a unit that settling on a floor always leaves. Being inside
	# the floor must not disable the wall ahead.
	var r := probe(Vector3(20, 0, -0.07), Vector3(-16, 0, 0))
	t.lt(r["unsafe"], 0.9, "a mover sunk into the floor must still be stopped")


func a_hair_inside_a_wall_does_not_free_the_fall(t) -> void:
	# 0.03 units inside the wall face — under DIST_EPSILON, so it is where traces
	# routinely leave a body, not an unusual pose. It used to read as "not in solid" to
	# the stuck test and "allsolid" to the sweep at once, and the sweep gave up without
	# looking ahead. Falling from there must still find the floor.
	var r := probe(Vector3(WALL_FACE - 0.03, 0, 40), Vector3(0, 0, -60))
	t.lt(r["unsafe"], 0.9,
		"a mover a hair inside a wall must not fall straight through the floor")


func a_hair_inside_a_SIDE_wall_does_not_free_the_path_ahead(t) -> void:
	# The pose that took longest to find. A mover under STUCK_SLOP inside one wall while
	# travelling PARALLEL to it, into another. The stuck test and the push-out measure
	# against a hull shrunk by that slop, so they call the point free and recovery does
	# nothing; the sweep uses no such band, calls the identical point allsolid, and gives
	# up without ever looking at the wall ahead. Neither answer is wrong alone.
	#
	# It needs a corner: with one wall, recovery pushes the mover clear before the sweep
	# runs and the disagreement never surfaces.
	swap_fixture(BspFixture.corner_on_floor())
	var r := probe(Vector3(40, WALL_FACE - 0.01, FLOOR_FEET), Vector3(-60, 0, 0))
	t.lt(r["unsafe"], 0.9,
		"a mover a hair inside the SIDE wall must still be stopped by the wall ahead")


func flush_against_a_wall_can_still_leave_it(t) -> void:
	# The fix must block the wall, not pin the body to it.
	var r := probe(Vector3(WALL_FACE - 0.03, 0, FLOOR_FEET), Vector3(16, 0, 0))
	t.gt(r["unsafe"], 0.9, "a mover flush on a wall stays free to walk away from it")


func open_air_is_unobstructed(t) -> void:
	# The control. Without it every assertion above is satisfied by a trace that blocks
	# everything, which is exactly what an over-eager fix produces.
	var r := probe(Vector3(200, 0, 200), Vector3(-16, 0, 0))
	t.gt(r["unsafe"], 0.99, "nothing in the way means the whole motion is allowed")
	t.near(r["travel_gs"].x, -16.0, 0.5, "and the body actually moves it")
