extends RefCounted
## What a body resting on a BSP floor does when you drag it: whether it rolls, tips, or
## scrubs to a halt flat.
##
## WHAT TIPS AND WHAT DOES NOT is the whole design here, and it is not a matter of taste.
## A face contact is a four-point manifold, one per bottom corner, regenerated every tick
## by face clipping (hop Phase 14). Friction pulls at the face with a moment of mu*N*h
## about the centre; the corners answer by redistributing the normal load, worth N*(L/2).
## So a dragged box goes over exactly when
##
##     mu*h > L/2
##
## and slides flat otherwise. The wide box below is the stable side of that line
## (mu*h = 0.05 against L/2 = 0.10) and the narrow box the unstable one (0.15 against
## 0.025), six times over, so between them they pin the threshold from both directions.
##
## HISTORY, because this file used to assert the opposite and it is worth knowing why.
## It was written to cover the contact WITNESS POINT — hop takes a lever arm as
## (impact − centre), and a hull-0 trace hands back one point, so at the time that point
## was the only thing the solver had to go on for a box on a floor. With a single witness
## at the face centre there is no restoring moment at all, so friction spun a dragged box
## up to the rolling-without-slipping law of a CYLINDER of radius h — −80 rad/s at v = 4 —
## and this file asserted exactly that. Manifolds landed two weeks later, gave the face
## its four corners, and a flat box on a flat floor correctly stopped rolling. The old
## assertions had been measuring the contact MODEL, not the arm, and they failed.
##
## So the witness point is no longer observable from here, and these tests do not cover
## it: putting either wrong answer back — the arm left at the mover's centre, or walked
## out to the box's CORNER instead of the face centre — leaves all three cases below
## passing, because the solver reads the clipped manifold rather than the witness for a
## box lying on a floor. That coverage moved to where it is still visible, as direct
## assertions on the witness in tests/test_bsp_traceable.cpp
## (test_solid_impact_is_on_the_surface): `.y` catches the centre answer and `.x`/`.z`
## catch the corner. Both verified by sabotage.

const BspFixture = preload("res://tests/bsp_fixture.gd")
const BspScene = preload("res://tests/bsp_scene.gd")

## The dropped and wide-sliding boxes are 0.2 x 0.1 x 0.2 — half-height 0.05 against a
## half-base of 0.10, so mu*h < L/2 and they stay down.
const HALF_HEIGHT := 0.05
## The narrow one is 0.05 x 0.3 x 0.05: half-height 0.15 against a half-base of 0.025, so
## mu*h > L/2 by six times and it has to go over.
const TALL_HALF_HEIGHT := 0.15
const SLIDE_SPEED := 4.0
## Frames allowed for a tip to finish. The narrow box is past 5 rad/s on the first frame
## and lies flat well inside this, so the assertions read a settled outcome rather than a
## transient.
const TIP_FRAMES := 40

var _tree: SceneTree
var _map: Node3D
var _dropped: RigidBody3D
var _sliding: RigidBody3D
var _narrow: RigidBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


## A box under gravity on the fixture's floor. No angular damp anywhere here — the whole
## point is what the CONTACT does to the spin.
func _box(at: Vector3, size: Vector3, lin: Vector3) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.collision_layer = 0
	body.collision_mask = 1
	body.angular_damp = 0.0
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	cs.shape = box
	body.add_child(cs)
	_map.add_child(body)
	body.global_position = at
	body.linear_velocity = lin
	return body


func setup() -> void:
	_map = BspScene.make_map(_tree, BspFixture.wall_on_floor())
	# Each parked far from the others so no two can ever meet. The fixture's floor is
	# everything below Godot y = 0.
	var wide := Vector3(0.2, HALF_HEIGHT * 2.0, 0.2)
	# Dropped square-on from a height.
	_dropped = _box(Vector3(20, 0.4, 20), wide, Vector3.ZERO)
	# Already resting, and shoved along the floor.
	_sliding = _box(Vector3(-20, HALF_HEIGHT + 0.002, -20), wide, Vector3(SLIDE_SPEED, 0, 0))
	# The same shove on a box too tall for its base to hold it.
	_narrow = _box(Vector3(20, TALL_HALF_HEIGHT + 0.002, -20),
		Vector3(0.05, TALL_HALF_HEIGHT * 2.0, 0.05), Vector3(SLIDE_SPEED, 0, 0))


func teardown() -> void:
	if is_instance_valid(_map):
		_map.free()
	_map = null
	_dropped = null
	_sliding = null
	_narrow = null


func _settle(frames: int) -> void:
	for i in frames:
		await _tree.physics_frame


## How far the body's own up has been turned off world up, in degrees — axis-agnostic, so
## it reads a tip about any horizontal axis rather than assuming one.
func _tilt_deg(body: RigidBody3D) -> float:
	var up: Vector3 = body.global_transform.basis.y.normalized()
	return rad_to_deg(acos(clampf(up.dot(Vector3.UP), -1.0, 1.0)))


func tests() -> Array:
	return [
		"a_flat_landing_adds_no_spin",
		"a_wide_box_slides_flat_without_rolling",
		"a_narrow_box_tips_when_dragged",
	]


func a_flat_landing_adds_no_spin(t) -> void:
	# A box dropped square-on has nothing to turn it: the surface pushes straight up
	# through its centre. Any spin at all here is the contact fabricating one.
	await _settle(90)
	t.lt(_dropped.angular_velocity.length(), 0.05,
		"a box dropped flat on a floor must land unspun")


func a_wide_box_slides_flat_without_rolling(t) -> void:
	# Friction's moment is real here — the same one that takes the narrow box over — but
	# the four corners of the face out-moment it, so the box must scrub to a halt still
	# flat. This is the side of the threshold that a too-eager manifold breaks: lose the
	# corners, or let one of them carry the whole normal load, and the box starts turning
	# the moment it is pushed.
	await _settle(20)
	t.lt(absf(_sliding.angular_velocity.z), 0.5,
		"a wide box dragged on its face must not roll (mu*h < L/2)")
	t.lt(_tilt_deg(_sliding), 5.0, "and must still be sitting flat")
	# Pins that friction is actually doing something, so the test cannot pass by the
	# contact being missed altogether.
	t.lt(_sliding.linear_velocity.x, SLIDE_SPEED - 0.5,
		"friction must still be scrubbing its speed off")


func a_narrow_box_tips_when_dragged(t) -> void:
	# The unstable side of the threshold. Friction's moment about the centre is mu*N*h and
	# the half-base is only a sixth of that, so this box cannot stay up — it is past
	# 5 rad/s on the first frame and lies flat long before TIP_FRAMES. A solver that
	# cannot turn a body from a face contact at all leaves it scrubbing to a stop bolt
	# upright, which is what this reads.
	await _settle(TIP_FRAMES)
	t.gt(_tilt_deg(_narrow), 45.0,
		"a box too tall for its base must tip when dragged (mu*h > L/2)")
