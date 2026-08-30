extends RefCounted
## Where a hull trace says a mover TOUCHES, as the contact solver reads it.
##
## hop takes a contact's lever arm as (impact − centre) and resolves the whole contact
## about it, so the witness point is not cosmetic — it decides whether a body can be
## torqued by the surface it is on, and whether the impulse holding it up torques it too.
## Hull 0 traces a mover as a point, so the point it hands back is the only thing the
## solver has to go on.
##
## COVERAGE, measured by putting each of the two wrong answers back and re-running:
##   * no walk-back at all — the point stays at the mover's CENTRE, every arm is zero and
##     nothing the mover slides on can ever turn it. Fails a_sliding_body_rolls with w
##     flat at 0.
##   * the box's CORNER, its support point in −n — exact for the plane the sweep stopped
##     on, and ruinous at rest: a floor normal says nothing about the tangential axes, so
##     the arm runs ACROSS the face and the support impulse becomes a torque. Fails
##     a_flat_landing_adds_no_spin at 23 rad/s out of a square drop, and takes
##     rolling_settles_where_the_contact_stops_slipping with it.
##
## Dynamics, not queries, because that is where the difference lives: body_test_motion
## reports the mover's origin, and only the solver reads `impact`.

const BspFixture = preload("res://tests/bsp_fixture.gd")
const BspScene = preload("res://tests/bsp_scene.gd")

const HALF_HEIGHT := 0.05          # the boxes below are 0.2 x 0.1 x 0.2
const SLIDE_SPEED := 4.0

var _tree: SceneTree
var _map: Node3D
var _dropped: RigidBody3D
var _sliding: RigidBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


## A box under gravity on the fixture's floor. No angular damp anywhere here — the whole
## point is what the CONTACT does to the spin.
func _box(at: Vector3, lin: Vector3) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.collision_layer = 0
	body.collision_mask = 1
	body.angular_damp = 0.0
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.2, HALF_HEIGHT * 2.0, 0.2)
	cs.shape = box
	body.add_child(cs)
	_map.add_child(body)
	body.global_position = at
	body.linear_velocity = lin
	return body


func setup() -> void:
	_map = BspScene.make_map(_tree, BspFixture.wall_on_floor())
	# Dropped square-on from a height, and parked far from the other so the two can
	# never meet. The fixture's floor is everything below Godot y = 0.
	_dropped = _box(Vector3(20, 0.4, 20), Vector3.ZERO)
	# Already resting, and shoved along the floor.
	_sliding = _box(Vector3(-20, HALF_HEIGHT + 0.002, -20), Vector3(SLIDE_SPEED, 0, 0))


func teardown() -> void:
	if is_instance_valid(_map):
		_map.free()
	_map = null
	_dropped = null
	_sliding = null


func _settle(frames: int) -> void:
	for i in frames:
		await _tree.physics_frame


func tests() -> Array:
	return [
		"a_flat_landing_adds_no_spin",
		"a_sliding_body_rolls",
		"rolling_settles_where_the_contact_stops_slipping",
	]


func a_flat_landing_adds_no_spin(t) -> void:
	# A box dropped square-on has nothing to turn it: the surface pushes straight up
	# through its centre. Any spin at all here is the contact fabricating one.
	await _settle(90)
	t.lt(_dropped.angular_velocity.length(), 0.05,
		"a box dropped flat on a floor must land unspun")


func a_sliding_body_rolls(t) -> void:
	# The other side of the same coin: friction across the face DOES have an arm, so a
	# body dragged along the floor has to start rolling. Rolling forward along +x is a
	# turn about −z.
	await _settle(20)
	t.lt(_sliding.angular_velocity.z, -1.0,
		"friction on the floor must roll a sliding body forward")


func rolling_settles_where_the_contact_stops_slipping(t) -> void:
	# Pins the arm's LENGTH, not just its existence. Friction acts until the contact
	# point itself stops moving — v + ω×r = 0 — which for an arm of exactly the box's
	# half-height means ω settles at −v/h and friction then has nothing left to do.
	# Rolling without slipping is the one speed that falls out of the right arm; a
	# shorter or longer one settles somewhere else.
	await _settle(60)
	# A body that has been brought to a dead stop satisfies w = -v/h trivially, so pin
	# that it is still going: the ratio is the claim, not the standstill.
	t.gt(_sliding.linear_velocity.x, 0.5, "a rolling body keeps rolling")
	t.near(_sliding.angular_velocity.z, -_sliding.linear_velocity.x / HALF_HEIGHT, 0.5,
		"a rolling body settles at w = -v/h, the speed that stills the contact point")
