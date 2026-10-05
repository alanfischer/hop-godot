extends RefCounted
## What Godot is told about a body's inertia, through PhysicsDirectBodyState3D.
##
## hop auto-derives inertia from the collision AABB treated as a solid box — I = m/12 ·
## (ey²+ez², ex²+ez², ex²+ey²) for full extents e — and the numbers below are that
## formula, written out, so a change to the derivation has to be deliberate.
##
## The two getters this covers returned a flat zero until now, with a "no rotation"
## comment left over from before hop integrated angular velocity at all. Nothing in the
## engine reads them — hop's solver goes to the solid directly — so the wrong answer was
## invisible, and a probe asking Godot whether a body could spin got told "it cannot"
## about a body that demonstrably tumbles.
##
## The body-frame diagonal and the world tensor are asserted SEPARATELY and on a turned
## body, because that is the whole distinction between the two calls: `inverse_inertia`
## is the principal diagonal in the body's own frame and does not move when the body
## turns, while `inverse_inertia_tensor` is R·diag(I⁻¹)·Rᵀ and does. Returning one where
## the other was asked for is the easy mistake, and on an unturned body the two are
## identical, so only a rotated case can tell them apart.

## Deliberately asymmetric — 0.2 x 0.1 x 0.4 — so all three principal values differ.
## A cube, or anything with two equal extents, cannot catch a swapped or transposed axis.
const SIZE := Vector3(0.2, 0.1, 0.4)
const MASS := 1.0

var _tree: SceneTree
var _root: Node3D
var _box: RigidBody3D
var _turned: RigidBody3D
var _locked: RigidBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


## The analytic inverse inertia for SIZE at MASS, as the reference the assertions read
## against. Zero extents would divide by zero; SIZE has none.
static func _expected_inv() -> Vector3:
	var c := MASS / 12.0
	var i := Vector3(
		c * (SIZE.y * SIZE.y + SIZE.z * SIZE.z),
		c * (SIZE.x * SIZE.x + SIZE.z * SIZE.z),
		c * (SIZE.x * SIZE.x + SIZE.y * SIZE.y))
	return Vector3(1.0 / i.x, 1.0 / i.y, 1.0 / i.z)


func _body(at: Vector3, rot: Vector3, lock: bool) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.mass = MASS
	body.collision_layer = 0
	body.collision_mask = 0          # no contacts wanted; this is about the tensor alone
	body.gravity_scale = 0.0         # and no falling, so nothing moves mid-test
	body.lock_rotation = lock
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = SIZE
	cs.shape = box
	body.add_child(cs)
	_root.add_child(body)
	body.global_position = at
	body.rotation = rot
	return body


func setup() -> void:
	_root = Node3D.new()
	_tree.get_root().add_child(_root)
	_box = _body(Vector3(0, 0, 0), Vector3.ZERO, false)
	# A quarter turn about Z maps the body's x onto world y, so the world tensor's
	# leading diagonal must come back with its x and y entries exchanged.
	_turned = _body(Vector3(10, 0, 0), Vector3(0, 0, PI * 0.5), false)
	# lock_rotation reaches hop as BODY_MODE_RIGID_LINEAR, which zeroes the inertia on
	# purpose so rotates_dynamically() stays false.
	_locked = _body(Vector3(20, 0, 0), Vector3.ZERO, true)


func teardown() -> void:
	if is_instance_valid(_root):
		_root.free()
	_root = null
	_box = null
	_turned = null
	_locked = null


func _state(body: RigidBody3D) -> PhysicsDirectBodyState3D:
	return PhysicsServer3D.body_get_direct_state(body.get_rid())


func tests() -> Array:
	return [
		"a_box_reports_its_box_inertia",
		"the_tensor_turns_with_the_body_and_the_diagonal_does_not",
		"a_rotation_locked_body_reports_no_inertia",
		"principal_axes_are_the_body_axes",
	]


func a_box_reports_its_box_inertia(t) -> void:
	await _tree.physics_frame
	var inv := _state(_box).inverse_inertia
	var want := _expected_inv()
	# Loose enough for float round-tripping through hop_scalar, tight enough that a
	# swapped axis (60 vs 70.6 vs 240 here) cannot slip through.
	t.near(inv.x, want.x, 0.5, "inverse inertia about x")
	t.near(inv.y, want.y, 0.5, "inverse inertia about y")
	t.near(inv.z, want.z, 0.5, "inverse inertia about z")
	# The headline regression: a dynamic box must not report itself unspinnable.
	t.gt(inv.length(), 0.0, "a dynamic body must report a non-zero inverse inertia")


func the_tensor_turns_with_the_body_and_the_diagonal_does_not(t) -> void:
	await _tree.physics_frame
	var want := _expected_inv()
	var st := _state(_turned)

	# Body frame: unmoved by the quarter turn.
	var inv := st.inverse_inertia
	t.near(inv.x, want.x, 0.5, "the body-frame diagonal ignores the body's rotation")
	t.near(inv.y, want.y, 0.5, "the body-frame diagonal ignores the body's rotation")

	# World frame: x and y exchanged by the quarter turn about z.
	var m := st.inverse_inertia_tensor
	t.near(m[0][0], want.y, 0.5, "turned about z, world xx carries the body's y value")
	t.near(m[1][1], want.x, 0.5, "turned about z, world yy carries the body's x value")
	t.near(m[2][2], want.z, 0.5, "the spin axis entry is untouched")
	# A quarter turn leaves the tensor diagonal; anything off it means the rotation was
	# applied one-sided (R·diag, not R·diag·Rᵀ) or transposed.
	t.lt(absf(m[0][1]), 0.5, "a quarter turn leaves no off-diagonal term")
	t.lt(absf(m[0][2]), 0.5, "a quarter turn leaves no off-diagonal term")


func a_rotation_locked_body_reports_no_inertia(t) -> void:
	await _tree.physics_frame
	# Zero is the documented marker for "never spins", and the one place the old
	# hard-coded zero was accidentally right.
	t.lt(_state(_locked).inverse_inertia.length(), 0.001,
		"a rotation-locked body reports zero inverse inertia")


func principal_axes_are_the_body_axes(t) -> void:
	await _tree.physics_frame
	# hop holds inertia as a body-frame diagonal, so the principal axes are the body's
	# own. On the quarter-turned body that is a real claim: its first axis points along
	# world +y.
	var axes := _state(_turned).principal_inertia_axes
	t.near(axes.x.y, 1.0, 0.01, "the turned body's first principal axis is world +y")
	t.lt(absf(axes.x.x), 0.01, "and has no world-x component left")
