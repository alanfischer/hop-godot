extends RefCounted
## BODY_PARAM_ANGULAR_DAMP, which hop has no equivalent of.
##
## hop's only drag is a fluid force on LINEAR velocity, so the angular parameter was
## stored on the body and never read: anything given spin kept all of it for as long as
## it lived. Debris is what notices — a gib is thrown with a random tumble and set to
## damp out of it, and under hop it span at its launch rate until it faded, where the
## same gib settles under GodotPhysics3D.
##
## Measured in free flight, one physics frame after the bodies reach the server (all the
## runner grants), so nothing but the damping has touched ω: gravity is linear, and
## there is no geometry to contact.
##
## COVERAGE, measured by reverting the fix: loses_spin_at_godot_s_rate fails with the
## ratio at 1.0 — the undamped control passes either way and exists to prove the frame
## count, not the fix.

const SPIN := Vector3(0.0, 10.0, 0.0)
const DAMP := 2.0
const DT := 1.0 / 60.0

var _tree: SceneTree
var _root: Node3D
var _damped: RigidBody3D
var _free: RigidBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


## A spinning box in empty space. No collision mask, so the pair can be parked anywhere
## and still never see each other or the world.
func _spinner(damp: float) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.collision_layer = 0
	body.collision_mask = 0
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.15, 0.08, 0.12)
	cs.shape = box
	body.add_child(cs)
	_root.add_child(body)
	body.angular_damp = damp
	body.angular_velocity = SPIN
	return body


func setup() -> void:
	_root = Node3D.new()
	_tree.get_root().add_child(_root)
	_damped = _spinner(DAMP)
	_free = _spinner(0.0)


func teardown() -> void:
	if is_instance_valid(_root):
		_root.free()
	_root = null
	_damped = null
	_free = null


func tests() -> Array:
	return [
		"loses_spin_at_godot_s_rate",
		"zero_damp_keeps_every_bit_of_its_spin",
		"damping_does_not_turn_the_spin_axis",
	]


func loses_spin_at_godot_s_rate(t) -> void:
	# Godot's own integrator does ω *= max(0, 1 − damp·dt) once per step, and the number
	# is pinned rather than merely "less than before" because a body that damps at the
	# wrong rate is exactly as wrong as one that never damps — debris that stops dead on
	# landing reads as stuck, not settled.
	t.near(_damped.angular_velocity.length() / _free.angular_velocity.length(),
		1.0 - DAMP * DT, 0.005,
		"one step of angular_damp = %s should scale ω by %s" % [DAMP, 1.0 - DAMP * DT])


func zero_damp_keeps_every_bit_of_its_spin(t) -> void:
	# The control. It also fixes the frame count the test above divides by: if the runner
	# ever grants a different number of steps, the ratio moves and this stays put.
	t.near(_free.angular_velocity.length(), SPIN.length(), 0.001,
		"a body with no angular damp is not slowed by anything")


func damping_does_not_turn_the_spin_axis(t) -> void:
	# Damping is a scale, not a torque. Cheap to get wrong by damping in the body frame
	# of a body whose orientation has already moved on.
	var w := _damped.angular_velocity
	t.near(w.normalized().dot(SPIN.normalized()), 1.0, 0.001,
		"angular damping scales ω, it does not rotate it")
