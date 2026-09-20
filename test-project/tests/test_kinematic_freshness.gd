extends RefCounted
## Where a kinematic body is, to everything that asks between two physics steps.
##
## hop does not teleport a kinematic body to its new transform: _step prescribes a
## velocity that sweeps it there over the tick, so a door pushes what stands in it and a
## platform carries its rider instead of passing through them. The sweep needs the place
## the body started from, and it used to read that off the hop solid — which meant the
## solid was deliberately left BEHIND the body, at the last place it was stepped, for the
## whole of the frame in which the body moved. Every query is served out of that solid,
## so a ray or a shape cast fired in the same _physics_process that moved a mover looked
## for it where it no longer was: the frame a body is placed is exactly when game code
## tends to ask about it.
##
## _step now keeps the start point itself (HopBodyData::sweep_prev_transform), so the
## solid can stand where the body stands and still be swept from where it was.
##
## COVERAGE, measured by reverting the fix:
##   * found_where_it_was_just_put fails on both halves — the ray misses the body's new
##     place and hits its old one, one frame stale.
##   * the other two are the guard on the way the fix works, not the fix: they fail if
##     the sweep is dropped (a mover that teleports pushes nothing, and publishes no
##     carry velocity) or if the start point stops advancing (velocity grows without
##     bound as the body's whole travel is re-swept every step).

const SIZE := Vector3(1.0, 1.0, 1.0)
const START := Vector3(0.0, 10.0, 0.0)
const HOP := Vector3(4.0, 0.0, 0.0)     # a jump big enough that old and new never overlap
const CREEP := Vector3(0.05, 0.0, 0.0)  # 3 m/s at 60 Hz — a sweep, not a teleport

var _tree: SceneTree
var _root: Node3D
var _mover: AnimatableBody3D
var _blocker: RigidBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


func setup() -> void:
	_root = Node3D.new()
	_tree.get_root().add_child(_root)

	_mover = AnimatableBody3D.new()
	# sync_to_physics would take the transform back out of our hands (it drives the body
	# from the server's own copy and drops writes made in _physics_process).
	_mover.sync_to_physics = false
	_mover.add_child(_shape())
	_root.add_child(_mover)
	_mover.global_position = START

	# Parked in the mover's path, weightless so nothing but the mover can move it.
	_blocker = RigidBody3D.new()
	_blocker.gravity_scale = 0.0
	_blocker.add_child(_shape())
	_root.add_child(_blocker)
	_blocker.global_position = START + Vector3(1.4, 0.0, 0.0)


func _shape() -> CollisionShape3D:
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = SIZE
	cs.shape = box
	return cs


func teardown() -> void:
	if is_instance_valid(_root):
		_root.free()
	_root = null
	_mover = null
	_blocker = null


## A ray dropped through `at`, answered by whatever the server has right now.
func _hits(at: Vector3) -> bool:
	var q := PhysicsRayQueryParameters3D.create(at + Vector3(0, 2, 0), at - Vector3(0, 2, 0))
	q.exclude = [_blocker.get_rid()]
	return not _root.get_world_3d().direct_space_state.intersect_ray(q).is_empty()


func tests() -> Array:
	return [
		"found_where_it_was_just_put",
		"still_pushes_what_stands_in_its_way",
		"publishes_one_step_of_carry_velocity",
	]


func found_where_it_was_just_put(t) -> void:
	# No await: the move and the query are in the same frame, as they are when one node
	# opens a door and another traces against it. force_update_transform is how a node's
	# move reaches the server before the frame's own flush — without it the server has
	# not been told yet and this would be measuring Godot's deferral, not hop.
	_mover.global_position = START + HOP
	_mover.force_update_transform()
	t.near(1.0 if _hits(START + HOP) else 0.0, 1.0, 0.5, "the body is where it was moved to")
	t.near(1.0 if _hits(START) else 0.0, 0.0, 0.5, "and is no longer where it was")


func still_pushes_what_stands_in_its_way(t) -> void:
	# The reason a kinematic body is swept at all. It has to cross the 0.4 m gap first,
	# so the frames are spent before anything can show.
	var before := _blocker.global_position.x
	for i in 20:
		_mover.global_position += CREEP
		await _tree.physics_frame
	t.gt(_blocker.global_position.x - before, 0.1,
		"20 steps of a mover creeping into a weightless box should shove it along")


func publishes_one_step_of_carry_velocity(t) -> void:
	# What a CharacterBody3D rider reads to carry itself on a platform: ONE step of
	# motion, every step. It stays put only because the start point advances with the
	# body — a start point that stuck would re-sweep the whole journey each time.
	var expected := CREEP.x * Engine.physics_ticks_per_second
	for i in 4:
		_mover.global_position += CREEP
		await _tree.physics_frame
		var v: Vector3 = PhysicsServer3D.body_get_state(
			_mover.get_rid(), PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY)
		t.near(v.x, expected, expected * 0.05, "step %d of an even creep" % (i + 1))
