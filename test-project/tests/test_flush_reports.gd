extends RefCounted
## What _flush_queries reports back to the scene.
##
## A kinematic body is called back only when its transform or velocity changed since the last
## report, as GodotPhysics reports only active bodies; the step that stops it still reports, or
## a rider keeps reading the old carry velocity. Areas still see enter and exit exactly once.

const SIZE := Vector3(1.0, 1.0, 1.0)
const START := Vector3(0.0, 10.0, 0.0)
const CREEP := Vector3(0.05, 0.0, 0.0)

var _tree: SceneTree
var _root: Node3D
var _body := RID()
var _shape: BoxShape3D
var _reports := 0
var _last_velocity := Vector3.ZERO


func _init(tree: SceneTree) -> void:
	_tree = tree


func setup() -> void:
	_root = Node3D.new()
	_tree.get_root().add_child(_root)
	_shape = BoxShape3D.new()
	_shape.size = SIZE
	_reports = 0
	_last_velocity = Vector3.ZERO


func teardown() -> void:
	if _body.is_valid():
		PhysicsServer3D.free_rid(_body)
	_body = RID()
	if is_instance_valid(_root):
		_root.free()
	_root = null


func tests() -> Array:
	return [
		"kinematic_reports_only_when_moved",
		"area_sees_body_enter_and_exit_once",
		"area_sees_area_enter_and_exit_once",
		"still_area_sees_a_layer_change_inside_it",
		"still_area_sees_a_body_spawned_inside_it",
		"area_moved_by_a_callback_sees_what_it_lands_on",
	]


func _on_state(state: PhysicsDirectBodyState3D) -> void:
	_reports += 1
	_last_velocity = state.linear_velocity


func _place(at: Vector3) -> void:
	PhysicsServer3D.body_set_state(_body, PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(), at))


func kinematic_reports_only_when_moved(t) -> void:
	_body = PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(_body, PhysicsServer3D.BODY_MODE_KINEMATIC)
	PhysicsServer3D.body_add_shape(_body, _shape.get_rid())
	PhysicsServer3D.body_set_space(_body, _root.get_world_3d().space)
	PhysicsServer3D.body_set_state_sync_callback(_body, _on_state)
	var at := START
	_place(at)
	for i in 4:
		at += CREEP
		_place(at)
		await _tree.physics_frame
	t.gt(_reports, 2, "a moving kinematic body is reported")
	t.gt(_last_velocity.x, 0.0, "with its carry velocity")
	for i in 3:
		await _tree.physics_frame
	t.near(_last_velocity.length(), 0.0, 0.0001, "the step that stops it is reported")
	var still := _reports
	for i in 10:
		await _tree.physics_frame
	t.near(_reports, still, 0.0, "a still kinematic body is not reported")
	_place(at + CREEP)
	await _tree.physics_frame
	await _tree.physics_frame
	t.gt(_reports, still, "moving it again is")


func _area() -> Area3D:
	var a := Area3D.new()
	var cs := CollisionShape3D.new()
	cs.shape = _shape
	a.add_child(cs)
	_root.add_child(a)
	a.global_position = START
	return a


func _count_into(sig: Signal, counts: Dictionary, key: String) -> void:
	counts[key] = 0
	sig.connect(func(_n): counts[key] += 1)


func area_sees_body_enter_and_exit_once(t) -> void:
	var area := _area()
	var mover := AnimatableBody3D.new()
	mover.sync_to_physics = false
	var cs := CollisionShape3D.new()
	cs.shape = _shape
	mover.add_child(cs)
	_root.add_child(mover)
	mover.global_position = START + Vector3(5, 0, 0)
	var counts := {}
	_count_into(area.body_entered, counts, "in")
	_count_into(area.body_exited, counts, "out")
	await _tree.physics_frame
	await _tree.physics_frame
	t.near(counts["in"], 0, 0.0, "nothing inside yet")
	mover.global_position = START
	for i in 5:
		await _tree.physics_frame
	t.near(counts["in"], 1, 0.0, "entered once, not once per flush")
	t.near(counts["out"], 0, 0.0, "and is still inside")
	mover.global_position = START + Vector3(5, 0, 0)
	for i in 3:
		await _tree.physics_frame
	t.near(counts["out"], 1, 0.0, "exited once")


func area_sees_area_enter_and_exit_once(t) -> void:
	var detector := _area()
	var other := _area()
	other.global_position = START + Vector3(5, 0, 0)
	var counts := {}
	_count_into(detector.area_entered, counts, "in")
	_count_into(detector.area_exited, counts, "out")
	await _tree.physics_frame
	await _tree.physics_frame
	t.near(counts["in"], 0, 0.0, "nothing inside yet")
	other.global_position = START
	for i in 5:
		await _tree.physics_frame
	t.near(counts["in"], 1, 0.0, "entered once, not once per flush")
	t.near(counts["out"], 0, 0.0, "and is still inside")
	other.global_position = START + Vector3(5, 0, 0)
	for i in 3:
		await _tree.physics_frame
	t.near(counts["out"], 1, 0.0, "exited once")


func _static_body(at: Vector3) -> StaticBody3D:
	var b := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	cs.shape = _shape
	b.add_child(cs)
	_root.add_child(b)
	b.global_position = at
	return b


## Neither the area nor the body moves: only the fingerprint can tell the area to look again.
func still_area_sees_a_layer_change_inside_it(t) -> void:
	var area := _area()
	var body := _static_body(START)
	var counts := {}
	_count_into(area.body_entered, counts, "in")
	_count_into(area.body_exited, counts, "out")
	for i in 3:
		await _tree.physics_frame
	t.near(counts["in"], 1, 0.0, "the body starts inside")
	body.collision_layer = 2
	for i in 3:
		await _tree.physics_frame
	t.near(counts["out"], 1, 0.0, "leaving the area's mask reads as an exit")
	body.collision_layer = 1
	for i in 3:
		await _tree.physics_frame
	t.near(counts["in"], 2, 0.0, "and rejoining it as an entry")


func still_area_sees_a_body_spawned_inside_it(t) -> void:
	var area := _area()
	var counts := {}
	_count_into(area.body_entered, counts, "in")
	for i in 3:
		await _tree.physics_frame
	_static_body(START)
	for i in 3:
		await _tree.physics_frame
	t.near(counts["in"], 1, 0.0, "a body that appears inside is seen")


## A handler that runs inside the flush moves another area onto a still body: nothing moved
## before the flush looked, so only the re-check after the callbacks can catch it.
func area_moved_by_a_callback_sees_what_it_lands_on(t) -> void:
	var trigger := _area()
	var carried := _area()
	var far := START + Vector3(0, 0, 20)
	carried.global_position = START + Vector3(0, 0, -20)
	_static_body(far)
	var counts := {}
	_count_into(carried.body_entered, counts, "in")
	trigger.body_entered.connect(func(_b):
		PhysicsServer3D.area_set_transform(carried.get_rid(), Transform3D(Basis(), far)))
	for i in 3:
		await _tree.physics_frame
	t.near(counts["in"], 0, 0.0, "nothing under it yet")
	_static_body(START)
	for i in 4:
		await _tree.physics_frame
	t.near(counts["in"], 1, 0.0, "moved mid-flush, it still sees the body it landed on")
