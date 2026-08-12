extends RefCounted
## The scene half of a BSP test fixture: a carrier body holding a blob, and the
## Godot<->GoldSrc conversions every suite states its geometry in.
##
## bsp_fixture.gd builds the BYTES; this builds the nodes that make a PhysicsServer3D
## treat them as collision. They are separate because the byte layout is shared with
## the C++ fixture and the scene contract is not.
##
## The carrier contract is a documented footgun and the reason this is one file rather
## than a paste in each suite:
##   * the blob lives on an ANCESTOR under "bsp_data", and the hull is built when a
##     carrier body enters the tree — setting the meta afterwards is a silent no-op;
##   * each carrier carries "bsp_model" (int) and "bsp_scale" (float). Scale has no
##     default on purpose: a missing one means the two sides disagree about units;
##   * a carrier still needs one enabled CollisionShape3D. The hull stands in for it,
##     but a body with no enabled shape is intangible by design — that is how a door
##     goes non-solid.

## A host engine's metres-per-GoldSrc-unit. Matches WizardWars.
const S := 0.025


## GoldSrc (x, y, z) -> Godot. Godot +x is GoldSrc -x; Godot y is GoldSrc z.
static func gs(x: float, y: float, z: float) -> Vector3:
	return Vector3(-x * S, z * S, y * S)


## Godot -> GoldSrc, for reading a travel or a position back out.
static func to_gs(v: Vector3) -> Vector3:
	return Vector3(-v.x / S, v.z / S, v.y / S)


## A map node with `blob` mounted on it, added to `tree`, with one static carrier for
## model 0. Free the returned node to tear the whole fixture down.
static func make_map(tree: SceneTree, blob: PackedByteArray) -> Node3D:
	var map := Node3D.new()
	map.set_meta("bsp_data", blob)
	tree.get_root().add_child(map)

	var world := StaticBody3D.new()
	world.collision_layer = 1
	world.set_meta("bsp_model", 0)
	world.set_meta("bsp_scale", S)
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.1, 0.1, 0.1)
	cs.shape = box
	world.add_child(cs)
	map.add_child(world)
	return map


## A mover with `shape`, parented to `map`, ready for body_test_motion.
static func add_mover(map: Node3D, shape: Shape3D, offset := Vector3.ZERO,
		margin := 0.001) -> CharacterBody3D:
	var body := CharacterBody3D.new()
	body.collision_layer = 0
	body.collision_mask = 1
	body.safe_margin = margin
	var cs := CollisionShape3D.new()
	cs.shape = shape
	cs.position = offset
	body.add_child(cs)
	map.add_child(body)
	return body


## One body_test_motion from a GoldSrc-space position with a GoldSrc-space motion.
## Travel comes back in GoldSrc units, so assertions read in the numbers the geometry
## is stated in.
static func probe(body: CharacterBody3D, from_gs: Vector3, motion_gs: Vector3,
		margin := 0.001, max_collisions := 0) -> Dictionary:
	var xform := Transform3D(Basis.IDENTITY, gs(from_gs.x, from_gs.y, from_gs.z))
	body.global_transform = xform
	var p := PhysicsTestMotionParameters3D.new()
	p.from = xform
	p.motion = gs(motion_gs.x, motion_gs.y, motion_gs.z)
	p.margin = margin
	if max_collisions > 0:
		p.recovery_as_collision = true
		p.max_collisions = max_collisions
	var r := PhysicsTestMotionResult3D.new()
	PhysicsServer3D.body_test_motion(body.get_rid(), p, r)
	return {
		"unsafe": r.get_collision_unsafe_fraction(),
		"travel_gs": to_gs(r.get_travel()),
		"contacts": r.get_collision_count(),
	}
