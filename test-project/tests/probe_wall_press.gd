extends SceneTree
## Press a hull into a static wall, tick after tick, and print where it rests.
##
## Reduced from WizardWars' ww_golem cockpit, where a rider holding "back" against the
## cockpit's rear wall sits a steady 0.4 units — exactly one safe_margin — INSIDE it,
## tick after tick, until one stride's bob cashes that penetration in and ejects them.
## Everything about the golem was ruled out there: the carry lands in full, the
## depenetration lands in full, the lift is free. What is left is a single step, and it
## needs no golem: a sweep into a wall, from just outside it, ending up inside it.
##
## Nothing here moves but the mover, and the wall is static BSP geometry, so whatever
## this prints belongs to _body_test_motion (its margin back-off, or its flush-start
## retry) or to CharacterBody3D's slide loop on top of it — MODE says which.
##
##   godot --headless --path test-project -s res://tests/probe_wall_press.gd
##
##   FIXTURE  wall (default, a wall standing on a floor — the golem's shape: the rider
##            rests on the deck and the seam between the two is where they live),
##            pillar (a free-standing slab, no floor, no seam), corner
##   MODE     slide (default, CharacterBody3D.move_and_slide — what locomotion uses),
##            collide (move_and_collide of the tick's motion — what the carry uses),
##            query (apply body_test_motion's own travel, no Godot layer at all)
##   SPEED    press speed, GoldSrc units/sec (default 226, the rider's backpedal)
##   MARGIN   the mover's safe_margin, metres (default 0.01, what player.tscn ships)
##   START    feet x, GoldSrc, to start from (default 20 — 12 units out from the face)
##   FEET     feet z, GoldSrc (default 0, resting on the floor)
##   TICKS    how many ticks to press for (default 60)
##   ADVANCE  units/tick the WALL travels, along +x, i.e. toward the mover (default 0 —
##            a static wall). 0.83 is one tick of ww_golem's stride. The carrier becomes
##            kinematic, so the geometry the mover leans on is now MOVING, which is the
##            one ingredient a static wall has not got.
##   FLUSH    1 = force the wall's freshly written transform through to the physics
##            server before anything queries it (Node3D.force_update_transform). A
##            transform written to a node during _physics_process is otherwise queued
##            and flushed only after EVERY node's _physics_process has run, so a mover
##            that ticks before its rider is answered to the rider one step stale.
##   CARRY    1 = also move the mover by the wall's own displacement, as a swept move,
##            before it presses — the way a host engine carries a rider standing on a
##            moving brush. With it the wall and the mover travel together and the only
##            relative motion is the press, which is the golem cockpit exactly. Without
##            it the wall simply runs the mover down.

## Runs the tick INSIDE a physics step, which is where a host engine's movers and
## movement code both run. A probe that worked between two steps instead (await
## physics_frame, then write a transform) asks its questions in the gap where the
## geometry has been placed but not yet stepped, and that gap is not a tick anything
## real lives in.
class TickDriver:
	extends Node
	signal ticked

	func _physics_process(_delta: float) -> void:
		ticked.emit()


const BspFixture = preload("res://tests/bsp_fixture.gd")
const BspScene = preload("res://tests/bsp_scene.gd")

const S := BspScene.S

## The wall's face in the EXPANDED tree a hull-1 mover is traced against: the brush's
## -8 grown by the hull's 16. Inside the wall is x < this, so penetration is face - x.
const WALL_FACE := 8.0


func _env(name: String, dflt: float) -> float:
	var v := OS.get_environment(name)
	return float(v) if v != "" else dflt


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var fixture := OS.get_environment("FIXTURE")
	var mode := OS.get_environment("MODE")
	if mode == "":
		mode = "slide"
	var speed := _env("SPEED", 226.0)
	var margin := _env("MARGIN", 0.01)
	var start_x := _env("START", 20.0)
	var feet_z := _env("FEET", 0.0)
	var ticks := int(_env("TICKS", 60.0))
	var advance := _env("ADVANCE", 0.0)
	var carry := OS.get_environment("CARRY") == "1"
	var flush := OS.get_environment("FLUSH") == "1"

	var blob: PackedByteArray
	match fixture:
		"pillar":
			blob = BspFixture.pillar()
		"corner":
			blob = BspFixture.corner_on_floor()
		_:
			fixture = "wall"
			blob = BspFixture.wall_on_floor()

	var map := BspScene.make_map(self, blob, advance != 0.0)
	var world := map.get_node("World") as Node3D
	var cap := CapsuleShape3D.new()
	cap.radius = 16 * S
	cap.height = 72 * S
	var mover := BspScene.add_mover(map, cap, Vector3(0, 36 * S, 0), margin)
	mover.floor_block_on_wall = false  # what the player ships
	mover.global_position = BspScene.gs(start_x, 0.0, feet_z)
	await physics_frame

	# GoldSrc -x is into the wall; one tick of it, as a Godot vector.
	var step := speed / 60.0
	var press := BspScene.gs(-step, 0.0, 0.0)

	print("\nfixture=%s mode=%s speed=%.0f u/s (%.2f u/tick) margin=%.4f m (%.2f u)" % [
		fixture, mode, speed, step, margin, margin / S])
	print("wall face x=%.2f (expanded, hull 1); feet start (%.2f, 0, %.2f)" % [
		WALL_FACE, start_x, feet_z])
	print("wall advance %.2f u/tick, carry %s, flush %s\n" % [
		advance, "on" if carry else "off", "on" if flush else "off"])

	var probe := PhysicsTestMotionParameters3D.new()
	probe.margin = margin
	probe.recovery_as_collision = true
	probe.max_collisions = 4
	var res := PhysicsTestMotionResult3D.new()

	# The mover's pose in the WALL's own frame — the only frame the penetration means
	# anything in once the wall is moving.
	var local := func() -> Vector3:
		var l: Vector3 = world.global_transform.affine_inverse() * mover.global_position
		return BspScene.to_gs(l)

	# The same pose in the frame the PHYSICS SERVER has for the wall, which is not
	# necessarily the frame the node has: a transform written to a Node3D is queued and
	# flushed, and a query made before that flush is answered against the old place.
	var served := func() -> Vector3:
		var t: Transform3D = PhysicsServer3D.body_get_state(
			world.get_rid(), PhysicsServer3D.BODY_STATE_TRANSFORM)
		var l: Vector3 = t.affine_inverse() * mover.global_position
		return BspScene.to_gs(l)

	var driver := TickDriver.new()
	get_root().add_child(driver)

	var worst := 0.0
	for tick in ticks:
		await driver.ticked
		# The wall moves FIRST, as a host engine's entities do (they are added to the
		# tree before the player, so Godot ticks them first). A mover that ticked first
		# would read a stale wall, which is its own class of bug and not this one.
		if advance != 0.0:
			world.global_position += BspScene.gs(advance, 0.0, 0.0)
			if flush:
				world.force_update_transform()
			if carry:
				# What a rider standing on the moving brush is given: the wall's own
				# displacement, as a collision-checked move.
				mover.move_and_collide(BspScene.gs(advance, 0.0, 0.0))

		var before: Vector3 = local.call()

		# What hop answers for the move about to be made, and the depenetration it
		# would fold in first (a zero-motion query's travel IS that recovery).
		probe.from = mover.global_transform
		probe.motion = press
		PhysicsServer3D.body_test_motion(mover.get_rid(), probe, res)
		var unsafe := res.get_collision_unsafe_fraction()
		var safe := res.get_collision_safe_fraction()
		var says := BspScene.to_gs(res.get_travel()).x
		var depth := 0.0
		for i in res.get_collision_count():
			depth = maxf(depth, res.get_collision_depth(i) / S)
		# Where the wall is, as far as the server (and so that query) is concerned.
		var served_x: float = served.call().x

		match mode:
			"collide":
				mover.move_and_collide(press)
			"query":
				probe.from = mover.global_transform
				probe.motion = press
				PhysicsServer3D.body_test_motion(mover.get_rid(), probe, res)
				mover.global_position += res.get_travel()
			_:
				mover.velocity = press * 60.0
				mover.move_and_slide()

		var after: Vector3 = local.call()
		var pen := WALL_FACE - after.x
		worst = maxf(worst, pen)
		print("t%3d  x %8.3f -> %8.3f  pen %7.3f  moved %7.3f (hop said %7.3f)  unsafe %.3f safe %.3f  depth %6.3f  served x %8.3f (pen %7.3f)" % [
			tick, before.x, after.x, pen, after.x - before.x, says, unsafe, safe, depth,
			served_x, WALL_FACE - served_x])

	print("\ndeepest resting penetration: %.3f units (%.2f margins)" % [
		worst, worst / (margin / S)])
	map.free()
	quit(0)
