extends RefCounted
## A mover pressed against geometry that is itself MOVING.
##
## test_bsp_body_test_motion covers walls that stay put. This covers the case a host
## engine's riders actually live in: a brush entity advancing into them while they lean
## on it — a lift wall, a train, the cockpit of WizardWars' walking golem. Nothing in a
## static fixture can reach it, because the whole question is which pose a sweep is
## answered against when the geometry moved earlier in the same frame.
##
## The invariant: a mover that presses into an advancing wall rests OUTSIDE it, by about
## the margin, every tick — never inside. Resting inside is not cosmetic. One margin
## inside a wall standing on a floor is past the floor's edge, so the next push-out can
## send the rider the wrong way: that is how the golem's riders ended up behind the
## cockpit's rear wall and out of the golem.
##
## The one rule it takes, and the reason this is a suite of its own: WHEN THE RIDER
## SWEEPS, THE NODE AND THE PHYSICS SERVER MUST HOLD THE SAME POSE. Godot queues a
## transform written to a Node3D and flushes it to the server only after every
## _physics_process has run, so a mover that writes mid-frame and a rider that ticks
## after it disagree by a whole step: the carry moves the rider against the node's new
## pose while the sweep is answered against the server's old one. The sweep stops a
## margin short of a wall that has since advanced, and the tick ends with the rider
## (advance - margin) deep inside. hop is answering honestly; it is being told one pose
## while the rider is moved by another.
##
## Which is why both tests below drive the SAME wall two ways — a transform written in
## the tick and flushed (force_update_transform), and a tween, which needs no flush
## because its write already lands after every _physics_process — and demand the same
## answer. Neither way is privileged; the rule is the agreement, not the mechanism.
## tests/probe_wall_press.gd is the same measurement knob by knob.
##
## COVERAGE, measured by dropping the flush and re-running:
##   * the written-in-the-tick half of an_advancing_wall_never_leaves_its_rider_inside_it
##     fails at 0.399 units inside — one margin, to three decimals, at 0.83 u/tick.
##   * the same half of a_wall_faster_than_the_margin_still_keeps_its_rider_out fails at
##     1.351 inside.
##   * the tween half of both is unaffected, as is the still-wall control. That contrast
##     is the measurement: it is the disagreement that costs penetration, not the motion.
## Not pinned to a past bug, but the invariants these assert: that _body_test_motion still
## leaves its margin-sized gap (step 3) and that recovery still clears the overlap each
## advance creates. Both would show up here as a rider resting flush or sinking a margin
## a tick.


const BspFixture = preload("res://tests/bsp_fixture.gd")
const BspScene = preload("res://tests/bsp_scene.gd")

const S := BspScene.S
const MARGIN := 0.01         # a typical CharacterBody3D safe_margin: 0.4 GoldSrc units
const WALL_FACE := 8.0       # expanded for hull 1; inside the wall is x < this
const START_X := 20.0        # feet, 12 units clear of the face
const PRESS := 226.0 / 60.0  # a player's backpedal, GoldSrc units per tick
const STRIDE := 0.83         # one tick of ww_golem's stride, GoldSrc units

var _map: Node3D
var _world: Node3D
var _mover: CharacterBody3D
var _tree: SceneTree


func _init(tree: SceneTree) -> void:
	_tree = tree


func setup() -> void:
	# A wall standing on a floor, on a KINEMATIC carrier: the rider rests on the deck and
	# leans on the wall, and the seam the two share is where a standing hull's trace point
	# lives. The golem's cockpit in miniature.
	_map = BspScene.make_map(_tree, BspFixture.wall_on_floor(), true)
	_world = _map.get_node("World")
	var cap := CapsuleShape3D.new()
	cap.radius = 16 * S
	cap.height = 72 * S
	_mover = BspScene.add_mover(_map, cap, Vector3(0, 36 * S, 0), MARGIN)
	_mover.floor_block_on_wall = false
	_mover.global_position = BspScene.gs(START_X, 0.0, 0.0)


func teardown() -> void:
	if is_instance_valid(_map):
		_map.free()
	_map = null
	_world = null
	_mover = null


## A fresh fixture for the second half of a test. Freeing the first is not optional: a
## leftover map keeps its BSP carrier in the tree, and every later suite then traces
## against two fixtures at once.
func _rebuild() -> void:
	teardown()
	setup()


## The mover's feet in the WALL's own frame — the only frame a penetration means anything
## in once the wall is moving.
func _local_x() -> float:
	var l: Vector3 = _world.global_transform.affine_inverse() * _mover.global_position
	return BspScene.to_gs(l).x


## Press into the wall for `ticks` while the wall advances `advance` units a tick into the
## mover, carrying it along as a deck would. Returns the deepest penetration reached in
## GoldSrc units — negative means it never got inside, which is the whole point.
##
## `tweened` picks how the wall's transform is written: by a tween, which writes after
## every _physics_process, or in the tick itself followed by the flush that makes the
## server agree. Both must answer the same.
func _ride(ticks: int, advance: float, tweened := false) -> float:
	var tween: Tween
	if tweened and advance != 0.0:
		tween = _world.create_tween()
		tween.set_process_mode(Tween.TWEEN_PROCESS_PHYSICS)
		tween.tween_property(_world, "global_position",
			_world.global_position + BspScene.gs(advance * ticks, 0.0, 0.0),
			float(ticks) / 60.0).set_trans(Tween.TRANS_LINEAR)

	var deepest := -1e9
	for _i in ticks:
		if advance != 0.0:
			if not tweened:
				_world.global_position += BspScene.gs(advance, 0.0, 0.0)
				_world.force_update_transform()
			_mover.move_and_collide(BspScene.gs(advance, 0.0, 0.0))  # the carry
		_mover.velocity = BspScene.gs(-PRESS, 0.0, 0.0) * 60.0
		_mover.move_and_slide()
		deepest = maxf(deepest, WALL_FACE - _local_x())
		await _tree.physics_frame
	if tween:
		tween.kill()
	return deepest


func tests() -> Array:
	return [
		"a_wall_that_stays_put_holds_its_rider_a_margin_clear",
		"an_advancing_wall_never_leaves_its_rider_inside_it",
		"a_wall_faster_than_the_margin_still_keeps_its_rider_out",
	]


func a_wall_that_stays_put_holds_its_rider_a_margin_clear(t) -> void:
	# The control. A rider pressing into a wall that is not moving rests a margin off it:
	# not flush (the next advance would bury it) and not a margin INSIDE (the old bug).
	var deepest: float = await _ride(30, 0.0)
	t.lt(deepest, 0.0, "a mover pressing a still wall never ends a tick inside it")
	t.near(-deepest, MARGIN / S, 0.2, "and rests about a margin off it")


func an_advancing_wall_never_leaves_its_rider_inside_it(t) -> void:
	# The golem cockpit: a 0.83 u/tick stride, a 226 u/s backpedal into the rear wall, and
	# a margin of 0.4 units. Every tick of this used to end 0.4 units inside the wall.
	var written: float = await _ride(60, STRIDE)
	t.lt(written, 0.0, "a wall written in the tick keeps its rider outside it")
	_rebuild()
	await _tree.physics_frame
	var tweened: float = await _ride(60, STRIDE, true)
	t.lt(tweened, 0.0, "and so does the same wall driven by a tween")
	t.near(written, tweened, 0.05, "the two ways of writing it answer the same")


func a_wall_faster_than_the_margin_still_keeps_its_rider_out(t) -> void:
	# The margin is not what holds the rider out — a wall moving well past it must not
	# change the answer. At 2 units a tick the old disagreement buried the rider 1.35 deep.
	var written: float = await _ride(40, 2.0)
	t.lt(written, 0.0, "a fast wall written in the tick keeps its rider outside it")
	_rebuild()
	await _tree.physics_frame
	var tweened: float = await _ride(40, 2.0, true)
	t.lt(tweened, 0.0, "and so does the same fast wall driven by a tween")
