extends RefCounted
## Small movers — projectiles — against a GoldSrc BSP hull.
##
## A projectile is not a player, and the hull trace used to treat it as a badly-fitting
## one. Two defects compounded:
##
##   * anything 8..32 units wide was handed hull 3, a 32x32x36 box several times its
##     size, and the trace point was placed at a CORNER of the mover's box (GoldSrc's
##     `mins - clip_mins`, which only lines up when the two boxes match);
##   * `margin` was applied by biasing every plane distance, which grows the solid on
##     planes whose back side is solid and shrinks it on the others — a brush does not
##     inflate under that, it TRANSLATES.
##
## Both are invisible against a wall you can only reach from one side: the error is the
## same every time and reads as a deliberate standoff. So every test here measures the
## SAME slab from both faces and compares. A 12-unit satchel used to stop 6 units short
## of one face and 26 short of the other.
##
## COVERAGE, measured by reverting each fix and re-running, not assumed:
##   caught  the plane-bias margin — stops_the_same_distance_from_either_face fails on
##           EVERY row, with the far-side gap going negative: the mover ends up inside
##           the wall, which is the signature of a translated brush rather than an
##           inflated one
##   caught  the old `sx <= 8` hull rule — the same test fails from 0.15 m up, at
##           exactly the 6-units-near / 26-units-far a satchel showed in the game
##   NOT caught: player_sized_movers_still_get_a_sized_hull passes under BOTH hull
##           rules. It pins the threshold's upper side against future edits, not
##           against the old bug.

const BspFixture = preload("res://tests/bsp_fixture.gd")
const BspScene = preload("res://tests/bsp_scene.gd")

const S := BspScene.S
const FACE := 8.0        # the slab's real faces, GoldSrc x = +/- 8
const STANDOFF := 0.07   # DIST_EPSILON + the server's resting gap, in GoldSrc units

## Every radius a test below asks for. The bodies are all built in setup(), because the
## runner grants exactly one physics frame between setup() and the test — a body added
## mid-test has not reached the server yet, and querying one that the server has never
## seen a shape for does not fail, it hangs.
const RADII: Array[float] = [0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4]

var _map: Node3D
var _tree: SceneTree
var _bodies: Array[CharacterBody3D] = []   # parallel to RADII
var _player: CharacterBody3D


func _init(tree: SceneTree) -> void:
	_tree = tree


func setup() -> void:
	_map = BspScene.make_map(_tree, BspFixture.pillar())

	_bodies.clear()
	for i in RADII.size():
		var sph := SphereShape3D.new()
		sph.radius = RADII[i]
		var body := BspScene.add_mover(_map, sph)
		# Parked far apart so no two of them can see each other, whatever else changes.
		body.global_position = BspScene.gs(0, 4096 * (i + 1), -4096)
		_bodies.append(body)

	# A standing player: 33 units wide and 100 tall, so it clears the 32-unit threshold
	# and is too tall for hull 3. Deliberately not a round 32 — a mover whose box
	# matches its hull exactly has a zero offset, so it cannot tell the sized-hull path
	# apart from hull 0 by symmetry alone.
	var pbox := BoxShape3D.new()
	pbox.size = Vector3(33 * S, 100 * S, 33 * S)
	_player = BspScene.add_mover(_map, pbox)
	_player.global_position = BspScene.gs(0, -4096, -4096)


func teardown() -> void:
	if is_instance_valid(_map):
		_map.free()
	_map = null
	_player = null
	_bodies.clear()


## Fly `body` at the slab from GoldSrc x = `from_x` and report where it stopped, in
## GoldSrc units. Always flown toward x = 0, so a gap on either face is measured the
## same way.
func _fly(body: CharacterBody3D, from_x: float) -> float:
	var r := BspScene.probe(body, Vector3(from_x, 0, 100),
		Vector3(-signf(from_x) * 100.0, 0, 0))
	return from_x + (r["travel_gs"] as Vector3).x


## The gaps a sphere of RADII[i] leaves on the slab's two faces.
func _gaps(i: int) -> Vector2:
	return Vector2(_fly(_bodies[i], 60.0) - FACE, -FACE - _fly(_bodies[i], -60.0))


func tests() -> Array:
	return [
		"stops_the_same_distance_from_either_face",
		"stops_a_radius_short_not_a_hull_short",
		"player_sized_movers_still_get_a_sized_hull",
	]


func stops_the_same_distance_from_either_face(t) -> void:
	# The invariant, and the one that actually failed. A slab is symmetric; a mover
	# approaching it has no way to know which side it came from, so the two gaps must
	# agree. Swept across the whole small-mover range, because the old rule was correct
	# at exactly one size and wrong either side of it.
	for i in RADII.size():
		var g := _gaps(i)
		t.near(g.x, g.y, 0.02,
			"radius %s: a slab must stop a mover alike from either side" % RADII[i])


func stops_a_radius_short_not_a_hull_short(t) -> void:
	# Symmetry alone is satisfied by a mover that stops a metre out on both sides, so
	# pin the distance too: the gap is the mover's own radius, plus the trace's standoff.
	for i in RADII.size():
		t.near(_gaps(i).x, RADII[i] / S, STANDOFF + 0.02,
			"radius %s: a projectile stops its own radius short, not a hull's" % RADII[i])


func player_sized_movers_still_get_a_sized_hull(t) -> void:
	# The upper side of the threshold. Hull 0 is the only hull without CLIP brushes in
	# it, so which side of 32 units a mover falls on decides whether clip geometry
	# blocks it — worth pinning in both directions. (The lower side is already covered
	# across the whole small range by stops_a_radius_short_not_a_hull_short.)
	#
	# Nothing here can read the hull index back, so it is inferred from the stopping
	# distance: a 33-unit mover held off by hull 1 clears the slab by the HULL's
	# half-width, 16, where hull 0 would give its own 16.5.
	#
	# Measured as the MEAN of the two faces, because low-corner alignment puts one face
	# at 16.5 and the other at 15.5 — the mean is the hull's half-width whatever the
	# offset does. Asserting the asymmetry itself would pin an artifact of that
	# alignment, and would have to fail if it were ever generalised away, even though
	# behaviour would have improved.
	var near := _fly(_player, 60.0) - FACE
	var far := -FACE - _fly(_player, -60.0)
	t.near((near + far) * 0.5, 16.0, 0.25,
		"a 33-unit mover is held off by hull 1's half-width, not by its own")
