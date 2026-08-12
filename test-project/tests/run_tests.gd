extends SceneTree
## Headless runner for the GDScript-level tests — the ones that need a live
## PhysicsServer3D and so cannot be reached from the C++ suite in ../../tests.
##
##   godot --headless --path test-project -s res://tests/run_tests.gd
##
## Exits non-zero on any failure, so it drops straight into CI next to ctest.

const SUITES := [
	preload("res://tests/test_bsp_body_test_motion.gd"),
	preload("res://tests/test_bsp_projectile.gd"),
]

var _pass := 0
var _fail := 0
var _suite := ""
var _test := ""


func lt(a: float, b: float, ctx := "") -> void:
	_check(a < b, "%s < %s" % [a, b], ctx)


func gt(a: float, b: float, ctx := "") -> void:
	_check(a > b, "%s > %s" % [a, b], ctx)


func near(a: float, b: float, tol: float, ctx := "") -> void:
	_check(absf(a - b) <= tol, "%s ≈ %s (±%s)" % [a, b, tol], ctx)


func _check(ok: bool, what: String, ctx: String) -> void:
	if ok:
		_pass += 1
		return
	_fail += 1
	printerr("  FAIL %s.%s: expected %s%s" % [
		_suite, _test, what, "" if ctx.is_empty() else " — " + ctx])


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	print("\n====== hop-godot GDScript suite ======\n")
	for script in SUITES:
		var inst = script.new(self)
		_suite = script.resource_path.get_file().get_basename()
		for name in inst.tests():
			_test = name
			var before := _fail
			inst.setup()
			# One physics frame so freshly added collision nodes reach the server.
			await physics_frame
			inst.call(name, self)
			inst.teardown()
			await physics_frame
			print("  %s %s" % ["✓" if _fail == before else "✗", name])

	var total := _pass + _fail
	print("\n====== %d/%d assertions passed ======" % [_pass, total])
	if _fail > 0:
		printerr("====== %d FAILED ======" % _fail)
	quit(1 if _fail > 0 else 0)
