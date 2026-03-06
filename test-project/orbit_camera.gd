extends Camera3D

var angle := 0.0

func _process(delta: float) -> void:
	angle += 0.3 * delta
	var dist := 18.0
	var height := 8.0
	position = Vector3(dist * cos(angle), height, dist * sin(angle))
	look_at(Vector3(0, 3, 0))
