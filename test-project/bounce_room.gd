extends Node3D

const ROPE_ANCHOR := Vector3(0, 6, 0)

var rope_mesh: ImmediateMesh
var rope_instance: MeshInstance3D
var rope_material: StandardMaterial3D

func _ready():
	# Connect collision signals on all dynamic bodies
	for body in [%Box, %Sphere, %Capsule1, %Capsule2]:
		body.body_entered.connect(_on_body_entered.bind(body))

	# Rope line visual
	rope_mesh = ImmediateMesh.new()
	rope_material = StandardMaterial3D.new()
	rope_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	rope_material.albedo_color = Color(0.75, 0.75, 0.75)
	rope_instance = MeshInstance3D.new()
	rope_instance.mesh = rope_mesh
	add_child(rope_instance)

func _process(_delta):
	rope_mesh.clear_surfaces()
	rope_mesh.surface_begin(Mesh.PRIMITIVE_LINES, rope_material)
	rope_mesh.surface_add_vertex(ROPE_ANCHOR)
	rope_mesh.surface_add_vertex(%Capsule1.global_position)
	rope_mesh.surface_end()

func _on_body_entered(other: Node, body: RigidBody3D):
	var contact_pos := _estimate_contact_pos(body, other)
	var speed := body.linear_velocity.length()
	var count := clampi(4 + int(speed * 1.5), 4, 16)
	_spawn_sparks(contact_pos, count)

func _estimate_contact_pos(body: RigidBody3D, other: Node) -> Vector3:
	var pos := body.global_position
	if other is StaticBody3D:
		match other.name:
			"Floor": return Vector3(pos.x, 0, pos.z)
			"Ceiling": return Vector3(pos.x, 6, pos.z)
			"WallNegX": return Vector3(-3, pos.y, pos.z)
			"WallPosX": return Vector3(3, pos.y, pos.z)
			"WallNegZ": return Vector3(pos.x, pos.y, -3)
			"WallPosZ": return Vector3(pos.x, pos.y, 3)
	if other is RigidBody3D:
		return (pos + other.global_position) * 0.5
	return pos

func _spawn_sparks(pos: Vector3, count: int = 8):
	var sparks := CPUParticles3D.new()
	sparks.position = pos
	sparks.emitting = true
	sparks.one_shot = true
	sparks.explosiveness = 1.0
	sparks.amount = count * 2
	sparks.lifetime = 0.3
	sparks.direction = Vector3.ZERO
	sparks.spread = 180.0
	sparks.initial_velocity_min = 3.0
	sparks.initial_velocity_max = 6.0
	sparks.gravity = Vector3(0, -9.8, 0)
	sparks.scale_amount_min = 0.15
	sparks.scale_amount_max = 0.3
	sparks.color = Color(1.0, 0.85, 0.2)
	var gradient := Gradient.new()
	gradient.set_color(0, Color(1.0, 0.95, 0.4, 1.0))
	gradient.set_color(1, Color(1.0, 0.4, 0.0, 0.0))
	sparks.color_ramp = gradient
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.albedo_color = Color(1.0, 0.85, 0.2)
	var mesh := SphereMesh.new()
	mesh.radius = 0.06
	mesh.height = 0.12
	mesh.material = mat
	sparks.mesh = mesh
	add_child(sparks)
	get_tree().create_timer(1.0).timeout.connect(sparks.queue_free)
