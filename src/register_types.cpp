#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/physics_server3d_manager.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

#include "hop_physics_server.h"
#include "hop_body_state.h"
#include "hop_space_state.h"

using namespace godot;

static HopPhysicsServer *hop_server_instance = nullptr;

static PhysicsServer3D *create_hop_physics_server() {
	hop_server_instance = memnew(HopPhysicsServer);
	return hop_server_instance;
}

void initialize_hop_physics_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}

	GDREGISTER_CLASS(HopPhysicsServer);
	GDREGISTER_CLASS(HopDirectBodyState);
	GDREGISTER_CLASS(HopDirectSpaceState);

	PhysicsServer3DManager::get_singleton()->register_server("Hop", callable_mp_static(create_hop_physics_server));
}

void uninitialize_hop_physics_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	hop_server_instance = nullptr;
}

extern "C" {
GDExtensionBool GDE_EXPORT hop_physics_library_init(
		GDExtensionInterfaceGetProcAddress p_get_proc_address,
		GDExtensionClassLibraryPtr p_library,
		GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_hop_physics_module);
	init_obj.register_terminator(uninitialize_hop_physics_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
