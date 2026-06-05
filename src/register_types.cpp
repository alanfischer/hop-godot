#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/physics_server3d_manager.hpp>
#include <godot_cpp/variant/callable.hpp>

#include "hop_physics_server.h"
#include "hop_body_state.h"
#include "hop_space_state.h"

using namespace godot;

// Factory class — Godot needs a bound method on an Object to create the server.
class HopPhysicsServerFactory : public Object {
	GDCLASS(HopPhysicsServerFactory, Object);

protected:
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("create_hop_callback"), &HopPhysicsServerFactory::_create_hop_callback);
	}

public:
	PhysicsServer3D *_create_hop_callback() {
		return memnew(HopPhysicsServer);
	}
};

static HopPhysicsServerFactory *hop_factory = nullptr;

void initialize_hop_physics_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}

	ClassDB::register_class<HopDirectBodyState>(true);
	ClassDB::register_class<HopDirectSpaceState>(true);
	ClassDB::register_class<HopPhysicsServer>();
	ClassDB::register_class<HopPhysicsServerFactory>();

	hop_factory = memnew(HopPhysicsServerFactory);
	PhysicsServer3DManager::get_singleton()->register_server("Hop Physics", Callable(hop_factory, "create_hop_callback"));
}

void uninitialize_hop_physics_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	if (hop_factory) {
		memdelete(hop_factory);
		hop_factory = nullptr;
	}
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
