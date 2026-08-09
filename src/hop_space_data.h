#pragma once

#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

#include <hop/hop.h>
#include <hop/bvh_manager.h>
#include <memory>
#include <vector>
#include "hop_conversions.h"

using namespace godot;

class HopDirectSpaceState;

struct HopSpaceData {
	RID self_rid;
	bool active = false;

	std::unique_ptr<hop::simulator<hop_scalar>> simulator;
	hop::bvh_manager<hop_scalar> bvh_manager;

	// Dedicated broadphase for area sensor solids — a pure spatial index, NOT
	// driven by the simulator (areas never step or block).  Accelerates area
	// overlap queries and area-area monitoring from O(areas) to O(log areas).
	// Holds only areas, so broadphase results never need body/area disambiguation.
	hop::bvh_manager<hop_scalar> area_bvh;

	// Space params (Godot defaults)
	float contact_recycle_radius = 0.01f;
	float contact_max_separation = 0.05f;
	float contact_max_allowed_penetration = 0.01f;
	float contact_default_bias = 0.8f;
	float body_linear_velocity_sleep_threshold = 0.01f;
	float body_angular_velocity_sleep_threshold = 0.05f;
	float body_time_to_sleep = 0.5f;
	float solver_iterations = 8.0f;

	// Debug contacts
	int max_debug_contacts = 0;
	PackedVector3Array debug_contacts;

	// Cached direct state (created lazily, owned by this space)
	HopDirectSpaceState *direct_state = nullptr;


	// The space's DEFAULT area, which Godot addresses by the space RID rather than an
	// area RID — that is how physics/3d/default_gravity reaches a PhysicsServer3D.
	// Magnitude and direction arrive as separate calls, so both are kept and the
	// simulator is re-pushed whenever either changes.
	float default_gravity = 9.81f;
	Vector3 default_gravity_direction = Vector3(0.0f, -1.0f, 0.0f);

	void apply_default_gravity() {
		Vector3 g = default_gravity_direction.normalized() * default_gravity;
		simulator->set_gravity(hop::vec3<hop_scalar>(
			to_hop_scalar(g.x), to_hop_scalar(g.y), to_hop_scalar(g.z)));
	}


	// --- Broad-phase result buffers ---
	//
	// find_solids_in_aa_box fills at most max_solids and clamps its return to it, so a
	// fixed-size array truncates *silently*. That is not a capacity nuisance: the static
	// bucket is scanned before the dynamic one (bvh_manager::find_solids_in_aa_box), so
	// the hits a short buffer drops are exactly the moving bodies callers care about — an
	// area overlapping enough static geometry would stop detecting players entirely.
	//
	// Sizing each buffer to the live solid/area count removes the failure mode by
	// construction, which is what hop's own simulator does internally (spacial_collection_
	// is resized to solids_.size()). The vectors keep their capacity between queries, so
	// steady state allocates nothing.
	//
	// One buffer per call site, not one shared: an area monitor callback runs GDScript,
	// which can re-enter the server and issue its own query on this same space. Separate
	// buffers mean such a re-entrant query can never overwrite results a caller is still
	// walking.
	std::vector<hop::solid<hop_scalar> *> monitor_body_buffer; // _flush_queries: area→body
	std::vector<hop::solid<hop_scalar> *> monitor_area_buffer; // _flush_queries: area→area
	std::vector<hop::solid<hop_scalar> *> query_body_buffer;   // space-state body queries
	std::vector<hop::solid<hop_scalar> *> query_area_buffer;   // space-state area queries

	// Resize `buf` to hold every solid the simulator could return. get_solids() covers the
	// manager-accelerated path and the linear-scan fallback alike, since both draw from it.
	std::vector<hop::solid<hop_scalar> *> &size_for_bodies(std::vector<hop::solid<hop_scalar> *> &buf) {
		buf.resize(simulator->get_solids().size());
		return buf;
	}

	// Same for the area index. Areas are registered static (add_area_to_space), but count
	// both buckets so this stays correct if that ever changes.
	std::vector<hop::solid<hop_scalar> *> &size_for_areas(std::vector<hop::solid<hop_scalar> *> &buf) {
		buf.resize(area_bvh.get_static_count() + area_bvh.get_dynamic_count());
		return buf;
	}

	HopSpaceData() {
		simulator = std::make_unique<hop::simulator<hop_scalar>>();
		simulator->set_manager(&bvh_manager);
		apply_default_gravity();
		simulator->set_integrator(hop::integrator_type::improved);
		// Default every body to the speculative solve (matches the old global
		// set_speculative_contacts(true); per-body overridable via set_contact_mode).
		simulator->set_default_contact_mode(hop::contact_mode::speculative);
		// Narrowphase accuracy for rounded-vs-polytope pairs: accurate GJK
		// (default) vs the cheap inflate/AABB path. Pick via the project setting
		// "physics/hop/accurate_narrowphase" (bool); absent => true.
		bool accurate = true;
		if (ProjectSettings *ps = ProjectSettings::get_singleton())
			accurate = ps->get_setting("physics/hop/accurate_narrowphase", true);
		simulator->set_accurate_narrowphase(accurate);
		// Angular substepping (opt-in continuous collision for fast spinners): cap the
		// number of fixed-orientation sub-traces per frame. "physics/hop/angular_substeps_max"
		// (int); absent / 1 => off (single snapshot, bit-identical). Raise it for a scene
		// with fast, thin spinners (a blade trap) that must not be stepped over angularly.
		int substeps = 1;
		if (ProjectSettings *ps = ProjectSettings::get_singleton())
			substeps = (int)ps->get_setting("physics/hop/angular_substeps_max", 1);
		simulator->set_angular_substeps_max(substeps);
	}
};
