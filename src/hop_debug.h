#pragma once

// Shared debug counters written by HopTrimeshTraceable::trace_solid.
// Use inline variables (C++17) so this header can be included from multiple TUs.
struct TrimeshTraceDebug {
	int bvh_hits = 0;
	int denom_pass = 0;
	int t_pass = 0;
	int bary_pass = 0;
};
inline TrimeshTraceDebug g_last_trimesh_trace_debug;
