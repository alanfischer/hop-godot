# hop-godot

A Godot 4 GDExtension that integrates the [hop](extern/hop/) physics engine as a `PhysicsServer3D` backend. Once enabled, Godot's built-in physics nodes (`RigidBody3D`, `StaticBody3D`, `CharacterBody3D`, etc.) run on hop instead of GodotPhysics.

## Prerequisites

- Godot 4.1+
- CMake 3.22+
- C++17 compiler
- [Zig](https://ziglang.org/) — only for `build.sh`, which uses it as the cross-compiler

## Building

```bash
git submodule update --init --recursive
cmake -B build
cmake --build build
```

That builds for the host platform only. To produce the shipping set of binaries
(macOS universal, Linux x86_64/arm64, Windows, Android — cross-compiled through Zig):

```bash
./build.sh                # default platforms: macos linux windows android
./build.sh macos linux    # or just the ones you want
```

Either way the built libraries land in `addons/hop-physics/bin/`.

## Usage

1. Copy the `addons/hop-physics/` directory into your Godot project's `addons/` folder.
2. In **Project > Project Settings > General > Physics > 3D > Physics Engine**, select **Hop Physics**.
3. Restart the editor. All 3D physics will now use hop.

The extension is deliberately **not** hot-reloadable (`reloadable = false`): recreating the
live server nulls Godot's `PhysicsServer3D` singleton and crashes on the next physics
frame. Rebuilding means restarting the editor.

## GoldSrc BSP hull collision

Maps imported by [goldsrc-godot](https://github.com/alanfischer/goldsrc-godot) ship the
map compiler's own clipnode trees alongside the usual trimesh/convex collision shapes.
When hop sees them it traces the real GoldSrc hulls instead of the shapes, which is what
gives exact plane normals, true 18-unit step-ups, and the same "solid" answers the
original engine gave.

### How a body opts in

goldsrc-godot's BSP importer stamps the metadata; hop reads it. Nothing else is needed —
no per-body setup, no scene changes.

| Where | Metadata | Meaning |
|---|---|---|
| Map scene root | `bsp_data` (`PackedByteArray`) | Stripped BSP30 blob — PLANES, NODES, CLIPNODES, LEAFS, MODELS, VISIBILITY. Rides once per map (it runs to megabytes); a body walks up its parents to find it. |
| Each collision body | `bsp_model` (int) | Which BSP model's tree this body stands for (0 = worldspawn, 1+ = brush entities). |
| Each collision body | `bsp_scale` (float) | Godot units per GoldSrc unit. **Required** — there is no default, because guessing it would silently trace the map at the wrong size. |
| Each collision body | `bsp_blocking` (int, optional) | Bitmask over `-contents` of what stops a trace (`SOLID = -2` → bit 2). Defaults to `BLOCK_SOLID`. The sky body sets `CONTENTS_SKY` so sky blocks players on hulls 1–3 while staying passable on hull 0. |

A body missing any of this falls back to its normal trimesh/convex shapes, which is also
exactly what default Godot physics does with the same scene. The metadata is purely additive.

The BSP tree itself is loaded once per map and shared (weakly, so unloading the map frees
it) across every body on it; a carrier body's own shape geometry is released once the hull
takes over.

### Hull selection

GoldSrc bakes four hull sizes into the engine, and the compiler expanded the clipnode
trees for exactly those boxes, so a mover is matched to one by its GoldSrc-space size:

| Hull | Box | Used for |
|---|---|---|
| 0 | point | Anything under 32 units wide — traced as a point hull inflated by the mover's own radius (exact, and what GoldSrc does with projectiles). Note hull 0 has no CLIP brushes in it. |
| 1 | 32×32×72 | Standing player |
| 2 | 64×64×64 | Large monster (over 36 units wide) |
| 3 | 32×32×36 | Crouched player |

### Turning it on and off

It is **on by default** whenever the metadata is present. Two switches:

```bash
HOP_NO_BSP_HULLS=1 ./your-game      # boot default only (CI, dedicated servers with no console)
```

```gdscript
# Runtime toggle — takes effect immediately, rebuilding every carrier body already
# in the world (so it is safe to flip mid-map, in either direction).
var server := PhysicsServer3D.get_singleton()
if server.has_method("set_bsp_hulls_enabled"):
    server.call("set_bsp_hulls_enabled", false)
    print(server.call("get_bsp_hulls_enabled"))
```

`has_method` guards are worth keeping: they make the same script work under GodotPhysics,
where the methods do not exist.

In a networked game this flag has to agree on both ends — a client colliding against
different geometry than the server mispredicts every step. WizardWars replicates it as
the `bsp_hulls` server var (`/bsp_hulls on|off`, or `--bsp-hulls on|off` at launch).

A body turned intangible the usual Godot way (disabling its `CollisionShape3D`s) is
intangible under hulls too — the hull answers to the same switch, so a door that opens by
going non-solid still opens.

## Project Structure

```
src/                  GDExtension source (PhysicsServer3D implementation)
extern/godot-cpp/     godot-cpp bindings (git submodule)
extern/hop/           hop physics library (git submodule)
addons/hop-physics/   GDExtension plugin (built binaries + .gdextension)
test-project/         Godot project for testing (+ GDScript test suites)
tests/                Standalone C++ tests for the glue code
```

## Tests

C++ tests cover the traceables and broadphase with no Godot dependency:

```bash
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

Tests that need a live `PhysicsServer3D` (BSP hull sweeps, `body_test_motion` against a
real hull, projectile behaviour) run headless in the test project:

```bash
godot --headless --path test-project -s res://tests/run_tests.gd
```

Both exit non-zero on failure.

## Status

Rigid body simulation, character movement, shape/ray queries, `body_test_motion`, and
GoldSrc BSP hull collision work. Soft bodies and some joint types are stubbed out.
