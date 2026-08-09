# hop-godot

A Godot 4 GDExtension that integrates the [hop](extern/hop/) physics engine as a `PhysicsServer3D` backend. Once enabled, Godot's built-in physics nodes (`RigidBody3D`, `StaticBody3D`, `CharacterBody3D`, etc.) run on hop instead of GodotPhysics.

## Prerequisites

- Godot 4.1+
- CMake 3.22+
- C++17 compiler

## Building

```bash
git submodule update --init --recursive
cmake -B build
cmake --build build
```

The built library is placed in `addons/hop-physics/bin/`.

## Usage

1. Copy the `addons/hop-physics/` directory into your Godot project's `addons/` folder.
2. In **Project > Project Settings > General > Physics > 3D > Physics Engine**, select **Hop**.
3. Restart the editor. All 3D physics will now use hop.

## Project Structure

```
src/                  GDExtension source (PhysicsServer3D implementation)
extern/godot-cpp/     godot-cpp bindings (git submodule)
extern/hop/           hop physics library (git submodule)
addons/hop-physics/   GDExtension plugin (built binaries + .gdextension)
test-project/         Godot project for testing
```

## Status

Early development. Rigid body simulation works; soft bodies and some joint types are stubbed out.
