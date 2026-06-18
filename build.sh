#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Initialize submodules if needed
if [ ! -f extern/godot-cpp/CMakeLists.txt ] || [ ! -f extern/hop/CMakeLists.txt ]; then
    git submodule update --init --recursive
fi

# Build debug (template_debug — needed by Godot editor)
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug

# Build release (template_release — needed for export)
cmake -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release

echo ""
echo "Built:"
ls addons/hop-physics/bin/
