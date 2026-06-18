#!/usr/bin/env bash
# Build the hop-physics GDExtension for all target platforms.
# Requires: cmake, ninja, zig (https://ziglang.org)
# On macOS: also uses lipo (bundled with Xcode Command Line Tools) to produce universal binaries.
#
# Usage:
#   ./build.sh              # build everything
#   ./build.sh macos        # macOS arm64 + x86_64 + universal only
#   ./build.sh linux        # Linux x86_64 + arm64 only
#   ./build.sh windows      # Windows x86_64 only
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$SCRIPT_DIR/addons/hop-physics/bin"
CMAKE_DIR="$SCRIPT_DIR/cmake"

filter="${1:-all}"

die() { echo "ERROR: $*" >&2; exit 1; }

require_cmd() {
    command -v "$1" &>/dev/null || die "'$1' not found — install it first"
}

require_cmd cmake
require_cmd zig

if command -v ninja &>/dev/null; then
    GENERATOR="-G Ninja"
else
    GENERATOR=""
fi

cmake_build() {
    local build_dir="$SCRIPT_DIR/$1"
    local toolchain="$CMAKE_DIR/$2"
    local build_type="$3"

    echo ""
    echo "==> Building: $1 ($build_type)"
    cmake -B "$build_dir" -S "$SCRIPT_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
        -DCMAKE_BUILD_TYPE="$build_type" \
        $GENERATOR \
        --log-level=WARNING
    cmake --build "$build_dir" --config "$build_type"
}

build_macos() {
    cmake_build build-macos-arm64-debug    toolchain-macos-arm64.cmake   Debug
    cmake_build build-macos-arm64-release  toolchain-macos-arm64.cmake   Release
    cmake_build build-macos-x86_64-debug   toolchain-macos-x86_64.cmake  Debug
    cmake_build build-macos-x86_64-release toolchain-macos-x86_64.cmake  Release

    if command -v lipo &>/dev/null; then
        echo ""
        echo "==> lipo: creating universal binaries"
        lipo -create \
            "$BIN_DIR/libhop-physics.macos.template_debug.arm64.dylib" \
            "$BIN_DIR/libhop-physics.macos.template_debug.x86_64.dylib" \
            -output "$BIN_DIR/libhop-physics.macos.template_debug.universal.dylib"
        lipo -create \
            "$BIN_DIR/libhop-physics.macos.template_release.arm64.dylib" \
            "$BIN_DIR/libhop-physics.macos.template_release.x86_64.dylib" \
            -output "$BIN_DIR/libhop-physics.macos.template_release.universal.dylib"
        echo "    wrote libhop-physics.macos.template_{debug,release}.universal.dylib"
    else
        echo "    (lipo not available — skipping universal binaries)"
    fi
}

build_linux() {
    cmake_build build-linux-x86_64-debug   toolchain-linux-x86_64.cmake  Debug
    cmake_build build-linux-x86_64-release toolchain-linux-x86_64.cmake  Release
    cmake_build build-linux-arm64-debug    toolchain-linux-arm64.cmake    Debug
    cmake_build build-linux-arm64-release  toolchain-linux-arm64.cmake    Release
}

build_windows() {
    cmake_build build-windows-x86_64-debug   toolchain-windows-x86_64.cmake  Debug
    cmake_build build-windows-x86_64-release toolchain-windows-x86_64.cmake  Release
}

case "$filter" in
    macos)   build_macos ;;
    linux)   build_linux ;;
    windows) build_windows ;;
    all)
        build_macos
        build_linux
        build_windows
        ;;
    *)
        echo "Unknown target: $filter"
        echo "Usage: $0 [macos|linux|windows|all]"
        exit 1
        ;;
esac

echo ""
echo "==> Done. Binaries in: $BIN_DIR"
ls -lh "$BIN_DIR"
