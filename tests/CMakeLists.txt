# Standalone tests for hop-godot glue code that depends only on hop core
# (no godot-cpp). Build & run:
#
#   cmake -S extern/hop-godot/tests -B extern/hop-godot/tests/build
#   cmake --build extern/hop-godot/tests/build
#   ctest --test-dir extern/hop-godot/tests/build --output-on-failure
#
cmake_minimum_required(VERSION 3.22)
project(hop-godot-tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# Tests rely on assert() — keep it live regardless of build type.
add_compile_options(-UNDEBUG)

# hop core headers + the hop-godot source dir (for the traceable headers).
function(add_hop_godot_test name)
    add_executable(${name} ${name}.cpp)
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../extern/hop/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_hop_godot_test(test_trimesh_traceable)
add_hop_godot_test(test_bsp_traceable)
add_hop_godot_test(test_broadphase_capacity)

# Benchmarks are built but not registered with ctest — they report timings, they
# do not pass or fail. Build explicitly: cmake --build <dir> --target bench_bsp_traceable
function(add_hop_godot_bench name)
    add_executable(${name} EXCLUDE_FROM_ALL ${name}.cpp)
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../extern/hop/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../extern/hop
        ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_compile_options(${name} PRIVATE -O2)
endfunction()

add_hop_godot_bench(bench_bsp_traceable)
