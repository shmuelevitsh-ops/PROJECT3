cmake_minimum_required(VERSION 3.15)

# Standalone configuration: only kicks in when this file is configured directly
# (Algorithm/ built on its own), not when pulled in via add_subdirectory() from
# the root CMakeLists.txt, which already provides all of this.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # The vcpkg manifest (vcpkg.json) lives at the repo root, shared by every
    # subproject; point vcpkg at it so a standalone configure can still resolve
    # dependencies via CMAKE_TOOLCHAIN_FILE alone.
    if(NOT DEFINED VCPKG_MANIFEST_DIR)
        set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE STRING "")
    endif()

    project(Algorithm LANGUAGES CXX)

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

    find_package(mp-units CONFIG REQUIRED)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common)
endif()

if(NOT COMMAND drone_warnings)
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

add_library(Algorithm_322889890_315113738 SHARED
    src/MappingAlgorithmImpl.cpp
)

set_target_properties(Algorithm_322889890_315113738 PROPERTIES
    PREFIX ""
)

target_include_directories(Algorithm_322889890_315113738 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(Algorithm_322889890_315113738 PRIVATE
    common::common
)

drone_warnings(Algorithm_322889890_315113738)


# for tests
find_package(GTest CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)

add_executable(algorithm_component_test
    tests/component/mapping_algorithm_test.cpp
    tests/component/support/RegistrationStub.cpp
    src/MappingAlgorithmImpl.cpp
    # Test-only: mapping_algorithm_test.cpp relies extensively on real map geometry, so it
    # links the Simulator-owned Map3DImpl directly (decided in Phase 3, see
    # PROJ2_TESTS_PLAN.md §6 / PROJ2_TESTS_EXECUTION_PLAN.md Phase 3 task 1). Introduces no
    # production dependency from Algorithm on Simulator; Algorithm_322889890_315113738
    # above is unaffected and stays independently built.
    ${CMAKE_CURRENT_SOURCE_DIR}/../Simulator/src/Map3DImpl.cpp
)

target_include_directories(algorithm_component_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/component
    ${CMAKE_CURRENT_SOURCE_DIR}/../Simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(algorithm_component_test PRIVATE
    common::common
    TinyNPY::TinyNPY
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
)

drone_warnings(algorithm_component_test)