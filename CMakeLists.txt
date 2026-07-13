cmake_minimum_required(VERSION 3.16)

project(f4flight
    VERSION 1.0.0
    DESCRIPTION "Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Library options
# ---------------------------------------------------------------------------
option(F4FLIGHT_BUILD_TESTS    "Build the f4flight unit tests"             ON)
option(F4FLIGHT_BUILD_TOOLS    "Build the dat2json/dat_validate/json_diff tools" ON)
option(F4FLIGHT_BUILD_SHARED   "Build f4flight as a shared library (DLL)"  OFF)
option(F4FLIGHT_ENABLE_WERROR  "Treat warnings as errors"                  OFF)
option(F4FLIGHT_USE_MSVC_UTF8  "Force /utf-8 on MSVC"                       ON)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# Default to Release if unspecified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
set(F4FLIGHT_SOURCES
    src/aircraft_config.cpp
    src/aerodynamics.cpp
    src/engine.cpp
    src/fcs.cpp
    src/gear.cpp
    src/eom.cpp
    src/flight_model.cpp
    src/steering.cpp
    src/dat_loader.cpp
    src/json_io.cpp
    src/config/f16c_config.cpp
    # digi/ subsystem (AI brain)
    src/digi/digi_brain.cpp
    src/digi/maneuvers/maneuver_primitives.cpp
    src/digi/ground/ground_avoid.cpp
    src/digi/defensive/missile_defeat.cpp
    src/digi/defensive/guns_jink.cpp
)

set(F4FLIGHT_HEADERS
    include/f4flight/f4flight.h
    include/f4flight/core/constants.h
    include/f4flight/core/types.h
    include/f4flight/core/math.h
    include/f4flight/core/lookup.h
    include/f4flight/core/units.h
    include/f4flight/core/trig.h
    include/f4flight/atmosphere.h
    include/f4flight/aircraft_config.h
    include/f4flight/aircraft_state.h
    include/f4flight/aerodynamics.h
    include/f4flight/engine.h
    include/f4flight/gear.h
    include/f4flight/fcs.h
    include/f4flight/eom.h
    include/f4flight/flight_model.h
    include/f4flight/steering.h
    include/f4flight/dat_loader.h
    include/f4flight/json_io.h
    include/f4flight/config/f16c_config.h
    # digi/ subsystem (AI brain)
    include/f4flight/digi/digi_brain.h
    include/f4flight/digi/digi_state.h
    include/f4flight/digi/digi_mode.h
    include/f4flight/digi/digi_skill.h
    include/f4flight/digi/digi_entity.h
    include/f4flight/digi/maneuvers/maneuver_primitives.h
    include/f4flight/digi/ground/ground_avoid.h
    include/f4flight/digi/defensive/missile_defeat.h
    include/f4flight/digi/defensive/guns_jink.h
)

add_library(f4flight
    ${F4FLIGHT_SOURCES}
    ${F4FLIGHT_HEADERS}
)

if(F4FLIGHT_BUILD_SHARED)
    set_target_properties(f4flight PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
        CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
    set_target_properties(f4flight PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
else()
    set_target_properties(f4flight PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
endif()

target_include_directories(f4flight
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(f4flight PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Compiler warnings
# ---------------------------------------------------------------------------
if(MSVC)
    set(F4FLIGHT_MSVC_OPTIONS /W4 /permissive- /Zc:__cplusplus /Zc:preprocessor)
    if(F4FLIGHT_ENABLE_WERROR)
        list(APPEND F4FLIGHT_MSVC_OPTIONS /WX)
    endif()
    if(F4FLIGHT_USE_MSVC_UTF8)
        list(APPEND F4FLIGHT_MSVC_OPTIONS /utf-8)
    endif()
    target_compile_options(f4flight PRIVATE ${F4FLIGHT_MSVC_OPTIONS})
else()
    set(F4FLIGHT_GCC_OPTIONS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(F4FLIGHT_ENABLE_WERROR)
        list(APPEND F4FLIGHT_GCC_OPTIONS -Werror)
    endif()
    target_compile_options(f4flight PRIVATE ${F4FLIGHT_GCC_OPTIONS})
endif()

# ---------------------------------------------------------------------------
# Alias target so consumers can use f4flight::f4flight
# ---------------------------------------------------------------------------
add_library(f4flight::f4flight ALIAS f4flight)

# ===========================================================================
# Tests
# ===========================================================================
# The test tree is split into:
#
#   tests/unit/      — gtest unit tests for each library module (math, aero,
#                      engine, fcs, eom, json_io, dat_loader, etc.). Quick
#                      correctness checks, no simulation.
#
#   tests/maneuver/  — scenario-based end-to-end integration tests that drive
#                      the full FlightModel + SteeringController through
#                      multi-phase maneuvers (climb, turn, orbit, waypoint
#                      circuit, ...). Each scenario self-registers; the
#                      runner picks up scenarios by name.
#
#   tests/fixtures/  — curated JSON aircraft files used by the maneuver
#                      tests. The set is intentionally small (10 aircraft)
#                      but representative: fighters (F-16, F-15, F-14,
#                      MiG-29, Su-27, EF2000, Rafale), attack (A-10),
#                      bomber (B-52), and transport (C-130). Adding a JSON
#                      to this directory automatically registers it for
#                      every maneuver scenario (see file(GLOB) below).
# ===========================================================================
if(F4FLIGHT_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    # On Windows / MSVC, prevent overriding the parent project's compiler/linker options
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # ----- unit tests -----
    file(GLOB F4FLIGHT_UNIT_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/unit/*.cpp"
    )
    add_executable(f4flight_tests ${F4FLIGHT_UNIT_TEST_SOURCES})
    target_link_libraries(f4flight_tests PRIVATE f4flight GTest::gtest)

    if(MSVC)
        target_compile_options(f4flight_tests PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
    else()
        target_compile_options(f4flight_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(f4flight_tests)

    # ----- maneuver tests -----
    set(F4FLIGHT_MANEUVER_TEST_SOURCES
        tests/maneuver/maneuver_test.cpp
        tests/maneuver/scenarios/scenario_ai_basic.cpp
        tests/maneuver/scenarios/scenario_ai_flightplan.cpp
        tests/maneuver/scenarios/scenario_ai_cruise.cpp
        tests/maneuver/scenarios/scenario_fcs_glimits.cpp
        tests/maneuver/scenarios/scenario_fcs_engine.cpp
        tests/maneuver/scenarios/scenario_fcs_roll.cpp
    )

    add_executable(maneuver_test ${F4FLIGHT_MANEUVER_TEST_SOURCES})
    target_include_directories(maneuver_test PRIVATE tests/maneuver/)
    target_link_libraries(maneuver_test PRIVATE f4flight)
    if(MSVC)
        target_compile_options(maneuver_test PRIVATE /W4 /utf-8)
    else()
        target_compile_options(maneuver_test PRIVATE -Wall -Wextra)
    endif()

    # Register every fixture x scenario combination as a ctest test.
    # Adding a JSON to tests/fixtures/ automatically registers it for every
    # listed scenario -- no need to edit this file.
    file(GLOB F4FLIGHT_FIXTURES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/*.json"
    )
    # All 6 registered scenarios are exercised via ctest. Previously only
    # `basic` and `flightplan` were listed, so combat/glimits/engine/roll
    # regression would not break CI even if they went from passing to
    # failing. The maneuver_test runner returns non-zero if any phase fails
    # (see maneuver_test.cpp), so ctest now sees real phase-level results.
    set(F4FLIGHT_MANEUVER_SCENARIOS
        ai_basic
        ai_flightplan
        ai_cruise
        fcs_glimits
        fcs_engine
        fcs_roll
    )
    foreach(fixture ${F4FLIGHT_FIXTURES})
        get_filename_component(ac_name "${fixture}" NAME_WE)
        foreach(sc ${F4FLIGHT_MANEUVER_SCENARIOS})
            add_test(
                NAME maneuver_${ac_name}_${sc}
                COMMAND maneuver_test "${fixture}" --scenario ${sc}
            )
        endforeach()
    endforeach()
endif()

# ===========================================================================
# Tools
# ===========================================================================
# Standalone command-line utilities that ship with the library. None of them
# are linked into f4flight itself; they're just thin mains over the public
# API.
#
#   dat2json        Convert a Falcon 4 .dat file to f4flight JSON.
#   dat_validate    Load a JSON and run AircraftConfig::validate().
#   json_diff      Field-by-field diff of two JSON aircraft files.
# ===========================================================================
if(F4FLIGHT_BUILD_TOOLS)
    set(F4FLIGHT_TOOLS
        tools/dat2json.cpp
        tools/dat_validate.cpp
        tools/json_diff.cpp
    )

    foreach(tool_src ${F4FLIGHT_TOOLS})
        get_filename_component(tool_name "${tool_src}" NAME_WE)
        add_executable(${tool_name} "${tool_src}")
        target_link_libraries(${tool_name} PRIVATE f4flight)
        if(MSVC)
            target_compile_options(${tool_name} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${tool_name} PRIVATE -Wall -Wextra)
        endif()
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install rules
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS f4flight
    EXPORT f4flightTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/f4flight
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT f4flightTargets
    FILE f4flightTargets.cmake
    NAMESPACE f4flight::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/f4flight
)

# Generate the config file so find_package(f4flight) works
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/f4flightConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/f4flightConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/f4flightConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/f4flightConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/f4flightConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/f4flight
)
