# tests/CMakeLists.txt
#
# Test suite, organized by (library × level):
#
#   tests/framework/         Shared scenario framework (static lib)
#   tests/flight/unit/       Flight Model unit tests (gtest)
#   tests/flight/scenarios/  Flight Model scenario tests (bypass AI)
#   tests/digi/unit/         Digi AI unit tests (gtest)
#   tests/digi/scenarios/    Digi AI scenario tests (full AI + FM)
#   tests/fixtures/          Curated JSON aircraft files (grouped by category)
#
# The scenario framework is shared between flight/scenarios and digi/scenarios.
# It's built as a static library that both scenario executables link against.

# ---------------------------------------------------------------------------
# Fetch GoogleTest (shared by both unit test executables)
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---------------------------------------------------------------------------
# Scenario framework (shared static lib)
# ---------------------------------------------------------------------------
# Extracted from the old tests/maneuver/{maneuver_test,maneuver_test_assertions}
# into a reusable lib so both flight/scenarios and digi/scenarios can link it.
add_library(f4flight_test_framework STATIC
    framework/scenario_framework.cpp
    framework/scenario_framework.h
    framework/scenario_assertions.h
)
target_link_libraries(f4flight_test_framework PUBLIC f4flight::digi GTest::gtest)
target_include_directories(f4flight_test_framework PUBLIC framework)

# ===========================================================================
# Flight Model tests
# ===========================================================================
# Unit tests — one gtest executable covering all FM modules.
# test_main.cpp provides the gtest main() entry point.
file(GLOB F4FLIGHT_FLIGHT_UNIT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/flight/unit/*.cpp"
)
add_executable(f4flight_flight_tests ${F4FLIGHT_FLIGHT_UNIT_SOURCES})
target_link_libraries(f4flight_flight_tests PRIVATE f4flight::flight GTest::gtest)
if(MSVC)
    target_compile_options(f4flight_flight_tests PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
else()
    target_compile_options(f4flight_flight_tests PRIVATE -Wall -Wextra -Wpedantic
        -Wno-deprecated-declarations)
endif()
include(GoogleTest)
gtest_discover_tests(f4flight_flight_tests)

# Scenario tests — bypasses AI, drives PilotInput directly via inputOverride().
# main() is provided by the scenario framework lib.
file(GLOB F4FLIGHT_FLIGHT_SCENARIO_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/flight/scenarios/*.cpp"
)
add_executable(f4flight_flight_scenarios ${F4FLIGHT_FLIGHT_SCENARIO_SOURCES})
target_link_libraries(f4flight_flight_scenarios PRIVATE f4flight_test_framework)
if(MSVC)
    target_compile_options(f4flight_flight_scenarios PRIVATE /W4 /utf-8)
else()
    target_compile_options(f4flight_flight_scenarios PRIVATE -Wall -Wextra
        -Wno-deprecated-declarations)
endif()

# ===========================================================================
# Digi AI tests
# ===========================================================================
# Unit tests — one gtest executable covering all AI modules.
# Uses gtest_main (auto-generated main) since we don't have a custom test_main.
file(GLOB F4FLIGHT_DIGI_UNIT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/unit/*.cpp"
)
add_executable(f4flight_digi_tests ${F4FLIGHT_DIGI_UNIT_SOURCES})
target_link_libraries(f4flight_digi_tests PRIVATE f4flight::digi GTest::gtest GTest::gtest_main)
if(MSVC)
    target_compile_options(f4flight_digi_tests PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
else()
    target_compile_options(f4flight_digi_tests PRIVATE -Wall -Wextra -Wpedantic
        -Wno-deprecated-declarations)
endif()
gtest_discover_tests(f4flight_digi_tests)

# Scenario tests — drives the full AI + FM via SteeringController::compute().
# main() is provided by the scenario framework lib.
file(GLOB F4FLIGHT_DIGI_SCENARIO_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/*.cpp"
)
add_executable(f4flight_digi_scenarios ${F4FLIGHT_DIGI_SCENARIO_SOURCES})
target_link_libraries(f4flight_digi_scenarios PRIVATE f4flight_test_framework)
if(MSVC)
    target_compile_options(f4flight_digi_scenarios PRIVATE /W4 /utf-8)
else()
    target_compile_options(f4flight_digi_scenarios PRIVATE -Wall -Wextra
        -Wno-deprecated-declarations)
endif()

# ===========================================================================
# Register scenario × fixture combinations as ctest tests
# ===========================================================================
# Fixtures are organized by category subdirectory. The GLOB_RECURSE picks up
# all JSONs in all category subdirs, so adding a new category (e.g.
# fixtures/aircraft/helicopter/) automatically registers its aircraft.
file(GLOB_RECURSE F4FLIGHT_FIXTURES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/aircraft/*.json"
)

# Flight-direct scenarios (test FM physics without AI)
set(F4FLIGHT_FLIGHT_SCENARIOS
    fcs_engine
    fcs_glimits
    fcs_roll
)
# AI-driven scenarios (test AI behavior through the FM)
set(F4FLIGHT_DIGI_SCENARIOS
    ai_basic
    ai_flightplan
    ai_cruise
    digi_defensive
    digi_wvr
    digi_groundops
    digi_sensors
    digi_guns
)

foreach(fixture ${F4FLIGHT_FIXTURES})
    get_filename_component(ac_name "${fixture}" NAME_WE)
    # Extract the category subdirectory for the test name
    get_filename_component(category_dir "${fixture}" DIRECTORY)
    get_filename_component(category_name "${category_dir}" NAME)

    # Flight-direct scenarios
    foreach(sc ${F4FLIGHT_FLIGHT_SCENARIOS})
        add_test(
            NAME flight_${category_name}_${ac_name}_${sc}
            COMMAND f4flight_flight_scenarios "${fixture}" --scenario ${sc}
        )
    endforeach()

    # AI-driven scenarios
    foreach(sc ${F4FLIGHT_DIGI_SCENARIOS})
        add_test(
            NAME digi_${category_name}_${ac_name}_${sc}
            COMMAND f4flight_digi_scenarios "${fixture}" --scenario ${sc}
        )
    endforeach()
endforeach()
