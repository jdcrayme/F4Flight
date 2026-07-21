# tests/CMakeLists.txt
#
# Test suite, organized by (library × level × tier):
#
#   tests/framework/                          Shared scenario framework (static lib)
#   tests/flight/unit/                        Flight Model unit tests (gtest)
#   tests/flight/scenarios/                   Flight Model scenario tests (bypass AI)
#   tests/digi/unit/                          Digi AI unit tests (gtest)
#   tests/digi/scenarios/low_level/           Digi AI scenarios — one behavior per test
#   tests/digi/scenarios/high_level/          Digi AI scenarios — chains of behaviors
#   tests/digi/scenarios/e2e/                 Digi AI scenarios — full AMIS_* missions
#   tests/fixtures/                           Curated JSON aircraft files (grouped by category)
#
# The scenario framework is shared between flight/scenarios and digi/scenarios.
# It's built as a static library that both scenario executables link against.
#
# Each digi scenario is tagged with a ctest LABELS property indicating its
# tier (LowLevel, HighLevel, EndToEnd). Use `ctest -L LowLevel` to run only
# the low-level tier, etc. The cascade workflow (E2E -> High -> Low) is
# implemented in the binary itself via `--cascade`.

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
# Links f4flight_trace so the runner can record traces and emit HTML reports
# via --html / --open.
add_library(f4flight_test_framework STATIC
    framework/scenario_framework.cpp
    framework/scenario_framework.h
    framework/scenario_assertions.h
)
target_link_libraries(f4flight_test_framework PUBLIC f4flight::digi f4flight_trace GTest::gtest)
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
set_property(TARGET f4flight_flight_scenarios PROPERTY DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")  
set_target_properties(f4flight_flight_scenarios PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "tests/fixtures/aircraft/fighters/f16bk50.json --open") 
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
#
# The 3-tier subfolder layout (low_level/, high_level/, e2e/) is purely an
# on-disk organizational choice — all three subfolders are linked into the
# SAME executable. Each scenario self-registers via RegisterScenario at
# process start; the binary selects which to run via --scenario / --level /
# --cascade flags.
file(GLOB_RECURSE F4FLIGHT_DIGI_SCENARIO_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/low_level/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/high_level/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/e2e/*.cpp"
)


add_executable(f4flight_digi_scenarios ${F4FLIGHT_DIGI_SCENARIO_SOURCES})
set_property(TARGET f4flight_digi_scenarios PROPERTY DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")  
set_target_properties(f4flight_digi_scenarios PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "tests/fixtures/aircraft/fighters/f16bk50.json --open") 
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

# Flight-direct scenarios (test FM physics without AI) - run on ALL aircraft
set(F4FLIGHT_FLIGHT_SCENARIOS
    fcs_engine
    fcs_glimits
    fcs_roll
)

# -----------------------------------------------------------------------------
# 3-tier DIGI scenario registration
#
# Each list corresponds to one tier from scenario_framework.h's TestTier enum.
# The tier name is also used as a ctest LABEL so `ctest -L LowLevel` works.
#
#   LowLevel  — one behavior per scenario (Takeoff, Climb, BVR Engage, ...)
#   HighLevel — chains of behaviors (Departure, AAR, Air-To-Air Engage, ...)
#   EndToEnd  — full AMIS_* missions (BARCAP, INTERCEPT, ESCORT, ...)
#
# Aircraft-category gating still applies — combat scenarios only run on
# fighters/attack, etc. See the foreach() block below.
# -----------------------------------------------------------------------------

# Low-level scenarios — one behavior per test. Covers every entry in the
# FreeFalcon DigiMode enum that has a corresponding test.
#
# Naming convention:
#   - Existing single-behavior scenarios keep their original "digi_*" / "ai_*"
#     name (digi_bvr, digi_wvr, ai_cruise, ...). These were already low-level.
#   - Scenarios split out of a multi-behavior chain use the "low_*" prefix
#     (low_taxi, low_takeoff, low_aar_vector, ...). These are NEW.
set(F4FLIGHT_DIGI_SCENARIOS_LOW_LEVEL
    # Autopilot / navigation behaviors
    ai_cruise
    ai_flightplan
    digi_loiter_orbit
    digi_rtb
    # Ground ops behaviors (split out of digi_groundops)
    low_taxi
    low_takeoff
    low_landing
    low_approach
    # AAR sub-behaviors (split out of digi_aar)
    low_aar_vector
    low_aar_pre_contact
    low_aar_contact
    low_aar_disconnect
    # Formation sub-behaviors (split out of digi_formation_maneuver)
    digi_formation
    digi_formation_types
    digi_formation_maneuver
    low_formation_turn
    # Air-to-air offensive behaviors
    digi_bvr
    digi_wvr
    digi_missile_engage
    digi_guns
    digi_guns_rear
    digi_merge
    digi_separate
    # Air-to-air tactical behaviors (split out of digi_tactics)
    low_roop
    low_overb
    # Air-to-air defensive behaviors (split out of digi_defensive)
    digi_collision
    low_missile_defeat
    low_guns_jink
    # Air-to-ground behaviors (split out of digi_ground_attack_profiles)
    digi_ground_attack
    low_ground_attack_dive
    low_ground_attack_toss
    low_ground_attack_high
    low_sead_harm
    # Sensors
    digi_sensors
)

# High-level scenarios — chains of related behaviors.
set(F4FLIGHT_DIGI_SCENARIOS_HIGH_LEVEL
    ai_basic
    digi_groundops
    digi_aar
    digi_defensive
    digi_ground_attack_profiles
    digi_tactics
    digi_wvr_defensive
    # New high-level chains (composed of low-level behaviors above).
    high_departure
    high_loiter_station
    high_formation_joinup
    high_air_to_air_engage
    high_air_to_ground
    high_recovery
    high_defensive_chain
)

# End-to-end scenarios — full AMIS_* mission types.
set(F4FLIGHT_DIGI_SCENARIOS_END_TO_END
    # Legacy generic E2E scenarios (kept for backward compat with old ctest
    # matrices). These were the original E2E tests before the AMIS_*
    # classification was introduced.
    digi_e2e_mission
    digi_e2e_formation
    digi_e2e_aar
    digi_e2e_ground_attack
    # New AMIS_*-classified E2E scenarios (core fighter missions).
    e2e_barcap
    e2e_tarcap
    e2e_sweep
    e2e_intercept
    e2e_escort
)

# -----------------------------------------------------------------------------
# Aircraft-category gating per scenario
# -----------------------------------------------------------------------------
# Scenarios that should run on ALL aircraft (fighters, attack, bomber, transport).
set(F4FLIGHT_DIGI_SCENARIOS_ALL_AC
    ai_basic
    ai_flightplan
    ai_cruise
    digi_rtb
    digi_loiter_orbit
    digi_groundops
    low_taxi
    low_takeoff
    low_landing
    low_approach
    high_departure
    high_loiter_station
    high_recovery
)

# Combat scenarios — only run on fighters and attack aircraft.
set(F4FLIGHT_DIGI_SCENARIOS_COMBAT_AC
    digi_defensive
    digi_wvr
    digi_wvr_defensive
    digi_sensors
    digi_guns
    digi_guns_rear
    digi_bvr
    digi_missile_engage
    digi_merge
    digi_collision
    digi_separate
    digi_tactics
    digi_e2e_mission
    low_roop
    low_overb
    low_missile_defeat
    low_guns_jink
    high_air_to_air_engage
    high_defensive_chain
    e2e_barcap
    e2e_tarcap
    e2e_sweep
    e2e_intercept
    e2e_escort
)

# Formation scenarios — only run on fighters and attack aircraft.
set(F4FLIGHT_DIGI_SCENARIOS_FORMATION_AC
    digi_formation
    digi_formation_types
    digi_formation_maneuver
    digi_e2e_formation
    low_formation_turn
    high_formation_joinup
)

# Strike & AAR scenarios — run on fighters, attack, and bombers.
set(F4FLIGHT_DIGI_SCENARIOS_STRIKE_AC
    digi_aar
    digi_e2e_aar
    digi_ground_attack
    digi_ground_attack_profiles
    digi_e2e_ground_attack
    low_ground_attack_dive
    low_ground_attack_toss
    low_ground_attack_high
    low_sead_harm
    low_aar_vector
    low_aar_pre_contact
    low_aar_contact
    low_aar_disconnect
    high_air_to_ground
)

# -----------------------------------------------------------------------------
# Helper: register a single ctest with tier + category labels.
# -----------------------------------------------------------------------------
# We use LABELS so `ctest -L LowLevel` (or HighLevel, EndToEnd) selects
# exactly that tier, and `ctest -L fighters` selects only the fighter
# aircraft subset. Multiple labels are allowed (comma-separated).
function(f4flight_add_digi_test name fixture category_name ac_name scenario_name tier_label)
    add_test(
        NAME ${name}
        COMMAND f4flight_digi_scenarios "${fixture}" --scenario ${scenario_name}
    )
    set_tests_properties(${name} PROPERTIES LABELS "${tier_label};${category_name};digi")
endfunction()

# Map scenario name to its tier label.
function(f4flight_tier_for_scenario scenario_name out_var)
    set(_tier "")
    foreach(s ${F4FLIGHT_DIGI_SCENARIOS_LOW_LEVEL})
        if(s STREQUAL scenario_name)
            set(_tier "LowLevel")
            break()
        endif()
    endforeach()
    if(_tier STREQUAL "")
        foreach(s ${F4FLIGHT_DIGI_SCENARIOS_HIGH_LEVEL})
            if(s STREQUAL scenario_name)
                set(_tier "HighLevel")
                break()
            endif()
        endforeach()
    endif()
    if(_tier STREQUAL "")
        foreach(s ${F4FLIGHT_DIGI_SCENARIOS_END_TO_END})
            if(s STREQUAL scenario_name)
                set(_tier "EndToEnd")
                break()
            endif()
        endforeach()
    endif()
    set(${out_var} "${_tier}" PARENT_SCOPE)
endfunction()

foreach(fixture ${F4FLIGHT_FIXTURES})
    get_filename_component(ac_name "${fixture}" NAME_WE)
    # Extract the category subdirectory for the test name
    get_filename_component(category_dir "${fixture}" DIRECTORY)
    get_filename_component(category_name "${category_dir}" NAME)

    # 1. Flight-direct scenarios (always run on all aircraft)
    foreach(sc ${F4FLIGHT_FLIGHT_SCENARIOS})
        add_test(
            NAME flight_${category_name}_${ac_name}_${sc}
            COMMAND f4flight_flight_scenarios "${fixture}" --scenario ${sc}
        )
        set_tests_properties(flight_${category_name}_${ac_name}_${sc} PROPERTIES
            LABELS "${category_name};flight")
    endforeach()

    # 2. AI autopilot / basic / navigation scenarios (always run on all aircraft)
    foreach(sc ${F4FLIGHT_DIGI_SCENARIOS_ALL_AC})
        f4flight_tier_for_scenario(${sc} _tier)
        f4flight_add_digi_test(
            digi_${category_name}_${ac_name}_${sc}
            "${fixture}" ${category_name} ${ac_name} ${sc}
            "${_tier}")
    endforeach()

    # 3. AI combat scenarios (only run on fighters and attack aircraft)
    if(category_name STREQUAL "fighters" OR category_name STREQUAL "attack")
        foreach(sc ${F4FLIGHT_DIGI_SCENARIOS_COMBAT_AC})
            f4flight_tier_for_scenario(${sc} _tier)
            f4flight_add_digi_test(
                digi_${category_name}_${ac_name}_${sc}
                "${fixture}" ${category_name} ${ac_name} ${sc}
                "${_tier}")
        endforeach()
    endif()

    # 4. AI formation scenarios (only run on fighters and attack aircraft)
    if(category_name STREQUAL "fighters" OR category_name STREQUAL "attack")
        foreach(sc ${F4FLIGHT_DIGI_SCENARIOS_FORMATION_AC})
            f4flight_tier_for_scenario(${sc} _tier)
            f4flight_add_digi_test(
                digi_${category_name}_${ac_name}_${sc}
                "${fixture}" ${category_name} ${ac_name} ${sc}
                "${_tier}")
        endforeach()
    endif()

    # 5. AI strike & AAR scenarios (run on fighters, attack, and bombers)
    if(category_name STREQUAL "fighters" OR category_name STREQUAL "attack" OR category_name STREQUAL "bomber")
        foreach(sc ${F4FLIGHT_DIGI_SCENARIOS_STRIKE_AC})
            f4flight_tier_for_scenario(${sc} _tier)
            f4flight_add_digi_test(
                digi_${category_name}_${ac_name}_${sc}
                "${fixture}" ${category_name} ${ac_name} ${sc}
                "${_tier}")
        endforeach()
    endif()
endforeach()
