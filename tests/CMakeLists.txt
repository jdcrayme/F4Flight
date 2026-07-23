# tests/CMakeLists.txt
#
# Simplified and streamlined test suite configuration.
# Registers only the baseline scenario against the F-16 aircraft config.

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
add_library(f4flight_test_framework STATIC
    framework/scenario_framework.cpp
    framework/scenario_framework.h
    framework/scenario_assertions.h
)
target_link_libraries(f4flight_test_framework PUBLIC f4flight::digi f4flight_trace GTest::gtest)
target_include_directories(f4flight_test_framework PUBLIC framework)

# ===========================================================================
# Flight Model unit tests
# ===========================================================================
# Unit tests — one gtest executable covering all FM modules.
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

# ===========================================================================
# Digi AI tests
# ===========================================================================
# Unit tests — one gtest executable covering all AI modules.
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
set(F4FLIGHT_DIGI_SCENARIO_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/scenario_digi_baseline.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/digi/scenarios/scenario_digi_flightplan.cpp"
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
# Register simplified scenario × fixture combinations as ctest tests
# ===========================================================================
set(F16_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/aircraft/fighters/f16bk50.json")

# Digi AI baseline scenario (always run only on baseline F-16)
add_test(
    NAME digi_f16bk50_digi_baseline
    COMMAND f4flight_digi_scenarios "${F16_FIXTURE}" --scenario digi_baseline
)
set_tests_properties(digi_f16bk50_digi_baseline PROPERTIES
    LABELS "LowLevel;fighters;digi")

# Digi AI basic flightplan navigation scenario
add_test(
    NAME digi_f16bk50_digi_flightplan
    COMMAND f4flight_digi_scenarios "${F16_FIXTURE}" --scenario digi_flightplan
)
set_tests_properties(digi_f16bk50_digi_flightplan PROPERTIES
    LABELS "LowLevel;fighters;digi")
