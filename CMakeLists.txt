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
option(F4FLIGHT_BUILD_EXAMPLES "Build the f4flight example programs"       ON)
option(F4FLIGHT_BUILD_SHARED   "Build f4flight as a shared library (DLL)"  OFF)
option(F4FLIGHT_ENABLE_WERROR "Treat warnings as errors"                   OFF)
option(F4FLIGHT_USE_MSVC_UTF8 "Force /utf-8 on MSVC"                       ON)

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

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------
if(F4FLIGHT_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    )
    # On Windows / MSVC, prevent overriding the parent project's compiler/linker options
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    set(F4FLIGHT_TEST_SOURCES
        tests/test_main.cpp
        tests/test_math.cpp
        tests/test_types.cpp
        tests/test_lookup.cpp
        tests/test_units.cpp
        tests/test_atmosphere.cpp
        tests/test_aerodynamics.cpp
        tests/test_engine.cpp
        tests/test_gear.cpp
        tests/test_fcs.cpp
        tests/test_eom.cpp
        tests/test_flight_model.cpp
        tests/test_f16c_config.cpp
        tests/test_json_io.cpp
        tests/test_dat_loader.cpp
        tests/test_steering.cpp
    )

    add_executable(f4flight_tests ${F4FLIGHT_TEST_SOURCES})
    target_link_libraries(f4flight_tests PRIVATE f4flight GTest::gtest)

    if(MSVC)
        target_compile_options(f4flight_tests PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
    else()
        target_compile_options(f4flight_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(f4flight_tests)


    
    set(F4FLIGHT_MANUVER_TEST_SOURCES
    tests/ManuverTest/maneuver_test.cpp
    tests/ManuverTest/scenarios/scenario_basic.cpp
    tests/ManuverTest/scenarios/scenario_flightplan.cpp
    tests/ManuverTest/scenarios/scenario_combat.cpp
    )

    add_executable(maneuver_test ${F4FLIGHT_MANUVER_TEST_SOURCES})

    target_include_directories(maneuver_test PRIVATE tests/ManuverTest/)

    target_link_libraries(maneuver_test PRIVATE f4flight)
    if(MSVC)
        target_compile_options(maneuver_test PRIVATE /W4 /utf-8)
    else()
        target_compile_options(maneuver_test PRIVATE -Wall -Wextra)
    endif()

    # Register the maneuver_test scenarios as ctest tests. Each aircraft x
    # scenario combination becomes a separate test, so `ctest` runs the full
    # maneuver suite alongside the gtest unit tests.
    # We only register the "basic" and "flightplan" scenarios (combat is a
    # placeholder that always reports 0/1 by design).
    set(F4FLIGHT_MANEUVER_AIRCRAFT
        f16bk50 f15c f18c f14b mig29a su27 a10a
        mirage2k5 rafalec ef2000 b52h c130 sr71)
    set(F4FLIGHT_MANEUVER_SCENARIOS basic flightplan)
    foreach(ac ${F4FLIGHT_MANEUVER_AIRCRAFT})
        foreach(sc ${F4FLIGHT_MANEUVER_SCENARIOS})
            add_test(
                NAME maneuver_${ac}_${sc}
                COMMAND maneuver_test ${CMAKE_CURRENT_SOURCE_DIR}/data/${ac}.json --scenario ${sc}
            )
        endforeach()
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(F4FLIGHT_BUILD_EXAMPLES)
    add_executable(simple_sim examples/simple_sim.cpp)
    target_link_libraries(simple_sim PRIVATE f4flight)
    if(MSVC)
        target_compile_options(simple_sim PRIVATE /W4 /utf-8)
    else()
        target_compile_options(simple_sim PRIVATE -Wall -Wextra)
    endif()

    add_executable(steering_demo examples/steering_demo.cpp)
    target_link_libraries(steering_demo PRIVATE f4flight)
    if(MSVC)
        target_compile_options(steering_demo PRIVATE /W4 /utf-8)
    else()
        target_compile_options(steering_demo PRIVATE -Wall -Wextra)
    endif()

    add_executable(tune_diag examples/tune_diag.cpp)
    target_link_libraries(tune_diag PRIVATE f4flight)
    if(MSVC)
        target_compile_options(tune_diag PRIVATE /W4 /utf-8)
    else()
        target_compile_options(tune_diag PRIVATE -Wall -Wextra)
    endif()

    add_executable(perf_check examples/perf_check.cpp)
    target_link_libraries(perf_check PRIVATE f4flight)
    if(MSVC)
        target_compile_options(perf_check PRIVATE /W4 /utf-8)
    else()
        target_compile_options(perf_check PRIVATE -Wall -Wextra)
    endif()

    # dat2json converter tool
    add_executable(dat2json tools/dat2json.cpp)
    target_link_libraries(dat2json PRIVATE f4flight)
    if(MSVC)
        target_compile_options(dat2json PRIVATE /W4 /utf-8)
    else()
        target_compile_options(dat2json PRIVATE -Wall -Wextra)
    endif()
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
