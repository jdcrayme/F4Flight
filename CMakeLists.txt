cmake_minimum_required(VERSION 3.16)

project(f4flight
    VERSION 1.0.0
    DESCRIPTION "Modern C++17 flight model + digi AI library extracted from Falcon 4 / FreeFalcon"
    LANGUAGES CXX
)

# ===========================================================================
# Overview
# ===========================================================================
# F4Flight is split into two libraries with a one-way dependency:
#
#   flight/  →  f4flight::flight   (pure flight model, no AI dependency)
#   digi/    →  f4flight::digi     (AI brain; depends on f4flight::flight)
#
# Hosts that only need flight physics can link f4flight::flight directly.
# Hosts that need AI link f4flight::digi (which transitively brings flight).
#
# Directory layout:
#
#   flight/          Flight Model library (aerodynamics, engine, FCS, EOM, gear)
#   digi/            Digi AI library (brain, maneuvers, sensors, weapons, ATC)
#   tests/           Test suite (framework + flight tests + digi tests)
#   tools/           Standalone CLI tools (dat conversion, visualization)
#   cmake/           CMake package config templates
#
# ===========================================================================
# Options
# ===========================================================================
option(F4FLIGHT_BUILD_TESTS    "Build the test suite"                     ON)
option(F4FLIGHT_BUILD_TOOLS    "Build the standalone CLI tools"           ON)
option(F4FLIGHT_ENABLE_WERROR  "Treat warnings as errors"                 OFF)
option(F4FLIGHT_USE_MSVC_UTF8  "Force /utf-8 on MSVC"                      ON)

# ===========================================================================
# C++ standard
# ===========================================================================
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# Default to Release if unspecified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ===========================================================================
# Libraries
# ===========================================================================
add_subdirectory(flight)
add_subdirectory(digi)
add_subdirectory(campaign)

# ===========================================================================
# Tools (must come before tests — f4flight_test_framework links f4flight_trace)
# ===========================================================================
if(F4FLIGHT_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# ===========================================================================
# Tests
# ===========================================================================
if(F4FLIGHT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ===========================================================================
# Install rules
# ===========================================================================
include(GNUInstallDirs)

# Install both libraries + their headers
install(TARGETS f4flight_flight f4flight_digi f4flight_campaign
    EXPORT f4flightTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY flight/include/f4flight
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY campaign/include/f4flight
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY digi/include/f4flight
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
