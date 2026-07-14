# flight/CMakeLists.txt
#
# Flight Model library — pure flight dynamics, NO AI dependency.
#
# Builds the `f4flight::flight` target. The digi AI library links against
# this; hosts that only need flight physics can link this directly.

# ---------------------------------------------------------------------------
# Library options
# ---------------------------------------------------------------------------
option(F4FLIGHT_FLIGHT_BUILD_SHARED "Build f4flight_flight as a shared library" OFF)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
set(F4FLIGHT_FLIGHT_SOURCES
    src/aerodynamics.cpp
    src/aircraft_config.cpp
    src/dat_loader.cpp
    src/engine.cpp
    src/eom.cpp
    src/fcs.cpp
    src/flight_model.cpp
    src/gear.cpp
    src/json_io.cpp
    src/config/f16c_config.cpp
    # Note: atmosphere.h is header-only (inline functions) — no .cpp
)

set(F4FLIGHT_FLIGHT_HEADERS
    include/f4flight/flight/f4flight.h
    include/f4flight/flight/core/constants.h
    include/f4flight/flight/core/types.h
    include/f4flight/flight/core/math.h
    include/f4flight/flight/core/lookup.h
    include/f4flight/flight/core/units.h
    include/f4flight/flight/core/trig.h
    include/f4flight/flight/atmosphere.h
    include/f4flight/flight/aircraft_config.h
    include/f4flight/flight/aircraft_state.h
    include/f4flight/flight/aerodynamics.h
    include/f4flight/flight/engine.h
    include/f4flight/flight/gear.h
    include/f4flight/flight/fcs.h
    include/f4flight/flight/eom.h
    include/f4flight/flight/flight_model.h
    include/f4flight/flight/dat_loader.h
    include/f4flight/flight/json_io.h
    include/f4flight/flight/config/f16c_config.h
)

add_library(f4flight_flight
    ${F4FLIGHT_FLIGHT_SOURCES}
    ${F4FLIGHT_FLIGHT_HEADERS}
)

if(F4FLIGHT_FLIGHT_BUILD_SHARED)
    set_target_properties(f4flight_flight PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()
set_target_properties(f4flight_flight PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(f4flight_flight
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(f4flight_flight PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Compiler warnings (same set as the old top-level CMakeLists)
# ---------------------------------------------------------------------------
if(MSVC)
    set(F4FLIGHT_MSVC_OPTIONS /W4 /permissive- /Zc:__cplusplus /Zc:preprocessor)
    if(F4FLIGHT_ENABLE_WERROR)
        list(APPEND F4FLIGHT_MSVC_OPTIONS /WX)
    endif()
    if(F4FLIGHT_USE_MSVC_UTF8)
        list(APPEND F4FLIGHT_MSVC_OPTIONS /utf-8)
    endif()
    target_compile_options(f4flight_flight PRIVATE ${F4FLIGHT_MSVC_OPTIONS})
else()
    set(F4FLIGHT_GCC_OPTIONS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(F4FLIGHT_ENABLE_WERROR)
        list(APPEND F4FLIGHT_GCC_OPTIONS -Werror)
    endif()
    target_compile_options(f4flight_flight PRIVATE ${F4FLIGHT_GCC_OPTIONS})
endif()

# ---------------------------------------------------------------------------
# Alias target so consumers can use f4flight::flight
# ---------------------------------------------------------------------------
add_library(f4flight::flight ALIAS f4flight_flight)
