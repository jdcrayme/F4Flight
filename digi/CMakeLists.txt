# digi/CMakeLists.txt
#
# Digi AI library — depends on f4flight::flight (one-way dependency).
#
# Builds the `f4flight::digi` target. Hosts that need AI link this; hosts
# that only need flight physics link `f4flight::flight` directly.

# ---------------------------------------------------------------------------
# Library options
# ---------------------------------------------------------------------------
option(F4FLIGHT_DIGI_BUILD_SHARED "Build f4flight_digi as a shared library" OFF)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
set(F4FLIGHT_DIGI_SOURCES
    src/digi_brain.cpp
    src/steering.cpp
    # maneuvers
    src/maneuvers/maneuver_primitives.cpp
    # ground
    src/ground/ground_avoid.cpp
    src/ground/ground_ops.cpp
    # defensive
    src/defensive/missile_defeat.cpp
    src/defensive/guns_jink.cpp
    src/defensive/collision_avoid.cpp
    src/defensive/handle_threat.cpp
    # offensive
    src/offensive/roll_and_pull.cpp
    src/offensive/guns_engage.cpp
    src/offensive/missile_engage.cpp
    src/offensive/bvr_engage.cpp
    src/offensive/merge.cpp
    # atc
    src/atc/atc_controller.cpp
    # sensors
    src/sensors/sensor_fusion.cpp
    src/sensors/radar_sensor.cpp
    src/sensors/rwr_sensor.cpp
    src/sensors/visual_sensor.cpp
    # Note: formation/ and comms/ and weapons/ are header-only — no .cpp
    # wingman
    src/wingman/wingman_state.cpp
    src/wingman/wingman_ai.cpp
    src/wingman/wingman_maneuvers.cpp
    # decision (Round 6: AirbaseCheck, SeparateCheck, CommandFlight)
    src/decision/decision_routines.cpp
)

set(F4FLIGHT_DIGI_HEADERS
    include/f4flight/digi/digi.h
    include/f4flight/digi/digi_brain.h
    include/f4flight/digi/digi_state.h
    include/f4flight/digi/digi_mode.h
    include/f4flight/digi/digi_skill.h
    include/f4flight/digi/digi_entity.h
    include/f4flight/digi/steering.h
    # maneuvers
    include/f4flight/digi/maneuvers/maneuver_primitives.h
    # ground
    include/f4flight/digi/ground/ground_avoid.h
    include/f4flight/digi/ground/ground_ops.h
    include/f4flight/digi/ground/ag_doctrine.h
    include/f4flight/digi/ground/ag_attack_phase.h
    # formation
    include/f4flight/digi/formation/formation_geometry.h
    # wingman
    include/f4flight/digi/wingman/wingman_state.h
    include/f4flight/digi/wingman/wingman_ai.h
    include/f4flight/digi/wingman/wingman_maneuvers.h
    # decision (Round 6)
    include/f4flight/digi/decision/decision_routines.h
    # defensive
    include/f4flight/digi/defensive/missile_defeat.h
    include/f4flight/digi/defensive/guns_jink.h
    include/f4flight/digi/defensive/collision_avoid.h
    include/f4flight/digi/defensive/handle_threat.h
    # offensive
    include/f4flight/digi/offensive/roll_and_pull.h
    include/f4flight/digi/offensive/guns_engage.h
    include/f4flight/digi/offensive/missile_engage.h
    include/f4flight/digi/offensive/bvr_engage.h
    include/f4flight/digi/offensive/merge.h
    # weapons
    include/f4flight/digi/weapons/weapon_spec.h
    include/f4flight/digi/weapons/weapon_types.h
    include/f4flight/digi/weapons/sms.h
    include/f4flight/digi/weapons/fire_control.h
    # comms
    include/f4flight/digi/comms/message.h
    include/f4flight/digi/comms/mailbox.h
    include/f4flight/digi/comms/message_bus.h
    # atc
    include/f4flight/digi/atc/atc_messages.h
    include/f4flight/digi/atc/atc_controller.h
    include/f4flight/digi/atc/taxi_graph.h
    # sensors
    include/f4flight/digi/sensors/sensor.h
    include/f4flight/digi/sensors/sensor_fusion.h
    include/f4flight/digi/sensors/sensor_picture.h
    include/f4flight/digi/sensors/radar_sensor.h
    include/f4flight/digi/sensors/rwr_sensor.h
    include/f4flight/digi/sensors/visual_sensor.h
)

add_library(f4flight_digi
    ${F4FLIGHT_DIGI_SOURCES}
    ${F4FLIGHT_DIGI_HEADERS}
)

if(F4FLIGHT_DIGI_BUILD_SHARED)
    set_target_properties(f4flight_digi PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()
set_target_properties(f4flight_digi PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# --- One-way dependency: digi depends on flight, never the reverse ---
target_link_libraries(f4flight_digi PUBLIC f4flight::flight)

target_include_directories(f4flight_digi
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(f4flight_digi PUBLIC cxx_std_17)

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
    target_compile_options(f4flight_digi PRIVATE ${F4FLIGHT_MSVC_OPTIONS})
else()
    set(F4FLIGHT_GCC_OPTIONS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(F4FLIGHT_ENABLE_WERROR)
        list(APPEND F4FLIGHT_GCC_OPTIONS -Werror)
    endif()
    target_compile_options(f4flight_digi PRIVATE ${F4FLIGHT_GCC_OPTIONS})
endif()

# ---------------------------------------------------------------------------
# Alias target so consumers can use f4flight::digi
# ---------------------------------------------------------------------------
add_library(f4flight::digi ALIAS f4flight_digi)
