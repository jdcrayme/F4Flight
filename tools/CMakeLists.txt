# tools/CMakeLists.txt
#
# Standalone CLI utilities that ship with the library. None are linked into
# the libraries themselves; they're thin mains over the public API.

# ---------------------------------------------------------------------------
# dat_tools/ — data conversion + validation
# ---------------------------------------------------------------------------
#   dat2json        Convert a Falcon 4 .dat file to f4flight JSON.
#   dat_validate    Load a JSON and run AircraftConfig::validate().
#   json_diff       Field-by-field diff of two JSON aircraft files.
set(F4FLIGHT_DAT_TOOLS
    dat_tools/dat2json.cpp
    dat_tools/dat_validate.cpp
    dat_tools/json_diff.cpp
)

foreach(tool_src ${F4FLIGHT_DAT_TOOLS})
    get_filename_component(tool_name "${tool_src}" NAME_WE)
    add_executable(${tool_name} "${tool_src}")
    target_link_libraries(${tool_name} PRIVATE f4flight::flight)
    if(MSVC)
        target_compile_options(${tool_name} PRIVATE /W4 /utf-8)
    else()
        target_compile_options(${tool_name} PRIVATE -Wall -Wextra)
    endif()
endforeach()

# ---------------------------------------------------------------------------
# viz/ — visualization + tracing
# ---------------------------------------------------------------------------
#   trace2svg       Convert a trace file to SVG.
#   f4flight_viz    Real-time visualization tool.
# These depend on trace.cpp (a shared util).
if(MSVC)
    # Skip viz tools on MSVC for now — they may need platform-specific code
    # (the trace lib uses POSIX file ops). Add later.
else()
    add_library(f4flight_trace STATIC viz/trace.cpp viz/trace.h)
    target_link_libraries(f4flight_trace PUBLIC f4flight::flight f4flight::digi)
    target_include_directories(f4flight_trace PUBLIC viz)

    add_executable(trace2svg viz/trace2svg.cpp)
    target_link_libraries(trace2svg PRIVATE f4flight_trace)
    target_compile_options(trace2svg PRIVATE -Wall -Wextra)

    # add_executable(f4flight_viz viz/f4flight_viz.cpp)
    # target_link_libraries(f4flight_viz PRIVATE f4flight_trace f4flight_test_framework)
    # target_compile_options(f4flight_viz PRIVATE -Wall -Wextra)
endif()
