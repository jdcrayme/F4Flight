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
#   trace2svg       Convert a trace file to a static 2-panel SVG.
#   trace2html      Convert one or more trace files to a self-contained
#                   interactive HTML report (dashboard + per-trace detail
#                   view with top-down flight path, altitude/speed/G
#                   time-series, scrubber, color-by controls).
#   f4flight_viz    Real-time 3D visualization tool (RayLib). Currently
#                   disabled — see the commented block below.
#
# f4flight_trace is a static lib shared by trace2svg, trace2html, and the
# test framework (so the scenario runner can record traces in-process and
# emit an HTML report via --html).
#
# All of these are pure standard C++ (no POSIX-specific calls in the lib
# itself; trace2html.cpp's realpath() is guarded by #ifdef __linux__).
# They build on GCC, Clang, and MSVC without platform guards.
add_library(f4flight_trace STATIC
    viz/trace.cpp viz/trace.h
    viz/html_report.cpp viz/html_report.h
)
target_link_libraries(f4flight_trace PUBLIC f4flight::flight f4flight::digi)
target_include_directories(f4flight_trace PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/viz)
if(MSVC)
    target_compile_options(f4flight_trace PRIVATE /W4 /utf-8)
else()
    target_compile_options(f4flight_trace PRIVATE -Wall -Wextra)
endif()

add_executable(trace2svg viz/trace2svg.cpp)
target_link_libraries(trace2svg PRIVATE f4flight_trace)
if(MSVC)
    target_compile_options(trace2svg PRIVATE /W4 /utf-8)
else()
    target_compile_options(trace2svg PRIVATE -Wall -Wextra)
endif()

add_executable(trace2html viz/trace2html.cpp)
target_link_libraries(trace2html PRIVATE f4flight_trace)
if(MSVC)
    target_compile_options(trace2html PRIVATE /W4 /utf-8)
else()
    target_compile_options(trace2html PRIVATE -Wall -Wextra)
endif()

add_executable(cmap_viz viz/cmap_viz.cpp viz/cmap_html_template.cpp)
target_link_libraries(cmap_viz PRIVATE f4flight_campaign)
if(MSVC)
    target_compile_options(cmap_viz PRIVATE /W4 /utf-8)
else()
    target_compile_options(cmap_viz PRIVATE -Wall -Wextra)
endif()

# add_executable(f4flight_viz viz/f4flight_viz.cpp)
# target_link_libraries(f4flight_viz PRIVATE f4flight_trace f4flight_test_framework)
# if(MSVC)
#     target_compile_options(f4flight_viz PRIVATE /W4 /utf-8)
# else()
#     target_compile_options(f4flight_viz PRIVATE -Wall -Wextra)
# endif()
