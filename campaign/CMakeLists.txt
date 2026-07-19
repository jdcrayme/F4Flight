# campaign/CMakeLists.txt
#
# Build configuration for f4flight_campaign library (map representation,
# pathfinding, procedural scenario generation, and campaign simulation).

add_library(f4flight_campaign STATIC
    src/campaign_state.cpp
    src/campaign_io.cpp
    src/campaign_generator.cpp
    src/unit.cpp
)

target_link_libraries(f4flight_campaign PUBLIC f4flight_flight)

target_include_directories(f4flight_campaign PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(MSVC)
    target_compile_options(f4flight_campaign PRIVATE /W4 /utf-8)
else()
    target_compile_options(f4flight_campaign PRIVATE -Wall -Wextra)
endif()
