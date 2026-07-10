// Development-only GoogleTest stub. This allows quick compilation of the tests
// without CMake/FetchContent. The real CMake build pulls in the official
// GoogleTest via FetchContent and does NOT use this file.
//
// To use: compile with -Itests/third_party instead of linking GTest::gtest.
// Do NOT use in production — the real GoogleTest provides better diagnostics,
// death tests, and parameterized tests.
#pragma once
#include "../minimal_gtest.h"
