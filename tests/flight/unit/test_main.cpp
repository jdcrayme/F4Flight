// f4flight unit tests - main entry point
// We do NOT use gtest_main; tests register themselves via TEST() macros.
// GoogleTest provides its own main() via gtest_main, but we link against
// GTest::gtest which doesn't pull in main. We provide a thin main here.

#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
