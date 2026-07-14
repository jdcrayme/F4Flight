// f4flight unit tests - lookup tables
#include "f4flight/flight/core/lookup.h"
#include <gtest/gtest.h>
#include <stdexcept>

using namespace f4flight;

TEST(Lookup1DTest, BasicInterpolation) {
    Lookup1D l({0.0, 1.0, 2.0}, {10.0, 20.0, 30.0});
    EXPECT_DOUBLE_EQ(l(0.0), 10.0);
    EXPECT_DOUBLE_EQ(l(1.0), 20.0);
    EXPECT_DOUBLE_EQ(l(0.5), 15.0);
    EXPECT_DOUBLE_EQ(l(1.5), 25.0);
}

TEST(Lookup1DTest, Clamping) {
    Lookup1D l({0.0, 1.0}, {10.0, 20.0});
    EXPECT_DOUBLE_EQ(l(-1.0), 10.0);
    EXPECT_DOUBLE_EQ(l(2.0), 20.0);
}

TEST(Lookup1DTest, SinglePoint) {
    Lookup1D l({5.0}, {42.0});
    EXPECT_DOUBLE_EQ(l(0.0), 42.0);
    EXPECT_DOUBLE_EQ(l(10.0), 42.0);
}

TEST(Lookup1DTest, ThrowsOnMismatch) {
    EXPECT_THROW(Lookup1D({0.0, 1.0}, {10.0}), std::invalid_argument);
    EXPECT_THROW(Lookup1D({}, {}), std::invalid_argument);
}

TEST(Lookup2DTest, BilinearCenter) {
    // 2x2 table: f(0,0)=0, f(1,0)=1, f(0,1)=2, f(1,1)=3
    Lookup2D l({0.0, 1.0}, {0.0, 1.0}, {0.0, 2.0, 1.0, 3.0});
    EXPECT_DOUBLE_EQ(l(0.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(l(1.0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(l(0.0, 1.0), 2.0);
    EXPECT_DOUBLE_EQ(l(1.0, 1.0), 3.0);
    EXPECT_DOUBLE_EQ(l(0.5, 0.5), 1.5);
}

TEST(Lookup2DTest, Clamping) {
    Lookup2D l({0.0, 1.0}, {0.0, 1.0}, {0.0, 2.0, 1.0, 3.0});
    EXPECT_DOUBLE_EQ(l(-1.0, -1.0), 0.0);
    EXPECT_DOUBLE_EQ(l(2.0, 2.0), 3.0);
}

TEST(Lookup2DTest, ThrowsOnSizeMismatch) {
    EXPECT_THROW(Lookup2D({0.0}, {0.0}, {0.0, 1.0}), std::invalid_argument);
    EXPECT_THROW(Lookup2D({}, {}, {}), std::invalid_argument);
}

TEST(Lookup2DTest, LargerTable) {
    // 3x3 table with f(x,y) = x + 10*y
    // data layout: data[ix*numY + iy]
    std::vector<double> data = {
        0.0,  10.0, 20.0,   // x=0
        1.0,  11.0, 21.0,   // x=1
        2.0,  12.0, 22.0,   // x=2
    };
    Lookup2D l({0.0, 1.0, 2.0}, {0.0, 1.0, 2.0}, data);
    EXPECT_DOUBLE_EQ(l(0.5, 0.5), 5.5);
    EXPECT_DOUBLE_EQ(l(1.5, 1.5), 16.5);
}
