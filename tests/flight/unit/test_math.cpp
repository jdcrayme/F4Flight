// f4flight unit tests - math helpers
#include "f4flight/flight/core/math.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(MathTest, LimitClamps) {
    EXPECT_DOUBLE_EQ(limit(5.0, -1.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(limit(-5.0, -1.0, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(limit(0.5, -1.0, 1.0), 0.5);
}

TEST(MathTest, LimitSymmetric) {
    EXPECT_DOUBLE_EQ(limit(5.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(limit(-5.0, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(limit(0.5, 1.0), 0.5);
}

TEST(MathTest, DeadBand) {
    EXPECT_DOUBLE_EQ(deadBand(0.5, 0.1), 0.4);
    EXPECT_DOUBLE_EQ(deadBand(-0.5, 0.1), -0.4);
    EXPECT_DOUBLE_EQ(deadBand(0.05, 0.1), 0.0);
}

TEST(MathTest, WrapPi) {
    EXPECT_NEAR(wrapPi(3.0 * PI / 2.0), -PI / 2.0, 1e-12);
    // The +/-PI boundary is ambiguous (both represent the same rotation);
    // accept either sign.
    const double w_pos = wrapPi(PI);
    EXPECT_NEAR(std::fabs(w_pos), PI, 1e-12);
    const double w_neg = wrapPi(-PI);
    EXPECT_NEAR(std::fabs(w_neg), PI, 1e-12);
    EXPECT_NEAR(wrapPi(0.5), 0.5, 1e-12);
}

TEST(MathTest, Wrap2Pi) {
    EXPECT_NEAR(wrap2Pi(3.0 * PI), PI, 1e-12);
    EXPECT_NEAR(wrap2Pi(-0.5), 2.0 * PI - 0.5, 1e-12);
}

TEST(MathTest, Lerp) {
    EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 1.0), 10.0);
    EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
}

TEST(MathTest, LagFilterStepResponse) {
    LagFilter f;
    // Step input of 1.0 with tau=1.0, dt=0.1
    // Tustin: y = (dt*(u_n + u_{n-1}) + y_{n-1}*(2*tau - dt)) / (2*tau + dt)
    // With u_n=1, u_{n-1}=0, y_{n-1}=0, tau=1, dt=0.1:
    // y = (0.1*1 + 0*1.9) / 2.1 = 0.1/2.1
    double y = f.step(1.0, 1.0, 0.1);
    EXPECT_NEAR(y, 0.1 / 2.1, 1e-9);

    // After many steps, should converge to 1.0 (DC gain = 1)
    for (int i = 0; i < 1000; ++i) y = f.step(1.0, 1.0, 0.1);
    EXPECT_NEAR(y, 1.0, 1e-3);
}

TEST(MathTest, IntegratorTrapezoidal) {
    Integrator integ;
    // Integrate constant 1.0 starting from u_prev=0:
    //   step 1: y = 0 + 0.5*0.1*(1+0) = 0.05
    //   step 2+: y += 0.5*0.1*(1+1) = 0.1
    // After 10 steps: 0.05 + 9*0.1 = 0.95
    for (int i = 0; i < 10; ++i) integ.step(1.0, 0.1);
    EXPECT_NEAR(integ.y_prev, 0.95, 1e-9);

    // Prime u_prev to 1.0 to remove startup transient, then integrate
    Integrator integ2;
    integ2.u_prev = 1.0;
    for (int i = 0; i < 10; ++i) integ2.step(1.0, 0.1);
    EXPECT_NEAR(integ2.y_prev, 1.0, 1e-9);
}

TEST(MathTest, AdamsBash2) {
    AdamsBash2 ab;
    // AB2 with startup transient (u_prev starts at 0):
    //   step 1: y = 0 + 0.5*0.1*(3*1 - 0) = 0.15
    //   step 2+: y += 0.5*0.1*(3*1 - 1) = 0.1
    // After 10 steps: 0.15 + 9*0.1 = 1.05
    ab.step(1.0, 0.1);
    for (int i = 0; i < 9; ++i) ab.step(1.0, 0.1);
    EXPECT_NEAR(ab.y_prev, 1.05, 1e-9);

    // Prime u_prev to remove transient
    AdamsBash2 ab2;
    ab2.u_prev = 1.0;
    ab2.u_now = 1.0;
    ab2.step(1.0, 0.1);
    for (int i = 0; i < 9; ++i) ab2.step(1.0, 0.1);
    EXPECT_NEAR(ab2.y_prev, 1.0, 1e-9);
}
