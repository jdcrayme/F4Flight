// f4flight unit tests - steering behaviors
#include "f4flight/steering.h"
#include "f4flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(PIDTest, ProportionalResponse) {
    PIDGains g; g.kp = 2.0; g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 0.1), 2.0, 1e-9);
    EXPECT_NEAR(pid.update(-1.0, 0.1), -2.0, 1e-9);
}

TEST(PIDTest, IntegralResponse) {
    PIDGains g; g.kp = 0.0; g.ki = 1.0; g.outputMin = -10.0; g.outputMax = 10.0;
    g.integMin = -10.0; g.integMax = 10.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 1.0), 1.0, 1e-9);
    EXPECT_NEAR(pid.update(1.0, 1.0), 2.0, 1e-9);
}

TEST(PIDTest, DerivativeResponse) {
    PIDGains g; g.kp = 0.0; g.ki = 0.0; g.kd = 1.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 0.1), 0.0, 1e-9);
    EXPECT_NEAR(pid.update(2.0, 0.1), 10.0, 1e-9);
}

TEST(PIDTest, OutputClamping) {
    PIDGains g; g.kp = 100.0; g.outputMin = -1.0; g.outputMax = 1.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 0.1), 1.0, 1e-9);
    EXPECT_NEAR(pid.update(-1.0, 0.1), -1.0, 1e-9);
}

TEST(PIDTest, AntiWindup) {
    PIDGains g; g.kp = 0.0; g.ki = 1.0; g.outputMin = -10.0; g.outputMax = 10.0;
    g.integMin = -2.0; g.integMax = 2.0;
    PID pid(g);
    for (int i = 0; i < 100; ++i) pid.update(1.0, 0.1);
    EXPECT_NEAR(pid.update(1.0, 0.1), 2.0, 1e-6);
}

TEST(PIDTest, Reset) {
    PIDGains g; g.kp = 0.0; g.ki = 1.0; g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    pid.update(5.0, 1.0);
    pid.reset();
    EXPECT_NEAR(pid.update(1.0, 1.0), 1.0, 1e-9);
}

// --- Utility functions ---
TEST(SteeringUtilsTest, HeadingError) {
    EXPECT_NEAR(headingError(0.5, 0.0), 0.5, 1e-9);
    EXPECT_NEAR(headingError(0.0, 0.5), -0.5, 1e-9);
    EXPECT_NEAR(headingError(4.0, 0.0), 4.0 - 2.0 * PI, 1e-9);
    EXPECT_NEAR(headingError(3.0, 0.0), 3.0, 1e-9);
}

TEST(SteeringUtilsTest, TurnCompensatedG) {
    AircraftState state;
    state.kin.phi = 0.0;
    EXPECT_NEAR(turnCompensatedG(state), 1.0, 1e-9);
    state.kin.phi = PI / 4.0;
    EXPECT_NEAR(turnCompensatedG(state), 1.0 / std::cos(PI / 4.0), 1e-6);
}

TEST(SteeringUtilsTest, VVICap) {
    double v1 = computeMaxVVI_fpm(100.0);
    double v2 = computeMaxVVI_fpm(1000.0);
    double v3 = computeMaxVVI_fpm(10000.0);
    EXPECT_GT(v2, v1);
    EXPECT_GT(v3, v2);
    EXPECT_GT(v1, 0.0);
}

// --- Behavior tests ---
class SteeringTest : public ::testing::Test {
protected:
    void SetUp() override { cfg_ = config::makeF16CConfig(); }
    AircraftConfig cfg_;
};

TEST_F(SteeringTest, AltitudeHoldClimbWhenBelow) {
    SteeringController sc;
    sc.setMaxGs(cfg_.geometry.maxGs);

    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(
        10000.0, 300.0, 300.0, 0.80, 1.0, 0.05, 300.0, 0.80));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(300.0));

    AircraftState state;
    state.kin.z = -5000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.mach = 0.5;
    state.loads.nzcgs = 1.0;
    state.aero.stallSpeed = 150.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    EXPECT_GT(out.pstick, 0.0);  // should pitch up to climb
}

TEST_F(SteeringTest, HeadingHoldTurnsTowardTarget) {
    SteeringController sc;
    sc.setMaxGs(9.0);
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(
        10000.0, 300.0));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.5));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(300.0));

    AircraftState state;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.mach = 0.5;
    state.loads.nzcgs = 1.0;
    state.aero.stallSpeed = 150.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    EXPECT_GT(out.rstick, 0.0);  // should bank right
}

TEST_F(SteeringTest, AltitudeHoldLevelWhenOnTarget) {
    SteeringController sc;
    sc.setMaxGs(9.0);
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(
        10000.0, 300.0));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(300.0));

    AircraftState state;
    state.kin.z = -10000.0;  // on target
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.mach = 0.5;
    state.loads.nzcgs = 1.0;
    state.aero.stallSpeed = 150.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // In level mode, pstick should be near the 1-G feed-forward
    EXPECT_GT(out.pstick, 0.0);
    EXPECT_LT(out.pstick, 0.5);  // not full deflection
}

TEST_F(SteeringTest, CombinedClimbAndTurn) {
    SteeringController sc;
    sc.setMaxGs(9.0);
    sc.setMaxBankAngle_deg(45.0);

    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(
        20000.0, 350.0, 350.0, 0.80, 1.0, 0.05, 350.0, 0.80));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(1.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(350.0));

    AircraftState state;
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 350.0;
    state.mach = 0.5;
    state.loads.nzcgs = 1.0;
    state.aero.stallSpeed = 150.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    EXPECT_GT(out.pstick, 0.0);  // climbing
    EXPECT_GT(out.rstick, 0.0);  // turning
    EXPECT_GT(out.throttle, 0.5); // at climb power
}

TEST_F(SteeringTest, SpeedHoldOutputsThrottle) {
    SteeringController sc;
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(400.0));

    AircraftState state;
    state.vcas = 300.0;

    // When vertical behavior is in level mode, throttle behavior runs
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(
        10000.0, 400.0));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));

    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.mach = 0.5;
    state.loads.nzcgs = 1.0;
    state.aero.stallSpeed = 150.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    EXPECT_GT(out.throttle, 0.0);
}
