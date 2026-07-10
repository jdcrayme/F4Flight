// f4flight unit tests - steering / autopilot
#include "f4flight/steering.h"
#include "f4flight/config/f16c_config.h"
#include "f4flight/flight_model.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(PIDTest, ProportionalResponse) {
    PIDGains g;
    g.kp = 2.0; g.ki = 0.0; g.kd = 0.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 0.1), 2.0, 1e-9);
    EXPECT_NEAR(pid.update(-1.0, 0.1), -2.0, 1e-9);
}

TEST(PIDTest, IntegralResponse) {
    PIDGains g;
    g.kp = 0.0; g.ki = 1.0; g.kd = 0.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    g.integMin = -10.0; g.integMax = 10.0;
    PID pid(g);
    // After 1 step of dt=1 with error=1, integ = 1, output = 1
    EXPECT_NEAR(pid.update(1.0, 1.0), 1.0, 1e-9);
    // After 2 steps, integ = 2, output = 2
    EXPECT_NEAR(pid.update(1.0, 1.0), 2.0, 1e-9);
}

TEST(PIDTest, DerivativeResponse) {
    PIDGains g;
    g.kp = 0.0; g.ki = 0.0; g.kd = 1.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    // First step: no derivative (no previous error)
    EXPECT_NEAR(pid.update(1.0, 0.1), 0.0, 1e-9);
    // Second step: error changed from 1 to 2, derivative = (2-1)/0.1 = 10
    EXPECT_NEAR(pid.update(2.0, 0.1), 10.0, 1e-9);
}

TEST(PIDTest, OutputClamping) {
    PIDGains g;
    g.kp = 100.0; g.ki = 0.0; g.kd = 0.0;
    g.outputMin = -1.0; g.outputMax = 1.0;
    PID pid(g);
    EXPECT_NEAR(pid.update(1.0, 0.1), 1.0, 1e-9);
    EXPECT_NEAR(pid.update(-1.0, 0.1), -1.0, 1e-9);
}

TEST(PIDTest, AntiWindup) {
    PIDGains g;
    g.kp = 0.0; g.ki = 1.0; g.kd = 0.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    g.integMin = -2.0; g.integMax = 2.0;
    PID pid(g);
    // Wind up the integrator
    for (int i = 0; i < 100; ++i) pid.update(1.0, 0.1);
    // Should be clamped at integMax = 2.0, so output = 2.0
    EXPECT_NEAR(pid.update(1.0, 0.1), 2.0, 1e-6);
}

TEST(PIDTest, Reset) {
    PIDGains g;
    g.kp = 0.0; g.ki = 1.0; g.kd = 0.0;
    g.outputMin = -10.0; g.outputMax = 10.0;
    PID pid(g);
    pid.update(5.0, 1.0);
    pid.reset();
    EXPECT_NEAR(pid.update(1.0, 1.0), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// SteeringController tests
// ---------------------------------------------------------------------------
class SteeringTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
    }
    AircraftConfig cfg_;
};

TEST_F(SteeringTest, DefaultModeIsManual) {
    SteeringController sc;
    EXPECT_EQ(sc.mode(), SteeringMode::Manual);
}

TEST_F(SteeringTest, ManualModeReturnsManualInput) {
    SteeringController sc;
    PilotInput manual;
    manual.throttle = 0.5;
    manual.pstick = 0.3;
    sc.setManualInput(manual);

    AircraftState state;
    PilotInput out = sc.compute(state, 0.1, 0.0);
    EXPECT_DOUBLE_EQ(out.throttle, 0.5);
    EXPECT_DOUBLE_EQ(out.pstick, 0.3);
}

TEST_F(SteeringTest, HeadingHoldTurnsTowardTarget) {
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);

    SteeringGoal goal;
    goal.hasHeading = true;
    goal.heading_rad = 0.5;  // target heading ~28 deg
    goal.hasAltitude = true;
    goal.altitude_ft = 10000.0;
    goal.hasSpeed = true;
    goal.speed_kts = 300.0;
    sc.setGoal(goal);

    AircraftState state;
    state.kin.psi = 0.0;  // currently heading North
    state.kin.phi = 0.0;  // level
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.aero.beta_deg = 0.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // Should command a right roll (positive rstick) to turn toward heading 0.5
    EXPECT_GT(out.rstick, 0.0);
}

TEST_F(SteeringTest, AltitudeHoldCommandsPitchUpWhenBelow) {
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);

    SteeringGoal goal;
    goal.hasAltitude = true;
    goal.altitude_ft = 10000.0;
    goal.hasSpeed = true;
    goal.speed_kts = 300.0;
    sc.setGoal(goal);

    AircraftState state;
    state.kin.z = -5000.0;  // 5000 ft, below target of 10000
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // Should command pitch up (positive pstick)
    EXPECT_GT(out.pstick, 0.0);
}

TEST_F(SteeringTest, SpeedHoldCommandsMoreThrottleWhenSlow) {
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);

    SteeringGoal goal;
    goal.hasAltitude = true;
    goal.altitude_ft = 10000.0;
    goal.hasSpeed = true;
    goal.speed_kts = 400.0;  // target 400 kts
    sc.setGoal(goal);

    AircraftState state;
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 300.0 * 1.6878;  // 300 kts in ft/s (slow)
    state.vcas = 300.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // Should command more throttle (positive, above the trim setting)
    EXPECT_GT(out.throttle, 0.5);
}

TEST_F(SteeringTest, WaypointModeAdvancesOnCapture) {
    SteeringController sc;
    sc.setMode(SteeringMode::Waypoint);
    sc.setWaypointCaptureRadius_ft(1000.0);

    std::vector<Vec3> wps = {
        {1000.0, 0.0, -10000.0},  // 1000 ft north
        {2000.0, 0.0, -10000.0},  // 2000 ft north
    };
    sc.setWaypoints(wps);

    SteeringGoal goal;
    goal.hasSpeed = true;
    goal.speed_kts = 300.0;
    sc.setGoal(goal);

    // Start at the first waypoint (within capture radius)
    AircraftState state;
    state.kin.x = 1000.0;
    state.kin.y = 0.0;
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 500.0;
    state.vcas = 300.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    sc.compute(state, 0.1, 0.0);
    // Should have advanced to waypoint 1
    EXPECT_EQ(sc.currentWaypointIndex(), 1u);
}

TEST_F(SteeringTest, TerrainFollowHoldsAGL) {
    SteeringController sc;
    sc.setMode(SteeringMode::TerrainFollow);

    SteeringGoal goal;
    goal.radarAltitude_ft = 500.0;
    goal.hasSpeed = true;
    goal.speed_kts = 400.0;
    sc.setGoal(goal);

    AircraftState state;
    state.kin.z = -400.0;  // 400 ft AGL (below 500 target)
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 600.0;
    state.vcas = 400.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // Below target AGL -> should command pitch up
    EXPECT_GT(out.pstick, 0.0);
}

TEST_F(SteeringTest, FormationFollowsLead) {
    SteeringController sc;
    sc.setMode(SteeringMode::Formation);

    SteeringGoal goal;
    goal.hasFormationLead = true;
    goal.leadPosition = Vec3{0.0, 0.0, -10000.0};
    goal.leadVelocity = Vec3{500.0, 0.0, 0.0};
    goal.formationOffset = Vec3{-200.0, 0.0, 0.0};  // 200 ft behind lead
    sc.setGoal(goal);

    AircraftState state;
    state.kin.x = 0.0;
    state.kin.y = 0.0;
    state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 400.0;  // slower than lead
    state.vcas = 250.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 1.0;
    state.qsom = 100.0;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // Slower than lead -> should command more throttle than idle
    EXPECT_GT(out.throttle, 0.3);
}

TEST_F(SteeringTest, SpeedProtectionReducesPitchAtHighG) {
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);
    sc.setMaxGs(9.0);

    SteeringGoal goal;
    goal.hasAltitude = true;
    goal.altitude_ft = 10000.0;
    goal.hasSpeed = true;
    goal.speed_kts = 300.0;
    sc.setGoal(goal);

    AircraftState state;
    state.kin.z = -5000.0;  // below target -> wants pitch up
    state.kin.zdot = 0.0;
    state.kin.psi = 0.0;
    state.kin.phi = 0.0;
    state.kin.vt = 300.0;
    state.vcas = 300.0;
    state.aero.stallSpeed = 150.0;
    state.loads.nzcgs = 8.0;  // already pulling 8 G
    state.qsom = 100.0;
    state.aero.clalpha = 0.05;
    state.aero.cl = 0.3;

    PilotInput out = sc.compute(state, 0.1, 0.0);
    // At 8 G (near max), the G protection should reduce the pitch-up command
    EXPECT_LT(out.pstick, 0.5);
}
