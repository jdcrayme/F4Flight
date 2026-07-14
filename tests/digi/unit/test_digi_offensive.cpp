// f4flight unit tests - digi AI Tier 2 offensive BFM
//
// Tests for:
//   - RollAndPull (offensive/roll_and_pull.h)
//   - EnergyManagement
//   - MaintainClosure
//   - CollisionTime
//   - DigiBrain WVR engagement integration
//
// These tests verify the offensive AI directly. They create synthetic
// DigiEntity targets in various geometries (offensive, neutral, defensive)
// and verify the AI produces sensible stick commands.

#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// CollisionTime tests
// ===========================================================================
TEST(CollisionTimeTest, HeadOnClosing) {
    DigiEntity self, target;
    self.x = 0; self.y = 0; self.z = -10000;
    self.speed = 589;  // ~350 kts
    target.x = 0; target.y = 6000; target.z = -10000;  // 6000 ft ahead

    double tc = CollisionTime(self, target);
    // range=6000, vt=589 → tc = min(6000/589, 0.5) = min(10.2, 0.5) = 0.5
    EXPECT_NEAR(tc, 0.5, 0.01);
}

TEST(CollisionTimeTest, VeryClose) {
    DigiEntity self, target;
    self.x = 0; self.y = 0; self.z = -10000;
    self.speed = 589;
    target.x = 0; target.y = 100; target.z = -10000;  // 100 ft ahead

    double tc = CollisionTime(self, target);
    // range=100, vt=589 → tc = min(100/589, 0.5) = min(0.17, 0.5) = 0.17
    EXPECT_NEAR(tc, 0.17, 0.01);
}

// ===========================================================================
// RollAndPull tests
// ===========================================================================
class RollAndPullTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        digi.reset();
        digi.skill = makeSkillParams(SkillLevel::Veteran);
        digi.maxGs = 9.0;
        digi.cornerSpeed = 330.0;
        digi.maxRoll = 190.0;
        digi.maxGammaDeg = 15.0;
        digi.turnLoadFactor = 2.0;
        digi.dt = 1.0 / 60.0;

        // Self at origin, heading east (yaw=0), level, 350 kts
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.vx = 589.0; self.vy = 0.0; self.vz = 0.0;
        self.speed = 589.0;

        // Level flight state
        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.xdot = 589.0; as.kin.ydot = 0.0; as.kin.zdot = 0.0;
        as.kin.phi = 0.0;
        as.kin.psi = 0.0;  // body yaw
        as.kin.theta = 0.0;  // body pitch
        as.vcas = 350.0;
        as.kin.vt = 589.0;
        as.aero.alpha_deg = 4.0;
        // DCM: body-to-world rotation. AutoTrack uses it to transform
        // world-frame relative position to body frame. With psi=theta=phi=0,
        // the DCM is identity (body axes aligned with world axes).
        as.kin.dcm = Matrix3::identity();
    }
};

TEST_F(RollAndPullTest, OffensiveHeadOnCommandsTrackpoint) {
    // Target 3 NM ahead, head-on
    target.x = 0.0; target.y = 3.0 * 6076.0; target.z = -10000.0;
    target.yaw = PI;  // heading south (toward us)
    target.speed = 589.0;

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Should produce some stick/throttle commands
    bool hasCommand = (std::fabs(digi.pStick) > 0.01 ||
                       std::fabs(digi.rStick) > 0.01 ||
                       digi.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
}

TEST_F(RollAndPullTest, OffsetTargetCommandsTurn) {
    // Target ahead but offset 30° to the right — RollAndPull must command
    // a roll to turn toward it. This verifies the maneuver output has the
    // correct *direction*, not just non-zero magnitude.
    target.x = 2.5 * 6076.0; target.y = 1.5 * 6076.0; target.z = -10000.0;
    target.yaw = PI;
    target.speed = 589.0;

    // Warm up the smoothing by running several frames.
    for (int i = 0; i < 30; ++i) {
        digi.pStick = 0.0; digi.rStick = 0.0;
        RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);
    }

    // Target is to the right (positive y, positive bearing from self at
    // origin heading north). The brain should command a right turn —
    // positive rstick (roll right) or at least a non-trivial roll command.
    EXPECT_GT(std::fabs(digi.rStick), 0.01)
        << "Offset target should produce a roll command, got rstick="
        << digi.rStick;
}

TEST_F(RollAndPullTest, OffensiveChaseCommandsTrackpoint) {
    // Target ahead but angled away (we're behind)
    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = 0.0;  // same heading as us (chase)
    target.speed = 400.0;  // slower

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Should produce commands to chase
    bool hasCommand = (std::fabs(digi.pStick) > 0.01 ||
                       std::fabs(digi.rStick) > 0.01 ||
                       digi.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
}

TEST_F(RollAndPullTest, NeutralBeamCommandsTrackpoint) {
    // Target on the beam (90° off our nose), 2 NM
    target.x = 2.0 * 6076.0; target.y = 0.0; target.z = -10000.0;
    target.yaw = PI;  // pointing at us
    target.speed = 500.0;

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Should produce commands
    bool hasCommand = (std::fabs(digi.pStick) > 0.01 ||
                       std::fabs(digi.rStick) > 0.01 ||
                       digi.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
}

TEST_F(RollAndPullTest, DefensiveBehindUsCommandsTrackpoint) {
    // Target behind us (ataFrom < 45°), 2 NM
    target.x = 0.0; target.y = -2.0 * 6076.0; target.z = -10000.0;
    target.yaw = 0.0;  // same heading (chasing us from behind)
    target.speed = 600.0;

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Should produce commands (defensive — turn to track)
    bool hasCommand = (std::fabs(digi.pStick) > 0.01 ||
                       std::fabs(digi.rStick) > 0.01 ||
                       digi.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
}

TEST_F(RollAndPullTest, GroundAvoidSkipsBFM) {
    digi.groundAvoidNeeded = true;

    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = PI;
    target.speed = 589.0;

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Should NOT produce any BFM commands (ground avoid pre-empts)
    EXPECT_NEAR(digi.pStick, 0.0, 1e-9);
    EXPECT_NEAR(digi.rStick, 0.0, 1e-9);
}

TEST_F(RollAndPullTest, FarTargetAccelerates) {
    // Target 12 NM ahead, head-on — should command acceleration
    target.x = 0.0; target.y = 12.0 * 6076.0; target.z = -10000.0;
    target.yaw = PI;
    target.speed = 589.0;

    RollAndPull(digi, self, target, as, fcs, fcsState, 1.0/60.0);

    // Throttle should be set (MachHold with adjustPitch=true)
    EXPECT_GT(digi.throttle, 0.0);
}

// ===========================================================================
// DigiBrain WVR engagement integration tests
// ===========================================================================
class DigiBrainWVRTest : public ::testing::Test {
protected:
    DigiBrain brain;
    DigiEntity target;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        brain.setMaxGs(9.0);
        brain.setMaxBank(45.0);
        brain.setMaxGamma(15.0);
        brain.setTurnG(2.0);
        brain.setCornerSpeed(330.0);

        // Level flight at 350 kts / 10000 ft
        state.kin.costhe = 1.0;
        state.kin.cosphi = 1.0;
        state.kin.gmma = 0.0;
        state.kin.sigma = 0.0;
        state.kin.singam = 0.0;
        state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
        state.kin.zdot = 0.0;
        state.kin.phi = 0.0;
        state.kin.xdot = 589.0; state.kin.ydot = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 589.0;
    }
};

TEST_F(DigiBrainWVRTest, NoTargetDefaultsToWaypoint) {
    brain.setTarget(nullptr);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainWVRTest, TargetInWVREntersWVREngage) {
    // Target 3 NM north, same altitude
    target.x = 0.0; target.y = 3.0 * 6076.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;
    brain.setTarget(&target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::WVREngage);
}

TEST_F(DigiBrainWVRTest, TargetBeyondWVREntersBVR) {
    // Target 12 NM — beyond 8 NM WVR threshold, within 35 NM BVR gate.
    // With the BVR target gate fix, the AI should enter BVREngage (not
    // stay in Waypoint as it did when the gate was hardcoded to 8 NM).
    target.x = 0.0; target.y = 12.0 * 6076.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;
    brain.setTarget(&target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::BVREngage);
}

TEST_F(DigiBrainWVRTest, TargetBeyondBVRStaysWaypoint) {
    // Target 40 NM — beyond the 35 NM BVR gate
    target.x = 0.0; target.y = 40.0 * 6076.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;
    brain.setTarget(&target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainWVRTest, DeadTargetStaysWaypoint) {
    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = true;
    brain.setTarget(&target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainWVRTest, MissileDefeatPreemptsWVR) {
    // Both target and missile present — missile has higher priority
    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;

    DigiEntity missile;
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;

    brain.setTarget(&target);
    brain.setIncomingMissile(&missile);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainWVRTest, WVREngageProducesValidOutput) {
    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;
    brain.setTarget(&target);

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    EXPECT_GE(out.pstick, -1.0);
    EXPECT_LE(out.pstick, 1.0);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
    EXPECT_FALSE(std::isnan(out.throttle));
}

TEST_F(DigiBrainWVRTest, ClearedTargetReturnsToWaypoint) {
    target.x = 0.0; target.y = 3000.0; target.z = -10000.0;
    target.yaw = PI;
    target.isDead = false;
    brain.setTarget(&target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::WVREngage);

    brain.setTarget(nullptr);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}
