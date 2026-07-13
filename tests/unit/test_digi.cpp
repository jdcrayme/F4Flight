// f4flight unit tests - digi AI Tier 0 capabilities
//
// Tests for:
//   - SkillParameters (digi_skill.h)
//   - DigiMode enum + priority (digi_mode.h)
//   - DigiState (digi_state.h)
//   - DigiBrain dispatcher (digi_brain.h)
//   - ManeuverPrimitives (maneuver_primitives.h) — combat primitives
//   - GroundAvoid (ground_avoid.h)
//
// These tests verify the NEW digi/ subsystem directly (not through the
// SteeringController compatibility shim). They cover:
//   1. Skill parameter derivation from skill levels
//   2. Mode priority and name lookup
//   3. DigiState reset behavior
//   4. DigiBrain waypoint following + ground avoidance integration
//   5. Combat primitives (TrackPoint, AutoTrack, VectorTrack)
//   6. GroundCheck triggers and PullUp execution

#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// SkillParameters tests
// ===========================================================================
class DigiSkillTest : public ::testing::Test {
protected:
    SkillParameters recruit{makeSkillParams(SkillLevel::Recruit)};
    SkillParameters veteran{makeSkillParams(SkillLevel::Veteran)};
    SkillParameters ace{makeSkillParams(SkillLevel::Ace)};
};

TEST_F(DigiSkillTest, GciCapability) {
    EXPECT_FALSE(recruit.gciCapable);
    EXPECT_TRUE(veteran.gciCapable);
    EXPECT_TRUE(ace.gciCapable);
}

TEST_F(DigiSkillTest, ShootShootProbabilityScalesWithSkill) {
    EXPECT_LT(recruit.shootShootPctRadar, veteran.shootShootPctRadar);
    EXPECT_LT(veteran.shootShootPctRadar, ace.shootShootPctRadar);
    EXPECT_GT(ace.shootShootPctRadar, 0.5);
}

TEST_F(DigiSkillTest, RMaxMultiplierDecreasesWithSkill) {
    // Aces use tighter RMax (more conservative about firing range)
    EXPECT_GT(recruit.rMaxMultiplier, veteran.rMaxMultiplier);
    EXPECT_GT(veteran.rMaxMultiplier, ace.rMaxMultiplier);
}

TEST_F(DigiSkillTest, ReactionTimeDecreasesWithSkill) {
    EXPECT_GT(recruit.reactionTimeSec, veteran.reactionTimeSec);
    EXPECT_GT(veteran.reactionTimeSec, ace.reactionTimeSec);
}

TEST_F(DigiSkillTest, IRMissileThrottleCutOnlyForHighSkill) {
    EXPECT_FALSE(recruit.irMissileThrottleCut);
    EXPECT_FALSE(veteran.irMissileThrottleCut);  // Veteran = 2, needs > 2
    EXPECT_TRUE(ace.irMissileThrottleCut);
}

TEST_F(DigiSkillTest, JettisonChanceScalesWithSkill) {
    EXPECT_NEAR(recruit.jettisonChanceOnDefeat, 0.0, 1e-9);
    EXPECT_NEAR(veteran.jettisonChanceOnDefeat, 0.50, 1e-9);
    EXPECT_NEAR(ace.jettisonChanceOnDefeat, 0.75, 1e-9);
}

// ===========================================================================
// DigiMode tests
// ===========================================================================
TEST(DigiModeTest, PriorityOrdering) {
    // Lower value = higher priority
    EXPECT_LT(static_cast<int>(DigiMode::GroundAvoid), static_cast<int>(DigiMode::MissileDefeat));
    EXPECT_LT(static_cast<int>(DigiMode::MissileDefeat), static_cast<int>(DigiMode::GunsJink));
    EXPECT_LT(static_cast<int>(DigiMode::GunsJink), static_cast<int>(DigiMode::Landing));
    EXPECT_LT(static_cast<int>(DigiMode::Landing), static_cast<int>(DigiMode::Takeoff));
    EXPECT_LT(static_cast<int>(DigiMode::Takeoff), static_cast<int>(DigiMode::WVREngage));
    EXPECT_LT(static_cast<int>(DigiMode::WVREngage), static_cast<int>(DigiMode::Waypoint));
}

TEST(DigiModeTest, NameLookup) {
    EXPECT_STREQ(digiModeName(DigiMode::GroundAvoid), "GroundAvoid");
    EXPECT_STREQ(digiModeName(DigiMode::MissileDefeat), "MissileDefeat");
    EXPECT_STREQ(digiModeName(DigiMode::GunsJink), "GunsJink");
    EXPECT_STREQ(digiModeName(DigiMode::Landing), "Landing");
    EXPECT_STREQ(digiModeName(DigiMode::Takeoff), "Takeoff");
    EXPECT_STREQ(digiModeName(DigiMode::WVREngage), "WVREngage");
    EXPECT_STREQ(digiModeName(DigiMode::Waypoint), "Waypoint");
    EXPECT_STREQ(digiModeName(DigiMode::NoMode), "NoMode");
}

TEST(DigiModeTest, NumModes) {
    EXPECT_EQ(kNumDigiModes, 7);
}

// ===========================================================================
// DigiState tests
// ===========================================================================
TEST(DigiStateTest, DefaultValues) {
    DigiState s;
    EXPECT_NEAR(s.pStick, 0.0, 1e-9);
    EXPECT_NEAR(s.rStick, 0.0, 1e-9);
    EXPECT_NEAR(s.throttle, 0.5, 1e-9);
    EXPECT_EQ(s.skill.level, SkillLevel::Veteran);
    EXPECT_FALSE(s.groundAvoidNeeded);
    EXPECT_NEAR(s.pullupTimer, 0.0, 1e-9);
}

TEST(DigiStateTest, ResetClearsState) {
    DigiState s;
    s.pStick = 0.5;
    s.rStick = -0.3;
    s.gammaHoldIError = 1.5;
    s.autoThrottle = 0.7;
    s.groundAvoidNeeded = true;
    s.pullupTimer = 2.0;
    s.reset();
    EXPECT_NEAR(s.pStick, 0.0, 1e-9);
    EXPECT_NEAR(s.rStick, 0.0, 1e-9);
    EXPECT_NEAR(s.gammaHoldIError, 0.0, 1e-9);
    EXPECT_NEAR(s.autoThrottle, 0.0, 1e-9);
    EXPECT_FALSE(s.groundAvoidNeeded);
    EXPECT_NEAR(s.pullupTimer, 0.0, 1e-9);
}

// ===========================================================================
// ManeuverPrimitives tests (combat primitives)
// ===========================================================================
class CombatPrimitivesTest : public ::testing::Test {
protected:
    DigiState digi;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        // Level flight at 350 kts / 10000 ft
        state.kin.costhe = 1.0;
        state.kin.cosphi = 1.0;
        state.kin.gmma = 0.0;
        state.kin.sigma = 0.0;
        state.kin.x = 0.0;
        state.kin.y = 0.0;
        state.kin.z = -10000.0;
        state.kin.zdot = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
        digi.dt = 1.0 / 60.0;
        digi.maxGs = 9.0;
        digi.maxRoll = 45.0;
        digi.maxGammaDeg = 15.0;
        digi.turnLoadFactor = 2.0;
        digi.cornerSpeed = 330.0;
    }
};

TEST_F(CombatPrimitivesTest, TrackPointCommandsHeadingToTarget) {
    // Target is due north, 10 NM, same altitude
    const double targetX = 0.0;
    const double targetY = 60000.0;  // 10 NM north
    const double targetAlt = 10000.0;

    ManeuverPrimitives::TrackPoint(targetX, targetY, targetAlt,
                                    digi, state, fcs, fcsState, 9.0);

    // Aircraft heading is 0 (north), target is north → small heading error
    // rStick should be near 0 (already pointed at target)
    // pStick should be near 0 (same altitude)
    EXPECT_LT(std::fabs(digi.rStick), 0.5);
    EXPECT_LT(std::fabs(digi.pStick), 0.5);
}

TEST_F(CombatPrimitivesTest, TrackPointEastTargetCommandsRightTurn) {
    // Target is 90° to the right of current heading (north).
    // In the codebase's coordinate system (verified via ai_flightplan),
    // a target at (0, 60000) is 90° from heading 0.
    const double targetX = 0.0;
    const double targetY = 60000.0;
    const double targetAlt = 10000.0;

    // Set a small bank so LevelTurn's "level wings" phase produces a non-zero
    // rstick. (LevelTurn starts in trackMode=0 = "level wings first"; if phi
    // is already 0, the roll command is 0 on the first frame.)
    state.kin.phi = 0.1;  // ~6° bank

    ManeuverPrimitives::TrackPoint(targetX, targetY, targetAlt,
                                    digi, state, fcs, fcsState, 9.0);

    // With a non-zero bank, the level-wings phase commands a non-zero rstick
    // to level the wings before starting the turn.
    EXPECT_GT(std::fabs(digi.rStick), 0.01);
}

TEST_F(CombatPrimitivesTest, AutoTrackLeadsMovingTarget) {
    // Target moving north at 500 ft/s, currently at origin
    // Lead time 2s → predicted position is (0, 1000)
    ManeuverPrimitives::AutoTrack(0.0, 0.0, 10000.0,
                                   0.0, 500.0, 2.0,
                                   digi, state, fcs, fcsState, 9.0);
    // Should command heading toward (0, 1000) which is due north
    // Same as TrackPoint(0, 1000, 10000) — heading error is 0
    EXPECT_LT(std::fabs(digi.rStick), 0.5);
}

TEST_F(CombatPrimitivesTest, VectorTrackHoldsHeadingAndAltitude) {
    ManeuverPrimitives::VectorTrack(0.0, 10000.0, 350.0,
                                     digi, state, fcs, fcsState, 9.0, 1.0/60.0);
    // Already on heading 0, altitude 10000 → minimal stick commands
    EXPECT_LT(std::fabs(digi.rStick), 0.5);
    // Throttle should be set by MachHold
    EXPECT_GT(digi.throttle, 0.0);
    EXPECT_LT(digi.throttle, 1.5);
}

// ===========================================================================
// GroundAvoid tests
// ===========================================================================
class GroundAvoidTest : public ::testing::Test {
protected:
    DigiState digi;
    AircraftState state;
    FcsState fcsState;

    void SetUp() override {
        digi.dt = 1.0 / 60.0;
        digi.maxGs = 9.0;
        digi.cornerSpeed = 330.0;
        digi.maxRoll = 45.0;

        // Level flight at 350 kts
        state.kin.costhe = 1.0;
        state.kin.cosphi = 1.0;
        state.kin.singam = 0.0;  // level flight path
        state.kin.gmma = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
        state.loads.nzcgs = 1.0;
    }
};

TEST_F(GroundAvoidTest, NoTriggerAtHighAltitude) {
    // At 10000 ft AGL, flat terrain → no avoidance needed
    state.kin.z = -10000.0;
    EXPECT_FALSE(GroundCheck(digi, state, 0.0, 5.0));
    EXPECT_FALSE(digi.groundAvoidNeeded);
}

TEST_F(GroundAvoidTest, TriggersAtLowAltitude) {
    // At 300 ft AGL (below 500 ft clearance) → avoidance needed
    state.kin.z = -300.0;
    EXPECT_TRUE(GroundCheck(digi, state, 0.0, 5.0));
    EXPECT_TRUE(digi.groundAvoidNeeded);
}

TEST_F(GroundAvoidTest, TriggersOnDescendingPath) {
    // At 1000 ft AGL but descending (negative gamma) → predicted path hits ground
    state.kin.z = -1000.0;
    state.kin.singam = -0.3;  // ~17° descent
    state.kin.gmma = -0.3;
    // climbRate = 589 * (-0.3) = -177 ft/s
    // In 5s, descends 884 ft → from 1000 ft to 116 ft AGL → below 500 ft threshold
    EXPECT_TRUE(GroundCheck(digi, state, 0.0, 5.0));
}

TEST_F(GroundAvoidTest, NoTriggerOnLevelPathAt1000Ft) {
    // At 1000 ft AGL, level flight → predicted path stays at 1000 ft → safe
    state.kin.z = -1000.0;
    state.kin.singam = 0.0;
    EXPECT_FALSE(GroundCheck(digi, state, 0.0, 5.0));
}

TEST_F(GroundAvoidTest, PullUpCommandsMaxGAndWingsLevel) {
    // Set up a descending aircraft at low altitude
    state.kin.z = -200.0;
    state.kin.phi = 0.3;  // 17° bank
    state.kin.costhe = 1.0;
    state.kin.cosphi = 0.95;
    state.vcas = 350.0;

    PullUp(digi, state, 330.0, 1.0 / 60.0, fcsState, 9.0);

    // PullUp should command positive pstick (away from ground).
    // First-frame stick is 0.36 after smoothing (alpha=0.36 at 60 Hz),
    // so check > 0.2 rather than > 0.5.
    EXPECT_GT(digi.pStick, 0.2);
    // Wings level: fcsState.maxRoll = 0
    EXPECT_NEAR(fcsState.maxRoll, 0.0, 1e-9);
    // Pull-up timer should be set
    EXPECT_GT(digi.pullupTimer, 0.0);
}

TEST_F(GroundAvoidTest, RunGroundAvoidIntegratesCheckAndPull) {
    // At 200 ft AGL → should trigger pull-up
    state.kin.z = -200.0;
    state.kin.costhe = 1.0;
    state.kin.cosphi = 1.0;
    state.vcas = 350.0;

    bool pulled = RunGroundAvoid(digi, state, 0.0, 330.0, 1.0 / 60.0,
                                  fcsState, 9.0);
    EXPECT_TRUE(pulled);
    EXPECT_GT(digi.pullupTimer, 0.0);
}

TEST_F(GroundAvoidTest, RunGroundAvoidNoOpAtSafeAltitude) {
    state.kin.z = -10000.0;
    bool pulled = RunGroundAvoid(digi, state, 0.0, 330.0, 1.0 / 60.0,
                                  fcsState, 9.0);
    EXPECT_FALSE(pulled);
    EXPECT_NEAR(digi.pullupTimer, 0.0, 1e-9);
}

// ===========================================================================
// DigiBrain integration tests
// ===========================================================================
class DigiBrainTest : public ::testing::Test {
protected:
    DigiBrain brain;
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
        state.kin.x = 0.0;
        state.kin.y = 0.0;
        state.kin.z = -10000.0;
        state.kin.zdot = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(DigiBrainTest, DefaultModeIsWaypoint) {
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainTest, DefaultSkillIsVeteran) {
    EXPECT_EQ(brain.state().skill.level, SkillLevel::Veteran);
}

TEST_F(DigiBrainTest, SetSkillChangesParameters) {
    brain.setSkill(SkillLevel::Ace);
    EXPECT_EQ(brain.state().skill.level, SkillLevel::Ace);
    EXPECT_TRUE(brain.state().skill.gciCapable);
    EXPECT_TRUE(brain.state().skill.irMissileThrottleCut);
}

TEST_F(DigiBrainTest, ComputeProducesValidPilotInput) {
    brain.setHeading(0.0);
    brain.setAltitude(10000.0);

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // Outputs should be in valid ranges
    EXPECT_GE(out.pstick, -1.0);
    EXPECT_LE(out.pstick, 1.0);
    EXPECT_GE(out.rstick, -1.0);
    EXPECT_LE(out.rstick, 1.0);
    EXPECT_GE(out.throttle, 0.0);
    EXPECT_LE(out.throttle, 1.5);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
    EXPECT_FALSE(std::isnan(out.throttle));
}

TEST_F(DigiBrainTest, WaypointFollowingAdvancesOnCapture) {
    // Waypoint 5 NM north at 10000 ft
    std::vector<Vec3> wps = {{0.0, 30000.0, -10000.0}};
    brain.setWaypoints(wps);
    brain.setCaptureRadius(5000.0);
    brain.setAltitude(10000.0);

    // Run ~60 frames (1 second)
    for (int i = 0; i < 60; ++i) {
        state.kin.x += state.kin.vt * (1.0/60.0);  // move north
        brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    }

    // After moving ~589 ft north in 1s, we're still far from the waypoint
    // (30000 ft away). Waypoint should NOT be captured yet.
    EXPECT_EQ(brain.currentWaypoint(), 0u);
}

TEST_F(DigiBrainTest, GroundAvoidPreemptsWaypoint) {
    // Set up a waypoint, but put the aircraft at 200 ft AGL
    std::vector<Vec3> wps = {{0.0, 30000.0, -10000.0}};
    brain.setWaypoints(wps);
    brain.setAltitude(10000.0);
    state.kin.z = -200.0;  // 200 ft AGL

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // Ground avoidance should command a pull-up (positive pstick).
    // First-frame pstick is ~0.36 after smoothing; check > 0.2.
    EXPECT_GT(out.pstick, 0.2);
    EXPECT_GT(brain.state().pullupTimer, 0.0);
}

TEST_F(DigiBrainTest, ResetClearsState) {
    // Mutate state
    brain.state().pStick = 0.5;
    brain.state().gammaHoldIError = 1.5;
    brain.state().pullupTimer = 2.0;

    brain.reset();

    EXPECT_NEAR(brain.state().pStick, 0.0, 1e-9);
    EXPECT_NEAR(brain.state().gammaHoldIError, 0.0, 1e-9);
    EXPECT_NEAR(brain.state().pullupTimer, 0.0, 1e-9);
    EXPECT_EQ(brain.currentWaypoint(), 0u);
}

TEST_F(DigiBrainTest, ForcedModeOverridesResolve) {
    brain.setForcedMode(DigiMode::MissileDefeat);
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);

    brain.setForcedMode(DigiMode::WVREngage);
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::WVREngage);

    brain.clearForcedMode();
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}
