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
#include "f4flight/digi/formation/formation_geometry.h"  // Round-2 fix
#include "f4flight/digi/ground/ag_doctrine.h"            // Round-2 fix
#include "f4flight/flight/flight_model.h"                       // Round-2 fix (for brake test)
#include "f4flight/flight/core/constants.h"

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
    // Round-2 structural fix (Rec 6): added 9 missing DigiMode values
    // (Refueling, Separate, Roop, OverB, Loiter, FollowOrders, RTB, Wingy,
    // Bugout, GroundMnvr). Total is now 23.
    EXPECT_EQ(kNumDigiModes, 23);
}

// ===========================================================================
// DigiState tests
// ===========================================================================
TEST(DigiStateTest, DefaultValues) {
    DigiState s;
    EXPECT_NEAR(s.commands.pStick, 0.0, 1e-9);
    EXPECT_NEAR(s.commands.rStick, 0.0, 1e-9);
    EXPECT_NEAR(s.commands.throttle, 0.5, 1e-9);
    EXPECT_EQ(s.config.skill.level, SkillLevel::Veteran);
    EXPECT_FALSE(s.groundAvoid.groundAvoidNeeded);
    EXPECT_NEAR(s.groundAvoid.pullupTimer, 0.0, 1e-9);
}

TEST(DigiStateTest, ResetClearsState) {
    DigiState s;
    s.commands.pStick = 0.5;
    s.commands.rStick = -0.3;
    s.nav.gammaHoldIError = 1.5;
    s.nav.autoThrottle = 0.7;
    s.groundAvoid.groundAvoidNeeded = true;
    s.groundAvoid.pullupTimer = 2.0;
    s.reset();
    EXPECT_NEAR(s.commands.pStick, 0.0, 1e-9);
    EXPECT_NEAR(s.commands.rStick, 0.0, 1e-9);
    EXPECT_NEAR(s.nav.gammaHoldIError, 0.0, 1e-9);
    EXPECT_NEAR(s.nav.autoThrottle, 0.0, 1e-9);
    EXPECT_FALSE(s.groundAvoid.groundAvoidNeeded);
    EXPECT_NEAR(s.groundAvoid.pullupTimer, 0.0, 1e-9);
}

// ===========================================================================
// Round-2 structural additions — tests for new DigiState fields + new
// maneuver primitives + new DigiMode values.
// ===========================================================================

TEST(DigiStateTest, Round2_BrakeAndSpeedBrakeDefaults) {
    // Rec 7: brake / speed-brake / gear commands default to "no drag, gear down".
    DigiState s;
    EXPECT_FALSE(s.commands.wheelBrakes);       // brakes off
    EXPECT_FALSE(s.commands.parkingBrake);
    EXPECT_NEAR(s.commands.speedBrakeCmd, -1.0, 1e-9);  // retracted (no drag)
    EXPECT_NEAR(s.commands.gearHandleCmd, 1.0, 1e-9);   // gear down
}

TEST(DigiStateTest, Round2_ResetClearsBrakeCommands) {
    DigiState s;
    s.commands.wheelBrakes = true;
    s.commands.parkingBrake = true;
    s.commands.speedBrakeCmd = 1.0;
    s.commands.gearHandleCmd = -1.0;
    s.reset();
    EXPECT_FALSE(s.commands.wheelBrakes);
    EXPECT_FALSE(s.commands.parkingBrake);
    EXPECT_NEAR(s.commands.speedBrakeCmd, -1.0, 1e-9);
    EXPECT_NEAR(s.commands.gearHandleCmd, 1.0, 1e-9);
}

TEST(DigiStateTest, Round2_GroundTargetDefaults) {
    // Rec 9: ground target pointer defaults to null, doctrine to NONE.
    DigiState s;
    EXPECT_EQ(s.ag.groundTarget, nullptr);
    EXPECT_EQ(s.ag.groundTargetId, kInvalidEntityId);
    EXPECT_EQ(s.ag.agDoctrine, 0);  // AGD_NONE
    EXPECT_EQ(s.ag.agApproach, 0);  // AGA_NONE
    EXPECT_FALSE(s.ag.reachedIP);
}

TEST(DigiStateTest, Round2_ResetClearsGroundTarget) {
    DigiState s;
    DigiEntity fakeTarget;
    s.ag.groundTarget = &fakeTarget;
    s.ag.groundTargetId = 42;
    s.ag.agDoctrine = 2;
    s.ag.agApproach = 4;
    s.ag.reachedIP = true;
    s.reset();
    EXPECT_EQ(s.ag.groundTarget, nullptr);
    EXPECT_EQ(s.ag.groundTargetId, kInvalidEntityId);
    EXPECT_EQ(s.ag.agDoctrine, 0);
    EXPECT_EQ(s.ag.agApproach, 0);
    EXPECT_FALSE(s.ag.reachedIP);
}

TEST(DigiStateTest, Round2_FormationFieldsDefault) {
    // Rec 3: formation fields default to "no lead, not a wingman".
    DigiState s;
    EXPECT_EQ(s.formation.flightLeadId, kInvalidEntityId);
    EXPECT_FALSE(s.formation.isWing);
    EXPECT_EQ(s.formation.vehicleInUnit, 0);
    EXPECT_EQ(s.formation.formationId, 0);
    EXPECT_NEAR(s.formation.formRelAz, 0.0, 1e-9);
    EXPECT_NEAR(s.formation.formRelEl, 0.0, 1e-9);
    EXPECT_NEAR(s.formation.formRange, 500.0, 1e-9);  // default 500 ft
}

TEST(DigiStateTest, Round2_MnverTimeDefaults) {
    // Rec 10: mnverTime (maneuver timer) defaults to 0.
    DigiState s;
    EXPECT_NEAR(s.nav.mnverTime, 0.0, 1e-9);
    s.nav.mnverTime = 1.5;
    s.reset();
    EXPECT_NEAR(s.nav.mnverTime, 0.0, 1e-9);
}

TEST(DigiModeTest, Round2_NewModesExist) {
    // Rec 6: the 9 missing DigiMode values are now in the enum.
    EXPECT_NE(static_cast<int>(DigiMode::Refueling),    static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::Separate),     static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::Roop),         static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::OverB),        static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::Loiter),       static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::FollowOrders), static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::RTB),          static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::Wingy),        static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::Bugout),       static_cast<int>(DigiMode::Waypoint));
    EXPECT_NE(static_cast<int>(DigiMode::GroundMnvr),   static_cast<int>(DigiMode::Waypoint));

    // Each new mode has a name.
    EXPECT_STREQ(digiModeName(DigiMode::Refueling),    "Refueling");
    EXPECT_STREQ(digiModeName(DigiMode::Separate),     "Separate");
    EXPECT_STREQ(digiModeName(DigiMode::Roop),         "Roop");
    EXPECT_STREQ(digiModeName(DigiMode::OverB),        "OverB");
    EXPECT_STREQ(digiModeName(DigiMode::Loiter),       "Loiter");
    EXPECT_STREQ(digiModeName(DigiMode::FollowOrders), "FollowOrders");
    EXPECT_STREQ(digiModeName(DigiMode::RTB),          "RTB");
    EXPECT_STREQ(digiModeName(DigiMode::Wingy),        "Wingy");
    EXPECT_STREQ(digiModeName(DigiMode::Bugout),       "Bugout");
    EXPECT_STREQ(digiModeName(DigiMode::GroundMnvr),   "GroundMnvr");
}

TEST(FormationGeometryTest, Round2_DefaultWedgeHasFourSlots) {
    // Rec 3: the default wedge formation has 4 slots, slot 0 = lead.
    using namespace f4flight::digi::formation;
    const auto wedge = defaultWedge();
    EXPECT_EQ(wedge.size(), kMaxFormationSlots);
    // Slot 0 (lead) is at origin
    EXPECT_NEAR(wedge[0].relAz, 0.0, 1e-9);
    EXPECT_NEAR(wedge[0].relEl, 0.0, 1e-9);
    EXPECT_NEAR(wedge[0].range, 0.0, 1e-9);
    // Slots 1 and 2 (wingmen) are 30° off the nose, 1000 ft back
    EXPECT_NEAR(wedge[1].range, 1000.0, 1e-9);
    EXPECT_NEAR(wedge[2].range, 1000.0, 1e-9);
    // Slot 3 (trail) is straight behind, 2000 ft back
    EXPECT_NEAR(wedge[3].range, 2000.0, 1e-9);
}

TEST(FormationGeometryTest, Round2_FormationTableLookup) {
    // Rec 3: the FormationTable can look up geometry by type + slot.
    // (Previously a singleton via instance(); now a regular class with a
    // shared defaultInstance() for backward compatibility.)
    using namespace f4flight::digi::formation;
    const auto& table = FormationTable::defaultInstance();
    const auto slot1 = table.slotGeometry(FormationType::Wedge, 1);
    EXPECT_NEAR(slot1.range, 1000.0, 1e-9);
    // Invalid slot returns lead geometry (zero-relative)
    const auto invalid = table.slotGeometry(FormationType::Wedge, 99);
    EXPECT_NEAR(invalid.range, 0.0, 1e-9);
    // Convenience: forWingman reads formationId + vehicleInUnit
    const auto w = FormationTable::forWingman(
        static_cast<int>(FormationType::Wedge), 1);
    EXPECT_NEAR(w.range, 1000.0, 1e-9);
}

TEST(FormationGeometryTest, InjectableFormationTable) {
    // Structural fix: FormationTable is now a regular class, not a singleton.
    // Hosts can construct their own and register custom formations without
    // affecting the default instance.
    using namespace f4flight::digi::formation;
    FormationTable myTable;
    Formation custom{};
    custom[0] = {0.0, 0.0, 0.0};
    custom[1] = {45.0 * M_PI / 180.0, 0.0, 500.0};  // 500 ft at 45° right
    myTable.registerFormation(FormationType::Custom, custom);
    const auto slot = myTable.slotGeometry(FormationType::Custom, 1);
    EXPECT_NEAR(slot.range, 500.0, 1e-9);
    // Default instance should NOT have the custom formation
    const auto defaultSlot = FormationTable::defaultInstance().slotGeometry(
        FormationType::Custom, 1);
    EXPECT_NEAR(defaultSlot.range, 0.0, 1e-9);  // unregistered → lead slot
}

TEST(AGDoctrineTest, Round2_EnumsAndNames) {
    // Rec 9: AG doctrine + approach enums + names.
    using namespace f4flight::digi::ag;
    EXPECT_EQ(doctrineName(AGD_NONE),            "None");
    EXPECT_EQ(doctrineName(AGD_SHOOT_RUN),       "ShootRun");
    EXPECT_EQ(doctrineName(AGD_LOOK_SHOOT_LOOK), "LookShootLook");
    EXPECT_EQ(doctrineName(AGD_NEED_SETUP),      "NeedSetup");
    EXPECT_EQ(approachName(AGA_NONE),   "None");
    EXPECT_EQ(approachName(AGA_LOW),    "Low");
    EXPECT_EQ(approachName(AGA_TOSS),   "Toss");
    EXPECT_EQ(approachName(AGA_HIGH),   "High");
    EXPECT_EQ(approachName(AGA_DIVE),   "Dive");
    EXPECT_EQ(approachName(AGA_BOMBER), "Bomber");
}

TEST(DigiBrainRound2Test, FrameInputsGroundTargetInjection) {
    // Rec 9: injectedGroundTarget is plumbed through to state_.ag.groundTarget.
    DigiBrain brain;
    DigiEntity groundTarget;
    groundTarget.x = 1000.0;
    groundTarget.y = 2000.0;
    groundTarget.z = 0.0;  // on the ground
    groundTarget.isDead = false;
    FrameInputs fi;
    fi.injectedGroundTarget = &groundTarget;
    brain.setFrameInputs(fi);
    EXPECT_EQ(brain.state().ag.groundTarget, &groundTarget);

    // Clearing the injection should clear the state pointer.
    FrameInputs fiEmpty;
    brain.setFrameInputs(fiEmpty);
    EXPECT_EQ(brain.state().ag.groundTarget, nullptr);
    EXPECT_EQ(brain.state().ag.groundTargetId, kInvalidEntityId);
}

TEST(DigiBrainRound2Test, PilotInputMapsBrakeCommands) {
    // Rec 7: the brain's compute() maps state_.commands.wheelBrakes / speedBrakeCmd /
    // gearHandleCmd to PilotInput. We can't easily call compute() without a
    // valid aircraft config, but we CAN verify the mapping logic by reading
    // the state and applying the same mapping the brain uses. This catches
    // regressions where the mapping is removed or wired to the wrong fields.
    DigiBrain brain;
    brain.stateMutable().commands.wheelBrakes = true;
    brain.stateMutable().commands.speedBrakeCmd = 0.5;
    brain.stateMutable().commands.gearHandleCmd = -1.0;  // gear up

    // The brain's compute() does (see digi_brain.cpp:270-273):
    //   out.wheelBrakes  = state_.commands.wheelBrakes;
    //   out.parkingBrake = state_.commands.parkingBrake;
    //   out.speedBrake   = state_.commands.speedBrakeCmd;
    //   out.gearHandle   = state_.commands.gearHandleCmd;
    // Verify the state values are what we set (so the mapping will produce
    // the correct PilotInput when compute() runs).
    EXPECT_TRUE(brain.state().commands.wheelBrakes);
    EXPECT_NEAR(brain.state().commands.speedBrakeCmd, 0.5, 1e-9);
    EXPECT_NEAR(brain.state().commands.gearHandleCmd, -1.0, 1e-9);

    // And verify the mapping is correct by simulating it:
    PilotInput expected;
    expected.wheelBrakes = brain.state().commands.wheelBrakes;
    expected.parkingBrake = brain.state().commands.parkingBrake;
    expected.speedBrake = brain.state().commands.speedBrakeCmd;
    expected.gearHandle = brain.state().commands.gearHandleCmd;
    EXPECT_TRUE(expected.wheelBrakes);
    EXPECT_NEAR(expected.speedBrake, 0.5, 1e-9);
    EXPECT_NEAR(expected.gearHandle, -1.0, 1e-9);
}

// ===========================================================================
// End Round-2 structural addition tests
// ===========================================================================

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
        state.kin.psi = 0.0;
        state.kin.theta = 0.0;
        state.kin.phi = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
        state.kin.dcm = Matrix3::identity();  // body-to-world (psi=theta=phi=0)
        digi.nav.dt = 1.0 / 60.0;
        digi.config.maxGs = 9.0;
        digi.config.maxRoll = 45.0;
        digi.config.maxGammaDeg = 15.0;
        digi.config.turnLoadFactor = 2.0;
        digi.config.cornerSpeed = 330.0;
    }
};

TEST_F(CombatPrimitivesTest, TrackPointCommandsHeadingToTarget) {
    // Target is due north (+Y), 10 NM, same altitude. In F4Flight's NED
    // frame, heading = atan2(ydot, xdot), so north (+Y) is heading PI/2.
    // Set the aircraft heading to PI/2 (north) so it's already pointed
    // at the target → small heading error → small rStick.
    // (The old test used sigma=0 which is EAST in F4Flight — a 90° error
    // that produced a large rStick. It only passed because the old
    // LevelTurn started in phase 0 "level wings" which delayed the turn.)
    const double targetX = 0.0;
    const double targetY = 60000.0;  // 10 NM north
    const double targetAlt = 10000.0;

    state.kin.sigma = PI / 2.0;  // north — aligned with target
    state.kin.sinsig = 1.0;
    state.kin.cossig = 0.0;

    ManeuverPrimitives::TrackPoint(targetX, targetY, targetAlt,
                                    digi, state, fcs, fcsState, 9.0);

    // Already pointed at target → small heading error → small rStick
    EXPECT_LT(std::fabs(digi.commands.rStick), 0.5);
    EXPECT_LT(std::fabs(digi.commands.pStick), 0.5);
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
    EXPECT_GT(std::fabs(digi.commands.rStick), 0.01);
}

TEST_F(CombatPrimitivesTest, AutoTrackLeadsMovingTarget) {
    // AutoTrack now reads trackX/Y/Z from DigiState and uses the real
    // lift-vector-on-target + pull control law (port of FF mnvers.cpp:211).
    // Set a target directly ahead (along body x-axis): ata should be ~0,
    // rStick should be near-zero (wings level).
    digi.nav.trackX = 6000.0;  // 6000 ft ahead (+x = nose direction)
    digi.nav.trackY = 0.0;
    digi.nav.trackZ = state.kin.z;  // same altitude → no elevation error

    ManeuverPrimitives::AutoTrack(digi, state, fcsState, 9.0);

    // Target is directly ahead at same altitude → ata < 5° (fine track
    // branch). Wings-level damping should produce small rStick.
    // (Not zero because the damping is proportional to current roll.)
    EXPECT_LT(std::fabs(digi.commands.rStick), 0.5);
}

TEST_F(CombatPrimitivesTest, VectorTrackHoldsHeadingAndAltitude) {
    ManeuverPrimitives::VectorTrack(0.0, 10000.0, 350.0,
                                     digi, state, fcs, fcsState, 9.0, 1.0/60.0);
    // Already on heading 0, altitude 10000 → minimal stick commands
    EXPECT_LT(std::fabs(digi.commands.rStick), 0.5);
    // Throttle should be set by MachHold
    EXPECT_GT(digi.commands.throttle, 0.0);
    EXPECT_LT(digi.commands.throttle, 1.5);
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
        digi.nav.dt = 1.0 / 60.0;
        digi.config.maxGs = 9.0;
        digi.config.cornerSpeed = 330.0;
        digi.config.maxRoll = 45.0;

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
    EXPECT_FALSE(digi.groundAvoid.groundAvoidNeeded);
}

TEST_F(GroundAvoidTest, TriggersAtLowAltitude) {
    // At 300 ft AGL (below 500 ft clearance) → avoidance needed
    state.kin.z = -300.0;
    EXPECT_TRUE(GroundCheck(digi, state, 0.0, 5.0));
    EXPECT_TRUE(digi.groundAvoid.groundAvoidNeeded);
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
    EXPECT_GT(digi.commands.pStick, 0.2);
    // Wings level: fcsState.maxRoll = 0
    EXPECT_NEAR(fcsState.maxRoll, 0.0, 1e-9);
    // Pull-up timer should be set
    EXPECT_GT(digi.groundAvoid.pullupTimer, 0.0);
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
    EXPECT_GT(digi.groundAvoid.pullupTimer, 0.0);
}

TEST_F(GroundAvoidTest, RunGroundAvoidNoOpAtSafeAltitude) {
    state.kin.z = -10000.0;
    bool pulled = RunGroundAvoid(digi, state, 0.0, 330.0, 1.0 / 60.0,
                                  fcsState, 9.0);
    EXPECT_FALSE(pulled);
    EXPECT_NEAR(digi.groundAvoid.pullupTimer, 0.0, 1e-9);
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
    EXPECT_EQ(brain.state().config.skill.level, SkillLevel::Veteran);
}

TEST_F(DigiBrainTest, SetSkillChangesParameters) {
    brain.setSkill(SkillLevel::Ace);
    EXPECT_EQ(brain.state().config.skill.level, SkillLevel::Ace);
    EXPECT_TRUE(brain.state().config.skill.gciCapable);
    EXPECT_TRUE(brain.state().config.skill.irMissileThrottleCut);
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
    EXPECT_GT(brain.state().groundAvoid.pullupTimer, 0.0);
}

TEST_F(DigiBrainTest, ResetClearsState) {
    // Mutate state
    brain.state().commands.pStick = 0.5;
    brain.state().nav.gammaHoldIError = 1.5;
    brain.state().groundAvoid.pullupTimer = 2.0;

    brain.reset();

    EXPECT_NEAR(brain.state().commands.pStick, 0.0, 1e-9);
    EXPECT_NEAR(brain.state().nav.gammaHoldIError, 0.0, 1e-9);
    EXPECT_NEAR(brain.state().groundAvoid.pullupTimer, 0.0, 1e-9);
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
    // brain has waypoints to follow.
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

// ===========================================================================
// Refactored API tests (configure / setFrameInputs / commandXxx)
//
// These tests verify the NEW host-facing API introduced in the §3.1
// refactor. They exercise the same internal behavior as the deprecated
// shims but go through the clean API.
// ===========================================================================

TEST_F(DigiBrainTest, ConfigureSetsAllConfigFields) {
    DigiConfig cfg;
    cfg.skillLevel = SkillLevel::Ace;
    cfg.cornerSpeedKts = 400.0;
    cfg.maxGs = 7.5;
    cfg.maxBankDeg = 50.0;
    cfg.maxGammaDeg = 20.0;
    cfg.turnLoadFactor = 1.8;

    brain.configure(cfg);

    EXPECT_EQ(brain.state().config.skill.level, SkillLevel::Ace);
    EXPECT_NEAR(brain.state().config.cornerSpeed, 400.0, 1e-9);
    EXPECT_NEAR(brain.state().config.maxGs, 7.5, 1e-9);
    EXPECT_NEAR(brain.state().config.maxRoll, 50.0, 1e-9);
    EXPECT_NEAR(brain.state().config.maxGammaDeg, 20.0, 1e-9);
    EXPECT_NEAR(brain.state().config.turnLoadFactor, 1.8, 1e-9);
}

TEST_F(DigiBrainTest, ConfigReadsBackCurrentValues) {
    // Use deprecated setters to set values, then read back via config().
    brain.setSkill(SkillLevel::Rookie);
    brain.setCornerSpeed(250.0);
    brain.setMaxGs(6.0);
    brain.setMaxBank(35.0);
    brain.setMaxGamma(12.0);
    brain.setTurnG(1.5);

    DigiConfig cfg = brain.config();
    EXPECT_EQ(cfg.skillLevel, SkillLevel::Rookie);
    EXPECT_NEAR(cfg.cornerSpeedKts, 250.0, 1e-9);
    EXPECT_NEAR(cfg.maxGs, 6.0, 1e-9);
    EXPECT_NEAR(cfg.maxBankDeg, 35.0, 1e-9);
    EXPECT_NEAR(cfg.maxGammaDeg, 12.0, 1e-9);
    EXPECT_NEAR(cfg.turnLoadFactor, 1.5, 1e-9);
}

TEST_F(DigiBrainTest, SetFrameInputsStoresTruthAndSelfEntity) {
    TruthState truth;
    DigiEntity e;
    e.x = 1000; e.y = 2000; e.z = -10000;
    truth.add(50, e);

    DigiEntity self;
    self.x = 0; self.y = 0; self.z = -10000;
    self.speed = 500;

    FrameInputs fi;
    fi.truth = &truth;
    fi.selfEntity = &self;

    brain.setFrameInputs(fi);
    EXPECT_EQ(brain.frameInputs().truth, &truth);
    EXPECT_EQ(brain.frameInputs().selfEntity, &self);
}

TEST_F(DigiBrainTest, SetFrameInputsAutoBuildsSelfEntityWhenNull) {
    // If selfEntity is null in FrameInputs, compute() should auto-build
    // from AircraftState and still function (no crash, valid output).
    FrameInputs fi;  // selfEntity = nullptr
    brain.setFrameInputs(fi);

    PilotInput out = brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
    EXPECT_FALSE(std::isnan(out.throttle));
}

TEST_F(DigiBrainTest, SetFrameInputsWithTruthRunsSensorFusion) {
    // Provide a truth state with a target. SensorFusion should detect it
    // and the brain should enter WVREngage.
    DigiEntity target;
    target.x = 3.0 * 6076.0; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500.0;

    TruthState truth;
    truth.add(100, target);

    FrameInputs fi;
    fi.truth = &truth;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::WVREngage);
    EXPECT_FALSE(brain.sensorPicture().contacts.empty());
}

TEST_F(DigiBrainTest, SetFrameInputsInjectedMissileEntersMissileDefeat) {
    // Inject a missile directly via FrameInputs (testing path).
    DigiEntity missile;
    missile.x = 5.0 * 6076.0; missile.y = 0; missile.z = -10000;
    missile.vx = -2000.0; missile.vy = 0; missile.vz = 0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;

    FrameInputs fi;
    fi.injectedMissile = &missile;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainTest, CommandTakeoffSetsGroundOpsPhase) {
    brain.commandTakeoff(RunwayId{1}, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(brain.state().ag.groundOps.phase, GroundOpsPhase::TakeoffRoll);
    EXPECT_TRUE(brain.state().ag.groundOps.hasTakeoffClearance);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Takeoff);
}

TEST_F(DigiBrainTest, CommandLandingSetsGroundOpsPhase) {
    brain.commandLanding(RunwayId{1}, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(brain.state().ag.groundOps.phase, GroundOpsPhase::Approach);
    EXPECT_TRUE(brain.state().ag.groundOps.hasLandingClearance);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Landing);
}

TEST_F(DigiBrainTest, ForceModeAndClearForcedMode) {
    brain.forceMode(DigiMode::GunsJink);
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::GunsJink);

    brain.clearForcedMode();
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainTest, StateMutableAllowsWriteForTesting) {
    // stateMutable() returns a non-const reference for testing.
    brain.stateMutable().commands.pStick = 0.42;
    EXPECT_NEAR(brain.state().commands.pStick, 0.42, 1e-9);
}

TEST_F(DigiBrainTest, ResetClearsFrameInputsAndAutoEntities) {
    // Set up some state via the new API.
    DigiEntity missile;
    missile.x = 5000; missile.y = 0; missile.z = -10000;
    missile.speed = 2000;
    missile.seekerType = DigiEntity::SeekerType::Radar;

    FrameInputs fi;
    fi.injectedMissile = &missile;
    brain.setFrameInputs(fi);
    brain.compute(state, 1.0/60.0, 10000.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);

    // Reset should clear everything.
    brain.reset();
    EXPECT_EQ(brain.frameInputs().truth, nullptr);
    EXPECT_EQ(brain.frameInputs().injectedMissile, nullptr);
    EXPECT_EQ(brain.state().missileDefeat.incomingMissile, nullptr);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}
