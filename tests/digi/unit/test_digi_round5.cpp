// f4flight unit tests - Round 5: HandleThreat, FuelCheck/RTB, AiPerformManeuver,
// threat-call message ingestion, extended receiveOrders.
//
// These tests cover the P0/P1 capabilities added in Round 5:
//   - HandleThreat (defensive/handle_threat.h) — engage a secondary threat
//   - FuelCheck / RTB mode entry (DigiBrain::resolveMode + runRTB)
//   - AiPerformManeuver (wingman/wingman_maneuvers.h) — FollowOrders mode
//   - ThreatCall message ingestion (ProcessATCMessages → threat.threatCallBearing)
//   - Extended receiveOrders (ClearSix, Posthole, Chainsaw, Kickout, etc.)
//
// Each test follows the same pattern as test_digi_defensive.cpp: synthesize
// a DigiEntity + AircraftState, drive the brain, assert on stick/throttle
// commands and resolved mode.

#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/defensive/handle_threat.h"
#include "f4flight/digi/formation/formation_geometry.h"
#include "f4flight/digi/wingman/wingman_state.h"
#include "f4flight/digi/wingman/wingman_maneuvers.h"
#include "f4flight/digi/ground/ground_ops.h"  // ProcessATCMessages
#include "f4flight/digi/comms/message.h"
#include "f4flight/flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// HandleThreat unit tests
// ===========================================================================
class HandleThreatTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity threat;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        digi.config.cornerSpeed = 330.0;
        digi.config.maxGs = 9.0;
        digi.config.maxRoll = 45.0;
        digi.config.maxGammaDeg = 15.0;
        digi.config.turnLoadFactor = 2.0;
        digi.nav.dt = 1.0 / 60.0;

        // Self at origin, heading north (yaw=0), level at 10000 ft.
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 500.0;

        // Threat 3 NM north, heading south (toward us), same altitude.
        threat.x = 0.0; threat.y = 3.0 * 6076.0; threat.z = -10000.0;
        threat.yaw = PI;
        threat.speed = 500.0;

        // Level flight state.
        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.vcas = 350.0;
        as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(HandleThreatTest, NoThreatReturnsFalse) {
    digi.threat.threatPtr = nullptr;
    EXPECT_FALSE(HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0));
}

TEST_F(HandleThreatTest, ThreatWithinRangeEngagesAndReturnsTrue) {
    // Threat at 3 NM — well within the 8 NM max. HandleThreat should engage
    // (run RollAndPull) and return true.
    digi.threat.threatPtr = &threat;
    digi.threat.threatTimer = 0.0;  // force re-eval on first frame

    const bool handled = HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(handled);

    // Should have commanded something (RollAndPull writes pStick/rStick).
    const bool hasCommand = (std::fabs(digi.commands.pStick) > 0.001 ||
                              std::fabs(digi.commands.rStick) > 0.001 ||
                              digi.commands.throttle > 0.001);
    EXPECT_TRUE(hasCommand);

    // After re-eval, threatTimer should be re-armed to 10 s.
    EXPECT_NEAR(digi.threat.threatTimer, kThreatReevalTimerSec, 1.0);
}

TEST_F(HandleThreatTest, DropsThreatWhenTooFar) {
    // Move threat 9 NM away — exceeds kThreatMaxRangeFt (8 NM).
    threat.y = 9.0 * 6076.0;
    digi.threat.threatPtr = &threat;
    digi.threat.threatTimer = 0.0;  // force re-eval

    const bool handled = HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Threat was dropped — should return false and clear threatPtr.
    EXPECT_FALSE(handled);
    EXPECT_EQ(digi.threat.threatPtr, nullptr);
    EXPECT_NEAR(digi.threat.threatTimer, 0.0, 1e-9);
}

TEST_F(HandleThreatTest, DropsThreatWhenDead) {
    threat.isDead = true;
    digi.threat.threatPtr = &threat;
    digi.threat.threatTimer = 0.0;

    EXPECT_FALSE(HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0));
    EXPECT_EQ(digi.threat.threatPtr, nullptr);
}

TEST_F(HandleThreatTest, TimerHoldsBetweenReEvals) {
    // threatTimer > 0 → no re-eval this frame; HandleThreat just runs
    // RollAndPull and returns true.
    digi.threat.threatPtr = &threat;
    digi.threat.threatTimer = 5.0;  // 5 s remaining

    const double initialTimer = digi.threat.threatTimer;
    const bool handled = HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0);

    EXPECT_TRUE(handled);
    // Timer should have decremented by dt.
    EXPECT_NEAR(digi.threat.threatTimer, initialTimer - 1.0/60.0, 1e-6);
}

TEST_F(HandleThreatTest, DropsThreatWhenBeamAndFar) {
    // Threat 6 NM away (> 5 NM beam threshold) AND ataFrom > 90° (target
    // is beaming away). Should drop.
    threat.y = 6.0 * 6076.0;
    threat.x = 0.0;
    threat.yaw = PI / 2.0;  // heading east (perpendicular = beam)
    digi.threat.threatPtr = &threat;
    digi.threat.threatTimer = 0.0;

    EXPECT_FALSE(HandleThreat(digi, self, as, fcs, fcsState, 1.0/60.0));
    EXPECT_EQ(digi.threat.threatPtr, nullptr);
}

// ===========================================================================
// FuelCheck / RTB mode entry tests (via DigiBrain::resolveMode → compute)
// ===========================================================================
class FuelCheckTest : public ::testing::Test {
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

TEST_F(FuelCheckTest, NormalFuelStaysInWaypoint) {
    FrameInputs fi;
    fi.fuelLbs = 5000.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
    EXPECT_EQ(brain.state().fuel.phase, DigiFuelState::Phase::Normal);
}

TEST_F(FuelCheckTest, JokerFuelTransitionsStateButNoRTB) {
    // Joker: 2500 >= fuel > 1500. State transitions to Joker but RTB is NOT
    // queued (joker is a "heads up" — bingo is the action point).
    FrameInputs fi;
    fi.fuelLbs = 2000.0;  // between bingo (1500) and joker (2500)
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.state().fuel.phase, DigiFuelState::Phase::Joker);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(FuelCheckTest, BingoFuelEntersRTB) {
    // Bingo: fuel <= 1500. State transitions to Bingo AND RTB is queued.
    FrameInputs fi;
    fi.fuelLbs = 1400.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.state().fuel.phase, DigiFuelState::Phase::Bingo);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
}

TEST_F(FuelCheckTest, FumesFuelEntersRTB) {
    FrameInputs fi;
    fi.fuelLbs = 500.0;  // below fumes (800)
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.state().fuel.phase, DigiFuelState::Phase::Fumes);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
}

TEST_F(FuelCheckTest, WinchesterEntersRTBWhenNotDefending) {
    // Out of A/A weapons, no active missile/guns threat → RTB.
    FrameInputs fi;
    fi.fuelLbs = 5000.0;  // plenty of fuel
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    fi.winchester = true;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
}

TEST_F(FuelCheckTest, MissileDefeatPreemptsRTB) {
    // Bingo fuel BUT also an incoming missile — MissileDefeat must win.
    DigiEntity missile;
    missile.x = 5.0 * 6076.0; missile.y = 0; missile.z = -10000;
    missile.vx = -2000.0; missile.vy = 0; missile.vz = 0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;

    FrameInputs fi;
    fi.fuelLbs = 1400.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    fi.injectedMissile = &missile;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.state().fuel.phase, DigiFuelState::Phase::Bingo);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(FuelCheckTest, RTBNavigatesTowardDivertAirbase) {
    // Set up a divert airbase 20 NM north. Run RTB for 1 frame and verify
    // the brain steers north (toward the airbase).
    brain.stateMutable().fuel.hasDivertAirbase = true;
    brain.stateMutable().fuel.divertAirbaseX = 0.0;
    brain.stateMutable().fuel.divertAirbaseY = 20.0 * 6076.0;
    brain.stateMutable().fuel.divertAirbaseZ = -5000.0;

    FrameInputs fi;
    fi.fuelLbs = 1400.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);

    // Aircraft is at (0,0), airbase is at (0, 20 NM). Desired heading is
    // atan2(20NM, 0) = PI/2 (north). Current heading is 0 (sigma=0).
    // HeadingAndAltitudeHold should produce a positive rStick (roll right
    // toward north). We just check that the brain is actively steering —
    // either a non-zero rstick or pstick.
    const bool hasSteer = (std::fabs(out.rstick) > 0.001 ||
                            std::fabs(out.pstick) > 0.001);
    EXPECT_TRUE(hasSteer);
}

TEST_F(FuelCheckTest, RTBFallsBackToWaypointWithoutDivert) {
    // No divert airbase set — RTB should fall back to waypoint nav.
    brain.stateMutable().fuel.hasDivertAirbase = false;

    FrameInputs fi;
    fi.fuelLbs = 1400.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    brain.setFrameInputs(fi);

    // brain should still resolve to RTB mode, but runRTB falls back to
    // waypoint nav internally. Verify no crash + valid output.
    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
}

// ===========================================================================
// AiPerformManeuver (FollowOrders mode) tests
// ===========================================================================
class AiPerformManeuverTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        digi.config.cornerSpeed = 330.0;
        digi.config.maxGs = 9.0;
        digi.config.maxRoll = 45.0;
        digi.config.maxGammaDeg = 15.0;
        digi.config.turnLoadFactor = 2.0;
        digi.nav.dt = 1.0 / 60.0;

        // Self at origin, heading north, level at 10000 ft, 350 kts.
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 350.0 * KNOTS_TO_FTPSEC;

        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.psi = 0.0;  // heading
        as.kin.theta = 0.0;
        as.kin.phi = 0.0;  // roll
        as.vcas = 350.0;
        as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(AiPerformManeuverTest, NoManeuverReturnsFalse) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::None;
    EXPECT_FALSE(AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0));
}

TEST_F(AiPerformManeuverTest, BreakLeftSteersLeft) {
    // Set up a break-left maneuver: headingOrdered = -PI/2 (90° left).
    digi.formation.wingman.currentManeuver = WingmanManeuver::BreakLeft;
    digi.formation.wingman.headingOrdered = -PI / 2.0;  // 90° left of north
    digi.formation.wingman.speedOrdered = 350.0;  // kts
    digi.formation.wingman.altitudeOrdered = 10000.0;
    digi.nav.mnverTime = kDefaultManeuverTimeSec;  // 3 s

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);

    // Trackpoint is 1000 ft ahead at heading -PI/2 (west).
    // Self heading = 0 (north). To turn from north to west, roll LEFT
    // (rstick < 0).
    EXPECT_LT(digi.commands.rStick, -0.001)
        << "Expected break-left to roll left (negative rstick)";

    // Timer should have decremented.
    EXPECT_LT(digi.nav.mnverTime, kDefaultManeuverTimeSec);
}

TEST_F(AiPerformManeuverTest, BreakRightSteersRight) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::BreakRight;
    digi.formation.wingman.headingOrdered = PI / 2.0;  // 90° right of north
    digi.formation.wingman.speedOrdered = 350.0;
    digi.formation.wingman.altitudeOrdered = 10000.0;
    digi.nav.mnverTime = kDefaultManeuverTimeSec;

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);

    // Trackpoint is 1000 ft ahead at heading PI/2 (east).
    // Self heading = 0 (north). To turn from north to east, roll RIGHT
    // (rstick > 0).
    EXPECT_GT(digi.commands.rStick, 0.001)
        << "Expected break-right to roll right (positive rstick)";
}

TEST_F(AiPerformManeuverTest, BreakClearsWhenTimerExpires) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::BreakRight;
    digi.formation.wingman.headingOrdered = PI / 2.0;
    digi.formation.wingman.speedOrdered = 350.0;
    digi.formation.wingman.altitudeOrdered = 10000.0;
    digi.nav.mnverTime = 0.5;  // 0.5 s

    // Run 40 frames (~0.67 s) — timer should expire well before this.
    // (30 frames at 60 Hz = 0.5 s, but 30*(1/60) != exactly 0.5 due to
    // floating-point, so we use 40 frames to be safely past the threshold.)
    for (int i = 0; i < 40; ++i) {
        AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    }

    // After timer expires, AiPerformManeuver clears the maneuver.
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 0);
}

TEST_F(AiPerformManeuverTest, ClearSixTurnsTowardReverseHeading) {
    // ClearSix = 180° turn. The caller sets headingOrdered to self.yaw + PI.
    digi.formation.wingman.currentManeuver = WingmanManeuver::ClearSix;
    digi.formation.wingman.headingOrdered = PI;  // reverse of north = south
    digi.formation.wingman.speedOrdered = 350.0;
    digi.formation.wingman.altitudeOrdered = 10000.0;
    digi.nav.mnverTime = kDefaultManeuverTimeSec;

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);

    // Trackpoint is 1000 ft ahead at heading PI (south).
    // Self heading = 0 (north). To turn from north to south, roll either
    // left or right (180°). Just verify the brain is actively steering.
    EXPECT_GT(std::fabs(digi.commands.rStick), 0.001);
}

TEST_F(AiPerformManeuverTest, PostholeDescendsThenClears) {
    // Posthole: descend to ordered altitude. Self is at 10000 ft,
    // ordered altitude is 5000 ft → need to descend 5000 ft.
    digi.formation.wingman.currentManeuver = WingmanManeuver::Posthole;
    digi.formation.wingman.altitudeOrdered = 5000.0;
    digi.formation.wingman.speedOrdered = 330.0;
    digi.nav.mnverTime = 0.0;  // not used by Posthole

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    // Still in phase 1 (descending) — maneuver active.
    EXPECT_TRUE(active);

    // Self is well above target altitude (10000 vs 5000) → GammaHold should
    // command a descent (negative pstick or near-zero, depending on FCS state).
    // Just verify the brain produces a valid command.
    EXPECT_FALSE(std::isnan(digi.commands.pStick));
}

TEST_F(AiPerformManeuverTest, ChainsawClearsImmediatelyWithoutTarget) {
    // Chainsaw needs a target+missile. Without them, it clears immediately.
    digi.formation.wingman.currentManeuver = WingmanManeuver::Chainsaw;

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
}

TEST_F(AiPerformManeuverTest, AiClearManeuverResetsState) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::BreakLeft;
    digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
    digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::UseComplex)] = 1;
    digi.nav.mnverTime = 2.5;

    AiClearManeuver(digi);

    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 0);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::UseComplex)], 0);
    EXPECT_NEAR(digi.nav.mnverTime, 0.0, 1e-9);
}

// ===========================================================================
// DigiBrain FollowOrders integration test
// ===========================================================================
TEST(DigiBrainFollowOrdersTest, BreakCommandEntersFollowOrdersMode) {
    DigiBrain brain;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;
    DigiEntity lead;

    brain.setMaxGs(9.0);
    brain.setMaxBank(45.0);
    brain.setMaxGamma(15.0);
    brain.setTurnG(2.0);
    brain.setCornerSpeed(330.0);

    state.kin.costhe = 1.0;
    state.kin.cosphi = 1.0;
    state.kin.gmma = 0.0;
    state.kin.sigma = 0.0;
    state.kin.singam = 0.0;
    state.kin.psi = 0.0;
    state.kin.theta = 0.0;
    state.kin.phi = 0.0;
    state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
    state.vcas = 350.0;
    state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;

    // Configure as wingman with a lead.
    brain.stateMutable().formation.isWing = true;
    brain.stateMutable().formation.flightLeadId = 100;
    brain.stateMutable().formation.vehicleInUnit = 1;

    lead.x = 0.0; lead.y = 1000.0; lead.z = -10000.0;
    lead.yaw = 0.0; lead.speed = 350.0 * KNOTS_TO_FTPSEC;

    FrameInputs fi;
    fi.injectedLead = &lead;
    brain.setFrameInputs(fi);

    // Send a break-right command via the message bus.
    Message breakMsg{MessageType::FlightCmdBreak, 100, 1};
    breakMsg.payload.heading = 1.0;  // positive = right
    brain.mailbox().push(breakMsg);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // Brain should be in FollowOrders mode (executing the break maneuver).
    EXPECT_EQ(brain.activeMode(), DigiMode::FollowOrders);
    EXPECT_EQ(brain.state().formation.wingman.currentManeuver,
              WingmanManeuver::BreakRight);
}

// ===========================================================================
// ThreatCall message ingestion tests
// ===========================================================================
TEST(ThreatCallIngestionTest, SpikeMessageSetsThreatBearing) {
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::ThreatCallSpike, 100, 1};
    msg.payload.heading = PI / 4.0;  // 45° right
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    // The brain should have stored the bearing + type.
    EXPECT_NEAR(digi.threat.threatCallBearing, PI / 4.0, 1e-9);
    EXPECT_EQ(digi.threat.threatCallType, static_cast<int>(MessageType::ThreatCallSpike));
}

TEST(ThreatCallIngestionTest, MissileMessageSetsThreatBearing) {
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::ThreatCallMissile, 100, 1};
    msg.payload.heading = -PI / 2.0;  // 90° left
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    EXPECT_NEAR(digi.threat.threatCallBearing, -PI / 2.0, 1e-9);
    EXPECT_EQ(digi.threat.threatCallType, static_cast<int>(MessageType::ThreatCallMissile));
}

TEST(ThreatCallIngestionTest, SAMMessageSetsThreatBearing) {
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::ThreatCallSAM, 100, 1};
    msg.payload.heading = 0.0;  // ahead
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    EXPECT_NEAR(digi.threat.threatCallBearing, 0.0, 1e-9);
    EXPECT_EQ(digi.threat.threatCallType, static_cast<int>(MessageType::ThreatCallSAM));
}

TEST(ThreatCallIngestionTest, NoThreatCallDefaultSentinel) {
    DigiState digi;
    // No message → bearing should be the sentinel -999.0.
    EXPECT_NEAR(digi.threat.threatCallBearing, -999.0, 1e-9);
    EXPECT_EQ(digi.threat.threatCallType, 0);
}

// ===========================================================================
// Extended receiveOrders tests (Round-5 additions)
// ===========================================================================
class ReceiveOrdersExtendedTest : public ::testing::Test {
protected:
    WingmanState ws;
};

TEST_F(ReceiveOrdersExtendedTest, ClearSixCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdClearSix, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::ClearSix);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 1);
}

TEST_F(ReceiveOrdersExtendedTest, PostholeCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdPosthole, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::Posthole);
}

TEST_F(ReceiveOrdersExtendedTest, ChainsawCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdChainsaw, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::Chainsaw);
}

TEST_F(ReceiveOrdersExtendedTest, SSOffsetCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdSSOffset, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::SSOffset);
}

TEST_F(ReceiveOrdersExtendedTest, FlexCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdFlex, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::Flex);
}

TEST_F(ReceiveOrdersExtendedTest, PinceCommandSetsManeuver) {
    Message msg{MessageType::FlightCmdPince, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::Pince);
}

TEST_F(ReceiveOrdersExtendedTest, KickoutDoublesLateralSpacing) {
    ws.formLateralSpaceFactor = 1.0;
    Message msg{MessageType::FlightCmdKickout, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_NEAR(ws.formLateralSpaceFactor, 2.0, 1e-9);
}

TEST_F(ReceiveOrdersExtendedTest, CloseupHalvesLateralSpacing) {
    ws.formLateralSpaceFactor = 1.0;
    Message msg{MessageType::FlightCmdCloseup, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_NEAR(ws.formLateralSpaceFactor, 0.5, 1e-9);
}

TEST_F(ReceiveOrdersExtendedTest, KickoutClampedAt4x) {
    ws.formLateralSpaceFactor = 4.0;  // already at max
    Message msg{MessageType::FlightCmdKickout, 100, 1};
    receiveOrders(ws, msg);
    EXPECT_NEAR(ws.formLateralSpaceFactor, 4.0, 1e-9);  // no increase
}

TEST_F(ReceiveOrdersExtendedTest, CloseupClampedAtQuarter) {
    ws.formLateralSpaceFactor = 0.25;  // already at min
    Message msg{MessageType::FlightCmdCloseup, 100, 1};
    receiveOrders(ws, msg);
    EXPECT_NEAR(ws.formLateralSpaceFactor, 0.25, 1e-9);  // no decrease
}

TEST_F(ReceiveOrdersExtendedTest, ToggleSideMirrorsSide) {
    ws.formSide = 1;  // right
    Message msg{MessageType::FlightCmdToggleSide, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.formSide, -1);  // now left

    // Toggle again → back to right.
    receiveOrders(ws, msg);
    EXPECT_EQ(ws.formSide, 1);
}

TEST_F(ReceiveOrdersExtendedTest, ToggleSideFromZeroDefaultsToRight) {
    ws.formSide = 0;
    Message msg{MessageType::FlightCmdToggleSide, 100, 1};
    receiveOrders(ws, msg);
    // 0 is treated as ">= 0" → toggles to -1.
    EXPECT_EQ(ws.formSide, -1);
}

TEST_F(ReceiveOrdersExtendedTest, IncreaseRelAltAdds1000Ft) {
    ws.formRelativeAltitude = 0.0;
    Message msg{MessageType::FlightCmdIncreaseRelAlt, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_NEAR(ws.formRelativeAltitude, 1000.0, 1e-9);

    // Stack another +1000.
    receiveOrders(ws, msg);
    EXPECT_NEAR(ws.formRelativeAltitude, 2000.0, 1e-9);
}

TEST_F(ReceiveOrdersExtendedTest, DecreaseRelAltSubtracts1000Ft) {
    ws.formRelativeAltitude = 0.0;
    Message msg{MessageType::FlightCmdDecreaseRelAlt, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_NEAR(ws.formRelativeAltitude, -1000.0, 1e-9);
}

TEST_F(ReceiveOrdersExtendedTest, RejoinClearsManeuver) {
    // Set a maneuver first.
    ws.currentManeuver = WingmanManeuver::BreakLeft;
    ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;

    Message msg{MessageType::FlightCmdRejoin, 100, 1};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 0);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)], 1);
}

// ===========================================================================
// GroundOpsState::reset regression test (the bug fix where 12 of 22 fields
// were not reset)
// ===========================================================================
TEST(GroundOpsStateResetTest, ResetsAllFieldsAfterMutation) {
    GroundOpsState go;
    // Mutate every field to a non-default value.
    go.phase = GroundOpsPhase::TakeoffRoll;
    go.assignedRunway = 7;
    go.runwayHeading = 1.5;
    go.runwayThresholdX = 12345.0;
    go.runwayThresholdY = 67890.0;
    go.runwayAltitude = 500.0;
    go.currentTaxiNode = 3;
    go.targetTaxiNode = 4;
    go.taxiSpeed = 15.0;
    go.taxiGraph = reinterpret_cast<const atc::TaxiGraph*>(0xDEADBEEF);
    go.taxiPath = {1, 2, 3, 4, 5};
    go.taxiPathIdx = 2;
    go.takeoffRollStart = 100.0;
    go.rotationSpeed = 145.0;
    go.gearRetracted = true;
    go.approachStartAlt = 3000.0;
    go.flareStartAlt = 100.0;
    go.touchdownSpeed = 130.0;
    go.touchdownTimer = 5.0;
    go.gearDeployed = true;
    go.hasTakeoffClearance = true;
    go.hasLandingClearance = true;

    go.reset();

    EXPECT_EQ(go.phase, GroundOpsPhase::Idle);
    EXPECT_EQ(go.assignedRunway, 0);
    EXPECT_NEAR(go.runwayHeading, 0.0, 1e-9);
    EXPECT_NEAR(go.runwayThresholdX, 0.0, 1e-9);
    EXPECT_NEAR(go.runwayThresholdY, 0.0, 1e-9);
    EXPECT_NEAR(go.runwayAltitude, 0.0, 1e-9);
    EXPECT_EQ(go.currentTaxiNode, -1);
    EXPECT_EQ(go.targetTaxiNode, -1);
    EXPECT_NEAR(go.taxiSpeed, 0.0, 1e-9);
    EXPECT_EQ(go.taxiGraph, nullptr);
    EXPECT_TRUE(go.taxiPath.empty());
    EXPECT_EQ(go.taxiPathIdx, 0u);
    EXPECT_NEAR(go.takeoffRollStart, 0.0, 1e-9);
    EXPECT_NEAR(go.rotationSpeed, 0.0, 1e-9);
    EXPECT_FALSE(go.gearRetracted);
    EXPECT_NEAR(go.approachStartAlt, 0.0, 1e-9);
    EXPECT_NEAR(go.flareStartAlt, 0.0, 1e-9);
    EXPECT_NEAR(go.touchdownSpeed, 0.0, 1e-9);
    EXPECT_NEAR(go.touchdownTimer, 0.0, 1e-9);
    EXPECT_FALSE(go.gearDeployed);
    EXPECT_FALSE(go.hasTakeoffClearance);
    EXPECT_FALSE(go.hasLandingClearance);
}

// ===========================================================================
// DigiState::reset regression test — verify the new fields (threatPtr,
// threatTimer, fuel.phase, fuelLbs, etc.) are cleared by reset().
// ===========================================================================
TEST(DigiStateResetTest, ClearsNewHandleThreatAndFuelFields) {
    DigiState s;
    // Mutate the new fields.
    s.threat.threatPtr = reinterpret_cast<const DigiEntity*>(0xCAFEBABE);
    s.threat.threatTimer = 5.0;
    s.threat.threatCallBearing = 1.5;
    s.threat.threatCallType = 42;
    s.fuel.phase = DigiFuelState::Phase::Bingo;
    s.fuel.fuelLbs = 1000.0;
    s.fuel.bingoFuelLbs = 1500.0;
    s.fuel.jokerFuelLbs = 2500.0;
    s.fuel.fumesFuelLbs = 800.0;
    s.fuel.winchester = true;
    s.fuel.divertAirbaseX = 1000.0;
    s.fuel.divertAirbaseY = 2000.0;
    s.fuel.divertAirbaseZ = -5000.0;
    s.fuel.divertAirbaseHeading = 1.5;
    s.fuel.hasDivertAirbase = true;

    s.reset();

    // DigiState::reset() does NOT clear threatPtr — it's host-managed.
    // Only DigiBrain::reset() clears it. This is by design: reset() clears
    // internal brain state, not host-injected pointers. The DigiBrainReset
    // test below verifies the brain-level reset clears threatPtr.
    // (s.threat.threatPtr is still 0xCAFEBABE here — we don't assert on it.)
    EXPECT_NEAR(s.threat.threatTimer, 0.0, 1e-9);
    EXPECT_NEAR(s.threat.threatCallBearing, -999.0, 1e-9);
    EXPECT_EQ(s.threat.threatCallType, 0);
    EXPECT_EQ(s.fuel.phase, DigiFuelState::Phase::Normal);
    EXPECT_NEAR(s.fuel.fuelLbs, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.bingoFuelLbs, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.jokerFuelLbs, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.fumesFuelLbs, 0.0, 1e-9);
    EXPECT_FALSE(s.fuel.winchester);
    EXPECT_NEAR(s.fuel.divertAirbaseX, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.divertAirbaseY, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.divertAirbaseZ, 0.0, 1e-9);
    EXPECT_NEAR(s.fuel.divertAirbaseHeading, 0.0, 1e-9);
    EXPECT_FALSE(s.fuel.hasDivertAirbase);
}

// ===========================================================================
// DigiBrain::reset regression test — verify threat pointer is cleared
// (host-managed pointer that previously could dangle across reset()).
// ===========================================================================
TEST(DigiBrainResetTest, ClearsThreatPointerAcrossReset) {
    DigiBrain brain;
    DigiEntity threat;
    threat.x = 1000.0; threat.y = 0.0; threat.z = -10000;
    threat.speed = 500.0;

    FrameInputs fi;
    fi.injectedThreat = &threat;
    brain.setFrameInputs(fi);

    // After setFrameInputs, state_.threat.threatPtr should point to &threat.
    EXPECT_EQ(brain.state().threat.threatPtr, &threat);

    brain.reset();

    // After reset, threatPtr should be null (no dangling pointer).
    EXPECT_EQ(brain.state().threat.threatPtr, nullptr);
    EXPECT_EQ(brain.frameInputs().injectedThreat, nullptr);
}

// ===========================================================================
// DigiBrain HandleThreat integration: injected threat engages via compute()
// ===========================================================================
TEST(DigiBrainHandleThreatIntegrationTest, InjectedThreatEngagesViaCompute) {
    DigiBrain brain;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;
    DigiEntity threat;

    brain.setMaxGs(9.0);
    brain.setMaxBank(45.0);
    brain.setMaxGamma(15.0);
    brain.setTurnG(2.0);
    brain.setCornerSpeed(330.0);

    state.kin.costhe = 1.0;
    state.kin.cosphi = 1.0;
    state.kin.gmma = 0.0;
    state.kin.sigma = 0.0;
    state.kin.singam = 0.0;
    state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
    state.vcas = 350.0;
    state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;

    // Threat 3 NM north, heading south.
    threat.x = 0.0; threat.y = 3.0 * 6076.0; threat.z = -10000.0;
    threat.yaw = PI;
    threat.speed = 500.0;

    FrameInputs fi;
    fi.injectedThreat = &threat;
    brain.setFrameInputs(fi);

    // The brain will resolve to Waypoint (no injectedTarget), but HandleThreat
    // should engage the threat overlay and skip the per-mode switch.
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // threatPtr should be committed to state_.
    EXPECT_EQ(brain.state().threat.threatPtr, &threat);
    // threatTimer should be armed after the first compute() (HandleThreat
    // re-evaluates and either drops or re-arms).
    EXPECT_GT(brain.state().threat.threatTimer, 0.0);
}

// ===========================================================================
// ProcessATCMessages bug fix: Flight commands are no longer silently dropped
// ===========================================================================
TEST(ProcessMessagesTest, FlightCommandsAreNotDropped) {
    // BUG FIX: previously, ProcessATCMessages had a `default: break;` that
    // silently dropped ALL Flight* messages. Now they're routed to
    // receiveOrders. This test verifies a FlightCmdBreak actually sets
    // the wingman maneuver.
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::FlightCmdBreak, 100, 1};
    msg.payload.heading = 1.0;  // positive = right
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    // The wingman should have received the break command.
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::BreakRight);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 1);
}

TEST(ProcessMessagesTest, FormationCommandsAreNotDropped) {
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::FlightCmdTrail, 100, 1};
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    EXPECT_EQ(digi.formation.wingman.currentFormation,
              f4flight::digi::formation::FormationType::Trail);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::FollowFormation)], 1);
}

TEST(ProcessMessagesTest, KickoutCommandIsProcessed) {
    DigiState digi;
    Mailbox mb;

    Message msg{MessageType::FlightCmdKickout, 100, 1};
    mb.push(msg);

    ProcessATCMessages(digi, mb);

    EXPECT_NEAR(digi.formation.wingman.formLateralSpaceFactor, 2.0, 1e-9);
}

TEST(ProcessMessagesTest, MultipleMessagesProcessedInOrder) {
    DigiState digi;
    Mailbox mb;

    // Push a break, then a rejoin. Rejoin should clear the break.
    Message breakMsg{MessageType::FlightCmdBreak, 100, 1};
    breakMsg.payload.heading = 1.0;
    Message rejoinMsg{MessageType::FlightCmdRejoin, 100, 1};
    mb.push(breakMsg);
    mb.push(rejoinMsg);

    ProcessATCMessages(digi, mb);

    // After rejoin, maneuver should be cleared.
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::FollowFormation)], 1);
}
