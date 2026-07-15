// f4flight unit tests - Round 6: AiExecPince/AiExecFlex, AiExecPosthole/Chainsaw
// with target+sms, AirbaseCheck, SeparateCheck, CommandFlight.
//
// These tests cover the P0 items completed in Round 6:
//   - AiInitPince / AiExecPince (2-point bracket maneuver)
//   - AiInitFlex / AiExecFlex (3-point S-curve maneuver)
//   - AiExecPosthole with target (can now engage via GunsEngage)
//   - AiExecChainsaw with target+sms (can now engage via MissileEngage)
//   - AirbaseCheck (auto-pick nearest airbase + RTB→Landing transition)
//   - SeparateCheck (damage abort, bugout timer, lateral separation)
//   - CommandFlight (flight-lead issues engage/rejoin orders)

#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/wingman/wingman_state.h"
#include "f4flight/digi/wingman/wingman_maneuvers.h"
#include "f4flight/digi/decision/decision_routines.h"
#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// AiInitPince / AiExecPince tests
// ===========================================================================
class PinceTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;
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

        // Self at origin, heading north, level at 10000 ft.
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 350.0 * KNOTS_TO_FTPSEC;

        // Target 10 NM north, heading south.
        target.x = 0.0; target.y = 10.0 * 6076.0; target.z = -10000.0;
        target.yaw = PI;
        target.speed = 500.0;

        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.psi = 0.0; as.kin.theta = 0.0; as.kin.phi = 0.0;
        as.vcas = 350.0;
        as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(PinceTest, InitPinceSetsTwoPoints) {
    AiInitPince(digi, self, &target, nullptr);

    // Point 0 should be ~20 NM along target bearing + 5 NM lateral.
    // Target bearing from self = atan2(10NM, 0) = PI/2 (north).
    // side = (vehicleInUnit & 1) ? 1 : -1. Default vehicleInUnit=0 → side=-1.
    // Point 0: cos(PI/2)*20NM - sin(PI/2)*5NM*(-1) = 0 + 5NM = 5NM east
    //          sin(PI/2)*20NM + cos(PI/2)*5NM*(-1) = 20NM + 0 = 20NM north
    EXPECT_NEAR(digi.formation.maneuverPoints[0].x, 5.0 * 6076.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[0].y, 20.0 * 6076.0, 1.0);

    // Point 1: 4 NM along + 5 NM lateral
    // cos(PI/2)*4NM - sin(PI/2)*5NM*(-1) = 0 + 5NM = 5NM east
    // sin(PI/2)*4NM + cos(PI/2)*5NM*(-1) = 4NM + 0 = 4NM north
    EXPECT_NEAR(digi.formation.maneuverPoints[1].x, 5.0 * 6076.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[1].y, 4.0 * 6076.0, 1.0);

    EXPECT_EQ(digi.formation.maneuverPointCounter, 0);
}

TEST_F(PinceTest, InitPinceOddSlotMirrorsSide) {
    digi.formation.vehicleInUnit = 1;  // odd slot → side = +1
    AiInitPince(digi, self, &target, nullptr);

    // side = +1: point 0 x = 0 - 5NM = -5NM east (mirrored)
    EXPECT_NEAR(digi.formation.maneuverPoints[0].x, -5.0 * 6076.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[0].y, 20.0 * 6076.0, 1.0);
}

TEST_F(PinceTest, InitPinceWithoutTargetUsesLeadYaw) {
    DigiEntity lead;
    lead.x = 0; lead.y = 0; lead.z = -10000;
    lead.yaw = 0.0;  // north
    lead.speed = 350.0 * KNOTS_TO_FTPSEC;

    AiInitPince(digi, self, nullptr, &lead);

    // trigYaw = lead.yaw = 0 (north).
    // Point 0: cos(0)*20NM - sin(0)*5NM*side = 20NM east... wait, cos(0)=1, sin(0)=0
    // x = 0 + 1*20NM - 0*5NM*side = 20NM
    // y = 0 + 0*20NM + 1*5NM*side = 5NM*side
    // side = -1 (vehicleInUnit=0) → y = -5NM
    EXPECT_NEAR(digi.formation.maneuverPoints[0].x, 20.0 * 6076.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[0].y, -5.0 * 6076.0, 1.0);
}

TEST_F(PinceTest, InitPinceWithoutTargetOrLeadUsesSelfYaw) {
    AiInitPince(digi, self, nullptr, nullptr);

    // trigYaw = self.yaw = 0 (north). Same as lead case above.
    EXPECT_NEAR(digi.formation.maneuverPoints[0].x, 20.0 * 6076.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[0].y, -5.0 * 6076.0, 1.0);
}

TEST_F(PinceTest, ExecPinceFliesToFirstPoint) {
    AiInitPince(digi, self, &target, nullptr);
    digi.formation.wingman.currentManeuver = WingmanManeuver::Pince;

    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);
    EXPECT_EQ(digi.formation.maneuverPointCounter, 0);  // haven't arrived yet

    // Should be steering toward point 0.
    EXPECT_FALSE(std::isnan(digi.commands.rStick));
}

TEST_F(PinceTest, ExecPinceAdvancesOnArrival) {
    AiInitPince(digi, self, &target, nullptr);

    // Move self to point 0 (within 5000 ft threshold).
    self.x = digi.formation.maneuverPoints[0].x;
    self.y = digi.formation.maneuverPoints[0].y;
    as.kin.x = self.x;
    as.kin.y = self.y;

    digi.formation.wingman.currentManeuver = WingmanManeuver::Pince;
    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);
    EXPECT_EQ(digi.formation.maneuverPointCounter, 1);  // advanced to point 1
}

TEST_F(PinceTest, ExecPinceCompletesAfterBothPoints) {
    AiInitPince(digi, self, &target, nullptr);

    // Visit point 0.
    self.x = digi.formation.maneuverPoints[0].x;
    self.y = digi.formation.maneuverPoints[0].y;
    as.kin.x = self.x; as.kin.y = self.y;
    digi.formation.wingman.currentManeuver = WingmanManeuver::Pince;
    AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_EQ(digi.formation.maneuverPointCounter, 1);

    // Visit point 1.
    self.x = digi.formation.maneuverPoints[1].x;
    self.y = digi.formation.maneuverPoints[1].y;
    as.kin.x = self.x; as.kin.y = self.y;
    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);  // maneuver complete
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
}

// ===========================================================================
// AiInitFlex / AiExecFlex tests
// ===========================================================================
class FlexTest : public ::testing::Test {
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

        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 350.0 * KNOTS_TO_FTPSEC;

        as.kin.costhe = 1.0; as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0; as.kin.sigma = 0.0; as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.psi = 0.0; as.kin.theta = 0.0; as.kin.phi = 0.0;
        as.vcas = 350.0; as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(FlexTest, InitFlexSetsThreePoints) {
    AiInitFlex(digi, self, nullptr, nullptr);

    // trigYaw = self.yaw = 0 (north). secondYaw = PI/2 (east).
    // spacing = 1 NM = 6076 ft.
    // Point 0: x = 0 + cos(PI/2)*6076 = 0; y = 0 + sin(0)*6076 = 0
    //   Wait, FF uses firstSin for point 0 y. firstSin = sin(0) = 0.
    //   So point 0 = (0, 0)? That seems wrong but matches FF's code.
    //   Actually FF: mpManeuverPoints[0][0] = XSelf + secondTrig.cos * spacing
    //                mpManeuverPoints[0][1] = YSelf + firstTrig.sin * spacing
    //   secondTrig.cos = cos(PI/2) = 0, firstTrig.sin = sin(0) = 0.
    //   So point 0 = (0, 0). This is a known FF quirk — point 0 is at self.
    EXPECT_NEAR(digi.formation.maneuverPoints[0].x, 0.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[0].y, 0.0, 1.0);

    // Point 1: x = 0 - cos(PI/2)*2*6076 = 0; y = 0 - sin(PI/2)*2*6076 = -2NM
    EXPECT_NEAR(digi.formation.maneuverPoints[1].x, 0.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[1].y, -2.0 * 6076.0, 1.0);

    // Point 2: x = 0 - cos(PI/2)*2.1*6076 = 0; y = 0 - sin(PI/2)*2.1*6076 = -2.1NM
    EXPECT_NEAR(digi.formation.maneuverPoints[2].x, 0.0, 1.0);
    EXPECT_NEAR(digi.formation.maneuverPoints[2].y, -2.1 * 6076.0, 1.0);

    EXPECT_EQ(digi.formation.maneuverPointCounter, 0);
}

TEST_F(FlexTest, ExecFlexCompletesAfterThreePoints) {
    AiInitFlex(digi, self, nullptr, nullptr);
    digi.formation.wingman.currentManeuver = WingmanManeuver::Flex;

    // Point 0 is at (0,0) — self is already there. Visit it.
    AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_EQ(digi.formation.maneuverPointCounter, 1);

    // Visit point 1.
    self.y = digi.formation.maneuverPoints[1].y;
    as.kin.y = self.y;
    AiPerformManeuver(digi, self, nullptr, nullptr, as, fcs, fcsState, 1.0/60.0);
    EXPECT_EQ(digi.formation.maneuverPointCounter, 2);

    // Visit point 2.
    self.y = digi.formation.maneuverPoints[2].y;
    as.kin.y = self.y;
    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
}

// ===========================================================================
// AiExecPosthole with target (Round 6: can now engage)
// ===========================================================================
class PostholeEngageTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;
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

        self.x = 0.0; self.y = 0.0; self.z = -5000.0;  // already at target alt
        self.yaw = 0.0; self.speed = 350.0 * KNOTS_TO_FTPSEC;

        target.x = 3000.0; target.y = 0.0; target.z = -5000.0;
        target.yaw = PI; target.speed = 500.0;

        as.kin.costhe = 1.0; as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0; as.kin.sigma = 0.0; as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -5000.0;
        as.kin.psi = 0.0; as.kin.theta = 0.0; as.kin.phi = 0.0;
        as.vcas = 350.0; as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(PostholeEngageTest, PostholeEngagesWhenTargetAvailable) {
    // Self already at ordered altitude → phase 2 immediately.
    digi.formation.wingman.currentManeuver = WingmanManeuver::Posthole;
    digi.formation.wingman.altitudeOrdered = 5000.0;
    digi.formation.wingman.speedOrdered = 350.0;

    const bool active = AiPerformManeuver(digi, self, &target, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    // With target available and at altitude, Posthole engages (returns true).
    EXPECT_TRUE(active);
    // Should have produced some stick command (GunsEngage writes pStick/rStick).
    EXPECT_FALSE(std::isnan(digi.commands.pStick));
}

TEST_F(PostholeEngageTest, PostholeRejoinsWhenNoTarget) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::Posthole;
    digi.formation.wingman.altitudeOrdered = 5000.0;
    digi.formation.wingman.speedOrdered = 350.0;

    // No target → rejoin (return false).
    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
}

TEST_F(PostholeEngageTest, PostholeDescendsToPhase2) {
    // Self at 10000 ft, ordered altitude 5000 → must descend first.
    self.z = -10000.0;
    as.kin.z = -10000.0;
    digi.formation.wingman.currentManeuver = WingmanManeuver::Posthole;
    digi.formation.wingman.altitudeOrdered = 5000.0;
    digi.formation.wingman.speedOrdered = 350.0;

    // Phase 1: descending. Should return true (still active).
    const bool active = AiPerformManeuver(digi, self, &target, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_TRUE(active);
    // Should NOT be in phase 2 yet (5000 ft away from target alt).
    EXPECT_EQ(digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::UseComplex)], 0);
}

// ===========================================================================
// AiExecChainsaw with target+sms (Round 6: can now engage)
// ===========================================================================
class ChainsawEngageTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;
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

        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 350.0 * KNOTS_TO_FTPSEC;

        target.x = 5.0 * 6076.0; target.y = 0.0; target.z = -10000.0;
        target.yaw = PI; target.speed = 500.0;

        as.kin.costhe = 1.0; as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0; as.kin.sigma = 0.0; as.kin.singam = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.psi = 0.0; as.kin.theta = 0.0; as.kin.phi = 0.0;
        as.vcas = 350.0; as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

// We use the real StoresManagementSystem (concrete class) with addHardpoint
// to set up the "has missiles" / "no missiles" test scenarios.
// The SMS is not abstract, so we configure it directly rather than mocking.

TEST_F(ChainsawEngageTest, ChainsawRejoinsWithoutTarget) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::Chainsaw;
    const bool active = AiPerformManeuver(digi, self, nullptr, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);
    EXPECT_EQ(digi.formation.wingman.currentManeuver, WingmanManeuver::None);
}

TEST_F(ChainsawEngageTest, ChainsawRejoinsWithoutSMS) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::Chainsaw;
    const bool active = AiPerformManeuver(digi, self, &target, nullptr,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);  // no SMS → rejoin
}

TEST_F(ChainsawEngageTest, ChainsawRejoinsWithSMSButNoMissiles) {
    StoresManagementSystem sms;  // empty SMS — no hardpoints added
    digi.formation.wingman.currentManeuver = WingmanManeuver::Chainsaw;
    const bool active = AiPerformManeuver(digi, self, &target, &sms,
                                           as, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(active);  // no missiles → rejoin
}

// ===========================================================================
// AirbaseCheck tests
// ===========================================================================
class AirbaseCheckTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    FrameInputs fi;

    void SetUp() override {
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 500.0;

        digi.fuel.bingoFuelLbs = 1500.0;
        digi.fuel.jokerFuelLbs = 2500.0;
        digi.fuel.fumesFuelLbs = 800.0;
    }
};

TEST_F(AirbaseCheckTest, NoAirbasesReturnsNone) {
    fi.airbases = nullptr;
    fi.airbaseCount = 0;
    EXPECT_EQ(AirbaseCheck(digi, self, fi, 0.0), AirbaseAction::None);
}

TEST_F(AirbaseCheckTest, BingoBeyondReturnDistanceSetsDivert) {
    // Airbase 60 NM north (beyond kBingoReturnDistanceNm=50).
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 60.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;
    fi.airbases = &ab;
    fi.airbaseCount = 1;

    digi.fuel.fuelLbs = 1400.0;  // below bingo
    digi.fuel.phase = DigiFuelState::Phase::Bingo;

    const auto action = AirbaseCheck(digi, self, fi, 0.0);
    EXPECT_EQ(action, AirbaseAction::RTB);
    EXPECT_TRUE(digi.fuel.hasDivertAirbase);
    EXPECT_NEAR(digi.fuel.divertAirbaseX, 0.0, 1.0);
    EXPECT_NEAR(digi.fuel.divertAirbaseY, 60.0 * 6076.0, 1.0);
}

TEST_F(AirbaseCheckTest, BingoWithinReturnDistanceNoDivert) {
    // Airbase 30 NM north (within 50 NM) — no divert needed yet.
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 30.0 * 6076.0; ab.z = -5000.0;
    fi.airbases = &ab;
    fi.airbaseCount = 1;

    digi.fuel.fuelLbs = 1400.0;
    digi.fuel.phase = DigiFuelState::Phase::Bingo;

    const auto action = AirbaseCheck(digi, self, fi, 0.0);
    // Bingo but within return distance — no divert set yet (brain keeps fighting).
    EXPECT_EQ(action, AirbaseAction::None);
    EXPECT_FALSE(digi.fuel.hasDivertAirbase);
}

TEST_F(AirbaseCheckTest, FumesForcesDivertAndLanding) {
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 5.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;
    fi.airbases = &ab;
    fi.airbaseCount = 1;

    digi.fuel.fuelLbs = 500.0;  // below fumes
    digi.fuel.phase = DigiFuelState::Phase::Fumes;

    const auto action = AirbaseCheck(digi, self, fi, 0.0);
    EXPECT_EQ(action, AirbaseAction::Landing);  // fumes → direct landing
    EXPECT_TRUE(digi.fuel.hasDivertAirbase);
    EXPECT_EQ(digi.ag.groundOps.phase, GroundOpsPhase::Approach);
}

TEST_F(AirbaseCheckTest, DamageBelowThresholdForcesLanding) {
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 5.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;
    fi.airbases = &ab;
    fi.airbaseCount = 1;

    digi.damage.pctStrength = 0.30;  // < 0.50 threshold

    const auto action = AirbaseCheck(digi, self, fi, 0.0);
    EXPECT_EQ(action, AirbaseAction::Landing);
}

TEST_F(AirbaseCheckTest, PicksNearestOfMultipleAirbases) {
    FrameInputs::AirbaseInfo abs[3];
    abs[0].x = 100.0 * 6076.0; abs[0].y = 0.0;     // 100 NM east
    abs[1].x = 0.0;             abs[1].y = 20.0 * 6076.0;  // 20 NM north (nearest)
    abs[2].x = -50.0 * 6076.0; abs[2].y = 0.0;    // 50 NM west
    fi.airbases = abs;
    fi.airbaseCount = 3;

    digi.fuel.fuelLbs = 500.0;
    digi.fuel.phase = DigiFuelState::Phase::Fumes;

    AirbaseCheck(digi, self, fi, 0.0);
    // Divert should be the nearest (20 NM north).
    EXPECT_NEAR(digi.fuel.divertAirbaseX, 0.0, 1.0);
    EXPECT_NEAR(digi.fuel.divertAirbaseY, 20.0 * 6076.0, 1.0);
}

TEST_F(AirbaseCheckTest, TransitionsToLandingWhenClose) {
    // Already have a divert airbase 5 NM north.
    digi.fuel.hasDivertAirbase = true;
    digi.fuel.divertAirbaseX = 0.0;
    digi.fuel.divertAirbaseY = 5.0 * 6076.0;  // 5 NM (within 10 NM transition)
    digi.fuel.divertAirbaseZ = -5000.0;
    digi.fuel.divertAirbaseHeading = 0.0;

    // Need an airbase entry (AirbaseCheck requires non-null list even when
    // divert is already set, because it uses the list for the approach setup).
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 5.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;
    fi.airbases = &ab;
    fi.airbaseCount = 1;

    const auto action = AirbaseCheck(digi, self, fi, 0.0);
    EXPECT_EQ(action, AirbaseAction::Landing);
    EXPECT_EQ(digi.ag.groundOps.phase, GroundOpsPhase::Approach);
}

// ===========================================================================
// SeparateCheck tests
// ===========================================================================
class SeparateCheckTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;

    void SetUp() override {
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 500.0;

        // Default: target 3 NM east, heading toward self (west).
        // Not deep-six (ataFrom ≈ 0).
        target.x = 3.0 * 6076.0; target.y = 0.0; target.z = -10000.0;
        target.yaw = PI; target.speed = 500.0;
    }
};

TEST_F(SeparateCheckTest, NoDamageNoTargetReturnsNone) {
    digi.damage.pctStrength = 1.0;
    EXPECT_EQ(SeparateCheck(digi, self, nullptr, 1.0/60.0), SeparateAction::None);
}

TEST_F(SeparateCheckTest, DamageBelowThresholdReturnsRTB) {
    digi.damage.pctStrength = 0.30;
    EXPECT_EQ(SeparateCheck(digi, self, nullptr, 1.0/60.0), SeparateAction::RTB);
}

TEST_F(SeparateCheckTest, DamageAtThresholdDoesNotAbort) {
    digi.damage.pctStrength = 0.50;  // exactly at threshold (not < 0.50)
    EXPECT_EQ(SeparateCheck(digi, self, nullptr, 1.0/60.0), SeparateAction::None);
}

TEST_F(SeparateCheckTest, DeepSixStartsBugoutTimer) {
    // Deep-six: self is on target's tail (target running away from self).
    // Coordinate convention: yaw=0 = +X = east (per test_digi_defensive.cpp).
    // Target at (3NM, 0) heading east (yaw=0). Self at origin.
    // bearing target→self = atan2(0-0, 0-3NM) = PI (west).
    // ataFrom = PI - target.yaw(0) = PI → |ataFrom| = 180° > 135°. Deep six.
    target.x = 3.0 * 6076.0; target.y = 0.0;
    target.yaw = 0.0;  // heading east (away from self, who is west of target)

    digi.damage.pctStrength = 1.0;
    const auto action = SeparateCheck(digi, self, &target, 1.0/60.0);

    // Timer should be active but not expired yet (90 s > 1 frame).
    EXPECT_TRUE(digi.damage.bugoutTimerActive);
    EXPECT_GT(digi.damage.bugoutTimer, 0.0);
    // Should return None (timer not expired).
    EXPECT_EQ(action, SeparateAction::None);
}

TEST_F(SeparateCheckTest, BugoutTimerExpiresAfter90Seconds) {
    // Set up deep-six geometry (target heading east, self west of target).
    target.x = 3.0 * 6076.0; target.y = 0.0;
    target.yaw = 0.0;
    digi.damage.pctStrength = 1.0;

    // Pre-arm the timer at 1 second remaining (so we don't have to run 5400
    // frames). The first SeparateCheck call starts the timer at 90 s; we
    // manually set it to 1.0 to simulate 89 s having passed.
    digi.damage.bugoutTimer = 1.0;
    digi.damage.bugoutTimerActive = true;

    // Run 70 frames (~1.17 s) — timer should expire.
    for (int i = 0; i < 70; ++i) {
        const auto action = SeparateCheck(digi, self, &target, 1.0/60.0);
        if (action == SeparateAction::Bugout) {
            return;  // success
        }
    }
    FAIL() << "Bugout did not trigger after timer expiry";
}

TEST_F(SeparateCheckTest, DeepSixResetWhenTargetTurns) {
    // Start deep-six (target heading east, self west).
    target.x = 3.0 * 6076.0; target.y = 0.0;
    target.yaw = 0.0;
    digi.damage.pctStrength = 1.0;
    SeparateCheck(digi, self, &target, 1.0/60.0);
    EXPECT_TRUE(digi.damage.bugoutTimerActive);

    // Target now turns toward self (heading west) → ataFrom ≈ 0 (not deep six).
    target.yaw = PI;  // heading west (toward self)
    SeparateCheck(digi, self, &target, 1.0/60.0);
    EXPECT_FALSE(digi.damage.bugoutTimerActive);
    EXPECT_NEAR(digi.damage.bugoutTimer, 0.0, 1e-9);
}

TEST_F(SeparateCheckTest, SeparateWhenBingoAndTargetInRange) {
    // Target 4 NM east, heading toward self (west) → not deep six.
    target.x = 4.0 * 6076.0; target.y = 0.0;
    target.yaw = PI;  // heading west (toward self)
    digi.damage.pctStrength = 1.0;
    digi.fuel.phase = DigiFuelState::Phase::Bingo;

    const auto action = SeparateCheck(digi, self, &target, 1.0/60.0);
    EXPECT_EQ(action, SeparateAction::Separate);
}

TEST_F(SeparateCheckTest, NoSeparateWhenTargetOutOfRange) {
    target.x = 8.0 * 6076.0; target.y = 0.0;  // 8 NM — beyond 6 NM max
    target.yaw = PI;
    digi.damage.pctStrength = 1.0;
    digi.fuel.phase = DigiFuelState::Phase::Bingo;

    const auto action = SeparateCheck(digi, self, &target, 1.0/60.0);
    EXPECT_EQ(action, SeparateAction::None);
}

// ===========================================================================
// CommandFlight tests
// ===========================================================================
class CommandFlightTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity target;
    MessageBus bus;
    Mailbox wingmanMailbox;
    static constexpr EntityId kLeadId = 100;
    static constexpr EntityId kWingmanId = 101;

    void SetUp() override {
        digi.formation.isWing = false;       // this aircraft is a lead
        digi.formation.vehicleInUnit = 0;    // slot 0 = lead
        digi.comm.selfId = kLeadId;

        target.x = 5.0 * 6076.0; target.y = 0.0; target.z = -10000.0;
        target.yaw = PI; target.speed = 500.0;
        target.isDead = false;

        // Register wingman mailbox + add to flight group.
        bus.registerMailbox(kWingmanId, &wingmanMailbox);
        bus.addToGroup(kLeadId, kWingmanId);  // flight group = lead ID
    }
};

TEST_F(CommandFlightTest, LeadWithTargetSendsEngageOrder) {
    CommandFlight(digi, &target, &bus, kLeadId, 0.0);

    // Wingman should have received FlightCmdEngage.
    auto msg = wingmanMailbox.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::FlightCmdEngage);
    EXPECT_EQ(msg->sender, kLeadId);

    // And FlightCmdWeaponsFree.
    msg = wingmanMailbox.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::FlightCmdWeaponsFree);
}

TEST_F(CommandFlightTest, LeadWithoutTargetSendsRejoin) {
    CommandFlight(digi, nullptr, &bus, kLeadId, 0.0);

    auto msg = wingmanMailbox.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::FlightCmdRejoin);
}

TEST_F(CommandFlightTest, WingmanDoesNotIssueOrders) {
    digi.formation.isWing = true;  // this aircraft is a wingman
    digi.formation.vehicleInUnit = 1;

    CommandFlight(digi, &target, &bus, kLeadId, 0.0);

    // Wingman should NOT have received anything (the "lead" is a wingman).
    EXPECT_TRUE(wingmanMailbox.empty());
}

TEST_F(CommandFlightTest, ThrottlesOrdersTo5Seconds) {
    // First call at t=0 sends orders.
    CommandFlight(digi, &target, &bus, kLeadId, 0.0);
    EXPECT_FALSE(wingmanMailbox.empty());
    while (wingmanMailbox.pop()) {}  // drain

    // Second call at t=2 (within 5 s throttle) — no new orders.
    CommandFlight(digi, &target, &bus, kLeadId, 2.0);
    EXPECT_TRUE(wingmanMailbox.empty());

    // Third call at t=6 (past 5 s) — new orders.
    CommandFlight(digi, &target, &bus, kLeadId, 6.0);
    EXPECT_FALSE(wingmanMailbox.empty());
}

TEST_F(CommandFlightTest, NoBusIsNoOp) {
    CommandFlight(digi, &target, nullptr, kLeadId, 0.0);
    // No crash, no orders (nothing to publish to).
}

TEST_F(CommandFlightTest, DeadTargetTriggersRejoin) {
    target.isDead = true;
    CommandFlight(digi, &target, &bus, kLeadId, 0.0);

    auto msg = wingmanMailbox.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::FlightCmdRejoin);
}

// ===========================================================================
// DigiState::reset regression test for Round 6 new fields
// ===========================================================================
TEST(DigiStateResetTestRound6, ClearsDamageAndManeuverPoints) {
    DigiState s;
    s.damage.pctStrength = 0.3;
    s.damage.bugoutTimer = 50.0;
    s.damage.bugoutTimerActive = true;
    s.damage.saidBingo = true;
    s.damage.saidFumes = true;
    s.damage.saidRTB = true;
    s.formation.maneuverPointCounter = 2;
    s.formation.maneuverPoints[0].x = 1234.0;
    s.formation.maneuverPoints[0].y = 5678.0;

    s.reset();

    EXPECT_NEAR(s.damage.pctStrength, 1.0, 1e-9);
    EXPECT_NEAR(s.damage.bugoutTimer, 0.0, 1e-9);
    EXPECT_FALSE(s.damage.bugoutTimerActive);
    EXPECT_FALSE(s.damage.saidBingo);
    EXPECT_FALSE(s.damage.saidFumes);
    EXPECT_FALSE(s.damage.saidRTB);
    EXPECT_EQ(s.formation.maneuverPointCounter, 0);
    EXPECT_NEAR(s.formation.maneuverPoints[0].x, 0.0, 1e-9);
    EXPECT_NEAR(s.formation.maneuverPoints[0].y, 0.0, 1e-9);
}

// ===========================================================================
// DigiBrain integration: AirbaseCheck end-to-end via compute()
// ===========================================================================
TEST(DigiBrainAirbaseIntegrationTest, FumesWithAirbaseEntersLanding) {
    DigiBrain brain;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    brain.setMaxGs(9.0);
    brain.setMaxBank(45.0);
    brain.setMaxGamma(15.0);
    brain.setTurnG(2.0);
    brain.setCornerSpeed(330.0);

    state.kin.costhe = 1.0; state.kin.cosphi = 1.0;
    state.kin.gmma = 0.0; state.kin.sigma = 0.0; state.kin.singam = 0.0;
    state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.vcas = 350.0; state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;

    // Airbase 5 NM north.
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 5.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;

    FrameInputs fi;
    fi.fuelLbs = 500.0;  // fumes
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    fi.airbases = &ab;
    fi.airbaseCount = 1;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // Should be in Landing mode (fumes + airbase within range → direct landing).
    EXPECT_EQ(brain.activeMode(), DigiMode::Landing);
    EXPECT_TRUE(brain.state().fuel.hasDivertAirbase);
}

TEST(DigiBrainAirbaseIntegrationTest, BingoWithAirbaseBeyond50NmEntersRTB) {
    DigiBrain brain;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    brain.setMaxGs(9.0);
    brain.setMaxBank(45.0);
    brain.setMaxGamma(15.0);
    brain.setTurnG(2.0);
    brain.setCornerSpeed(330.0);

    state.kin.costhe = 1.0; state.kin.cosphi = 1.0;
    state.kin.gmma = 0.0; state.kin.sigma = 0.0; state.kin.singam = 0.0;
    state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
    state.kin.zdot = 0.0;
    state.vcas = 350.0; state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;

    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 60.0 * 6076.0; ab.z = -5000.0;  // 60 NM (beyond 50)
    ab.runwayHeading = 0.0; ab.id = 100;

    FrameInputs fi;
    fi.fuelLbs = 1400.0;  // bingo
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    fi.airbases = &ab;
    fi.airbaseCount = 1;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
    EXPECT_TRUE(brain.state().fuel.hasDivertAirbase);
}
