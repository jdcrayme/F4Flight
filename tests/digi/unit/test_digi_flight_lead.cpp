// f4flight unit tests - flight lead logic (decision/flight_lead.h)
//
// Tests for:
//   - FlightLeadDecisions (engage/disengage, formation management)
//   - TargetPriority (range, aspect, speed, altitude scoring)
//   - ShouldEngage (range, fuel, damage, weapons checks)
//   - ShouldDisengage (fuel, damage, winchester, target escaped)
//   - ShouldRejoin (no target, no threat)

#include "f4flight/digi/decision/flight_lead.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/flight/core/constants.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight::digi;

namespace {
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper: create a DigiEntity at a position with heading + speed.
f4flight::digi::DigiEntity makeEntity(double x, double y, double z,
                                       double yaw, double speedFtps) {
    f4flight::digi::DigiEntity e{};
    e.x = x; e.y = y; e.z = z;
    e.yaw = yaw;
    e.speed = speedFtps;
    e.isDead = false;
    return e;
}
} // namespace

// ===========================================================================
// TargetPriority tests
// ===========================================================================

TEST(FlightLeadTest, TargetPriorityCloseTargetHigherThanFar) {
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity close = makeEntity(0, 5*6076, -15000, M_PI, 400);  // 5 NM, tail-on
    DigiEntity far = makeEntity(0, 50*6076, -15000, M_PI, 400);   // 50 NM, tail-on

    double closeScore = TargetPriority(self, close);
    double farScore = TargetPriority(self, far);
    EXPECT_GT(closeScore, farScore)
        << "Close target should have higher priority than far target";
}

TEST(FlightLeadTest, TargetPriorityNoseOnHigherThanTailOn) {
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    // Nose-on target: 10 NM north, pointing SOUTH (toward us).
    // In F4Flight's convention (yaw=0 → +X/East, yaw=π/2 → +Y/North),
    // south = -Y = yaw = -π/2.
    DigiEntity noseOn = makeEntity(0, 10*6076, -15000, -M_PI/2, 400);
    // Tail-on target: 10 NM north, pointing NORTH (away from us).
    // North = +Y = yaw = +π/2.
    DigiEntity tailOn = makeEntity(0, 10*6076, -15000, M_PI/2, 400);

    double noseOnScore = TargetPriority(self, noseOn);
    double tailOnScore = TargetPriority(self, tailOn);
    EXPECT_GT(noseOnScore, tailOnScore)
        << "Nose-on target (pointing at us) should have higher priority than tail-on";
}

TEST(FlightLeadTest, TargetPrioritySlowTargetHigherThanFast) {
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity slow = makeEntity(0, 10*6076, -15000, M_PI, 300);   // 300 ft/s
    DigiEntity fast = makeEntity(0, 10*6076, -15000, M_PI, 1000);  // 1000 ft/s

    double slowScore = TargetPriority(self, slow);
    double fastScore = TargetPriority(self, fast);
    EXPECT_GT(slowScore, fastScore)
        << "Slow target should have higher priority than fast";
}

TEST(FlightLeadTest, TargetPriorityCoAltHigherThanSeparated) {
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity coAlt = makeEntity(0, 10*6076, -15000, M_PI, 400);   // same alt
    DigiEntity highAlt = makeEntity(0, 10*6076, -35000, M_PI, 400); // 20k ft higher

    double coAltScore = TargetPriority(self, coAlt);
    double highAltScore = TargetPriority(self, highAlt);
    EXPECT_GT(coAltScore, highAltScore)
        << "Co-altitude target should have higher priority";
}

// ===========================================================================
// ShouldEngage tests
// ===========================================================================

TEST(FlightLeadTest, ShouldEngageWithinRange) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);  // 10 NM
    EXPECT_TRUE(ShouldEngage(digi, self, target));
}

TEST(FlightLeadTest, ShouldNotEngageOutOfRange) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 100*6076, -15000, M_PI, 400);  // 100 NM
    EXPECT_FALSE(ShouldEngage(digi, self, target));
}

TEST(FlightLeadTest, ShouldNotEngageBingoFuel) {
    DigiState digi{};
    digi.fuel.fuelLbs = 1400;  // below bingo
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_FALSE(ShouldEngage(digi, self, target));
}

TEST(FlightLeadTest, ShouldNotEngageHeavyDamage) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 0.2;  // heavily damaged
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_FALSE(ShouldEngage(digi, self, target));
}

TEST(FlightLeadTest, ShouldNotEngageDeadTarget) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    target.isDead = true;
    EXPECT_FALSE(ShouldEngage(digi, self, target));
}

// ===========================================================================
// ShouldDisengage tests
// ===========================================================================

TEST(FlightLeadTest, ShouldDisengageBingoFuel) {
    DigiState digi{};
    digi.fuel.fuelLbs = 1400;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_TRUE(ShouldDisengage(digi, self, &target));
}

TEST(FlightLeadTest, ShouldDisengageHeavyDamage) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 0.4;  // < 0.5
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_TRUE(ShouldDisengage(digi, self, &target));
}

TEST(FlightLeadTest, ShouldDisengageWinchester) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    digi.fuel.winchester = true;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_TRUE(ShouldDisengage(digi, self, &target));
}

TEST(FlightLeadTest, ShouldDisengageTargetEscaped) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 100*6076, -15000, M_PI, 400);  // 100 NM
    EXPECT_TRUE(ShouldDisengage(digi, self, &target));
}

TEST(FlightLeadTest, ShouldDisengageNullTarget) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    EXPECT_TRUE(ShouldDisengage(digi, self, nullptr));
}

TEST(FlightLeadTest, ShouldNotDisengageHealthy) {
    DigiState digi{};
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_FALSE(ShouldDisengage(digi, self, &target));
}

// ===========================================================================
// ShouldRejoin tests
// ===========================================================================

TEST(FlightLeadTest, ShouldRejoinNoTarget) {
    DigiState digi{};
    EXPECT_TRUE(ShouldRejoin(digi, nullptr));
}

TEST(FlightLeadTest, ShouldRejoinDeadTarget) {
    DigiState digi{};
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    target.isDead = true;
    EXPECT_TRUE(ShouldRejoin(digi, &target));
}

TEST(FlightLeadTest, ShouldNotRejoinActiveTarget) {
    DigiState digi{};
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    EXPECT_FALSE(ShouldRejoin(digi, &target));
}

TEST(FlightLeadTest, ShouldNotRejoinWithMissileThreat) {
    DigiState digi{};
    // Simulate an incoming missile (non-null pointer — we just need the
    // pointer to be set; FlightLeadDecisions checks for null)
    digi.missileDefeat.incomingMissile = reinterpret_cast<const DigiEntity*>(0x1);
    EXPECT_FALSE(ShouldRejoin(digi, nullptr));
    digi.missileDefeat.incomingMissile = nullptr;  // cleanup
}

// ===========================================================================
// FlightLeadDecisions integration tests
// ===========================================================================

TEST(FlightLeadTest, FlightLeadDecisionsNoOpForWingman) {
    // Wingmen (isWing=true) should not make lead decisions.
    DigiState digi{};
    digi.formation.isWing = true;
    digi.formation.vehicleInUnit = 1;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    SensorPicture pic{};

    // Should not crash and should not modify state.
    FlightLeadDecisions(digi, self, &target, pic, 0.1);
    // saidRTB should not be set (wingmen don't signal RTB)
    EXPECT_FALSE(digi.damage.saidRTB);
}

TEST(FlightLeadTest, FlightLeadDecisionsSignalsRTBOnBingo) {
    // A flight lead at bingo fuel should signal RTB.
    DigiState digi{};
    digi.formation.isWing = false;
    digi.formation.vehicleInUnit = 0;
    digi.fuel.fuelLbs = 1400;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    SensorPicture pic{};

    FlightLeadDecisions(digi, self, &target, pic, 0.1);
    EXPECT_TRUE(digi.damage.saidRTB)
        << "Lead at bingo fuel should signal RTB";
}

TEST(FlightLeadTest, FlightLeadDecisionsNoRTBWhenHealthy) {
    // A healthy flight lead with a target should not signal RTB.
    DigiState digi{};
    digi.formation.isWing = false;
    digi.formation.vehicleInUnit = 0;
    digi.fuel.fuelLbs = 5000;
    digi.fuel.bingoFuelLbs = 1500;
    digi.damage.pctStrength = 1.0;
    DigiEntity self = makeEntity(0, 0, -15000, 0, 500);
    DigiEntity target = makeEntity(0, 10*6076, -15000, M_PI, 400);
    SensorPicture pic{};

    FlightLeadDecisions(digi, self, &target, pic, 0.1);
    EXPECT_FALSE(digi.damage.saidRTB);
}
