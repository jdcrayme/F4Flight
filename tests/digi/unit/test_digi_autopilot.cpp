// f4flight unit tests - autopilot (autopilot/autopilot.h)
//
// Tests for:
//   - AutopilotMode enum and name lookup
//   - Autopilot mode + target setters/getters
//   - Autopilot::update dispatches correctly on each mode
//   - Off mode does nothing

#include "f4flight/digi/autopilot/autopilot.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"
#include <gtest/gtest.h>

using namespace f4flight::digi;

namespace {
// Helper: create a minimal AircraftState for a cruising aircraft.
f4flight::AircraftState makeCruiseState() {
    f4flight::AircraftState as{};
    as.kin.z = -15000.0;  // 15000 ft
    as.kin.vt = 400.0 * f4flight::KNOTS_TO_FTPSEC;
    as.vcas = 350.0;
    as.kin.sigma = 0.0;  // heading east
    as.kin.gmma = 0.0;
    as.kin.theta = 0.0;
    as.kin.phi = 0.0;
    as.kin.zdot = 0.0;
    as.vtDot = 0.0;
    as.kin.costhe = 1.0;
    as.kin.cosphi = 1.0;
    as.kin.cosgam = 1.0;
    as.kin.q = 0.0;
    as.kin.p = 0.0;
    as.aero.stallSpeed = 130.0;
    return as;
}
} // namespace

// ===========================================================================
// AutopilotMode enum tests
// ===========================================================================

TEST(AutopilotTest, ModeNamesAreCorrect) {
    EXPECT_STREQ(autopilotModeName(AutopilotMode::Off), "Off");
    EXPECT_STREQ(autopilotModeName(AutopilotMode::AltitudeHold), "AltitudeHold");
    EXPECT_STREQ(autopilotModeName(AutopilotMode::HeadingSelect), "HeadingSelect");
    EXPECT_STREQ(autopilotModeName(AutopilotMode::AltitudeSelect), "AltitudeSelect");
    EXPECT_STREQ(autopilotModeName(AutopilotMode::PitchRollHold), "PitchRollHold");
}

// ===========================================================================
// Autopilot setter/getter tests
// ===========================================================================

TEST(AutopilotTest, DefaultModeIsOff) {
    Autopilot ap;
    EXPECT_EQ(ap.mode(), AutopilotMode::Off);
}

TEST(AutopilotTest, SetMode) {
    Autopilot ap;
    ap.setMode(AutopilotMode::AltitudeHold);
    EXPECT_EQ(ap.mode(), AutopilotMode::AltitudeHold);
    ap.setMode(AutopilotMode::HeadingSelect);
    EXPECT_EQ(ap.mode(), AutopilotMode::HeadingSelect);
}

TEST(AutopilotTest, SetTargetAltitude) {
    Autopilot ap;
    ap.setTargetAltitude(20000.0);
    EXPECT_DOUBLE_EQ(ap.targetAltitude(), 20000.0);
}

TEST(AutopilotTest, SetTargetHeading) {
    Autopilot ap;
    ap.setTargetHeading(1.5);
    EXPECT_DOUBLE_EQ(ap.targetHeading(), 1.5);
}

TEST(AutopilotTest, SetTargetSpeed) {
    Autopilot ap;
    ap.setTargetSpeed(350.0);
    EXPECT_DOUBLE_EQ(ap.targetSpeed(), 350.0);
}

// ===========================================================================
// Autopilot::update dispatch tests
// ===========================================================================

TEST(AutopilotTest, OffModeDoesNothing) {
    Autopilot ap;
    ap.setMode(AutopilotMode::Off);

    DigiState digi;
    digi.nav.dt = 1.0 / 60.0;
    digi.config.maxGs = 9.0;
    digi.config.maxRoll = 30.0;
    digi.config.maxGammaDeg = 60.0;
    digi.config.cornerSpeed = 350.0;

    auto as = makeCruiseState();
    f4flight::FlightControlSystem fcs;
    f4flight::FcsState fcsState;

    // Set initial commands
    digi.commands.pStick = 0.0;
    digi.commands.rStick = 0.0;
    digi.commands.throttle = 0.5;

    ap.update(digi, as, fcs, fcsState, 1.0 / 60.0);

    // Off mode should not modify any commands
    EXPECT_DOUBLE_EQ(digi.commands.pStick, 0.0);
    EXPECT_DOUBLE_EQ(digi.commands.rStick, 0.0);
    EXPECT_DOUBLE_EQ(digi.commands.throttle, 0.5);
}

TEST(AutopilotTest, AltitudeHoldSetsFlightPhase) {
    Autopilot ap;
    ap.setMode(AutopilotMode::AltitudeHold);
    ap.setTargetAltitude(15000.0);
    ap.setTargetHeading(0.0);

    DigiState digi;
    digi.nav.dt = 1.0 / 60.0;
    digi.config.maxGs = 9.0;
    digi.config.maxRoll = 30.0;
    digi.config.maxGammaDeg = 60.0;
    digi.config.cornerSpeed = 350.0;
    digi.nav.flightPhase = FlightPhase::Combat;  // start in non-cruise

    auto as = makeCruiseState();
    f4flight::FlightControlSystem fcs;
    f4flight::FcsState fcsState;

    ap.update(digi, as, fcs, fcsState, 1.0 / 60.0);

    // Autopilot should set the flight phase to Cruise
    EXPECT_EQ(digi.nav.flightPhase, FlightPhase::Cruise);
}

TEST(AutopilotTest, AltitudeHoldGeneratesCommands) {
    Autopilot ap;
    ap.setMode(AutopilotMode::AltitudeHold);
    ap.setTargetAltitude(15000.0);  // at target altitude
    ap.setTargetHeading(0.0);       // heading east (current heading)

    DigiState digi;
    digi.nav.dt = 1.0 / 60.0;
    digi.config.maxGs = 9.0;
    digi.config.maxRoll = 30.0;
    digi.config.maxGammaDeg = 60.0;
    digi.config.cornerSpeed = 350.0;

    auto as = makeCruiseState();
    f4flight::FlightControlSystem fcs;
    f4flight::FcsState fcsState;

    ap.update(digi, as, fcs, fcsState, 1.0 / 60.0);

    // At target altitude + heading, the commands should be small
    // (the controller should be near steady-state).
    // Just verify the throttle was set (non-zero — holding speed).
    EXPECT_GT(digi.commands.throttle, 0.0);
}

TEST(AutopilotTest, AltitudeSelectUsesDampedGains) {
    Autopilot ap;
    ap.setMode(AutopilotMode::AltitudeSelect);
    ap.setTargetAltitude(20000.0);  // 5000 ft above current (15000)

    DigiState digi;
    digi.nav.dt = 1.0 / 60.0;
    digi.config.maxGs = 9.0;
    digi.config.maxRoll = 30.0;
    digi.config.maxGammaDeg = 60.0;
    digi.config.cornerSpeed = 350.0;

    auto as = makeCruiseState();
    f4flight::FlightControlSystem fcs;
    f4flight::FcsState fcsState;

    ap.update(digi, as, fcs, fcsState, 1.0 / 60.0);

    // Altitude select should command a climb (positive pstick)
    // when target is above current.
    EXPECT_GT(digi.commands.pStick, 0.0)
        << "AltitudeSelect should command nose-up when target is above";
}

TEST(AutopilotTest, PitchRollHoldCommandsWingsLevel) {
    Autopilot ap;
    ap.setMode(AutopilotMode::PitchRollHold);

    DigiState digi;
    digi.nav.dt = 1.0 / 60.0;
    digi.config.maxGs = 9.0;
    digi.config.maxRoll = 30.0;
    digi.config.maxGammaDeg = 60.0;
    digi.config.cornerSpeed = 350.0;

    auto as = makeCruiseState();
    as.kin.phi = 15.0 * f4flight::DTR;  // 15° right bank
    f4flight::FlightControlSystem fcs;
    f4flight::FcsState fcsState;

    ap.update(digi, as, fcs, fcsState, 1.0 / 60.0);

    // PitchRollHold should command wings level (negative rstick = roll left)
    EXPECT_LT(digi.commands.rStick, 0.0)
        << "PitchRollHold should command roll left when banked right";
}
