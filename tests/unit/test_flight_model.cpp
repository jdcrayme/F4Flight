// f4flight unit tests - flight model integration
#include "f4flight/flight_model.h"
#include "f4flight/config/f16c_config.h"
#include <gtest/gtest.h>

#include <cstdio>
#include <limits>

using namespace f4flight;

class FlightModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
    }
    AircraftConfig cfg_;
};

TEST_F(FlightModelTest, InitAirborne) {
    FlightModel fm;
    fm.init(cfg_, 5000.0, 500.0, 0.0, true);
    EXPECT_NEAR(fm.state().kin.z, -5000.0, 1e-6);
    EXPECT_NEAR(fm.state().kin.vt, 500.0, 1e-6);
    EXPECT_TRUE(fm.state().gear.inAir);
    EXPECT_NEAR(fm.state().fuel.fuel_lbs, cfg_.geometry.internalFuel_lbs, 1e-6);
    EXPECT_GT(fm.state().fuel.mass_slugs, 0.0);
}

TEST_F(FlightModelTest, InitOnGround) {
    FlightModel fm;
    fm.init(cfg_, 0.0, 0.0, 0.0, false);
    EXPECT_FALSE(fm.state().gear.inAir);
    EXPECT_NEAR(fm.state().aero.gearPos, 1.0, 1e-6);
}

TEST_F(FlightModelTest, UpdateDoesNotCrash) {
    FlightModel fm;
    fm.init(cfg_, 5000.0, 500.0, 0.0, true);
    PilotInput input;
    input.throttle = 0.7;
    input.pstick = 0.0;
    input.rstick = 0.0;
    input.ypedal = 0.0;

    // Run a few seconds of simulation
    for (int i = 0; i < 100; ++i) {
        fm.update(0.1, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    // Should not have crashed; aircraft should still be moving
    EXPECT_GT(fm.state().kin.vt, 0.0);
}

TEST_F(FlightModelTest, FuelBurnsOverTime) {
    FlightModel fm;
    fm.init(cfg_, 5000.0, 500.0, 0.0, true);
    const double fuel0 = fm.state().fuel.fuel_lbs;

    PilotInput input;
    input.throttle = 1.0; // MIL
    for (int i = 0; i < 100; ++i) {
        fm.update(0.1, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    EXPECT_LT(fm.state().fuel.fuel_lbs, fuel0);
}

TEST_F(FlightModelTest, AtmosphereUpdatesWithAltitude) {
    FlightModel fm;
    fm.init(cfg_, 0.0, 500.0, 0.0, true);
    const double rho0 = fm.state().rho;

    // Move to altitude
    fm.state().kin.z = -30000.0;
    fm.update(0.1, PilotInput{}, 0.0, Vec3{0.0, 0.0, 1.0});
    EXPECT_LT(fm.state().rho, rho0 * 0.6);
}

TEST_F(FlightModelTest, FullForwardStickCausesPitchDown) {
    FlightModel fm;
    fm.init(cfg_, 10000.0, 400.0, 0.0, true);

    PilotInput input;
    input.throttle = 0.6;
    input.pstick = -1.0; // full forward -> nose down

    for (int i = 0; i < 100; ++i) {
        fm.update(0.05, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    // Pitch attitude should be decreasing (nose down)
    EXPECT_LT(fm.state().kin.theta, 0.0);
}

TEST_F(FlightModelTest, LevelFlightApproximatelyMaintainsAltitude) {
    FlightModel fm;
    fm.init(cfg_, 20000.0, 500.0, 0.0, true);

    // Trim-ish: small back stick, mid throttle
    PilotInput input;
    input.throttle = 0.65;
    input.pstick = 0.05;
    input.gearHandle = -1.0; // gear up

    const double alt0 = -fm.state().kin.z;
    for (int i = 0; i < 200; ++i) {
        fm.update(0.05, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    const double alt1 = -fm.state().kin.z;
    // Should be roughly maintaining altitude (within 5000 ft -- this is a
    // rough trim, not a converged steady state)
    EXPECT_NEAR(alt1, alt0, 5000.0);
}

TEST_F(FlightModelTest, ThrottleAffectsThrust) {
    FlightModel fm1, fm2;
    fm1.init(cfg_, 5000.0, 500.0, 0.0, true);
    fm2.init(cfg_, 5000.0, 500.0, 0.0, true);

    PilotInput idle, mil;
    idle.throttle = 0.0;
    mil.throttle = 1.0;

    // Run enough steps for spool dynamics to differentiate
    for (int i = 0; i < 100; ++i) {
        fm1.update(0.05, idle, 0.0, Vec3{0.0, 0.0, 1.0});
        fm2.update(0.05, mil, 0.0, Vec3{0.0, 0.0, 1.0});
    }

    EXPECT_LT(fm1.state().engine.thrust, fm2.state().engine.thrust);
}

// ===========================================================================
// Refactor 3: AircraftState::reset()
// ===========================================================================
TEST_F(FlightModelTest, StateResetClearsAllFields) {
    FlightModel fm;
    fm.init(cfg_, 5000.0, 500.0, 0.0, true);

    // Dirty the state with non-default values
    fm.state().kin.x = 1234.0;
    fm.state().kin.y = 5678.0;
    fm.state().kin.vt = 999.0;
    fm.state().aero.alpha_deg = 7.5;
    fm.state().fuel.fuel_lbs = 5000.0;
    fm.state().engine.rpm = 0.9;
    fm.state().rho = 0.002;

    // Reset
    fm.state().reset();

    // Every field should be back at its default-constructed value.
    EXPECT_DOUBLE_EQ(fm.state().kin.x, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().kin.y, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().kin.vt, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().aero.alpha_deg, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().fuel.fuel_lbs, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().engine.rpm, 0.0);
    EXPECT_DOUBLE_EQ(fm.state().rho, 0.0);
    // Gear wheels vector is cleared too (was sized by gear_.init).
    EXPECT_TRUE(fm.state().gear.wheels.empty());
}

TEST_F(FlightModelTest, StateResetThenReInitProducesSameStateAsFreshInit) {
    // A reset+init sequence should leave the aircraft in the same state as a
    // brand-new FlightModel that was only init'd once. This guards against
    // any hidden carry-over (filter states, lag histories, etc.).
    FlightModel fmFresh;
    fmFresh.init(cfg_, 15000.0, 500.0, 0.5, true);

    FlightModel fmReused;
    fmReused.init(cfg_, 5000.0, 400.0, 0.0, true);   // different ICs first
    fmReused.update(0.1, PilotInput{}, 0.0, Vec3{0,0,1}); // run a frame
    fmReused.state().reset();
    fmReused.init(cfg_, 15000.0, 500.0, 0.5, true);  // now reuse with new ICs

    EXPECT_NEAR(fmReused.state().kin.z, fmFresh.state().kin.z, 1e-9);
    EXPECT_NEAR(fmReused.state().kin.vt, fmFresh.state().kin.vt, 1e-9);
    EXPECT_NEAR(fmReused.state().kin.psi, fmFresh.state().kin.psi, 1e-9);
    EXPECT_NEAR(fmReused.state().fuel.fuel_lbs, fmFresh.state().fuel.fuel_lbs, 1e-9);
    EXPECT_NEAR(fmReused.state().fuel.mass_slugs, fmFresh.state().fuel.mass_slugs, 1e-9);
}

// ===========================================================================
// Refactor 2: AircraftConfig::validate()
// ===========================================================================
TEST_F(FlightModelTest, ValidateAcceptsGoodConfig) {
    auto report = cfg_.validate();
    if (!report.ok()) {
        std::printf("%s", report.format().c_str());
    }
    EXPECT_TRUE(report.ok());
    EXPECT_EQ(report.errorCount(), 0u);
}

TEST_F(FlightModelTest, ValidateRejectsEmptyAeroTables) {
    AircraftConfig bad = cfg_;
    bad.aero.mach.clear();
    bad.aero.alpha_deg.clear();
    bad.aero.clift.clear();
    auto report = bad.validate();
    EXPECT_FALSE(report.ok());
    EXPECT_GE(report.errorCount(), 1u);
}

TEST_F(FlightModelTest, ValidateRejectsTableSizeMismatch) {
    AircraftConfig bad = cfg_;
    // Add a stray element to clift so mach*alpha != clift.size()
    bad.aero.clift.push_back(0.0);
    auto report = bad.validate();
    EXPECT_FALSE(report.ok());
    // Should mention aero.clift in the report
    bool mentionsClift = false;
    for (const auto& i : report.issues) {
        if (i.field.find("aero.clift") != std::string::npos) {
            mentionsClift = true;
            break;
        }
    }
    EXPECT_TRUE(mentionsClift);
}

TEST_F(FlightModelTest, ValidateRejectsBackwardsAoALimits) {
    AircraftConfig bad = cfg_;
    bad.geometry.aoaMin_deg = 30.0;
    bad.geometry.aoaMax_deg = -5.0;
    auto report = bad.validate();
    EXPECT_FALSE(report.ok());
}

TEST_F(FlightModelTest, ValidateFormatProducesNonEmptyOutputForBadConfig) {
    AircraftConfig bad = cfg_;
    bad.aero.clift.clear();
    auto report = bad.validate();
    EXPECT_FALSE(report.format().empty());
}

TEST_F(FlightModelTest, ValidateRejectsNaNInGeometry) {
    AircraftConfig bad = cfg_;
    bad.geometry.area_ft2 = std::numeric_limits<double>::quiet_NaN();
    auto report = bad.validate();
    EXPECT_FALSE(report.ok());
}

// ===========================================================================
// Refactor 5: typed limiter accessors
// ===========================================================================
TEST_F(FlightModelTest, LimiterAccessorRoundTrips) {
    AircraftConfig cfg = cfg_;
    Limiter l;
    l.type = LimiterType::Value;
    l.x1 = 7.5;
    cfg.setLimiter(LimiterKey::AOALimiter, l);

    const Limiter& got = cfg.limiter(LimiterKey::AOALimiter);
    EXPECT_EQ(got.type, LimiterType::Value);
    EXPECT_DOUBLE_EQ(got.x1, 7.5);
    EXPECT_DOUBLE_EQ(got.limit(0.0), 7.5);  // Value limiter ignores input
}

TEST_F(FlightModelTest, LimiterAccessorMatchesArrayIndex) {
    // The typed accessor must return the same object as raw array indexing
    // for every key -- otherwise existing code that reads limiters[] would
    // silently start reading a different limiter.
    AircraftConfig cfg = cfg_;
    for (int i = 0; i < static_cast<int>(LimiterKey::Count); ++i) {
        const auto key = static_cast<LimiterKey>(i);
        EXPECT_EQ(&cfg.limiter(key), &cfg.limiters[i]);
    }
}

TEST_F(FlightModelTest, LimiterAccessorMutableAllowsInPlaceEdit) {
    AircraftConfig cfg = cfg_;
    cfg.limiter(LimiterKey::RollRateLimiter).type = LimiterType::MinMax;
    cfg.limiter(LimiterKey::RollRateLimiter).x1   = -2.0;
    cfg.limiter(LimiterKey::RollRateLimiter).x2   =  2.0;
    EXPECT_EQ(cfg.limiters[static_cast<int>(LimiterKey::RollRateLimiter)].type,
              LimiterType::MinMax);
    EXPECT_DOUBLE_EQ(cfg.limiter(LimiterKey::RollRateLimiter).limit(5.0), 2.0);
    EXPECT_DOUBLE_EQ(cfg.limiter(LimiterKey::RollRateLimiter).limit(-5.0), -2.0);
}

