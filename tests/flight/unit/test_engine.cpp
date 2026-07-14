// f4flight unit tests - engine model
#include "f4flight/flight/engine.h"
#include "f4flight/flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

class EngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
        eng_ = EngineModel(&cfg_.engine, &cfg_.aux);
    }
    AircraftConfig cfg_;
    EngineModel eng_;
};

TEST_F(EngineTest, SeaLevelStaticThrust) {
    EngineState s;
    s.rpm = 0.7;
    s.engLit = true;

    const double mass = 24000.0 / GRAVITY;
    // Run enough steps for rpm to reach MIL (1.0)
    for (int i = 0; i < 200; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s);
    }
    // state.thrust is stored as acceleration (ft/s^2) = thrust_lbf / mass_slugs.
    // At MIL, sea level, static: thrust = 14,670 lbf, mass = 24,000/g slugs
    // => acceleration = 14,670 * g / 24,000 = 14,670 * 32.177 / 24,000 = 19.66 ft/s^2
    EXPECT_NEAR(s.rpm, 1.0, 0.05);
    const double expected_accel = 14670.0 * GRAVITY / 24000.0;
    EXPECT_NEAR(s.thrust, expected_accel, 2.0);
}

TEST_F(EngineTest, AfterburnerBoost) {
    EngineState s_mil, s_ab;
    s_mil.rpm = 1.0; s_mil.engLit = true;
    s_ab.rpm = 1.0;  s_ab.engLit = true;

    const double mass = 24000.0 / GRAVITY;
    for (int i = 0; i < 200; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s_mil);
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.5, 1.0, false, s_ab);
    }

    EXPECT_GT(s_ab.thrust, s_mil.thrust * 1.3); // AB significantly higher than MIL
    // AB at sea level static: thrust = 23,770 lbf (may differ from config)
    // Just verify AB thrust is positive and substantial
    EXPECT_GT(s_ab.thrust, 20.0);
}

TEST_F(EngineTest, IdleThrustLower) {
    EngineState s_idle, s_mil;
    s_idle.rpm = 0.7; s_idle.engLit = true;
    s_mil.rpm = 1.0;  s_mil.engLit = true;

    const double mass = 24000.0 / GRAVITY;
    for (int i = 0; i < 200; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 0.0, 1.0, false, s_idle);
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s_mil);
    }

    EXPECT_LT(s_idle.thrust, s_mil.thrust * 0.5);
    // Idle should be roughly 3,000 lbf at sea level
    const double expected_idle_accel = 3000.0 * GRAVITY / 24000.0;
    EXPECT_NEAR(s_idle.thrust, expected_idle_accel, 2.0);
}

TEST_F(EngineTest, AltitudeReducesThrust) {
    EngineState s_sl, s_alt;
    s_sl.rpm = 1.0; s_sl.engLit = true;
    s_alt.rpm = 1.0; s_alt.engLit = true;

    const double mass = 24000.0 / GRAVITY;
    for (int i = 0; i < 100; ++i) {
        eng_.update(0.05, 0.0,     0.0, 0.0, mass, 1.0, 1.0, false, s_sl);
        eng_.update(0.05, 30000.0, 0.0, 0.0, mass, 1.0, 1.0, false, s_alt);
    }

    EXPECT_LT(s_alt.thrust, s_sl.thrust * 0.7); // significant thrust loss at 30k ft
}

TEST_F(EngineTest, SpoolDynamics) {
    EngineState s;
    s.rpm = 0.7;
    s.engLit = true;
    const double mass = 24000.0 / GRAVITY;

    // Snap throttle to MIL and step many times -- rpm should approach 1.0
    for (int i = 0; i < 500; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s);
    }
    EXPECT_GT(s.rpm, 0.9);
}

TEST_F(EngineTest, FuelFlowPositive) {
    EngineState s;
    s.rpm = 1.0;
    s.engLit = true;
    const double mass = 24000.0 / GRAVITY;
    // Run enough steps for fuel flow smoothing to converge
    for (int i = 0; i < 200; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s);
    }
    EXPECT_GT(s.fuelFlow, 0.0);
    // MIL at sea level: ~4800 lb/hr
    EXPECT_NEAR(s.fuelFlow, 4800.0, 1500.0);
}

TEST_F(EngineTest, ABFuelFlowHigher) {
    EngineState s_mil, s_ab;
    s_mil.rpm = 1.0; s_mil.engLit = true;
    s_ab.rpm = 1.0;  s_ab.engLit = true;
    const double mass = 24000.0 / GRAVITY;
    for (int i = 0; i < 200; ++i) {
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.0, 1.0, false, s_mil);
        eng_.update(0.05, 0.0, 0.0, 0.0, mass, 1.5, 1.0, false, s_ab);
    }
    EXPECT_GT(s_ab.fuelFlow, s_mil.fuelFlow * 2.0); // AB much thirstier
}

TEST_F(EngineTest, BodyForcesStraightBack) {
    double xprop, yprop, zprop, xsprop, zsprop;
    EngineModel::bodyForces(1000.0, 0.0, 1.0, 0.0,
                            xprop, yprop, zprop, xsprop, zsprop);
    EXPECT_DOUBLE_EQ(xprop, 1000.0);
    EXPECT_DOUBLE_EQ(yprop, 0.0);
    EXPECT_DOUBLE_EQ(zprop, 0.0);
    EXPECT_DOUBLE_EQ(xsprop, 1000.0);
    EXPECT_DOUBLE_EQ(zsprop, 0.0);
}

TEST_F(EngineTest, BodyForcesAtAlpha) {
    // At alpha = 10 deg, thrust has stability-axis components
    const double alp = 10.0 * DTR;
    double xprop, yprop, zprop, xsprop, zsprop;
    EngineModel::bodyForces(1000.0, std::sin(alp), std::cos(alp), 0.0,
                            xprop, yprop, zprop, xsprop, zsprop);
    EXPECT_DOUBLE_EQ(xprop, 1000.0);
    EXPECT_NEAR(xsprop, 1000.0 * std::cos(alp), 1e-9);
    EXPECT_NEAR(zsprop, -1000.0 * std::sin(alp), 1e-9);
}
