// f4flight unit tests - aerodynamics
#include "f4flight/flight/aerodynamics.h"
#include "f4flight/flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

class AeroTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
        aero_ = Aerodynamics(&cfg_.aero, &cfg_.geometry, &cfg_.aux);
    }
    AircraftConfig cfg_;
    Aerodynamics aero_;
};

TEST_F(AeroTest, ZeroAlphaZeroLift) {
    AeroState s;
    s.gearPos = 0.0;
    aero_.update(0.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s);
    // At alpha=0, CL should be near zero (with F-16 wing, CL0 = 0)
    EXPECT_NEAR(s.cl, 0.0, 0.05);
    EXPECT_NEAR(s.lift, 0.0, 5.0);
}

TEST_F(AeroTest, PositiveAlphaProducesPositiveLift) {
    AeroState s1, s2;
    s1.gearPos = 0.0;
    s2.gearPos = 0.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s1);
    aero_.update(-5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s2);
    EXPECT_GT(s1.cl, 0.0);
    EXPECT_LT(s2.cl, 0.0);
    EXPECT_GT(s1.lift, 0.0);
    EXPECT_LT(s2.lift, 0.0);
}

TEST_F(AeroTest, GearDragAdded) {
    AeroState s_up, s_down;
    s_up.gearPos = 0.0;
    s_down.gearPos = 1.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s_up);
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s_down);
    // Gear down should add CDLDGFactor (0.06) to CD
    EXPECT_GT(s_down.cd, s_up.cd);
    EXPECT_NEAR(s_down.cd - s_up.cd, cfg_.aux.CDLDGFactor, 1e-6);
}

TEST_F(AeroTest, SpeedBrakeDragAdded) {
    AeroState s_off, s_on;
    s_off.dbrake = 0.0;  s_off.gearPos = 0.0;
    s_on.dbrake = 1.0;   s_on.gearPos = 0.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s_off);
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s_on);
    EXPECT_GT(s_on.cd, s_off.cd);
    EXPECT_NEAR(s_on.cd - s_off.cd, cfg_.aux.CDSPDBFactor, 1e-6);
}

TEST_F(AeroTest, BodyAxesFromLiftDrag) {
    AeroState s;
    s.gearPos = 0.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -5000.0, 250.0, 0.0, s);
    // At positive alpha, lift is up (+ in stability axis = -Z in body)
    // zaero should be negative (lift pushes aircraft up = -Z in body)
    EXPECT_LT(s.zaero, 0.0);
    // Drag is positive (it's a magnitude; the body-axis X force is -drag*cos(a))
    EXPECT_GT(s.drag, 0.0);
}

TEST_F(AeroTest, GroundEffectBoostsLift) {
    AeroState s_ground, s_altitude;
    s_ground.gearPos = 0.0;
    s_altitude.gearPos = 0.0;

    // Aircraft very low (within 0.2 * span = 6 ft of ground)
    // z = groundZ - 3 (3 ft AGL)
    const double groundZ = -5000.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, groundZ, groundZ - 3.0, 250.0, 0.0, s_ground);

    // Aircraft at altitude (well above span = 30 ft)
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, groundZ, groundZ - 100.0, 250.0, 0.0, s_altitude);

    EXPECT_GT(s_ground.cl, s_altitude.cl);
    // Boost factor is 1.13 -> 13% more lift
    EXPECT_NEAR(s_ground.cl / s_altitude.cl, 1.13, 0.01);
}

TEST_F(AeroTest, CLalphaSensible) {
    AeroState s;
    s.gearPos = 0.0;
    aero_.update(5.0, 0.0, 0.5, 500.0, 100.0, 100.0 * 300.0 / 500.0, 0.2,
                 5000.0, 0.0, -10000.0, 250.0, 0.0, s);
    // CL_alpha should be positive (lift increases with alpha)
    EXPECT_GT(s.clalpha, 0.0);
    // clalph0 is the static slope (per degree) computed from CL(0) to CL(10).
    // Typical F-16 CL_alpha ~ 0.075/deg at low alpha. Our table gives ~0.077.
    EXPECT_GT(s.clalph0, 0.05);
    EXPECT_LT(s.clalph0, 0.15);
}
