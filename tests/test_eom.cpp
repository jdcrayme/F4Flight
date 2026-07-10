// f4flight unit tests - equations of motion
#include "f4flight/eom.h"
#include "f4flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

class EomTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
        eom_ = EquationsOfMotion(&cfg_.geometry, &cfg_.aux);
    }
    AircraftConfig cfg_;
    EquationsOfMotion eom_;
};

TEST_F(EomTest, StationaryNoDrift) {
    AircraftState s;
    PilotInput input;
    // No forces, no rotation -> should stay still
    s.kin.vt = 0.0;
    s.kin.quat = Quaternion{1.0, 0.0, 0.0, 0.0};
    s.kin.xdot = 0.0; s.kin.ydot = 0.0; s.kin.zdot = 0.0;
    s.gear.inAir = true;
    s.aero.xwaero = 0.0;
    s.aero.gearPos = 0.0;
    s.fcs.pstab = 0.0;
    s.loads.nzcgs = 1.0;  // 1G so gravity is balanced (no descent)
    s.loads.nycgw = 0.0;
    s.kin.cosmu = 1.0; s.kin.cosgam = 1.0; s.kin.singam = 0.0;
    s.kin.cosbet = 1.0; s.kin.cosalp = 1.0; s.kin.sinalp = 0.0;
    s.qsom = 100.0;
    s.aero.cnalpha = 1.0;

    const double x0 = s.kin.x, y0 = s.kin.y, z0 = s.kin.z;
    eom_.update(0.1, input, s);
    // With 1G lift and zero velocity, position shouldn't drift significantly
    EXPECT_NEAR(s.kin.x, x0, 0.01);
    EXPECT_NEAR(s.kin.y, y0, 0.01);
    EXPECT_NEAR(s.kin.z, z0, 0.01);
}

TEST_F(EomTest, ForwardMotionIntegratesPosition) {
    AircraftState s;
    PilotInput input;
    s.kin.vt = 100.0;
    s.kin.xdot = 100.0;
    s.kin.ydot = 0.0;
    s.kin.zdot = 0.0;
    s.kin.quat = Quaternion{1.0, 0.0, 0.0, 0.0};
    s.gear.inAir = true;
    s.aero.xwaero = 0.0;
    s.aero.gearPos = 0.0;
    s.fcs.pstab = 0.0;
    s.loads.nzcgs = 1.0;
    s.loads.nycgw = 0.0;
    s.kin.cosmu = 1.0; s.kin.cosgam = 1.0; s.kin.singam = 0.0;
    s.kin.cosbet = 1.0; s.kin.cosalp = 1.0; s.kin.sinalp = 0.0;
    s.qsom = 100.0;
    s.aero.cnalpha = 1.0;

    const double x0 = s.kin.x;
    eom_.update(0.1, input, s);
    // Should have moved ~10 ft forward (100 ft/s * 0.1 s)
    EXPECT_NEAR(s.kin.x - x0, 10.0, 1e-6);
}

TEST_F(EomTest, QuaternionStaysNormalized) {
    AircraftState s;
    PilotInput input;
    s.kin.vt = 100.0;
    s.kin.quat = Quaternion{1.0, 0.0, 0.0, 0.0};
    s.kin.xdot = 100.0; s.kin.ydot = 0.0; s.kin.zdot = 0.0;
    s.gear.inAir = true;
    s.aero.xwaero = 0.0;
    s.aero.gearPos = 0.0;
    s.fcs.pstab = 0.1; // small roll command
    s.fcs.rollRateLag.y_prev = 0.0;
    s.loads.nzcgs = 1.0;
    s.loads.nycgw = 0.0;
    s.kin.cosmu = 1.0; s.kin.cosgam = 1.0; s.kin.singam = 0.0;
    s.kin.cosbet = 1.0; s.kin.cosalp = 1.0; s.kin.sinalp = 0.0;
    s.qsom = 100.0;
    s.aero.cnalpha = 1.0;

    for (int i = 0; i < 100; ++i) {
        eom_.update(0.01, input, s);
        EXPECT_NEAR(s.kin.quat.norm(), 1.0, 1e-9);
    }
}

TEST_F(EomTest, GravityPullsDown) {
    AircraftState s;
    PilotInput input;
    s.kin.vt = 100.0;
    s.kin.xdot = 100.0; s.kin.ydot = 0.0; s.kin.zdot = 0.0;
    s.kin.quat = Quaternion{1.0, 0.0, 0.0, 0.0};
    s.gear.inAir = true;
    s.aero.xwaero = 0.0;
    s.aero.gearPos = 0.0;
    s.fcs.pstab = 0.0;
    s.loads.nzcgs = 0.0;  // no lift
    s.loads.nycgw = 0.0;
    s.kin.cosmu = 1.0; s.kin.cosgam = 1.0; s.kin.singam = 0.0;
    s.kin.cosbet = 1.0; s.kin.cosalp = 1.0; s.kin.sinalp = 0.0;
    s.qsom = 100.0;
    s.aero.cnalpha = 1.0;

    // With no lift (NZ=0), gravity should eventually pull the aircraft down.
    // z is DOWN in our NED frame, so "descending" means z INCREASES.
    const double z0 = s.kin.z;
    for (int i = 0; i < 50; ++i) {
        eom_.update(0.05, input, s);
    }
    // After 2.5 seconds with no lift, aircraft should have descended (z > z0)
    EXPECT_GT(s.kin.z, z0);
}

TEST_F(EomTest, TrigonometryConsistency) {
    AircraftState s;
    s.kin.psi = 0.5;
    s.kin.theta = 0.3;
    s.kin.phi = 0.2;
    s.kin.xdot = 100.0; s.kin.ydot = 0.0; s.kin.zdot = 0.0;
    s.kin.vt = 100.0;
    s.aero.alpha_deg = 5.0;
    s.aero.beta_deg = 0.0;

    // Use a friend approach: trigger trigonometry via update
    PilotInput input;
    s.gear.inAir = true;
    s.aero.xwaero = 0.0;
    s.aero.gearPos = 0.0;
    s.fcs.pstab = 0.0;
    s.loads.nzcgs = 1.0;
    s.loads.nycgw = 0.0;
    s.kin.cosmu = 1.0; s.kin.cosgam = 1.0; s.kin.singam = 0.0;
    s.kin.cosbet = 1.0; s.kin.cosalp = 1.0; s.kin.sinalp = 0.0;
    s.qsom = 100.0;
    s.aero.cnalpha = 1.0;
    s.kin.quat = quatFromEuler(0.5, 0.3, 0.2);

    eom_.update(0.001, input, s);

    // After update, trig values should match the euler angles
    EXPECT_NEAR(s.kin.sinpsi, std::sin(0.5), 1e-6);
    EXPECT_NEAR(s.kin.cospsi, std::cos(0.5), 1e-6);
    EXPECT_NEAR(s.kin.sinthe, std::sin(0.3), 1e-6);
    EXPECT_NEAR(s.kin.costhe, std::cos(0.3), 1e-6);
    EXPECT_NEAR(s.kin.sinphi, std::sin(0.2), 1e-6);
    EXPECT_NEAR(s.kin.cosphi, std::cos(0.2), 1e-6);
}
