// f4flight unit tests - equations of motion
#include "f4flight/eom.h"
#include "f4flight/config/f16c_config.h"
#include "f4flight/core/trig.h"
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

// ===========================================================================
// Refactor 4: recomputeKinematicTrig() helper
// ===========================================================================
TEST_F(EomTest, RecomputeKinematicTrigFillsAllFields) {
    KinematicState k;
    k.psi   = 0.5;
    k.theta = 0.3;
    k.phi   = 0.2;

    const double alpha_deg = 5.0;
    const double beta_deg  = 2.0;

    recomputeKinematicTrig(k, alpha_deg, beta_deg);

    // Aero-angle trig
    EXPECT_NEAR(k.sinalp, std::sin(5.0 * DTR), 1e-12);
    EXPECT_NEAR(k.cosalp, std::cos(5.0 * DTR), 1e-12);
    EXPECT_NEAR(k.sinbet, std::sin(2.0 * DTR), 1e-12);
    EXPECT_NEAR(k.cosbet, std::cos(2.0 * DTR), 1e-12);

    // Body euler trig
    EXPECT_NEAR(k.sinpsi, std::sin(0.5), 1e-12);
    EXPECT_NEAR(k.cospsi, std::cos(0.5), 1e-12);
    EXPECT_NEAR(k.sinthe, std::sin(0.3), 1e-12);
    EXPECT_NEAR(k.costhe, std::cos(0.3), 1e-12);
    EXPECT_NEAR(k.sinphi, std::sin(0.2), 1e-12);
    EXPECT_NEAR(k.cosphi, std::cos(0.2), 1e-12);

    // Velocity-vector euler angles -- MATCH FREEFALCON.
    // FreeFalcon eom.cpp:855-857 extracts sigma/gamma/mu directly from the
    // body quaternion, which (after the (e1,e2,e3,e4)=(qw,qz,qy,qx) swap)
    // is just the standard ZYX extraction. So:
    //   gamma = theta       (NOT theta - alpha*cos(phi) -- that was the old,
    //                         physically-more-correct formula that did not
    //                         match FreeFalcon and caused EOM divergence)
    //   sigma = psi
    //   mu    = phi
    const double expectedGamma = 0.3;
    EXPECT_NEAR(k.gmma,    expectedGamma,            1e-12);
    EXPECT_NEAR(k.singam,  std::sin(expectedGamma),  1e-12);
    EXPECT_NEAR(k.cosgam,  std::cos(expectedGamma),  1e-12);

    EXPECT_NEAR(k.sigma,   0.5,                  1e-12);
    EXPECT_NEAR(k.sinsig,  std::sin(0.5),        1e-12);
    EXPECT_NEAR(k.cossig,  std::cos(0.5),        1e-12);

    EXPECT_NEAR(k.mu,      0.2,                  1e-12);
    EXPECT_NEAR(k.sinmu,   std::sin(0.2),        1e-12);
    EXPECT_NEAR(k.cosmu,   std::cos(0.2),        1e-12);
}

TEST_F(EomTest, RecomputeKinematicTrigMatchesUpdatePath) {
    // The shared helper and the EOM::update() path must agree on every trig
    // field. To compare them on the SAME inputs, we use dt=0.0 so the
    // integration step does not modify psi/theta/phi (the body-rate filters
    // and position/velocity integrators are all multiplied by dt, so with
    // dt=0 the kinematic state is unchanged and trigonometry() fills the
    // trig fields from the same angles the helper saw).
    AircraftState sUpdate;
    sUpdate.kin.psi = 0.4;
    sUpdate.kin.theta = 0.1;
    sUpdate.kin.phi = -0.2;
    sUpdate.kin.vt = 100.0;
    sUpdate.kin.xdot = 100.0;
    sUpdate.aero.alpha_deg = 4.0;
    sUpdate.aero.beta_deg = 1.5;

    PilotInput input;
    sUpdate.gear.inAir = true;
    sUpdate.aero.xwaero = 0.0;
    sUpdate.aero.gearPos = 0.0;
    sUpdate.fcs.pstab = 0.0;
    sUpdate.loads.nzcgs = 1.0;
    sUpdate.loads.nycgw = 0.0;
    sUpdate.kin.cosmu = 1.0; sUpdate.kin.cosgam = 1.0; sUpdate.kin.singam = 0.0;
    sUpdate.kin.cosbet = 1.0; sUpdate.kin.cosalp = 1.0; sUpdate.kin.sinalp = 0.0;
    sUpdate.qsom = 100.0;
    sUpdate.aero.cnalpha = 1.0;
    sUpdate.kin.quat = quatFromEuler(0.4, 0.1, -0.2);

    // Snapshot the pre-update helper output.
    KinematicState kHelper = sUpdate.kin;
    recomputeKinematicTrig(kHelper, 4.0, 1.5);

    // Run the EOM with dt=0 so the euler angles don't move.
    eom_.update(0.0, input, sUpdate);

    // Every trig field should now match the helper's output exactly.
    // This guards against accidental divergence if someone later inlines
    // the math back into eom.cpp.
    EXPECT_NEAR(sUpdate.kin.sinalp, kHelper.sinalp, 1e-12);
    EXPECT_NEAR(sUpdate.kin.cosalp, kHelper.cosalp, 1e-12);
    EXPECT_NEAR(sUpdate.kin.sinbet, kHelper.sinbet, 1e-12);
    EXPECT_NEAR(sUpdate.kin.cosbet, kHelper.cosbet, 1e-12);
    EXPECT_NEAR(sUpdate.kin.sinpsi, kHelper.sinpsi, 1e-12);
    EXPECT_NEAR(sUpdate.kin.cospsi, kHelper.cospsi, 1e-12);
    EXPECT_NEAR(sUpdate.kin.sinthe, kHelper.sinthe, 1e-12);
    EXPECT_NEAR(sUpdate.kin.costhe, kHelper.costhe, 1e-12);
    EXPECT_NEAR(sUpdate.kin.sinphi, kHelper.sinphi, 1e-12);
    EXPECT_NEAR(sUpdate.kin.cosphi, kHelper.cosphi, 1e-12);
    EXPECT_NEAR(sUpdate.kin.gmma,   kHelper.gmma,   1e-12);
    EXPECT_NEAR(sUpdate.kin.singam, kHelper.singam, 1e-12);
    EXPECT_NEAR(sUpdate.kin.cosgam, kHelper.cosgam, 1e-12);
    EXPECT_NEAR(sUpdate.kin.sigma,  kHelper.sigma,  1e-12);
    EXPECT_NEAR(sUpdate.kin.mu,     kHelper.mu,     1e-12);
    EXPECT_NEAR(sUpdate.kin.sinmu,  kHelper.sinmu,  1e-12);
    EXPECT_NEAR(sUpdate.kin.cosmu,  kHelper.cosmu,  1e-12);
}
