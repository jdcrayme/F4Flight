// f4flight unit tests - FCS (flight control system)
#include "f4flight/flight/fcs.h"
#include "f4flight/flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

class FcsTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
        fcs_ = FlightControlSystem(&cfg_, &cfg_.geometry, &cfg_.aux);
    }
    AircraftConfig cfg_;
    FlightControlSystem fcs_;
};

TEST_F(FcsTest, LimiterNegG) {
    // The NegG limiter is a LineLimiter from (0 kts, -1 G) to (250 kts, -3 G).
    // It interpolates linearly between those endpoints and clamps outside.
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::NegGLimiter, 0.0),   -1.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::NegGLimiter, 125.0),  -2.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::NegGLimiter, 250.0),  -3.0, 1e-6);
    // Above 250 kts, clamped to -3
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::NegGLimiter, 300.0),  -3.0, 1e-6);
}

TEST_F(FcsTest, LimiterPosGMinMax) {
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::PosGLimiter, 100.0), 9.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::PosGLimiter, -100.0), -3.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::PosGLimiter, 5.0), 5.0, 1e-6);
}

TEST_F(FcsTest, LimiterAOA) {
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::AOALimiter, 30.0), 25.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::AOALimiter, -10.0), -5.0, 1e-6);
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::AOALimiter, 10.0), 10.0, 1e-6);
}

TEST_F(FcsTest, LimiterRollRate) {
    // Below 0 deg alpha: full authority (1.0)
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::RollRateLimiter, 0.0), 1.0, 1e-6);
    // At 25 deg: faded to 0.4
    EXPECT_NEAR(fcs_.applyLimiter(LimiterKey::RollRateLimiter, 25.0), 0.4, 1e-6);
}

TEST_F(FcsTest, UpdateProducesAlpha) {
    AircraftState state;
    PilotInput input;
    input.pstick = 0.5; // half back stick
    input.throttle = 0.7;
    state.aero.clift0 = 0.0;
    state.aero.clalph0 = 0.5;
    state.aero.clalpha = 0.5;
    state.aero.cnalpha = 0.5;
    state.aero.gearPos = 0.0;

    fcs_.update(0.1, 100.0, 100.0 * 300.0 / 700.0, 0.5, 500.0, 300.0,
                5.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, true, 1.0, 0.0,
                false, false, false, input, state.fcs, state.aero);
    // Pulling back should produce positive alpha command
    EXPECT_GT(state.fcs.aoacmd, 0.0);
    EXPECT_GT(state.aero.alpha_deg, 0.0);
}

TEST_F(FcsTest, RollRateCommanded) {
    AircraftState state;
    PilotInput input;
    input.rstick = 1.0; // full right stick
    state.aero.gearPos = 0.0;
    state.aero.clift0 = 0.0;
    state.aero.clalph0 = 0.5;
    state.aero.clalpha = 0.5;
    state.aero.cnalpha = 0.5;

    fcs_.update(0.1, 100.0, 100.0 * 300.0 / 700.0, 0.5, 500.0, 300.0,
                0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, true, 1.0, 0.0,
                false, false, false, input, state.fcs, state.aero);

    // Full right stick should produce a positive roll-rate command
    EXPECT_GT(state.fcs.pscmd, 0.0);
    // After lag filter, pstab should approach pscmd
    EXPECT_GT(state.fcs.pstab, 0.0);
}

TEST_F(FcsTest, StickShapingQuadratic) {
    AircraftState state;
    PilotInput input;
    input.pstick = 0.5;
    state.aero.gearPos = 0.0;
    state.aero.clift0 = 0.0;
    state.aero.clalph0 = 0.5;
    state.aero.clalpha = 0.5;
    state.aero.cnalpha = 0.5;

    fcs_.update(0.1, 100.0, 100.0 * 300.0 / 700.0, 0.5, 500.0, 300.0,
                5.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, true, 1.0, 0.0,
                false, false, false, input, state.fcs, state.aero);
    // pshape = stick^2 * sign = 0.5^2 = 0.25
    EXPECT_NEAR(state.fcs.pshape, 0.25, 1e-6);
}
