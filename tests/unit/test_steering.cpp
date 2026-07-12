// f4flight unit tests - steering (digi AI port)
// Basic tests for the DigiAI steering functions.
#include "f4flight/steering.h"
#include "f4flight/fcs.h"
#include "f4flight/core/constants.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;

TEST(SteeringUtilsTest, HeadingError) {
    EXPECT_NEAR(headingError(1.0, 0.0), 1.0, 1e-6);
    EXPECT_NEAR(headingError(0.0, 1.0), -1.0, 1e-6);
    // Wrap-around: 350° -> 10° = +20° = 0.349 rad
    EXPECT_NEAR(headingError(10.0 * DTR, 350.0 * DTR), 20.0 * DTR, 1e-6);
}

TEST(SteeringUtilsTest, TurnCompensatedG) {
    AircraftState state;
    state.kin.cosphi = 1.0;  // level
    EXPECT_NEAR(turnCompensatedG(state), 1.0, 1e-6);

    state.kin.cosphi = 0.707;  // 45° bank
    EXPECT_NEAR(turnCompensatedG(state), 1.414, 0.01);
}

TEST(DigiAITest, SetPstickGCommand) {
    DigiState digi;
    AircraftState state;
    state.kin.costhe = 1.0;  // level
    state.vcas = 350.0;      // normal speed

    // Command 1G (level) -> pstick should be ~0
    digi.pStick = 0.0;
    DigiAI::SetPstick(1.0, 9.0, DigiAI::GCommand, digi, state);
    // 1G maps to sqrt((1-1)/(9-1)) = 0
    EXPECT_NEAR(digi.pStick, 0.0, 0.01);

    // Command 4G -> pstick should be positive
    digi.pStick = 0.0;
    DigiAI::SetPstick(4.0, 9.0, DigiAI::GCommand, digi, state);
    EXPECT_GT(digi.pStick, 0.0);
    EXPECT_LT(digi.pStick, 1.0);

    // Command -2G -> pstick should be negative
    digi.pStick = 0.0;
    DigiAI::SetPstick(-2.0, 9.0, DigiAI::GCommand, digi, state);
    EXPECT_LT(digi.pStick, 0.0);
}

TEST(DigiAITest, SetPstickLowSpeed) {
    DigiState digi;
    AircraftState state;
    state.kin.costhe = 1.0;
    state.vcas = 200.0;  // low speed

    // At low speed, stick authority is reduced
    digi.pStick = 0.0;
    DigiAI::SetPstick(4.0, 9.0, DigiAI::GCommand, digi, state);
    double lowSpeedPstick = digi.pStick;

    state.vcas = 400.0;  // normal speed
    digi.pStick = 0.0;
    DigiAI::SetPstick(4.0, 9.0, DigiAI::GCommand, digi, state);
    double normalPstick = digi.pStick;

    EXPECT_LT(lowSpeedPstick, normalPstick);
}

TEST(DigiAITest, GammaHoldLevel) {
    DigiState digi;
    AircraftState state;
    state.kin.gmma = 0.0;      // level flight path
    state.kin.cosphi = 1.0;    // wings level
    state.kin.costhe = 1.0;
    state.vcas = 350.0;

    // Command 0° gamma (level) -> should command ~1G
    digi.pStick = 0.0;
    DigiAI::GammaHold(0.0, digi, state, 9.0);
    // pStick should be near 0 (1G = level)
    EXPECT_NEAR(digi.pStick, 0.0, 0.1);
}

TEST(DigiAITest, GammaHoldClimb) {
    DigiState digi;
    AircraftState state;
    state.kin.gmma = 0.0;
    state.kin.cosphi = 1.0;
    state.kin.costhe = 1.0;
    state.vcas = 350.0;

    // Command +5° gamma (climb) -> should command >1G
    digi.pStick = 0.0;
    DigiAI::GammaHold(5.0, digi, state, 9.0);
    EXPECT_GT(digi.pStick, 0.0);
}
