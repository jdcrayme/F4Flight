// f4flight unit tests - atmosphere model
#include "f4flight/flight/atmosphere.h"
#include "f4flight/flight/core/constants.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(AtmosphereTest, SeaLevelConditions) {
    double ttheta, rsigma;
    double pdelta = calcPressureRatio(0.0, ttheta, rsigma);
    EXPECT_NEAR(ttheta, 1.0, 1e-9);
    EXPECT_NEAR(rsigma, 1.0, 1e-9);
    EXPECT_NEAR(pdelta, 1.0, 1e-9);
}

TEST(AtmosphereTest, TroposphereDecreasingDensity) {
    double ttheta0, rsigma0, ttheta1, rsigma1;
    calcPressureRatio(0.0, ttheta0, rsigma0);
    calcPressureRatio(10000.0, ttheta1, rsigma1);
    EXPECT_LT(ttheta1, ttheta0);
    EXPECT_LT(rsigma1, rsigma0);
}

TEST(AtmosphereTest, Tropopause) {
    // At exactly the tropopause altitude (36089 ft), the troposphere formula
    // should match the lower-stratosphere constant temperature.
    double ttheta_t, rsigma_t;
    calcPressureRatio(TROPO_ALT_FT, ttheta_t, rsigma_t);
    EXPECT_NEAR(ttheta_t, STRATO_TTHETA, 1e-4);

    // Density ratio at the tropopause from the troposphere formula:
    //   rsigma = (1 - 6.875e-6 * 36089)^4.255876
    double expected = std::pow(1.0 - TROPO_LAPSE * TROPO_ALT_FT, TROPO_RHO_EXP);
    EXPECT_NEAR(rsigma_t, expected, 1e-4);
}

TEST(AtmosphereTest, LowerStratosphereConstantTemperature) {
    double ttheta1, rsigma1, ttheta2, rsigma2;
    calcPressureRatio(40000.0, ttheta1, rsigma1);
    calcPressureRatio(60000.0, ttheta2, rsigma2);
    // Temperature is constant in this layer
    EXPECT_NEAR(ttheta1, ttheta2, 1e-9);
    EXPECT_NEAR(ttheta1, STRATO_TTHETA, 1e-9);
    // Density continues to drop
    EXPECT_LT(rsigma2, rsigma1);
}

TEST(AtmosphereTest, UpperStratosphere) {
    double ttheta, rsigma;
    calcPressureRatio(70000.0, ttheta, rsigma);
    // Temperature rises again with altitude in this layer
    EXPECT_GT(ttheta, STRATO_TTHETA);
    // Density continues to drop
    EXPECT_LT(rsigma, 0.1);
}

TEST(AtmosphereTest, ComputeAtmosphereOutput) {
    auto a = computeAtmosphere(0.0, 500.0, 300.0, 500.0 / GRAVITY);
    EXPECT_NEAR(a.rho, RHOASL, 1e-9);
    EXPECT_NEAR(a.pa, PASL, 1e-3);
    EXPECT_NEAR(a.sound, AASL, 1e-9);
    EXPECT_NEAR(a.mach, 500.0 / AASL, 1e-9);
    EXPECT_NEAR(a.qbar, 0.5 * RHOASL * 500.0 * 500.0, 1e-6);
}

TEST(AtmosphereTest, QsomNormalization) {
    // qsom = q * S / m
    const double area = 300.0;
    const double mass = 500.0;
    const double vt   = 500.0;
    auto a = computeAtmosphere(0.0, vt, area, mass);
    const double expectedQ = 0.5 * RHOASL * vt * vt;
    const double expectedQsom = expectedQ * area / mass;
    EXPECT_NEAR(a.qsom, expectedQsom, 1e-6);
}

TEST(AtmosphereTest, KcasFromMachRoundtrip) {
    // For subsonic Mach, KCAS -> Mach -> KCAS should round-trip
    const double mach = 0.5;
    const double pa = PASL * 0.5; // arbitrary pressure
    double kcas = calcKcasFromMach(mach, pa);
    double mach2 = calcMachFromKcas(kcas, pa);
    EXPECT_NEAR(mach2, mach, 0.005);
}

TEST(AtmosphereTest, KcasAtSeaLevel) {
    // At sea level, KCAS = Mach * speed_of_sound (knots)
    const double mach = 0.5;
    double kcas = calcKcasFromMach(mach, PASL);
    EXPECT_NEAR(kcas, mach * AASLK, 1.0);
}
