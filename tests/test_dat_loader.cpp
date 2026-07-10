// f4flight unit tests - dat loader
#include "f4flight/dat_loader.h"
#include <gtest/gtest.h>
#include <fstream>

using namespace f4flight;

// A minimal .dat file for testing the parser. It contains just enough data
// to exercise the positional input-data parser and the CL table.
static const char* MINIMAL_DAT =
    "# Title: Test Aircraft\n"
    "# Author: Test\n"
    "#-----------------------------------------------------\n"
    "         +19900.000000  # Empty Weight (lbs)\n"
    "           +300.000000  # Reference Area\n"
    "          +7162.000000  # Internal Fuel\n"
    "            +40.000000  # AOA Max\n"
    "             -8.000000  # AOA Min\n"
    "            +30.000000  # Beta Max\n"
    "            -30.000000  # Beta Min\n"
    "             +9.000000  # Max G\n"
    "           +190.000000  # Max Roll\n"
    "           +250.000000  # Min Vcas\n"
    "           +850.000000  # Max Vcas\n"
    "           +420.000000  # Corner Vcas\n"
    "            +13.000000  # Theta Max\n"
    "             +3.000000  # Num Gear\n"
    "         +16.50         +0.00         +5.88         +90.00\n"
    "         +30.00         -3.88         +5.88         +90.00\n"
    "         +30.00         +3.88         +5.88         +90.00\n"
    "            +27.000000  # CG Loc\n"
    "            +47.000000  # Length\n"
    "            +32.000000  # Span\n"
    "             +2.500000  # Fus Radius\n"
    "             +4.500000  # Tail Ht\n"
    "aeropt AdvancedTEF\n"
    "+2.000000  # Num MACH\n"
    "  +0.000000  +1.000000\n"
    "+3.000000  # Num Alpha\n"
    "  -4.000000  +0.000000  +10.000000\n"
    "+1.000000   # Table Multiplier\n"
    "  -0.5  +0.0  +0.7\n"
    "  -0.5  +0.0  +0.6\n"
    "+1.000000   # CD Table Multiplier\n"
    "  +0.02  +0.02  +0.05\n"
    "  +0.03  +0.03  +0.06\n"
    "+1.000000   # CY Table Multiplier\n"
    "  -0.3  +0.0  +0.3\n"
    "  -0.3  +0.0  +0.3\n"
    "engopt fuelflow\n"
    "      +1.000000   # Thrust multiplier\n"
    "      +1.000000   # Fuel Flow Multiplier\n"
    "+2.000000  # Number of Mach Breaks\n"
    "  +0.000000  +1.000000\n"
    "+2.000000  # Number of Alt Break Points\n"
    "  +0.000000 +30000.000000\n"
    "+2.000000  # Number of Alpha Break Points\n"
    " -10.000000  +10.000000\n"
    "# Thrust alpha factor (alt=0)\n"
    "  +1.000000  +1.000000\n"
    "# Thrust alpha factor (alt=30000)\n"
    "  +1.000000  +1.000000\n"
    "# Thrust idle (alt=0)\n"
    "  +500.000000  +600.000000\n"
    "# Thrust idle (alt=30000)\n"
    "  +200.000000  +300.000000\n"
    "# Thrust mil (alt=0)\n"
    "  +14000.000000  +15000.000000\n"
    "# Thrust mil (alt=30000)\n"
    "  +5000.000000  +6000.000000\n"
    "# Thrust AB (alt=0)\n"
    "  +24000.000000  +28000.000000\n"
    "# Thrust AB (alt=30000)\n"
    "  +10000.000000  +12000.000000\n"
    "# Fuelflow idle (alt=0)\n"
    "  +800.000000  +900.000000\n"
    "# Fuelflow idle (alt=30000)\n"
    "  +800.000000  +800.000000\n"
    "# Fuelflow mil (alt=0)\n"
    "  +10000.000000  +12000.000000\n"
    "# Fuelflow mil (alt=30000)\n"
    "  +4000.000000  +5000.000000\n"
    "# Fuelflow AB (alt=0)\n"
    "  +50000.000000  +60000.000000\n"
    "# Fuelflow AB (alt=30000)\n"
    "  +20000.000000  +25000.000000\n"
    "7 # Num ALPHA\n"
    "   -10   0   10   20   25   30   90\n"
    "7 # DYNAMIC PRESSURE BREAKPOINTS\n"
    "   0   100   200   300   400   500   2000\n"
    "1   # Table Multiplier\n"
    " 0.00 171.82 243.41 257.73 257.73 257.73 229.09\n"
    " 0.00 214.77 286.36 300.68 315.00 315.00 286.36\n"
    " 0.00 171.82 243.41 257.73 257.73 257.73 229.09\n"
    " 0.00 71.59 143.18 157.50 157.50 157.50 128.86\n"
    " 0.00 60.00 93.07 107.39 107.39 107.39 107.39\n"
    " 0.00 30.00 42.95 57.27 57.27 57.27 28.64\n"
    " 0.00 0.00 0.00 0.00 0.00 0.00 0.00\n"
    "17   # Number of limiters\n"
    "0 0     250.0     -3.0     100.0     0.0\n"
    "3 1     15.0     9.0     20.4     7.3     25.2     1.0\n"
    "0 2     15.0     1.0     25.0     1.0\n"
    "0 3     14.0     1.0     26.0     0.0\n"
    "0 4     20.0     1.0     360.0     0.0\n"
    "0 5     350.0     18.0     420.0     15.0\n"
    "1 6     16.0\n"
    "2 7     0.6\n"
    "0 8     3.0     1.0     15.0     0.0\n"
    "0 9     20.0     1.0     180.0     0.0\n"
    "3 10     0.00     0.30     15.00     0.85     50.00     1.00\n"
    "3 11     0.00     0.60     15.00     0.85     50.00     1.00\n"
    "1 12     15\n"
    "3 13     0.01     0.10     40.00     0.80     60.00     1.00\n"
    "0 14     0.9     0.000100     1.0     0.000283\n"
    "1 16     6.5\n"
    "1 17     25.2\n"
    "typeEngine 5\n"
    "nEngines 1\n"
    "normSpoolRate 3.0\n"
    "abSpoolRate 10.5\n"
    "pitchMomentum 1.30\n"
    "CDSPDBFactor 0.034\n"
    "CDLDGFactor 0.060\n";

TEST(DatLoaderTest, ParsesMinimalDat) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
}

TEST(DatLoaderTest, TitleExtracted) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.name, "Test Aircraft");
}

TEST(DatLoaderTest, GeometryParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_NEAR(result.config.geometry.emptyWeight_lbs, 19900.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.area_ft2, 300.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.internalFuel_lbs, 7162.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.aoaMax_deg, 40.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.aoaMin_deg, -8.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.maxGs, 9.0, 1e-6);
    EXPECT_NEAR(result.config.geometry.span_ft, 32.0, 1e-6);
    EXPECT_EQ(result.config.geometry.gear.size(), 3u);
    EXPECT_NEAR(result.config.geometry.gear[0].x, 16.50, 1e-6);
    EXPECT_NEAR(result.config.geometry.gear[1].y, -3.88, 1e-6);
}

TEST(DatLoaderTest, AeroTableParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.aero.mach.size(), 2u);
    EXPECT_EQ(result.config.aero.alpha_deg.size(), 3u);
    EXPECT_EQ(result.config.aero.clift.size(), 6u);
    EXPECT_NEAR(result.config.aero.mach[0], 0.0, 1e-9);
    EXPECT_NEAR(result.config.aero.mach[1], 1.0, 1e-9);
    EXPECT_NEAR(result.config.aero.alpha_deg[0], -4.0, 1e-9);
    EXPECT_NEAR(result.config.aero.clift[0], -0.5, 1e-9);
    // CD should be scaled by 1.5x on read
    EXPECT_NEAR(result.config.aero.cdrag[0], 0.02 * 1.5, 1e-9);
}

TEST(DatLoaderTest, EngineTableParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.engine.alt_ft.size(), 2u);
    EXPECT_EQ(result.config.engine.mach.size(), 2u);
    EXPECT_EQ(result.config.engine.thrust_idle.size(), 4u);
    EXPECT_EQ(result.config.engine.thrust_mil.size(), 4u);
    EXPECT_EQ(result.config.engine.thrust_ab.size(), 4u);
    // Sea-level MIL thrust at mach=0
    EXPECT_NEAR(result.config.engine.thrust_mil[0], 14000.0, 1e-6);
    // Sea-level AB thrust at mach=0
    EXPECT_NEAR(result.config.engine.thrust_ab[0], 24000.0, 1e-6);
}

TEST(DatLoaderTest, HasAB) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_TRUE(result.config.engine.hasAB());
}

TEST(DatLoaderTest, FuelFlowTablesParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.engine.fuelflow_idle.size(), 4u);
    EXPECT_EQ(result.config.engine.fuelflow_mil.size(), 4u);
    EXPECT_EQ(result.config.engine.fuelflow_ab.size(), 4u);
    EXPECT_NEAR(result.config.engine.fuelflow_mil[0], 10000.0, 1e-6);
}

TEST(DatLoaderTest, RollTableParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.rollCmd.alpha_deg.size(), 7u);
    EXPECT_EQ(result.config.rollCmd.qbar.size(), 7u);
    EXPECT_EQ(result.config.rollCmd.rollRate.size(), 49u);
    EXPECT_NEAR(result.config.rollCmd.alpha_deg[0], -10.0, 1e-9);
    EXPECT_NEAR(result.config.rollCmd.qbar[1], 100.0, 1e-9);
}

TEST(DatLoaderTest, LimitersParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    // Neg G limiter (key 0, type Line)
    const auto& negG = result.config.limiters[static_cast<int>(LimiterKey::NegGLimiter)];
    EXPECT_EQ(negG.type, LimiterType::Line);
    EXPECT_NEAR(negG.x1, 250.0, 1e-6);
    EXPECT_NEAR(negG.y1, -3.0, 1e-6);

    // Pos G limiter (key 1, type ThreePoint)
    const auto& posG = result.config.limiters[static_cast<int>(LimiterKey::PosGLimiter)];
    EXPECT_EQ(posG.type, LimiterType::ThreePoint);

    // Cat III AOA limiter (key 6, type Value)
    const auto& cat3aoa = result.config.limiters[static_cast<int>(LimiterKey::CatIIIAOALimiter)];
    EXPECT_EQ(cat3aoa.type, LimiterType::Value);
    EXPECT_NEAR(cat3aoa.x1, 16.0, 1e-6);

    // Cat III Roll Rate limiter (key 7, type Percent)
    const auto& cat3roll = result.config.limiters[static_cast<int>(LimiterKey::CatIIIRollRateLimiter)];
    EXPECT_EQ(cat3roll.type, LimiterType::Percent);
    EXPECT_NEAR(cat3roll.x1, 0.6, 1e-6);
}

TEST(DatLoaderTest, AuxAeroParsed) {
    auto result = dat::loadString(MINIMAL_DAT, "test.dat");
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.config.aux.typeEngine, 5);
    EXPECT_EQ(result.config.aux.nEngines, 1);
    EXPECT_NEAR(result.config.aux.normSpoolRate, 3.0, 1e-6);
    EXPECT_NEAR(result.config.aux.abSpoolRate, 10.5, 1e-6);
    EXPECT_NEAR(result.config.aux.pitchMomentum, 1.30, 1e-6);
    EXPECT_NEAR(result.config.aux.CDSPDBFactor, 0.034, 1e-6);
    EXPECT_NEAR(result.config.aux.CDLDGFactor, 0.060, 1e-6);
}

TEST(DatLoaderTest, NonexistentFile) {
    auto result = dat::loadFile("/nonexistent/path/file.dat");
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
}
