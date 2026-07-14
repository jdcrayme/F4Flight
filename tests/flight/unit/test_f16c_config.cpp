// f4flight unit tests - F-16C configuration
#include "f4flight/flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(F16CConfigTest, Name) {
    auto cfg = config::makeF16CConfig();
    EXPECT_EQ(cfg.name, "F-16C Fighting Falcon");
}

TEST(F16CConfigTest, GeometryValues) {
    auto cfg = config::makeF16CConfig();
    EXPECT_NEAR(cfg.geometry.emptyWeight_lbs, 17400.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.area_ft2, 300.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.span_ft, 30.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.length_ft, 49.3, 1e-6);
    EXPECT_NEAR(cfg.geometry.internalFuel_lbs, 6990.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.maxGs, 9.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.aoaMax_deg, 25.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.aoaMin_deg, -5.0, 1e-6);
    EXPECT_NEAR(cfg.geometry.cornerVcas_kts, 330.0, 1e-6);
}

TEST(F16CConfigTest, GearPoints) {
    auto cfg = config::makeF16CConfig();
    EXPECT_EQ(cfg.geometry.gear.size(), 3u);
    // Nose gear at X ~ +11.5
    EXPECT_GT(cfg.geometry.gear[0].x, 0.0);
    // Main gear behind nose
    EXPECT_LT(cfg.geometry.gear[1].x, cfg.geometry.gear[0].x);
    EXPECT_LT(cfg.geometry.gear[2].x, cfg.geometry.gear[0].x);
    // Left main on +Y, right main on -Y
    EXPECT_GT(cfg.geometry.gear[1].y, 0.0);
    EXPECT_LT(cfg.geometry.gear[2].y, 0.0);
}

TEST(F16CConfigTest, AeroTableSizes) {
    auto cfg = config::makeF16CConfig();
    const auto& a = cfg.aero;
    EXPECT_EQ(a.mach.size(), 12u);
    EXPECT_EQ(a.alpha_deg.size(), 12u);
    EXPECT_EQ(a.clift.size(), 12u * 12u);
    EXPECT_EQ(a.cdrag.size(), 12u * 12u);
    EXPECT_EQ(a.cy.size(), 12u * 12u);
}

TEST(F16CConfigTest, EngineTableSizes) {
    auto cfg = config::makeF16CConfig();
    const auto& e = cfg.engine;
    EXPECT_EQ(e.alt_ft.size(), 11u);
    EXPECT_EQ(e.mach.size(), 12u);
    EXPECT_EQ(e.thrust_idle.size(), 11u * 12u);
    EXPECT_EQ(e.thrust_mil.size(), 11u * 12u);
    EXPECT_EQ(e.thrust_ab.size(), 11u * 12u);
    EXPECT_EQ(e.fuelflow_idle.size(), 11u * 12u);
}

TEST(F16CConfigTest, EngineHasAB) {
    auto cfg = config::makeF16CConfig();
    EXPECT_TRUE(cfg.engine.hasAB());
}

TEST(F16CConfigTest, EngineSeaLevelStaticThrust) {
    auto cfg = config::makeF16CConfig();
    const auto& e = cfg.engine;
    // At alt=0, mach=0 -> thrust should match the F100-PW-220 spec
    // (idle ~3000, mil ~14670, AB ~23770)
    EXPECT_NEAR(e.thrust_mil[0], 14670.0, 1.0);
    EXPECT_NEAR(e.thrust_ab[0], 23770.0, 1.0);
    EXPECT_GT(e.thrust_ab[0], e.thrust_mil[0]);
}

TEST(F16CConfigTest, RollCommandTable) {
    auto cfg = config::makeF16CConfig();
    EXPECT_FALSE(cfg.rollCmd.alpha_deg.empty());
    EXPECT_FALSE(cfg.rollCmd.qbar.empty());
    EXPECT_EQ(cfg.rollCmd.rollRate.size(),
              cfg.rollCmd.alpha_deg.size() * cfg.rollCmd.qbar.size());
}

TEST(F16CConfigTest, LimitersConfigured) {
    auto cfg = config::makeF16CConfig();
    // NegG limiter
    const auto& negG = cfg.limiters[static_cast<int>(LimiterKey::NegGLimiter)];
    EXPECT_EQ(negG.type, LimiterType::Line);

    // PosG limiter
    const auto& posG = cfg.limiters[static_cast<int>(LimiterKey::PosGLimiter)];
    EXPECT_EQ(posG.type, LimiterType::MinMax);

    // AOA limiter
    const auto& aoaL = cfg.limiters[static_cast<int>(LimiterKey::AOALimiter)];
    EXPECT_EQ(aoaL.type, LimiterType::MinMax);
}

TEST(F16CConfigTest, AuxAeroDefaults) {
    auto cfg = config::makeF16CConfig();
    EXPECT_EQ(cfg.aux.nEngines, 1);
    EXPECT_EQ(cfg.aux.typeEngine, 2);  // PW-220
    EXPECT_TRUE(cfg.aux.hasLef);
    EXPECT_TRUE(cfg.aux.hasTef);
    EXPECT_NEAR(cfg.aux.normSpoolRate, 0.7, 1e-6);
    EXPECT_NEAR(cfg.aux.abSpoolRate, 0.4, 1e-6);
}

TEST(F16CConfigTest, LookupTablesBuildable) {
    auto cfg = config::makeF16CConfig();
    // Should not throw
    auto cl = cfg.aero.makeClLookup();
    auto cd = cfg.aero.makeCdLookup();
    auto cy = cfg.aero.makeCyLookup();
    EXPECT_EQ(cl.sizeX(), 12u);
    EXPECT_EQ(cl.sizeY(), 12u);

    // Sample a value
    double v = cl(0.5, 5.0);
    EXPECT_GT(v, 0.0);  // CL at alpha=5, mach=0.5 should be positive
}
