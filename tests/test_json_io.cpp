// f4flight unit tests - JSON I/O
#include "f4flight/json_io.h"
#include "f4flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(JsonIoTest, RoundtripF16C) {
    AircraftConfig original = config::makeF16CConfig();
    std::string json = json::write(original);
    ASSERT_FALSE(json.empty());

    AircraftConfig loaded;
    auto result = json::read(json, loaded);
    ASSERT_TRUE(result.ok);

    // Verify key fields round-trip
    EXPECT_EQ(loaded.name, original.name);
    EXPECT_NEAR(loaded.geometry.emptyWeight_lbs, original.geometry.emptyWeight_lbs, 1e-6);
    EXPECT_NEAR(loaded.geometry.area_ft2, original.geometry.area_ft2, 1e-6);
    EXPECT_NEAR(loaded.geometry.span_ft, original.geometry.span_ft, 1e-6);
    EXPECT_NEAR(loaded.geometry.maxGs, original.geometry.maxGs, 1e-6);
    EXPECT_NEAR(loaded.aux.normSpoolRate, original.aux.normSpoolRate, 1e-6);
    EXPECT_EQ(loaded.aero.mach.size(), original.aero.mach.size());
    EXPECT_EQ(loaded.aero.alpha_deg.size(), original.aero.alpha_deg.size());
    EXPECT_EQ(loaded.aero.clift.size(), original.aero.clift.size());
    EXPECT_EQ(loaded.engine.alt_ft.size(), original.engine.alt_ft.size());
    EXPECT_EQ(loaded.engine.mach.size(), original.engine.mach.size());
    EXPECT_EQ(loaded.engine.thrust_mil.size(), original.engine.thrust_mil.size());
    EXPECT_EQ(loaded.rollCmd.alpha_deg.size(), original.rollCmd.alpha_deg.size());
}

TEST(JsonIoTest, RoundtripAeroTableValues) {
    AircraftConfig original = config::makeF16CConfig();
    std::string json = json::write(original);
    AircraftConfig loaded;
    ASSERT_TRUE(json::read(json, loaded).ok);

    // Check a few specific table values
    ASSERT_FALSE(original.aero.clift.empty());
    EXPECT_NEAR(loaded.aero.clift[0], original.aero.clift[0], 1e-9);
    EXPECT_NEAR(loaded.aero.clift[original.aero.clift.size() / 2],
                original.aero.clift[original.aero.clift.size() / 2], 1e-9);
    EXPECT_NEAR(loaded.aero.clift.back(), original.aero.clift.back(), 1e-9);
}

TEST(JsonIoTest, RoundtripEngineTables) {
    AircraftConfig original = config::makeF16CConfig();
    std::string json = json::write(original);
    AircraftConfig loaded;
    ASSERT_TRUE(json::read(json, loaded).ok);

    ASSERT_FALSE(original.engine.thrust_mil.empty());
    EXPECT_NEAR(loaded.engine.thrust_mil[0], original.engine.thrust_mil[0], 1e-9);
    EXPECT_NEAR(loaded.engine.thrust_ab[0], original.engine.thrust_ab[0], 1e-9);
}

TEST(JsonIoTest, RoundtripLimiters) {
    AircraftConfig original = config::makeF16CConfig();
    std::string json = json::write(original);
    AircraftConfig loaded;
    ASSERT_TRUE(json::read(json, loaded).ok);

    const auto& origNegG = original.limiters[static_cast<int>(LimiterKey::NegGLimiter)];
    const auto& loadedNegG = loaded.limiters[static_cast<int>(LimiterKey::NegGLimiter)];
    EXPECT_EQ(loadedNegG.type, origNegG.type);
    EXPECT_NEAR(loadedNegG.x1, origNegG.x1, 1e-9);
    EXPECT_NEAR(loadedNegG.y1, origNegG.y1, 1e-9);
    EXPECT_NEAR(loadedNegG.x2, origNegG.x2, 1e-9);
    EXPECT_NEAR(loadedNegG.y2, origNegG.y2, 1e-9);
}

TEST(JsonIoTest, RoundtripGearPoints) {
    AircraftConfig original = config::makeF16CConfig();
    std::string json = json::write(original);
    AircraftConfig loaded;
    ASSERT_TRUE(json::read(json, loaded).ok);

    ASSERT_EQ(loaded.geometry.gear.size(), original.geometry.gear.size());
    for (std::size_t i = 0; i < original.geometry.gear.size(); ++i) {
        EXPECT_NEAR(loaded.geometry.gear[i].x, original.geometry.gear[i].x, 1e-9);
        EXPECT_NEAR(loaded.geometry.gear[i].y, original.geometry.gear[i].y, 1e-9);
        EXPECT_NEAR(loaded.geometry.gear[i].z, original.geometry.gear[i].z, 1e-9);
    }
}

TEST(JsonIoTest, InvalidJsonReturnsError) {
    AircraftConfig cfg;
    auto result = json::read("{invalid json}", cfg);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
}

TEST(JsonIoTest, EmptyJsonObjectOk) {
    AircraftConfig cfg;
    auto result = json::read("{}", cfg);
    EXPECT_TRUE(result.ok);
    // Empty JSON leaves defaults (0.0 for numeric fields)
    EXPECT_DOUBLE_EQ(cfg.geometry.emptyWeight_lbs, 0.0);
}
