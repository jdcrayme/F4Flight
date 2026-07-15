// f4flight unit tests - gear model
#include "f4flight/flight/gear.h"
#include "f4flight/flight/config/f16c_config.h"
#include <gtest/gtest.h>

using namespace f4flight;

class GearTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_ = config::makeF16CConfig();
        gear_ = GearModel(&cfg_.geometry, &cfg_.aux);
    }
    AircraftConfig cfg_;
    GearModel gear_;
};

TEST_F(GearTest, InitCreatesWheels) {
    GearState s;
    gear_.init(s);
    EXPECT_EQ(s.wheels.size(), cfg_.geometry.gear.size());
    EXPECT_EQ(s.wheels.size(), 3u); // F-16 has 3 gear points
}

TEST_F(GearTest, MinHeightIsMaxStrutZ) {
    GearState s;
    gear_.init(s);
    const double h = gear_.computeMinHeight(s);
    // F-16 gear z = 3.0 (all three)
    EXPECT_NEAR(h, 3.0, 1e-6);
}

TEST_F(GearTest, FrictionRolling) {
    EXPECT_NEAR(GearModel::calcMuFric(false, false, false, true), 0.04, 1e-9);
}

TEST_F(GearTest, FrictionBrakes) {
    // Brake friction increased from 0.4 to 0.7 for realistic deceleration
    // (carbon brakes: 0.6-0.8 friction coefficient)
    EXPECT_NEAR(GearModel::calcMuFric(true, false, false, true), 0.7, 1e-9);
    EXPECT_NEAR(GearModel::calcMuFric(false, true, false, true), 0.7, 1e-9);
}

TEST_F(GearTest, FrictionCarrier) {
    EXPECT_NEAR(GearModel::calcMuFric(false, false, true, true), 20.0, 1e-9);
}

TEST_F(GearTest, FrictionGrass) {
    EXPECT_NEAR(GearModel::calcMuFric(false, false, false, false), 0.5, 1e-9);
}

TEST_F(GearTest, GearPosExtension) {
    double pos = 0.0; // gear up
    // Command gear down (+1), step enough times to fully extend (3 seconds)
    double t = 0.0;
    while (t < 5.0) {
        gear_.updateGearPos(pos, 1.0, 0.1);
        t += 0.1;
    }
    EXPECT_NEAR(pos, 1.0, 1e-6);
}

TEST_F(GearTest, GearPosRetraction) {
    double pos = 1.0; // gear down
    double t = 0.0;
    while (t < 5.0) {
        gear_.updateGearPos(pos, -1.0, 0.1);
        t += 0.1;
    }
    EXPECT_NEAR(pos, 0.0, 1e-6);
}

TEST_F(GearTest, StrutCompression) {
    GearState s;
    gear_.init(s);

    // Aircraft 1 ft above ground -- strut (3 ft extended) should compress ~2 ft
    const double groundZ = -1000.0;
    const double z = groundZ - 1.0; // 1 ft AGL (z is negative, NED)
    gear_.updateStrutCompression(s, groundZ, z, 100.0, 0.1);

    EXPECT_GT(s.wheels[0].strutCompression_ft, 0.0);
    EXPECT_TRUE(s.wheels[0].onGround);
    EXPECT_NEAR(s.wheels[0].strutCompression_ft, 2.0, 0.1);
}

TEST_F(GearTest, StrutOffGround) {
    GearState s;
    gear_.init(s);

    // Aircraft 100 ft above ground -- strut fully extended (no compression)
    gear_.updateStrutCompression(s, -1000.0, -1000.0 - 100.0, 100.0, 0.1);
    EXPECT_FALSE(s.wheels[0].onGround);
    EXPECT_NEAR(s.wheels[0].strutCompression_ft, 0.0, 1e-6);
}
