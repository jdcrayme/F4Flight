// f4flight unit tests - digi AI sensor system (Phase 3)
//
// Tests for:
//   - SensorContact / SensorPicture (sensor_picture.h)
//   - TruthState (sensor.h)
//   - RadarSensor (radar_sensor.h)
//   - RWRSensor (rwr_sensor.h)
//   - VisualSensor (visual_sensor.h)
//   - SensorFusion (sensor_fusion.h)
//   - DigiBrain autonomous detection integration

#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/digi/sensors/sensor.h"
#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/digi/sensors/radar_sensor.h"
#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/digi/sensors/visual_sensor.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// SensorContact / SensorPicture tests
// ===========================================================================
TEST(SensorPictureTest, DefaultEmpty) {
    SensorPicture pic;
    EXPECT_TRUE(pic.contacts.empty());
    EXPECT_EQ(pic.highestThreat, nullptr);
    EXPECT_EQ(pic.bestTarget, nullptr);
    EXPECT_EQ(pic.incomingMissile, nullptr);
    EXPECT_EQ(pic.gunsThreat, nullptr);
    EXPECT_FALSE(pic.spiked);
}

TEST(SensorPictureTest, ClearResetsAll) {
    SensorPicture pic;
    pic.contacts.push_back(SensorContact{});
    pic.spiked = true;

    pic.clear();

    EXPECT_TRUE(pic.contacts.empty());
    EXPECT_FALSE(pic.spiked);
}

TEST(SensorPictureTest, FindById) {
    SensorPicture pic;
    SensorContact c1;
    c1.entityId = 100;
    SensorContact c2;
    c2.entityId = 200;
    pic.contacts.push_back(c1);
    pic.contacts.push_back(c2);

    const SensorContact* found = pic.findById(200);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->entityId, 200);

    EXPECT_EQ(pic.findById(999), nullptr);
}

TEST(SensorContactTest, SensorMaskOperations) {
    SensorContact c;
    EXPECT_FALSE(c.detectedBy(SensorType::Radar));
    EXPECT_FALSE(c.detectedBy(SensorType::RWR));

    c.addSensor(SensorType::Radar);
    EXPECT_TRUE(c.detectedBy(SensorType::Radar));
    EXPECT_FALSE(c.detectedBy(SensorType::RWR));

    c.addSensor(SensorType::RWR);
    EXPECT_TRUE(c.detectedBy(SensorType::Radar));
    EXPECT_TRUE(c.detectedBy(SensorType::RWR));
}

// ===========================================================================
// TruthState tests
// ===========================================================================
TEST(TruthStateTest, AddAndClear) {
    TruthState truth;
    EXPECT_EQ(truth.size(), 0u);

    DigiEntity e;
    e.x = 1000; e.y = 2000; e.z = -10000;
    truth.add(100, e, false);

    EXPECT_EQ(truth.size(), 1u);
    EXPECT_EQ(truth.ids[0], 100);
    EXPECT_NEAR(truth.entities[0].x, 1000.0, 1e-9);
    EXPECT_FALSE(truth.firing[0]);

    truth.clear();
    EXPECT_EQ(truth.size(), 0u);
}

// ===========================================================================
// RadarSensor tests
// ===========================================================================
class RadarSensorTest : public ::testing::Test {
protected:
    DigiEntity self;
    TruthState truth;
    SkillParameters skill{makeSkillParams(SkillLevel::Veteran)};

    void SetUp() override {
        self.x = 0; self.y = 0; self.z = -10000;
        self.yaw = 0; self.pitch = 0; self.roll = 0;
        self.speed = 500;
    }
};

TEST_F(RadarSensorTest, DetectsTargetInRange) {
    DigiEntity target;
    // yaw=0 means +x direction, so target ahead is at +x
    target.x = 30000; target.y = 0; target.z = -10000;  // 5 NM ahead
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    RadarSensor radar;
    std::vector<SensorContact> contacts;
    radar.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_EQ(contacts[0].entityId, 100);
    EXPECT_TRUE(contacts[0].detectedBy(SensorType::Radar));
    EXPECT_GT(contacts[0].confidence, 0.0);
}

TEST_F(RadarSensorTest, NoDetectionOutOfRange) {
    DigiEntity target;
    target.x = 100 * 6076; target.y = 0; target.z = -10000;  // 100 NM ahead (beyond 80 NM)
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    RadarSensor radar;
    std::vector<SensorContact> contacts;
    radar.update(self, truth, skill, 1.0/60.0, contacts);

    EXPECT_TRUE(contacts.empty());
}

TEST_F(RadarSensorTest, NoDetectionOutOfAzimuth) {
    // Target 90° off nose (behind us in this convention)
    DigiEntity target;
    target.x = 0; target.y = -30000; target.z = -10000;  // behind (180° off nose)
    target.yaw = 0; target.speed = 500;
    truth.add(100, target);

    RadarSensor radar;
    std::vector<SensorContact> contacts;
    radar.update(self, truth, skill, 1.0/60.0, contacts);

    // 180° is outside ±60° field of regard — should not detect
    EXPECT_TRUE(contacts.empty());
}

TEST_F(RadarSensorTest, IdentifiesAtCloseRange) {
    DigiEntity target;
    target.x = 5 * 6076; target.y = 0; target.z = -10000;  // 5 NM ahead
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    RadarSensor radar;
    std::vector<SensorContact> contacts;
    radar.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_EQ(contacts[0].quality, ContactQuality::Identified);
    EXPECT_EQ(contacts[0].type, ContactType::Fighter);
}

// ===========================================================================
// RWRSensor tests
// ===========================================================================
class RWRSensorTest : public ::testing::Test {
protected:
    DigiEntity self;
    TruthState truth;
    SkillParameters skill{makeSkillParams(SkillLevel::Veteran)};

    void SetUp() override {
        self.x = 0; self.y = 0; self.z = -10000;
        self.yaw = 0; self.pitch = 0; self.roll = 0;
        self.speed = 500;
    }
};

TEST_F(RWRSensorTest, DetectsRadarMissile) {
    DigiEntity missile;
    missile.x = 50000; missile.y = 0; missile.z = -10000;  // ahead
    missile.yaw = PI; missile.speed = 2000;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    truth.add(200, missile);

    RWRSensor rwr;
    std::vector<SensorContact> contacts;
    rwr.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_TRUE(contacts[0].detectedBy(SensorType::RWR));
    EXPECT_EQ(contacts[0].type, ContactType::Missile);
    EXPECT_TRUE(contacts[0].isMissile);
}

TEST_F(RWRSensorTest, DetectsAircraftRadar) {
    DigiEntity target;
    target.x = 50 * 6076; target.y = 0; target.z = -10000;  // 50 NM ahead
    target.yaw = PI; target.speed = 500;
    target.seekerType = DigiEntity::SeekerType::None;  // aircraft
    truth.add(100, target);

    RWRSensor rwr;
    std::vector<SensorContact> contacts;
    rwr.update(self, truth, skill, 1.0/60.0, contacts);

    EXPECT_EQ(contacts.size(), 1u);
}

TEST_F(RWRSensorTest, SpikeDetection) {
    // Aircraft pointing at us, close range
    DigiEntity target;
    target.x = 20 * 6076; target.y = 0; target.z = -10000;  // 20 NM ahead
    target.yaw = PI;  // pointing west (back at us)
    target.speed = 500;
    truth.add(100, target);

    RWRSensor rwr;
    std::vector<SensorContact> contacts;
    rwr.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_TRUE(contacts[0].isThreat);  // spike detected
}

// ===========================================================================
// VisualSensor tests
// ===========================================================================
class VisualSensorTest : public ::testing::Test {
protected:
    DigiEntity self;
    TruthState truth;
    SkillParameters skill{makeSkillParams(SkillLevel::Veteran)};

    void SetUp() override {
        self.x = 0; self.y = 0; self.z = -10000;
        self.yaw = 0; self.pitch = 0; self.roll = 0;
        self.speed = 500;
    }
};

TEST_F(VisualSensorTest, DetectsTargetInRange) {
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;  // 3 NM ahead
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    VisualSensor visual;
    std::vector<SensorContact> contacts;
    visual.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_TRUE(contacts[0].detectedBy(SensorType::Visual));
}

TEST_F(VisualSensorTest, NoDetectionOutOfRange) {
    DigiEntity target;
    target.x = 15 * 6076; target.y = 0; target.z = -10000;  // 15 NM ahead (beyond 8 NM)
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    VisualSensor visual;
    std::vector<SensorContact> contacts;
    visual.update(self, truth, skill, 1.0/60.0, contacts);

    EXPECT_TRUE(contacts.empty());
}

TEST_F(VisualSensorTest, DetectsGunFire) {
    DigiEntity target;
    target.x = 3000; target.y = 0; target.z = -10000;  // 3000 ft ahead
    target.yaw = PI; target.speed = 500;
    truth.add(100, target, true);  // isFiring = true

    VisualSensor visual;
    std::vector<SensorContact> contacts;
    visual.update(self, truth, skill, 1.0/60.0, contacts);

    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_TRUE(contacts[0].isFiring);
    EXPECT_TRUE(contacts[0].isThreat);
}

TEST_F(VisualSensorTest, NoDetectionBehind) {
    // Target behind us (-x direction, 180° off nose)
    DigiEntity target;
    target.x = -3000; target.y = 0; target.z = -10000;  // behind
    target.yaw = 0; target.speed = 500;
    truth.add(100, target);

    VisualSensor visual;
    std::vector<SensorContact> contacts;
    visual.update(self, truth, skill, 1.0/60.0, contacts);

    // ±90° field of regard — target at 180° should not be detected
    EXPECT_TRUE(contacts.empty());
}

// ===========================================================================
// SensorFusion tests
// ===========================================================================
class SensorFusionTest : public ::testing::Test {
protected:
    DigiEntity self;
    TruthState truth;
    SkillParameters skill{makeSkillParams(SkillLevel::Veteran)};
    SensorFusion fusion;

    void SetUp() override {
        self.x = 0; self.y = 0; self.z = -10000;
        self.yaw = 0; self.pitch = 0; self.roll = 0;
        self.speed = 500;
    }
};

TEST_F(SensorFusionTest, DefaultSensorSuite) {
    // SensorFusion constructor adds radar + RWR + visual
    EXPECT_EQ(fusion.sensorCount(), 3u);
}

TEST_F(SensorFusionTest, MergesContactsFromMultipleSensors) {
    // Target at 3 NM ahead — detectable by radar, RWR, and visual
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    fusion.update(self, truth, skill, 1.0/60.0);

    const SensorPicture& pic = fusion.picture();
    ASSERT_EQ(pic.contacts.size(), 1u);
    // Should be detected by multiple sensors
    EXPECT_TRUE(pic.contacts[0].detectedBy(SensorType::Radar));
    EXPECT_TRUE(pic.contacts[0].detectedBy(SensorType::RWR));
    EXPECT_TRUE(pic.contacts[0].detectedBy(SensorType::Visual));
}

TEST_F(SensorFusionTest, IdentifiesMissileThreat) {
    DigiEntity missile;
    missile.x = 30000; missile.y = 0; missile.z = -10000;
    missile.yaw = PI; missile.speed = 2000;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    truth.add(200, missile);

    fusion.update(self, truth, skill, 1.0/60.0);

    const SensorPicture& pic = fusion.picture();
    EXPECT_NE(pic.incomingMissile, nullptr);
    EXPECT_TRUE(pic.incomingMissile->isMissile);
}

TEST_F(SensorFusionTest, IdentifiesGunsThreat) {
    DigiEntity target;
    target.x = 3000; target.y = 0; target.z = -10000;
    target.yaw = PI;  // pointing west (at us)
    target.speed = 500;
    truth.add(100, target, true);  // firing

    fusion.update(self, truth, skill, 1.0/60.0);

    const SensorPicture& pic = fusion.picture();
    EXPECT_NE(pic.gunsThreat, nullptr);
    if (pic.gunsThreat) {
        EXPECT_TRUE(pic.gunsThreat->isFiring);
    }
}

TEST_F(SensorFusionTest, IdentifiesBestTarget) {
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    fusion.update(self, truth, skill, 1.0/60.0);

    const SensorPicture& pic = fusion.picture();
    EXPECT_NE(pic.bestTarget, nullptr);
    EXPECT_EQ(pic.bestTarget->entityId, 100);
}

TEST_F(SensorFusionTest, ComputesThreatScore) {
    // Close target = high threat score
    DigiEntity close;
    close.x = 3000; close.y = 0; close.z = -10000;
    close.yaw = PI; close.speed = 500;
    truth.add(100, close);

    // Far target = low threat score
    DigiEntity far;
    far.x = 6 * 6076; far.y = 0; far.z = -10000;
    far.yaw = PI; far.speed = 500;
    truth.add(101, far);

    fusion.update(self, truth, skill, 1.0/60.0);

    const SensorPicture& pic = fusion.picture();
    ASSERT_EQ(pic.contacts.size(), 2u);

    const SensorContact* closeContact = pic.findById(100);
    const SensorContact* farContact = pic.findById(101);
    ASSERT_NE(closeContact, nullptr);
    ASSERT_NE(farContact, nullptr);

    EXPECT_GT(closeContact->threatScore, farContact->threatScore);
    EXPECT_EQ(pic.highestThreat, closeContact);
}

TEST_F(SensorFusionTest, AgesAndPurgesContacts) {
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500;
    truth.add(100, target);

    // Detect the target
    fusion.update(self, truth, skill, 1.0/60.0);
    EXPECT_EQ(fusion.picture().contacts.size(), 1u);

    // Stop providing truth — contact should age and expire
    TruthState empty;
    for (int i = 0; i < 400; ++i) {  // 400 frames ≈ 6.7s at 60 Hz
        fusion.update(self, empty, skill, 1.0/60.0);
    }
    EXPECT_TRUE(fusion.picture().contacts.empty());
}

// ===========================================================================
// DigiBrain autonomous detection tests
// ===========================================================================
class DigiBrainSensorTest : public ::testing::Test {
protected:
    DigiBrain brain;
    DigiEntity selfEntity;
    TruthState truth;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        brain.setMaxGs(9.0);
        brain.setMaxBank(45.0);
        brain.setMaxGamma(15.0);
        brain.setTurnG(2.0);
        brain.setCornerSpeed(330.0);
        brain.setTruth(&truth);

        selfEntity.x = 0; selfEntity.y = 0; selfEntity.z = -10000;
        selfEntity.yaw = 0; selfEntity.pitch = 0; selfEntity.roll = 0;
        selfEntity.speed = 500;
        brain.setSelfEntity(&selfEntity);

        state.kin.costhe = 1.0;
        state.kin.cosphi = 1.0;
        state.kin.gmma = 0.0;
        state.kin.sigma = 0.0;
        state.kin.singam = 0.0;
        state.kin.x = 0; state.kin.y = 0; state.kin.z = -10000;
        state.kin.zdot = 0;
        state.kin.phi = 0;
        state.vcas = 350.0;
        state.kin.vt = 589.0;
    }
};

TEST_F(DigiBrainSensorTest, NoTruthDefaultsToWaypoint) {
    truth.clear();
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainSensorTest, DetectsMissileAndEntersMissileDefeat) {
    DigiEntity missile;
    missile.x = 30000; missile.y = 0; missile.z = -10000;
    missile.yaw = PI; missile.speed = 2000;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    truth.clear();
    truth.add(200, missile);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainSensorTest, DetectsGunsThreatAndEntersGunsJink) {
    DigiEntity target;
    target.x = 3000; target.y = 0; target.z = -10000;
    target.yaw = PI;  // pointing west (at us)
    target.speed = 500;
    truth.clear();
    truth.add(100, target, true);  // firing

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::GunsJink);
}

TEST_F(DigiBrainSensorTest, DetectsTargetAndEntersWVREngage) {
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;  // 3 NM ahead
    target.yaw = PI; target.speed = 500;
    truth.clear();
    truth.add(100, target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::WVREngage);
}

TEST_F(DigiBrainSensorTest, TargetBeyondWVRStaysWaypoint) {
    DigiEntity target;
    target.x = 15 * 6076; target.y = 0; target.z = -10000;  // 15 NM (beyond WVR)
    target.yaw = PI; target.speed = 500;
    truth.clear();
    truth.add(100, target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainSensorTest, MissilePreemptsTarget) {
    // Both missile and target present
    DigiEntity missile;
    missile.x = 30000; missile.y = 0; missile.z = -10000;
    missile.yaw = PI; missile.speed = 2000;
    missile.seekerType = DigiEntity::SeekerType::Radar;

    DigiEntity target;
    target.x = 3000; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500;

    truth.clear();
    truth.add(200, missile);
    truth.add(100, target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainSensorTest, SensorPictureAvailableAfterCompute) {
    DigiEntity target;
    target.x = 3 * 6076; target.y = 0; target.z = -10000;
    target.yaw = PI; target.speed = 500;
    truth.clear();
    truth.add(100, target);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    const SensorPicture& pic = brain.sensorPicture();
    EXPECT_FALSE(pic.contacts.empty());
    EXPECT_NE(pic.bestTarget, nullptr);
}
