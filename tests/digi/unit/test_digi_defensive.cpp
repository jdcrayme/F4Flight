// f4flight unit tests - digi AI Tier 1 defensive capabilities
//
// Tests for:
//   - DigiEntity + RelativeGeometry (digi_entity.h)
//   - MissileDefeat (defensive/missile_defeat.h)
//   - GunsJink (defensive/guns_jink.h)
//   - DigiBrain integration of defensive modes
//
// These tests verify the Tier 1 defensive AI directly. They create synthetic
// DigiEntity threats (missiles, guns) and verify the brain selects the correct
// defensive mode and produces sensible stick commands.

#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// DigiEntity + RelativeGeometry tests
// ===========================================================================
class RelativeGeometryTest : public ::testing::Test {
protected:
    DigiEntity self;
    DigiEntity target;

    void SetUp() override {
        // Self at origin, heading north (yaw=0), level
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 500.0;
    }
};

TEST_F(RelativeGeometryTest, TargetDueNorth) {
    // Target 10000 ft due north, same altitude
    target.x = 0.0; target.y = 10000.0; target.z = -10000.0;
    target.yaw = PI;  // heading south (toward us)

    RelativeGeometry rg = computeRelativeGeometry(self, target);

    EXPECT_NEAR(rg.range, 10000.0, 1.0);
    // Bearing to target is north (atan2(10000, 0) = PI/2)
    // Self heading is 0 (north)
    // ata = PI/2 - 0 = PI/2 (90° off nose)
    EXPECT_NEAR(rg.ata, PI / 2.0, 0.01);
    // Target heading is PI (south), bearing to self is PI/2 + PI = 3PI/2 = -PI/2
    // ataFrom = -PI/2 - PI = -3PI/2 → wrap to PI/2
    EXPECT_NEAR(std::fabs(rg.ataFrom), PI / 2.0, 0.01);
}

TEST_F(RelativeGeometryTest, TargetDueEast) {
    // Target 10000 ft due east
    target.x = 10000.0; target.y = 0.0; target.z = -10000.0;
    target.yaw = 0.0;

    RelativeGeometry rg = computeRelativeGeometry(self, target);

    EXPECT_NEAR(rg.range, 10000.0, 1.0);
    // Bearing to target: atan2(0, 10000) = 0 (east in this coord system)
    // Wait — atan2(dy, dx) = atan2(0, 10000) = 0
    // Self heading = 0, so ata = 0 - 0 = 0 (target is directly ahead)
    // Hmm, that means (10000, 0) is "ahead" if heading=0. Let me check the
    // coordinate convention: x=east, y=north, yaw=0=north.
    // Then atan2(dy, dx) = atan2(0, 10000) = 0 = east bearing.
    // Self yaw=0=north. So ata = 0 - 0 = 0. But target is east, not north!
    // This means our convention is: yaw=0 → +x direction (east).
    // The ai_flightplan test uses atan2(dy, dx) for heading, so heading 0
    // means +x. Target at (10000, 0) is directly ahead. OK.
    EXPECT_NEAR(rg.ata, 0.0, 0.01);
}

TEST_F(RelativeGeometryTest, TargetPullingAway) {
    // Self ahead of target, both moving north, self faster → range increasing
    self.vx = 0.0; self.vy = 500.0; self.vz = 0.0;   // moving north at 500 ft/s
    target.x = 0.0; target.y = -5000.0; target.z = -10000.0;
    target.vx = 0.0; target.vy = 1000.0; target.vz = 0.0;  // moving north at 1000 ft/s

    // Wait — target at (0, -5000) moving north at 1000, self at origin moving north at 500.
    // Target is behind and faster → closing. Let me fix: self faster than target.
    self.vy = 1000.0;
    target.vy = 500.0;

    RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Self at (0,0) moving north at 1000. Target at (0,-5000) moving north at 500.
    // Self is pulling away from target → range increasing → rangedot > 0
    EXPECT_GT(rg.rangedot, 0.0);
    EXPECT_LT(rg.closure, 0.0);
}

TEST_F(RelativeGeometryTest, TargetClosingHeadOn) {
    // Head-on: self moving north, target moving south
    self.vx = 0.0; self.vy = 500.0; self.vz = 0.0;
    target.x = 0.0; target.y = 10000.0; target.z = -10000.0;
    target.vx = 0.0; target.vy = -500.0; target.vz = 0.0;

    RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Closing: range decreasing
    EXPECT_LT(rg.rangedot, 0.0);
    EXPECT_GT(rg.closure, 0.0);
}

// ===========================================================================
// MissileDefeatCheck tests
// ===========================================================================
class MissileDefeatCheckTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity missile;

    void SetUp() override {
        digi.reset();
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);

        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.speed = 500.0;

        // Missile 5 NM away, closing
        missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
        missile.seekerType = DigiEntity::SeekerType::Radar;
        missile.isDead = false;
        missile.speed = 2000.0;
    }
};

TEST_F(MissileDefeatCheckTest, NoMissileReturnsFalse) {
    digi.missileDefeat.incomingMissile = nullptr;
    EXPECT_FALSE(MissileDefeatCheck(digi, self, 1.0/60.0));
}

TEST_F(MissileDefeatCheckTest, DeadMissileReturnsFalse) {
    missile.isDead = true;
    digi.missileDefeat.incomingMissile = &missile;
    EXPECT_FALSE(MissileDefeatCheck(digi, self, 1.0/60.0));
    EXPECT_EQ(digi.missileDefeat.incomingMissile, nullptr);
}

TEST_F(MissileDefeatCheckTest, ActiveMissileReturnsTrue) {
    digi.missileDefeat.incomingMissile = &missile;
    EXPECT_TRUE(MissileDefeatCheck(digi, self, 1.0/60.0));
}

TEST_F(MissileDefeatCheckTest, MissilePassingExitsAfterEvadeTimer) {
    digi.missileDefeat.incomingMissile = &missile;

    // First frame: missile at 30000 ft, closing
    MissileDefeatCheck(digi, self, 1.0/60.0);
    EXPECT_NEAR(digi.missileDefeat.incomingMissileRange, 30000.0, 1.0);

    // Missile now moving away (range increasing)
    missile.y = 31000.0;  // range increased
    // Veteran skill = 2, evade hold = 6 - 2 = 4 seconds
    // Run for 5 seconds (past the 4s threshold)
    for (int i = 0; i < 400; ++i) {  // 400 frames at 1/60s ≈ 6.7s
        MissileDefeatCheck(digi, self, 1.0/60.0);
        missile.y += 100.0;  // keep moving away
    }
    // Should have cleared the missile
    EXPECT_EQ(digi.missileDefeat.incomingMissile, nullptr);
}

TEST_F(MissileDefeatCheckTest, MissileStillClosingDoesNotExit) {
    digi.missileDefeat.incomingMissile = &missile;

    // Run 10 seconds with missile closing
    for (int i = 0; i < 600; ++i) {
        missile.y -= 50.0;  // closing
        MissileDefeatCheck(digi, self, 1.0/60.0);
    }
    // Missile should still be tracked
    EXPECT_NE(digi.missileDefeat.incomingMissile, nullptr);
}

// ===========================================================================
// MissileDefeat maneuver tests
// ===========================================================================
class MissileDefeatTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity missile;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        digi.reset();
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);
        digi.config.maxGs = 9.0;
        digi.config.cornerSpeed = 330.0;
        digi.config.maxRoll = 45.0;
        digi.config.maxGammaDeg = 15.0;
        digi.config.turnLoadFactor = 2.0;
        digi.nav.dt = 1.0 / 60.0;

        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 500.0;

        // Level flight state
        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.vcas = 350.0;
        as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(MissileDefeatTest, FarMissileSelectsDragOrBeam) {
    // Missile 5 NM away — TTGO = 30000/2000 = 15s > LD_TIME(1s)
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;  // heading south (toward us)
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    digi.missileDefeat.incomingMissile = &missile;

    MissileDefeat(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Should have commanded something (non-zero stick or throttle)
    bool hasCommand = (std::fabs(digi.commands.pStick) > 0.01 ||
                       std::fabs(digi.commands.rStick) > 0.01 ||
                       digi.commands.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
    // TTGO should be initialized
    EXPECT_GE(digi.missileDefeat.missileDefeatTtgo, 0.0);
}

TEST_F(MissileDefeatTest, CloseMissileSelectsLastDitch) {
    // Missile 1500 ft away — TTGO = 1500/2000 = 0.75s < LD_TIME(1s)
    missile.x = 0.0; missile.y = 1500.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    digi.missileDefeat.incomingMissile = &missile;

    MissileDefeat(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Last ditch = max G pull → positive pstick
    EXPECT_GT(digi.commands.pStick, 0.0);
    // TTGO should be < LD_TIME
    EXPECT_LT(digi.missileDefeat.missileDefeatTtgo, kLDTime);
}

TEST_F(MissileDefeatTest, IRMissileThrottleCutForHighSkill) {
    digi.config.skill = makeSkillParams(SkillLevel::Ace);
    digi.config.skill.irMissileThrottleCut = true;

    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::IR;
    digi.missileDefeat.incomingMissile = &missile;

    MissileDefeat(digi, self, as, fcs, fcsState, 1.0/60.0);

    // IR + skill > 2 → throttle capped at 0.99
    EXPECT_LE(digi.commands.throttle, 0.991);
}

TEST_F(MissileDefeatTest, IRMissileNoThrottleCutForLowSkill) {
    digi.config.skill = makeSkillParams(SkillLevel::Rookie);
    digi.config.skill.irMissileThrottleCut = false;

    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::IR;
    digi.missileDefeat.incomingMissile = &missile;

    MissileDefeat(digi, self, as, fcs, fcsState, 1.0/60.0);

    // IR + skill <= 2 → throttle NOT capped (may exceed 0.99 if MachHold wants AB)
    // Just verify it doesn't crash and produces a valid throttle
    EXPECT_GE(digi.commands.throttle, 0.0);
    EXPECT_LE(digi.commands.throttle, 1.5);
}

// ===========================================================================
// GunsJinkCheck tests
// ===========================================================================
class GunsJinkCheckTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity threat;

    void SetUp() override {
        digi.reset();
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);

        // Self heading north (yaw=0 in our convention: +x = ahead)
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 400.0;
    }
};

TEST_F(GunsJinkCheckTest, NoThreatReturnsFalse) {
    digi.gunsJink.gunsThreat = nullptr;
    EXPECT_FALSE(GunsJinkCheck(digi, self));
}

TEST_F(GunsJinkCheckTest, DeadThreatReturnsFalse) {
    threat.isDead = true;
    digi.gunsJink.gunsThreat = &threat;
    EXPECT_FALSE(GunsJinkCheck(digi, self));
}

TEST_F(GunsJinkCheckTest, ThreatOutOfRangeReturnsFalse) {
    // Threat 7000 ft away (> 6000 ft max)
    threat.x = 7000.0; threat.y = 0.0; threat.z = -10000.0;
    threat.yaw = PI;  // pointing at us
    threat.isFiring = true;
    digi.gunsJink.gunsThreat = &threat;
    EXPECT_FALSE(GunsJinkCheck(digi, self));
}

TEST_F(GunsJinkCheckTest, ThreatNotFiringReturnsFalse) {
    threat.x = 3000.0; threat.y = 0.0; threat.z = -10000.0;
    threat.yaw = PI;
    threat.isFiring = false;
    digi.gunsJink.gunsThreat = &threat;
    EXPECT_FALSE(GunsJinkCheck(digi, self));
}

TEST_F(GunsJinkCheckTest, ThreatInFiringPositionReturnsTrue) {
    // Threat 3000 ft ahead, pointing at us, firing
    // Self yaw=0 (+x ahead). Threat at (3000, 0).
    // Bearing to target = atan2(0, 3000) = 0. Self yaw=0. ata = 0.
    // Bearing to self = atan2(0, -3000) = PI. Threat yaw=PI. ataFrom = PI - PI = 0.
    // azFrom = 0 → within ±15° ✓
    // elFrom = 0 (same altitude) → within -10° to +4° ✓
    threat.x = 3000.0; threat.y = 0.0; threat.z = -10000.0;
    threat.yaw = PI;  // pointing at us
    threat.isFiring = true;
    digi.gunsJink.gunsThreat = &threat;
    EXPECT_TRUE(GunsJinkCheck(digi, self));
}

TEST_F(GunsJinkCheckTest, ThreatOutOfAzimuthReturnsFalse) {
    // Threat at 60° off our nose — azFrom would be ~60°, outside ±15°
    // Threat at (3000, 5196) ≈ 60° from +x axis
    threat.x = 3000.0; threat.y = 5196.0; threat.z = -10000.0;
    threat.yaw = atan2(-5196.0, -3000.0);  // pointing back at us
    threat.isFiring = true;
    digi.gunsJink.gunsThreat = &threat;
    EXPECT_FALSE(GunsJinkCheck(digi, self));
}

// ===========================================================================
// GunsJink maneuver tests
// ===========================================================================
class GunsJinkTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity threat;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        digi.reset();
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);
        digi.config.maxGs = 9.0;
        digi.config.cornerSpeed = 330.0;
        digi.config.maxRoll = 190.0;  // fighters allow unlimited roll in jink
        digi.config.maxGammaDeg = 15.0;
        digi.config.turnLoadFactor = 2.0;
        digi.nav.dt = 1.0 / 60.0;

        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.speed = 400.0;

        threat.x = 3000.0; threat.y = 0.0; threat.z = -10000.0;
        threat.yaw = PI;
        threat.isFiring = true;
        threat.isDead = false;
        digi.gunsJink.gunsThreat = &threat;

        as.kin.costhe = 1.0;
        as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0;
        as.kin.sigma = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.phi = 0.0;
        as.vcas = 350.0;
        as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(GunsJinkTest, InitPhasePicksRollAngle) {
    // First frame: jinkTime = -1 → should pick a roll angle and set jinkTime = 0
    bool stillJinking = GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    EXPECT_TRUE(stillJinking);
    EXPECT_EQ(digi.gunsJink.jinkTime, 0);  // moved to rolling phase
    // newRoll should be set to a non-zero value
    EXPECT_NE(digi.gunsJink.newRoll, 0.0);
}

TEST_F(GunsJinkTest, RollingPhaseCommandsNonZeroStick) {
    // Start in rolling phase
    digi.gunsJink.jinkTime = 0;
    digi.gunsJink.newRoll = 70.0 * DTR;  // 70° target bank
    self.roll = 0.0;  // currently level

    GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Should command non-zero rstick to roll
    EXPECT_NE(digi.commands.rStick, 0.0);
}

TEST_F(GunsJinkTest, PullPhaseCommandsMaxG) {
    // Start in pull phase
    digi.gunsJink.jinkTime = 1;
    digi.gunsJink.jinkTimer = 0.0;
    self.roll = 0.0;

    GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Should command positive pstick (max G pull)
    EXPECT_GT(digi.commands.pStick, 0.0);
    EXPECT_GT(digi.gunsJink.jinkTimer, 0.0);  // timer incremented
}

TEST_F(GunsJinkTest, PullPhaseExitsAfterDuration) {
    digi.gunsJink.jinkTime = 1;
    digi.gunsJink.jinkTimer = kJinkPullDuration - 0.01;  // almost done

    bool stillJinking = GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    // Should exit (return false) after exceeding pull duration
    EXPECT_FALSE(stillJinking);
    EXPECT_EQ(digi.gunsJink.jinkTime, -1);  // reset
}

TEST_F(GunsJinkTest, ExitsWhenThreatOutOfRange) {
    // Threat moved beyond 4000 ft
    threat.x = 4500.0;
    digi.gunsJink.jinkTime = 1;

    bool stillJinking = GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    EXPECT_FALSE(stillJinking);
    EXPECT_EQ(digi.gunsJink.jinkTime, -1);
}

TEST_F(GunsJinkTest, NoThreatExits) {
    digi.gunsJink.gunsThreat = nullptr;
    digi.gunsJink.jinkTime = 1;

    bool stillJinking = GunsJink(digi, self, as, fcs, fcsState, 1.0/60.0);

    EXPECT_FALSE(stillJinking);
    EXPECT_EQ(digi.gunsJink.jinkTime, -1);
}

// ===========================================================================
// DigiBrain defensive integration tests
// ===========================================================================
class DigiBrainDefensiveTest : public ::testing::Test {
protected:
    DigiBrain brain;
    DigiEntity selfEntity;
    DigiEntity missile;
    DigiEntity gunsThreat;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        brain.setMaxGs(9.0);
        brain.setMaxBank(45.0);
        brain.setMaxGamma(15.0);
        brain.setTurnG(2.0);
        brain.setCornerSpeed(330.0);

        selfEntity.x = 0.0; selfEntity.y = 0.0; selfEntity.z = -10000.0;
        selfEntity.yaw = 0.0; selfEntity.pitch = 0.0; selfEntity.roll = 0.0;
        selfEntity.speed = 500.0;
        brain.setSelfEntity(&selfEntity);

        // Level flight state
        state.kin.costhe = 1.0;
        state.kin.cosphi = 1.0;
        state.kin.gmma = 0.0;
        state.kin.sigma = 0.0;
        state.kin.singam = 0.0;
        state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
        state.kin.zdot = 0.0;
        state.kin.phi = 0.0;
        state.vcas = 350.0;
        state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(DigiBrainDefensiveTest, NoThreatsDefaultsToWaypoint) {
    brain.setIncomingMissile(nullptr);
    brain.setGunsThreat(nullptr);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainDefensiveTest, IncomingMissileTriggersMissileDefeat) {
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;
    brain.setIncomingMissile(&missile);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainDefensiveTest, GunsThreatTriggersGunsJink) {
    gunsThreat.x = 3000.0; gunsThreat.y = 0.0; gunsThreat.z = -10000.0;
    gunsThreat.yaw = PI;
    gunsThreat.isFiring = true;
    gunsThreat.isDead = false;
    brain.setGunsThreat(&gunsThreat);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::GunsJink);
}

TEST_F(DigiBrainDefensiveTest, MissileDefeatPreemptsGunsJink) {
    // Both threats present — missile has higher priority
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;

    gunsThreat.x = 3000.0; gunsThreat.y = 0.0; gunsThreat.z = -10000.0;
    gunsThreat.yaw = PI;
    gunsThreat.isFiring = true;
    gunsThreat.isDead = false;

    brain.setIncomingMissile(&missile);
    brain.setGunsThreat(&gunsThreat);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);
}

TEST_F(DigiBrainDefensiveTest, MissileDefeatProducesValidOutput) {
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;
    brain.setIncomingMissile(&missile);

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    EXPECT_GE(out.pstick, -1.0);
    EXPECT_LE(out.pstick, 1.0);
    EXPECT_GE(out.rstick, -1.0);
    EXPECT_LE(out.rstick, 1.0);
    EXPECT_GE(out.throttle, 0.0);
    EXPECT_LE(out.throttle, 1.5);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
    EXPECT_FALSE(std::isnan(out.throttle));
}

TEST_F(DigiBrainDefensiveTest, GunsJinkProducesValidOutput) {
    gunsThreat.x = 3000.0; gunsThreat.y = 0.0; gunsThreat.z = -10000.0;
    gunsThreat.yaw = PI;
    gunsThreat.isFiring = true;
    gunsThreat.isDead = false;
    brain.setGunsThreat(&gunsThreat);

    PilotInput out = brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    EXPECT_GE(out.pstick, -1.0);
    EXPECT_LE(out.pstick, 1.0);
    EXPECT_FALSE(std::isnan(out.pstick));
    EXPECT_FALSE(std::isnan(out.rstick));
}

TEST_F(DigiBrainDefensiveTest, ClearedMissileReturnsToWaypoint) {
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;

    brain.setIncomingMissile(&missile);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::MissileDefeat);

    // Clear the missile
    brain.setIncomingMissile(nullptr);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(DigiBrainDefensiveTest, GroundAvoidPreemptsMissileDefeat) {
    // Missile present but aircraft at 200 ft AGL
    missile.x = 0.0; missile.y = 30000.0; missile.z = -10000.0;
    missile.yaw = PI;
    missile.speed = 2000.0;
    missile.seekerType = DigiEntity::SeekerType::Radar;
    missile.isDead = false;
    brain.setIncomingMissile(&missile);

    state.kin.z = -200.0;  // 200 ft AGL
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);

    // Ground avoidance should pre-empt (pull-up commanded)
    // The mode might still say MissileDefeat but the output should be a pull-up
    EXPECT_GT(brain.state().groundAvoid.pullupTimer, 0.0);
}
