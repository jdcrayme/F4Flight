// f4flight unit tests - Round 7 (P1): ChooseRadarMode, ApplyGCI, ApplyNCTR,
// DoTargeting, Loiter entry.
//
// These tests cover the P1 capabilities added in Round 7:
//   - ChooseRadarMode (radModeSelect → RadarMode mapping + throttle)
//   - ApplyGCI (skill-gated detection beyond sensor range)
//   - ApplyNCTR (radar-based type identification at close range)
//   - DoTargeting (autonomous target selection from SensorPicture)
//   - Loiter mode entry (no waypoints → orbit)

#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/decision/decision_routines.h"
#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/flight/core/constants.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// ChooseRadarMode tests
// ===========================================================================
class ChooseRadarModeTest : public ::testing::Test {
protected:
    DigiState digi;

    void SetUp() override {
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);
        digi.weapon.lastRadarModeTime = -1e9;  // force first call to evaluate
    }
};

TEST_F(ChooseRadarModeTest, DefaultRadModeSelectIsRWS) {
    EXPECT_EQ(digi.weapon.radModeSelect, 3);  // RWS default
    ChooseRadarMode(digi, 0.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::RWS));
}

TEST_F(ChooseRadarModeTest, STTModeSelect) {
    digi.weapon.radModeSelect = 0;  // STT
    ChooseRadarMode(digi, 0.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::STT));
}

TEST_F(ChooseRadarModeTest, SAMModeSelect) {
    digi.weapon.radModeSelect = 1;  // SAM
    ChooseRadarMode(digi, 0.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::SAM));
}

TEST_F(ChooseRadarModeTest, TWSModeSelectWithTWS) {
    digi.weapon.radModeSelect = 2;  // TWS
    ChooseRadarMode(digi, 0.0, /*hasTWS=*/true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::TWS));
}

TEST_F(ChooseRadarModeTest, TWSModeSelectWithoutTWSFallsBackToRWS) {
    digi.weapon.radModeSelect = 2;  // TWS
    ChooseRadarMode(digi, 0.0, /*hasTWS=*/false);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::RWS));
}

TEST_F(ChooseRadarModeTest, OFFModeSelect) {
    digi.weapon.radModeSelect = 4;  // OFF
    ChooseRadarMode(digi, 0.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::OFF));
}

TEST_F(ChooseRadarModeTest, ThrottlePreventsRapidReevaluation) {
    ChooseRadarMode(digi, 0.0, true);
    const int firstMode = digi.weapon.radarMode;

    // Change radModeSelect and call again within the throttle window.
    digi.weapon.radModeSelect = 0;  // STT
    ChooseRadarMode(digi, 1.0, true);  // 1s later — within throttle (8s for veteran)

    // Mode should NOT have changed (throttled).
    EXPECT_EQ(digi.weapon.radarMode, firstMode);

    // After the throttle window, it should re-evaluate.
    ChooseRadarMode(digi, 9.0, true);  // 9s later — past throttle (8s)
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::STT));
}

TEST_F(ChooseRadarModeTest, AceHasShorterThrottle) {
    digi.config.skill = makeSkillParams(SkillLevel::Ace);
    ChooseRadarMode(digi, 0.0, true);

    // Ace throttle = 4 + (4 - 4) = 4s. Call at 3s — throttled.
    digi.weapon.radModeSelect = 0;  // STT
    ChooseRadarMode(digi, 3.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::RWS));  // unchanged

    // Call at 5s — past throttle.
    ChooseRadarMode(digi, 5.0, true);
    EXPECT_EQ(digi.weapon.radarMode, static_cast<int>(RadarMode::STT));
}

// ===========================================================================
// ApplyGCI tests
// ===========================================================================
class ApplyGCITest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    SensorPicture pic;

    void SetUp() override {
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 500.0;
    }
};

TEST_F(ApplyGCITest, RecruitDoesNotGetGCI) {
    digi.config.skill = makeSkillParams(SkillLevel::Recruit);
    // Contact 20 NM away (within GCI range) but quality=None.
    SensorContact c;
    c.x = 20.0 * 6076.0; c.y = 0; c.z = -10000;
    c.quality = ContactQuality::None;
    pic.contacts.push_back(c);

    ApplyGCI(digi, pic, self);

    // Recruit: GCI not capable → contact unchanged.
    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::None);
}

TEST_F(ApplyGCITest, VeteranDetectsContactWithin30NM) {
    digi.config.skill = makeSkillParams(SkillLevel::Veteran);
    SensorContact c;
    c.x = 20.0 * 6076.0; c.y = 0; c.z = -10000;
    c.quality = ContactQuality::None;
    pic.contacts.push_back(c);

    ApplyGCI(digi, pic, self);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Detected);
    EXPECT_TRUE(pic.contacts[0].detectedBy(SensorType::GCI));
}

TEST_F(ApplyGCITest, GCIDoesNotDetectBeyond30NM) {
    digi.config.skill = makeSkillParams(SkillLevel::Ace);
    SensorContact c;
    c.x = 35.0 * 6076.0; c.y = 0; c.z = -10000;  // 35 NM — beyond 30 NM
    c.quality = ContactQuality::None;
    pic.contacts.push_back(c);

    ApplyGCI(digi, pic, self);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::None);
}

TEST_F(ApplyGCITest, GCIDoesNotUpgradeAlreadyDetectedContact) {
    digi.config.skill = makeSkillParams(SkillLevel::Veteran);
    SensorContact c;
    c.x = 20.0 * 6076.0; c.y = 0; c.z = -10000;
    c.quality = ContactQuality::Tracked;  // already better than Detected
    c.confidence = 0.9;
    pic.contacts.push_back(c);

    ApplyGCI(digi, pic, self);

    // Quality should NOT be downgraded to Detected.
    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Tracked);
}

TEST_F(ApplyGCITest, GCIIgnoresMissiles) {
    digi.config.skill = makeSkillParams(SkillLevel::Ace);
    SensorContact c;
    c.x = 10.0 * 6076.0; c.y = 0; c.z = -10000;
    c.isMissile = true;
    c.quality = ContactQuality::None;
    pic.contacts.push_back(c);

    ApplyGCI(digi, pic, self);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::None);  // not upgraded
}

// ===========================================================================
// ApplyNCTR tests
// ===========================================================================
class ApplyNCTRTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    SensorPicture pic;

    void SetUp() override {
        digi.config.skill = makeSkillParams(SkillLevel::Veteran);
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 500.0;
    }
};

TEST_F(ApplyNCTRTest, IdentifiesContactWithinRangeAndAspect) {
    // Contact 10 NM east, heading west (toward self) → ataFrom ≈ 0°.
    SensorContact c;
    c.x = 10.0 * 6076.0; c.y = 0; c.z = -10000;
    c.yaw = PI;  // heading west (toward self)
    c.quality = ContactQuality::Tracked;
    c.type = ContactType::Unknown;
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, /*hasNCTR=*/true, /*maxNctrRangeFt=*/60.0 * 6076.0);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Identified);
    EXPECT_EQ(pic.contacts[0].type, ContactType::Fighter);  // guessed
}

TEST_F(ApplyNCTRTest, DoesNotIdentifyBeyondMaxRange) {
    SensorContact c;
    c.x = 80.0 * 6076.0; c.y = 0; c.z = -10000;  // 80 NM — beyond 60 NM max
    c.yaw = PI;
    c.quality = ContactQuality::Tracked;
    c.type = ContactType::Unknown;
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, true, 60.0 * 6076.0);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Tracked);  // not upgraded
}

TEST_F(ApplyNCTRTest, DoesNotIdentifyBadAspect) {
    // Contact heading east (same as self) → ataFrom ≈ 180° (behind).
    SensorContact c;
    c.x = 10.0 * 6076.0; c.y = 0; c.z = -10000;
    c.yaw = 0.0;  // heading east (same as self) → not face-on
    c.quality = ContactQuality::Tracked;
    c.type = ContactType::Unknown;
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, true, 60.0 * 6076.0);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Tracked);  // not identified
}

TEST_F(ApplyNCTRTest, NoNCRTRadarIsNoOp) {
    SensorContact c;
    c.x = 10.0 * 6076.0; c.y = 0; c.z = -10000;
    c.yaw = PI;
    c.quality = ContactQuality::Tracked;
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, /*hasNCTR=*/false, 60.0 * 6076.0);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Tracked);  // unchanged
}

TEST_F(ApplyNCTRTest, DoesNotReidentifyAlreadyIdentified) {
    SensorContact c;
    c.x = 10.0 * 6076.0; c.y = 0; c.z = -10000;
    c.yaw = PI;
    c.quality = ContactQuality::Identified;
    c.type = ContactType::Bomber;  // already identified as Bomber
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, true, 60.0 * 6076.0);

    // Should NOT overwrite the existing type with Fighter.
    EXPECT_EQ(pic.contacts[0].type, ContactType::Bomber);
}

TEST_F(ApplyNCTRTest, AceHasLongerEffectiveRange) {
    digi.config.skill = makeSkillParams(SkillLevel::Ace);
    // Contact at 50 NM. Ace effective range = 60NM / (2*(16-4)/16) = 60/1.5 = 40NM.
    // 50 NM > 40 NM → should NOT be identified.
    SensorContact c;
    c.x = 50.0 * 6076.0; c.y = 0; c.z = -10000;
    c.yaw = PI;
    c.quality = ContactQuality::Tracked;
    c.type = ContactType::Unknown;
    pic.contacts.push_back(c);

    ApplyNCTR(digi, pic, self, true, 60.0 * 6076.0);

    EXPECT_EQ(pic.contacts[0].quality, ContactQuality::Tracked);  // out of range
}

// ===========================================================================
// DoTargeting tests
// ===========================================================================
class DoTargetingTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    SensorPicture pic;

    void SetUp() override {
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.speed = 500.0;
    }
};

TEST_F(DoTargetingTest, NoContactsReturnsNull) {
    const DigiEntity* target = DoTargeting(digi, pic, self);
    EXPECT_EQ(target, nullptr);
}

TEST_F(DoTargetingTest, PicksHighestThreatScore) {
    SensorContact c1;
    c1.x = 10.0 * 6076.0; c1.y = 0; c1.z = -10000;
    c1.quality = ContactQuality::Detected;
    c1.threatScore = 50.0;
    pic.contacts.push_back(c1);

    SensorContact c2;
    c2.x = 5.0 * 6076.0; c2.y = 0; c2.z = -10000;  // closer → higher threat
    c2.quality = ContactQuality::Detected;
    c2.threatScore = 80.0;
    pic.contacts.push_back(c2);

    const DigiEntity* target = DoTargeting(digi, pic, self);
    ASSERT_NE(target, nullptr);
    // Should pick c2 (higher threatScore).
    EXPECT_NEAR(target->x, 5.0 * 6076.0, 1.0);
}

TEST_F(DoTargetingTest, SkipsMissiles) {
    SensorContact missile;
    missile.x = 5.0 * 6076.0; missile.y = 0; missile.z = -10000;
    missile.isMissile = true;
    missile.threatScore = 100.0;  // highest score, but it's a missile
    pic.contacts.push_back(missile);

    const DigiEntity* target = DoTargeting(digi, pic, self);
    EXPECT_EQ(target, nullptr);  // missile skipped, no other contacts
}

TEST_F(DoTargetingTest, SkipsUndetectedContacts) {
    SensorContact c;
    c.x = 5.0 * 6076.0; c.y = 0; c.z = -10000;
    c.quality = ContactQuality::None;  // not detected
    c.threatScore = 100.0;
    pic.contacts.push_back(c);

    const DigiEntity* target = DoTargeting(digi, pic, self);
    EXPECT_EQ(target, nullptr);  // undetected contact skipped
}

// ===========================================================================
// Loiter mode entry tests (via DigiBrain)
// ===========================================================================
class LoiterEntryTest : public ::testing::Test {
protected:
    DigiBrain brain;
    AircraftState state;
    FlightControlSystem fcs;
    FcsState fcsState;

    void SetUp() override {
        brain.setMaxGs(9.0);
        brain.setMaxBank(45.0);
        brain.setMaxGamma(15.0);
        brain.setTurnG(2.0);
        brain.setCornerSpeed(330.0);

        state.kin.costhe = 1.0; state.kin.cosphi = 1.0;
        state.kin.gmma = 0.0; state.kin.sigma = 0.0; state.kin.singam = 0.0;
        state.kin.x = 0.0; state.kin.y = 0.0; state.kin.z = -10000.0;
        state.kin.zdot = 0.0;
        state.vcas = 350.0; state.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    }
};

TEST_F(LoiterEntryTest, NoWaypointsDefaultsToWaypoint) {
    // No waypoints set → brain defaults to Waypoint (heading/altitude hold).
    // Loiter is NOT auto-entered; it must be explicitly requested via
    // SteeringController::Mode::Loiter or forceMode(Loiter).
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(LoiterEntryTest, WithWaypointsEntersWaypoint) {
    // Set a waypoint → brain should enter Waypoint.
    std::vector<Vec3> wps = {{0.0, 30000.0, -10000.0}};
    brain.setWaypoints(wps);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Waypoint);
}

TEST_F(LoiterEntryTest, ForcedLoiterWorks) {
    // Loiter can be explicitly forced via forceMode.
    brain.forceMode(DigiMode::Loiter);
    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Loiter);
}

TEST_F(LoiterEntryTest, RTBPreemptsWaypoint) {
    // No waypoints + Bingo fuel → RTB should win over Waypoint.
    FrameInputs::AirbaseInfo ab;
    ab.x = 0.0; ab.y = 20.0 * 6076.0; ab.z = -5000.0;
    ab.runwayHeading = 0.0; ab.id = 100;

    FrameInputs fi;
    fi.fuelLbs = 1400.0;
    fi.bingoFuelLbs = 1500.0;
    fi.jokerFuelLbs = 2500.0;
    fi.fumesFuelLbs = 800.0;
    fi.airbases = &ab;
    fi.airbaseCount = 1;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::RTB);
}

TEST_F(LoiterEntryTest, WingyPreemptsWaypoint) {
    // No waypoints + wingman with lead → Wingy should win over Waypoint.
    DigiEntity lead;
    lead.x = 0.0; lead.y = 1000.0; lead.z = -10000.0;
    lead.yaw = 0.0; lead.speed = 350.0 * KNOTS_TO_FTPSEC;

    brain.stateMutable().formation.isWing = true;
    brain.stateMutable().formation.flightLeadId = 1;
    brain.stateMutable().formation.vehicleInUnit = 1;

    FrameInputs fi;
    fi.injectedLead = &lead;
    brain.setFrameInputs(fi);

    brain.compute(state, 1.0/60.0, 0.0, fcs, fcsState);
    EXPECT_EQ(brain.activeMode(), DigiMode::Wingy);
}

// ===========================================================================
// DigiState::reset regression test for Round 7 new fields
// ===========================================================================
TEST(DigiStateResetTestRound7, ClearsRadarModeFields) {
    DigiState s;
    s.weapon.radModeSelect = 0;  // STT
    s.weapon.radarMode = 0;
    s.weapon.lastRadarModeTime = 100.0;

    s.reset();

    EXPECT_EQ(s.weapon.radModeSelect, 3);  // RWS default
    EXPECT_EQ(s.weapon.radarMode, 3);
    EXPECT_NEAR(s.weapon.lastRadarModeTime, -1e9, 1e9);
}
