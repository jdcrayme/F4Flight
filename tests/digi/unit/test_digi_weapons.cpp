// f4flight unit tests - weapon infrastructure + GunsEngage
//
// Tests for:
//   - WeaponType / WeaponClass / WeaponDomain enums + helpers
//   - WeaponSpec hardcoded envelopes (gun, Aim9, Aim120)
//   - StoresManagementSystem (SMS)
//   - FireControl (CanFire, SelectBestWeapon, ShouldFire)
//   - GunsEngageCheck (mode entry/exit)
//   - GunsEngage (end-to-end: does the AI fire when the target is in the
//     gun envelope?)

#include "f4flight/digi/weapons/weapon_types.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/weapons/fire_control.h"
#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/types.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::digi;

// ===========================================================================
// WeaponType / WeaponClass / WeaponDomain tests
// ===========================================================================
TEST(WeaponTypesTest, HelpersReturnCorrectClass) {
    EXPECT_EQ(weaponClassOf(WeaponType::Aim9), WeaponClass::AimWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::Aim120), WeaponClass::AimWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::Guns), WeaponClass::GunWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::Agm65), WeaponClass::AgmWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::Mk82), WeaponClass::BombWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::GBU), WeaponClass::GbuWpn);
    EXPECT_EQ(weaponClassOf(WeaponType::None), WeaponClass::NoWpn);
}

TEST(WeaponTypesTest, HelpersReturnCorrectDomain) {
    EXPECT_EQ(weaponDomainOf(WeaponType::Aim9), WeaponDomain::Air);
    EXPECT_EQ(weaponDomainOf(WeaponType::Aim120), WeaponDomain::Air);
    EXPECT_EQ(weaponDomainOf(WeaponType::Guns), WeaponDomain::Air);
    EXPECT_EQ(weaponDomainOf(WeaponType::Agm65), WeaponDomain::Ground);
    EXPECT_EQ(weaponDomainOf(WeaponType::Mk82), WeaponDomain::Ground);
    EXPECT_EQ(weaponDomainOf(WeaponType::None), WeaponDomain::NoDomain);
}

TEST(WeaponTypesTest, HelpersReturnCorrectSeeker) {
    EXPECT_EQ(seekerTypeOf(WeaponType::Aim9), SeekerType::IR);
    EXPECT_EQ(seekerTypeOf(WeaponType::Aim120), SeekerType::ARH);
    EXPECT_EQ(seekerTypeOf(WeaponType::Agm88), SeekerType::AntiRadiation);
    EXPECT_EQ(seekerTypeOf(WeaponType::Guns), SeekerType::None);
}

TEST(WeaponTypesTest, IsAirToAir) {
    EXPECT_TRUE(isAirToAir(WeaponType::Aim9));
    EXPECT_TRUE(isAirToAir(WeaponType::Aim120));
    EXPECT_TRUE(isAirToAir(WeaponType::Guns));
    EXPECT_FALSE(isAirToAir(WeaponType::Agm65));
    EXPECT_FALSE(isAirToAir(WeaponType::Mk82));
}

// ===========================================================================
// WeaponSpec tests
// ===========================================================================
TEST(WeaponSpecTest, GunSpecHasCorrectValues) {
    WeaponSpec g = gunSpec();
    EXPECT_EQ(g.type, WeaponType::Guns);
    EXPECT_TRUE(g.isGun());
    EXPECT_FALSE(g.isMissile());
    EXPECT_FALSE(g.isBomb());
    EXPECT_NEAR(g.muzzleVelFtps, 3380.0, 1.0);
    EXPECT_GT(g.roundsRemaining, 0);
    EXPECT_GT(g.maxRangeFt, 0.0);
}

TEST(WeaponSpecTest, Aim9SpecHasCorrectValues) {
    WeaponSpec s = aim9Spec();
    EXPECT_EQ(s.type, WeaponType::Aim9);
    EXPECT_TRUE(s.isMissile());
    EXPECT_EQ(s.seeker, SeekerType::IR);
    EXPECT_GT(s.wezMaxNm, 0.0);
    EXPECT_GT(s.wezMinNm, 0.0);
    EXPECT_GT(s.seekerGimbalRad, 0.0);
    EXPECT_LT(s.wezMinNm, s.wezMaxNm);
}

TEST(WeaponSpecTest, Aim120SpecHasCorrectValues) {
    WeaponSpec s = aim120Spec();
    EXPECT_EQ(s.type, WeaponType::Aim120);
    EXPECT_TRUE(s.isMissile());
    EXPECT_EQ(s.seeker, SeekerType::ARH);
    EXPECT_GT(s.wezMaxNm, aim9Spec().wezMaxNm);  // BVR > WVR
}

TEST(WeaponSpecTest, WeaponSpecOfLookupWorks) {
    EXPECT_EQ(weaponSpecOf(WeaponType::Aim9).type, WeaponType::Aim9);
    EXPECT_EQ(weaponSpecOf(WeaponType::Aim120).type, WeaponType::Aim120);
    EXPECT_EQ(weaponSpecOf(WeaponType::Guns).type, WeaponType::Guns);
    EXPECT_EQ(weaponSpecOf(WeaponType::None).type, WeaponType::None);
}

// ===========================================================================
// SMS tests
// ===========================================================================
TEST(SMSTest, EmptyByDefault) {
    StoresManagementSystem sms;
    EXPECT_EQ(sms.numHardpoints(), 0);
    EXPECT_FALSE(sms.hasWeaponClass(WeaponClass::GunWpn));
    EXPECT_FALSE(sms.hasWeaponType(WeaponType::Aim9));
}

TEST(SMSTest, AddHardpointAndQuery) {
    StoresManagementSystem sms;
    sms.addHardpoint(1, WeaponType::Aim9, 2);
    sms.addHardpoint(9, WeaponType::Guns, 510);

    EXPECT_EQ(sms.numHardpoints(), 2);
    EXPECT_TRUE(sms.hasWeaponClass(WeaponClass::AimWpn));
    EXPECT_TRUE(sms.hasWeaponClass(WeaponClass::GunWpn));
    EXPECT_TRUE(sms.hasWeaponType(WeaponType::Aim9));
    EXPECT_TRUE(sms.hasWeaponType(WeaponType::Guns));

    EXPECT_EQ(sms.remainingOfType(WeaponType::Aim9), 2);
    EXPECT_EQ(sms.remainingOfType(WeaponType::Guns), 510);
    EXPECT_EQ(sms.remainingAirToAir(), 512);  // 2 missiles + 510 rounds

    const Hardpoint* gun = sms.gun();
    ASSERT_NE(gun, nullptr);
    EXPECT_EQ(gun->station, 9);
}

TEST(SMSTest, ExpendDecrementsCount) {
    StoresManagementSystem sms;
    sms.addHardpoint(1, WeaponType::Aim9, 2);

    EXPECT_TRUE(sms.expend(1));
    EXPECT_EQ(sms.remainingOfType(WeaponType::Aim9), 1);
    EXPECT_TRUE(sms.expend(1));
    EXPECT_EQ(sms.remainingOfType(WeaponType::Aim9), 0);
    EXPECT_FALSE(sms.expend(1));  // empty
}

TEST(SMSTest, ExpendOfTypeFindsFirstAvailable) {
    StoresManagementSystem sms;
    sms.addHardpoint(1, WeaponType::Aim120, 1);
    sms.addHardpoint(2, WeaponType::Aim120, 1);

    EXPECT_TRUE(sms.expendOfType(WeaponType::Aim120));
    EXPECT_EQ(sms.remainingOfType(WeaponType::Aim120), 1);
    EXPECT_TRUE(sms.expendOfType(WeaponType::Aim120));
    EXPECT_EQ(sms.remainingOfType(WeaponType::Aim120), 0);
}

// ===========================================================================
// FireControl tests
// ===========================================================================
TEST(FireControlTest, CanFireGunInRangeAndOnNose) {
    WeaponSpec gun = gunSpec();
    FiringEnvelope env;
    env.rangeFt = 2000.0;
    env.ata = 5.0 * DTR;
    EXPECT_TRUE(FireControl::canFire(gun, env));
}

TEST(FireControlTest, CannotFireGunOutOfRange) {
    WeaponSpec gun = gunSpec();
    FiringEnvelope env;
    env.rangeFt = 7000.0;  // > maxRangeFt (6000)
    env.ata = 5.0 * DTR;
    EXPECT_FALSE(FireControl::canFire(gun, env));
}

TEST(FireControlTest, CannotFireGunOffNose) {
    WeaponSpec gun = gunSpec();
    FiringEnvelope env;
    env.rangeFt = 2000.0;
    env.ata = 30.0 * DTR;  // > 20° limit
    EXPECT_FALSE(FireControl::canFire(gun, env));
}

TEST(FireControlTest, CanFireMissileInRange) {
    WeaponSpec msl = aim9Spec();
    FiringEnvelope env;
    env.rangeFt = 3.0 * 6076.0;  // 3 NM (within 0.5–8 NM WEZ)
    env.ata = 10.0 * DTR;        // within ±28° gimbal
    EXPECT_TRUE(FireControl::canFire(msl, env));
}

TEST(FireControlTest, CannotFireMissileOutOfRange) {
    WeaponSpec msl = aim9Spec();
    FiringEnvelope env;
    env.rangeFt = 0.1 * 6076.0;  // 0.1 NM (< 0.5 NM RMin)
    env.ata = 10.0 * DTR;
    EXPECT_FALSE(FireControl::canFire(msl, env));
}

TEST(FireControlTest, CannotFireMissileOffGimbal) {
    WeaponSpec msl = aim9Spec();
    FiringEnvelope env;
    env.rangeFt = 3.0 * 6076.0;
    env.ata = 40.0 * DTR;  // > 0.95 × 28° = 26.6°
    EXPECT_FALSE(FireControl::canFire(msl, env));
}

TEST(FireControlTest, SelectBestWeaponPrefersMissileForBVR) {
    StoresManagementSystem sms;
    sms.addHardpoint(1, WeaponType::Aim120, 1);
    sms.addHardpoint(2, WeaponType::Aim9, 1);
    sms.addHardpoint(9, WeaponType::Guns, 510);

    FiringEnvelope env;
    env.rangeFt = 15.0 * 6076.0;  // 15 NM (BVR)
    env.ata = 10.0 * DTR;

    WeaponType best = FireControl::selectBestWeapon(sms, env);
    EXPECT_EQ(best, WeaponType::Aim120);  // ARH preferred for BVR
}

TEST(FireControlTest, SelectBestWeaponReturnsGunIfOnlyGunInRange) {
    StoresManagementSystem sms;
    sms.addHardpoint(1, WeaponType::Aim120, 1);  // but target too close for Aim120
    sms.addHardpoint(9, WeaponType::Guns, 510);

    FiringEnvelope env;
    env.rangeFt = 1500.0;  // 1500 ft — within gun range, below Aim120 RMin
    env.ata = 5.0 * DTR;

    WeaponType best = FireControl::selectBestWeapon(sms, env);
    EXPECT_EQ(best, WeaponType::Guns);
}

TEST(FireControlTest, ShouldFireShootShootWaits4Seconds) {
    // Shoot-shoot: 2nd missile 4 seconds after 1st
    EXPECT_FALSE(FireControl::shouldFire(SeekerType::ARH, 0.8, 2.0, 10.0));
    EXPECT_TRUE(FireControl::shouldFire(SeekerType::ARH, 0.8, 4.0, 10.0));
    EXPECT_TRUE(FireControl::shouldFire(SeekerType::ARH, 0.8, 5.0, 10.0));
}

TEST(FireControlTest, ShouldFireShootLookWaitsTofPlusBuffer) {
    // Shoot-look: wait TOF + 5 + min(TOF*0.5, 5) = 10 + 5 + 5 = 20s
    EXPECT_FALSE(FireControl::shouldFire(SeekerType::ARH, 0.3, 19.0, 10.0));
    EXPECT_TRUE(FireControl::shouldFire(SeekerType::ARH, 0.3, 20.0, 10.0));
}

// ===========================================================================
// GunsEngageCheck tests
// ===========================================================================
class GunsEngageCheckTest : public ::testing::Test {
protected:
    DigiEntity self;
    DigiEntity target;
    WeaponSpec gun;

    void SetUp() override {
        gun = gunSpec(510);

        self.x = 0; self.y = 0; self.z = -10000;
        self.yaw = 0; self.speed = 500;
        self.vx = 500; self.vy = 0; self.vz = 0;

        // Target 2000 ft ahead, co-altitude
        target.x = 2000; target.y = 0; target.z = -10000;
        target.yaw = PI; target.speed = 400;  // heading toward us (head-on)
        target.vx = -400; target.vy = 0; target.vz = 0;
    }
};

TEST_F(GunsEngageCheckTest, EntersWhenTargetInRangeAndOnNose) {
    DigiState digi;
    EXPECT_TRUE(GunsEngageCheck(digi, self, target, gun, true));
}

TEST_F(GunsEngageCheckTest, RejectsWhenTargetOutOfRange) {
    target.x = 5000;  // > 3500 ft
    DigiState digi;
    EXPECT_FALSE(GunsEngageCheck(digi, self, target, gun, true));
}

TEST_F(GunsEngageCheckTest, RejectsWhenGunEmpty) {
    gun.roundsRemaining = 0;
    DigiState digi;
    EXPECT_FALSE(GunsEngageCheck(digi, self, target, gun, true));
}

TEST_F(GunsEngageCheckTest, RejectsWhenTargetTooFarOffNose) {
    // Target at 90° off nose
    target.x = 0; target.y = 2000;
    DigiState digi;
    EXPECT_FALSE(GunsEngageCheck(digi, self, target, gun, true));
}

// ===========================================================================
// GunsEngage end-to-end tests
// ===========================================================================
class GunsEngageTest : public ::testing::Test {
protected:
    DigiState digi;
    DigiEntity self;
    DigiEntity target;
    AircraftState as;
    FlightControlSystem fcs;
    FcsState fcsState;
    WeaponSpec gun;

    void SetUp() override {
        digi.reset();
        digi.skill = makeSkillParams(SkillLevel::Veteran);
        digi.maxGs = 9.0;
        digi.cornerSpeed = 330.0;
        digi.maxRoll = 190.0;
        digi.maxGammaDeg = 15.0;
        digi.turnLoadFactor = 2.0;
        digi.dt = 1.0 / 60.0;

        gun = gunSpec(510);

        // Self at origin, heading east, level, 350 kts
        self.x = 0.0; self.y = 0.0; self.z = -10000.0;
        self.yaw = 0.0; self.pitch = 0.0; self.roll = 0.0;
        self.vx = 589.0; self.vy = 0.0; self.vz = 0.0;
        self.speed = 589.0;

        // Target 2000 ft ahead, head-on
        target.x = 2000.0; target.y = 0.0; target.z = -10000.0;
        target.yaw = PI; target.speed = 400.0;
        target.vx = -400.0; target.vy = 0.0; target.vz = 0.0;

        // Aircraft state matching self
        as.kin.costhe = 1.0; as.kin.cosphi = 1.0;
        as.kin.gmma = 0.0; as.kin.sigma = 0.0;
        as.kin.x = 0.0; as.kin.y = 0.0; as.kin.z = -10000.0;
        as.kin.xdot = 589.0; as.kin.ydot = 0.0; as.kin.zdot = 0.0;
        as.kin.phi = 0.0; as.kin.psi = 0.0; as.kin.theta = 0.0;
        as.kin.vt = 589.0; as.vcas = 350.0;
        as.kin.dcm = Matrix3::identity();
        as.aero.alpha_deg = 4.0;
        as.loads.nzcgb = 1.0;
    }
};

TEST_F(GunsEngageTest, ProducesStickCommands) {
    GunsEngage(digi, self, target, as, gun, fcs, fcsState, 1.0/60.0);

    // Should produce some stick/throttle commands
    bool hasCommand = (std::fabs(digi.pStick) > 0.01 ||
                       std::fabs(digi.rStick) > 0.01 ||
                       digi.throttle > 0.01);
    EXPECT_TRUE(hasCommand);
}

TEST_F(GunsEngageTest, DoesNotFireImmediately) {
    // On the first frame, the AI should be in coarse track (not firing)
    GunsEngage(digi, self, target, as, gun, fcs, fcsState, 1.0/60.0);
    EXPECT_FALSE(digi.gunFireFlag)
        << "AI should not fire on the first frame (needs to track first)";
}

TEST_F(GunsEngageTest, CanFireAfterTracking) {
    // Run several frames to let the AI track the target.
    // With a head-on target at 2000 ft, the pipper should converge and
    // the AI should fire within a few seconds.
    bool fired = false;
    for (int i = 0; i < 180; ++i) {  // 3 seconds at 60 Hz
        // Update target position (closing head-on)
        target.x -= 400.0 * (1.0/60.0);
        as.kin.x = self.x;
        GunsEngage(digi, self, target, as, gun, fcs, fcsState, 1.0/60.0);
        if (digi.gunFireFlag) {
            fired = true;
            break;
        }
    }
    // The AI should fire at some point during a 3-second head-on pass
    // at 2000 ft. (If this fails, the pipper tracking may need tuning —
    // but the infrastructure is correct.)
    EXPECT_TRUE(fired) << "AI did not fire during a 3-second head-on pass";
}
