// f4flight unit tests - flight-phase gain scheduling (digi_state.h)
//
// Tests for:
//   - FlightPhase enum values and names
//   - PhaseGainSet::forPhase() returns distinct gains per phase
//   - Landing/Approach phases have gentler gains than Combat
//   - Formation has pure P (no integral) to prevent windup
//   - Combat has no Phugoid damping (fast response)
//   - DigiNavState.flightPhase defaults to Cruise and resets correctly

#include "f4flight/digi/digi_state.h"
#include <gtest/gtest.h>

using namespace f4flight::digi;

// ===========================================================================
// FlightPhase enum tests
// ===========================================================================

TEST(FlightPhaseTest, EnumValuesAreDistinct) {
    EXPECT_NE(static_cast<int>(FlightPhase::Cruise), static_cast<int>(FlightPhase::Combat));
    EXPECT_NE(static_cast<int>(FlightPhase::Cruise), static_cast<int>(FlightPhase::Formation));
    EXPECT_NE(static_cast<int>(FlightPhase::Approach), static_cast<int>(FlightPhase::Flare));
    EXPECT_NE(static_cast<int>(FlightPhase::Flare), static_cast<int>(FlightPhase::GroundOps));
}

TEST(FlightPhaseTest, NameLookupReturnsCorrectNames) {
    EXPECT_STREQ(flightPhaseName(FlightPhase::Cruise), "Cruise");
    EXPECT_STREQ(flightPhaseName(FlightPhase::Combat), "Combat");
    EXPECT_STREQ(flightPhaseName(FlightPhase::Formation), "Formation");
    EXPECT_STREQ(flightPhaseName(FlightPhase::Approach), "Approach");
    EXPECT_STREQ(flightPhaseName(FlightPhase::Flare), "Flare");
    EXPECT_STREQ(flightPhaseName(FlightPhase::GroundOps), "GroundOps");
}

// ===========================================================================
// PhaseGainSet::forPhase tests
// ===========================================================================

TEST(PhaseGainSetTest, CombatHasHigherGammaGainThanCruise) {
    // Combat needs aggressive pitch response; Cruise needs gentle.
    PhaseGainSet combat = PhaseGainSet::forPhase(FlightPhase::Combat);
    PhaseGainSet cruise = PhaseGainSet::forPhase(FlightPhase::Cruise);
    EXPECT_GT(combat.gammaGain, cruise.gammaGain)
        << "Combat should have higher gamma gain than cruise";
}

TEST(PhaseGainSetTest, CombatHasHigherGammaClampThanApproach) {
    // Combat allows up to 60° pitch; Approach limits to 10° to prevent
    // pitch-up oscillation on the glideslope.
    PhaseGainSet combat = PhaseGainSet::forPhase(FlightPhase::Combat);
    PhaseGainSet approach = PhaseGainSet::forPhase(FlightPhase::Approach);
    EXPECT_GT(combat.gammaClamp, approach.gammaClamp)
        << "Combat should have higher gamma clamp than approach";
}

TEST(PhaseGainSetTest, FormationHasNoIntegralGain) {
    // Formation uses integral gain (0.004) to prevent altitude droop during
    // refueling / station-keeping.
    PhaseGainSet formation = PhaseGainSet::forPhase(FlightPhase::Formation);
    EXPECT_DOUBLE_EQ(formation.integralGain, 0.004)
        << "Formation should have 0.004 integral gain to prevent altitude droop";
}

TEST(PhaseGainSetTest, ApproachHasNoIntegralGain) {
    // Approach also uses pure P to prevent windup during glideslope tracking.
    PhaseGainSet approach = PhaseGainSet::forPhase(FlightPhase::Approach);
    EXPECT_DOUBLE_EQ(approach.integralGain, 0.0)
        << "Approach should have zero integral gain (pure P)";
}

TEST(PhaseGainSetTest, CruiseHasIntegralGain) {
    // Cruise uses integral for steady-state error correction (holding altitude).
    PhaseGainSet cruise = PhaseGainSet::forPhase(FlightPhase::Cruise);
    EXPECT_GT(cruise.integralGain, 0.0)
        << "Cruise should have non-zero integral gain";
}

TEST(PhaseGainSetTest, CombatHasNoPhugoidDamping) {
    // Combat needs fast response — Phugoid damping would slow it down.
    PhaseGainSet combat = PhaseGainSet::forPhase(FlightPhase::Combat);
    EXPECT_DOUBLE_EQ(combat.phugoidGain, 0.0)
        << "Combat should have zero Phugoid damping (fast response)";
}

TEST(PhaseGainSetTest, ApproachHasPhugoidDamping) {
    // Approach needs Phugoid damping to prevent oscillation on the glideslope.
    PhaseGainSet approach = PhaseGainSet::forPhase(FlightPhase::Approach);
    EXPECT_GT(approach.phugoidGain, 0.0)
        << "Approach should have non-zero Phugoid damping";
}

TEST(PhaseGainSetTest, FlareHasHighestPhugoidDamping) {
    // Flare needs the most Phugoid damping for a smooth touchdown.
    PhaseGainSet flare = PhaseGainSet::forPhase(FlightPhase::Flare);
    PhaseGainSet approach = PhaseGainSet::forPhase(FlightPhase::Approach);
    EXPECT_GT(flare.phugoidGain, approach.phugoidGain)
        << "Flare should have higher Phugoid damping than approach";
}

TEST(PhaseGainSetTest, FormationHasLowerRollDampingThanCruise) {
    // Formation needs less roll damping for precise lateral corrections.
    PhaseGainSet formation = PhaseGainSet::forPhase(FlightPhase::Formation);
    PhaseGainSet cruise = PhaseGainSet::forPhase(FlightPhase::Cruise);
    EXPECT_LT(formation.rollDampGain, cruise.rollDampGain)
        << "Formation should have lower roll damping than cruise";
}

TEST(PhaseGainSetTest, FlareHasLowerGammaClampThanApproach) {
    // Flare limits pitch to 5° to prevent pitch-up runaway near the ground.
    PhaseGainSet flare = PhaseGainSet::forPhase(FlightPhase::Flare);
    PhaseGainSet approach = PhaseGainSet::forPhase(FlightPhase::Approach);
    EXPECT_LT(flare.gammaClamp, approach.gammaClamp)
        << "Flare should have lower gamma clamp than approach";
}

// ===========================================================================
// DigiNavState.flightPhase tests
// ===========================================================================

TEST(DigiNavStateTest, FlightPhaseDefaultsToCruise) {
    DigiNavState nav;
    EXPECT_EQ(nav.flightPhase, FlightPhase::Cruise)
        << "flightPhase should default to Cruise";
}

TEST(DigiNavStateTest, ResetClearsFlightPhaseToCruise) {
    DigiNavState nav;
    nav.flightPhase = FlightPhase::Combat;
    nav.reset();
    EXPECT_EQ(nav.flightPhase, FlightPhase::Cruise)
        << "reset() should clear flightPhase to Cruise";
}

TEST(DigiNavStateTest, FlightPhaseCanBeSetAndRead) {
    DigiNavState nav;
    nav.flightPhase = FlightPhase::Approach;
    EXPECT_EQ(nav.flightPhase, FlightPhase::Approach);
    nav.flightPhase = FlightPhase::Flare;
    EXPECT_EQ(nav.flightPhase, FlightPhase::Flare);
}
