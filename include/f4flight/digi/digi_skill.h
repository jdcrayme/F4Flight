// f4flight - digi/digi_skill.h
//
// SkillParameters — pilot skill scaling for the digi AI.
//
// FreeFalcon's DigitalBrain uses a 0-4 skill level (Recruit, Rookie, Veteran,
// Ace) at ~30 call sites to scale:
//   - GCI spotting range (aces get GCI, recruits don't)
//   - shoot-shoot vs shoot-look missile doctrine probability
//   - missile RMax multiplier (aces use tighter RMax)
//   - jettison chance on missile defeat (skill * 25%)
//   - radar mode re-evaluation interval
//   - IR-missile idle-throttle (only skill > 2)
//   - reaction time (lower skill = slower maneuver recomputation)
//
// Rather than threading a raw int everywhere, we bundle the derived parameters
// into a struct so each call site reads a named field. This makes the behavior
// explicit and tunable per-skill without scattered magic numbers.
//
// Direct port concept from FreeFalcon simbrain.h:14 (BaseBrain::skillLevel)
// and the ~30 SkillLevel() call sites in the digi source.

#pragma once

namespace f4flight {
namespace digi {

// Raw skill levels (match FreeFalcon's 0-4 scale).
enum class SkillLevel : int {
    Recruit = 0,
    Rookie  = 1,
    Veteran = 2,
    Ace     = 3,
    SuperAce = 4  // FreeFalcon's "Ace" is 4 in some code paths; we expose both
};

// Derived skill parameters. Constructed from a SkillLevel via makeSkillParams().
// All fields are in engineering units so call sites don't need to do math.
struct SkillParameters {
    SkillLevel level{SkillLevel::Veteran};

    // --- Sensors ---
    bool   gciCapable{true};         // GCI spotting (Veteran and above)
    double gciRangeNm{30.0};         // GCI range (nm)
    double sensorTimeoutSec{6.0};    // contact tracking timeout (scales with skill)

    // --- Weapons ---
    double shootShootPctRadar{0.5};  // shoot-shoot doctrine for radar missiles
    double shootShootPctHeat{0.4};   // shoot-shoot doctrine for heat missiles
    double rMaxMultiplier{1.0};      // scales missile RMax (aces use tighter)
    bool   smartTargetSelection{true}; // skill > 0: avoid random re-targeting

    // --- Defensive ---
    double jettisonChanceOnDefeat{0.5};  // skill * 0.25
    bool   irMissileThrottleCut{true};   // only skill > 2
    double evadeHoldSec{4.0};            // (6 - skill) seconds defensive hold

    // --- Reaction time ---
    double reactionTimeSec{3.0};     // maneuver recomputation interval
    double radarReevalSec{4.0};      // radar mode re-evaluation interval
};

// Construct derived parameters from a raw skill level.
// Centralizes the skill-scaling formulas so they can be tuned in one place.
inline SkillParameters makeSkillParams(SkillLevel level) {
    const int s = static_cast<int>(level);
    SkillParameters p;
    p.level = level;

    // GCI: Veteran (2) and above
    p.gciCapable = (s >= 2);
    p.gciRangeNm = 30.0;
    p.sensorTimeoutSec = 6.0 * (s + 1);  // 6, 12, 18, 24, 30 s

    // Weapons doctrine: higher skill = more aggressive shoot-shoot
    p.shootShootPctRadar = 0.3 + 0.15 * s;  // 0.30, 0.45, 0.60, 0.75, 0.90
    p.shootShootPctHeat  = 0.2 + 0.12 * s;  // 0.20, 0.32, 0.44, 0.56, 0.68
    p.rMaxMultiplier = 0.95 - 0.05 * s;     // 0.95, 0.90, 0.85, 0.80, 0.75
    p.smartTargetSelection = (s > 0);

    // Defensive
    p.jettisonChanceOnDefeat = 0.25 * s;   // 0, 0.25, 0.50, 0.75, 1.00
    p.irMissileThrottleCut = (s > 2);      // Ace and above
    p.evadeHoldSec = 6.0 - s;              // 6, 5, 4, 3, 2 s

    // Reaction time: higher skill = faster
    p.reactionTimeSec = 4.0 - 0.5 * s;     // 4.0, 3.5, 3.0, 2.5, 2.0
    p.radarReevalSec = 4.0 + (4 - s);      // 8, 7, 6, 5, 4

    return p;
}

} // namespace digi
} // namespace f4flight
