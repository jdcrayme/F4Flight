// f4flight - digi/ground/ag_attack_phase.h
//
// AgAttackPhase — the A/G attack state machine phases.
//
// FreeFalcon reuses the WaypointState onStation enum for BOTH the landing
// pattern AND the A/G attack phases (digi.h:260-264). This is FF quirk #19
// — one variable, three semantic roles (landing pattern, A/G attack, taxi),
// and it's a known source of confusion.
//
// F4Flight uses a SEPARATE enum for A/G attack to avoid repeating
// FreeFalcon's mistake. The GroundOpsPhase enum (ground_ops.h) owns
// takeoff/landing; this enum owns the A/G attack state machine.
//
// Port of FreeFalcon's onStation A/G phases (gndattck.cpp:96-3216).
// Round-3 structural addition (DIGI_AUDIT_ROUND3.md §3.3).

#pragma once

namespace f4flight {
namespace digi {

// AgAttackPhase — the phases of a ground-attack run.
//
// Sequence: NotThereYet → (Crosswind | HoldInPlace) → Downwind → Base →
//           Final → Final1 → Stabalizing → back to NotThereYet (for
//           LOOK_SHOOT_LOOK) or mission complete (for SHOOT_RUN).
//
// This matches FreeFalcon's onStation values used by gndattck.cpp, but as
// a dedicated enum so the A/G state machine is self-documenting and can't
// accidentally collide with landing pattern phases.
enum class AgAttackPhase : int {
    NotThereYet   = 0,  // tracking to IP, not yet at the target area
    HoldInPlace   = 1,  // holding near the target, selecting weapon
    Crosswind     = 2,  // perpendicular run-in to target
    Downwind      = 3,  // fly-out away from target to set up another run
    Base          = 4,  // base turn to final
    Final         = 5,  // weapon release phase (dive/level/toss)
    Final1        = 6,  // post-release tracking and escape
    Stabalizing   = 7,  // stabilize at altitude after pull-out
};

inline const char* agAttackPhaseName(AgAttackPhase p) {
    switch (p) {
        case AgAttackPhase::NotThereYet: return "NotThereYet";
        case AgAttackPhase::HoldInPlace: return "HoldInPlace";
        case AgAttackPhase::Crosswind:   return "Crosswind";
        case AgAttackPhase::Downwind:     return "Downwind";
        case AgAttackPhase::Base:         return "Base";
        case AgAttackPhase::Final:        return "Final";
        case AgAttackPhase::Final1:       return "Final1";
        case AgAttackPhase::Stabalizing:  return "Stabalizing";
    }
    return "Unknown";
}

// AgAttackProfile — which delivery geometry runGroundAttack() executes.
//
// Task 11 added a single dive-bomb profile; Task 15-a expands this to three
// profiles. The host selects the profile via FrameInputs.injectedAgProfile
// (committed to state_.ag.agProfile by DigiBrain::setFrameInputs).
//
//   DiveBomb       — the original Task 11 profile (default for backward
//                    compatibility). 4-phase state machine:
//                    approach → dive → pullout → egress.
//   LevelDelivery  — level bombing against soft/area targets. Flies level
//                    over the target at low altitude and releases when
//                    directly overhead. approach → level → release → egress.
//   TossBomb       — loft (toss) bombing for standoff delivery. Approaches
//                    low and fast, pulls up into a 4G climb, releases at
//                    the apex (45° pitch or 3000 ft AGL), and continues the
//                    climb to egress altitude.
//                    approach → pull-up → release → egress.
enum class AgAttackProfile : int {
    DiveBomb       = 0,
    LevelDelivery  = 1,
    TossBomb       = 2,
};

inline const char* agAttackProfileName(AgAttackProfile p) {
    switch (p) {
        case AgAttackProfile::DiveBomb:      return "DiveBomb";
        case AgAttackProfile::LevelDelivery: return "LevelDelivery";
        case AgAttackProfile::TossBomb:      return "TossBomb";
    }
    return "Unknown";
}

} // namespace digi
} // namespace f4flight
