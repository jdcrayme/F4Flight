// f4flight - digi/digi_mode.h
//
// DigiMode — the priority-ordered mode stack for the digi AI.
//
// FreeFalcon's DigitalBrain uses a priority-ordered mode enum (digi.h:222).
// Lower numerical values = higher priority. The brain's FrameExec() dispatches
// to the per-mode maneuver function via a giant switch. AddMode() enforces
// that a lower-priority mode cannot pre-empt one already queued.
//
// We start with a 4-mode skeleton (Waypoint, GroundAvoid, MissileDefeat,
// WVREngage) and grow it as more capabilities are ported. The full FreeFalcon
// mode stack has ~25 entries; see the comment in DigiMode for the complete
// list.
//
// Port of FreeFalcon digimode.h / dlogic.cpp:729 (AddMode / ResolveModeConflicts).

#pragma once

#include <cstdint>

namespace f4flight {
namespace digi {

// DigiMode — priority-ordered modes. Lower value = higher priority.
// The mode stack is evaluated top-to-bottom; the first active mode wins.
//
// Round-2 structural fix (Rec 6): added the 9 missing DigiMode values.
// They now have dispatch targets (the 4 new maneuver primitives from
// Rec 10, plus existing primitives). The dispatch is stubbed — each new
// mode falls through to Waypoint until its behavior is ported. This
// lets the brain's resolveMode() recognize the modes (so future porting
// work can incrementally wire them up) without producing dead code.
enum class DigiMode : int {
    // --- Active (ported or stubbed) ---
    GroundAvoid     = 0,   // terrain avoidance (PullUp)
    MissileDefeat   = 1,   // defensive maneuvering vs incoming missile
    GunsJink        = 2,   // defensive jink vs guns fire
    CollisionAvoid  = 3,   // air-air collision avoidance
    Landing         = 4,   // landing approach + rollout
    Takeoff         = 5,   // takeoff roll + climbout
    MissileEngage   = 6,   // offensive missile firing (A/A)
    GunsEngage      = 7,   // offensive gun tracking + firing
    Merge           = 8,   // first-pass merge geometry maneuver
    Accel           = 9,   // regain corner speed after merge
    WVREngage       = 10,  // within-visual-range dogfight (RollAndPull)
    BVREngage       = 11,  // beyond-visual-range engagement
    Waypoint        = 12,  // navigation / waypoint following

    // --- Round-2 structural additions (Rec 6): the 9 missing modes.
    // Dispatch stubs exist in DigiBrain::compute() — each falls through
    // to Waypoint navigation until its behavior is ported. This lets
    // resolveMode() recognize the modes (so future porting work can
    // incrementally wire them up) without producing dead code.
    Refueling       = 13,  // air refueling (needs refuel.cpp port + tanker entity)
    Separate        = 14,  // disengage from fight (needs separate.cpp)
    Roop            = 15,  // roll out of plane (RoopMode — uses RollOutOfPlane)
    OverB           = 16,  // over-bank for separation (uses OverBank)
    Loiter          = 17,  // holding pattern (uses ManeuverPrimitives::Loiter)
    FollowOrders    = 18,  // wingman following orders (needs wingman system)
    RTB             = 19,  // return to base (needs AirbaseCheck + bingo fuel)
    Wingy           = 20,  // wingman formation flying (needs formdata.cpp)
    Bugout          = 21,  // bug out from fight (uses WvrBugOut)
    GroundMnvr      = 22,  // A/G attack (needs gndattck.cpp port)

    NoMode          = 99,  // no active mode
};

// Number of active modes (for array sizing in the dispatcher).
constexpr int kNumDigiModes = 23;

// Return the human-readable name of a mode (for debugging / test output).
inline const char* digiModeName(DigiMode m) {
    switch (m) {
        case DigiMode::GroundAvoid:   return "GroundAvoid";
        case DigiMode::MissileDefeat: return "MissileDefeat";
        case DigiMode::GunsJink:      return "GunsJink";
        case DigiMode::CollisionAvoid:return "CollisionAvoid";
        case DigiMode::Landing:       return "Landing";
        case DigiMode::Takeoff:       return "Takeoff";
        case DigiMode::MissileEngage: return "MissileEngage";
        case DigiMode::GunsEngage:    return "GunsEngage";
        case DigiMode::Merge:         return "Merge";
        case DigiMode::Accel:         return "Accel";
        case DigiMode::WVREngage:     return "WVREngage";
        case DigiMode::BVREngage:     return "BVREngage";
        case DigiMode::Waypoint:      return "Waypoint";
        // Round-2 additions
        case DigiMode::Refueling:     return "Refueling";
        case DigiMode::Separate:      return "Separate";
        case DigiMode::Roop:          return "Roop";
        case DigiMode::OverB:         return "OverB";
        case DigiMode::Loiter:        return "Loiter";
        case DigiMode::FollowOrders:  return "FollowOrders";
        case DigiMode::RTB:           return "RTB";
        case DigiMode::Wingy:         return "Wingy";
        case DigiMode::Bugout:        return "Bugout";
        case DigiMode::GroundMnvr:    return "GroundMnvr";
        case DigiMode::NoMode:        return "NoMode";
    }
    return "Unknown";
}

} // namespace digi
} // namespace f4flight
