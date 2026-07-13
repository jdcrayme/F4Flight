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
// This is a SUBSET of FreeFalcon's 25-mode enum. Modes not yet ported are
// listed in the comment for reference but not in the enum — this prevents
// accidental dispatch to an unimplemented mode.
enum class DigiMode : int {
    // --- Active (ported or stubbed) ---
    GroundAvoid     = 0,   // terrain avoidance (PullUp)
    MissileDefeat   = 1,   // defensive maneuvering vs incoming missile
    GunsJink        = 2,   // defensive jink vs guns fire
    WVREngage       = 3,   // within-visual-range dogfight
    Waypoint        = 4,   // navigation / waypoint following

    // --- Not yet ported (will be added as capabilities land) ---
    // TakeoffMode, LandingMode, RefuelingMode, CollisionAvoid,
    // GunsEngage, MissileEngage, MergeMode, RoopMode, OverBMode,
    // BVREngage, LoiterMode, FollowOrdersMode, RTBMode, WingyMode,
    // BugoutMode, GroundMnvrMode

    NoMode          = 99,  // no active mode
};

// Number of active modes (for array sizing in the dispatcher).
constexpr int kNumDigiModes = 5;

// Return the human-readable name of a mode (for debugging / test output).
inline const char* digiModeName(DigiMode m) {
    switch (m) {
        case DigiMode::GroundAvoid:   return "GroundAvoid";
        case DigiMode::MissileDefeat: return "MissileDefeat";
        case DigiMode::GunsJink:      return "GunsJink";
        case DigiMode::WVREngage:     return "WVREngage";
        case DigiMode::Waypoint:      return "Waypoint";
        case DigiMode::NoMode:        return "NoMode";
    }
    return "Unknown";
}

} // namespace digi
} // namespace f4flight
