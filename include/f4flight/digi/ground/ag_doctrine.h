// f4flight - digi/ground/ag_doctrine.h
//
// Air-to-Ground doctrine + approach enums.
//
// Port of FreeFalcon's AG doctrine + approach enums (gndattck.cpp:4,930 LOC).
// F4Flight has not yet ported gndattck.cpp — this header exists so that
// future porting work has the enum types ready, and so the DigiState can
// already carry agDoctrine / agApproach fields (Round-2 structural fix —
// DIGI_AUDIT.md Rec 9 / Gap 2).
//
// Once gndattck.cpp is ported, GroundAttackMode will dispatch on agApproach
// to select the dive/toss/level/bomber profile, and on agDoctrine to decide
// shoot-run vs look-shoot-look vs setup-required.

#pragma once

namespace f4flight {
namespace digi {
namespace ag {

// A/G doctrine — when to shoot, how much setup is required.
// Direct port of FreeFalcon AG_DOCTRINE enum (gndattck.cpp).
enum Doctrine {
    AGD_NONE           = 0,  // no doctrine (default — no A/G attack)
    AGD_SHOOT_RUN      = 1,  // single pass, no reattack
    AGD_LOOK_SHOOT_LOOK= 2,  // attack, assess, reattack if needed
    AGD_NEED_SETUP     = 3,  // requires IP + profile setup before attack
};

// A/G approach profile — how the weapon is delivered.
// Direct port of FreeFalcon AG_APPROACH enum (gndattck.cpp).
enum Approach {
    AGA_NONE   = 0,  // no approach selected
    AGA_LOW    = 1,  // low-level level delivery (dumb bombs, guns)
    AGA_TOSS   = 2,  // toss bombing (pull-up + release in climb)
    AGA_HIGH   = 3,  // high-altitude level delivery
    AGA_DIVE   = 4,  // dive toss / dive bombing (CCIP pipper)
    AGA_BOMBER = 5,  // level bomber pattern (B-52, B-1, B-2)
};

// Returns a human-readable name for the doctrine (debugging / test output).
inline const char* doctrineName(Doctrine d) {
    switch (d) {
        case AGD_NONE:            return "None";
        case AGD_SHOOT_RUN:       return "ShootRun";
        case AGD_LOOK_SHOOT_LOOK: return "LookShootLook";
        case AGD_NEED_SETUP:      return "NeedSetup";
    }
    return "Unknown";
}

// Returns a human-readable name for the approach (debugging / test output).
inline const char* approachName(Approach a) {
    switch (a) {
        case AGA_NONE:   return "None";
        case AGA_LOW:    return "Low";
        case AGA_TOSS:   return "Toss";
        case AGA_HIGH:   return "High";
        case AGA_DIVE:   return "Dive";
        case AGA_BOMBER: return "Bomber";
    }
    return "Unknown";
}

} // namespace ag
} // namespace digi
} // namespace f4flight
