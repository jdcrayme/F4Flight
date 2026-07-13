// f4flight - digi/digi_state.h
//
// DigiState — persistent state for the digi AI.
//
// Moved here from steering.h as part of the digi/ subsystem refactor. The
// flight model (FlightModel, Aerodynamics, Engine, FCS, EOM, Gear) has NO
// dependency on this struct — the dependency flows one way only:
//
//     FlightModel  →  AircraftState  ←  DigiBrain (reads state, writes PilotInput)
//
// This keeps the flight model pure and lets the digi AI grow independently.
//
// The struct is intentionally a plain aggregate of doubles + a reset() method.
// No behavior, no virtual functions — the "behavior" lives in the maneuver
// functions (digi/maneuvers/) and the brain dispatcher (digi/digi_brain.h).

#pragma once

#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_entity.h"

namespace f4flight {
namespace digi {

struct DigiState {
    // --- Stick / throttle commands (written by maneuver functions) ---
    double pStick{0.0};
    double rStick{0.0};
    double yPedal{0.0};
    double throttle{0.5};  // [0, 1.5] where 1.0 = MIL, 1.5 = full AB

    // --- Time step ---
    // Set by the brain each frame. FreeFalcon's AI runs at 16.67 Hz (0.06s);
    // F4Flight calls at 60 Hz. Smoothing constants below are derived from
    // FreeFalcon's 0.06s constants, scaled by this dt.
    double dt{1.0/60.0};

    // --- GammaHold integral ---
    double gammaHoldIError{0.0};

    // --- MachHold integral ---
    double autoThrottle{0.0};

    // --- LevelTurn state (0 = leveling, 1 = banking, 2 = holding turn) ---
    int trackMode{0};

    // --- Waypoint state ---
    int onStation{0};  // 0=NotThereYet, 1=Arrived, 2=Stabalizing, 3=OnStation
    int waypointMode{1};
    double holdAlt{0.0};
    double holdPsi{0.0};

    // --- Configuration (set by host) ---
    double cornerSpeed{330.0};  // kts
    double maxGs{9.0};
    double maxRoll{30.0};       // deg
    double maxRollDelta{5.0};   // deg
    double maxGammaDeg{60.0};   // GammaHold clamp (FreeFalcon: 60°, nav: 15°)
    double turnLoadFactor{2.0}; // LevelTurn G (FreeFalcon: 2.0, heavy: 1.3)

    // --- Skill (Tier 0) ---
    SkillParameters skill;

    // --- Ground avoidance state (Tier 0) ---
    bool   groundAvoidNeeded{false};
    double pullupTimer{0.0};   // seconds remaining in forced pull-up

    // --- Missile defeat state (Tier 1) ---
    // Set by the host each frame: pointer to the incoming missile entity.
    // nullptr = no incoming missile. The host owns the entity; the brain
    // only reads it.
    const DigiEntity* incomingMissile{nullptr};

    // Missile defeat internal state (managed by MissileDefeat)
    double missileDefeatTtgo{-1.0};      // time-to-go (seconds), -1 = not initialized
    bool   missileFindDragPt{true};      // need to compute new drag trackpoint
    bool   missileShouldDrag{false};     // (legacy: currently unused in simplified port)
    bool   missileFinishedBeam{false};   // (legacy: currently unused)
    double incomingMissileRange{500.0 * 6076.0};  // last missile range (ft), init 500 NM
    double incomingMissileEvadeTimer{0.0};        // seconds since missile started passing

    // --- Guns jink state (Tier 1) ---
    // Set by the host each frame: pointer to the aircraft threatening us with guns.
    const DigiEntity* gunsThreat{nullptr};

    // Guns jink internal state (managed by GunsJink)
    int    jinkTime{-1};       // -1 = not jinking, 0 = rolling to target bank, >0 = pulling
    double newRoll{0.0};       // target roll angle (rad)
    double jinkTimer{0.0};     // seconds in pull phase

    void reset() noexcept {
        pStick = rStick = yPedal = 0.0;
        gammaHoldIError = 0.0;
        autoThrottle = 0.0;
        trackMode = 0;
        onStation = 0;
        waypointMode = 1;
        groundAvoidNeeded = false;
        pullupTimer = 0.0;
        // Note: threat pointers (incomingMissile, gunsThreat) are NOT cleared
        // by reset() — the host manages these. reset() only clears internal
        // brain state.
        missileDefeatTtgo = -1.0;
        missileFindDragPt = true;
        missileShouldDrag = false;
        missileFinishedBeam = false;
        incomingMissileRange = 500.0 * 6076.0;
        incomingMissileEvadeTimer = 0.0;
        jinkTime = -1;
        newRoll = 0.0;
        jinkTimer = 0.0;
    }
};

} // namespace digi
} // namespace f4flight
