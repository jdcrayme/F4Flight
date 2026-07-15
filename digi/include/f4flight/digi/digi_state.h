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
// The struct is a plain aggregate of sub-structs + a reset() method.
// No behavior, no virtual functions — the "behavior" lives in the maneuver
// functions (digi/maneuvers/) and the brain dispatcher (digi/digi_brain.h).
//
// ---------------------------------------------------------------------------
// SUB-STRUCT LAYOUT
// ---------------------------------------------------------------------------
// DigiState is composed of 10 sub-structs, each owning one domain of state:
//
//   commands       — stick/throttle/brake/gear outputs (written by maneuvers)
//   config         — persistent config (skill, cornerSpeed, maxGs, etc.)
//   nav            — navigation state (dt, holdAlt/Psi, trackMode, trackX/Y/Z)
//   groundAvoid    — ground avoidance state (groundAvoidNeeded, pullupTimer)
//   missileDefeat  — incoming missile + defeat state
//   gunsJink       — guns threat + jink state
//   weapon         — fire control + merge/missile/guns engage state
//   ag             — air-to-ground target + ground ops + A/G attack phase
//   formation      — wingman/formation state (flightLeadId, isWing, wingman)
//   comm           — communication (mailbox, selfId)
//
// Access pattern: digi.commands.pStick, digi.config.maxGs, digi.nav.trackX, etc.
// ---------------------------------------------------------------------------

#pragma once

#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/wingman/wingman_state.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// Sub-structs
// ===========================================================================

// Stick / throttle / brake / gear commands — written by maneuver functions,
// read by DigiBrain::compute() to produce PilotInput.
struct DigiCommands {
    double pStick{0.0};
    double rStick{0.0};
    double yPedal{0.0};
    double throttle{0.5};  // [0, 1.5] where 1.0 = MIL, 1.5 = full AB

    // Brake / speed-brake / gear commands. Convention matches PilotInput:
    // speedBrakeCmd = -1 (retracted, no drag) .. +1 (full extend)
    // gearHandleCmd = -1 (up) .. +1 (down)
    bool   wheelBrakes{false};
    bool   parkingBrake{false};
    double speedBrakeCmd{-1.0};     // default retracted
    double gearHandleCmd{1.0};      // default down

    void reset() noexcept {
        pStick = rStick = yPedal = 0.0;
        throttle = 0.5;
        wheelBrakes = false;
        parkingBrake = false;
        speedBrakeCmd = -1.0;
        gearHandleCmd = 1.0;
    }
};

// Persistent configuration — set once at init via DigiBrain::configure().
struct DigiConfigState {
    SkillParameters skill;
    double cornerSpeed{330.0};  // kts
    double maxGs{9.0};
    double maxRoll{30.0};       // deg — navigation bank limit
    double maxRollDelta{5.0};   // deg — roll-rate damping window (DAT-PRESERVED)
    double maxGammaDeg{60.0};   // GammaHold clamp (FreeFalcon: 60°, nav: 15°)
    double turnLoadFactor{2.0}; // LevelTurn G (FreeFalcon: 2.0, heavy: 1.3)

    // NOTE: reset() does NOT clear config — config persists across mode resets.
    // Only DigiBrain::reset() (full brain reset) clears it via value-init.
};

// Navigation state — waypoint following, track points, maneuver timers.
struct DigiNavState {
    double dt{1.0/60.0};       // set by brain each frame
    double holdAlt{0.0};       // held altitude (ft, positive up)
    double holdPsi{0.0};       // held heading (rad)
    int    onStation{0};       // 0=NotThereYet, 1=Arrived, 2=Stabalizing, 3=OnStation
    int    waypointMode{1};
    int    trackMode{0};       // LevelTurn: 0=leveling, 1=banking, 2=holding
    double gammaHoldIError{0.0};  // GammaHold integral
    double autoThrottle{0.0};     // MachHold integral
    double mnverTime{0.0};        // RoopMode/OverBMode/MergeManeuver timer (s)
    double trackX{0.0};           // track point (used by AutoTrack, TrackPointLanding)
    double trackY{0.0};
    double trackZ{0.0};

    void reset() noexcept {
        dt = 1.0/60.0;
        holdAlt = 0.0;
        holdPsi = 0.0;
        onStation = 0;
        waypointMode = 1;
        trackMode = 0;
        gammaHoldIError = 0.0;
        autoThrottle = 0.0;
        mnverTime = 0.0;
        trackX = trackY = trackZ = 0.0;
    }
};

// Ground avoidance state — concurrent overlay, not a dispatched mode.
struct DigiGroundAvoidState {
    bool   groundAvoidNeeded{false};
    double pullupTimer{0.0};   // seconds remaining in forced pull-up

    void reset() noexcept {
        groundAvoidNeeded = false;
        pullupTimer = 0.0;
    }
};

// Missile defeat state — incoming missile tracking + defeat maneuver state.
struct DigiMissileDefeatState {
    const DigiEntity* incomingMissile{nullptr};
    EntityId incomingMissileId{kInvalidEntityId};
    double missileDefeatTtgo{-1.0};      // -1 = not initialized
    bool   missileFindDragPt{true};
    bool   missileShouldDrag{false};     // DAT-PRESERVED (legacy)
    bool   missileFinishedBeam{false};   // DAT-PRESERVED (legacy)
    double incomingMissileRange{500.0 * 6076.0};  // init 500 NM sentinel
    double incomingMissileEvadeTimer{0.0};

    void reset() noexcept {
        // NOTE: incomingMissile pointer is NOT cleared by reset() —
        // the host manages it. reset() only clears internal state.
        incomingMissileId = kInvalidEntityId;
        missileDefeatTtgo = -1.0;
        missileFindDragPt = true;
        missileShouldDrag = false;
        missileFinishedBeam = false;
        incomingMissileRange = 500.0 * 6076.0;
        incomingMissileEvadeTimer = 0.0;
    }
};

// Guns jink state — guns threat tracking + jink maneuver state.
struct DigiGunsJinkState {
    const DigiEntity* gunsThreat{nullptr};
    int    jinkTime{-1};       // -1 = not jinking, 0 = rolling, >0 = pulling
    double newRoll{0.0};       // target roll angle (rad)
    double jinkTimer{0.0};     // seconds in pull phase

    void reset() noexcept {
        // NOTE: gunsThreat pointer is NOT cleared by reset() — host manages it.
        jinkTime = -1;
        newRoll = 0.0;
        jinkTimer = 0.0;
    }
};

// Weapon / fire control state — merge, missile engage, guns engage.
struct DigiWeaponState {
    double mergeTimer{-1.0};          // -1 = not in merge, >0 = seconds remaining
    bool   gunFireFlag{false};        // fire the internal gun this frame
    bool   mslFireFlag{false};        // release a missile this frame
    int    fireStation{0};            // which hardpoint to release from
    double missileShotTimer{0.0};     // seconds since last missile fired
    bool   inShootShoot{false};       // currently in shoot-shoot doctrine
    bool   waitingForShot{false};     // in fine-track firing phase
    double pastPstick{0.0};           // G held when entering fine track
    double pastAta{0.0};              // previous frame's pipper ATA (rad)
    double pastPipperAta{0.0};        // previous frame's pipper ATA (rad)
    double ataDot{0.0};               // rate of change of ATA (rad/s)
    double maxAAWpnRange{0.0};        // set by host from SMS; 0 = no missiles

    void reset() noexcept {
        mergeTimer = -1.0;
        gunFireFlag = false;
        mslFireFlag = false;
        fireStation = 0;
        missileShotTimer = 0.0;
        inShootShoot = false;
        waitingForShot = false;
        pastPstick = 0.0;
        pastAta = 0.0;
        pastPipperAta = 0.0;
        ataDot = 0.0;
        // maxAAWpnRange is NOT reset — it's config-like (set by host from SMS)
    }
};

// Air-to-ground state — ground target, doctrine, ground ops, A/G attack phase.
struct DigiGroundState {
    const DigiEntity* groundTarget{nullptr};
    EntityId groundTargetId{kInvalidEntityId};
    int      agDoctrine{0};    // AGD_NONE / SHOOT_RUN / LOOK_SHOOT_LOOK / NEED_SETUP
    int      agApproach{0};    // AGA_LOW / TOSS / HIGH / DIVE / BOMBER
    bool     reachedIP{false}; // true once past the Initial Point
    GroundOpsState groundOps;
    AgAttackPhase  agAttackPhase{AgAttackPhase::NotThereYet};

    void reset() noexcept {
        groundTarget = nullptr;
        groundTargetId = kInvalidEntityId;
        agDoctrine = 0;
        agApproach = 0;
        reachedIP = false;
        groundOps.reset();
        agAttackPhase = AgAttackPhase::NotThereYet;
    }
};

// Formation / wingman state — who to follow, what slot, what formation.
struct DigiFormationState {
    EntityId flightLeadId{kInvalidEntityId};  // who to follow
    bool     isWing{false};                    // true if this aircraft is a wingman
    int      vehicleInUnit{0};                 // slot index 0..3 (lead, 2, 3, 4)
    int      formationId{0};                   // FormationType enum value
    double   formRelAz{0.0};                   // desired bearing from lead (rad)
    double   formRelEl{0.0};                   // desired elevation from lead (rad)
    double   formRange{500.0};                 // desired range from lead (ft)
    WingmanState wingman;                      // wingman command state

    void reset() noexcept {
        flightLeadId = kInvalidEntityId;
        isWing = false;
        vehicleInUnit = 0;
        formationId = 0;
        formRelAz = 0.0;
        formRelEl = 0.0;
        formRange = 500.0;
        wingman.reset();
    }
};

// Communication state — mailbox + own entity ID.
struct DigiCommState {
    Mailbox mailbox;
    EntityId selfId{kInvalidEntityId};

    void reset() noexcept {
        mailbox.clear();
        // selfId is NOT reset — it's set once at init
    }
};

// ===========================================================================
// DigiState — the aggregate.
// ===========================================================================
struct DigiState {
    DigiCommands         commands;
    DigiConfigState      config;
    DigiNavState         nav;
    DigiGroundAvoidState groundAvoid;
    DigiMissileDefeatState missileDefeat;
    DigiGunsJinkState    gunsJink;
    DigiWeaponState      weapon;
    DigiGroundState      ag;
    DigiFormationState   formation;
    DigiCommState        comm;

    void reset() noexcept {
        commands.reset();
        // config is NOT reset — persists across mode resets
        nav.reset();
        groundAvoid.reset();
        missileDefeat.reset();
        gunsJink.reset();
        weapon.reset();
        ag.reset();
        formation.reset();
        comm.reset();
    }
};

} // namespace digi
} // namespace f4flight
