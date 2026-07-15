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
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/wingman/wingman_state.h"

namespace f4flight {
namespace digi {

struct DigiState {
    // --- Stick / throttle commands (written by maneuver functions) ---
    double pStick{0.0};
    double rStick{0.0};
    double yPedal{0.0};
    double throttle{0.5};  // [0, 1.5] where 1.0 = MIL, 1.5 = full AB

    // --- Brake / speed-brake / gear commands (Round-2 structural fix) ---
    // Previously PilotInput had wheelBrakes/speedBrake/gearHandle but the
    // digi brain had no way to set them — the brain wrote only pStick/
    // rStick/yPedal/throttle. Now the brain writes these and the host's
    // compute() maps them to PilotInput. This unblocks credible landing
    // rollout (wheel brakes) and A/G dive-bomb delivery (speed brakes).
    //
    // Convention matches PilotInput: speedBrakeCmd = -1 means retracted
    // (no drag), +1 means full extend. Default is -1 (retracted) so the
    // aircraft doesn't bleed speed when the brain isn't commanding brakes.
    bool   wheelBrakes{false};      // toe brakes (rollout deceleration)
    bool   parkingBrake{false};     // parking brake (startup / shutdown)
    double speedBrakeCmd{-1.0};     // -1 (retract) .. +1 (extend) — default retracted
    double gearHandleCmd{1.0};      // -1 (up) .. +1 (down) — default down

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

    // --- Maneuver timer (used by RoopMode / OverBMode / MergeManeuver) ---
    // Port of FreeFalcon's mnverTime (mnvers.cpp). Counts down in seconds;
    // when > 0, the maneuver stays active. Reset by the maneuver's first-frame
    // branch. Round-2 structural addition (Rec 10).
    double mnverTime{0.0};

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

    // Identity of the missile currently being defeated. When the brain
    // detects a different missile (host swap or SensorFusion swap), the
    // per-missile state (drag trackpoint, ttgo, evade timer) is re-initialized.
    // Set by DigiBrain::resolveMode from SensorContact::entityId, or by the
    // host when injecting via setIncomingMissile.
    EntityId incomingMissileId{kInvalidEntityId};

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

    // --- Merge state (Tier 2 — offensive) ---
    // FreeFalcon merge.cpp:9-52 enters MergeMode when range ≤ 1000 ft and
    // exits when mergeTimer expires (3 seconds). The previous F4Flight code
    // had no mergeTimer field — MergeCheck returned true based purely on
    // geometry, so the AI stayed in Merge mode indefinitely as long as the
    // geometry matched. This field is now decremented by MergeManeuver and
    // checked by MergeCheck to time out the mode.
    // -1.0 = not in merge (initial / reset), > 0 = seconds remaining in merge.
    double mergeTimer{-1.0};

    // --- Track point (used by AutoTrack, TrackPointLanding) ---
    // FreeFalcon stores trackX/Y/Z on the DigitalBrain and smooths it
    // (0.1*new + 0.9*old) in PullToCollisionPoint. We store it here so
    // AutoTrack and TrackPointLanding can read it without passing it
    // through every call.
    double trackX{0.0};
    double trackY{0.0};
    double trackZ{0.0};

    // --- Weapon / fire control state (Tier 2 — offensive) ---
    // Port of FreeFalcon DigitalBrain weapon engage fields (digi.h:827-836).
    //
    // Fire flags: cleared at the top of each compute() frame, set by the
    // offensive mode handlers (GunsEngage, MissileEngage). The host reads
    // PilotInput.fireGun / releaseConsent to actually fire.
    bool   gunFireFlag{false};       // fire the internal gun this frame
    bool   mslFireFlag{false};       // release a missile this frame
    int    fireStation{0};           // which hardpoint to release from

    // Missile engage state
    double missileShotTimer{0.0};    // seconds since last missile fired
    bool   inShootShoot{false};      // currently in shoot-shoot doctrine

    // Guns engage state (port of FF digi.h:829-836)
    bool   waitingForShot{false};    // in the fine-track firing phase
    double pastPstick{0.0};          // G held when entering fine track
    double pastAta{0.0};             // previous frame's pipper ATA (rad)
    double pastPipperAta{0.0};       // previous frame's pipper ATA (rad)
    double ataDot{0.0};              // rate of change of ATA (rad/s)

    // Max A/A weapon range (set by host from SMS, used by mode checks).
    // 0 = no weapons loaded (brain uses default BVR gate of 35 NM)
    double maxAAWpnRange{0.0};

    // --- A/G target + doctrine (Round-2 structural fix — Rec 9 / Gap 2) ---
    // Pointer to the current ground target. Set by the host (via
    // FrameInputs.injectedGroundTarget) or auto-tracked from SensorFusion
    // when the picture contains a ground target. The brain's future
    // GroundMnvr / GroundAttack modes (unported) will read this.
    //
    // AG doctrine + approach enums live in digi/ground/ag_doctrine.h
    // (new header — included by ground_ops.h).
    const DigiEntity* groundTarget{nullptr};
    EntityId groundTargetId{kInvalidEntityId};
    int      agDoctrine{0};    // AGD_NONE / SHOOT_RUN / LOOK_SHOOT_LOOK / NEED_SETUP
    int      agApproach{0};    // AGA_LOW / TOSS / HIGH / DIVE / BOMBER
    bool     reachedIP{false}; // true once the AI has passed the Initial Point

    // --- Ground ops state (Phase 1-2) ---
    GroundOpsState groundOps;

    // --- A/G attack phase (Round-3 structural addition — DIGI_AUDIT_ROUND3.md §3.3) ---
    // Separate from GroundOpsPhase to avoid FreeFalcon's quirk of reusing
    // the onStation enum for both landing and A/G. The future GroundMnvr
    // mode will drive this state machine.
    AgAttackPhase agAttackPhase{AgAttackPhase::NotThereYet};

    // --- Communication (Phase 1) ---
    Mailbox mailbox;           // incoming messages from ATC, flight lead, etc.
    EntityId selfId{kInvalidEntityId};  // this aircraft's entity ID (for addressing)

    // --- Formation / wingman state (Round-2 structural fix — Rec 3 / Gap 1) ---
    // FreeFalcon's DigitalBrain has flightLead, isWing, vehicleInUnit fields
    // (digimain.cpp:41-62 + formdata.cpp). Without these the brain cannot
    // know who to follow or what slot it's in. The fields are populated by
    // the host (setLead / setWingmanSlot); the brain's future Wingy /
    // FollowOrders modes (unported) will read them. PositionData is defined
    // in digi/formation/formation_geometry.h (new header).
    //
    // These are intentionally plain values — no wingman-table data structure
    // yet (that lands with the formdata.cpp port). The host can populate
    // these from its own wingman model.
    EntityId flightLeadId{kInvalidEntityId};  // who to follow (kInvalidEntityId = none)
    bool     isWing{false};                    // true if this aircraft is a wingman
    int      vehicleInUnit{0};                 // slot index 0..3 (lead, 2, 3, 4)
    int      formationId{0};                   // formation type (see FormationType enum)
    // Formation geometry (relative to lead) — set by host or by wingman AI.
    // Lead writes these into wingmen via the MessageBus; wingmen read them
    // to compute their desired position.
    double   formRelAz{0.0};     // desired bearing from lead (rad)
    double   formRelEl{0.0};     // desired elevation from lead (rad)
    double   formRange{500.0};   // desired range from lead (ft)

    // --- Wingman command state (Round-3 structural addition — §3.2) ---
    // Populated by receiveOrders() when Flight* messages arrive. Read by
    // the future AiRunDecisionRoutines / AiFollowLead / AiPerformManeuver
    // modes (unported). The Wingy and FollowOrders modes will consume this.
    WingmanState wingman;

    void reset() noexcept {
        pStick = rStick = yPedal = 0.0;
        // Brake / speed-brake / gear commands (Round-2 fix)
        wheelBrakes = false;
        parkingBrake = false;
        speedBrakeCmd = -1.0;  // retracted by default
        gearHandleCmd = 1.0;  // gear down by default
        gammaHoldIError = 0.0;
        autoThrottle = 0.0;
        trackMode = 0;
        mnverTime = 0.0;  // Round-2 fix: RoopMode/OverBMode timer
        onStation = 0;
        waypointMode = 1;
        groundAvoidNeeded = false;
        pullupTimer = 0.0;
        // Note: threat pointers (incomingMissile, gunsThreat) are NOT cleared
        // by reset() — the host manages these. reset() only clears internal
        // brain state.
        incomingMissileId = kInvalidEntityId;
        missileDefeatTtgo = -1.0;
        missileFindDragPt = true;
        missileShouldDrag = false;
        missileFinishedBeam = false;
        incomingMissileRange = 500.0 * 6076.0;
        incomingMissileEvadeTimer = 0.0;
        jinkTime = -1;
        newRoll = 0.0;
        jinkTimer = 0.0;
        mergeTimer = -1.0;  // Round-2 fix: merge mode timer (3s when active)
        trackX = trackY = trackZ = 0.0;
        // Weapon / fire control
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
        groundOps.reset();
        mailbox.clear();
        // A/G target + doctrine (Round-2 fix)
        groundTarget = nullptr;
        groundTargetId = kInvalidEntityId;
        agDoctrine = 0;
        agApproach = 0;
        reachedIP = false;
        // Formation / wingman state (Round-2 fix)
        flightLeadId = kInvalidEntityId;
        isWing = false;
        vehicleInUnit = 0;
        formationId = 0;
        formRelAz = 0.0;
        formRelEl = 0.0;
        formRange = 500.0;
        // A/G attack phase (Round-3)
        agAttackPhase = AgAttackPhase::NotThereYet;
        // Wingman command state (Round-3)
        wingman.reset();
    }
};

} // namespace digi
} // namespace f4flight
