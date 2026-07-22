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
#include "f4flight/digi/comms/radio_calls.h"
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

    // Flap commands (normalized 0..1, matches PilotInput.tefCmd/lefCmd).
    // 0 = retracted (clean), 1 = fully extended.
    // Set by ground_ops.cpp for landing (FreeFalcon's af->SetFlaps(true)).
    double tefCmd{0.0};             // trailing-edge flap
    double lefCmd{0.0};             // leading-edge flap

    void reset() noexcept {
        pStick = rStick = yPedal = 0.0;
        throttle = 0.5;
        wheelBrakes = false;
        parkingBrake = false;
        speedBrakeCmd = -1.0;
        gearHandleCmd = 1.0;
        tefCmd = 0.0;
        lefCmd = 0.0;
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

// ===========================================================================
// FlightPhase — the AI's current flight phase, used for gain scheduling.
//
// FreeFalcon does NOT have an explicit flight-phase enum for the AI. Instead,
// it relies on:
//   1. The FCS-level `landingGains` flag (triggered by gear/flaps down) to
//      soften the FCS response at low speed.
//   2. Different error formulations per mode (GammaHold vs AltHold vs
//      TrackPointLanding) — each mode feeds a different error signal to the
//      same PID controller.
//
// F4Flight adds an explicit FlightPhase enum because:
//   - The landing approach/flare need MUCH gentler PID gains than combat
//     (the Phugoid oscillation is excited by aggressive gains at low speed).
//   - Formation needs different lateral damping than navigation.
//   - Having the phase explicit makes the gain scheduling visible and
//     tunable, rather than implicit in the error formulation.
//
// The phase is set by the brain's mode dispatch (runWaypoint, runRTB,
// runGroundAttack, ground_ops, etc.) and read by GammaHold/MachHold/
// HeadingAndAltitudeHold to select per-phase gains.
// ===========================================================================
enum class FlightPhase : int {
    Cruise      = 0,  // waypoint navigation, RTB, loiter (gentle, stable)
    Combat      = 1,  // BVR/WVR/guns engagement (aggressive, high-G)
    Formation   = 2,  // wingman formation following (precise, damped)
    Approach    = 3,  // landing approach (gentle, Phugoid-damped)
    Flare       = 4,  // landing flare (very gentle, nose-up bias)
    GroundOps   = 5,  // takeoff/taxi/rollout (ground steering)
};

inline const char* flightPhaseName(FlightPhase p) {
    switch (p) {
        case FlightPhase::Cruise:    return "Cruise";
        case FlightPhase::Combat:    return "Combat";
        case FlightPhase::Formation: return "Formation";
        case FlightPhase::Approach:  return "Approach";
        case FlightPhase::Flare:     return "Flare";
        case FlightPhase::GroundOps: return "GroundOps";
    }
    return "Unknown";
}

// PhaseGainSet — per-phase PID gains for the AI maneuver primitives.
//
// These gains are used by GammaHold, HeadingAndAltitudeHold, and MachHold
// when the brain is in the corresponding flight phase. The brain sets
// state_.nav.flightPhase each frame; the primitives read it to select gains.
//
// The gains are:
//   gammaGain     — GammaHold error gain (altErr * gammaGain → desired gamma)
//   gammaClamp    — GammaHold max gamma (deg) — limits the pitch command
//   integralGain  — GammaHold integral gain (0 = pure P, no integral)
//   phugoidGain   — PhugoidDamper gain (pitch-rate feedback)
//   rollDampGain  — HeadingAndAltitudeHold roll-rate damping gain
//   speedGain     — MachHold proportional gain (eProp * speedGain → throttle)
//
// Default values are tuned per phase. Hosts can override via DigiBrain::
// configurePhaseGains() if needed.
struct PhaseGainSet {
    double gammaGain{0.015};
    double gammaClamp{15.0};
    double integralGain{0.0025};  // 0 = pure P
    double phugoidGain{0.3};
    double rollDampGain{0.3};
    double speedGain{0.02};

    static PhaseGainSet forPhase(FlightPhase phase) {
        switch (phase) {
            case FlightPhase::Cruise:
                // Gentle, stable. Phugoid-damped for smooth cruise.
                return PhaseGainSet{0.015, 15.0, 0.0025, 0.3, 0.3, 0.02};
            case FlightPhase::Combat:
                // Aggressive, high-G. No Phugoid damping (combat maneuvers
                // need fast response, not smoothness).
                return PhaseGainSet{0.05, 60.0, 0.0025, 0.0, 0.5, 0.02};
            case FlightPhase::Formation:
                // Precise, damped. Higher Phugoid damping for stable formation.
                // Stiffer gains for precise altitude tracking during refueling.
                return PhaseGainSet{0.04, 15.0, 0.004, 0.4, 0.15, 0.02};
            case FlightPhase::Approach:
                // Gentle, Phugoid-damped. Lower gain to prevent oscillation
                // on the glideslope. Pure P (no integral) to prevent windup.
                return PhaseGainSet{0.015, 10.0, 0.0, 0.5, 0.3, 0.015};
            case FlightPhase::Flare:
                // Very gentle. Low gamma clamp to prevent pitch-up runaway.
                // High Phugoid damping for a smooth flare.
                return PhaseGainSet{0.01, 5.0, 0.0, 0.6, 0.2, 0.01};
            case FlightPhase::GroundOps:
                // Ground steering — minimal pitch, no Phugoid damping.
                return PhaseGainSet{0.01, 5.0, 0.0, 0.0, 0.3, 0.02};
        }
        return PhaseGainSet{};  // default
    }
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

    // Flight phase for gain scheduling (set by the brain's mode dispatch
    // each frame). Read by GammaHold/HeadingAndAltitudeHold/MachHold to
    // select per-phase PID gains via PhaseGainSet::forPhase().
    FlightPhase flightPhase{FlightPhase::Cruise};

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
        flightPhase = FlightPhase::Cruise;
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

    // Round 7 (P1): radar mode management.
    // radModeSelect is set by the offensive modes (BvrEngage, MissileEngage,
    // WvrEngage) to request a radar mode. ChooseRadarMode translates it to
    // radarMode and applies it. Values: 0=STT, 1=SAM, 2=TWS, 3=RWS(default),
    // 4=OFF. See decision_routines.h::RadarMode.
    int    radModeSelect{3};           // default: RWS
    int    radarMode{3};               // current applied mode (RadarMode enum int)
    double lastRadarModeTime{-1e9};    // last chooseRadarMode call (throttle)

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
        // Round 7: radar mode fields
        radModeSelect = 3;  // RWS default
        radarMode = 3;
        lastRadarModeTime = -1e9;
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

    // Task 15-a: which delivery geometry runGroundAttack() executes.
    // Selected by the host via FrameInputs.injectedAgProfile. Defaults to
    // DiveBomb so existing scenarios that don't set a profile keep the
    // Task 11 dive-bomb behavior (backward compat).
    AgAttackProfile agProfile{AgAttackProfile::DiveBomb};

    // SEAD HARM Targeting System Mode
    HtsMode htsMode{HtsMode::PreBriefed};

    void reset() noexcept {
        groundTarget = nullptr;
        groundTargetId = kInvalidEntityId;
        agDoctrine = 0;
        agApproach = 0;
        reachedIP = false;
        groundOps.reset();
        agAttackPhase = AgAttackPhase::NotThereYet;
        agProfile = AgAttackProfile::DiveBomb;
        htsMode = HtsMode::PreBriefed;
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

    // Multi-point maneuver trackpoints (port of FF mpManeuverPoints).
    // Used by AiExecPince (2 points) and AiExecFlex (3 points). Each point
    // is a world (x, y) position; the altitude comes from altitudeOrdered.
    // Indexed by maneuverPointCounter. FF uses a fixed [4][2] float array;
    // we use a small fixed-size array of Vec2-like pairs.
    //
    // The counter advances when the wingman reaches a point (within 5000 ft
    // for Pince, 900 ft for Flex — matching FF's thresholds). When the
    // counter exceeds the number of points, the maneuver completes.
    static constexpr int kMaxManeuverPoints = 4;
    struct ManeuverPoint { double x{0.0}, y{0.0}; };
    ManeuverPoint maneuverPoints[kMaxManeuverPoints];
    int maneuverPointCounter{0};

    void reset() noexcept {
        flightLeadId = kInvalidEntityId;
        isWing = false;
        vehicleInUnit = 0;
        formationId = 0;
        formRelAz = 0.0;
        formRelEl = 0.0;
        formRange = 500.0;
        wingman.reset();
        for (int i = 0; i < kMaxManeuverPoints; ++i) {
            maneuverPoints[i] = ManeuverPoint{};
        }
        maneuverPointCounter = 0;
    }
};

// Communication state — mailbox + own entity ID.
struct DigiCommState {
    Mailbox mailbox;
    EntityId selfId{kInvalidEntityId};
    // Round 6: last CommandFlight order time (for 5 s throttle). Stored here
    // so it's per-brain (not global). CommandFlight reads/writes this.
    double lastOrderTime{-1e9};  // -1e9 = "never ordered" sentinel

    // Radio call queue — the brain pushes RadioCallType events here; the
    // host drains them each frame for display/logging/voice playback.
    // Port of FreeFalcon's wingradio.cpp AiMakeRadioCall/AiMakeRadioResponse.
    RadioCallQueue radioCalls;

    // Track which calls have been made (to prevent repeating). Each bit
    // corresponds to a RadioCallType. Once a call is made, it's not repeated
    // until reset.
    uint32_t callsMade{0};

    void reset() noexcept {
        mailbox.clear();
        // selfId is NOT reset — it's set once at init
        lastOrderTime = -1e9;
        radioCalls.reset();
        callsMade = 0;
    }
};

// Secondary threat state — HandleThreat (port of FF handlethreat.cpp).
// A "secondary threat" is an aircraft that has spiked us, fired a missile at
// us, or been called out by a wingman/RWR — but is NOT the primary offensive
// target, NOT the incoming missile (that's DigiMissileDefeatState), and NOT
// the guns threat (that's DigiGunsJinkState).
//
// When threatPtr is non-null and the brain is NOT already in MissileDefeat,
// HandleThreat() runs instead of the per-mode switch and engages the threat
// via RollAndPull. threatTimer counts down from 10 s; when it expires,
// HandleThreat re-evaluates whether the threat is still worth chasing.
struct DigiThreatState {
    const DigiEntity* threatPtr{nullptr};
    double threatTimer{0.0};

    // Threat-call bearing: when a ThreatCallSpike/Missile/SAM message
    // arrives, the bearing (relative to self heading, radians) is stored
    // here. The host or SensorFusion can then resolve it to an entity by
    // looking up contacts at that bearing. This is a one-shot value —
    // processMessages sets it and the host reads + clears it.
    //
    // -999.0 = no pending threat call (sentinel).
    double threatCallBearing{-999.0};
    // The type of the most recent threat call (mirrors MessageType:
    // ThreatCallSpike / ThreatCallMissile / ThreatCallSAM / ThreatCallBuddySpike).
    // 0 = none. The host reads this to decide how to react.
    int threatCallType{0};

    void reset() noexcept {
        // NOTE: threatPtr is NOT cleared by reset() — the host manages it.
        // DigiBrain::reset() clears it explicitly.
        threatTimer = 0.0;
        threatCallBearing = -999.0;
        threatCallType = 0;
    }
};

// Fuel / RTB state — port of FF separate.cpp FuelCheck + actions.cpp AirbaseCheck.
// Without these fields the AI has no fuel-state awareness — it flies until
// flameout. FuelCheck() (called from resolveMode) transitions fuelState based
// on fuelLbs each frame; resolveMode reads fuelState to decide whether to
// enter RTB mode.
//
// The host sets fuelLbs each frame from AircraftState.fuel.fuel_lbs and
// configures bingoLbs / jokerLbs / fumesLbs once at mission start.
// Winchester (out of A/A weapons) is set by the host when SMS reports zero
// A/A weapons remaining.
struct DigiFuelState {
    enum class Phase : int {
        Normal = 0,    // > joker
        Joker  = 1,    // joker .. bingo (time to start thinking about RTB)
        Bingo  = 2,    // bingo .. fumes (commit to RTB / nearest airbase)
        Fumes  = 3,    // < fumes (fuel-critical — direct divert)
        Flameout = 4,  // engine out (host sets this; brain can't detect it)
    };
    Phase  phase{Phase::Normal};
    double fuelLbs{0.0};          // current fuel (lb) — set each frame by host
    double bingoFuelLbs{0.0};     // bingo fuel (lb) — set at mission start
    double jokerFuelLbs{0.0};     // joker fuel (lb) — set at mission start
    double fumesFuelLbs{0.0};     // fumes fuel (lb) — set at mission start
    bool   winchester{false};     // true = out of A/A weapons (host sets)

    // Divert airbase: set by the host when AirbaseCheck picks the nearest
    // friendly field (a future port will auto-pick from a host-provided list).
    // RTB mode reads these to navigate.
    double divertAirbaseX{0.0};
    double divertAirbaseY{0.0};
    double divertAirbaseZ{0.0};        // world Z (negative = MSL altitude)
    double divertAirbaseHeading{0.0};  // runway heading (rad)
    bool   hasDivertAirbase{false};

    void reset() noexcept {
        phase = Phase::Normal;
        fuelLbs = 0.0;
        bingoFuelLbs = 0.0;
        jokerFuelLbs = 0.0;
        fumesFuelLbs = 0.0;
        winchester = false;
        divertAirbaseX = 0.0;
        divertAirbaseY = 0.0;
        divertAirbaseZ = 0.0;
        divertAirbaseHeading = 0.0;
        hasDivertAirbase = false;
    }
};

// Damage / disengage state — port of FF separate.cpp SeparateCheck.
// Tracks airframe damage + bugout timer for the disengage decision.
// The host sets pctStrength each frame (1.0 = pristine, 0.0 = destroyed);
// SeparateCheck reads it to decide whether to abort the mission.
struct DigiDamageState {
    // Airframe strength fraction (1.0 = pristine, 0.0 = destroyed).
    // Set by the host each frame from the damage model.
    // FF uses self->pctStrength; we mirror that here.
    double pctStrength{1.0};

    // Bugout timer (port of FF bugoutTimer). When the AI is "deep six"
    // (target ataFrom > 135° for > 90 seconds), it disengages via BugoutMode.
    // The timer is in seconds of sim time; 0.0 = not running.
    // FF uses absolute SimLibElapsedTime + 90s; we use a countdown from 90.
    double bugoutTimer{0.0};
    bool   bugoutTimerActive{false};

    // RTB flags (port of FF ATC flags SaidBingo / SaidFumes / SaidRTB).
    // These are sticky once set — the brain doesn't un-say them.
    bool   saidBingo{false};
    bool   saidFumes{false};
    bool   saidRTB{false};

    void reset() noexcept {
        pctStrength = 1.0;
        bugoutTimer = 0.0;
        bugoutTimerActive = false;
        saidBingo = false;
        saidFumes = false;
        saidRTB = false;
    }
};

// ===========================================================================
// Air-to-air refueling (AAR) state.
//
// Port of FreeFalcon's refuel.cpp refuelstatus enum + AiRefuel state machine.
// The brain tracks the refueling phase: approach → contact → disconnect.
//
// The host injects the tanker via FrameInputs.injectedTanker. When non-null,
// the brain enters Refueling mode and flies to the tanker's boom position.
// The tanker must be a friendly aircraft flying straight and level.
//
// Phases:
//   None      — not refueling (no tanker injected)
//   Approach  — flying to the boom position (behind and below the tanker)
//   Contact   — holding position at the boom, taking fuel
//   Disconnect — departing the tanker after refueling complete
// ===========================================================================
struct DigiRefuelState {
    enum class Phase : int {
        None       = 0,
        Inbound    = 1,
        Approach   = 1,  // Backward compatibility alias
        Precontact = 2,
        Contact    = 3,
        Disconnect = 4,
    };

    Phase  phase{Phase::None};
    double contactTimer{0.0};    // seconds in Contact phase (fuel taken)
    double contactDuration{30.0}; // seconds to hold contact before disconnect
    bool   refuelComplete{false}; // true when disconnect phase finished

    // Boom position (computed each frame from tanker position + geometry).
    // The boom is behind and below the tanker. The receiver flies to this
    // position and holds.
    double boomX{0.0}, boomY{0.0}, boomZ{0.0};

    void reset() noexcept {
        phase = Phase::None;
        contactTimer = 0.0;
        contactDuration = 30.0;
        refuelComplete = false;
        boomX = boomY = boomZ = 0.0;
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
    DigiThreatState      threat;   // HandleThreat (FF handlethreat.cpp)
    DigiFuelState        fuel;     // FuelCheck / RTB (FF separate.cpp + AirbaseCheck)
    DigiDamageState      damage;   // SeparateCheck / BugoutMode (FF separate.cpp)
    DigiRefuelState      refuel;   // AAR (FF refuel.cpp)

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
        threat.reset();
        fuel.reset();
        damage.reset();
        refuel.reset();
    }
};

} // namespace digi
} // namespace f4flight
