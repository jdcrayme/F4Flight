// f4flight - digi/ground/ground_ops.h
//
// Ground operations — taxi, takeoff, and landing AI.
//
// This is the DigiBrain's ground operations layer. It handles:
//   - Taxi: follow taxi graph nodes to the runway hold-short point
//   - Takeoff: lineup, brake release, rotation, gear up, climbout
//   - Landing: approach, flare, touchdown, rollout, vacate runway
//
// The AI communicates with ATC via the MessageBus to get clearances.
// Without clearance, the aircraft holds short and waits.
//
// Design:
//   - GroundOpsState tracks the aircraft's current ground ops phase.
//   - The DigiBrain's GroundOps/Takeoff/Landing modes dispatch to these
//     functions.
//   - All stick/throttle commands use the existing ManeuverPrimitives.
//   - ATC coordination is via MessageBus — the brain reads its mailbox each
//     frame to check for clearances.
//
// Comparison to FreeFalcon:
//   FreeFalcon's landme.cpp is 4,778 LOC with a ~30-state state machine
//   interleaved with ATC radio calls, taxi collision avoidance, parking
//   spot allocation, and carrier ops. We implement a simplified version:
//   - Straight-in approach (no pattern work)
//   - Single runway (no complex taxi routing)
//   - No carrier ops (yet)
//   - ATC via MessageBus (not direct function calls)

#pragma once

#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/digi/atc/atc_messages.h"

#include <vector>

namespace f4flight {
namespace digi {

namespace atc { class TaxiGraph; }  // forward declaration (full header is taxi_graph.h)

// Forward declaration — DigiState is defined in digi_state.h, which includes
// this file. We only need a reference here.
struct DigiState;
class Mailbox;

// Ground ops phase (sub-state within the Takeoff/Landing DigiModes)
enum class GroundOpsPhase {
    // Takeoff phases
    Parking,         // at parking spot, engines off
    RequestTaxi,     // requesting taxi clearance from ATC
    TaxiToRunway,    // taxiing to hold-short point
    HoldingShort,    // at hold-short, waiting for takeoff clearance
    LiningUp,        // taxiing onto runway, lining up
    TakeoffRoll,     // brakes released, accelerating
    Rotation,        // nose up, lifting off
    AfterTakeoff,    // gear up, climbing out

    // Landing phases
    Approach,        // on final, descending
    Flare,           // leveling off just above runway
    Touchdown,       // main gear on runway
    Rollout,         // decelerating on runway
    VacatingRunway,  // taxiing off runway
    TaxiToParking,   // taxiing back to parking

    // General
    Idle,            // not in ground ops
};

// GroundOpsState — persistent state for ground operations.
// Owned by DigiState so it persists across frames.
struct GroundOpsState {
    GroundOpsPhase phase{GroundOpsPhase::Idle};

    // Runway info
    RunwayId assignedRunway{0};       // runway ATC assigned
    double runwayHeading{0.0};        // radians
    double runwayThresholdX{0.0};     // world ft
    double runwayThresholdY{0.0};     // world ft
    double runwayAltitude{0.0};       // ft MSL

    // Taxi state
    int currentTaxiNode{-1};          // current taxi graph node
    int targetTaxiNode{-1};           // next node to taxi to
    double taxiSpeed{0.0};            // target taxi speed (kts)
    // Round-2 structural fix (Rec 4 / Bug K): the TaxiGraph pointer was
    // missing entirely, so RunTaxi was dead code — it had no way to look
    // up the next node. Now the host sets taxiGraph before commanding a
    // TaxiToRunway / TaxiToParking phase, and RunTaxi follows the BFS
    // path node-by-node.
    const atc::TaxiGraph* taxiGraph{nullptr};
    // Cached BFS path from currentTaxiNode to the final destination.
    // Re-computed when targetTaxiNode changes or when the aircraft reaches
    // a node. Each entry is a TaxiNodeId.
    // (Stored as ints to avoid pulling the TaxiGraph header into this header;
    // the IDs are stable indices into TaxiGraph::node().)
    std::vector<int> taxiPath;
    std::size_t taxiPathIdx{0};

    // Takeoff state
    double takeoffRollStart{0.0};     // sim time when roll began
    double rotationSpeed{0.0};        // V_R (kts)
    bool gearRetracted{false};

    // Landing state
    double approachStartAlt{0.0};     // altitude when approach began
    double flareStartAlt{0.0};        // altitude when flare began
    double touchdownSpeed{0.0};       // speed at touchdown (kts)
    double touchdownDescentRate{0.0}; // descent rate at touchdown (ft/s, + = descending)
    double touchdownPitch{0.0};       // pitch attitude at touchdown (radians)
    double touchdownTimer{0.0};       // time since touchdown (s) — for nose settling
    bool gearDeployed{false};

    // ATC clearance
    bool hasTakeoffClearance{false};
    bool hasLandingClearance{false};

    void reset() {
        // BUG FIX: previously only 10 of 22 fields were reset. Stale runway
        // heading, taxi graph pointer, takeoff roll start time, flare start
        // altitude etc. leaked across phases — e.g. a Takeoff test followed
        // by a Landing test would inherit the previous runway's heading
        // because runwayHeading wasn't zeroed. Reset every field so reset()
        // actually means "back to fresh state".
        phase = GroundOpsPhase::Idle;
        assignedRunway = 0;
        runwayHeading = 0.0;
        runwayThresholdX = 0.0;
        runwayThresholdY = 0.0;
        runwayAltitude = 0.0;
        currentTaxiNode = -1;
        targetTaxiNode = -1;
        taxiSpeed = 0.0;
        taxiGraph = nullptr;
        taxiPath.clear();
        taxiPathIdx = 0;
        takeoffRollStart = 0.0;
        rotationSpeed = 0.0;
        gearRetracted = false;
        approachStartAlt = 0.0;
        flareStartAlt = 0.0;
        touchdownSpeed = 0.0;
        touchdownDescentRate = 0.0;
        touchdownPitch = 0.0;
        touchdownTimer = 0.0;
        gearDeployed = false;
        hasTakeoffClearance = false;
        hasLandingClearance = false;
    }
};

// --- Takeoff ---
// RunTakeoff — execute the takeoff sequence.
//
//   digi      : AI state (reads groundOps, writes stick/throttle)
//   as        : aircraft state
//   fcsState  : FCS state (written)
//   dt        : frame time
//   simTime   : current sim time (seconds)
//   groundZ   : terrain altitude at aircraft position (ft)
void RunTakeoff(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ);

// --- Landing ---
// RunLanding — execute the landing sequence.
//
//   digi      : AI state
//   as        : aircraft state
//   fcsState  : FCS state
//   dt        : frame time
//   simTime   : current sim time
//   groundZ   : terrain altitude
void RunLanding(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ);

// --- Taxi ---
// RunTaxi — taxi toward the target taxi node.
//
//   digi      : AI state
//   as        : aircraft state
//   fcsState  : FCS state
//   dt        : frame time
void RunTaxi(DigiState& digi, const AircraftState& as,
             FcsState& fcsState, double dt);

// --- Message processing ---
// ProcessATCMessages — read ATC messages from the brain's mailbox and update
// ground ops state accordingly.
//
//   digi      : AI state (reads mailbox, writes groundOps)
//   mailbox   : the brain's mailbox (ATC messages arrive here)
void ProcessATCMessages(DigiState& digi, class Mailbox& mailbox);

} // namespace digi
} // namespace f4flight
