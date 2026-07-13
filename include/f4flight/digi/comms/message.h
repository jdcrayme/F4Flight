// f4flight - digi/comms/message.h
//
// Message — the universal communication unit for the digi AI.
//
// All coordination between entities (ATC clearances, flight lead commands,
// wingman reports, threat calls) flows through messages. This decouples the
// sender from the receiver — the sender publishes to the MessageBus, the
// bus routes to the recipient's Mailbox, and the recipient's DigiBrain reads
// its mailbox each frame.
//
// Design principles:
//   1. Messages are plain data (no virtual functions, no inheritance).
//      This makes them cheap to copy and easy to serialize later.
//   2. Addressing is by EntityId, not by pointer. This avoids dangling
//      pointers and lets the bus route without knowing about entity types.
//   3. Message types are strongly-typed enums. Each type has a known payload
//      layout. A variant or tagged union could work, but for C++17 a simple
//      struct with a type tag + a small payload union is the most portable.
//   4. No audio/voice coupling. FreeFalcon's radio messages trigger audio
//      playback — that's a presentation concern, not a logic concern. The
//      host can subscribe to messages and play audio if desired.
//
// Comparison to FreeFalcon:
//   FreeFalcon uses FalconRadioChatterMessage (a VuMessage subclass) with
//   ~60 wingman command values, voice_id, edata[10], and time_to_play. It's
//   tightly coupled to the Vu multiplayer system and the audio system.
//   We separate the message (logic) from delivery (bus) from presentation
//   (host's audio).

#pragma once

#include <cstdint>
#include <string>

namespace f4flight {
namespace digi {

// EntityId — unique identifier for any entity in the sim (aircraft, ATC,
// flight, etc.). 0 = invalid/broadcast. The host assigns these; the digi
// only uses them for addressing.
using EntityId = uint32_t;

constexpr EntityId kInvalidEntityId = 0;
constexpr EntityId kBroadcastId = 0xFFFFFFFF;  // all recipients

// MessageType — the category of message. Each type determines how the
// payload is interpreted.
//
// These are grouped by domain:
//   - ATC: air traffic control clearances and instructions
//   - Flight: flight lead → wingman commands and wingman → lead reports
//   - Threat: spike/missile calls between flight members
//   - System: status (bingo, winchester, flameout)
enum class MessageType : uint16_t {
    // --- ATC ---
    ATCClearanceRequest,      // aircraft → ATC: request takeoff/landing clearance
    ATCClearanceGranted,      // ATC → aircraft: cleared for takeoff/landing
    ATCClearanceDenied,       // ATC → aircraft: hold short, wait
    ATCTaxiInstruction,       // ATC → aircraft: taxi to point X
    ATCHoldShort,             // ATC → aircraft: hold short of runway
    ATCTakeoffPosition,       // ATC → aircraft: taxi onto runway, line up
    ATCClearedTakeoff,        // ATC → aircraft: cleared for takeoff
    ATCClearedLanding,        // ATC → aircraft: cleared to land
    ATCGoAround,              // ATC → aircraft: abort landing, go around
    ATCRunwayClear,           // ATC → aircraft: runway is clear, vacate
    ATCTrafficAdvisory,       // ATC → aircraft: traffic at (position)

    // --- Flight commands (lead → wingman) ---
    FlightCmdEngage,          // engage target
    FlightCmdEngageMyTarget,  // engage lead's target
    FlightCmdBreak,           // break (direction in payload)
    FlightCmdRejoin,          // rejoin formation
    FlightCmdWedge,           // set formation: wedge
    FlightCmdTrail,           // set formation: trail
    FlightCmdSpread,          // set formation: spread
    FlightCmdEchelon,         // set formation: echelon
    FlightCmdFingerFour,      // set formation: finger four
    FlightCmdRTB,             // return to base
    FlightCmdJettison,        // drop stores
    FlightCmdECMOn,           // turn on ECM
    FlightCmdECMOff,          // turn off ECM
    FlightCmdRadarOn,         // turn on radar
    FlightCmdRadarOff,        // turn on standby
    FlightCmdWeaponsHold,     // weapons hold (don't fire)
    FlightCmdWeaponsFree,     // weapons free (cleared to fire)
    FlightCmdPromote,         // promote wingman to lead

    // --- Flight reports (wingman → lead, or anyone → flight) ---
    FlightReportBingo,        // bingo fuel
    FlightReportWinchester,   // out of weapons
    FlightReportSplash,       // killed a bandit
    FlightReportBandit,       // bandit sighted (position in payload)
    FlightReportTally,        // visual tally on bandit
    FlightReportNoJoy,        // no visual on bandit
    FlightReportFlameout,     // engine flameout
    FlightReportRequestHelp,  // need help (under attack)

    // --- Threat calls (between flight members) ---
    ThreatCallSpike,          // radar spike (direction in payload)
    ThreatCallMissile,        // missile launch (direction in payload)
    ThreatCallSAM,            // SAM launch (direction in payload)
    ThreatCallBuddySpike,     // friendly radar spike (someone locking us)

    // --- System ---
    SystemPing,               // for testing
};

// Payload — the data carried by a message. A small fixed-size union keeps
// messages cheap to copy and avoids heap allocation. Different message types
// use different fields.
struct MessagePayload {
    // Position (world frame, ft) — for traffic advisories, bandit calls
    double x{0.0}, y{0.0}, z{0.0};

    // Heading (radians) — for break direction, spike direction
    double heading{0.0};

    // Entity reference — for engage target, promote, etc.
    EntityId entityId{kInvalidEntityId};

    // Generic integer value — for formation type, runway number, etc.
    int value{0};

    // Generic double value — for fuel state, altitude, etc.
    double dvalue{0.0};
};

// Message — one communication unit.
struct Message {
    MessageType type{MessageType::SystemPing};
    EntityId sender{kInvalidEntityId};
    EntityId recipient{kBroadcastId};  // kBroadcastId = all
    MessagePayload payload{};

    // Timestamp (seconds, sim time) — set by the bus when sent.
    // Recipients can use this to expire stale messages.
    double timestamp{0.0};

    Message() = default;
    Message(MessageType t, EntityId s, EntityId r)
        : type(t), sender(s), recipient(r) {}
};

// Addressing helpers — recipient can be a specific entity or a group.
// Groups are resolved by the MessageBus (the host registers which entities
// belong to which group).
enum class AddressType {
    SpecificEntity,  // one aircraft
    Flight,          // all members of a flight (lead + wingmen)
    Package,         // all flights in a package
    ATC,             // the airbase ATC controller
    Broadcast,       // everyone
};

// An addressed recipient. The host maps groupId to actual entity IDs when
// the MessageBus delivers.
struct Address {
    AddressType type{AddressType::Broadcast};
    EntityId id{kBroadcastId};  // entity ID, flight ID, or airbase ID
};

inline bool operator==(const Address& a, const Address& b) {
    return a.type == b.type && a.id == b.id;
}
inline bool operator!=(const Address& a, const Address& b) {
    return !(a == b);
}

} // namespace digi
} // namespace f4flight
