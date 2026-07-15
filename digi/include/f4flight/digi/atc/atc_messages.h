// f4flight - digi/atc/atc_messages.h
//
// ATC message types and helpers.
//
// These are the specific message constructions used by the ATC system.
// They use the generic Message struct from comms/message.h but provide
// factory functions so callers don't have to remember which payload fields
// to set for each message type.

#pragma once

#include "f4flight/digi/comms/message.h"

namespace f4flight {
namespace digi {

// Runway identifier (number × 10 + side: 270 = runway 27, 271 = 27L, 272 = 27R)
// Defined at digi namespace level so all digi subsystems can use it.
using RunwayId = int;

namespace atc {

// ATC clearance states (what the ATC has cleared the aircraft to do).
enum class ClearanceState {
    None,               // no clearance
    RequestTaxi,        // aircraft requested taxi
    ClearedTaxi,        // ATC cleared taxi to runway
    HoldingShort,       // aircraft holding short of runway
    RequestTakeoff,     // aircraft requests takeoff
    ClearedTakeoff,     // ATC cleared takeoff
    OnRunway,           // aircraft is on the runway
    Airborne,           // aircraft is airborne
    RequestLanding,     // aircraft requests landing
    ClearedLanding,     // ATC cleared landing
    OnFinal,            // aircraft on final approach
    GoAround,           // ATC instructed go around
    Landed,             // aircraft has landed
    VacatingRunway,     // aircraft vacating runway
};

// Message factories — construct the right Message for each ATC interaction.

inline Message makeClearanceRequest(EntityId sender, EntityId atcId) {
    Message m(MessageType::ATCClearanceRequest, sender, atcId);
    return m;
}

inline Message makeClearanceGranted(EntityId atcId, EntityId aircraft, RunwayId rwy) {
    Message m(MessageType::ATCClearanceGranted, atcId, aircraft);
    m.payload.value = rwy;
    return m;
}

inline Message makeClearanceDenied(EntityId atcId, EntityId aircraft) {
    Message m(MessageType::ATCClearanceDenied, atcId, aircraft);
    return m;
}

inline Message makeTaxiInstruction(EntityId atcId, EntityId aircraft,
                                    double taxiPointX, double taxiPointY) {
    Message m(MessageType::ATCTaxiInstruction, atcId, aircraft);
    m.payload.x = taxiPointX;
    m.payload.y = taxiPointY;
    return m;
}

inline Message makeHoldShort(EntityId atcId, EntityId aircraft, RunwayId rwy) {
    Message m(MessageType::ATCHoldShort, atcId, aircraft);
    m.payload.value = rwy;
    return m;
}

inline Message makeTakeoffPosition(EntityId atcId, EntityId aircraft, RunwayId rwy) {
    Message m(MessageType::ATCTakeoffPosition, atcId, aircraft);
    m.payload.value = rwy;
    return m;
}

inline Message makeClearedTakeoff(EntityId atcId, EntityId aircraft, RunwayId rwy) {
    Message m(MessageType::ATCClearedTakeoff, atcId, aircraft);
    m.payload.value = rwy;
    return m;
}

inline Message makeClearedLanding(EntityId atcId, EntityId aircraft, RunwayId rwy) {
    Message m(MessageType::ATCClearedLanding, atcId, aircraft);
    m.payload.value = rwy;
    return m;
}

inline Message makeGoAround(EntityId atcId, EntityId aircraft) {
    Message m(MessageType::ATCGoAround, atcId, aircraft);
    return m;
}

inline Message makeRunwayClear(EntityId atcId, EntityId aircraft) {
    Message m(MessageType::ATCRunwayClear, atcId, aircraft);
    return m;
}

inline Message makeTrafficAdvisory(EntityId atcId, EntityId aircraft,
                                    double trafficX, double trafficY, double trafficZ) {
    Message m(MessageType::ATCTrafficAdvisory, atcId, aircraft);
    m.payload.x = trafficX;
    m.payload.y = trafficY;
    m.payload.z = trafficZ;
    return m;
}

// Round-2 fix: factories for the new ATC message types that wire up the
// previously-dead handler functions (handleLandingRequest, handleAirborne,
// handleLanded).

inline Message makeLandingRequest(EntityId sender, EntityId atcId) {
    return Message(MessageType::ATCLandingRequest, sender, atcId);
}

inline Message makeAirborneReport(EntityId sender, EntityId atcId) {
    return Message(MessageType::ATCAirborne, sender, atcId);
}

inline Message makeLandedReport(EntityId sender, EntityId atcId) {
    return Message(MessageType::ATCLanded, sender, atcId);
}

// Extract runway ID from a message payload.
inline RunwayId runwayFromMessage(const Message& msg) {
    return msg.payload.value;
}

// Extract taxi point from a message payload.
inline void taxiPointFromMessage(const Message& msg, double& x, double& y) {
    x = msg.payload.x;
    y = msg.payload.y;
}

} // namespace atc
} // namespace digi
} // namespace f4flight
