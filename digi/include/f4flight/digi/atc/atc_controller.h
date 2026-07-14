// f4flight - digi/atc/atc_controller.h
//
// ATCController — per-airbase air traffic control.
//
// Manages the runway queue, taxi clearances, and takeoff/landing sequencing
// for one airbase. Aircraft request clearance via the MessageBus; the ATC
// controller grants or denies based on runway state and queue position.
//
// Design:
//   - One ATCController per airbase. The host creates it, gives it an
//     EntityId (so aircraft can address messages to it), and registers its
//     mailbox on the MessageBus.
//   - The controller processes incoming clearance requests each frame and
//     sends responses via the MessageBus.
//   - Runway state: Idle, Departing (aircraft taking off), Arriving
//     (aircraft landing), Closed.
//   - Departure queue: FIFO of aircraft waiting for takeoff.
//   - Arrival queue: ordered by distance to threshold (closest first).
//
// Comparison to FreeFalcon:
//   FreeFalcon's ATCBrain is 2,395 LOC and reaches directly into
//   AircraftClass to command throttle/brakes and read position. Our
//   ATCController is a pure logic class — it only reads messages and sends
//   messages. The DigiBrain's ground ops mode is responsible for actually
//   flying the aircraft based on ATC instructions.

#pragma once

#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/atc/taxi_graph.h"

#include <deque>
#include <vector>

namespace f4flight {
namespace digi {
namespace atc {

// Runway state
enum class RunwayState {
    Idle,       // no one on runway
    Departing,  // aircraft taking off
    Arriving,   // aircraft landing
    Closed,     // runway closed
};

// One runway's state
struct Runway {
    RunwayId id{0};
    RunwayState state{RunwayState::Idle};
    EntityId currentUser{kInvalidEntityId};  // aircraft on runway
    double clearTime{0.0};  // sim time when runway will be clear
};

class ATCController {
public:
    ATCController(EntityId atcId) : atcId_(atcId) {}

    EntityId id() const { return atcId_; }

    // Set the taxi graph for this airbase.
    void setTaxiGraph(const TaxiGraph* graph) { taxiGraph_ = graph; }

    // Add a runway to this airbase.
    void addRunway(RunwayId rwy) {
        Runway r;
        r.id = rwy;
        runways_.push_back(r);
    }

    // Get the mailbox for receiving messages.
    Mailbox& mailbox() { return mailbox_; }

    // Process incoming messages and update state. Called each frame.
    //   simTime : current sim time (seconds)
    //   bus     : message bus for sending responses
    void update(double simTime, MessageBus& bus);

    // Request takeoff clearance (called by aircraft via message).
    // Returns the runway assigned, or kInvalidRunway if denied.
    // (Internal logic — aircraft use the message system, not this directly.)
    void handleClearanceRequest(EntityId aircraft, double simTime, MessageBus& bus);

    // Request landing clearance.
    void handleLandingRequest(EntityId aircraft, double simTime, MessageBus& bus);

    // Notify ATC that an aircraft has vacated the runway.
    void handleRunwayVacated(EntityId aircraft, double simTime, MessageBus& bus);

    // Notify ATC that an aircraft has taken off.
    void handleAirborne(EntityId aircraft, double simTime, MessageBus& bus);

    // Notify ATC that an aircraft has landed.
    void handleLanded(EntityId aircraft, double simTime, MessageBus& bus);

    // Query state (for testing)
    std::size_t departureQueueSize() const { return departureQueue_.size(); }
    std::size_t arrivalQueueSize() const { return arrivalQueue_.size(); }
    RunwayState runwayState(RunwayId rwy) const;

private:
    EntityId atcId_;
    Mailbox mailbox_;
    const TaxiGraph* taxiGraph_{nullptr};
    std::vector<Runway> runways_;

    // Departure queue: FIFO of aircraft waiting for takeoff
    std::deque<EntityId> departureQueue_;

    // Arrival queue: FIFO of aircraft waiting to land
    std::deque<EntityId> arrivalQueue_;

    // Find an idle runway, or the one with the soonest clear time.
    Runway* findAvailableRunway();
    Runway* findRunway(RunwayId rwy);
};

} // namespace atc
} // namespace digi
} // namespace f4flight
