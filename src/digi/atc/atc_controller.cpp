// f4flight - digi/atc/atc_controller.cpp
//
// ATC controller implementation.

#include "f4flight/digi/atc/atc_controller.h"
#include "f4flight/digi/comms/message_bus.h"

#include <algorithm>

namespace f4flight {
namespace digi {
namespace atc {

void ATCController::update(double simTime, MessageBus& bus) {
    // Process all messages in the mailbox
    while (auto msg = mailbox_.pop()) {
        switch (msg->type) {
            case MessageType::ATCClearanceRequest:
                // Determine if this is a takeoff or landing request.
                // For simplicity, we treat all clearance requests as takeoff
                // requests unless the aircraft is already on approach.
                // A real implementation would have separate request types.
                handleClearanceRequest(msg->sender, simTime, bus);
                break;

            case MessageType::ATCRunwayClear:
                handleRunwayVacated(msg->sender, simTime, bus);
                break;

            default:
                break;
        }
    }

    // Check if any runway is ready to clear
    for (auto& rwy : runways_) {
        if (rwy.state != RunwayState::Idle && rwy.clearTime > 0.0 &&
            simTime >= rwy.clearTime) {
            rwy.state = RunwayState::Idle;
            rwy.currentUser = kInvalidEntityId;
            rwy.clearTime = 0.0;
        }
    }

    // Process departure queue: if a runway is idle, clear the next departure
    for (auto& rwy : runways_) {
        if (rwy.state == RunwayState::Idle && !departureQueue_.empty()) {
            EntityId next = departureQueue_.front();
            departureQueue_.pop_front();
            rwy.state = RunwayState::Departing;
            rwy.currentUser = next;
            rwy.clearTime = 0.0;  // will be set when aircraft reports airborne

            // Send takeoff clearance
            bus.publish(makeClearedTakeoff(atcId_, next, rwy.id), simTime);
        }
    }

    // Process arrival queue: if a runway is idle, clear the next arrival
    for (auto& rwy : runways_) {
        if (rwy.state == RunwayState::Idle && !arrivalQueue_.empty()) {
            EntityId next = arrivalQueue_.front();
            arrivalQueue_.pop_front();
            rwy.state = RunwayState::Arriving;
            rwy.currentUser = next;
            rwy.clearTime = 0.0;

            // Send landing clearance
            bus.publish(makeClearedLanding(atcId_, next, rwy.id), simTime);
        }
    }
}

void ATCController::handleClearanceRequest(EntityId aircraft, double simTime, MessageBus& bus) {
    // For now, all requests go to the departure queue.
    // A real implementation would distinguish takeoff vs landing requests.

    // Check if aircraft is already in queue
    auto it = std::find(departureQueue_.begin(), departureQueue_.end(), aircraft);
    if (it != departureQueue_.end()) {
        // Already queued — deny duplicate
        bus.publish(makeClearanceDenied(atcId_, aircraft), simTime);
        return;
    }

    // Check if an idle runway is available
    Runway* rwy = findAvailableRunway();
    if (rwy) {
        // Immediate clearance
        rwy->state = RunwayState::Departing;
        rwy->currentUser = aircraft;
        bus.publish(makeClearedTakeoff(atcId_, aircraft, rwy->id), simTime);
    } else {
        // Queue the request
        departureQueue_.push_back(aircraft);
        // Send hold-short instruction
        bus.publish(makeHoldShort(atcId_, aircraft, runways_.empty() ? 0 : runways_[0].id), simTime);
    }
}

void ATCController::handleLandingRequest(EntityId aircraft, double simTime, MessageBus& bus) {
    auto it = std::find(arrivalQueue_.begin(), arrivalQueue_.end(), aircraft);
    if (it != arrivalQueue_.end()) {
        return;  // already queued
    }

    Runway* rwy = findAvailableRunway();
    if (rwy) {
        rwy->state = RunwayState::Arriving;
        rwy->currentUser = aircraft;
        bus.publish(makeClearedLanding(atcId_, aircraft, rwy->id), simTime);
    } else {
        arrivalQueue_.push_back(aircraft);
        // Aircraft must wait — send traffic advisory
        bus.publish(makeTrafficAdvisory(atcId_, aircraft, 0, 0, 0), simTime);
    }
}

void ATCController::handleRunwayVacated(EntityId aircraft, double simTime, MessageBus& bus) {
    (void)bus;
    (void)simTime;  // vacate is event-driven; no timestamp needed
    for (auto& rwy : runways_) {
        if (rwy.currentUser == aircraft) {
            rwy.state = RunwayState::Idle;
            rwy.currentUser = kInvalidEntityId;
            rwy.clearTime = 0.0;
            break;
        }
    }
}

void ATCController::handleAirborne(EntityId aircraft, double simTime, MessageBus& bus) {
    (void)simTime;
    (void)bus;
    // Mark the runway as clear after a short delay
    for (auto& rwy : runways_) {
        if (rwy.currentUser == aircraft) {
            rwy.clearTime = simTime + 10.0;  // runway clear in 10s
            break;
        }
    }
}

void ATCController::handleLanded(EntityId aircraft, double simTime, MessageBus& bus) {
    (void)simTime;
    (void)bus;
    // Mark the runway as clear after a short delay
    for (auto& rwy : runways_) {
        if (rwy.currentUser == aircraft) {
            rwy.clearTime = simTime + 15.0;  // runway clear in 15s
            break;
        }
    }
}

Runway* ATCController::findAvailableRunway() {
    for (auto& rwy : runways_) {
        if (rwy.state == RunwayState::Idle) return &rwy;
    }
    return nullptr;
}

Runway* ATCController::findRunway(RunwayId rwyId) {
    for (auto& rwy : runways_) {
        if (rwy.id == rwyId) return &rwy;
    }
    return nullptr;
}

RunwayState ATCController::runwayState(RunwayId rwy) const {
    for (const auto& r : runways_) {
        if (r.id == rwy) return r.state;
    }
    return RunwayState::Closed;
}

} // namespace atc
} // namespace digi
} // namespace f4flight
