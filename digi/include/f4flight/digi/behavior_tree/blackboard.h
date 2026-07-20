#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/digi/digi_entity.h"
#include <memory>

namespace f4flight {
namespace digi {

class FlightPlan;

class Blackboard {
public:
    // References to the active, live simulation parameters
    const AircraftState* as {nullptr};
    double dt {0.0};
    double groundZ {0.0};

    // Core inputs and persistence state
    DigiState* state {nullptr};
    const DigiEntity* self {nullptr};
    const DigiEntity* target {nullptr};

    // Tactical role information
    bool isLead {true};
    EntityId leadId {kInvalidEntityId};

    // Active flight plan
    std::shared_ptr<FlightPlan> flightPlan;
};

} // namespace digi
} // namespace f4flight
