// f4flight - campaign/include/f4flight/campaign/unit.h
//
// Units represent the mobile combat forces (Ground Battalions, Air Squadrons)
// in the campaign. Supports dynamic formation layout calculations.

#pragma once

#include "f4flight/flight/core/types.h"
#include <string>
#include <vector>

namespace f4flight {
namespace campaign {

enum class UnitType {
    GroundBattalion,
    AirSquadron,
    NavalTaskForce
};

enum class FormationState {
    Column,  // marching sequentially along a road or path
    Wedge,   // tactical wedge dispersal
    Line     // tactical line dispersal
};

inline std::string unitTypeToString(UnitType type) {
    switch (type) {
        case UnitType::GroundBattalion: return "GroundBattalion";
        case UnitType::AirSquadron:    return "AirSquadron";
        case UnitType::NavalTaskForce:   return "NavalTaskForce";
    }
    return "Unknown";
}

inline std::string formationStateToString(FormationState state) {
    switch (state) {
        case FormationState::Column: return "Column";
        case FormationState::Wedge:  return "Wedge";
        case FormationState::Line:   return "Line";
    }
    return "Unknown";
}

struct SubUnit {
    int id{0};
    std::string name;
    double health{1.0}; // 0.0 to 1.0
    Vec3 position{};    // world ENU (ft)
    double heading{0.0}; // radians (yaw)
};

struct Unit {
    int id{0};
    std::string name;
    UnitType type{UnitType::GroundBattalion};
    int faction{0};     // 1 = Blue, 2 = Red, 0 = Neutral
    Vec3 position{};    // world ENU (ft)
    Vec3 destination{}; // target world ENU (ft)
    double speed{0.0};  // ft/s
    double health{1.0}; // 0.0 to 1.0
    bool isActive{true};
    bool hasTargetObjective{false};
    int targetObjectiveId{-1};

    FormationState formation{FormationState::Column};

    // Sub-units / vehicles / aircraft
    std::vector<SubUnit> subUnits;

    // Movement Path / Waypoints (marching routes or flightpaths)
    std::vector<Vec3> path;
    size_t currentPathIndex{0};

    // Computes dynamic coordinates for all subUnits based on the current formation,
    // position, path, and heading. This is used by the simulation tick and renderer.
    void updateSubUnitPositions();
};

} // namespace campaign
} // namespace f4flight
