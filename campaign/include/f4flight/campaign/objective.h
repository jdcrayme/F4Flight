// f4flight - campaign/include/f4flight/campaign/objective.h
//
// Objectives are stationary strategic points of interest in the campaign map,
// like airbases, cities, and ports.

#pragma once

#include "f4flight/flight/core/types.h"
#include <string>
#include <vector>

namespace f4flight {
namespace campaign {

enum class ObjectiveType {
    Airbase,
    City,
    Depot,
    Port,
    RadarStation,
    Headquarters,
    Factory,
    Fort
};

inline std::string objectiveTypeToString(ObjectiveType type) {
    switch (type) {
        case ObjectiveType::Airbase:      return "Airbase";
        case ObjectiveType::City:         return "City";
        case ObjectiveType::Depot:        return "Depot";
        case ObjectiveType::Port:         return "Port";
        case ObjectiveType::RadarStation: return "RadarStation";
        case ObjectiveType::Headquarters: return "Headquarters";
        case ObjectiveType::Factory:      return "Factory";
        case ObjectiveType::Fort:         return "Fort";
    }
    return "Unknown";
}

struct ObjectiveStructure {
    int id{0};
    std::string name;
    double health{1.0}; // 0.0 to 1.0
    Vec3 offset{};      // relative to objective center (ft)
};

struct Runway {
    int id{0};
    std::string name;
    Vec3 start{};       // world ENU (ft)
    Vec3 end{};         // world ENU (ft)
    double width{150.0}; // ft
    double heading{0.0}; // radians, yaw
    double health{1.0}; // 0.0 to 1.0
};

struct TaxiNode {
    int id{0};
    Vec3 position{};    // world ENU (ft)
    std::string label;  // e.g., "ParkingSpot", "HoldShort", "Threshold", "Exit"
};

struct TaxiEdge {
    int fromId{0};
    int toId{0};
    double distance{0.0}; // ft
};

struct Objective {
    int id{0};
    std::string name;
    ObjectiveType type{ObjectiveType::City};
    Vec3 position{};    // world ENU (ft)
    int faction{0};     // 1 = Blue, 2 = Red, 0 = Neutral
    double health{1.0}; // overall health (0.0 to 1.0)
    std::vector<ObjectiveStructure> structures;

    // Airbase specific fields (only populated if type == Airbase)
    std::vector<Runway> runways;
    std::vector<TaxiNode> taxiNodes;
    std::vector<TaxiEdge> taxiEdges;
};

} // namespace campaign
} // namespace f4flight
