// f4flight - campaign/include/f4flight/campaign/campaign_state.h
//
// CampaignState represents the complete snapshot of the battlefield in memory,
// including the terrain map, road networks, objectives, and units. It runs
// the core simulation tick.

#pragma once

#include "terrain_map.h"
#include "road_network.h"
#include "objective.h"
#include "unit.h"
#include <vector>
#include <memory>

namespace f4flight {
namespace campaign {

class CampaignState {
public:
    CampaignState() = default;

    TerrainMap& terrain() { return terrain_; }
    const TerrainMap& terrain() const { return terrain_; }

    RoadNetwork& roadNetwork() { return roadNetwork_; }
    const RoadNetwork& roadNetwork() const { return roadNetwork_; }

    std::vector<Objective>& objectives() { return objectives_; }
    const std::vector<Objective>& objectives() const { return objectives_; }

    std::vector<Unit>& units() { return units_; }
    const std::vector<Unit>& units() const { return units_; }

    double time() const { return time_; }
    void setTime(double t) { time_ = t; }

    // Add elements
    void addObjective(const Objective& obj) { objectives_.push_back(obj); }
    void addUnit(const Unit& unit) { units_.push_back(unit); }

    // Simulation tick - executes ground movement, formation shifts, combat,
    // air squadron strikes, and objective capture mechanics.
    void tick(double dt);

private:
    void handleGroundMovement(Unit& unit, double dt);
    void handleAirMovementAndStrike(Unit& unit, double dt);
    void handleCombatAndFormations(double dt);
    void handleObjectiveCapture(double dt);

    TerrainMap terrain_;
    RoadNetwork roadNetwork_;
    std::vector<Objective> objectives_;
    std::vector<Unit> units_;
    double time_{0.0};
};

} // namespace campaign
} // namespace f4flight
