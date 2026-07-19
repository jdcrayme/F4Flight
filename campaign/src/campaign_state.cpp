// f4flight - campaign/src/campaign_state.cpp
//
// Implementation of core campaign simulation tick, including movement,
// formations, combat, air strikes, and objective capture.

#include "f4flight/campaign/campaign_state.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace f4flight {
namespace campaign {

void CampaignState::tick(double dt) {
    time_ += dt;

    // 1. Reset ground unit speeds and combat states before recalculation
    for (auto& unit : units_) {
        if (!unit.isActive) continue;
        if (unit.type == UnitType::GroundBattalion) {
            unit.speed = 40.0; // standard march speed (ft/s) ~ 24 kts
            unit.formation = FormationState::Column;
        } else if (unit.type == UnitType::AirSquadron) {
            unit.speed = 500.0; // standard cruise speed (ft/s) ~ 300 kts
            unit.formation = FormationState::Wedge;
        }
    }

    // 2. Handle combat and formation updates
    handleCombatAndFormations(dt);

    // 3. Move units
    for (auto& unit : units_) {
        if (!unit.isActive) continue;
        if (unit.type == UnitType::GroundBattalion) {
            handleGroundMovement(unit, dt);
        } else if (unit.type == UnitType::AirSquadron) {
            handleAirMovementAndStrike(unit, dt);
        }
    }

    // 4. Handle objective captures
    handleObjectiveCapture(dt);
}

void CampaignState::handleGroundMovement(Unit& unit, double dt) {
    if (unit.speed <= 0.0) {
        // Stopped in combat or otherwise
        unit.updateSubUnitPositions();
        return;
    }

    // Pathfinding initialization
    if (unit.path.empty() && unit.hasTargetObjective && unit.targetObjectiveId != -1) {
        // Find destination objective
        for (const auto& obj : objectives_) {
            if (obj.id == unit.targetObjectiveId) {
                unit.destination = obj.position;
                unit.path = roadNetwork_.findPath(unit.position, unit.destination);
                unit.currentPathIndex = 0;
                break;
            }
        }
    }

    // Move along path
    if (!unit.path.empty() && unit.currentPathIndex < unit.path.size()) {
        Vec3 target = unit.path[unit.currentPathIndex];
        Vec3 dir = target - unit.position;
        double dist = dir.norm();
        double step = unit.speed * dt;

        if (dist <= step) {
            unit.position = target;
            unit.currentPathIndex++;
        } else {
            unit.position += dir.normalized() * step;
        }
    }

    unit.updateSubUnitPositions();
}

void CampaignState::handleAirMovementAndStrike(Unit& unit, double dt) {
    // Air units fly directly (cross-country)
    if (unit.path.empty() && unit.hasTargetObjective && unit.targetObjectiveId != -1) {
        for (const auto& obj : objectives_) {
            if (obj.id == unit.targetObjectiveId) {
                // Mission path: base -> target -> back to base
                unit.destination = obj.position;
                unit.path = { unit.position, obj.position, unit.position };
                unit.currentPathIndex = 0;
                break;
            }
        }
    }

    if (!unit.path.empty() && unit.currentPathIndex < unit.path.size()) {
        Vec3 target = unit.path[unit.currentPathIndex];
        Vec3 dir = target - unit.position;
        double dist = dir.norm();
        double step = unit.speed * dt;

        if (dist <= step) {
            unit.position = target;
            unit.currentPathIndex++;

            // Overhead of target objective (index 1 is target, so incrementing past it means we hit it)
            if (unit.currentPathIndex == 2) {
                // Strike target objective
                for (auto& obj : objectives_) {
                    if (obj.id == unit.targetObjectiveId && obj.faction != unit.faction) {
                        obj.health = std::max(0.0, obj.health - 0.3);
                        for (auto& s : obj.structures) {
                            s.health = std::max(0.0, s.health - 0.4);
                        }
                        for (auto& rwy : obj.runways) {
                            rwy.health = std::max(0.0, rwy.health - 0.5);
                        }
                    }
                }
            }
        } else {
            unit.position += dir.normalized() * step;
        }
    }

    unit.updateSubUnitPositions();
}

void CampaignState::handleCombatAndFormations(double dt) {
    const double combatRange = 4000.0; // ft

    for (size_t i = 0; i < units_.size(); ++i) {
        auto& u1 = units_[i];
        if (!u1.isActive || u1.type != UnitType::GroundBattalion) continue;

        bool inCombat = false;
        for (size_t j = 0; j < units_.size(); ++j) {
            if (i == j) continue;
            auto& u2 = units_[j];
            if (!u2.isActive || u2.type != UnitType::GroundBattalion) continue;
            if (u1.faction == u2.faction) continue; // allies don't fight

            double dx = u1.position.x - u2.position.x;
            double dy = u1.position.y - u2.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= combatRange) {
                inCombat = true;
                u1.speed = 0.0; // stop to deploy and fight
                u1.formation = FormationState::Wedge;

                // Inflict damage proportional to relative strength
                double u1Power = u1.subUnits.size() * u1.health;
                double u2Power = u2.subUnits.size() * u2.health;

                u1.health = std::max(0.0, u1.health - 0.02 * u2Power * dt);

                // Damage sub-units
                for (auto& su : u1.subUnits) {
                    su.health = std::max(0.0, su.health - 0.04 * u2Power * dt);
                }

                if (u1.health <= 0.0) {
                    u1.isActive = false;
                    break;
                }
            }
        }
    }
}

void CampaignState::handleObjectiveCapture(double dt) {
    const double captureRange = 1500.0; // ft

    for (auto& unit : units_) {
        if (!unit.isActive || unit.type != UnitType::GroundBattalion) continue;

        for (auto& obj : objectives_) {
            if (obj.faction == unit.faction) continue;

            double dx = unit.position.x - obj.position.x;
            double dy = unit.position.y - obj.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= captureRange) {
                // Check if there are any active defender units nearby holding the base
                bool defended = false;
                for (const auto& def : units_) {
                    if (def.isActive && def.faction == obj.faction && def.type == UnitType::GroundBattalion) {
                        double defDx = def.position.x - obj.position.x;
                        double defDy = def.position.y - obj.position.y;
                        if (std::sqrt(defDx * defDx + defDy * defDy) <= captureRange * 2.0) {
                            defended = true;
                            break;
                        }
                    }
                }

                if (!defended) {
                    // Attack the objective and siege it!
                    obj.health = std::max(0.0, obj.health - 0.1 * dt);
                    if (obj.health <= 0.0) {
                        // Capture complete!
                        obj.faction = unit.faction;
                        obj.health = 0.5; // captured in damaged state
                        for (auto& s : obj.structures) {
                            s.health = 0.3; // heavily damaged
                        }
                        for (auto& r : obj.runways) {
                            r.health = 0.3;
                        }
                    }
                }
            }
        }
    }
}

} // namespace campaign
} // namespace f4flight
