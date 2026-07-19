// f4flight - campaign/src/unit.cpp
//
// Implementation of dynamic subunit positions based on formation states.

#include "f4flight/campaign/unit.h"
#include <cmath>

namespace f4flight {
namespace campaign {

void Unit::updateSubUnitPositions() {
    if (subUnits.empty()) return;

    // 1. Compute heading and unit direction
    double hdg = 0.0;
    if (currentPathIndex < path.size()) {
        Vec3 toNext = path[currentPathIndex] - position;
        if (toNext.norm() > 1e-3) {
            hdg = std::atan2(toNext.y, toNext.x);
        }
    } else if (!path.empty() && currentPathIndex > 0) {
        Vec3 prevDir = position - path[currentPathIndex - 1];
        if (prevDir.norm() > 1e-3) {
            hdg = std::atan2(prevDir.y, prevDir.x);
        }
    }

    Vec3 forwardDir(std::cos(hdg), std::sin(hdg), 0.0);
    Vec3 rightDir(std::sin(hdg), -std::cos(hdg), 0.0); // perpendicular to forward

    const size_t numVehicles = subUnits.size();
    const double spacing = 80.0; // ft between vehicles
    const double stagger = 50.0; // ft stagger back for wedge

    if (formation == FormationState::Column) {
        // Arrange vehicles sequentially behind the leader along the path or reverse heading
        subUnits[0].position = position;
        subUnits[0].heading = hdg;

        for (size_t i = 1; i < numVehicles; ++i) {
            double targetDist = i * spacing;
            Vec3 vehiclePos = position;
            double vehicleHdg = hdg;

            // Attempt to walk backward along the path to find the segment at targetDist
            double distRemaining = targetDist;
            Vec3 currentRef = position;
            size_t pathIdx = currentPathIndex;

            bool foundOnPath = false;
            while (pathIdx > 0 && distRemaining > 0.0) {
                Vec3 prevPoint = path[pathIdx - 1];
                Vec3 segment = prevPoint - currentRef;
                double segLen = segment.norm();

                if (segLen >= distRemaining) {
                    vehiclePos = currentRef + segment.normalized() * distRemaining;
                    if (segLen > 1e-3) {
                        vehicleHdg = std::atan2(-segment.y, -segment.x);
                    }
                    distRemaining = 0.0;
                    foundOnPath = true;
                    break;
                } else {
                    distRemaining -= segLen;
                    currentRef = prevPoint;
                    pathIdx--;
                }
            }

            // Fallback: project backward along the reverse heading
            if (!foundOnPath) {
                vehiclePos = currentRef - forwardDir * distRemaining;
                vehicleHdg = hdg;
            }

            subUnits[i].position = vehiclePos;
            subUnits[i].heading = vehicleHdg;
        }
    } else if (formation == FormationState::Wedge) {
        // Wedge (V-shape)
        subUnits[0].position = position;
        subUnits[0].heading = hdg;

        for (size_t i = 1; i < numVehicles; ++i) {
            double rank = std::ceil(i / 2.0);
            Vec3 latOffset = rightDir * (rank * spacing);
            Vec3 longOffset = -forwardDir * (rank * stagger);

            if (i % 2 == 0) {
                // Left wing
                subUnits[i].position = position - latOffset + longOffset;
            } else {
                // Right wing
                subUnits[i].position = position + latOffset + longOffset;
            }
            subUnits[i].heading = hdg;
        }
    } else if (formation == FormationState::Line) {
        // Flat perpendicular line
        for (size_t i = 0; i < numVehicles; ++i) {
            double offsetFactor = i - (numVehicles - 1) / 2.0;
            subUnits[i].position = position + rightDir * (offsetFactor * spacing);
            subUnits[i].heading = hdg;
        }
    }
}

} // namespace campaign
} // namespace f4flight
