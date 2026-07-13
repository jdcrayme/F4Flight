// f4flight - digi/sensors/sensor_fusion.cpp
//
// SensorFusion implementation.

#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/digi/sensors/radar_sensor.h"
#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/digi/sensors/visual_sensor.h"
#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

SensorFusion::SensorFusion() {
    // Default sensor suite: radar + RWR + visual (typical fighter)
    addSensor(std::make_unique<RadarSensor>());
    addSensor(std::make_unique<RWRSensor>());
    addSensor(std::make_unique<VisualSensor>());
}

void SensorFusion::update(const DigiEntity& self, const TruthState& truth,
                           const SkillParameters& skill, double dt) {
    // Age existing contacts and remove expired ones
    ageAndPurge(dt, 5.0);

    // Run each sensor and collect contacts
    std::vector<SensorContact> newContacts;
    for (auto& sensor : sensors_) {
        if (!sensor->config().enabled) continue;
        sensor->update(self, truth, skill, dt, newContacts);
    }

    // Merge new contacts into the picture
    for (const auto& c : newContacts) {
        mergeContact(c);
    }

    // Compute threat scores and identify highest threat, best target, etc.
    computeThreatScores(self);
}

void SensorFusion::mergeContact(const SensorContact& contact) {
    // Find existing contact with same entity ID
    for (auto& existing : picture_.contacts) {
        if (existing.entityId == contact.entityId) {
            // Merge: update position/velocity, combine sensor mask, take max confidence
            existing.x = contact.x;
            existing.y = contact.y;
            existing.z = contact.z;
            existing.vx = contact.vx;
            existing.vy = contact.vy;
            existing.vz = contact.vz;
            existing.yaw = contact.yaw;
            existing.pitch = contact.pitch;
            existing.roll = contact.roll;
            existing.speed = contact.speed;
            existing.sensorMask |= contact.sensorMask;
            existing.confidence = std::max(existing.confidence, contact.confidence);
            existing.age = 0.0;  // refresh

            // Upgrade quality if the new contact is higher
            if (contact.quality > existing.quality) {
                existing.quality = contact.quality;
            }

            // Merge flags
            existing.isMissile = existing.isMissile || contact.isMissile;
            existing.isFiring = existing.isFiring || contact.isFiring;
            existing.isThreat = existing.isThreat || contact.isThreat;

            // Upgrade type if identified
            if (contact.type != ContactType::Unknown && existing.type == ContactType::Unknown) {
                existing.type = contact.type;
            }
            return;
        }
    }

    // New contact — add to picture
    picture_.contacts.push_back(contact);
}

void SensorFusion::computeThreatScores(const DigiEntity& self) {
    // Reset pre-computed pointers
    picture_.highestThreat = nullptr;
    picture_.bestTarget = nullptr;
    picture_.incomingMissile = nullptr;
    picture_.gunsThreat = nullptr;
    picture_.spiked = false;
    picture_.spikeHeading = 0.0;

    double maxThreatScore = 0.0;
    double bestTargetScore = 0.0;

    for (auto& c : picture_.contacts) {
        const RelativeGeometry rg = computeRelativeGeometry(self,
            DigiEntity{c.x, c.y, c.z, c.vx, c.vy, c.vz,
                       c.yaw, c.pitch, c.roll, c.speed});

        // --- Threat score ---
        // Higher = more dangerous
        double threat = 0.0;

        // Range factor: closer = more dangerous
        if (rg.range > 0.0) {
            threat += 10000.0 / std::max(rg.range, 1000.0);
        }

        // ATA factor: pointing at us = dangerous
        threat += (1.0 - std::fabs(rg.ataFrom) / PI) * 50.0;

        // Type factor
        if (c.isMissile) {
            threat += 200.0;  // missiles are very dangerous
        } else if (c.type == ContactType::Fighter) {
            threat += 50.0;
        } else if (c.type == ContactType::SAM) {
            threat += 100.0;
        }

        // Firing factor
        if (c.isFiring) {
            threat += 100.0;
        }

        // Confidence factor
        threat *= c.confidence;

        c.threatScore = threat;

        // Track highest threat
        if (threat > maxThreatScore) {
            maxThreatScore = threat;
            picture_.highestThreat = &c;
        }

        // Track incoming missile
        if (c.isMissile && rg.range < 30.0 * 6076.0) {
            if (!picture_.incomingMissile ||
                rg.range < computeRelativeGeometry(self,
                    DigiEntity{picture_.incomingMissile->x,
                               picture_.incomingMissile->y,
                               picture_.incomingMissile->z,
                               picture_.incomingMissile->vx,
                               picture_.incomingMissile->vy,
                               picture_.incomingMissile->vz}).range) {
                picture_.incomingMissile = &c;
            }
        }

        // Track guns threat (firing + close + in gun cone)
        if (c.isFiring && rg.range < 6000.0 &&
            std::fabs(rg.ataFrom) < 30.0 * DTR) {
            if (!picture_.gunsThreat) {
                picture_.gunsThreat = &c;
            }
        }

        // Track best target (aircraft, not missile, in WVR range)
        if (!c.isMissile && c.type == ContactType::Fighter &&
            rg.range < 8.0 * 6076.0) {
            // Best target: highest confidence, closest range
            double targetScore = c.confidence * 100.0 - rg.range / 1000.0;
            if (targetScore > bestTargetScore) {
                bestTargetScore = targetScore;
                picture_.bestTarget = &c;
            }
        }

        // Spike detection: someone locking us up (radar pointing at us)
        if (c.detectedBy(SensorType::RWR) && std::fabs(rg.ataFrom) < 15.0 * DTR &&
            rg.range < 50.0 * 6076.0) {
            picture_.spiked = true;
            picture_.spikeHeading = std::atan2(c.y - self.y, c.x - self.x);
        }
    }
}

void SensorFusion::ageAndPurge(double dt, double maxAge) {
    // Age all contacts
    for (auto& c : picture_.contacts) {
        c.age += dt;
    }

    // Remove expired contacts
    picture_.contacts.erase(
        std::remove_if(picture_.contacts.begin(), picture_.contacts.end(),
            [maxAge](const SensorContact& c) { return c.age > maxAge; }),
        picture_.contacts.end());
}

} // namespace digi
} // namespace f4flight
