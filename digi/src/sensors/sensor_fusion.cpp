// f4flight - digi/sensors/sensor_fusion.cpp
//
// SensorFusion implementation.

#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/digi/sensors/radar_sensor.h"
#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/digi/sensors/visual_sensor.h"
#include "f4flight/flight/core/constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
    // Save the sticky missile ID before ageAndPurge invalidates the pointer.
    if (picture_.incomingMissile) {
        stickyMissileId_ = picture_.incomingMissile->entityId;
    }

    // Clear all pre-computed pointers — ageAndPurge will invalidate them.
    picture_.highestThreat = nullptr;
    picture_.bestTarget = nullptr;
    picture_.incomingMissile = nullptr;
    picture_.gunsThreat = nullptr;

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
    // Pre-computed pointers were cleared at the top of update(), so we
    // only need to reset the spike flag here.
    picture_.spiked = false;
    picture_.spikeHeading = 0.0;

    double maxThreatScore = 0.0;
    double bestTargetScore = 0.0;

    // Find the range of the previously-tracked (sticky) missile, if any,
    // so we can decide whether to keep tracking it or swap to a closer one.
    double stickyMissileRange = std::numeric_limits<double>::infinity();
    bool stickyMissileStillPresent = false;
    if (stickyMissileId_ != kInvalidEntityId) {
        for (const auto& c : picture_.contacts) {
            if (c.entityId == stickyMissileId_) {
                const RelativeGeometry rg = computeRelativeGeometry(self,
                    DigiEntity{c.x, c.y, c.z, c.vx, c.vy, c.vz,
                               c.yaw, c.pitch, c.roll, c.speed});
                stickyMissileRange = rg.range;
                stickyMissileStillPresent = true;
                break;
            }
        }
    }
    if (!stickyMissileStillPresent) {
        stickyMissileId_ = kInvalidEntityId;
    }

    double newMissileRange = std::numeric_limits<double>::infinity();

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

        // Track incoming missile. Sticky-track by entityId: keep the
        // current missile unless a different missile is dramatically closer
        // (< 0.5× range). This prevents two missiles at similar ranges from
        // thrashing pic.incomingMissile every frame, which would defeat the
        // brain's per-missile state initialization (Bug D/H).
        if (c.isMissile && rg.range < 30.0 * 6076.0) {
            const bool isStickyMissile =
                (stickyMissileId_ != kInvalidEntityId) &&
                (c.entityId == stickyMissileId_);
            if (isStickyMissile) {
                // Keep the sticky missile.
                picture_.incomingMissile = &c;
            } else if (!picture_.incomingMissile) {
                // No current missile — pick this one if it's the closest so far.
                if (rg.range < newMissileRange) {
                    newMissileRange = rg.range;
                    picture_.incomingMissile = &c;
                }
            } else {
                // We have a missile but it's not this one. Swap only if this
                // one is dramatically closer (e.g. < 0.5× range) — prevents
                // oscillation between two similar-range missiles.
                if (rg.range < 0.5 * stickyMissileRange && rg.range < newMissileRange) {
                    newMissileRange = rg.range;
                    picture_.incomingMissile = &c;
                }
            }
        }

        // Track guns threat (firing + close + in gun cone)
        if (c.isFiring && rg.range < 6000.0 &&
            std::fabs(rg.ataFrom) < 30.0 * DTR) {
            if (!picture_.gunsThreat) {
                picture_.gunsThreat = &c;
            }
        }

        // Track best target. We consider any airborne hostile (Fighter,
        // Bomber, Helicopter) plus SAM/AAA for AG missions — not just
        // Fighters. Transports/Tankers/AWACS are excluded unless they are
        // the only thing detected (rare; the brain can still target them
        // via host injection).
        const bool isAirTarget =
            c.type == ContactType::Fighter ||
            c.type == ContactType::Bomber ||
            c.type == ContactType::Helicopter;
        if (!c.isMissile && isAirTarget && rg.range < 8.0 * 6076.0) {
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

    // Update sticky ID for next frame.
    if (picture_.incomingMissile) {
        stickyMissileId_ = picture_.incomingMissile->entityId;
    } else {
        stickyMissileId_ = kInvalidEntityId;
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
