// f4flight - digi/sensors/sensor_fusion.h
//
// SensorFusion — merges contacts from all sensors into a unified SensorPicture.
//
// Each frame:
//   1. The host calls SensorFusion::update() with the TruthState and the
//      list of active sensors.
//   2. Each sensor filters the truth into a list of contacts.
//   3. SensorFusion merges these into a single contact list, resolving
//      duplicates (same entity detected by multiple sensors).
//   4. SensorFusion computes threat scores and identifies the highest
//      threat, best target, incoming missiles, and guns threats.
//   5. The DigiBrain reads the resulting SensorPicture.
//
// This replaces the "injected threat" model. Instead of the host setting
// incomingMissile / gunsThreat / target pointers, the brain detects them
// autonomously via the SensorPicture.
//
// Threat scoring (simplified from FreeFalcon sfusion.cpp):
//   - Range: closer = more dangerous
//   - ATA: pointing at us = more dangerous
//   - Type: missile > fighter > bomber > transport
//   - Aspect: behind us = more dangerous (we can't see it)

#pragma once

#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/digi/sensors/sensor.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/comms/message.h"  // for EntityId, kInvalidEntityId

#include <vector>
#include <memory>

namespace f4flight {
namespace digi {

class SensorFusion {
public:
    SensorFusion();

    // Add a sensor to the fusion (takes ownership)
    void addSensor(std::unique_ptr<Sensor> sensor) {
        sensors_.push_back(std::move(sensor));
    }

    // Remove all sensors
    void clearSensors() {
        sensors_.clear();
    }

    // Get the number of sensors
    std::size_t sensorCount() const { return sensors_.size(); }

    // Update: run all sensors, merge contacts, compute threat scores.
    //   self      : own aircraft entity
    //   truth     : all entities in the world
    //   skill     : AI skill level
    //   dt        : frame time (seconds)
    void update(const DigiEntity& self, const TruthState& truth,
                const SkillParameters& skill, double dt);

    // Get the current sensor picture (read by the DigiBrain)
    const SensorPicture& picture() const { return picture_; }

    // Get a mutable reference (for testing)
    SensorPicture& picture() { return picture_; }

    // Reset all tracked contacts and the sticky missile track. Called by
    // DigiBrain::reset() so stale contacts from a previous scenario (e.g.
    // a guns target left in bestTarget) don't leak into the next scenario
    // and trigger spurious WVREngage / GunsEngage entries.
    void reset() {
        picture_.clear();
        stickyMissileId_ = kInvalidEntityId;
    }

private:
    std::vector<std::unique_ptr<Sensor>> sensors_;
    SensorPicture picture_;

    // Sticky-track ID for the incoming missile. Prevents thrash between
    // two similar-range missiles. Stored here (not in SensorPicture) because
    // picture_.incomingMissile is a pointer that gets invalidated every
    // frame by ageAndPurge.
    EntityId stickyMissileId_{kInvalidEntityId};

    // Merge a new contact into the picture (combines with existing if same ID)
    void mergeContact(const SensorContact& contact);

    // Compute threat scores and identify highest threat, best target, etc.
    void computeThreatScores(const DigiEntity& self);

    // Age all contacts and remove expired ones
    void ageAndPurge(double dt, double maxAge = 5.0);
};

} // namespace digi
} // namespace f4flight
