// f4flight - digi/sensors/sensor.h
//
// Sensor — abstract base for all sensor types.
//
// Each sensor filters the TruthState (all entities) into a list of
// SensorContacts. The SensorFusion then merges contacts from all sensors
// into a unified SensorPicture.
//
// Design:
//   - Sensors are stateless filters — they don't own contacts, just produce them.
//   - Each sensor has a detection model: range, field of regard, detection
//     probability, skill gating.
//   - The host provides a TruthState (list of all entities) each frame.
//   - Sensors produce a list of SensorContacts with confidence and sensor type.
//
// Comparison to FreeFalcon:
//   FreeFalcon's SensorClass hierarchy (Radar, RWR, Visual, IRST, HTS) is
//   coupled to SimObjectType and reads directly from SimMoverClass. Detection
//   probability is scattered. Our sensors are pure functions of (truth, self,
//   skill) — no sim entity coupling.

#pragma once

#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_skill.h"

#include <vector>

namespace f4flight {
namespace digi {

// TruthState — the complete world state the host provides each frame.
// This is what the sensors filter. The host populates this from its own
// entity model.
struct TruthState {
    // All entities in the world (aircraft, missiles, SAMs, etc.)
    std::vector<DigiEntity> entities;

    // Entity IDs corresponding to each entity (parallel array)
    std::vector<EntityId> ids;

    // Which entities are firing guns this frame? (parallel array, by index)
    std::vector<bool> firing;

    void clear() {
        entities.clear();
        ids.clear();
        firing.clear();
    }

    void add(EntityId id, const DigiEntity& entity, bool isFiring = false) {
        entities.push_back(entity);
        ids.push_back(id);
        firing.push_back(isFiring);
    }

    std::size_t size() const { return entities.size(); }
};

// SensorConfig — per-sensor configuration (range, field of regard, etc.)
struct SensorConfig {
    double maxRangeFt{0.0};       // maximum detection range (ft)
    double maxAzRad{PI};          // max azimuth off-nose (field of regard)
    double maxElRad{PI};          // max elevation off-nose
    double minConfidence{0.1};    // minimum confidence to report a contact
    bool   enabled{true};
};

// Sensor — abstract base class.
class Sensor {
public:
    virtual ~Sensor() = default;

    // What type of sensor is this?
    virtual SensorType type() const = 0;

    // Configure the sensor (range, field of regard, etc.)
    void configure(const SensorConfig& cfg) { config_ = cfg; }
    const SensorConfig& config() const { return config_; }

    // Update: filter the truth state into contacts.
    //   self      : own aircraft entity
    //   truth     : all entities in the world
    //   skill     : AI skill level (affects detection probability)
    //   dt        : frame time (seconds)
    //   out       : output contacts (appended to)
    virtual void update(const DigiEntity& self, const TruthState& truth,
                        const SkillParameters& skill, double dt,
                        std::vector<SensorContact>& out) = 0;

protected:
    SensorConfig config_;

    // Helper: compute relative geometry and check if target is in field of regard
    bool inFieldOfRegard(const DigiEntity& self, const DigiEntity& target,
                         double& range, double& az, double& el) const {
        const RelativeGeometry rg = computeRelativeGeometry(self, target);
        range = rg.range;
        az = rg.az;
        el = rg.el;
        return (range <= config_.maxRangeFt &&
                std::fabs(az) <= config_.maxAzRad &&
                std::fabs(el) <= config_.maxElRad);
    }

    // Helper: create a basic contact from an entity
    SensorContact makeContact(EntityId id, const DigiEntity& entity,
                               double confidence) const {
        SensorContact c;
        c.entityId = id;
        c.x = entity.x; c.y = entity.y; c.z = entity.z;
        c.vx = entity.vx; c.vy = entity.vy; c.vz = entity.vz;
        c.yaw = entity.yaw; c.pitch = entity.pitch; c.roll = entity.roll;
        c.speed = entity.speed;
        c.isFiring = entity.isFiring;
        c.isRadarEmitting = entity.isRadarEmitting;
        c.addSensor(type());
        c.confidence = confidence;
        c.quality = ContactQuality::Detected;
        return c;
    }
};

} // namespace digi
} // namespace f4flight
