// f4flight - digi/sensors/sensor_picture.h
//
// SensorPicture — the AI's "knowledge" of the world.
//
// The SensorPicture is a filtered view of the truth state. It contains only
// what the AI's sensors can detect: contacts with position, velocity, type
// (if identified), confidence, and which sensor detected them.
//
// Design principles:
//   1. The AI NEVER reads truth directly — only the SensorPicture.
//   2. Contacts are ephemeral — they appear when detected, fade when lost.
//   3. Each contact tracks which sensor detected it and a confidence value.
//   4. The SensorFusion merges contacts from all sensors into one picture.
//
// This replaces the "injected threat" model (host sets incomingMissile /
// gunsThreat / target pointers directly) with an autonomous detection model.
// The host populates a TruthState; sensors filter it; the brain reads the
// SensorPicture.

#pragma once

#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/comms/message.h"  // for EntityId
#include "f4flight/core/types.h"

#include <vector>
#include <cstdint>

namespace f4flight {
namespace digi {

// SensorType — which sensor detected this contact
enum class SensorType : uint8_t {
    None    = 0,
    Radar   = 1,   // radar (RWS/TWS/SAM/STT)
    RWR     = 2,   // radar warning receiver (detects radar emissions)
    Visual  = 3,   // eyeball / visual
    IRST    = 4,   // infrared search & track
    HTS     = 5,   // HARM targeting system (SEAD)
    GCI     = 6,   // ground control intercept (skill-gated)
};

// ContactType — what kind of entity this is (if identified)
enum class ContactType : uint8_t {
    Unknown     = 0,
    Fighter     = 1,   // identified fighter
    Bomber      = 2,   // identified bomber
    Transport   = 3,   // identified transport/cargo
    Tanker      = 4,   // identified tanker
    AWACS       = 5,   // identified AWACS/JSTARS
    Helicopter  = 6,
    Missile     = 7,   // incoming missile
    SAM         = 8,   // surface-to-air missile site
    AAA         = 9,   // anti-aircraft artillery
};

// ContactQuality — how well the contact is tracked
enum class ContactQuality : uint8_t {
    None        = 0,  // not tracked
    Detected    = 1,  // something is there, position uncertain
    Tracked     = 2,  // position tracked, type unknown
    Identified  = 3,  // position + type known
};

// SensorContact — one entity in the SensorPicture.
struct SensorContact {
    EntityId entityId{0};           // unique ID (from truth)

    // Position (world frame, ft) — may be stale or uncertain
    double x{0.0}, y{0.0}, z{0.0};

    // Velocity (ft/s)
    double vx{0.0}, vy{0.0}, vz{0.0};

    // Attitude (radians) — only if visually tracked
    double yaw{0.0}, pitch{0.0}, roll{0.0};

    // Speed (ft/s) — convenience
    double speed{0.0};

    // Classification
    ContactType type{ContactType::Unknown};
    ContactQuality quality{ContactQuality::Detected};

    // Which sensor detected this contact (bitmask — multiple sensors can contribute)
    uint16_t sensorMask{0};

    // Confidence [0, 1] — detection probability * track quality
    double confidence{0.0};

    // Time since last update (seconds) — contacts expire if not refreshed
    double age{0.0};

    // Is this contact threatening us? (set by SensorFusion threat scoring)
    bool isThreat{false};

    // Is this an incoming missile? (set by SensorFusion)
    bool isMissile{false};

    // Is this contact firing guns at us? (set by SensorFusion)
    bool isFiring{false};

    // Threat score (set by SensorFusion — higher = more dangerous)
    double threatScore{0.0};

    // --- Convenience ---
    bool detectedBy(SensorType s) const {
        return (sensorMask & (1u << static_cast<uint8_t>(s))) != 0;
    }
    void addSensor(SensorType s) {
        sensorMask |= (1u << static_cast<uint8_t>(s));
    }
};

// SensorPicture — the complete filtered view the AI reads each frame.
struct SensorPicture {
    std::vector<SensorContact> contacts;

    // The highest-threat contact (pre-computed by SensorFusion for convenience)
    const SensorContact* highestThreat{nullptr};

    // The best target to engage (pre-computed by SensorFusion)
    const SensorContact* bestTarget{nullptr};

    // Any incoming missile? (pre-computed)
    const SensorContact* incomingMissile{nullptr};

    // Any guns threat? (pre-computed)
    const SensorContact* gunsThreat{nullptr};

    // Spike: is someone locking us up? (radar emission toward us)
    bool spiked{false};
    double spikeHeading{0.0};  // heading to the spike source (radians)

    void clear() {
        contacts.clear();
        highestThreat = nullptr;
        bestTarget = nullptr;
        incomingMissile = nullptr;
        gunsThreat = nullptr;
        spiked = false;
        spikeHeading = 0.0;
    }

    // Find a contact by entity ID
    const SensorContact* findById(EntityId id) const {
        for (const auto& c : contacts) {
            if (c.entityId == id) return &c;
        }
        return nullptr;
    }

    // Find a contact by type
    const SensorContact* findByType(ContactType t) const {
        for (const auto& c : contacts) {
            if (c.type == t) return &c;
        }
        return nullptr;
    }
};

} // namespace digi
} // namespace f4flight
