// f4flight - digi/sensors/visual_sensor.h
//
// VisualSensor — eyeball / visual detection.
//
// The visual sensor detects other aircraft by sight. It's:
//   - Short range (typically < 10 NM, less at night or bad weather)
//   - Skill-gated (aces have sharper eyes)
//   - Best in the forward hemisphere (can't see behind you)
//   - Provides full attitude data (yaw/pitch/roll) — visual can see attitude
//   - Can identify aircraft type at close range
//   - Can detect gun fire (muzzle flash)
//
// This is the sensor that triggers GunsJink (visual detection of a firing bandit)
// and provides the best target tracking for WVR engagements.

#pragma once

#include "f4flight/digi/sensors/sensor.h"

namespace f4flight {
namespace digi {

class VisualSensor : public Sensor {
public:
    VisualSensor() {
        // Visual: 8 NM range, ±90° azimuth (forward hemisphere), ±45° el
        SensorConfig cfg;
        cfg.maxRangeFt = 8.0 * 6076.0;   // 8 NM
        cfg.maxAzRad = 90.0 * DTR;        // forward hemisphere
        cfg.maxElRad = 45.0 * DTR;
        cfg.minConfidence = 0.2;
        cfg.enabled = true;
        config_ = cfg;
    }

    SensorType type() const override { return SensorType::Visual; }

    void update(const DigiEntity& self, const TruthState& truth,
                const SkillParameters& skill, double dt,
                std::vector<SensorContact>& out) override;
};

} // namespace digi
} // namespace f4flight
