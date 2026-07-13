// f4flight - digi/sensors/radar_sensor.h
//
// RadarSensor — detects aircraft via radar.
//
// Detection model:
//   - Range-limited (maxRangeFt from config)
//   - Field of regard: ±60° azimuth, ±60° elevation (typical fighter radar)
//   - Detection probability decreases with range and aspect (beam-on is harder)
//   - Skill-gated: higher skill = longer effective range
//   - Identifies contact type (fighter/bomber/etc.) at close range
//
// Comparison to FreeFalcon:
//   FreeFalcon's RadarClass has modes (RWS/TWS/SAM/STT) and gimbal limits.
//   We simplify to a single detection model — mode management is a future
//   enhancement.

#pragma once

#include "f4flight/digi/sensors/sensor.h"

namespace f4flight {
namespace digi {

class RadarSensor : public Sensor {
public:
    RadarSensor() {
        // Default fighter radar: 80 NM range, ±60° az/el
        SensorConfig cfg;
        cfg.maxRangeFt = 80.0 * 6076.0;  // 80 NM
        cfg.maxAzRad = 60.0 * DTR;
        cfg.maxElRad = 60.0 * DTR;
        cfg.minConfidence = 0.1;
        cfg.enabled = true;
        config_ = cfg;
    }

    SensorType type() const override { return SensorType::Radar; }

    void update(const DigiEntity& self, const TruthState& truth,
                const SkillParameters& skill, double dt,
                std::vector<SensorContact>& out) override;
};

} // namespace digi
} // namespace f4flight
