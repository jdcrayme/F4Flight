// f4flight - digi/sensors/rwr_sensor.h
//
// RWRSensor — Radar Warning Receiver.
//
// Detects radar emissions directed at own aircraft. The RWR doesn't detect
// the emitting aircraft directly — it detects the radar signal. This means:
//   - Only detects aircraft/SAMs with active radars
//   - Detects emissions from any direction (RWR is omnidirectional)
//   - Cannot determine range (only bearing + signal strength)
//   - Can detect missile launches (when a fire-control radar locks on)
//
// In our simplified model:
//   - Any entity with seekerType == Radar is "emitting"
//   - Detection range is very long (RWR picks up radar at long range)
//   - Confidence is based on signal strength (range-dependent)
//   - Identifies missile lock (spike) when an entity is pointing at us

#pragma once

#include "f4flight/digi/sensors/sensor.h"

namespace f4flight {
namespace digi {

class RWRSensor : public Sensor {
public:
    RWRSensor() {
        // RWR: very long range, omnidirectional
        SensorConfig cfg;
        cfg.maxRangeFt = 250.0 * 6076.0;  // 250 NM (RWR detects at long range)
        cfg.maxAzRad = PI;                 // omnidirectional
        cfg.maxElRad = PI;
        cfg.minConfidence = 0.1;
        cfg.enabled = true;
        config_ = cfg;
    }

    SensorType type() const override { return SensorType::RWR; }

    void update(const DigiEntity& self, const TruthState& truth,
                const SkillParameters& skill, double dt,
                std::vector<SensorContact>& out) override;
};

} // namespace digi
} // namespace f4flight
