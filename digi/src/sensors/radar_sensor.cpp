// f4flight - digi/sensors/radar_sensor.cpp
//
// RadarSensor implementation.

#include "f4flight/digi/sensors/radar_sensor.h"
#include "f4flight/flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

void RadarSensor::update(const DigiEntity& self, const TruthState& truth,
                          const SkillParameters& skill, double /*dt*/,
                          std::vector<SensorContact>& out) {
    if (!config_.enabled) return;

    // Effective range scales with skill (aces get longer detection)
    const double skillRangeMult = 0.8 + 0.1 * static_cast<int>(skill.level);
    const double effRange = config_.maxRangeFt * skillRangeMult;

    for (std::size_t i = 0; i < truth.size(); ++i) {
        const auto& entity = truth.entities[i];
        if (entity.isDead) continue;

        double range, az, el;
        const RelativeGeometry rg = computeRelativeGeometry(self, entity);
        range = rg.range;
        az = rg.az;
        el = rg.el;

        if (range > effRange) continue;
        if (std::fabs(az) > config_.maxAzRad) continue;
        if (std::fabs(el) > config_.maxElRad) continue;

        // Detection probability: high at close range, decreases with range
        // and when target is beam-on (small RCS)
        const double rangeFactor = 1.0 - (range / effRange) * 0.5;
        const double rcsFactor = 1.0 - std::fabs(std::sin(rg.ataFrom)) * 0.3;
        double confidence = rangeFactor * rcsFactor * skillRangeMult;
        confidence = std::max(confidence, 0.0);

        if (confidence < config_.minConfidence) continue;

        SensorContact c = makeContact(truth.ids[i], entity, confidence);
        // makeContact already calls c.addSensor(type()) — no need to repeat.

        // Identify type at close range
        if (range < 20.0 * 6076.0 && confidence > 0.5) {
            c.quality = ContactQuality::Identified;
            // (In a full impl, we'd classify by RCS, NCTR, etc.)
            // For now, mark as Fighter if it's an aircraft (not a missile)
            if (entity.seekerType == DigiEntity::SeekerType::None) {
                c.type = ContactType::Fighter;
            }
        } else {
            c.quality = ContactQuality::Tracked;
        }

        out.push_back(c);
    }
}

} // namespace digi
} // namespace f4flight
