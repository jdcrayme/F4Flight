// f4flight - digi/sensors/visual_sensor.cpp
//
// VisualSensor implementation.

#include "f4flight/digi/sensors/visual_sensor.h"
#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

void VisualSensor::update(const DigiEntity& self, const TruthState& truth,
                           const SkillParameters& skill, double /*dt*/,
                           std::vector<SensorContact>& out) {
    if (!config_.enabled) return;

    // Effective range scales with skill (aces have sharper eyes)
    const double skillRangeMult = 0.7 + 0.15 * static_cast<int>(skill.level);
    const double effRange = config_.maxRangeFt * skillRangeMult;

    for (std::size_t i = 0; i < truth.size(); ++i) {
        const auto& entity = truth.entities[i];
        if (entity.isDead) continue;

        const RelativeGeometry rg = computeRelativeGeometry(self, entity);
        if (rg.range > effRange) continue;
        if (std::fabs(rg.az) > config_.maxAzRad) continue;
        if (std::fabs(rg.el) > config_.maxElRad) continue;

        // Visual detection: high at close range, drops sharply with distance
        // and is harder for small targets (missiles)
        const double rangeFactor = 1.0 - (rg.range / effRange) * 0.8;
        double sizeFactor = 1.0;
        if (entity.seekerType != DigiEntity::SeekerType::None) {
            sizeFactor = 0.5;  // missiles are small
        }

        double confidence = rangeFactor * sizeFactor * skillRangeMult;
        confidence = std::max(confidence, 0.0);

        if (confidence < config_.minConfidence) continue;

        SensorContact c = makeContact(truth.ids[i], entity, confidence);
        c.addSensor(SensorType::Visual);

        // Visual provides full attitude data
        c.yaw = entity.yaw;
        c.pitch = entity.pitch;
        c.roll = entity.roll;

        // Visual identifies type at close range
        if (rg.range < 3.0 * 6076.0 && confidence > 0.4) {
            c.quality = ContactQuality::Identified;
            if (entity.seekerType == DigiEntity::SeekerType::None) {
                c.type = ContactType::Fighter;  // simplified
            } else {
                c.type = ContactType::Missile;
                c.isMissile = true;
            }
        } else {
            c.quality = ContactQuality::Tracked;
        }

        // Visual can detect gun fire (muzzle flash) at close range
        if (truth.firing.size() > i && truth.firing[i] && rg.range < 6000.0) {
            c.isFiring = true;
            c.isThreat = true;
        }

        out.push_back(c);
    }
}

} // namespace digi
} // namespace f4flight
