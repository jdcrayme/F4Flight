// f4flight - digi/sensors/rwr_sensor.cpp
//
// RWRSensor implementation.

#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

void RWRSensor::update(const DigiEntity& self, const TruthState& truth,
                        const SkillParameters& skill, double /*dt*/,
                        std::vector<SensorContact>& out) {
    if (!config_.enabled) return;

    for (std::size_t i = 0; i < truth.size(); ++i) {
        const auto& entity = truth.entities[i];
        if (entity.isDead) continue;

        const RelativeGeometry rg = computeRelativeGeometry(self, entity);
        if (rg.range > config_.maxRangeFt) continue;

        // RWR detects radar emissions. In our model, any entity with a
        // radar seeker type is "emitting." Aircraft (seekerType=None) don't
        // emit unless they have an active radar — we approximate this by
        // detecting all aircraft at shorter range (their fire-control radar).
        bool isEmitting = false;
        if (entity.seekerType == DigiEntity::SeekerType::Radar) {
            // Missile with radar seeker — always emits
            isEmitting = true;
        } else if (entity.seekerType == DigiEntity::SeekerType::None) {
            // Aircraft — emit at shorter range (we assume active radar)
            if (rg.range < 100.0 * 6076.0) {  // 100 NM
                isEmitting = true;
            }
        }

        if (!isEmitting) continue;

        // Confidence: signal strength decreases with range
        const double rangeFactor = 1.0 - (rg.range / config_.maxRangeFt) * 0.7;
        double confidence = std::max(rangeFactor, 0.1);

        if (confidence < config_.minConfidence) continue;

        SensorContact c = makeContact(truth.ids[i], entity, confidence);
        c.addSensor(SensorType::RWR);

        // RWR can't determine range precisely, but we include position
        // for simplicity (in a full impl, RWR only gives bearing)
        c.quality = ContactQuality::Detected;

        // Identify missiles
        if (entity.seekerType == DigiEntity::SeekerType::Radar) {
            c.type = ContactType::Missile;
            c.isMissile = true;
        }

        // Check for spike (fire-control radar pointing at us)
        if (std::fabs(rg.ataFrom) < 15.0 * DTR && rg.range < 50.0 * 6076.0) {
            c.isThreat = true;
        }

        out.push_back(c);
    }
}

} // namespace digi
} // namespace f4flight
