// f4flight - digi/sensors/rwr_sensor.cpp
//
// RWRSensor implementation.

#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

void RWRSensor::update(const DigiEntity& self, const TruthState& truth,
                        const SkillParameters& skill, double /*dt*/,
                        std::vector<SensorContact>& out) {
    (void)skill;  // RWR is non-skilled (always detects emissions in range)
    if (!config_.enabled) return;

    for (std::size_t i = 0; i < truth.size(); ++i) {
        const auto& entity = truth.entities[i];
        if (entity.isDead) continue;

        const RelativeGeometry rg = computeRelativeGeometry(self, entity);
        if (rg.range > config_.maxRangeFt) continue;

        // RWR detects radar emissions. Only entities that actually emit
        // radar should be detected:
        //   - Missiles with radar seekers (active radar homing)
        //   - Aircraft with active fire-control radar (isRadarEmitting=true)
        //
        // BUG FIX: the previous code treated ALL aircraft (seekerType=None)
        // within 100 NM as emitting radar. This flooded the RWR picture with
        // false emissions (tankers, transports, friendly aircraft, even
        // gliders). Now we use the DigiEntity.isRadarEmitting flag (default
        // true) so hosts that model radar modes can mark non-emitting
        // aircraft. Missiles with radar seekers always emit while guiding.
        bool isEmitting = false;
        if (entity.seekerType == DigiEntity::SeekerType::Radar) {
            // Missile with radar seeker — always emits while guiding
            isEmitting = true;
        } else if (entity.seekerType == DigiEntity::SeekerType::None &&
                   entity.isRadarEmitting) {
            // Aircraft with active fire-control radar
            isEmitting = true;
        }
        // Note: IR missiles (seekerType=IR) and aircraft with radar off are
        // NOT detected by RWR. This is correct — RWR only sees radar emissions.

        if (!isEmitting) continue;

        // Confidence: signal strength decreases with range
        const double rangeFactor = 1.0 - (rg.range / config_.maxRangeFt) * 0.7;
        double confidence = std::max(rangeFactor, 0.1);

        if (confidence < config_.minConfidence) continue;

        SensorContact c = makeContact(truth.ids[i], entity, confidence);
        // makeContact already calls c.addSensor(type()) — no need to repeat.

        // RWR can't determine range precisely, but we include position
        // for simplicity (in a full impl, RWR only gives bearing)
        c.quality = ContactQuality::Detected;

        // Identify missiles vs SAMs
        if (entity.seekerType == DigiEntity::SeekerType::Radar) {
            c.type = ContactType::Missile;
            c.isMissile = true;
        } else if (entity.z >= -10.0) {
            c.type = ContactType::SAM;
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
