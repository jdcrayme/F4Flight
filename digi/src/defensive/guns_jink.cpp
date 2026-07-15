// f4flight - digi/defensive/guns_jink.cpp
//
// Guns defense implementation.
//
// Direct port of FreeFalcon gunsjink.cpp (331 LOC), simplified to use the
// DigiEntity model.
//
// Key simplifications vs. FreeFalcon:
//   - No team stance check (host decides hostility by setting gunsThreat)
//   - No SMS / AG jettison (no SMS model yet)
//   - Uses RelativeGeometry computed on-demand instead of cached localData
//   - jinkTime as int (-1/0/>0) preserved from FreeFalcon for fidelity
//   - Pull duration uses real time (dt accumulation) instead of frame count
//     to be frame-rate independent

#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// GunsJinkCheck
// ===========================================================================
bool GunsJinkCheck(const DigiState& digi, const DigiEntity& self) {
    const DigiEntity* threat = digi.gunsJink.gunsThreat;
    if (!threat || threat->isDead) return false;

    // Compute relative geometry
    const RelativeGeometry rg = computeRelativeGeometry(self, *threat);

    // Range checks
    // FreeFalcon gunsjink.cpp:42-44
    if (rg.range <= 0.0 || rg.range >= kGunRangeMax) return false;
    if (rg.range >= kInitGunVel) return false;  // 4500 ft

    // Threat must be firing (simplified: FF also checks team stance == War)
    // FreeFalcon gunsjink.cpp:36-38
    if (!threat->isFiring) return false;

    // azFrom check: within ±15° (in-plane) or predicted within ±5°
    // FreeFalcon gunsjink.cpp:53-66
    const double azFromThresh = 15.0 * DTR;
    bool twoSeconds = false;

    if (rg.azFrom > -azFromThresh && rg.azFrom < azFromThresh) {
        twoSeconds = true;
    }
    // The predicted-within-±5° branches (gunsjink.cpp:57-66) require
    // azFromdot which we don't track. Skip them — the ±15° window is
    // the primary trigger.

    if (!twoSeconds) return false;

    // elFrom check: within -10° to +4°
    // FreeFalcon gunsjink.cpp:72-75
    const double elFromLow = -10.0 * DTR;
    const double elFromHigh = 4.0 * DTR;
    if (rg.elFrom < elFromLow || rg.elFrom > elFromHigh) return false;

    // tgt_time <= att_time check
    // FreeFalcon gunsjink.cpp:101-120
    // tgt_time = ataFrom / ataFromdot  (time for target to point at us)
    // att_time = ata / atadot          (time for us to point at target)
    // If either is within ±13°, the respective time is 0.
    // We don't track ataFromdot/atadot, so we use a simplified check:
    // if the target is roughly pointed at us (|ataFrom| < 30°) and we're
    // not yet pointed at the target (|ata| > 30°), the target can fire first.
    const double ataFromBand = 13.0 * DTR;
    const double ataBand = 13.0 * DTR;

    double tgtTime, attTime;
    if (rg.ataFrom > -ataFromBand && rg.ataFrom < ataFromBand) {
        tgtTime = 0.0;  // target already pointed at us
    } else {
        // Without ataFromdot, assume the target will be pointed at us soon
        tgtTime = 0.5;  // placeholder: target gets there first
    }

    if (rg.ata > -ataBand && rg.ata < ataBand) {
        attTime = 0.0;  // we're already pointed at target
    } else {
        attTime = 1.0;  // placeholder: we take longer
    }

    // FreeFalcon gunsjink.cpp:126: trigger jink if tgt_time <= att_time
    return (tgtTime <= attTime);
}

// ===========================================================================
// GunsJink
// ===========================================================================
bool GunsJink(DigiState& digi, const DigiEntity& self,
              const AircraftState& as,
              const FlightControlSystem& fcs, FcsState& fcsState,
              double dt) {
    const DigiEntity* threat = digi.gunsJink.gunsThreat;

    // Bail if no target or target is exploding or range > 4000 ft
    // FreeFalcon gunsjink.cpp:157-163
    if (!threat || threat->isDead) {
        digi.gunsJink.jinkTime = -1;
        return false;
    }

    const RelativeGeometry rg = computeRelativeGeometry(self, *threat);
    if (rg.range > kGunJinkExitRange) {
        digi.gunsJink.jinkTime = -1;
        return false;
    }

    // Don't jink if we need to avoid the ground
    // FreeFalcon gunsjink.cpp:166-167
    if (digi.groundAvoid.groundAvoidNeeded) return true;  // stay in mode but don't maneuver

    // Energy management: hold corner speed
    // FreeFalcon gunsjink.cpp:172
    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, false,
                                  digi, as, 200.0, 800.0, dt, 100.0);

    // --- Phase -1: initialize, pick roll angle ---
    // FreeFalcon gunsjink.cpp:182-270
    if (digi.gunsJink.jinkTime == -1) {
        // (AG jettison skipped — no SMS model)

        // Find target aspect: 180° - ata
        // FreeFalcon gunsjink.cpp:177, 198
        const double aspect = 180.0 * DTR - rg.ata;

        // Reset max roll (will be overridden below)
        // FreeFalcon gunsjink.cpp:192: ResetMaxRoll()
        // We don't have ResetMaxRoll; set to default 80°
        fcsState.maxRoll = 80.0;

        if (aspect >= 90.0 * DTR) {
            // Aspect >= 90°: put plane of wings on attacker
            // FreeFalcon gunsjink.cpp:198-206
            double rollOffset;
            if (rg.droll >= 0.0) {
                rollOffset = rg.droll - 90.0 * DTR;
            } else {
                rollOffset = rg.droll + 90.0 * DTR;
            }
            digi.gunsJink.newRoll = self.roll + rollOffset;
        } else {
            // Aspect < 90°: roll ±70°
            // FreeFalcon gunsjink.cpp:211-253
            // Simplified: skip the in-plane crossing special case
            // (gunsjink.cpp:214-233) — it requires comparing yaw/pitch/roll
            // differences and is a minor variation.
            if (rg.droll > 0.0) {
                digi.gunsJink.newRoll = self.roll - kJinkRollAngle * DTR;
            } else {
                digi.gunsJink.newRoll = self.roll + kJinkRollAngle * DTR;
            }

            // Roll down if speed <= 80% of corner speed
            // FreeFalcon gunsjink.cpp:246-252
            if (as.vcas <= 0.8 * digi.config.cornerSpeed) {
                if (digi.gunsJink.newRoll >= 0.0 && digi.gunsJink.newRoll <= 45.0 * DTR) {
                    digi.gunsJink.newRoll += 30.0 * DTR;
                } else if (digi.gunsJink.newRoll <= 0.0 && digi.gunsJink.newRoll >= -45.0 * DTR) {
                    digi.gunsJink.newRoll -= 30.0 * DTR;
                }
            }
        }

        // Roll angle corrections: wrap to [-180°, 180°]
        // FreeFalcon gunsjink.cpp:258-261
        while (digi.gunsJink.newRoll >  180.0 * DTR) digi.gunsJink.newRoll -= 360.0 * DTR;
        while (digi.gunsJink.newRoll < -180.0 * DTR) digi.gunsJink.newRoll += 360.0 * DTR;

        // Clamp to max roll
        // FreeFalcon gunsjink.cpp:264-267
        digi.gunsJink.newRoll = std::max(-digi.config.maxRoll * DTR,
                                 std::min(digi.gunsJink.newRoll, digi.config.maxRoll * DTR));

        digi.gunsJink.jinkTime = 0;
    }

    // Allow unlimited rolling (set maxRoll to 190°)
    // FreeFalcon gunsjink.cpp:273-274
    fcsState.maxRoll = 190.0;

    // --- Phase 0: roll to target bank angle ---
    // FreeFalcon gunsjink.cpp:279-305
    if (digi.gunsJink.jinkTime == 0) {
        // Initial negative G to break lift — roll the lift vector OFF the
        // current flight path so the aircraft starts to slice/drop.
        //
        // BUG FIX: the previous code immediately overwrote this with a
        // positive-G pull on line 211, so the negative-G "break lift" never
        // took effect. Now we sequence them: negative G during the roll,
        // positive G only after the roll is captured (phase 1).
        ManeuverPrimitives::SetPstick(-2.0, digi.config.maxGs,
                                       CommandType::GCommand, digi, as);

        // Roll error
        double eroll = digi.gunsJink.newRoll - self.roll;
        // Wrap to [-PI, PI]
        while (eroll >  PI) eroll -= 2.0 * PI;
        while (eroll < -PI) eroll += 2.0 * PI;

        // Roll to target — SetRstick takes roll error in degrees
        // FreeFalcon gunsjink.cpp:291: SetRstick(eroll * RTD * 4.0F)
        ManeuverPrimitives::SetRstick(eroll * RTD * 4.0, digi, fcs, fcsState);
        fcsState.maxRollDelta = std::fabs(eroll * RTD);

        // Stop rolling and pull when within 5°
        // FreeFalcon gunsjink.cpp:300-304
        if (std::fabs(eroll) < kJinkRollTolerance * DTR) {
            digi.gunsJink.jinkTime = 1;
            digi.gunsJink.jinkTimer = 0.0;
            ManeuverPrimitives::SetRstick(0.5, digi, fcs, fcsState);
        }
    }

    // --- Phase 1: pull max G for ~2 seconds ---
    // FreeFalcon gunsjink.cpp:310-325
    if (digi.gunsJink.jinkTime > 0 || digi.groundAvoid.groundAvoidNeeded) {
        // BUG FIX: the previous code used `std::max(0.8 * digi.config.maxGs, digi.config.maxGs)`
        // which is always `digi.config.maxGs` (because x > 0.8*x for x > 0). The intent
        // (per FF gunsjink.cpp:310) was to pull maxGs. Just use maxGs directly.
        const double maxPull = digi.config.maxGs;
        ManeuverPrimitives::SetPstick(maxPull, digi.config.maxGs,
                                       CommandType::GCommand, digi, as);

        digi.gunsJink.jinkTimer += dt;
        if (digi.gunsJink.jinkTimer > kJinkPullDuration) {
            // Done — reset and exit
            // FreeFalcon gunsjink.cpp:317-318
            fcsState.maxRoll = 80.0;  // ResetMaxRoll equivalent
            digi.gunsJink.jinkTime = -1;
            digi.gunsJink.jinkTimer = 0.0;
            return false;  // exit GunsJink mode
        }
    }

    return true;  // stay in GunsJink mode
}

} // namespace digi
} // namespace f4flight
