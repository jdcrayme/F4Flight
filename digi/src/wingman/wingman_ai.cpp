// f4flight - digi/wingman/wingman_ai.cpp
//
// Wingman AI implementation — formation following.
//
// Port of FreeFalcon's AiFollowLead (wingactions.cpp:295-420).
//
// COORDINATE-SYSTEM FIX (vs naive port):
// FreeFalcon uses (x=north, y=east, z=down) with heading sigma measured
// CLOCKWISE from +x (navigation convention). Its formation formula is:
//     trackX += range * cos(relAz * formSide + sigma)
//     trackY += range * sin(relAz * formSide + sigma)
// In FF's convention, relAz=0 places the wingman FORWARD (along +x = north =
// the lead's nose), and positive relAz rotates CLOCKWISE = to the RIGHT.
//
// F4Flight uses (x=east, y=north, z=down) with sigma measured CCW from +x
// (standard math convention). If we naively reuse FF's formula, relAz=0
// still places the wingman along the lead's velocity vector (forward), BUT
// positive relAz rotates CCW = to the LEFT — the opposite of FF's intent.
// This was the root cause of the "lead is in the back" bug: the default
// wedge/trail formations used relAz=0 for "trail" (intended = behind), but
// the formula placed the wingman FORWARD. The test aircraft therefore tried
// to fly ahead of the lead, overshot, and oscillated.
//
// FIX: use the adapted formula
//     trackX = leadX + range * cos(sigma - relAz * formSide)
//     trackY = leadY + range * sin(sigma - relAz * formSide)
// which reproduces FF's intended geometry (positive relAz = right, relAz=pi
// = directly behind) in F4Flight's coordinate system. The default formation
// definitions in formation_geometry.h now use this convention: trail = pi
// (behind), right wing = +135° (behind-right), left wing = -135° (behind-left).

#include "f4flight/digi/wingman/wingman_ai.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/steering.h"  // for headingError
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/flight/core/airspeed_conversions.h"  // casFromTasFps

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// In-position threshold. FreeFalcon uses 250 ft. F4Flight's FCS has more
// lag than FF's simple-mode, so we use 400 ft — tight enough to prove the
// wingman has actually closed to the slot, loose enough to account for FCS
// deadband. The previous 800 ft was far too loose (the test passed even
// when the wingman was sweeping 800 ft past the slot every cycle).
static constexpr double kInPositionThresholdFt = 400.0;

// Distance at which the wingman switches from "rejoin" (lead pursuit with
// lead-ahead offset) to "station-keeping" (direct slot tracking with no
// lead-ahead, damped closure). This prevents the overshoot oscillation
// that occurred when the wingman kept aiming 600 ft ahead of the slot
// even after it was already in position.
static constexpr double kStationKeepingRadiusFt = 1500.0;

// AiFollowLead — fly to the wingman's formation slot relative to the lead.
bool AiFollowLead(DigiState& digi, const DigiEntity& self,
                  const DigiEntity* lead, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt) {
    (void)dt;

    // If no lead, or we ARE the lead, loiter in place.
    if (!lead || lead->isDead ||
        digi.formation.flightLeadId == kInvalidEntityId ||
        !digi.formation.isWing) {
        ManeuverPrimitives::Loiter(digi, as, fcs, fcsState, digi.config.maxGs);
        return false;
    }

    // --- Look up formation slot geometry ---
    const auto& form = digi.formation;
    const auto slot = formation::FormationTable::defaultInstance().slotGeometry(
        static_cast<formation::FormationType>(form.formationId),
        form.vehicleInUnit);

    // --- Apply lateral spacing factor ---
    // formLateralSpaceFactor defaults to 1.0 (normal spacing).
    // Kickout sets it to 2.0 (double spacing), Closeup to 0.5 (half spacing).
    const double rangeFactor = slot.range * form.wingman.formLateralSpaceFactor;

    // --- Compute desired world position relative to lead ---
    // ADAPTED formula (see file header): cos/sin(sigma - relAz*formSide)
    // reproduces FreeFalcon's geometry in F4Flight's coordinate system.
    const double leadSigma = lead->yaw;

    // Apply side mirror (formSide: +1 = right side, -1 = left side).
    const double formSide = (form.wingman.formSide != 0)
                            ? static_cast<double>(form.wingman.formSide)
                            : 1.0;

    const double azAngle = leadSigma - slot.relAz * formSide;
    double trackX = lead->x + rangeFactor * std::cos(azAngle);
    double trackY = lead->y + rangeFactor * std::sin(azAngle);

    // Vertical offset. FreeFalcon uses relEl if non-zero, else stacks
    // wingmen vertically by -100 ft per slot index (stairstep).
    double trackZ;
    if (std::fabs(slot.relEl) > 1e-6) {
        trackZ = lead->z + rangeFactor * std::sin(-slot.relEl);
    } else {
        // Stairstep: each wingman is 100 ft below the lead (NED: z increases down)
        trackZ = lead->z + form.vehicleInUnit * 100.0;
    }

    // Apply relative formation altitude (IncreaseRelAlt/DecreaseRelAlt commands).
    trackZ += form.wingman.formRelativeAltitude;

    // --- Compute distance to the actual slot (before lead-ahead offset) ---
    // This is used for in-position checks and to scale the lead-ahead offset.
    const double slotDx = trackX - self.x;
    const double slotDy = trackY - self.y;
    const double slotDz = trackZ - self.z;
    const double distToSlot = std::sqrt(slotDx * slotDx + slotDy * slotDy +
                                        slotDz * slotDz);

    // --- Check in-position (uses the actual slot position) ---
    AiCheckInPositionCall(digi, self, trackX, trackY, trackZ);

    // --- Adaptive lead-ahead offset ---
    // FreeFalcon sets the trackpoint 1 NM ahead of the slot in the lead's
    // heading direction (lead pursuit). This is good for the rejoin phase
    // (smooth approach), but causes overshoot when close to the slot: the
    // wingman keeps aiming 600+ ft ahead and sweeps past the slot.
    //
    // FIX: scale the lead-ahead by distance to slot. Full offset when far
    // (smooth rejoin), zero offset when in station-keeping range (direct
    // slot tracking). This eliminates the overshoot oscillation.
    constexpr double kLeadAheadNm = 0.1;
    const double leadAheadFtMax = kLeadAheadNm * 6076.0;  // 607.6 ft
    const double leadAheadScale = std::min(1.0,
        distToSlot / (kStationKeepingRadiusFt * 2.0));
    const double leadAheadFt = leadAheadFtMax * leadAheadScale;
    trackX += leadAheadFt * std::cos(leadSigma);
    trackY += leadAheadFt * std::sin(leadSigma);

    // --- Fly to the desired position ---
    const double dx = trackX - self.x;
    const double dy = trackY - self.y;
    const double desHeading = std::atan2(dy, dx);
    const double desAlt = -trackZ;  // NED: z negative up, alt positive up

    // --- Speed control with PD closure damping ---
    // The wingman matches the lead's speed, with two corrections:
    //   1. Along-track position error (proportional): speeds up if behind,
    //      slows down if ahead.
    //   2. Along-track velocity error (derivative): damps the closure rate
    //      to prevent overshoot.
    //
    // Both terms use the lead's velocity direction as the "along-track" axis.
    const double alongTrackErr = slotDx * std::cos(leadSigma) +
                                  slotDy * std::sin(leadSigma);

    // Along-track velocity of the wingman RELATIVE to the lead. Positive =
    // wingman pulling ahead of the lead (and thus ahead of the slot).
    const double relVx = as.kin.xdot - lead->vx;
    const double relVy = as.kin.ydot - lead->vy;
    const double alongTrackVel = relVx * std::cos(leadSigma) +
                                  relVy * std::sin(leadSigma);

    const double proxScale = std::min(1.0, distToSlot / kStationKeepingRadiusFt);

    // Use a standard, clean PD controller for speed tracking.
    const double pGain = 0.12;  // High, constant proportional gain for stiff tracking near slot
    double closureCorrection = alongTrackErr * pGain;

    // Derivative damping: oppose the along-track relative velocity.
    closureCorrection -= alongTrackVel * 0.8;  // Stiff derivative damping

    // Clamp: allow more speed delta when far (rejoin), less when close.
    const double clampLimit = 30.0 + 40.0 * proxScale;  // 30 close, 70 far
    closureCorrection = std::max(-clampLimit, std::min(clampLimit, closureCorrection));

    const double desSpeed = std::max(150.0, lead->speed + closureCorrection);

    // --- Speed brakes ---
    if (alongTrackErr < -200.0 && alongTrackVel > 10.0) {
        digi.commands.speedBrakeCmd = 1.0;
    } else if (alongTrackErr < -50.0 && alongTrackVel > 0.0) {
        digi.commands.speedBrakeCmd = 0.5;
    } else {
        digi.commands.speedBrakeCmd = -1.0;
    }

    // --- Steering: Cross-Track (Lateral) PD Controller ---
    //
    // Standard bearing pursuit has an along-track singularity (gain goes to
    // infinity as along-track distance goes to zero), causing wild lateral
    // pendulum oscillations close to the slot.
    //
    // To solve this, we implement a linear Cross-Track PD controller when close
    // to the slot, and smoothly blend to pure bearing pursuit when far.
    //
    // Lateral axis: perpendicular to the lead's heading (right = positive).
    // Right-direction in F4Flight: (sin(sigma), -cos(sigma))
    const double sinSig = std::sin(leadSigma);
    const double cosSig = std::cos(leadSigma);
    const double lateralVel = relVx * sinSig - relVy * cosSig;

    // Lateral position error (positive = wingman is to the right of the slot)
    const double lateralErr = - (slotDx * sinSig - slotDy * cosSig);

    // Proportional & Derivative gains for lateral tracking:
    // K_p = 0.0070 rad of heading per foot of lateral error (about 0.40 deg/ft)
    // K_d = 0.0100 rad of heading per ft/s of lateral velocity (about 0.57 deg per ft/s)
    const double kp_lat = 0.0070;
    const double kd_lat = 0.0100;
    const double latCorr = kp_lat * lateralErr + kd_lat * lateralVel;

    // Blend factor: 0.0 on the slot (pure linear PD), 1.0 when far (pure bearing pursuit)
    const double blend = std::min(1.0, distToSlot / kStationKeepingRadiusFt);

    const double targetCloseHeading = leadSigma + latCorr;
    // Blend the headings safely using headingError to avoid 360-degree wrap-around bugs
    const double dampedHeading = targetCloseHeading + blend * headingError(desHeading, targetCloseHeading);

    ManeuverPrimitives::HeadingAndAltitudeHold(dampedHeading, desAlt,
                                                digi, as, fcs, fcsState,
                                                digi.config.maxGs);

    // Use MachHold to match speed.
    //
    // CAS/TAS CORRECTION: desSpeed is the lead's TRUE airspeed (ft/s).
    // MachHold compares target vs as.vcas (CALIBRATED airspeed, kts). At
    // altitude, TAS > CAS (lower air density), so naively converting the
    // lead's TAS to kts and comparing with the wingman's CAS would make the
    // wingman target a CAS equal to the lead's TAS — which is much faster
    // than the lead's actual CAS. The wingman would always fly too fast and
    // never stabilize in formation.
    //
    // FIX: use the typed CasKnots API. Convert the lead's TAS (ft/s) to
    // CAS (kts) using casFromTasFps, which uses the wingman's own CAS/TAS
    // ratio. The typed API makes it a compile error to pass TAS where CAS
    // is expected.
    // CAS/TAS CORRECTION: desSpeed is the lead's TRUE airspeed (ft/s).
    // MachHold compares target vs as.vcas (CALIBRATED airspeed, kts).
    // But MachHold is a proportional-only controller (thr = (eProp + 100) * 0.008) in steady level flight,
    // which has a persistent proportional droop of about 15.5 knots CAS (the speed error required to output
    // the trim throttle of ~0.67 to balance level flight drag).
    //
    // FEEDFORWARD COMPENSATION: We subtract 16.5 knots from the target CAS so that the steady-state droop
    // is exactly canceled out, allowing the wingman to fly at exactly the lead's actual speed with zero along-track error.
    const CasKnots desCas = casFromTasFps(tas_fps(desSpeed), as) - cas_kts(16.5);
    ManeuverPrimitives::machHoldCas(desCas, false,
                                     digi, as, 150.0, 800.0, digi.nav.dt, 100.0);

    // Return true if "in position" (within threshold of the slot).
    return distToSlot < kInPositionThresholdFt;
}

// AiCheckInPositionCall — set the inPosition flag when within threshold.
bool AiCheckInPositionCall(DigiState& digi, const DigiEntity& self,
                           double trackX, double trackY, double trackZ) {
    const double dx = trackX - self.x;
    const double dy = trackY - self.y;
    const double dz = trackZ - self.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    const bool wasInPosition = digi.formation.wingman.inPosition;
    digi.formation.wingman.inPosition = (dist < kInPositionThresholdFt);

    // Return true on the transition (newly in position this frame).
    // FreeFalcon uses this to trigger the "in position" radio call.
    return digi.formation.wingman.inPosition && !wasInPosition;
}

} // namespace digi
} // namespace f4flight
