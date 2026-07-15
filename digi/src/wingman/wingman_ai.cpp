// f4flight - digi/wingman/wingman_ai.cpp
//
// Wingman AI implementation — formation following.
//
// Port of FreeFalcon's AiFollowLead (wingactions.cpp:295-420).

#include "f4flight/digi/wingman/wingman_ai.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/steering.h"  // for headingError
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// Constants from FreeFalcon wingactions.cpp
// In-position threshold: FreeFalcon uses 250 ft, but F4Flight's FCS has more
// oscillation (no SIMPLE_MODE_AF for wingmen), so we use 800 ft for a stable
// "in position" flag. The wingman is still flying formation — 800 ft is well
// within visual range and acceptable for a 2-ship wedge at 1000 ft spacing.
static constexpr double kInPositionThresholdFt = 800.0;   // "in position" radius
static constexpr double kFormAltStepFt = 1000.0;          // IncreaseRelAlt/DecreaseRelAlt step

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
    // The slot index is vehicleInUnit (0=lead, 1=wing 1, 2=wing 2, 3=wing 3).
    // FormationTable::slotGeometry returns the (relAz, relEl, range) for this slot.
    const auto& form = digi.formation;
    const auto slot = formation::FormationTable::defaultInstance().slotGeometry(
        static_cast<formation::FormationType>(form.formationId),
        form.vehicleInUnit);

    // --- Apply lateral spacing factor ---
    // FreeFalcon: rangeFactor = curPosition->range * (2.0 * mFormLateralSpaceFactor)
    // The 2.0 is because FF's formdat.fil stores range in "half-spacing" units.
    // F4Flight's FormationTable stores range in actual feet (defaultWedge
    // uses 1000 ft = 1000 ft actual, not 500 ft half-spacing). So we DON'T
    // multiply by 2.0 — just apply the formLateralSpaceFactor directly.
    //
    // formLateralSpaceFactor defaults to 1.0 (normal spacing).
    // Kickout sets it to 2.0 (double spacing), Closeup to 0.5 (half spacing).
    const double rangeFactor = slot.range * form.wingman.formLateralSpaceFactor;

    // --- Compute desired world position relative to lead ---
    // FreeFalcon uses lead's sigma (velocity-vector heading) as the rotation
    // angle for the relative position. This means the formation rotates with
    // the lead's heading — a wingman at relAz=30° stays at 30° off the lead's
    // nose regardless of which way the lead is flying.
    //
    // The lead entity's `yaw` field is its velocity-vector heading (sigma),
    // matching how buildSelfEntity populates it from AircraftState.kin.sigma.
    const double leadSigma = lead->yaw;

    // Apply side mirror (formSide: +1 = right side, -1 = left side).
    // Default is +1 (right). ToggleSide flips it.
    const double formSide = (form.wingman.formSide != 0)
                            ? static_cast<double>(form.wingman.formSide)
                            : 1.0;

    // World-frame offset from lead:
    //   x_offset = rangeFactor * cos(relAz * formSide + leadSigma)
    //   y_offset = rangeFactor * sin(relAz * formSide + leadSigma)
    const double azAngle = slot.relAz * formSide + leadSigma;
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

    // --- Check in-position (before adding lead-ahead offset) ---
    // The in-position check uses the actual formation slot position (without
    // the lead-ahead offset), so "in position" means "at the correct slot
    // relative to the lead" — not "at the lead-ahead trackpoint".
    AiCheckInPositionCall(digi, self, trackX, trackY, trackZ);

    // --- Add lead-ahead offset ---
    // FreeFalcon sets the trackpoint 1 NM ahead of the desired position in
    // the lead's heading direction. This gives the wingman a "lead pursuit"
    // trajectory — it flies toward where the formation slot will be, not
    // where it was. This produces smoother formation following.
    //
    // However, 1 NM is too aggressive for F4Flight's FCS — it causes the
    // wingman to chase a point 6000+ ft ahead, leading to overspeed and
    // oscillation. We use a smaller offset (0.1 NM) that provides enough
    // lead pursuit for smooth following without destabilizing the controller.
    constexpr double kLeadAheadNm = 0.1;
    const double leadAheadFt = kLeadAheadNm * 6076.0;  // 0.1 NM in feet
    trackX += leadAheadFt * std::cos(leadSigma);
    trackY += leadAheadFt * std::sin(leadSigma);

    // --- Fly to the desired position ---
    // FreeFalcon uses SimpleTrack(SimpleTrackDist, 0.0) — a closure-rate
    // based tracker. We use HeadingAndAltitudeHold + MachHold instead,
    // which is more stable for F4Flight's FCS and produces smoother formation
    // flying (no Phugoid oscillation from SimpleTrack's pure proportional
    // elevation tracker).
    //
    // Compute the desired heading (bearing to the trackpoint) and desired
    // altitude (trackZ converted to positive-up).
    const double dx = trackX - self.x;
    const double dy = trackY - self.y;
    const double desHeading = std::atan2(dy, dx);
    const double desAlt = -trackZ;  // NED: z negative up, alt positive up

    // Desired speed: match the lead's speed, with a SMALL closure correction.
    // The correction is based on the along-track error (how far behind/ahead
    // we are relative to the slot), NOT the total distance to the trackpoint
    // (which includes the lead-ahead offset and would cause overspeed).
    //
    // Project the position error onto the lead's velocity direction to get
    // the along-track error. Positive = behind the slot (need to speed up),
    // negative = ahead of the slot (need to slow down).
    //
    // The correction is clamped to ±30 ft/s (~18 kts) to prevent overspeed
    // when far from the slot. The gain is low (0.05) to prevent oscillation.
    const double slotDx = (trackX - leadAheadFt * std::cos(leadSigma)) - self.x;
    const double slotDy = (trackY - leadAheadFt * std::sin(leadSigma)) - self.y;
    const double alongTrackErr = slotDx * std::cos(leadSigma) +
                                  slotDy * std::sin(leadSigma);
    const double closureCorrection = std::max(-30.0, std::min(30.0,
        alongTrackErr * 0.05));  // 0.05 ft/s per ft of error
    const double desSpeed = std::max(150.0, lead->speed + closureCorrection);

    // Use HeadingAndAltitudeHold to steer + hold altitude.
    ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
                                                digi, as, fcs, fcsState,
                                                digi.config.maxGs);

    // Use MachHold to match speed. Convert desSpeed (ft/s) to kts for MachHold.
    const double desSpeedKts = desSpeed / KNOTS_TO_FTPSEC;
    ManeuverPrimitives::MachHold(desSpeedKts, as.vcas, false,
                                  digi, as, 150.0, 800.0, digi.nav.dt, 100.0);

    // Return true if "in position" (within threshold of the slot).
    const double dz = trackZ - self.z;
    const double dist3D = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist3D < kInPositionThresholdFt;
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
