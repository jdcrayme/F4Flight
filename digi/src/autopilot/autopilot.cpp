// f4flight - digi/autopilot/autopilot.cpp
//
// Structured autopilot implementation.
//
// Port of FreeFalcon's autopilot.cpp. Uses the existing ManeuverPrimitives
// under the hood, structured into a clean mode hierarchy.

#include "f4flight/digi/autopilot/autopilot.h"
#include "f4flight/digi/steering.h"  // headingError
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/flight/core/airspeed_conversions.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// update — dispatch on the selected mode
// ===========================================================================
void Autopilot::update(DigiState& digi, const AircraftState& as,
                        const FlightControlSystem& fcs, FcsState& fcsState,
                        double dt) {
    if (mode_ != AutopilotMode::PitchRollHold) {
        attitudeCaptured_ = false;
    }

    if (mode_ == AutopilotMode::Off) return;

    // Set the flight phase to Cruise for gain scheduling.
    // The autopilot is a cruise-mode controller — combat/formation/landing
    // phases use their own mode dispatch, not the autopilot.
    digi.nav.flightPhase = FlightPhase::Cruise;

    switch (mode_) {
        case AutopilotMode::AltitudeHold:
            altitudeHold(digi, as, fcs, fcsState, dt);
            break;
        case AutopilotMode::HeadingSelect:
            headingSelect(digi, as, fcs, fcsState, dt);
            break;
        case AutopilotMode::AltitudeSelect:
            altitudeSelect(digi, as, fcs, fcsState, dt);
            break;
        case AutopilotMode::PitchRollHold:
            pitchRollHold(digi, as, fcs, fcsState, dt);
            break;
        default:
            break;
    }
}

// ===========================================================================
// altitudeHold — hold target altitude + heading
//
// Port of FreeFalcon's AltHold (autopilot.cpp:323-378) + ThreeAxisAP.
//
// FreeFalcon uses a damped altitude-hold: near the target altitude (within
// 15 ft), it commands level flight (gamma=0). Further away, it uses a
// proportional gamma correction with increasing gain. This prevents the
// "porpoising" that a constant-gain controller produces.
//
// F4Flight uses HeadingAndAltitudeHold which already has the PD formulation
// + Phugoid damping. We delegate to it with the cruise-phase gains.
// ===========================================================================
void Autopilot::altitudeHold(DigiState& digi, const AircraftState& as,
                              const FlightControlSystem& fcs, FcsState& fcsState,
                              double dt) {
    (void)fcs; (void)fcsState;
    // HeadingAndAltitudeHold handles both heading + altitude with the
    // phase-aware gains (Cruise phase: gammaGain 0.015, integralGain 0.0025,
    // phugoidGain 0.3, rollDampGain 0.3).
    ManeuverPrimitives::HeadingAndAltitudeHold(targetHeading_, targetAlt_,
        digi, as, fcs, fcsState, digi.config.maxGs);

    // Speed control: hold target speed (or corner speed if not set).
    const double speedKts = (targetSpeed_ > 0.0) ? targetSpeed_ : digi.config.cornerSpeed;
    ManeuverPrimitives::machHoldCas(cas_kts(speedKts), true,
        digi, as, 200.0, 800.0, dt, 700.0);
}

// ===========================================================================
// headingSelect — turn to target heading, then hold altitude + heading
//
// Port of FreeFalcon's HDGSel (autopilot.cpp:523-550).
//
// This is the same as AltitudeHold but with emphasis on the heading turn.
// The HeadingAndAltitudeHold controller already handles this — it commands
// a bank proportional to the heading error, capped at maxRoll.
// ===========================================================================
void Autopilot::headingSelect(DigiState& digi, const AircraftState& as,
                                const FlightControlSystem& fcs, FcsState& fcsState,
                                double dt) {
    (void)fcs; (void)fcsState;
    ManeuverPrimitives::HeadingAndAltitudeHold(targetHeading_, targetAlt_,
        digi, as, fcs, fcsState, digi.config.maxGs);

    const double speedKts = (targetSpeed_ > 0.0) ? targetSpeed_ : digi.config.cornerSpeed;
    ManeuverPrimitives::machHoldCas(cas_kts(speedKts), true,
        digi, as, 200.0, 800.0, dt, 700.0);
}

// ===========================================================================
// altitudeSelect — climb/descend to target altitude
//
// Port of FreeFalcon's AltHold with the LantirnAP gamma correction.
//
// Commands a gamma proportional to the altitude error. When within 100 ft
// of the target, transitions to altitude hold.
// ===========================================================================
void Autopilot::altitudeSelect(DigiState& digi, const AircraftState& as,
                                 const FlightControlSystem& /*fcs*/, FcsState& /*fcsState*/,
                                 double dt) {
    const double altErr = (targetAlt_ + as.kin.z) - as.kin.zdot;
    // FreeFalcon's damped gain schedule:
    //   |err| < 15 ft:  gamma = 0 (level)
    //   |err| < 50 ft:  gamma = err * 0.0015
    //   |err| < 100 ft: gamma = err * 0.005
    //   |err| >= 100:   gamma = err * 0.015
    double gammaGain;
    const double absErr = std::fabs(altErr);
    if (absErr < 15.0) {
        gammaGain = 0.0;
    } else if (absErr < 50.0) {
        gammaGain = 0.0015;
    } else if (absErr < 100.0) {
        gammaGain = 0.005;
    } else {
        gammaGain = 0.015;
    }

    // Clear the integrator (pure P for altitude select — no windup).
    digi.nav.gammaHoldIError = 0.0;
    ManeuverPrimitives::GammaHold(altErr * gammaGain, digi, as, digi.config.maxGs);
    ManeuverPrimitives::PhugoidDamper(digi, as);  // phase-aware

    // Wings level (no heading command during altitude select).
    ManeuverPrimitives::SetYpedal(0.0, digi);
    const double rollDeg = as.kin.phi * RTD;
    // Command wings level: roll error = -2 * current roll (proportional).
    const double rollErr = -rollDeg * 2.0;
    digi.commands.rStick = std::max(-1.0, std::min(1.0, rollErr * DTR * 0.75));

    // Speed control.
    const double speedKts = (targetSpeed_ > 0.0) ? targetSpeed_ : digi.config.cornerSpeed;
    ManeuverPrimitives::machHoldCas(cas_kts(speedKts), true,
        digi, as, 200.0, 800.0, dt, 700.0);
}

// ===========================================================================
// pitchRollHold — hold pitch + roll attitude
//
// Port of FreeFalcon's PitchRollHold (autopilot.cpp:380-470).
//
// Holds the captured pitch (gamma) and roll (phi) attitudes from the moment
// the mode is engaged.
// ===========================================================================
void Autopilot::pitchRollHold(DigiState& digi, const AircraftState& as,
                                const FlightControlSystem& /*fcs*/, FcsState& /*fcsState*/,
                                double /*dt*/) {
    // Capture attitude on first frame of entering the mode
    if (!attitudeCaptured_) {
        targetPitch_ = as.kin.gmma * RTD;
        targetRoll_ = as.kin.phi * RTD;
        attitudeCaptured_ = true;
    }

    // Hold the captured pitch attitude (gamma target)
    digi.nav.gammaHoldIError = 0.0;
    ManeuverPrimitives::GammaHold(targetPitch_, digi, as, digi.config.maxGs);
    ManeuverPrimitives::PhugoidDamper(digi, as);

    // Hold the captured roll attitude (phi target)
    ManeuverPrimitives::SetYpedal(0.0, digi);
    double rollDeg = as.kin.phi * RTD;
    // Command roll hold: roll error = targetRoll_ - rollDeg (proportional).
    double rollErr = targetRoll_ - rollDeg;
    digi.commands.rStick = std::max(-1.0, std::min(1.0, rollErr * DTR * 0.75));

    // Speed control.
    const double speedKts = (targetSpeed_ > 0.0) ? targetSpeed_ : digi.config.cornerSpeed;
    ManeuverPrimitives::machHoldCas(cas_kts(speedKts), true,
        digi, as, 200.0, 800.0, digi.nav.dt, 700.0);
}

} // namespace digi
} // namespace f4flight
