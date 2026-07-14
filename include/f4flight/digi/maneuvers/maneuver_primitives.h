// f4flight - digi/maneuvers/maneuver_primitives.h
//
// Core maneuver primitives — the "fly to a point" library used by every
// digi AI mode. These are direct ports of FreeFalcon's DigitalBrain steering
// functions (mnvers.cpp, autopilot.cpp).
//
// All functions are static (no instance state) and operate on DigiState +
// AircraftState + FcsState. The DigiBrain owns the DigiState and dispatches
// to these functions based on the active mode.
//
// Source mapping (FreeFalcon sim/digi/):
//   SetPstick   <- mnvers.cpp:300-362
//   SetRstick   <- mnvers.cpp:364-390
//   SetYpedal   <- mnvers.cpp:392-397
//   GammaHold   <- mnvers.cpp:837-866
//   AltHold     <- autopilot.cpp:323-378
//   AltitudeHold<- mnvers.cpp:759-784
//   HeadingAlt  <- mnvers.cpp:786-835
//   LevelTurn   <- mnvers.cpp:713-757
//   MachHold    <- mnvers.cpp:414-665
//   Loiter      <- mnvers.cpp:667+
//
// This header is the NEW home for these declarations. The old steering.h
// re-exports them for backward compatibility.

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"

namespace f4flight {
namespace digi {

// Command type for SetPstick (matches FreeFalcon AirframeClass flags)
enum class CommandType : int {
    ErrorCommand = 0,
    GCommand     = 1,
    AlphaCommand = 2
};

class ManeuverPrimitives {
public:
    // SetPstick — convert a G-command (or error/alpha) to a stick deflection.
    // Direct port of DigitalBrain::SetPstick (mnvers.cpp:300-362).
    static void SetPstick(double pitchError, double gLimit, CommandType commandType,
                          DigiState& digi, const AircraftState& state);

    // SetRstick — convert a roll error (degrees) to a stick deflection.
    static void SetRstick(double rollError, DigiState& digi,
                          const FlightControlSystem& fcs,
                          const FcsState& fcsState);

    // SetYpedal — convert a yaw error to a pedal deflection.
    static void SetYpedal(double yawError, DigiState& digi);

    // GammaHold — hold a desired flight path angle (gamma, degrees).
    static void GammaHold(double desGamma, DigiState& digi,
                          const AircraftState& state, double maxGs);

    // AltHold — hold a desired altitude (feet, positive up).
    static void AltHold(double desAlt, DigiState& digi, const AircraftState& state,
                        double maxGs);

    // AltitudeHold — full altitude hold with wings level.
    // Returns true if within 25 ft of target.
    static bool AltitudeHold(double desAlt, DigiState& digi,
                             const AircraftState& state,
                             const FlightControlSystem& fcs,
                             FcsState& fcsState, double maxGs);

    // HeadingAndAltitudeHold — hold heading + altitude.
    // Returns true if within tolerance.
    static bool HeadingAndAltitudeHold(double desPsi, double desAlt,
                                       DigiState& digi, const AircraftState& state,
                                       const FlightControlSystem& fcs,
                                       FcsState& fcsState, double maxGs);

    // LevelTurn — turn to a heading at a specified load factor.
    static void LevelTurn(double loadFactor, double turnDir, bool newTurn,
                          DigiState& digi, const AircraftState& state,
                          const FlightControlSystem& fcs,
                          FcsState& fcsState, double maxGs);

    // MachHold — hold a target speed via throttle.
    // Returns true if within 10% of target speed.
    static bool MachHold(double targetSpeed, double currentSpeed, bool adjustPitch,
                         DigiState& digi, const AircraftState& state,
                         double minVcas, double maxVcas,
                         double dt, double burnerDelta = 500.0);

    // Loiter — orbit pattern (30° bank).
    static void Loiter(DigiState& digi, const AircraftState& state,
                       const FlightControlSystem& fcs,
                       FcsState& fcsState, double maxGs);

    // --- Combat primitives ---

    // TrackPoint — fly toward a specific world point (x, y, alt).
    // Navigation primitive: turns velocity vector to heading + holds altitude.
    // FreeFalcon mnvers.cpp:105 (TrackPoint → AutoTrack in complex mode).
    static void TrackPoint(double targetX, double targetY, double targetAlt,
                           DigiState& digi, const AircraftState& state,
                           const FlightControlSystem& fcs,
                           FcsState& fcsState, double maxGs);

    // AutoTrack — the core offensive BFM primitive.
    // Rolls the lift vector onto the target and pulls along the body z-axis.
    // Reads trackX/Y/Z from DigiState (set by the caller before invoking).
    // Port of FreeFalcon mnvers.cpp:211-298.
    //
    // Three branches based on off-boresight angle (ata):
    //   ata < 5°:  fine track — ErrorCommand pitch, wings-level roll
    //   ata < 10° (BVR only): flip-over branch (roll opposite + neg G)
    //   large ata: roll lift vector onto target + AlphaCommand pull
    static double AutoTrack(DigiState& digi, const AircraftState& state,
                            FcsState& fcsState, double maxGs);

    // GunsAutoTrack — gun-specific tracking primitive.
    // SEPARATE from offensive AutoTrack. Adds lead for bullet TOF +
    // gravity drop, biases rz by 2× for lead, uses GCommand (vs
    // AutoTrack's AlphaCommand). Port of FreeFalcon gengage.cpp:362-433.
    //
    // Reads trackX/Y/Z from DigiState. The caller (CoarseGunsTrack) sets
    // these to the lead-aim point (target position + velocity * TOF -
    // gravity drop).
    static double GunsAutoTrack(DigiState& digi, const AircraftState& state,
                                FcsState& fcsState, double maxGs);

    // TrackPointLanding — landing-specific primitive.
    // Uses SimpleTrackAzimuth (proportional roll) + SimpleTrackElevation
    // (proportional pitch, no integral, no +1G bias). Structurally incapable
    // of the Phugoid oscillation that GammaHold produces on a moving target.
    // Port of FreeFalcon mnvers.cpp:33-103.
    //
    // Reads trackX/Y/Z from DigiState. The caller sets trackX/Y to the
    // runway threshold and trackZ to the glideslope altitude.
    static void TrackPointLanding(double targetSpeedKts,
                                   DigiState& digi, const AircraftState& state,
                                   double dt);

    // SimpleTrackElevation — pure proportional elevation-angle tracker.
    // Port of FreeFalcon wingmnvers.cpp:333-397.
    //   zft    : altitude error (target_alt - current_alt), positive = need to climb
    //   scale  : horizontal distance (ft) — used as the proportional gain denominator
    // Returns pitch command in [-0.5, +0.5].
    static double SimpleTrackElevation(double zft, double scale,
                                       const AircraftState& state);

    // SimpleTrackAzimuth — proportional azimuth tracker.
    // Port of FreeFalcon wingmnvers.cpp:253-323.
    //   rx, ry : target position in body frame (ft)
    // Returns roll command in [-1, +1].
    static double SimpleTrackAzimuth(double rx, double ry);

    // VectorTrack — track a commanded velocity vector (heading + speed + alt).
    // FreeFalcon mnvers.cpp:VectorTrack. Used by formation following.
    static void VectorTrack(double desHeading, double desAlt, double desSpeed,
                            DigiState& digi, const AircraftState& state,
                            const FlightControlSystem& fcs,
                            FcsState& fcsState, double maxGs, double dt);

    // -----------------------------------------------------------------------
    // Round-2 structural additions (Rec 10): the 4 missing maneuver
    // primitives that the 9 unported DigiMode values dispatch to.
    // Without these, adding the modes to the enum would be dead code.
    // -----------------------------------------------------------------------

    // PullToCollisionPoint — lead-pursuit trackpoint smoother.
    // Port of FreeFalcon randp.cpp:445-555.
    //
    // Sets trackX/Y/Z to the predicted collision point with the target:
    //   tc = range / closure_rate (collision time)
    //   if tc > 0: trackPoint = target_pos + target_vel * tc
    //   else:      trackPoint = target_pos + target_vel * MAGIC_NUMBER (2 s)
    //
    // On the first frame (lastMode != curMode), sets the trackpoint directly.
    // On subsequent frames, SMOOTHS the trackpoint: 0.1*new + 0.9*old. This
    // smoothing is the key behavior — without it, target jitter propagates
    // directly to the AutoTrack pitch/roll commands and the aircraft
    // oscillates. F4Flight's TrackPoint/AutoTrack currently set the
    // trackpoint each frame with no smoothing, which is why BFM tracking
    // is jittery.
    //
    // After setting trackX/Y/Z, calls AutoTrack(maxGs) to fly to it.
    //
    //   target : the entity to track (read for position + velocity)
    //   self   : own aircraft entity (read for position)
    //   firstFrame : true on the first frame of the mode (no smoothing)
    static void PullToCollisionPoint(DigiState& digi,
                                     const DigiEntity& self,
                                     const DigiEntity& target,
                                     const AircraftState& as,
                                     const FlightControlSystem& fcs,
                                     FcsState& fcsState,
                                     double maxGs, bool firstFrame);

    // OverBank — roll to (target.droll ± delta) for lateral separation.
    // Port of FreeFalcon mnvers.cpp:920-965.
    //
    // Used by OverBMode to gain separation in a turning fight. Computes a
    // target roll angle = target's roll + delta (or - delta if our roll is
    // negative), then SetRstick toward it. Skipped in vertical fights
    // (|pitch| > 45°).
    //
    //   target : the entity whose droll (roll difference) we add delta to
    //   delta  : bank offset (radians) — typically 30° per FF randp.cpp
    //   firstFrame : true on the first frame of the mode (compute newRoll)
    static void OverBank(DigiState& digi,
                         const DigiEntity& self,
                         const DigiEntity& target,
                         const FlightControlSystem& fcs,
                         FcsState& fcsState,
                         double delta, bool firstFrame);

    // RollOutOfPlane — roll 30° toward vertical to break a stalemate.
    // Port of FreeFalcon mnvers.cpp:868-918.
    //
    // Used by RoopMode. On first frame, picks a newRoll = self.roll ± 30°
    // (toward vertical). Then pulls max-G (gsAvail) + rolls toward newRoll
    // for 1 second.
    //
    //   firstFrame : true on the first frame of the mode (compute newRoll)
    //   returns    : true while the maneuver is still active (mnverTime > 0)
    static bool RollOutOfPlane(DigiState& digi,
                                const DigiEntity& self,
                                const AircraftState& as,
                                const FlightControlSystem& fcs,
                                FcsState& fcsState,
                                double dt, bool firstFrame);

    // WvrBugOut — disengage: hold heading + altitude, accelerate to 2x corner.
    // Port of FreeFalcon wvrengage.cpp:727-731.
    //
    // Used by SeparateMode and BugoutMode. The simplest of the four new
    // primitives — just fly straight and fast to escape the fight.
    static void WvrBugOut(DigiState& digi,
                          const AircraftState& as,
                          const FlightControlSystem& fcs,
                          FcsState& fcsState,
                          double dt);
};

} // namespace digi
} // namespace f4flight
