// f4flight - digi/ground/ground_avoid.h
//
// Ground avoidance — Tier 0 digi capability.
//
// Direct port concept of FreeFalcon's ground.cpp (GroundCheck + PullUp, 304 LOC).
// Computes whether the aircraft is on a collision course with terrain and,
// if so, commands a max-G wings-level pull-up.
//
// FreeFalcon ground.cpp:24 (GroundCheck):
//   - Computes turn radius from SustainedGs(TRUE) and current roll rate
//   - Looks ahead along the predicted recovery path (sampling every 0.5s)
//   - Sets groundAvoidNeeded = TRUE when altitude above terrain < 2*turnRadius
//
// FreeFalcon ground.cpp:208 (PullUp):
//   - MachHold(cornerSpeed - 100)  — smallest turn-radius speed
//   - Limits roll to gaRoll (wings level)
//   - Full pstick (1.0) when within g_nCriticalPullup time-to-impact
//   - pullupTimer holds the pull for g_fPullupTime seconds
//
// We implement a simplified version that uses a flat-earth ground model
// (groundZ passed in by the host). A real terrain sampler can be plugged in
// later by replacing the groundZ parameter with a callback.

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/flight/aircraft_state.h"

namespace f4flight {
namespace digi {

// GroundCheck — assess whether ground avoidance is needed.
//
//   state      : current aircraft state (needs kin.x/y/z, kin.vt, kin.theta,
//                kin.phi, kin.gmma, loads.nzcgs)
//   groundZ    : terrain altitude at the aircraft's current position (ft,
//                positive up). For flat-earth testing, pass 0.0.
//   lookahead  : how far ahead to project the recovery path (seconds)
//
// Returns: true if the aircraft is within 2*turnRadius of the terrain along
//          the predicted path and a pull-up is needed.
//
// Updates: digi.groundAvoid.groundAvoidNeeded
bool GroundCheck(DigiState& digi, const AircraftState& state,
                 double groundZ, double lookahead = 5.0);

// PullUp — execute the max-G wings-level pull-up maneuver.
//
// Commands:
//   - Wings level (rStick drives phi to 0)
//   - Full pstick (1.0) — max G away from ground
//   - MachHold at cornerSpeed (smallest turn radius for recovery)
//
// Updates: digi.commands.pStick, digi.commands.rStick, digi.commands.yPedal, digi.groundAvoid.pullupTimer
// Writes:  fcsState.maxRoll = 0 (wings level), fcsState.maxRollDelta = 5
void PullUp(DigiState& digi, const AircraftState& state,
            double cornerSpeed, double dt,
            FcsState& fcsState, double maxGs);

// Convenience: run GroundCheck + (if needed) PullUp in one call.
// Returns true if a pull-up was commanded this frame.
bool RunGroundAvoid(DigiState& digi, const AircraftState& state,
                    double groundZ, double cornerSpeed, double dt,
                    FcsState& fcsState, double maxGs,
                    double lookahead = 5.0);

} // namespace digi
} // namespace f4flight
