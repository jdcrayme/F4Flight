// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/trig.h
//
// Consolidated trigonometry cache for KinematicState.
//
// The legacy model recomputes 16 sin/cos values every frame in
// EquationsOfMotion::trigonometry(). The init path in FlightModel::init()
// recomputes a SUBSET of the same fields. The two paths had drifted slightly
// (the init path left singam/cosgam/sinpsi/cospsi/etc. as their default 0/1,
// which is harmless because the first update() call fills them in, but it
// means the initial state is inconsistent if a host reads it before the first
// update).
//
// This header provides a single `recomputeKinematicTrig()` helper that both
// paths call. Behaviour is preserved (the same formulas, the same field
// assignments), but the logic now lives in one place.
//
// What the helper does:
//   - Fills sinalp/cosalp, sinbet/cosbet from alpha_deg, beta_deg
//   - Fills sinpsi/cospsi, sinthe/costhe, sinphi/cosphi from psi, theta, phi
//   - Computes the velocity-vector euler angles:
//       gamma = theta - alpha_rad * cos(phi)
//       sigma = psi
//       mu    = phi
//     and fills singam/cosgam, sinsig/cossig, sinmu/cosmu, and the
//     angle fields gmma, sigma, mu.
//
// What it does NOT do (left to the caller):
//   - Update the body-to-world DCM (`KinematicState::dcm`)
//   - Recompute world-frame velocity (xdot, ydot, zdot)
//
// Those two operations depend on context (wind, etc.) that the caller owns, so
// they stay in EquationsOfMotion::trigonometry() and FlightModel::init().

#pragma once

#include "f4flight/aircraft_state.h"
#include "f4flight/core/constants.h"

#include <cmath>

namespace f4flight {

// Recompute every sin*/cos* field and the velocity-vector euler angles in
// `kin` from the body euler angles (psi, theta, phi) and the aero angles
// (alpha_deg, beta_deg). See file header for the exact formulas.
inline void recomputeKinematicTrig(KinematicState& kin,
                                   double alpha_deg,
                                   double beta_deg) noexcept {
    const double alp = alpha_deg * DTR;
    const double bet = beta_deg  * DTR;

    // Aero-angle trig
    kin.sinalp = std::sin(alp);
    kin.cosalp = std::cos(alp);
    kin.sinbet = std::sin(bet);
    kin.cosbet = std::cos(bet);

    // Body euler trig
    kin.sinpsi = std::sin(kin.psi);   kin.cospsi = std::cos(kin.psi);
    kin.sinthe = std::sin(kin.theta); kin.costhe = std::cos(kin.theta);
    kin.sinphi = std::sin(kin.phi);   kin.cosphi = std::cos(kin.phi);

    // Velocity-vector euler from body euler + alpha/beta.
    //   gamma = theta - alpha (in radians), with a small roll-coupling term
    //   so that at non-zero bank the flight path angle is reduced by the
    //   alpha*cos(phi) projection. Matches the legacy behaviour for small
    //   alpha/beta.
    const double gmma = kin.theta - alp * kin.cosphi;
    kin.gmma   = gmma;
    kin.singam = std::sin(gmma);
    kin.cosgam = std::cos(gmma);

    // sigma = psi (velocity heading ~ body heading for small beta)
    const double sigma = kin.psi;
    kin.sigma  = sigma;
    kin.sinsig = std::sin(sigma);
    kin.cossig = std::cos(sigma);

    // mu = phi (wind-axis roll ~ body roll)
    kin.mu     = kin.phi;
    kin.sinmu  = kin.sinphi;
    kin.cosmu  = kin.cosphi;
}

} // namespace f4flight
