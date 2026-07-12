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
//   - Computes the velocity-vector euler angles to MATCH FREEFALCON:
//       gamma = theta       (FreeFalcon eom.cpp:856 extracts gmma directly
//                            from the body quaternion; it is the body pitch
//                            angle, NOT the true flight-path angle. Naming
//                            is misleading but we match FreeFalcon so the
//                            FCS gains and EOM gravity terms behave the same.)
//       sigma = psi         (body yaw; no beta correction in FreeFalcon)
//       mu    = phi         (body roll)
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
    (void)alpha_deg; (void)beta_deg;  // aero angles are NOT used by FreeFalcon's
                                       // sigma/gamma/mu extraction (they come
                                       // from the body quaternion alone)
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

    // Velocity-vector euler angles -- MATCH FREEFALCON.
    //
    // FreeFalcon eom.cpp:855-857 extracts sigma/gamma/mu directly from the
    // body quaternion, which (after resolving the (e1,e2,e3,e4)=(qw,qz,qy,qx)
    // storage swap) reduces to the standard ZYX extraction. So FreeFalcon's
    // "gmma" is actually theta (body pitch), its "sigma" is psi, its "mu" is
    // phi. The legacy variable names are misleading ("flight path angle")
    // but the math is just body euler.
    //
    // Previously f4flight used the physically-correct flight-path-angle
    // formula gmma = theta - alpha*cos(phi). This is more accurate physics
    // but does NOT match FreeFalcon -- it changes every gravity term in the
    // EOM (qptchc, rstab, vtDot, xdot/ydot/zdot) and was a source of subtle
    // dynamic divergence from the original. We now match FreeFalcon exactly.
    kin.gmma   = kin.theta;
    kin.singam = kin.sinthe;
    kin.cosgam = kin.costhe;

    kin.sigma  = kin.psi;
    kin.sinsig = kin.sinpsi;
    kin.cossig = kin.cospsi;

    kin.mu     = kin.phi;
    kin.sinmu  = kin.sinphi;
    kin.cosmu  = kin.cosphi;
}

} // namespace f4flight
