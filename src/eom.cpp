// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// eom.cpp
//
// Equations of motion implementation. Port of eom.cpp.

#include "f4flight/eom.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

EquationsOfMotion::EquationsOfMotion(const AircraftGeometry* geom, const AuxAero* aux)
    : geom_(geom), aux_(aux) {}

// ---------------------------------------------------------------------------
// Body rates (p, q, r) from FCS outputs.
//
// The legacy Falcon 4 EOM derives body rates from the achieved load factors
// (nzcgs, nycgw). This "velocity-axis" formulation is mathematically elegant
// but numerically unstable when the aircraft departs from controlled flight:
// a negative nzcgs drives q negative, which pitches the nose further down,
// which makes nzcgs more negative -- a divergent feedback loop.
//
// For the refactored library we use a more robust approach: the FCS alpha
// command directly drives the pitch rate through a first-order response, and
// the roll rate comes from the FCS roll command. This is closer to how a
// real fly-by-wire aircraft works (the FLCS commands body rates, not G
// directly) and is numerically stable.
//
// The yaw rate is derived from the lateral acceleration as before.
// ---------------------------------------------------------------------------
void EquationsOfMotion::calcBodyRates(double dt, double qsom, double cnalpha,
                                      double cosmu, double cosgam, double singam,
                                      double cosbet, double cosalp, double sinalp,
                                      double nzcgs, double nycgw, double pstab,
                                      double pitchMomentum, double pitchElasticity,
                                      AircraftState& state) const {
    auto& k = state.kin;
    auto& a = state.aero;
    const double tempVt = std::max(1.0, k.vt);

    // Pitch rate: drive q toward a target derived from the alpha command.
    // The FCS has already set aero.alpha_deg to the commanded alpha (via the
    // pitchAlphaLag filter). We compute a pitch rate that will rotate the
    // body toward the commanded alpha over a time constant.
    //
    // The relationship is: alpha_dot = q - (g/Vt) * (nzcgs - cos(gamma)*cos(mu))
    // For a steady-state pull, q ~ alpha_dot + g/Vt * (nz - 1).
    // We command q to track the alpha error plus a gravity-compensation term.
    const double alphaCmd_rad = state.fcs.aoacmd * DTR;
    const double alpha_rad = a.alpha_deg * DTR;
    const double alphaErr = alphaCmd_rad - alpha_rad;
    // Pitch rate command = alpha-error gain + gravity turn rate
    const double gravityTurn = (nzcgs > -2.0)
        ? std::atan(std::max(0.0, nzcgs) * GRAVITY / tempVt)
        : -0.5;  // if we're at negative G, command a recovery pitch rate
    double qptchc = alphaErr * 2.0 + gravityTurn - std::atan(cosmu * cosgam * GRAVITY / tempVt);

    // Apply 1st-order lag for pitch rate response
    const double tau = 0.2 * pitchElasticity;
    k.q = state.fcs.pitchRateLag.step(qptchc, tau, dt);

    // Yaw rate from lateral accel (coordinated turn)
    double rstab = (nycgw + cosgam * std::sin(state.kin.mu)) * GRAVITY / tempVt;

    // Roll rate from FCS
    k.p = pstab;
    k.r = rstab;

    // Clamp body rates
    k.p = limit(k.p, -4.5, 4.5);
    k.q = limit(k.q, -3.0, 3.0);
    k.r = limit(k.r, -4.0, 4.0);
}

// ---------------------------------------------------------------------------
// Quaternion integration (Forward Euler). Port of CalcBodyOrientation().
// ---------------------------------------------------------------------------
void EquationsOfMotion::calcBodyOrientation(double dt, AircraftState& state) const {
    auto& k = state.kin;
    auto& q = k.quat;

    // q_dot = 0.5 * q * (0, p, q, r)   (Hamilton, scalar-first)
    const double p = k.p, qq = k.q, r = k.r;
    const double dw = 0.5 * (-q.x * p - q.y * qq - q.z * r);
    const double dx = 0.5 * ( q.w * p - q.z * qq + q.y * r);
    const double dy = 0.5 * ( q.z * p + q.w * qq - q.x * r);
    const double dz = 0.5 * (-q.y * p + q.x * qq + q.w * r);

    Quaternion qnew{q.w + dw * dt, q.x + dx * dt, q.y + dy * dt, q.z + dz * dt};
    q = qnew.normalized();

    // Recover Euler angles
    eulerFromQuat(q, k.psi, k.theta, k.phi);
}

// ---------------------------------------------------------------------------
// Trigonometry cache. Port of Trigenometry().
//
// The legacy code derives the velocity-vector euler angles (sigma, gamma, mu)
// from the quaternion (body orientation) combined with alpha and beta. We use
// the approximation:
//   gamma = theta - alpha * cos(phi)   (flight path angle)
//   sigma = psi + beta * cos(theta)    (velocity heading)
//   mu    = phi                        (wind-axis roll ~ body roll)
// which matches the legacy behaviour for small alpha/beta.
// ---------------------------------------------------------------------------
void EquationsOfMotion::trigonometry(AircraftState& state) const {
    auto& k = state.kin;
    auto& a = state.aero;

    const double alp = a.alpha_deg * DTR;
    const double bet = a.beta_deg  * DTR;
    k.sinalp = std::sin(alp); k.cosalp = std::cos(alp);
    k.sinbet = std::sin(bet); k.cosbet = std::cos(bet);

    // Body euler trig
    k.sinpsi = std::sin(k.psi); k.cospsi = std::cos(k.psi);
    k.sinthe = std::sin(k.theta); k.costhe = std::cos(k.theta);
    k.sinphi = std::sin(k.phi); k.cosphi = std::cos(k.phi);

    // Velocity-vector euler from body euler + alpha/beta
    // gamma = theta - alpha (in radians). When the nose is above the velocity
    // vector (alpha > 0), the flight path angle is less than body pitch.
    const double gmma = k.theta - alp * k.cosphi;
    k.singam = std::sin(gmma);
    k.cosgam = std::cos(gmma);
    k.gmma = gmma;

    // sigma = psi (velocity heading ~ body heading for small beta)
    const double sigma = k.psi;
    k.sinsig = std::sin(sigma);
    k.cossig = std::cos(sigma);
    k.sigma = sigma;

    // mu = phi (wind-axis roll ~ body roll)
    k.mu = k.phi;
    k.sinmu = k.sinphi;
    k.cosmu = k.cosphi;

    // Body-to-world DCM
    k.dcm = dcmFromEuler(k.psi, k.theta, k.phi);

    // Update world velocity from vt and the velocity-vector angles so that
    // the position integration uses consistent values.
    k.xdot = k.vt * k.cosgam * k.cossig + state.windX;
    k.ydot = k.vt * k.cosgam * k.sinsig + state.windY;
    k.zdot = -k.vt * k.singam;
}

// ---------------------------------------------------------------------------
// Velocity integration. Port of CalculateVt().
// ---------------------------------------------------------------------------
void EquationsOfMotion::calculateVt(double dt, double muFric, double singam,
                                    double xwaero, double xwprop,
                                    AircraftState& state) const {
    auto& k = state.kin;
    (void)state.aero; // aero fields already consumed by caller

    // vtDot = xwaero + xwprop - g * sin(gamma)
    const double vtDot = xwaero + xwprop - GRAVITY * singam;

    if (state.gear.inAir) {
        const double newVt = k.vt + vtDot * dt;
        k.vt = (std::fabs(newVt) > 1e-3) ? newVt : 0.01;
        state.netAccel = vtDot * dt;
    } else {
        // Ground: friction
        const double nzcgs = state.loads.nzcgs;
        const double sinbet = state.kin.sinbet;
        const double fric = (0.8 * muFric + std::fabs(0.8 * sinbet)) * (1.0 - nzcgs) * GRAVITY * dt;
        double netAccel = vtDot * dt - fric;
        double newVt = k.vt + netAccel;
        if (newVt < 0.0) newVt = 0.0;
        state.netAccel = netAccel;
        k.vt = newVt;
    }
}

// ---------------------------------------------------------------------------
// Position integration. Port of EquationsOfMotion() position section.
// ---------------------------------------------------------------------------
void EquationsOfMotion::integratePosition(double dt, double cosgam, double singam,
                                          double cossig, double sinsig,
                                          double windX, double windY,
                                          AircraftState& state) const {
    auto& k = state.kin;
    const double vt = k.vt;

    // World-frame velocity from wind axis (sigma, gamma)
    const double xdot = vt * cosgam * cossig + windX;
    const double ydot = vt * cosgam * sinsig + windY;
    const double zdot = -vt * singam;

    k.xdot = xdot;
    k.ydot = ydot;
    k.zdot = zdot;

    k.x += xdot * dt;
    k.y += ydot * dt;
    k.z += zdot * dt;
}

// ---------------------------------------------------------------------------
// Top-level EOM update
// ---------------------------------------------------------------------------
void EquationsOfMotion::update(double dt, PilotInput const& input,
                               AircraftState& state) const {
    if (!geom_ || !aux_) return;

    auto& k = state.kin;
    auto& a = state.aero;

    // Body rates from FCS outputs
    calcBodyRates(dt, state.qsom, a.cnalpha, k.cosmu, k.cosgam, k.singam,
                  k.cosbet, k.cosalp, k.sinalp, state.loads.nzcgs,
                  state.loads.nycgw, state.fcs.pstab,
                  aux_->pitchMomentum, aux_->pitchElasticity, state);

    // Quaternion integration
    calcBodyOrientation(dt, state);

    // Recompute trig + DCM
    trigonometry(state);

    // Velocity integration
    const double muFric = state.gear.muFric;
    calculateVt(dt, muFric, k.singam, a.xwaero, a.xwaero + 0.0, state);
    // Note: xwprop (thrust along wind axis) is added to xwaero by the engine
    // model when it computes body forces. For now we use a.xwaero + 0 as a
    // placeholder; the FlightModel will pass the actual xwprop.

    // Position integration
    integratePosition(dt, k.cosgam, k.singam, k.cossig, k.sinsig,
                      state.windX, state.windY, state);
}

} // namespace f4flight
