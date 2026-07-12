// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// eom.cpp
//
// Equations of motion implementation. Port of eom.cpp.

#include "f4flight/eom.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"
#include "f4flight/core/trig.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

EquationsOfMotion::EquationsOfMotion(const AircraftGeometry* geom, const AuxAero* aux)
    : geom_(geom), aux_(aux) {}

// ---------------------------------------------------------------------------
// Body rates (p, q, r) from FCS outputs.
//
// Bug #3 fix: ported from FreeFalcon eom.cpp:595-717 (in-air section).
//
// FreeFalcon derives body rates from achieved load factors (nzcgs, nycgw)
// and FCS roll-rate command (pstab). This is a "velocity-axis" formulation:
//   q = lag( atan(nzcgs*g/Vt) - atan(gearDrag*g/Vt) + pitch*cosbet
//            - atan(cosmu*cosgam*g/Vt),  tp01*pitchElasticity, dt )
//   r = (nycgw + cosgam*sinmu) * g / Vt
//   p = pstab  (from FCS roll controller)
//
// The previous f4flight code used an ad-hoc alpha-error proportional
// controller with hardcoded rate clamps (±4.5/±3.0/±4.0 rad/s), which is
// completely different dynamics. It also clamped nzcgs to non-negative,
// preventing negative-G maneuvers.
// ---------------------------------------------------------------------------
void EquationsOfMotion::calcBodyRates(double dt, double qsom, double cnalpha,
                                      double cosmu, double cosgam, double singam,
                                      double cosbet, double cosalp, double sinalp,
                                      double nzcgs, double nycgw, double pstab,
                                      double pitchMomentum, double pitchElasticity,
                                      AircraftState& state) const {
    auto& k = state.kin;
    auto& a = state.aero;
    const double tempVt = std::max(4.0, std::fabs(k.vt));
    (void)singam; // not used by FreeFalcon in this section
    (void)cnalpha; // not used by FreeFalcon in this section
    (void)cosalp;  // used only in slice coupling (turbulence, omitted)
    (void)sinalp;

    // --- Pitch rate (body axis) ---
    // FreeFalcon eom.cpp:605-642
    double qptchc = 0.0;
    const double gearPos = a.gearPos;
    if (gearPos < 1.0) {
        qptchc += std::atan(nzcgs * GRAVITY / tempVt)
                - std::atan(0.2 * gearPos * qsom / tempVt);
    } else {
        qptchc += std::atan(nzcgs * GRAVITY / tempVt)
                - std::atan(0.1 * gearPos * qsom / tempVt);
    }
    // pitch * cosbet term (pitch = turbulence disturbance, ~0 in normal flight)
    // qptchc += 0.0 * cosbet;  // omitted: no turbulence model

    // Gravity turn-rate compensation
    qptchc -= std::atan(cosmu * cosgam * GRAVITY / tempVt);

    // First-order lag using tp01 * pitchElasticity as the time constant.
    // FreeFalcon: q = FLTust(qptchc, tp01 * pitchElasticity, dt, oldp05)
    // Note: tp01 is set by the FCS gain computation (default 0.2). We use
    // the fcs.tp01 value from state.
    const double tau_q = state.fcs.tp01 * pitchElasticity;
    k.q = state.fcs.pitchRateLag.step(qptchc, tau_q, dt);

    // --- Yaw rate (body axis) ---
    // FreeFalcon eom.cpp:693-694
    // rstab = (nycgw + cosgam * sinmu) * g / Vt
    const double rstab = (nycgw + cosgam * k.sinmu) * GRAVITY / tempVt;
    k.r = rstab;

    // --- Roll rate (body axis) ---
    // FreeFalcon eom.cpp:701: p = pstab (from FCS roll controller)
    // (rollCouple is added to pstab in the FCS, not here)
    k.p = pstab;

    // No hardcoded rate clamps — FreeFalcon has none.
    // The FCS and aerodynamics naturally limit the rates.
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

    // All sin*/cos* fields and the velocity-vector euler angles (sigma,
    // gmma, mu) come from the shared helper. This is the exact same math
    // that used to be inlined here; consolidating it means init() and the
    // per-frame update can no longer drift apart.
    recomputeKinematicTrig(k, a.alpha_deg, a.beta_deg);

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

    // Velocity integration.
    //
    // Bug #2 fix (was: calculateVt(dt, muFric, k.singam, a.xwaero, a.xwaero + 0.0, state)).
    //
    // The flight model (flight_model.cpp:303) already adds thrust into
    // a.xwaero (xwaero += xwprop). Passing xwaero as BOTH the xwaero and
    // xwprop arguments caused calculateVt() to compute
    //     vtDot = xwaero + xwprop = (drag+thrust) + (drag+thrust) = 2*(drag+thrust)
    // i.e. thrust and drag were double-counted, giving ~2x the correct
    // acceleration. Measured ratio was 1.92 (see scripts/bug2_check.cpp).
    //
    // Fix: pass 0 for xwprop since thrust is already in xwaero.
    const double muFric = state.gear.muFric;
    calculateVt(dt, muFric, k.singam, a.xwaero, 0.0, state);

    // Position integration
    integratePosition(dt, k.cosgam, k.singam, k.cossig, k.sinsig,
                      state.windX, state.windY, state);
}

} // namespace f4flight
