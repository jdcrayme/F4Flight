// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// eom.cpp
//
// Equations of motion implementation. Port of eom.cpp.

#include "f4flight/flight/eom.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/flight/core/trig.h"

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
    (void)cosbet;  // used only in slice coupling (turbulence, omitted)
    (void)cosalp;  // used only in slice coupling (turbulence, omitted)
    (void)sinalp;
    (void)pitchMomentum; // FreeFalcon scales qptchc by pitchMomentum; we use pitchElasticity only

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

    // Body-rate clamps. FreeFalcon eom.cpp:805-808 clamps p/q/r to keep
    // forward-Euler quaternion integration stable ("if we rotate too fast
    // the quaternions go nutty"). The previous f4flight code had a comment
    // claiming "FreeFalcon has none" -- that was incorrect. Without these
    // clamps, any transient that drives |p|/|q|/|r| above ~4 rad/s (stall
    // departure, large abrupt input, NaN-adjacent feedback) will cause the
    // quaternion to tumble, after which eulerFromQuat produces garbage and
    // the aircraft crashes.
    k.p = limit(k.p, -4.5, 4.5);
    k.q = limit(k.q, -3.0, 3.0);
    k.r = limit(k.r, -4.0, 4.0);

    // FreeFalcon eom.cpp:810: integrate roll rate into startRoll for the
    // FCS RollIt() damping term (1 - startRoll/maxRollDelta). startRoll is
    // NOT reset here — the steering layer resets it via SetMaxRollDelta when
    // starting a new turn.
    state.fcs.startRoll += k.p * dt;
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
        state.vtDot    = vtDot;          // ft/s^2 (true airspeed rate)
        state.netAccel = vtDot * dt;     // ft/s, per-frame delta-Vt (legacy field)
    } else {
        // Ground: friction. The weight-on-wheels factor is nzcgs (normal G).
        // At nzcgs=1.0 (sitting on ground) friction is MAXIMUM; as the aircraft
        // rotates and nzcgs drops below 1.0, weight shifts off the wheels and
        // friction decreases.
        //
        // The previous formula had two bugs:
        //   1. (1.0 - nzcgs) INVERTED the weight-on-wheels logic
        //   2. A 0.8× multiplier on muFric reduced effective brake friction
        //      from 0.4 to 0.32, giving only ~3.7 kts/s deceleration (drag only)
        //      instead of the expected ~10+ kts/s with real braking.
        //
        // Fix: use nzcgs directly as the weight-on-wheels factor, and remove
        // the 0.8× multiplier. The brake friction coefficient (0.7 in
        // calcMuFric) now produces ~15 kts/s deceleration at full brakes,
        // matching real aircraft performance.
        const double nzcgs = state.loads.nzcgs;
        const double sinbet = state.kin.sinbet;
        // Weight on wheels: at high speed the wings generate lift even on
        // the ground, reducing nzcgs below 1.0 and thus reducing braking
        // effectiveness. Real aircraft have this same issue — braking is
        // less effective above liftoff speed. We enforce a minimum of 0.5
        // (50% of weight always on the wheels via strut compression) so
        // that braking is meaningful at high speed. Without this floor, an
        // F-16 at 200 kts on the ground generates enough lift to reduce
        // nzcgs to ~0.3, cutting brake effectiveness to near-zero.
        const double weightOnWheels = std::max(0.5, std::min(1.0, nzcgs));
        const double fric = (muFric + std::fabs(0.3 * sinbet))
                          * weightOnWheels * GRAVITY * dt;
        double netAccel = vtDot * dt - fric;
        double newVt = k.vt + netAccel;
        if (newVt < 0.0) newVt = 0.0;
        state.vtDot    = netAccel / std::max(dt, 1e-6);  // effective accel incl. friction
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

    // Ground clamp: prevent the aircraft from going underground.
    // FreeFalcon has a full gear model with strut compression + ground
    // reaction forces. F4Flight's gear model tracks strut compression
    // but does NOT generate a ground reaction force — without this clamp,
    // the aircraft passes through z=0 and keeps going.
    // Only clamp when the aircraft is actually descending (zdot > 0 in NED
    // = moving toward ground). This prevents the clamp from firing on
    // aircraft that start on the ground at z=0.
    const double groundZ = state.gear.groundZ_ft;
    if (k.z > groundZ && k.zdot > 0.0) {
        k.z = groundZ;
        k.zdot = 0.0;  // kill descent rate (don't bounce)
        k.singam = 0.0;  // level the flight path
        k.cosgam = 1.0;
        k.gmma = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Top-level EOM update
// ---------------------------------------------------------------------------
void EquationsOfMotion::update(double dt, PilotInput const& input,
                               AircraftState& state) const {
    // NOTE: input IS used on the ground for nose-wheel steering (see below).
    // In the air, the FCS has already processed input into body rates.
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

    // --- Ground clamp (attitude) ---
    // When on the ground, clamp body attitude to prevent the quaternion
    // singularity at phi=±180° that occurs when roll rate is near zero.
    // FreeFalcon does this in AirframeClass ground handling: zero the body
    // rates, clamp roll to 0, and hold the heading steady. Without this,
    // tiny numerical perturbations in the quaternion produce 180° roll
    // flips on the ground, which cause the AI steering to see a flipped
    // heading and command full rudder — the "erratic spinning on the
    // ground" behavior.
    //
    // The clamp only applies when the aircraft is within 5 ft of the ground.
    // Above 5 ft AGL, even if gear.inAir hasn't transitioned yet, the aircraft
    // is effectively airborne and needs full attitude freedom to rotate and
    // climb. Without this altitude threshold, the clamp prevents the takeoff
    // rotation from exceeding 10° pitch, which stalls light fighters (F-5,
    // F-15C, MiG-21) that need 12-15° pitch to sustain a climb at low speed.
    const double altAGL_ground = state.gear.groundZ_ft - k.z;
    if (!state.gear.inAir && altAGL_ground < 5.0) {
        // Zero roll and yaw rates on the ground (prevents the quaternion
        // singularity at phi=±180°). DO NOT zero pitch rate (q) — the
        // FCS needs q for pitch damping during rotation and climbout.
        k.p = 0.0;
        k.r = 0.0;
        // Clamp roll to 0 (wings level on the ground)
        k.phi = 0.0;
        // Clamp pitch to a small nose-up attitude (for ground attitude)
        // rather than letting it drift. Real aircraft sit at ~0-5° pitch
        // on the ground. We allow up to 15° (takeoff rotation) then clamp.
        k.theta = std::max(-2.0 * DTR, std::min(15.0 * DTR, k.theta));

        // --- Nose-wheel steering ---
        // On the ground, the rudder (input.rstick) controls the nose wheel,
        // not the aerodynamic rudder. The FCS's aerodynamic roll/yaw control
        // is zeroed above (k.p = k.r = 0), so we apply the rudder command
        // directly to psi. This gives responsive ground steering without
        // depending on aerodynamic forces (which are negligible at taxi
        // speed). The steering rate is proportional to rstick and inversely
        // proportional to speed (more authority at low speed, less at high
        // speed — matching real nose-wheel steering).
        //
        // Max turn rate: 30°/s at full rudder at <30 kts, decreasing to
        // 5°/s above 100 kts (rudder pedal travel is limited at high speed
        // on real aircraft to prevent oversteering).
        const double steerRate = (k.vt < 50.0) ? 30.0
                              : (k.vt < 150.0) ? 30.0 * (150.0 - k.vt) / 100.0 + 5.0
                              : 5.0;
        k.psi += input.rstick * steerRate * DTR * dt;
        // Wrap psi to [-PI, PI]
        while (k.psi >  PI) k.psi -= 2.0 * PI;
        while (k.psi < -PI) k.psi += 2.0 * PI;

        // Rebuild the quaternion from the clamped Euler angles
        k.quat = quatFromEuler(k.psi, k.theta, k.phi);
    }

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
