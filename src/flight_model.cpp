// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// flight_model.cpp
//
// Top-level orchestrator. Port of AirframeClass::Exec(), Init(), TrimModel(),
// Accelerometers().

#include "f4flight/flight_model.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"
#include "f4flight/core/trig.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace f4flight {

FlightModel::FlightModel()
    : aero_(), engine_(), fcs_(), gear_(), eom_() {}

void FlightModel::init(const AircraftConfig& cfg,
                       double initialAltitude_ft,
                       double initialVt_ftps,
                       double initialHeading_rad,
                       bool inAir) {
    cfg_ = cfg;

    // Validate the configuration -- if the aero tables are empty, the
    // flight model cannot run. This catches degenerate data files (like
    // vapor.dat) that are not real aircraft.
    if (cfg_.aero.mach.empty() || cfg_.aero.alpha_deg.empty() ||
        cfg_.aero.clift.empty()) {
        throw std::runtime_error(
            "FlightModel::init: aircraft config has empty aero tables -- "
            "not a valid aircraft data file");
    }

    // Re-construct subsystems with the new config pointers. Subsystems only
    // hold const pointers and Lookup2D (cheaply copyable), so default copy
    // assignment is fine.
    aero_   = Aerodynamics(&cfg_.aero, &cfg_.geometry, &cfg_.aux);
    engine_ = EngineModel(&cfg_.engine, &cfg_.aux);
    fcs_    = FlightControlSystem(&cfg_, &cfg_.geometry, &cfg_.aux);
    gear_   = GearModel(&cfg_.geometry, &cfg_.aux);
    eom_    = EquationsOfMotion(&cfg_.geometry, &cfg_.aux);

    // Reset state. Using the named reset() method instead of `state_ = {}`
    // so the intent is explicit (and so hosts that subclass / wrap
    // FlightModel have a single canonical entry point to override).
    state_.reset();

    // Position: altitude positive up means z = -altitude
    state_.kin.z = -initialAltitude_ft;
    state_.kin.vt = initialVt_ftps;
    state_.kin.psi = initialHeading_rad;
    state_.kin.sigma = initialHeading_rad;
    state_.kin.theta = 0.0;
    state_.kin.phi = 0.0;
    state_.kin.gmma = 0.0;
    state_.kin.mu = 0.0;
    state_.kin.quat = quatFromEuler(initialHeading_rad, 0.0, 0.0);
    state_.kin.dcm = dcmFromEuler(initialHeading_rad, 0.0, 0.0);

    // Set initial horizontal velocity along heading
    state_.kin.xdot = initialVt_ftps * std::cos(initialHeading_rad);
    state_.kin.ydot = initialVt_ftps * std::sin(initialHeading_rad);
    state_.kin.zdot = 0.0;

    // Fuel
    state_.fuel.emptyWeight_lbs = cfg_.geometry.emptyWeight_lbs;
    state_.fuel.fuel_lbs = cfg_.geometry.internalFuel_lbs;
    state_.fuel.weight_lbs = state_.fuel.emptyWeight_lbs + state_.fuel.fuel_lbs;
    state_.fuel.mass_slugs = state_.fuel.weight_lbs / GRAVITY;
    state_.fuel.loadingFraction = state_.fuel.weight_lbs / state_.fuel.emptyWeight_lbs;

    // Gear
    gear_.init(state_.gear);
    state_.gear.inAir = inAir;
    state_.gear.planted = !inAir;
    state_.aero.gearPos = inAir ? 0.0 : 1.0;

    // Engine
    state_.engine.rpm = 0.7;
    state_.engine.engLit = true;

    // Set up initial atmosphere
    updateAtmosphere();

    // Initial aero state -- alpha/beta zero
    state_.aero.alpha_deg = 0.0;
    state_.aero.beta_deg = 0.0;

    // Compute initial aero forces
    const double alt = -state_.kin.z;
    aero_.update(state_.aero.alpha_deg, state_.aero.beta_deg,
                 state_.mach, state_.kin.vt, state_.qbar, state_.qsom,
                 state_.qovt, alt, state_.gear.groundZ_ft, state_.kin.z,
                 state_.vcas, 0.0, state_.aero);

    // --- Trim: set a reasonable initial alpha for level flight.
    // We use a simple analytic estimate: CL = W / (q * S), then invert
    // the CL table to find the corresponding alpha. This is more stable
    // than the iterative approach.
    if (inAir) {
        // Compute the CL needed for 1 G level flight
        const double targetCl = GRAVITY / std::max(1e-6, state_.qsom);
        // Search the CL table for the alpha that gives this CL at the
        // current Mach. We do a simple linear scan of the alpha breakpoints
        // at the first Mach breakpoint (close enough for a starting point).
        if (!cfg_.aero.alpha_deg.empty() && !cfg_.aero.clift.empty()) {
            const std::size_t numAlpha = cfg_.aero.alpha_deg.size();
            (void)cfg_.aero.mach.size();
            // Use the first Mach row (mach=0) as a reference
            double bestAlpha = 3.0;  // default
            double bestErr = 1e9;
            for (std::size_t i = 0; i < numAlpha; ++i) {
                const double cl = cfg_.aero.clift[i];  // mach=0, alpha=i
                const double err = std::fabs(cl - targetCl);
                if (err < bestErr) {
                    bestErr = err;
                    bestAlpha = cfg_.aero.alpha_deg[i];
                }
            }
            // Clamp to a reasonable range for level flight
            state_.aero.alpha_deg = limit(bestAlpha, -2.0, 10.0);
        }
    }

    // Set initial engine state to MIL (rpm = 1.0) so it's not spooling up
    // from zero on the first frame.
    state_.engine.rpm = 1.0;
    state_.engine.rpmLag.y_prev = 1.0;
    state_.engine.rpmLag.u_prev = 1.0;
    state_.engine.rpmLagInitialized = true;

    // Initialize the FCS lag filters from the trim alpha so the first frame
    // doesn't reset alpha to zero.
    state_.fcs.pitchAlphaLag.reset(state_.aero.alpha_deg);
    state_.fcs.pitchRateLag.y_prev = 0.0;
    state_.fcs.pitchRateLag.u_prev = 0.0;
    state_.fcs.oldp03[0] = state_.aero.alpha_deg;
    state_.fcs.aoacmd = state_.aero.alpha_deg;

    // Set the initial pitch attitude (theta) to the trim alpha so that the
    // flight path angle (gamma = theta - alpha) starts at zero. Without this,
    // the aircraft starts with a descent rate even at 1 G.
    state_.kin.theta = state_.aero.alpha_deg * DTR;
    state_.kin.quat = quatFromEuler(state_.kin.psi, state_.kin.theta, state_.kin.phi);
    state_.kin.dcm = dcmFromEuler(state_.kin.psi, state_.kin.theta, state_.kin.phi);

    // Recompute every sin*/cos* field and the velocity-vector euler angles
    // (sigma, gmma, mu) via the shared helper. This used to be a partial
    // inline computation that left singam/cosgam/sinpsi/cospsi/etc. as their
    // default 0/1; calling the helper here makes the initial state fully
    // consistent (all 16 trig fields populated) without changing any math.
    recomputeKinematicTrig(state_.kin, state_.aero.alpha_deg, state_.aero.beta_deg);
}

void FlightModel::updateAtmosphere() {
    const double alt = -state_.kin.z;
    const auto a = computeAtmosphere(alt, state_.kin.vt, cfg_.geometry.area_ft2,
                                     state_.fuel.mass_slugs);
    state_.rho   = a.rho;
    state_.pa    = a.pa;
    state_.mach  = a.mach;
    state_.qbar  = a.qbar;
    state_.qsom  = a.qsom;
    state_.qovt  = a.qovt;
    state_.vcas  = a.vcas;
    state_.sound = a.sound;
}

void FlightModel::updateGear(double dt) {
    // Integrate the gear handle position
    state_.aero.gearPos = gear_.updateGearPos(state_.aero.gearPos,
                                              state_.gear.inAir ? -1.0 : 1.0, dt);
    // Update strut compression for visuals
    gear_.updateStrutCompression(state_.gear, state_.gear.groundZ_ft,
                                 state_.kin.z, state_.kin.vt, dt);
    // Recompute friction
    // NOTE: Round-2 audit found this was hardcoded to (false, false) — the
    // PilotInput.wheelBrakes / parkingBrake fields existed but were never
    // plumbed through. We now cache the input fields (see update()) so the
    // rollout can actually brake. Without this, RunLanding::Rollout sets
    // throttle=0 but the aircraft doesn't decelerate (no friction increase).
    state_.gear.muFric = GearModel::calcMuFric(lastInput_.wheelBrakes,
                                               lastInput_.parkingBrake,
                                               state_.gear.onObject,
                                               state_.gear.overRunway);
    // Recompute minHeight
    state_.gear.minHeight_ft = gear_.computeMinHeight(state_.gear);

    // Determine if any wheel is on the ground
    bool anyOnGround = false;
    for (auto const& w : state_.gear.wheels) {
        if (w.onGround) { anyOnGround = true; break; }
    }
    // Transition in-air <-> on-ground based on gear contact and lift
    if (state_.gear.inAir && anyOnGround) {
        // Touchdown
        state_.gear.inAir = false;
        state_.gear.planted = false;
    } else if (!state_.gear.inAir) {
        // Check for lift-off: lift > weight and some vertical velocity
        const double lift_lbs = state_.aero.lift * state_.fuel.mass_slugs;
        const double weight_lbs = state_.fuel.weight_lbs;
        if (lift_lbs > weight_lbs * 1.05 && state_.kin.zdot < -0.5) {
            state_.gear.inAir = true;
        }
    }
}

void FlightModel::accelerometers() {
    // Load factors (G) along each axis. Convention matches FreeFalcon
    // meters.cpp:51-78: NZ is the non-gravity acceleration along the axis
    // (positive Z is down, lift is up so it produces -zaero), normalized
    // by g. Level flight has lift = mg, so nzcgs = mg/mg = 1.0. Gravity is
    // NOT added -- it is not a "felt" acceleration.
    //
    // Thrust contributions (xprop / xsprop / zsprop / xwprop) are already
    // folded into the aero force sums in minorStep() (see Bug #1 fix above),
    // so we just consume xaero/xsaero/xwaero etc. directly.

    state_.loads.nxcgb =  state_.aero.xaero  / GRAVITY;
    state_.loads.nycgb =  state_.aero.yaero  / GRAVITY;
    state_.loads.nzcgb = -state_.aero.zaero  / GRAVITY;  // -zaero = lift

    state_.loads.nxcgs =  state_.aero.xsaero / GRAVITY;
    state_.loads.nycgs =  state_.aero.ysaero / GRAVITY;
    state_.loads.nzcgs = -state_.aero.zsaero / GRAVITY;  // FCS pitch feedback

    state_.loads.nxcgw =  state_.aero.xwaero / GRAVITY;
    state_.loads.nycgw =  state_.aero.ywaero / GRAVITY;
    state_.loads.nzcgw = -state_.aero.zwaero / GRAVITY;
}

void FlightModel::computeLoadFactors() { accelerometers(); }

void FlightModel::minorStep(double dt, const PilotInput& input) {
    // 1. Atmosphere
    updateAtmosphere();

    // 2. FCS (gains + pitch + roll + yaw + axial)
    const bool gearDown = state_.aero.gearPos > 0.5;
    fcs_.update(dt, state_.qbar, state_.qsom, state_.mach, state_.kin.vt,
                state_.vcas, state_.aero.alpha_deg, state_.aero.beta_deg,
                state_.kin.cosmu, state_.kin.cosgam, state_.kin.singam,
                state_.kin.costhe, state_.kin.cosphi, state_.kin.phi,
                state_.fuel.loadingFraction, state_.gear.inAir,
                state_.loads.nzcgs, state_.loads.nycgw,
                gearDown, input.refueling, gearDown,
                input, state_.fcs, state_.aero);

    // 3. Aerodynamics (recompute forces with new alpha/beta)
    const double alt = -state_.kin.z;
    aero_.update(state_.aero.alpha_deg, state_.aero.beta_deg,
                 state_.mach, state_.kin.vt, state_.qbar, state_.qsom,
                 state_.qovt, alt, state_.gear.groundZ_ft, state_.kin.z,
                 state_.vcas, input.pstick, state_.aero);

#ifdef F4FLIGHT_NAN_DEBUG
    if (std::isnan(state_.aero.alpha_deg) || std::isnan(state_.aero.zaero) ||
        std::isnan(state_.kin.vt) || std::isnan(state_.qbar)) {
        std::fprintf(stderr, "NaN after aero: alpha=%.3f zaero=%.3f vt=%.1f qbar=%.1f mach=%.3f vcas=%.1f\n",
            state_.aero.alpha_deg, state_.aero.zaero, state_.kin.vt, state_.qbar, state_.mach, state_.vcas);
        std::abort();
    }
#endif
    // 4. Engine
    const double ethrst = 1.0; // no thrust reverse for now
    engine_.update(dt, alt, state_.mach, state_.kin.vt,
                   state_.fuel.mass_slugs, input.throttle, ethrst,
                   state_.simplified, state_.engine);

    // Add thrust to aero force sums. Matches FreeFalcon meters.cpp:58-77 and
    // engine.cpp:780-808. state.engine.thrust is ALREADY an acceleration
    // (thrust_lbf / mass_slugs) -- do NOT divide by mass again.
    //
    //   xprop  = thrust                     (body X, no cosalp)
    //   zprop  = 0                          (body Z; non-vectored nozzle)
    //   xsprop = xprop * cosalp             (stability X)
    //   zsprop = -xprop * sinalp            (stability Z; negative = up at +alpha)
    //   xwprop = xsprop * cosbet            (wind X)
    //
    // Previously only xwaero was augmented and accelerometers() then added
    // xsprop/G to nxcgb and zsprop/G to nzcgb -- i.e. stability-axis
    // contributions applied to body-axis load factors, while nxcgs / nzcgs
    // (the FCS pitch feedback) got NO thrust contribution at all. At 15 deg
    // alpha / MIL this introduced a ~0.13 G error in nzcgs, causing the FCS
    // to over-command alpha and the aircraft to climb/crash in level flight.
    const double xprop  = state_.engine.thrust;
    const double xsprop =  xprop * state_.kin.cosalp;
    const double zsprop = -xprop * state_.kin.sinalp;
    const double xwprop =  xsprop * state_.kin.cosbet;
    state_.aero.xaero  += xprop;
    state_.aero.xsaero += xsprop;
    state_.aero.zsaero += zsprop;
    state_.aero.xwaero += xwprop;

    // 5. Accelerometers (load factors)
    accelerometers();

    // 6. Equations of motion
    eom_.update(dt, input, state_);

#ifdef F4FLIGHT_NAN_DEBUG
    if (std::isnan(state_.kin.z) || std::isnan(state_.kin.vt) ||
        std::isnan(state_.kin.theta) || std::isnan(state_.kin.phi)) {
        std::fprintf(stderr, "NaN after EOM: z=%.1f vt=%.1f theta=%.4f phi=%.4f alpha=%.3f pstick=%.3f throttle=%.3f qbar=%.1f\n",
            state_.kin.z, state_.kin.vt, state_.kin.theta, state_.kin.phi,
            state_.aero.alpha_deg, input.pstick, input.throttle, state_.qbar);
        std::abort();
    }
#endif

    // 7. Burn fuel
    const double burnRate_lbs_per_sec = state_.engine.fuelFlow / 3600.0;
    state_.fuel.fuel_lbs -= burnRate_lbs_per_sec * dt;
    if (state_.fuel.fuel_lbs < 0.0) state_.fuel.fuel_lbs = 0.0;
    state_.fuel.weight_lbs = state_.fuel.emptyWeight_lbs +
                             state_.fuel.fuel_lbs +
                             state_.fuel.externalFuel_lbs;
    state_.fuel.mass_slugs = state_.fuel.weight_lbs / GRAVITY;
    state_.fuel.loadingFraction = state_.fuel.weight_lbs / state_.fuel.emptyWeight_lbs;
}

void FlightModel::update(double dt, const PilotInput& input,
                         double groundZ_ft, const Vec3& groundNormal) {
    state_.gear.groundZ_ft = groundZ_ft;
    state_.gear.groundNormal = groundNormal;

    // Cache input so updateGear() can read wheelBrakes/parkingBrake.
    // (Round-2 audit fix — previously these fields were dead.)
    lastInput_ = input;

    // Map speed-brake handle (-1 retract .. +1 extend) to aero.dbrake (0..1).
    // Previously dbrake was never set from input — the FCS didn't drive it.
    // This is the second half of the Rec 7 fix: the digi brain can now
    // command speed brakes for descent / deceleration.
    //
    // NOTE: PilotInput.speedBrake defaults to 0.0 (mid-range), but in
    // flight 0.0 should mean RETRACTED (no drag), not half-extended. Map
    // -1..+1 → 0..1 so that:
    //   speedBrake = -1.0  →  dbrake = 0.0   (retracted, no drag — default)
    //   speedBrake =  0.0  →  dbrake = 0.5   (half extended)
    //   speedBrake = +1.0  →  dbrake = 1.0   (full extended)
    //
    // The digi brain's DigiState.speedBrakeCmd defaults to 0.0 (retracted)
    // and the brain maps it to PilotInput.speedBrake = -1.0 by default
    // (see digi_brain.cpp compute()). So in practice:
    //   - Aircraft in normal flight: dbrake = 0.0 (no drag)
    //   - Brain commands speedBrakeCmd = +1.0: dbrake = 1.0 (full drag)
    //
    // The mapping in compute() (digi_brain.cpp):
    //   out.speedBrake = state_.speedBrakeCmd;
    // is intentional — speedBrakeCmd already uses the PilotInput -1..+1
    // convention. We need to convert PilotInput.speedBrake to dbrake here.
    state_.aero.dbrake = limit((input.speedBrake + 1.0) * 0.5, 0.0, 1.0);

    // Update gear state once per major frame
    updateGear(dt);

    // Sub-step at the minor-frame rate
    const double minorDt = dt / static_cast<double>(minorPerMajor_);
    for (int i = 0; i < minorPerMajor_; ++i) {
        minorStep(minorDt, input);
    }
}

bool FlightModel::trim() {
    // Iterative trim for steady-state 1-G level flight. Matches the
    // FreeFalcon TrimModel() goal (trim.cpp:50-172): drive (nzcgs - 1) and
    // (drag - thrust) to zero simultaneously by adjusting alpha and throttle.
    //
    // Previously this function had three bugs that prevented convergence:
    //   (1) state.engine.thrust (already ft/s^2, i.e. lbf/slug) was divided
    //       by mass_slugs AGAIN -- units error.
    //   (2) zaero/G has the wrong sign: zaero = -lift, so for level flight
    //       zaero = -g, and -zaero/G = +1. The old code used +zaero/G = -1.
    //   (3) the "- cosmu*cosgam" gravity term does not belong in the "felt"
    //       load factor -- gravity is not felt.
    //
    // The correct stability-axis formula (matching accelerometers()) is:
    //     nzcgs = -(zsaero + zsprop) / g
    //           = -(-lift + (-xprop*sinalp)) / g
    //           = (lift + xprop*sinalp) / g
    // Level flight: lift = mg, xprop*sinalp ~ 0.05g at 15 deg alpha / MIL,
    // so nzcgs ~= 1.0 + small. We drive nzcgs -> 1.0.
    constexpr int    kMaxIter  = 80;
    constexpr double kTolNz    = 0.05;   // 0.05 G
    constexpr double kTolAx    = 0.5;    // 0.5 ft/s^2
    constexpr double kAlphaK   = 1.5;    // deg of alpha per G of error
    constexpr double kThrotK   = 0.01;   // throttle per (ft/s^2) of axial error

    // Start from MIL throttle (1.0). The previous version used state.engine.rpmCmd
    // which defaults to 0 and never gets a sensible initial value, so the
    // iteration started from idle and could not spool up in 50 iterations.
    double throttleCmd = 0.7;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        updateAtmosphere();
        const double alt = -state_.kin.z;
        aero_.update(state_.aero.alpha_deg, state_.aero.beta_deg,
                     state_.mach, state_.kin.vt, state_.qbar, state_.qsom,
                     state_.qovt, alt, state_.gear.groundZ_ft, state_.kin.z,
                     state_.vcas, 0.0, state_.aero);
        // Use a large dt (10 s) so the engine spool lag fully catches up
        // to the commanded throttle each iteration. Without this, the spool
        // filter (tau ~ 3-5 s) prevents RPM from tracking throttleCmd within
        // the iteration and trim never converges on the thrust balance.
        engine_.update(10.0, alt, state_.mach, state_.kin.vt,
                       state_.fuel.mass_slugs, throttleCmd, 1.0,
                       false, state_.engine);

        // Thrust is already an acceleration (ft/s^2). Do NOT divide by mass.
        const double xprop  = state_.engine.thrust;
        const double xsprop =  xprop * state_.kin.cosalp;
        const double zsprop = -xprop * state_.kin.sinalp;

        // Stability-axis load factor (matches accelerometers()).
        const double nzcgs = -(state_.aero.zsaero + zsprop) / GRAVITY;
        // Axial acceleration along stability X (drag minus thrust component).
        // xsaero = -drag (drag is positive, xsaero is negative).
        // xsprop = +xprop*cosalp (thrust forward).
        // ax_stab > 0 means accelerating (too much thrust).
        const double ax_stab = state_.aero.xsaero + xsprop;

        const double nzErr = 1.0 - nzcgs;       // >0 means need more lift
        const double axErr = ax_stab;            // >0 means too much thrust
        if (std::fabs(nzErr) < kTolNz && std::fabs(axErr) < kTolAx) {
            state_.engine.rpmCmd = throttleCmd;
            return true;
        }

        // Adjust alpha: more alpha -> more lift -> more nzcgs.
        state_.aero.alpha_deg += nzErr * kAlphaK;
        state_.aero.alpha_deg = limit(state_.aero.alpha_deg,
                                      cfg_.geometry.aoaMin_deg,
                                      cfg_.geometry.aoaMax_deg);

        // Adjust throttle: positive axErr (too much thrust) -> reduce.
        throttleCmd -= axErr * kThrotK;
        throttleCmd = limit(throttleCmd, 0.0, 1.5);
    }
    state_.engine.rpmCmd = throttleCmd;

    // Final pass: update aero + engine + augment aero state with thrust
    // contributions (matching what minorStep does), so a subsequent call to
    // computeLoadFactors() / accelerometers() returns a consistent nzcgs.
    // Without this, the test "TrimProducesOneGNormalLoadFactor" would see
    // state_.loads.nzcgs = 0 (the unaugmented aero forces give nzcgs = lift/g,
    // but the FCS pitch feedback expects the thrust-augmented value).
    updateAtmosphere();
    const double alt = -state_.kin.z;
    aero_.update(state_.aero.alpha_deg, state_.aero.beta_deg,
                 state_.mach, state_.kin.vt, state_.qbar, state_.qsom,
                 state_.qovt, alt, state_.gear.groundZ_ft, state_.kin.z,
                 state_.vcas, 0.0, state_.aero);
    engine_.update(10.0, alt, state_.mach, state_.kin.vt,
                   state_.fuel.mass_slugs, throttleCmd, 1.0,
                   false, state_.engine);
    const double xprop  = state_.engine.thrust;
    const double xsprop =  xprop * state_.kin.cosalp;
    const double zsprop = -xprop * state_.kin.sinalp;
    const double xwprop =  xsprop * state_.kin.cosbet;
    state_.aero.xaero  += xprop;
    state_.aero.xsaero += xsprop;
    state_.aero.zsaero += zsprop;
    state_.aero.xwaero += xwprop;
    accelerometers();
    return false;
}

} // namespace f4flight
