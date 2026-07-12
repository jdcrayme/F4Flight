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
    state_.fcs.pitchAlphaLag.y_prev = state_.aero.alpha_deg;
    state_.fcs.pitchAlphaLag.u_prev = state_.aero.alpha_deg;
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
    state_.gear.muFric = GearModel::calcMuFric(false, false,
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
    // Body-axis load factors (G)
    const double& cosalp = state_.kin.cosalp;
    const double& sinalp = state_.kin.sinalp;
    // (cosgam unused)
    // (cosmu unused)

    // Load factor convention: NZ = 1.0 for level flight (1 G felt by the
    // pilot). This is the F-16 FLCS convention.
    //
    // The aerodynamic force along body Z is zaero (positive = down). Lift is
    // up (negative zaero). The gravity component along body Z is
    // +g * cos(mu) * cos(gamma) (also positive = down).
    //
    // The "felt" load factor is the total non-gravity acceleration divided
    // by g, plus 1 (for the gravity that the pilot feels at rest).
    //
    // NZ = -(zaero / g)   [lift contribution; -zaero = lift, positive up]
    // For level flight, lift = g, so NZ = -(g/g) = ... no.
    //
    // Actually the correct formula is:
    //   NZ = (lift_accel) / g
    // where lift_accel = -zsaero (the aerodynamic Z force, negated because
    // lift is positive up but Z is down).
    // For level flight: lift_accel = g (balances gravity), so NZ = 1.0.

    // Body axes
    const double nxb =  state_.aero.xaero / GRAVITY;
    const double nyb =  state_.aero.yaero / GRAVITY;
    const double nzb = -state_.aero.zaero / GRAVITY;  // -zaero = lift; NZ = 1 in level flight

    // Stability axes
    const double nxs =  state_.aero.xsaero / GRAVITY;
    const double nys =  state_.aero.ysaero / GRAVITY;
    const double nzs = -state_.aero.zsaero / GRAVITY;  // -zsaero = lift

    // Wind axes
    const double nxw =  state_.aero.xwaero / GRAVITY;
    const double nyw =  state_.aero.ywaero / GRAVITY;
    const double nzw = -state_.aero.zwaero / GRAVITY;

    state_.loads.nxcgb = nxb; state_.loads.nycgb = nyb; state_.loads.nzcgb = nzb;
    state_.loads.nxcgs = nxs; state_.loads.nycgs = nys; state_.loads.nzcgs = nzs;
    state_.loads.nxcgw = nxw; state_.loads.nycgw = nyw; state_.loads.nzcgw = nzw;

    // Include thrust contribution (thrust along body X, with alpha coupling)
    const double xprop = state_.engine.thrust / std::max(1e-6, state_.fuel.mass_slugs);
    state_.loads.nxcgb += xprop / GRAVITY * cosalp;
    state_.loads.nzcgb += xprop / GRAVITY * sinalp;  // thrust pitch-up at alpha
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

    // Add thrust to aero x-prop (wind axis)
    // NOTE: state.engine.thrust is ALREADY an acceleration (thrust_lbf / mass_slugs),
    // computed in EngineModel::update(). Do NOT divide by mass again.
    const double xprop = state_.engine.thrust;
    const double xsprop =  xprop * state_.kin.cosalp;
    const double xwprop =  xsprop * state_.kin.cosbet; // + ysprop*sinbet (ysprop=0)
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

    // Update gear state once per major frame
    updateGear(dt);

    // Sub-step at the minor-frame rate
    const double minorDt = dt / static_cast<double>(minorPerMajor_);
    for (int i = 0; i < minorPerMajor_; ++i) {
        minorStep(minorDt, input);
    }
}

bool FlightModel::trim() {
    // Simple iterative trim: nudge alpha and throttle toward 1-G level flight.
    // This is a placeholder; a real trim would solve the nonlinear system.
    constexpr int kMaxIter = 50;
    constexpr double kTol  = 0.05; // 0.05 G

    for (int iter = 0; iter < kMaxIter; ++iter) {
        // Compute current loads
        updateAtmosphere();
        const double alt = -state_.kin.z;
        aero_.update(state_.aero.alpha_deg, state_.aero.beta_deg,
                     state_.mach, state_.kin.vt, state_.qbar, state_.qsom,
                     state_.qovt, alt, state_.gear.groundZ_ft, state_.kin.z,
                     state_.vcas, 0.0, state_.aero);
        const double xprop = state_.engine.thrust / std::max(1e-6, state_.fuel.mass_slugs);
        const double nzb = state_.aero.zaero / GRAVITY - state_.kin.cosmu * state_.kin.cosgam
                           - xprop / GRAVITY * std::sin(state_.aero.alpha_deg * DTR);

        // NZ error
        const double nzErr = 1.0 - nzb;
        if (std::fabs(nzErr) < kTol) {
            return true;
        }
        // Nudge alpha (more alpha = more lift = more NZ)
        const double dAlpha = nzErr * 2.0;
        state_.aero.alpha_deg += dAlpha;
        state_.aero.alpha_deg = limit(state_.aero.alpha_deg,
                                      cfg_.geometry.aoaMin_deg,
                                      cfg_.geometry.aoaMax_deg);

        // Nudge throttle to maintain airspeed
        // (Very rough: assume thrust = drag for steady state)
        const double drag_lbs = state_.aero.drag * state_.fuel.mass_slugs;
        const double thrust_lbs = state_.engine.thrust;
        const double speedErr = drag_lbs - thrust_lbs;
        state_.engine.thrust += speedErr * 0.5;
        // Map back to a throttle command via the engine model (single update)
        engine_.update(0.1, alt, state_.mach, state_.kin.vt,
                       state_.fuel.mass_slugs, 0.5, 1.0, false, state_.engine);
    }
    return false;
}

} // namespace f4flight
