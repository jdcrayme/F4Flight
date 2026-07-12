// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// engine.cpp
//
// Engine model implementation. Port of engine.cpp.

#include "f4flight/engine.h"
#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

EngineModel::EngineModel(const EngineTable* table, const AuxAero* aux)
    : table_(table), aux_(aux) {
    if (table_ && !table_->alt_ft.empty() && !table_->mach.empty() &&
        !table_->thrust_mil.empty()) {
        thrustIdle_ = table_->makeThrustLookup(0);
        thrustMil_  = table_->makeThrustLookup(1);
        thrustAb_   = table_->makeThrustLookup(2);
        hasFuelFlowTables_ = table_->hasFuelFlow();
        if (hasFuelFlowTables_) {
            ffIdle_ = Lookup2D(table_->alt_ft, table_->mach, table_->fuelflow_idle);
            ffMil_  = Lookup2D(table_->alt_ft, table_->mach, table_->fuelflow_mil);
            ffAb_   = Lookup2D(table_->alt_ft, table_->mach, table_->fuelflow_ab);
        }
    }
}

// Per-engine-type RPM schedule (PW-100/220/229, GE-110/129). Port of
// engine.cpp:2686-2798. Only the F-16-relevant bits are preserved; other
// types fall through with no modification.
double EngineModel::engineRpmMods(double rpmCmd, double alt_ft, double mach,
                                  double vcas) const noexcept {
    if (!aux_) return rpmCmd;

    double cmd = rpmCmd;

    switch (aux_->typeEngine) {
        case 1: // PW-100
        case 2: // PW-220
            // Idle rises with mach toward MIL between M=0.84 and M=1.4
            if (mach < 0.84) {
                // baseline
            } else if (mach <= 1.4) {
                cmd = std::max(cmd, mach / 1.4);
            } else {
                cmd = std::max(cmd, 0.99);
            }
            // Idle rises with altitude (stall margin)
            if (alt_ft > 10000.0) {
                cmd = std::max(cmd, (alt_ft / 10000.0) / 30.0 + 0.7);
            }
            // AB zone limits (above 35 kft)
            if (alt_ft >= 35000.0 && alt_ft <= 45000.0 && mach >= 0.4 && mach <= 0.8) {
                cmd = std::min(cmd, 1.025);
            } else if (alt_ft > 45000.0 && alt_ft <= 55000.0 && mach >= 0.4 && mach <= 0.95) {
                cmd = std::min(cmd, 1.01);
            } else if (alt_ft > 55000.0 || mach <= 0.4) {
                cmd = std::min(cmd, 0.99);
            }
            break;

        case 3: // PW-229
        case 4: // GE-110
        case 5: // GE-129
            // Reduced Speed Excursion
            if (mach > 0.55 && mach < 1.1) {
                cmd = std::max(cmd, 0.79);
            } else if (mach >= 1.1 && mach <= 1.4) {
                cmd = std::max(cmd, mach / 1.4);
            }
            // AB no-light zone
            if (alt_ft > 50000.0 && vcas < 250.0) {
                cmd = std::min(cmd, 0.99);
            }
            break;
        default:
            break;
    }

    return cmd;
}

void EngineModel::update(double dt,
                         double alt_ft,
                         double mach,
                         double vt_ftps,
                         double mass_slugs,
                         double throttle,
                         double ethrst,
                         bool   simplified,
                         EngineState& state) const {
    if (!table_ || !aux_ || mass_slugs <= 1e-6 ||
        table_->alt_ft.empty() || table_->mach.empty() ||
        table_->thrust_mil.empty()) {
        // No engine data (e.g., SR-71's J58 combined-cycle engine isn't
        // representable in the simple thrust-table format). Produce zero
        // thrust rather than crashing on empty lookups.
        state.thrust = 0.0;
        state.fuelFlow = 0.0;
        return;
    }

    // On the first call, synchronise the lag-filter state with the current
    // rpm so we don't reset the engine to zero.
    if (!state.rpmLagInitialized && state.rpm > 0.0) {
        state.rpmLag.y_prev = state.rpm;
        state.rpmLag.u_prev = state.rpm;
        state.rpmLagInitialized = true;
    }

    const bool hasAB = table_->hasAB();
    double pwrlev = throttle;
    pwrlev = limit(pwrlev, 0.0, hasAB ? 1.5 : 1.0);

    // Spool rate baseline + altitude/mach bias
    double spoolrate = aux_->normSpoolRate;
    const double spoolAltRate = (-alt_ft / 25000.0) - (mach / 2.0);
    spoolrate += spoolAltRate;

    double thrtb1 = 0.0;
    double rpmCmd = 0.0;

    if (state.rpm < 0.68 && state.engLit) {
        // Below idle -- lightup
        rpmCmd = 0.7;
        spoolrate = aux_->lightupSpoolRate;
        thrtb1 = 0.0;
    } else if (pwrlev <= 1.0) {
        // MIL or below
        const double th1 = thrustIdle_(alt_ft, mach);
        const double th2 = thrustMil_(alt_ft, mach);
        thrtb1 = ((th2 - th1) * pwrlev + th1) / mass_slugs;
        rpmCmd = 0.7 + 0.3 * pwrlev;
    } else if (hasAB) {
        // Afterburner
        const double th1 = thrustMil_(alt_ft, mach);
        const double th2 = thrustAb_(alt_ft, mach);
        thrtb1 = (2.0 * (th2 - th1) * (pwrlev - 1.0) + th1) / mass_slugs;
        rpmCmd = 1.0 + 0.06 * (pwrlev - 1.0);
    } else {
        thrtb1 = thrustMil_(alt_ft, mach) / mass_slugs;
        rpmCmd = 0.7 + 0.3 * pwrlev;
    }

    // Per-engine-type RPM schedule
    const double vcas_kts = vt_ftps * FTPSEC_TO_KNOTS;
    rpmCmd = engineRpmMods(rpmCmd, alt_ft, mach, vcas_kts);

    // Spool dynamics (first-order lag)
    state.rpmCmd = rpmCmd;
    state.rpm = state.rpmLag.step(rpmCmd, spoolrate, dt);

    // AB lit detection
    state.aburnLit = (pwrlev > 1.0) && (state.rpm > 0.95) && hasAB;
    if (!state.engLit) {
        state.rpm = state.rpmLag.step(0.0, aux_->flameoutSpoolRate, dt);
        state.aburnLit = false;
    }

    // Thrust scaling.
    //
    // The thrust tables in the .dat file are PER SINGLE ENGINE (this
    // matches the FreeFalcon convention — engine.cpp runs the engine
    // model once per engine and sums the results). For multi-engine
    // aircraft (F-15, F-18, F-14: nEngines=2; B-52: nEngines=4; etc.),
    // we multiply the single-engine thrust by nEngines to get the
    // combined thrust. Without this, multi-engine fighters have ~half
    // the thrust they should (F-15 TWR drops from ~0.6 to ~0.3) and
    // can't maintain altitude/speed in climbs.
    //
    // This is a simplification — it assumes all engines are at the same
    // throttle and ignores asymmetric-thrust yaw coupling. The FreeFalcon
    // code models each engine independently with its own spool dynamics;
    // a future enhancement could do the same here. For now, multiplying
    // by nEngines gives the correct steady-state thrust.
    const double nEngines = (aux_->nEngines > 0) ? static_cast<double>(aux_->nEngines) : 1.0;
    const double thrtab = thrtb1 * table_->thrustFactor;
    state.thrust = thrtab * ethrst * nEngines;

    // --- Fuel flow ---
    // FreeFalcon multiplies the per-engine fuel flow by nEngines
    // (engine.cpp:568: `fuelFlowSS *= auxaeroData->nEngines`).
    double fuelFlowSS = 0.0;
    if (hasFuelFlowTables_) {
        double ff1, ff2;
        if (state.aburnLit) {
            ff1 = ffMil_(alt_ft, mach);
            ff2 = ffAb_(alt_ft, mach);
            fuelFlowSS = 2.0 * (ff2 - ff1) * (pwrlev - 1.0) + ff1;
        } else {
            ff1 = ffIdle_(alt_ft, mach);
            ff2 = ffMil_(alt_ft, mach);
            fuelFlowSS = (ff2 - ff1) * pwrlev + ff1;
        }
    } else {
        // Legacy: fuel flow = factor * thrust * mass.
        // state.thrust already includes the nEngines multiplier (applied
        // above), so this path produces the combined fuel flow directly.
        const double factor = state.aburnLit ? aux_->fuelFlowFactorAb : aux_->fuelFlowFactorNormal;
        fuelFlowSS = factor * state.thrust * mass_slugs;
    }

    if (simplified) fuelFlowSS *= 0.75;
    // The table-based fuel flow is per-engine; multiply by nEngines to
    // get the combined fuel flow (matches FreeFalcon engine.cpp:568).
    // The legacy path is already correct (uses the nEngines-scaled thrust).
    if (hasFuelFlowTables_) fuelFlowSS *= nEngines;
    if (fuelFlowSS < aux_->minFuelFlow) fuelFlowSS = aux_->minFuelFlow;

    // 10-frame smoothing (1 Hz one-pole)
    state.fuelFlow += (fuelFlowSS - state.fuelFlow) / 10.0;
    if (state.fuelFlow < aux_->minFuelFlow) state.fuelFlow = aux_->minFuelFlow;
    lastFuelFlow_ = state.fuelFlow;

    // --- FTIT (Forward Turbine Inlet Temperature), normalized 0..10 ---
    double ftitCmd = 0.0;
    if (state.rpm < 0.7) {
        ftitCmd = 5.1 * (state.rpm / 0.7);
    } else if (state.rpm < 0.9) {
        ftitCmd = 5.1 + (state.rpm - 0.7) / 0.2 * 1.0;
    } else if (state.rpm < 1.0) {
        ftitCmd = 6.1 + (state.rpm - 0.9) / 0.1 * 1.5;
    } else {
        ftitCmd = 7.6 + (state.rpm - 1.0) / 0.03 * 0.1;
    }
    ftitCmd = limit(ftitCmd, 0.0, 10.0);
    state.ftit += (ftitCmd - state.ftit) * (dt / (dt + 0.7)); // 0.7s lag
}

void EngineModel::bodyForces(double thrust_accel,
                             double sinAlpha,
                             double cosAlpha,
                             double nozzlePos,
                             double& xprop,
                             double& yprop,
                             double& zprop,
                             double& xsprop,
                             double& zsprop) {
    if (nozzlePos <= 1e-6) {
        // Straight back
        xprop = thrust_accel;
        yprop = 0.0;
        zprop = 0.0;
    } else {
        // Vectored (Harrier-style)
        const double noz_rad = nozzlePos * DTR;
        const double cosN = std::cos(noz_rad);
        const double sinN = std::sin(noz_rad);
        xprop = thrust_accel * cosN;
        zprop = -thrust_accel * sinN;
        yprop = 0.0;
    }
    xsprop =  xprop * cosAlpha;
    zsprop = -xprop * sinAlpha + zprop * cosAlpha;
}

} // namespace f4flight
