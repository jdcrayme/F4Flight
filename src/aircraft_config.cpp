// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aircraft_config.cpp
//
// Implementation of Limiter::limit(), AircraftConfig::validate(), and
// ConfigValidationReport::format().

#include "f4flight/aircraft_config.h"
#include "f4flight/core/math.h"

#include <cmath>
#include <sstream>

namespace f4flight {

// Evaluate a limiter. Direct port of the legacy Limiter subclasses.
double Limiter::limit(double x) const noexcept {
    switch (type) {
        case LimiterType::Line: {
            // y = m x + b, then clamp the output range to [y1, y2] and clamp
            // the input range to [x1, x2].
            const double dx = x2 - x1;
            if (std::fabs(dx) < 1e-12) return y1;
            const double xc = f4flight::limit(x, std::min(x1, x2), std::max(x1, x2));
            const double t = (xc - x1) / dx;
            const double y = y1 + t * (y2 - y1);
            return f4flight::limit(y, std::min(y1, y2), std::max(y1, y2));
        }
        case LimiterType::Value:
            return x1;
        case LimiterType::Percent:
            return x * x1;
        case LimiterType::ThreePoint: {
            // Two segments: (x0,y0)-(x1,y1) and (x1,y1)-(x2,y2)
            if (x <= x1) {
                const double dx = x1 - x0;
                if (std::fabs(dx) < 1e-12) return y1;
                const double t = f4flight::limit((x - x0) / dx, 0.0, 1.0);
                return y0 + t * (y1 - y0);
            } else {
                const double dx = x2 - x1;
                if (std::fabs(dx) < 1e-12) return y1;
                const double t = f4flight::limit((x - x1) / dx, 0.0, 1.0);
                return y1 + t * (y2 - y1);
            }
        }
        case LimiterType::MinMax:
            return f4flight::limit(x, x1, x2);
    }
    return x;
}

// ---------------------------------------------------------------------------
// ConfigValidationReport::format()
// ---------------------------------------------------------------------------
std::string ConfigValidationReport::format() const {
    std::ostringstream os;
    for (const auto& i : issues) {
        os << (i.severity == Severity::Error ? "E: " : "W: ")
           << '[' << i.field << "] " << i.message << '\n';
    }
    return os.str();
}

// ---------------------------------------------------------------------------
// AircraftConfig::validate()
//
// Adds issues to the report for every problem found. Does NOT short-circuit
// on the first error -- the goal is to give the host a complete diagnostic in
// one pass so a malformed data file can be fixed without round-tripping.
// ---------------------------------------------------------------------------
namespace {

using Issue = ConfigValidationReport::Issue;
using Sev   = ConfigValidationReport::Severity;

inline bool isFinite(double x) noexcept { return std::isfinite(x); }

void checkFinite(ConfigValidationReport& r, const char* field, double x) {
    if (!isFinite(x)) {
        r.issues.push_back({Sev::Error, field,
                            "value is NaN or Inf"});
    }
}

void checkPositive(ConfigValidationReport& r, const char* field, double x,
                   double lo = 0.0) {
    if (!isFinite(x)) {
        r.issues.push_back({Sev::Error, field, "value is NaN or Inf"});
    } else if (x <= lo) {
        r.issues.push_back({Sev::Error, field,
                            "value must be > " + std::to_string(lo) +
                            " but is " + std::to_string(x)});
    }
}

void checkNonNegative(ConfigValidationReport& r, const char* field, double x) {
    if (!isFinite(x)) {
        r.issues.push_back({Sev::Error, field, "value is NaN or Inf"});
    } else if (x < 0.0) {
        r.issues.push_back({Sev::Error, field,
                            "value must be >= 0 but is " + std::to_string(x)});
    }
}

} // anonymous namespace

ConfigValidationReport AircraftConfig::validate() const {
    ConfigValidationReport r;

    // -----------------------------------------------------------------------
    // Aero tables
    // -----------------------------------------------------------------------
    if (aero.mach.empty() || aero.alpha_deg.empty() || aero.clift.empty()) {
        r.issues.push_back({Sev::Error, "aero",
                            "aero tables are empty -- not a valid aircraft data file"});
    } else {
        const std::size_t expected = aero.mach.size() * aero.alpha_deg.size();
        if (aero.clift.size() != expected) {
            r.issues.push_back({Sev::Error, "aero.clift",
                "size mismatch: expected " + std::to_string(expected) +
                " (= " + std::to_string(aero.mach.size()) + " mach x " +
                std::to_string(aero.alpha_deg.size()) + " alpha) but got " +
                std::to_string(aero.clift.size())});
        }
        if (aero.cdrag.size() != expected) {
            r.issues.push_back({Sev::Error, "aero.cdrag",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(aero.cdrag.size())});
        }
        if (aero.cy.size() != expected) {
            r.issues.push_back({Sev::Warning, "aero.cy",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(aero.cy.size()) +
                " (cy will be treated as zero where missing)"});
        }
        // Sanity-check the breakpoints: alpha should be strictly ascending
        // (the lookup clamps, so out-of-order isn't fatal, but it's a sign of
        // a corrupt data file).
        for (std::size_t i = 1; i < aero.mach.size(); ++i) {
            if (aero.mach[i] <= aero.mach[i-1]) {
                r.issues.push_back({Sev::Warning, "aero.mach",
                    "breakpoints are not strictly ascending at index " +
                    std::to_string(i)});
                break;
            }
        }
        for (std::size_t i = 1; i < aero.alpha_deg.size(); ++i) {
            if (aero.alpha_deg[i] <= aero.alpha_deg[i-1]) {
                r.issues.push_back({Sev::Warning, "aero.alpha_deg",
                    "breakpoints are not strictly ascending at index " +
                    std::to_string(i)});
                break;
            }
        }
        // Spot-check the CL table for NaN/Inf (full scan would be expensive
        // for big tables, so we sample).
        if (!aero.clift.empty()) {
            const std::size_t step = std::max<std::size_t>(1, aero.clift.size() / 32);
            for (std::size_t i = 0; i < aero.clift.size(); i += step) {
                checkFinite(r, "aero.clift", aero.clift[i]);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Engine tables
    // -----------------------------------------------------------------------
    if (engine.alt_ft.empty() || engine.mach.empty() ||
        engine.thrust_idle.empty() || engine.thrust_mil.empty()) {
        r.issues.push_back({Sev::Error, "engine",
                            "engine thrust tables are empty"});
    } else {
        const std::size_t expected = engine.alt_ft.size() * engine.mach.size();
        if (engine.thrust_idle.size() != expected) {
            r.issues.push_back({Sev::Error, "engine.thrust_idle",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(engine.thrust_idle.size())});
        }
        if (engine.thrust_mil.size() != expected) {
            r.issues.push_back({Sev::Error, "engine.thrust_mil",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(engine.thrust_mil.size())});
        }
        // AB table is optional; if present it must match.
        if (!engine.thrust_ab.empty() && engine.thrust_ab.size() != expected) {
            r.issues.push_back({Sev::Error, "engine.thrust_ab",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(engine.thrust_ab.size())});
        }
    }

    // -----------------------------------------------------------------------
    // Roll command table (optional -- empty means "use FCS defaults")
    // -----------------------------------------------------------------------
    if (!rollCmd.alpha_deg.empty() || !rollCmd.qbar.empty() ||
        !rollCmd.rollRate.empty()) {
        const std::size_t expected = rollCmd.alpha_deg.size() * rollCmd.qbar.size();
        if (rollCmd.rollRate.size() != expected) {
            r.issues.push_back({Sev::Error, "rollCmd.rollRate",
                "size mismatch: expected " + std::to_string(expected) +
                " but got " + std::to_string(rollCmd.rollRate.size())});
        }
    }

    // -----------------------------------------------------------------------
    // Geometry
    // -----------------------------------------------------------------------
    checkPositive(r, "geometry.emptyWeight_lbs", geometry.emptyWeight_lbs);
    checkPositive(r, "geometry.area_ft2",        geometry.area_ft2);
    checkPositive(r, "geometry.span_ft",         geometry.span_ft);
    checkNonNegative(r, "geometry.internalFuel_lbs", geometry.internalFuel_lbs);
    checkNonNegative(r, "geometry.length_ft",    geometry.length_ft);

    // AOA / beta limits
    if (geometry.aoaMax_deg <= geometry.aoaMin_deg) {
        r.issues.push_back({Sev::Error, "geometry.aoaMax_deg",
            "aoaMax (" + std::to_string(geometry.aoaMax_deg) +
            ") must be > aoaMin (" + std::to_string(geometry.aoaMin_deg) + ")"});
    }
    if (geometry.aoaMax_deg > 90.0 || geometry.aoaMax_deg < 1.0) {
        r.issues.push_back({Sev::Warning, "geometry.aoaMax_deg",
            "value " + std::to_string(geometry.aoaMax_deg) +
            " is outside the typical 1..90 deg range"});
    }
    if (geometry.betaMax_deg <= geometry.betaMin_deg) {
        r.issues.push_back({Sev::Error, "geometry.betaMax_deg",
            "betaMax must be > betaMin"});
    }

    // Performance envelope
    checkPositive(r, "geometry.maxGs",          geometry.maxGs);
    checkPositive(r, "geometry.maxRoll_deg",    geometry.maxRoll_deg);
    checkPositive(r, "geometry.minVcas_kts",    geometry.minVcas_kts);
    checkPositive(r, "geometry.maxVcas_kts",    geometry.maxVcas_kts);
    checkPositive(r, "geometry.cornerVcas_kts", geometry.cornerVcas_kts);
    if (geometry.maxVcas_kts <= geometry.minVcas_kts) {
        r.issues.push_back({Sev::Error, "geometry.maxVcas_kts",
            "maxVcas must be > minVcas"});
    }
    checkFinite(r, "geometry.thetaMax_rad", geometry.thetaMax_rad);

    // -----------------------------------------------------------------------
    // AuxAero -- only the few fields that the flight model actually reads.
    // -----------------------------------------------------------------------
    checkPositive(r, "aux.fuelFlowFactorNormal", aux.fuelFlowFactorNormal);
    checkPositive(r, "aux.normSpoolRate",        aux.normSpoolRate, 0.0);
    checkPositive(r, "aux.abSpoolRate",          aux.abSpoolRate,   0.0);
    if (aux.nEngines < 1) {
        r.issues.push_back({Sev::Warning, "aux.nEngines",
            "nEngines < 1 is unusual; defaulting to single-engine model"});
    }

    // -----------------------------------------------------------------------
    // Gear points
    // -----------------------------------------------------------------------
    for (std::size_t i = 0; i < geometry.gear.size(); ++i) {
        const auto& g = geometry.gear[i];
        const std::string field = "geometry.gear[" + std::to_string(i) + "]";
        checkFinite(r, (field + ".x").c_str(), g.x);
        checkFinite(r, (field + ".y").c_str(), g.y);
        checkFinite(r, (field + ".z").c_str(), g.z);
        if (g.range < 0.0) {
            r.issues.push_back({Sev::Warning, field + ".range",
                "negative strut range is unusual"});
        }
    }

    return r;
}

// ---------------------------------------------------------------------------
// deriveProfile — auto-derive a PerformanceProfile from the aircraft data.
//
// Uses TWR (thrust-to-weight ratio at MIL power, sea level), wing loading
// (empty weight / wing area), maxGs, maxVcas, and nEngines to categorize
// the aircraft and pick reasonable starting speeds.
//
// The categories and their default speeds are based on real-world aviation
// practice:
//   Interceptor: high TWR, high maxVcas — F-14, MiG-31. Cruise 450, climb 380.
//   Fighter:     high TWR, maxGs >= 7   — F-16, F-15, MiG-29. Cruise 380, climb 320.
//   Attack:      medium TWR, maxGs 4-7  — A-10. Cruise 280, climb 230.
//   Bomber:      low maxGs, heavy       — B-52, B-1. Cruise 300, climb 250.
//   Transport:   low maxGs, very heavy  — C-130. Cruise 250, climb 210.
//
// Within each category, speeds are adjusted based on TWR — higher TWR
// allows higher cruise speeds; lower TWR forces lower speeds. The cruise
// altitude is adjusted based on the aircraft's service ceiling (approximated
// from the engine thrust table altitude breakpoints).
// ---------------------------------------------------------------------------
void AircraftConfig::deriveProfile() {
    const double emptyWeight = geometry.emptyWeight_lbs;
    const double fuel = geometry.internalFuel_lbs;
    const double grossWeight = emptyWeight + fuel;
    const double area = geometry.area_ft2;
    const double maxGs = geometry.maxGs;
    const double maxVcas = geometry.maxVcas_kts;
    const double minVcas = geometry.minVcas_kts;
    const int nEngines = aux.nEngines > 0 ? aux.nEngines : 1;

    // Compute sea-level MIL thrust-to-weight ratio.
    // The thrust tables are per-engine; total thrust = table[0] * nEngines.
    double slMilThrust = 0.0;
    if (!engine.thrust_mil.empty() && !engine.mach.empty()) {
        slMilThrust = engine.thrust_mil[0] * nEngines; // mach=0, alt=0
    }
    const double twr = (grossWeight > 0.0) ? (slMilThrust / grossWeight) : 0.0;

    // Wing loading (psf)
    const double wingLoading = (area > 0.0) ? (grossWeight / area) : 0.0;

    // --- Categorize ---
    // The categorization uses maxGs, maxVcas, empty weight, and TWR.
    //
    // Attack: maxVcas <= 550 (slow CAS aircraft — A-10 at 500, Su-25).
    //   These are subsonic ground-attack aircraft regardless of maxGs.
    // Interceptor: maxVcas >= 900 AND maxGs >= 7 (MiG-31, F-14 at high
    //   speed). Most fighters (F-16 maxVcas=850, F-15=800) are NOT
    //   interceptors — they're multirole fighters.
    // Fighter: maxGs >= 7 (F-16, F-15, MiG-29, etc.).
    // Bomber: maxGs < 4, medium-heavy (B-52, B-1, Tu-22).
    // Transport: maxGs < 4, very heavy (C-130, C-5, An-124).
    // Trainer: everything else (light, moderate performance).
    std::string category;
    if (maxVcas <= 550.0 && maxGs >= 4.0) {
        category = "attack";
    } else if (maxGs >= 7.0 && maxVcas >= 900.0) {
        category = "interceptor";
    } else if (maxGs >= 7.0) {
        category = "fighter";
    } else if (maxGs >= 4.0 && emptyWeight < 50000.0) {
        category = "attack";
    } else if (maxGs < 4.0 && emptyWeight > 70000.0) {
        category = "transport";
    } else if (maxGs < 4.0) {
        category = "bomber";
    } else {
        category = "trainer";
    }

    // Override: very low TWR (< 0.2) forces transport-like speeds
    // regardless of maxGs (e.g., heavy aircraft with weak engines).
    if (twr < 0.2 && category != "transport") {
        category = "transport";
    }

    // --- Base speeds per category ---
    // Convention: climb < cruise < descent
    double cruiseSpeed, climbSpeed, descentSpeed, maxBank;
    if (category == "interceptor") {
        cruiseSpeed = 450; climbSpeed = 380; descentSpeed = 500; maxBank = 45;
    } else if (category == "fighter") {
        cruiseSpeed = 380; climbSpeed = 320; descentSpeed = 420; maxBank = 45;
    } else if (category == "attack") {
        cruiseSpeed = 280; climbSpeed = 230; descentSpeed = 320; maxBank = 30;
    } else if (category == "bomber") {
        cruiseSpeed = 300; climbSpeed = 250; descentSpeed = 340; maxBank = 30;
    } else if (category == "transport") {
        cruiseSpeed = 250; climbSpeed = 210; descentSpeed = 290; maxBank = 25;
    } else { // trainer
        cruiseSpeed = 320; climbSpeed = 270; descentSpeed = 360; maxBank = 40;
    }

    // --- Adjust speeds based on TWR ---
    // High TWR (> 0.6): can maintain higher cruise speeds
    // Low TWR (< 0.3): must fly slower, but keep the climb/cruise/descent
    // spread intact (don't flatten the schedule).
    if (twr > 0.0 && twr < 0.3) {
        // Low TWR — reduce all speeds by up to 15%, preserving the spread
        const double factor = 0.85 + (twr / 0.3) * 0.15; // 0.85 at TWR=0, 1.0 at TWR=0.3
        cruiseSpeed *= factor;
        climbSpeed *= factor;
        descentSpeed *= factor;
    } else if (twr > 0.7) {
        // High TWR — increase cruise and descent by up to 10%
        const double factor = 1.0 + std::min(0.10, (twr - 0.7) * 0.2);
        cruiseSpeed *= factor;
        descentSpeed *= factor;
        // Keep climb speed the same — higher TWR doesn't mean we should climb faster
    }

    // --- Clamp to aircraft limits ---
    // Cruise speed should be comfortably above minVcas (with 20% margin)
    const double minSafe = std::max(minVcas * 1.2, 150.0); // never below 150 kts
    if (cruiseSpeed < minSafe) cruiseSpeed = minSafe;
    if (climbSpeed < minSafe) climbSpeed = minSafe;
    if (cruiseSpeed > maxVcas * 0.85) cruiseSpeed = maxVcas * 0.85;
    if (descentSpeed > maxVcas * 0.95) descentSpeed = maxVcas * 0.95;

    // Ensure the climb < cruise < descent ordering is preserved after
    // all the adjustments. If clamping collapsed the spread, nudge them apart.
    if (climbSpeed >= cruiseSpeed) climbSpeed = cruiseSpeed * 0.85;
    if (descentSpeed <= cruiseSpeed) descentSpeed = cruiseSpeed * 1.10;

    // --- Cruise altitude ---
    // Pick a conservative cruise altitude based on category and TWR.
    // The previous approach (65% of the highest engine table altitude)
    // produced altitudes that were too high for the aircraft to maintain
    // speed at — the F-16 was assigned 45000 ft, where it can't hold
    // 450 kts CAS at MIL power with the current engine model.
    //
    // Real-world cruise altitudes:
    //   Fighters:    20000-30000 ft (cruise), up to 40000 for ferry
    //   Interceptors: 25000-35000 ft
    //   Attack:       15000-20000 ft
    //   Bombers:      25000-35000 ft
    //   Transports:   20000-30000 ft
    //
    // We pick the low end of these ranges to ensure the aircraft can
    // maintain speed. The tune_profile tool can adjust upward if the
    // aircraft handles it well.
    double cruiseAlt;
    if (category == "interceptor") {
        cruiseAlt = 25000.0;
    } else if (category == "fighter") {
        cruiseAlt = 20000.0;
    } else if (category == "attack") {
        cruiseAlt = 15000.0;
    } else if (category == "bomber") {
        cruiseAlt = 25000.0;
    } else if (category == "transport") {
        cruiseAlt = 20000.0;
    } else {
        cruiseAlt = 15000.0;
    }
    // Reduce for low-TWR aircraft (they struggle at altitude)
    if (twr < 0.3) {
        cruiseAlt = std::min(cruiseAlt, 15000.0);
    }
    // Clamp
    cruiseAlt = f4flight::limit(cruiseAlt, 5000.0, 35000.0);

    // --- Climb/descent altitudes ---
    const double climbAlt = cruiseAlt + 10000.0;
    const double descentAlt = std::max(3000.0, cruiseAlt - 10000.0);

    // --- Power settings ---
    // Most aircraft climb at MIL (1.0). High-TWR fighters can climb at
    // reduced power (0.9) to avoid overspeeding.
    double climbPower = 1.0;
    if (twr > 0.7 && category == "fighter") {
        climbPower = 0.95;
    }
    // Heavy/low-TWR aircraft may need full AB to climb. We don't model
    // this by default — AB climbs are inefficient and the test scenarios
    // don't require them. Leave climbPower at 1.0 (MIL).
    double descentPower = 0.05; // near idle

    // --- Assign ---
    profile.category = category;
    profile.cruiseSpeed_kts = cruiseSpeed;
    profile.climbSpeed_kts = climbSpeed;
    profile.climbMach = 0.80;
    profile.climbPower = climbPower;
    profile.descentSpeed_kts = descentSpeed;
    profile.descentMach = 0.80;
    profile.descentPower = descentPower;
    profile.cruiseAlt_ft = cruiseAlt;
    profile.climbAlt_ft = climbAlt;
    profile.descentAlt_ft = descentAlt;
    profile.maxBank_deg = maxBank;
    profile.levelBand_ft = 200.0;
    profile.tuned = false;
}

} // namespace f4flight
