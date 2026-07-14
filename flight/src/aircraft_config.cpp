// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aircraft_config.cpp
//
// Implementation of Limiter::limit(), AircraftConfig::validate(), and
// ConfigValidationReport::format().

#include "f4flight/flight/aircraft_config.h"
#include "f4flight/flight/core/math.h"

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

} // namespace f4flight
