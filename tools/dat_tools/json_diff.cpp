// json_diff - Diff two JSON aircraft files field-by-field.
//
// Loads both files into AircraftConfig structs and compares every field. The
// comparison is structural (not textual), so whitespace/key-order differences
// in the JSON source are ignored. Output is a unified-diff-style report
// listing every field that differs, with the old and new values.
//
// Exit codes:
//   0  files are equivalent (no differences)
//   1  files differ
//   2  could not load one or both files
//
// Usage:
//   json_diff <old.json> <new.json>
//   json_diff <old.json> <new.json> --summary     # only print counts, not details
//   json_diff <old.json> <new.json> --threshold 1e-9   # numeric tolerance (default 1e-12)

#include "f4flight/flight/json_io.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Options {
    bool summary_only = false;
    double tolerance = 1e-12;
};

// Compare two doubles with tolerance. Returns true if "equal".
bool eq(double a, double b, double tol) {
    if (a == b) return true;
    return std::fabs(a - b) <= tol;
}

void diffDouble(const std::string& name, double a, double b, const Options& opts,
                std::vector<std::string>& diffs) {
    if (!eq(a, b, opts.tolerance)) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %-40s  %.12g  ->  %.12g", name.c_str(), a, b);
        diffs.push_back(buf);
    }
}

void diffInt(const std::string& name, int a, int b, std::vector<std::string>& diffs) {
    if (a != b) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %-40s  %d  ->  %d", name.c_str(), a, b);
        diffs.push_back(buf);
    }
}

void diffBool(const std::string& name, bool a, bool b, std::vector<std::string>& diffs) {
    if (a != b) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %-40s  %s  ->  %s",
                      name.c_str(), a ? "true" : "false", b ? "true" : "false");
        diffs.push_back(buf);
    }
}

void diffString(const std::string& name, const std::string& a, const std::string& b,
                std::vector<std::string>& diffs) {
    if (a != b) {
        diffs.push_back("  " + name);
        diffs.push_back("    - " + a);
        diffs.push_back("    + " + b);
    }
}

void diffDoubleArray(const std::string& name,
                     const std::vector<double>& a, const std::vector<double>& b,
                     const Options& opts, std::vector<std::string>& diffs) {
    if (a.size() != b.size()) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %-40s  size %zu  ->  %zu", name.c_str(), a.size(), b.size());
        diffs.push_back(buf);
        return;
    }
    bool any = false;
    std::string detail = "  " + name + "  [";
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!eq(a[i], b[i], opts.tolerance)) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "    [%zu]  %.12g  ->  %.12g", i, a[i], b[i]);
            detail += "\n" + std::string(buf);
            any = true;
        }
    }
    if (any) diffs.push_back(detail);
}

void diffStringMap(const std::string& name,
                   const std::map<std::string, std::string>& a,
                   const std::map<std::string, std::string>& b,
                   std::vector<std::string>& diffs) {
    // Find keys only in a, only in b, and in both with different values.
    std::vector<std::string> only_a, only_b, value_diff;
    auto ia = a.begin(); auto ib = b.begin();
    while (ia != a.end() && ib != b.end()) {
        if (ia->first < ib->first) { only_a.push_back(ia->first); ++ia; }
        else if (ib->first < ia->first) { only_b.push_back(ib->first); ++ib; }
        else {
            if (ia->second != ib->second) value_diff.push_back(ia->first);
            ++ia; ++ib;
        }
    }
    while (ia != a.end()) { only_a.push_back(ia->first); ++ia; }
    while (ib != b.end()) { only_b.push_back(ib->first); ++ib; }

    if (!only_a.empty() || !only_b.empty() || !value_diff.empty()) {
        diffs.push_back("  " + name + "  (map)");
        for (const auto& k : only_a) diffs.push_back("    - " + k + " = " + a.at(k));
        for (const auto& k : only_b) diffs.push_back("    + " + k + " = " + b.at(k));
        for (const auto& k : value_diff) {
            diffs.push_back("    ~ " + k);
            diffs.push_back("        - " + a.at(k));
            diffs.push_back("        + " + b.at(k));
        }
    }
}

void diffStringVec(const std::string& name,
                   const std::vector<std::string>& a,
                   const std::vector<std::string>& b,
                   std::vector<std::string>& diffs) {
    if (a != b) {
        diffs.push_back("  " + name + "  (vector<string>)");
        for (const auto& s : a) diffs.push_back("    - " + s);
        for (const auto& s : b) diffs.push_back("    + " + s);
    }
}

void diffLimiter(const std::string& name,
                 const f4flight::Limiter& a, const f4flight::Limiter& b,
                 std::vector<std::string>& diffs) {
    if (a.type != b.type) {
        diffs.push_back("  " + name + ".type  " +
                        std::to_string(static_cast<int>(a.type)) + " -> " +
                        std::to_string(static_cast<int>(b.type)));
        return;
    }
    const char* f[] = {"x0", "y0", "x1", "y1", "x2", "y2"};
    double va[] = {a.x0, a.y0, a.x1, a.y1, a.x2, a.y2};
    double vb[] = {b.x0, b.y0, b.x1, b.y1, b.x2, b.y2};
    for (int i = 0; i < 6; ++i) {
        if (va[i] != vb[i]) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "  %s.%s  %.12g  ->  %.12g",
                          name.c_str(), f[i], va[i], vb[i]);
            diffs.push_back(buf);
        }
    }
}

void diffGear(const std::string& name,
              const std::vector<f4flight::GearPoint>& a,
              const std::vector<f4flight::GearPoint>& b,
              std::vector<std::string>& diffs) {
    if (a.size() != b.size()) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %-40s  size %zu  ->  %zu",
                      name.c_str(), a.size(), b.size());
        diffs.push_back(buf);
        return;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!eq(a[i].x, b[i].x, 1e-12) || !eq(a[i].y, b[i].y, 1e-12) ||
            !eq(a[i].z, b[i].z, 1e-12) || !eq(a[i].range, b[i].range, 1e-12)) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "  %s[%zu]  (%.6g,%.6g,%.6g,%.6g)  ->  (%.6g,%.6g,%.6g,%.6g)",
                name.c_str(), i, a[i].x, a[i].y, a[i].z, a[i].range,
                b[i].x, b[i].y, b[i].z, b[i].range);
            diffs.push_back(buf);
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "Usage: %s <old.json> <new.json> [--summary] [--threshold T]\n", argv[0]);
        return 2;
    }

    const std::string pathA = argv[1];
    const std::string pathB = argv[2];

    Options opts;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--summary") opts.summary_only = true;
        else if (a == "--threshold" && i + 1 < argc) {
            opts.tolerance = std::stod(argv[++i]);
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
            return 2;
        }
    }

    f4flight::AircraftConfig a, b;
    auto ioA = f4flight::json::readFile(pathA, a);
    if (!ioA.ok) {
        std::fprintf(stderr, "ERROR: Could not load %s\n", pathA.c_str());
        for (auto const& e : ioA.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 2;
    }
    auto ioB = f4flight::json::readFile(pathB, b);
    if (!ioB.ok) {
        std::fprintf(stderr, "ERROR: Could not load %s\n", pathB.c_str());
        for (auto const& e : ioB.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 2;
    }

    std::vector<std::string> diffs;

    diffString("name", a.name, b.name, diffs);
    diffString("description", a.GetDescription, b.GetDescription, diffs);
    diffString("sourceTitle", a.sourceTitle, b.sourceTitle, diffs);
    diffString("sourceAuthor", a.sourceAuthor, b.sourceAuthor, diffs);
    diffString("sourceRevision", a.sourceRevision, b.sourceRevision, diffs);
    diffString("sourceFile", a.sourceFile, b.sourceFile, diffs);
    diffStringVec("aeroOptions", a.aeroOptions, b.aeroOptions, diffs);
    diffStringVec("engineOptions", a.engineOptions, b.engineOptions, diffs);
    diffStringMap("rawAuxAeroData", a.rawAuxAeroData, b.rawAuxAeroData, diffs);

    // Geometry
    diffDouble("geometry.emptyWeight_lbs", a.geometry.emptyWeight_lbs, b.geometry.emptyWeight_lbs, opts, diffs);
    diffDouble("geometry.area_ft2", a.geometry.area_ft2, b.geometry.area_ft2, opts, diffs);
    diffDouble("geometry.internalFuel_lbs", a.geometry.internalFuel_lbs, b.geometry.internalFuel_lbs, opts, diffs);
    diffDouble("geometry.maxFuel_lbs", a.geometry.maxFuel_lbs, b.geometry.maxFuel_lbs, opts, diffs);
    diffDouble("geometry.aoaMax_deg", a.geometry.aoaMax_deg, b.geometry.aoaMax_deg, opts, diffs);
    diffDouble("geometry.aoaMin_deg", a.geometry.aoaMin_deg, b.geometry.aoaMin_deg, opts, diffs);
    diffDouble("geometry.betaMax_deg", a.geometry.betaMax_deg, b.geometry.betaMax_deg, opts, diffs);
    diffDouble("geometry.betaMin_deg", a.geometry.betaMin_deg, b.geometry.betaMin_deg, opts, diffs);
    diffDouble("geometry.maxGs", a.geometry.maxGs, b.geometry.maxGs, opts, diffs);
    diffDouble("geometry.maxRoll_deg", a.geometry.maxRoll_deg, b.geometry.maxRoll_deg, opts, diffs);
    diffDouble("geometry.minVcas_kts", a.geometry.minVcas_kts, b.geometry.minVcas_kts, opts, diffs);
    diffDouble("geometry.maxVcas_kts", a.geometry.maxVcas_kts, b.geometry.maxVcas_kts, opts, diffs);
    diffDouble("geometry.cornerVcas_kts", a.geometry.cornerVcas_kts, b.geometry.cornerVcas_kts, opts, diffs);
    diffDouble("geometry.thetaMax_rad", a.geometry.thetaMax_rad, b.geometry.thetaMax_rad, opts, diffs);
    diffDouble("geometry.cgLoc_ft", a.geometry.cgLoc_ft, b.geometry.cgLoc_ft, opts, diffs);
    diffDouble("geometry.length_ft", a.geometry.length_ft, b.geometry.length_ft, opts, diffs);
    diffDouble("geometry.span_ft", a.geometry.span_ft, b.geometry.span_ft, opts, diffs);
    diffDouble("geometry.fusRadius_ft", a.geometry.fusRadius_ft, b.geometry.fusRadius_ft, opts, diffs);
    diffDouble("geometry.tailHt_ft", a.geometry.tailHt_ft, b.geometry.tailHt_ft, opts, diffs);
    diffGear("geometry.gear", a.geometry.gear, b.geometry.gear, diffs);

    // Aux (typed subset)
    #define D_AUX(field) diffDouble("aux." #field, a.aux.field, b.aux.field, opts, diffs)
    D_AUX(fuelFlowFactorNormal); D_AUX(fuelFlowFactorAb); D_AUX(minFuelFlow);
    D_AUX(normSpoolRate); D_AUX(abSpoolRate); D_AUX(jfsSpoolUpRate);
    D_AUX(jfsSpoolUpLimit); D_AUX(lightupSpoolRate); D_AUX(flameoutSpoolRate);
    D_AUX(mainGenRpm); D_AUX(stbyGenRpm); D_AUX(epuBurnTime);
    D_AUX(tefMaxAngle); D_AUX(lefMaxAngle); D_AUX(tefRate); D_AUX(lefRate);
    D_AUX(tefTakeOff); D_AUX(lefGround); D_AUX(lefMaxMach);
    D_AUX(rudderMaxAngle); D_AUX(aileronMaxAngle); D_AUX(airbrakeMaxAngle);
    D_AUX(CLtefFactor); D_AUX(CDtefFactor); D_AUX(CDlefFactor);
    D_AUX(CDSPDBFactor); D_AUX(CDLDGFactor); D_AUX(dragChuteCd); D_AUX(area2Span);
    D_AUX(rollMomentum); D_AUX(pitchMomentum); D_AUX(yawMomentum);
    D_AUX(pitchElasticity); D_AUX(sinkRate); D_AUX(gearPitchFactor);
    D_AUX(rollGearGain); D_AUX(yawGearGain); D_AUX(pitchGearGain);
    D_AUX(landingAOA); D_AUX(rollCouple);
    #undef D_AUX
    diffBool("aux.hasLef", a.aux.hasLef, b.aux.hasLef, diffs);
    diffBool("aux.hasTef", a.aux.hasTef, b.aux.hasTef, diffs);
    diffBool("aux.elevatorRolls", a.aux.elevatorRolls, b.aux.elevatorRolls, diffs);
    diffInt("aux.nEngines", a.aux.nEngines, b.aux.nEngines, diffs);
    diffInt("aux.typeEngine", a.aux.typeEngine, b.aux.typeEngine, diffs);

    // Aero tables
    diffDoubleArray("aero.mach", a.aero.mach, b.aero.mach, opts, diffs);
    diffDoubleArray("aero.alpha_deg", a.aero.alpha_deg, b.aero.alpha_deg, opts, diffs);
    diffDoubleArray("aero.clift", a.aero.clift, b.aero.clift, opts, diffs);
    diffDoubleArray("aero.cdrag", a.aero.cdrag, b.aero.cdrag, opts, diffs);
    diffDoubleArray("aero.cy", a.aero.cy, b.aero.cy, opts, diffs);
    diffDouble("aero.clFactor", a.aero.clFactor, b.aero.clFactor, opts, diffs);
    diffDouble("aero.cdFactor", a.aero.cdFactor, b.aero.cdFactor, opts, diffs);
    diffDouble("aero.cyFactor", a.aero.cyFactor, b.aero.cyFactor, opts, diffs);

    // Engine
    diffDouble("engine.thrustFactor", a.engine.thrustFactor, b.engine.thrustFactor, opts, diffs);
    diffDouble("engine.fuelFlowFactor", a.engine.fuelFlowFactor, b.engine.fuelFlowFactor, opts, diffs);
    diffDoubleArray("engine.alt_ft", a.engine.alt_ft, b.engine.alt_ft, opts, diffs);
    diffDoubleArray("engine.mach", a.engine.mach, b.engine.mach, opts, diffs);
    diffDoubleArray("engine.thrust_idle", a.engine.thrust_idle, b.engine.thrust_idle, opts, diffs);
    diffDoubleArray("engine.thrust_mil", a.engine.thrust_mil, b.engine.thrust_mil, opts, diffs);
    diffDoubleArray("engine.thrust_ab", a.engine.thrust_ab, b.engine.thrust_ab, opts, diffs);
    diffDoubleArray("engine.fuelflow_idle", a.engine.fuelflow_idle, b.engine.fuelflow_idle, opts, diffs);
    diffDoubleArray("engine.fuelflow_mil", a.engine.fuelflow_mil, b.engine.fuelflow_mil, opts, diffs);
    diffDoubleArray("engine.fuelflow_ab", a.engine.fuelflow_ab, b.engine.fuelflow_ab, opts, diffs);

    // Roll command
    diffDoubleArray("rollCmd.alpha_deg", a.rollCmd.alpha_deg, b.rollCmd.alpha_deg, opts, diffs);
    diffDoubleArray("rollCmd.qbar", a.rollCmd.qbar, b.rollCmd.qbar, opts, diffs);
    diffDoubleArray("rollCmd.rollRate", a.rollCmd.rollRate, b.rollCmd.rollRate, opts, diffs);
    diffDouble("rollCmd.scale", a.rollCmd.scale, b.rollCmd.scale, opts, diffs);

    // Limiters
    for (int i = 0; i < static_cast<int>(f4flight::LimiterKey::Count); ++i) {
        diffLimiter("limiters[" + std::to_string(i) + "]", a.limiters[i], b.limiters[i], diffs);
    }

    if (diffs.empty()) {
        std::printf("No differences.\n");
        return 0;
    }

    if (opts.summary_only) {
        std::printf("%zu differences.\n", diffs.size());
    } else {
        std::printf("--- %s\n", pathA.c_str());
        std::printf("+++ %s\n", pathB.c_str());
        for (const auto& d : diffs) std::printf("%s\n", d.c_str());
        std::printf("\n%zu differences.\n", diffs.size());
    }
    return 1;
}
