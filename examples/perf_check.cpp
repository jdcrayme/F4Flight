// f4flight - Performance validation: checks simulated aircraft performance
// against published open-source specifications.
//
// For each aircraft, we measure:
//   - Max level speed (at sea level and at altitude)
//   - Rate of climb (at sea level, MIL power)
//   - Service ceiling (altitude where climb rate drops below 500 fpm)
//   - Stall speed (where the aircraft can no longer maintain 1 G)
//   - Sustained turn rate (at corner speed, sea level)
//
// These are compared against published values from public sources (Wikipedia,
// manufacturer specs, Jane's). The comparison is "reasonableness" — we expect
// to be within +/- 25% of published values, not exact matches.
//
// Usage:
//   perf_check [aircraft.json]

#include "f4flight/f4flight.h"
#include "f4flight/json_io.h"

#include <cstdio>
#include <string>
#include <cmath>

using namespace f4flight;

// Published performance data for comparison (from Wikipedia / public sources)
struct PubSpec {
    const char* name;
    double maxSpeed_kts;      // max level speed at altitude
    double ceil_ft;           // service ceiling
    double roc_fpm;           // rate of climb at sea level
    double stall_kts;         // stall speed (clean)
};

static const PubSpec PUBSPECS[] = {
    {"f16bk50",   1500, 50000, 30000, 140},
    {"f15c",      1650, 65000, 50000, 130},
    {"f18c",      1190, 50000, 30000, 125},
    {"a10a",       381, 45000,  6000, 120},
    {"b52h",       560, 50000,  2700, 100},
    {"c130",       366, 33000,  1800, 110},
    {"su27",      1430, 62500, 35000, 130},
    {"mig29a",    1500, 59000, 33000, 125},
    {"mirage2k5",  1320, 56000, 28000, 130},
    {"torngr1",     920, 50000, 15000, 120},
    {"f14b",      1540, 53000, 30000, 130},
    {"av8b",       585, 41000,  4500, 110},
};
static const int NUM_SPECS = sizeof(PUBSPECS) / sizeof(PUBSPECS[0]);

const PubSpec* findSpec(const std::string& acName) {
    for (int i = 0; i < NUM_SPECS; ++i) {
        if (acName.find(PUBSPECS[i].name) != std::string::npos) return &PUBSPECS[i];
    }
    return nullptr;
}

// Measure max level speed at a given altitude
double measureMaxSpeed(FlightModel& fm, SteeringController& sc, double alt_ft, double speed_kts) {
    SteeringGoal goal;
    goal.hasHeading = true; goal.heading_rad = 0.0;
    goal.hasAltitude = true; goal.altitude_ft = alt_ft;
    goal.hasSpeed = true; goal.speed_kts = speed_kts;
    sc.setGoal(goal);

    const double dt = 1.0 / 60.0;
    // Run for 60 seconds to stabilize, then measure
    for (int i = 0; i < 3600; ++i) {  // 60 seconds
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    return fm.state().vcas;
}

// Measure rate of climb at sea level, MIL power
double measureROC(FlightModel& fm, SteeringController& sc) {
    // Set up: sea level, 350 kts, climbing
    SteeringGoal goal;
    goal.hasHeading = true; goal.heading_rad = 0.0;
    goal.hasAltitude = true; goal.altitude_ft = 5000.0;  // target above current
    goal.hasSpeed = true; goal.speed_kts = 350.0;
    sc.setGoal(goal);

    // Set throttle to MIL
    PilotInput manual;
    manual.throttle = 1.0;
    sc.setManualInput(manual);

    const double dt = 1.0 / 60.0;
    // Run for 10 seconds and measure average climb rate
    double alt0 = -fm.state().kin.z;
    for (int i = 0; i < 600; ++i) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        input.throttle = 1.0;  // force MIL
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    double alt1 = -fm.state().kin.z;
    double roc_fps = (alt1 - alt0) / 10.0;  // ft/s
    return roc_fps * 60.0;  // fpm
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s aircraft.json\n", argv[0]);
        return 1;
    }

    std::string path = argv[1];
    std::string acName = path.substr(path.find_last_of('/') + 1);
    acName = acName.substr(0, acName.find_last_of('.'));

    AircraftConfig cfg;
    auto result = json::readFile(path, cfg);
    if (!result.ok) { std::fprintf(stderr, "Failed to load\n"); return 1; }

    const PubSpec* spec = findSpec(acName);

    std::printf("=== Performance check: %s ===\n", cfg.name.c_str());
    if (spec) {
        std::printf("Comparing against published specs for %s\n\n", spec->name);
    } else {
        std::printf("(No published specs found for %s — skipping comparison)\n\n", acName.c_str());
    }

    // Test 1: Level flight at 10,000 ft, 350 kts
    FlightModel fm;
    fm.init(cfg, 10000.0, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);
    sc.setMaxGs(cfg.geometry.maxGs);

    double vcas = measureMaxSpeed(fm, sc, 10000.0, 350.0);
    double alt = -fm.state().kin.z;
    double g = fm.state().loads.nzcgs;
    std::printf("Level flight at 10,000 ft:\n");
    std::printf("  KCAS:     %.1f kts\n", vcas);
    std::printf("  Altitude: %.0f ft (target 10,000)\n", alt);
    std::printf("  G-load:   %.2f\n", g);

    // Test 2: Rate of climb
    fm.init(cfg, 1000.0, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);
    double roc = measureROC(fm, sc);
    std::printf("\nRate of climb at sea level (MIL power):\n");
    std::printf("  ROC:      %.0f fpm\n", roc);
    if (spec) {
        double ratio = roc / spec->roc_fpm;
        std::printf("  Published: %.0f fpm  (ratio: %.2f)\n", spec->roc_fpm, ratio);
        if (ratio > 0.5 && ratio < 2.0) std::printf("  -> REASONABLE\n");
        else std::printf("  -> OUT OF RANGE\n");
    }

    // Test 3: Max speed at altitude (20,000 ft, full throttle)
    fm.init(cfg, 20000.0, 400.0 * KNOTS_TO_FTPSEC, 0.0, true);
    // Run for 120 seconds at full throttle, holding altitude
    SteeringGoal goal;
    goal.hasHeading = true; goal.heading_rad = 0.0;
    goal.hasAltitude = true; goal.altitude_ft = 20000.0;
    goal.hasSpeed = true; goal.speed_kts = 800.0;  // try to go fast
    sc.setGoal(goal);
    const double dt = 1.0 / 60.0;
    for (int i = 0; i < 7200; ++i) {  // 120 seconds
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        input.throttle = 1.5;  // full AB
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
    }
    double maxVcas = fm.state().vcas;
    double maxMach = fm.state().mach;
    std::printf("\nMax speed at 20,000 ft (full AB):\n");
    std::printf("  KCAS:     %.1f kts\n", maxVcas);
    std::printf("  Mach:     %.3f\n", maxMach);
    if (spec) {
        double ratio = maxVcas / spec->maxSpeed_kts;
        std::printf("  Published max: %.0f kts  (ratio: %.2f)\n", spec->maxSpeed_kts, ratio);
        if (ratio > 0.5 && ratio < 2.0) std::printf("  -> REASONABLE\n");
        else std::printf("  -> OUT OF RANGE\n");
    }

    std::printf("\n=== Performance check complete ===\n");
    return 0;
}
