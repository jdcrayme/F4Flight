// f4flight - Maneuver test runner.
//
// Usage:
//   maneuver_test <aircraft.json> [options]
//
// Options:
//   --list                List available scenarios and exit.
//   --scenario NAME       Run only the named scenario. May be repeated to
//                         run several scenarios back-to-back.
//   --all                 Run every registered scenario (default).
//   --max-time SEC        Per-phase hard cap (default: taken from each phase).
//   -h, --help            Show usage.
//
// Scenarios self-register at startup (see src/scenarios/*.cpp), so adding a
// new scenario is a matter of dropping a new .cpp file in src/scenarios/ and
// re-building. The runner never needs editing.
//
// Examples:
//   maneuver_test f16bk50.json                       # run all scenarios
//   maneuver_test f16bk50.json --scenario basic     # just the basic sequence
//   maneuver_test f16bk50.json --list               # show what's available

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace f4flight;
using namespace manuver_test;

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------
struct CliOptions {
    std::string aircraftPath;
    std::vector<std::string> scenarios;  // empty => run all
    bool list{false};
    bool help{false};
};

static void printHelp() {
    std::printf(
        "Usage: maneuver_test <aircraft.json> [options]\n"
        "\n"
        "Options:\n"
        "  --list              List available scenarios and exit.\n"
        "  --scenario NAME     Run only the named scenario. May be repeated.\n"
        "  --all               Run every registered scenario (default).\n"
        "  -h, --help          Show this help.\n"
        "\n"
        "Examples:\n"
        "  maneuver_test f16bk50.json                       # run all scenarios\n"
        "  maneuver_test f16bk50.json --scenario basic     # just 'basic'\n"
        "  maneuver_test f16bk50.json --list               # list scenarios\n");
}

static CliOptions parseArgs(int argc, char** argv) {
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            opt.help = true;
        } else if (a == "--list") {
            opt.list = true;
        } else if (a == "--all") {
            opt.scenarios.clear();
        } else if (a == "--scenario") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --scenario requires a name argument\n");
                opt.help = true;
                return opt;
            }
            opt.scenarios.push_back(argv[++i]);
        } else if (a.size() > 0 && a[0] == '-') {
            std::fprintf(stderr, "Error: unknown option '%s'\n\n", a.c_str());
            opt.help = true;
        } else if (opt.aircraftPath.empty()) {
            opt.aircraftPath = a;
        } else {
            std::fprintf(stderr, "Error: unexpected positional argument '%s'\n", a.c_str());
            opt.help = true;
        }
    }
    return opt;
}

// ---------------------------------------------------------------------------
// Run one scenario end-to-end. Returns the number of phases that passed
// (out of total).
// ---------------------------------------------------------------------------
struct ScenarioResult {
    int passed{0};
    int total{0};
};

static ScenarioResult runScenario(ManeuverScenario& scenario,
                                  FlightModel& fm,
                                  SteeringController& sc,
                                  const ScenarioContext& sctx) {
    ScenarioResult res;
    auto tests = scenario.StartScenario(fm, sctx);
    res.total = static_cast<int>(tests.size());

    const double dt = 1.0 / 60.0;

    for (auto& test : tests) {
        // Note: we deliberately do NOT call sc.reset() between phases.
        // The throttle-comparison fix in SteeringController::compute (using
        // a sentinel value instead of comparing to the manual default)
        // already prevents SpeedHold from winding up during climb/descent
        // phases where AltitudeHold owns the throttle. Resetting the PIDs
        // between phases would wipe the throttle PID's steady-state
        // integral, causing the throttle to drop to ~0 at the start of the
        // next phase — which in the flightplan scenario leads to a
        // deceleration → stall → NaN cascade within 10 seconds.
        test->Init(sc, fm);

        while (!test->IsFinished()) {
            PilotInput input = sc.compute(fm.state(), dt, 0.0);

            // If the phase wants a direct bank-angle override (e.g. Orbit,
            // high-G turn), apply it as a roll-rate command.
            const double bankCmd = test->bankOverride_rad();
            if (bankCmd >= 0.0) {
                input.rstick = limit((bankCmd - fm.state().kin.phi) * 2.0, -1.0, 1.0);
            }

            fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
            test->Evaluate(fm.state(), input, dt);
        }

        test->Finish();
        std::printf("\n");

        if (test->IsPassed()) res.passed++;
    }
    return res;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {

    CliOptions opt = parseArgs(argc, argv);
    if (opt.help) { printHelp(); return 0; }

    auto& registry = ScenarioRegistry::instance();
    if (opt.list) {
        std::printf("Available scenarios:\n");
        for (const auto& name : registry.list()) {
            auto s = registry.create(name);
            std::printf("  %-12s %s\n", name.c_str(),
                        s ? s->GetDescription().c_str() : "(no description)");
        }
        return 0;
    }

    if (opt.aircraftPath.empty()) {
        std::fprintf(stderr, "Error: no aircraft JSON specified.\n\n");
        printHelp();
        return 1;
    }

    // Load the aircraft
    AircraftConfig cfg;
    auto result = json::readFile(opt.aircraftPath, cfg);
    if (!result.ok) {
        std::fprintf(stderr, "Failed to load %s\n", opt.aircraftPath.c_str());
        for (auto const& e : result.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 1;
    }
    // Sanity-check the config (uses the new validate() facility from v3.2).
    auto vreport = cfg.validate();
    if (!vreport.ok()) {
        std::fprintf(stderr, "Warning: aircraft config has validation errors:\n");
        std::fprintf(stderr, "%s", vreport.format().c_str());
        std::fprintf(stderr, "(attempting to run anyway)\n\n");
    }

    // Build the list of scenarios to run
    std::vector<std::string> scenarioNames = opt.scenarios;
    if (scenarioNames.empty()) {
        scenarioNames = registry.list();
    } else {
        // Validate requested scenarios
        for (const auto& name : scenarioNames) {
            if (!registry.has(name)) {
                std::fprintf(stderr, "Error: unknown scenario '%s'.\n", name.c_str());
                std::fprintf(stderr, "Available scenarios:\n");
                for (const auto& n : registry.list()) std::fprintf(stderr, "  %s\n", n.c_str());
                return 1;
            }
        }
    }

    // Initialize flight model + steering controller ONCE.
    // Each scenario should reinitialize to start from a clean state,
    // but we don't want to re-read the aircraft JSON for each scenario.
    // The steering controller's max bank comes from the per-aircraft
    // performance profile (fighters: 45°, heavies: 25-30°).
    FlightModel fm;
    fm.init(cfg, cfg.profile.cruiseAlt_ft,
            calcTasFromKcas(cfg.profile.cruiseSpeed_kts, cfg.profile.cruiseAlt_ft),
            0.0, true);

    SteeringController sc;
    sc.setMaxBankAngle_deg(cfg.profile.maxBank_deg);
    sc.setMaxGs(cfg.geometry.maxGs);

    ScenarioContext sctx{cfg};

    int totalPassed = 0, totalTests = 0;

    for (const auto& name : scenarioNames) {
        auto scenario = registry.create(name);
        if (!scenario) {
            std::fprintf(stderr, "Error: failed to create scenario '%s'\n", name.c_str());
            continue;
        }

        std::printf("=== Scenario: %s ===\n", scenario->name().c_str());
        std::printf("%s\n\n", scenario->GetDescription().c_str());

        ScenarioResult r = runScenario(*scenario, fm, sc, sctx);

        std::printf("  Scenario '%s' result: %d/%d\n\n",
                    scenario->name().c_str(),
                    r.passed, r.total);

        totalPassed   += r.passed;
        totalTests    += r.total;
    }

    // Final summary
    std::printf("=== Overall ===\n");
    std::printf("Scenarios run: %zu\n", scenarioNames.size());
    std::printf("Phases: %d/%d passed\n", totalPassed, totalTests);

    return 0;
}
