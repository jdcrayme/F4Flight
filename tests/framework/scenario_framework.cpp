// f4flight - Maneuver test runner.
//
// Usage:
//   f4flight_scenario_test <aircraft.json> [options]
//
// Options:
//   --list                List available scenarios and exit.
//   --scenario NAME       Run only the named scenario. May be repeated to
//                         run several scenarios back-to-back.
//   --all                 Run every registered scenario (default).
//   --trace-dir DIR       Write one trace JSON per scenario to DIR (created
//                         if it does not exist). Traces are never written
//                         unless this flag or --html is given.
//   --html FILE           After running, emit a self-contained interactive
//                         HTML flight-path report to FILE (embeds all traces
//                         from this run inline — no external files needed).
//   --open                Open the HTML report in the default browser after
//                         writing it. Implies --html if no --html path given
//                         (writes to ./f4flight_report.html).
//   --max-time SEC        Per-phase hard cap (default: taken from each phase).
//   -h, --help            Show usage.
//
// Scenarios self-register at startup (see src/scenarios/*.cpp), so adding a
// new scenario is a matter of dropping a new .cpp file in src/scenarios/ and
// re-building. The runner never needs editing.
//
// Examples:
//   f4flight_scenario_test f16bk50.json                       # run all scenarios
//   f4flight_scenario_test f16bk50.json --scenario basic     # just the basic sequence
//   f4flight_scenario_test f16bk50.json --list               # show what's available
//   f4flight_scenario_test f16bk50.json --html report.html --open   # run + view

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi_mode.h"
#include "scenario_framework.h"
#include "trace.h"
#include "html_report.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight_test;

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------
struct CliOptions {
    std::string aircraftPath;
    std::vector<std::string> scenarios;  // empty => run all
    std::string traceDir;               // --trace-dir: write per-scenario JSON here
    std::string htmlPath;               // --html: write aggregated HTML report here
    bool doOpen{false};                 // --open: launch browser on the HTML
    bool list{false};
    bool help{false};
};

static void printHelp() {
    std::printf(
        "Usage: f4flight_scenario_test <aircraft.json> [options]\n"
        "\n"
        "Options:\n"
        "  --list              List available scenarios and exit.\n"
        "  --scenario NAME     Run only the named scenario. May be repeated.\n"
        "  --all               Run every registered scenario (default).\n"
        "  --trace-dir DIR     Write one trace JSON per scenario to DIR.\n"
        "  --html FILE         Write an interactive HTML flight-path report to FILE.\n"
        "  --open              Open the HTML report in the default browser.\n"
        "  -h, --help          Show this help.\n"
        "\n"
        "Examples:\n"
        "  f4flight_scenario_test f16bk50.json                       # run all scenarios\n"
        "  f4flight_scenario_test f16bk50.json --scenario basic     # just 'basic'\n"
        "  f4flight_scenario_test f16bk50.json --html r.html --open  # run + view report\n");
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
        } else if (a == "--trace-dir") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --trace-dir requires a path\n");
                opt.help = true;
                return opt;
            }
            opt.traceDir = argv[++i];
        } else if (a == "--html") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --html requires a path\n");
                opt.help = true;
                return opt;
            }
            opt.htmlPath = argv[++i];
        } else if (a == "--open") {
            opt.doOpen = true;
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
    // --open implies --html (default path if none given).
    if (opt.doOpen && opt.htmlPath.empty()) {
        opt.htmlPath = "f4flight_report.html";
    }
    return opt;
}

// ---------------------------------------------------------------------------
// Run one scenario end-to-end. Returns the number of phases that passed
// (out of total). If `rec` is non-null, records every frame and phase
// boundary into it for visualization.
// ---------------------------------------------------------------------------
struct ScenarioResult {
    int passed{0};
    int total{0};
};

static ScenarioResult runScenario(ManeuverScenario& scenario,
                                  FlightModel& fm,
                                  SteeringController& sc,
                                  const ScenarioContext& sctx,
                                  const std::string& aircraftName,
                                  TraceRecorder* rec) {
    ScenarioResult res;
    auto tests = scenario.StartScenario(fm, sctx);
    res.total = static_cast<int>(tests.size());

    const double dt = 1.0 / 60.0;

    // Clear any navigation state left over from a previous scenario. The
    // SteeringController (and its DigiBrain) is initialized once in main()
    // and shared across scenarios, so waypoints set by e.g. ai_flightplan
    // would otherwise leak into digi_defensive and show up in its trace.
    sc.setWaypoints({});

    if (rec) {
        rec->start(aircraftName, scenario.name());
        // Capture scene geometry (runway, taxiways, etc.) once per scenario.
        for (const auto& line : scenario.sceneGeometry()) {
            rec->addSceneLine(line);
        }
    }

    // Global simulation clock (continuous across phases) for the trace.
    double simT = 0.0;

    for (auto& test : tests) {
        // Detect whether this phase re-initializes the flight model (calls
        // fm.init()). We use two independent signals:
        //
        //  1. Position jump: save x/y/z before Init and compare afterward.
        //     fm.init() repositions the aircraft to a new spot; if the
        //     position moved more than a small threshold, a repositioning
        //     happened. This is the most reliable signal and catches reinits
        //     even from a near-stationary state (e.g. on the ground).
        //
        //  2. Body-rate reset: fm.init() zeros body rates (p,q,r). If the
        //     aircraft was maneuvering (rates > 0.01) and they dropped to
        //     ~0 after Init, a reinit happened even if the position didn't
        //     change much (e.g. attitude/velocity reset in place).
        const double preX = fm.state().kin.x;
        const double preY = fm.state().kin.y;
        const double preZ = fm.state().kin.z;
        const double preP = fm.state().kin.p;
        const double preQ = fm.state().kin.q;
        const double preR = fm.state().kin.r;

        sc.brain().setFrameInputs({});
        test->Init(sc, fm);

        const double dx = fm.state().kin.x - preX;
        const double dy = fm.state().kin.y - preY;
        const double dz = fm.state().kin.z - preZ;
        const double jumpFt = std::sqrt(dx * dx + dy * dy + dz * dz);

        const bool posJumped = jumpFt > 50.0;
        const bool ratesDropped =
            (std::fabs(preP) > 0.01 || std::fabs(preQ) > 0.01 || std::fabs(preR) > 0.01) &&
            (std::fabs(fm.state().kin.p) < 0.001 &&
             std::fabs(fm.state().kin.q) < 0.001 &&
             std::fabs(fm.state().kin.r) < 0.001);
        const bool reinitializes = posJumped || ratesDropped;

        // After Init, capture waypoints from the brain (set by Waypoint mode).
        // Done once per phase — if a later phase changes waypoints, the trace
        // picks up the latest set.
        if (rec) {
            const auto& wps = sc.brain().waypoints();
            if (!wps.empty()) {
                std::vector<Waypoint> twps;
                twps.reserve(wps.size());
                for (size_t i = 0; i < wps.size(); ++i) {
                    twps.push_back({wps[i].x, wps[i].y, wps[i].z,
                                    "WP" + std::to_string(i + 1)});
                }
                rec->setWaypoints(twps);
            }
        }

        const double phaseStartT = simT;

        while (!test->IsFinished()) {
            PilotInput input;
            std::string modeName;
            if (test->inputOverride(input, fm.state())) {
                modeName = "Manual";
            } else {
                input = sc.compute(fm.state(), dt, 0.0, fm.fcs(), fm.state().fcs);
                const double bankCmd = test->bankOverride_rad();
                if (bankCmd >= 0.0) {
                    input.rstick = limit((bankCmd - fm.state().kin.phi) * 2.0, -1.0, 1.0);
                }
                modeName = digiModeName(sc.brain().activeMode());
            }

            fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
            test->Evaluate(fm.state(), input, dt);

            if (rec) {
                // Extract active threats/targets from the brain state for
                // the trace. These are per-frame snapshots — the HTML viewer
                // draws missile tracks (moving points + trails) and bearing
                // lines from them.
                std::vector<ThreatEntity> threats;
                // Bind to a const DigiBrain& so overload resolution picks
                // the non-deprecated const state() overload (the non-const
                // state() is [[deprecated]] — see digi_brain.h).
                const auto& brain = sc.brain();
                const auto& ds = brain.state();
                if (ds.missileDefeat.incomingMissile) {
                    threats.push_back({"missile",
                        ds.missileDefeat.incomingMissile->x, ds.missileDefeat.incomingMissile->y,
                        ds.missileDefeat.incomingMissile->z, ds.missileDefeat.incomingMissile->speed});
                }
                if (ds.gunsJink.gunsThreat) {
                    threats.push_back({"guns",
                        ds.gunsJink.gunsThreat->x, ds.gunsJink.gunsThreat->y,
                        ds.gunsJink.gunsThreat->z, ds.gunsJink.gunsThreat->speed});
                }
                if (ds.ag.groundTarget) {
                    threats.push_back({"target",
                        ds.ag.groundTarget->x, ds.ag.groundTarget->y,
                        ds.ag.groundTarget->z, ds.ag.groundTarget->speed});
                }
                // Also extract the flight lead (if injected) so it shows up
                // in the visualization as a green track.
                if (brain.frameInputs().injectedLead) {
                    const auto* lead = brain.frameInputs().injectedLead;
                    threats.push_back({"lead",
                        lead->x, lead->y, lead->z, lead->speed});
                }
                // Merge custom trace entities from the test (formation slots,
                // ghost wingmen, etc.)
                auto custom = test->traceEntities();
                threats.insert(threats.end(), custom.begin(), custom.end());
                rec->record(simT, fm.state(), input, modeName, test->name(), threats);
            }
            simT += dt;
        }

        const double phaseEndT = simT;
        if (rec) {
            rec->markPhase(test->name(), phaseStartT, phaseEndT,
                           test->IsPassed(), /*skipped=*/false,
                           reinitializes, test->criteria());
        }

        test->Finish();
        std::printf("\n");

        if (test->IsPassed()) res.passed++;
    }

    if (rec) rec->finish(simT);
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

    // --- Visualization setup ---
    // Derive a short aircraft name from the file path (e.g. "f16bk50").
    std::string aircraftName = "aircraft";
    {
        std::error_code ec;
        auto p = std::filesystem::path(opt.aircraftPath).stem();
        if (!p.empty()) aircraftName = p.string();
    }
    // Record traces if either --trace-dir or --html is set.
    const bool recordTraces = !opt.traceDir.empty() || !opt.htmlPath.empty();
    if (recordTraces && !opt.traceDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(opt.traceDir, ec);
        if (ec) {
            std::fprintf(stderr, "Warning: could not create trace dir '%s': %s\n",
                        opt.traceDir.c_str(), ec.message().c_str());
        }
    }
    std::vector<Trace> traces;  // collected for the HTML report

    // Initialize flight model + steering controller ONCE.
    FlightModel fm;
    // Use the aircraft's corner speed for the initial condition, not a
    // hardcoded 300 kts. Each scenario's StartScenario() re-inits the
    // flight model at its own condition, but this default should still be
    // sensible for the aircraft.
    const double initCs = cfg.geometry.cornerVcas_kts > 0 ? cfg.geometry.cornerVcas_kts : 330.0;
    fm.init(cfg, 10000, initCs * KNOTS_TO_FTPSEC, 0.0, true);

    SteeringController sc;
    sc.setCornerSpeed(initCs);
    sc.setMaxGs(cfg.geometry.maxGs);
    sc.setMaxBank(45.0);
    sc.setAltitude(10000.0);
    sc.setHeading(0.0);
    // Use a conservative gamma clamp for AI navigation. FreeFalcon hardcodes
    // 60° (the F-16 autopilot attitude-hold envelope), but at AI navigation
    // speeds a 60° pitch command saturates the gamma loop and the aircraft
    // zooms, bleeds airspeed, and stalls. 15° matches a sustained climb.
    sc.setMaxGamma(15.0);

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

        TraceRecorder rec;
        ScenarioResult r = runScenario(*scenario, fm, sc, sctx, aircraftName,
                                      recordTraces ? &rec : nullptr);

        std::printf("  Scenario '%s' result: %d/%d\n\n",
                    scenario->name().c_str(),
                    r.passed, r.total);

        if (recordTraces) {
            // Write per-scenario JSON if --trace-dir was given.
            if (!opt.traceDir.empty()) {
                std::string fname = aircraftName + "_" + name + ".json";
                std::string path = (std::filesystem::path(opt.traceDir) / fname).string();
                if (rec.write(path)) {
                    std::printf("  trace written: %s\n", path.c_str());
                } else {
                    std::fprintf(stderr, "  warning: could not write trace %s\n", path.c_str());
                }
            }
            // Keep a copy for the HTML report.
            traces.push_back(rec.trace());
        }

        totalPassed   += r.passed;
        totalTests    += r.total;
    }

    // Final summary
    std::printf("=== Overall ===\n");
    std::printf("Scenarios run: %zu\n", scenarioNames.size());
    std::printf("Phases: %d/%d passed\n", totalPassed, totalTests);

    // --- Generate the HTML report if requested ---
    if (recordTraces && !opt.htmlPath.empty()) {
        std::printf("\nGenerating HTML report...\n");
        std::ofstream hf(opt.htmlPath);
        if (!hf) {
            std::fprintf(stderr, "Error: cannot write HTML report to %s\n", opt.htmlPath.c_str());
        } else {
            HtmlReportOptions hopts;
            hopts.title = "F4Flight — " + aircraftName;
            generateHtmlReport(traces, hf, hopts);
            hf.close();
            // Report file size.
            std::ifstream sz(opt.htmlPath, std::ios::ate | std::ios::binary);
            long bytes = sz ? (long)sz.tellg() : 0;
            std::printf("HTML report written: %s (%.1f KB, %zu trace%s)\n",
                        opt.htmlPath.c_str(), bytes / 1024.0,
                        traces.size(), traces.size() == 1 ? "" : "s");
            if (opt.doOpen) {
                // Build a file:// URL and launch the default browser.
                std::string absPath = opt.htmlPath;
                std::error_code ec;
                auto abs = std::filesystem::absolute(opt.htmlPath, ec);
                if (!ec) absPath = abs.string();
                std::string url = "file://";
                for (char c : absPath) {
                    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '/' || c == '-' ||
                        c == '_' || c == '.' || c == '~') {
                        url += c;
                    } else {
                        char hex[4];
                        std::snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
                        url += hex;
                    }
                }
                std::printf("Opening: %s\n", url.c_str());
#if defined(_WIN32) || defined(_WIN64)
                std::string cmd = "start \"\" \"" + absPath + "\"";
#elif defined(__APPLE__)
                std::string cmd = "open \"" + url + "\"";
#else
                std::string cmd = "xdg-open \"" + url + "\" 2>/dev/null";
#endif
                std::system(cmd.c_str());
            }
        }
    }

    // Exit non-zero if any phase failed so ctest/CI sees the real result.
    // Previously this returned 0 unconditionally, which meant ctest reported
    // PASS for every maneuver_*_basic and maneuver_*_flightplan test even
    // when 100% of phases failed.
    return (totalPassed == totalTests && totalTests > 0) ? 0 : 1;
}
