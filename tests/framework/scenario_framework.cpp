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
#include <set>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight_test;

// ===========================================================================
// Cascade mapping tables
//
// These tables define the relationship between the three test tiers:
//
//   g_e2eToHigh : E2E scenario name  -> list of HighLevel scenario names
//   g_highToLow : HighLevel scenario name -> list of LowLevel scenario names
//
// When an E2E test fails, the cascade runner uses g_e2eToHigh to look up the
// HighLevel chains that compose it, and runs each of those. When a HighLevel
// chain fails, g_highToLow is used to look up the individual LowLevel
// behaviors that compose the chain, and runs each of those. The result is a
// drill-down from "mission broken" to "specific behavior broken".
//
// Keep these tables in sync with the actual scenario registrations. The
// coverage matrix in COVERAGE.md is generated from this table.
// ===========================================================================
static const std::map<std::string, std::vector<std::string>> g_e2eToHigh = {
    // Core fighter missions (AMIS_BARCAP, AMIS_TARCAP, AMIS_SWEEP,
    // AMIS_INTERCEPT, AMIS_ESCORT).
    {"e2e_barcap",     {"high_departure", "high_loiter_station", "high_air_to_air_engage", "high_recovery"}},
    {"e2e_tarcap",     {"high_departure", "high_loiter_station", "high_air_to_air_engage", "high_recovery"}},
    {"e2e_sweep",      {"high_departure", "high_air_to_air_engage", "high_recovery"}},
    {"e2e_intercept",  {"high_departure", "high_air_to_air_engage", "high_recovery"}},
    {"e2e_escort",     {"high_departure", "high_formation_joinup", "high_air_to_air_engage", "high_recovery"}},
    // Legacy generic E2E (kept for backward compat with the old test matrix).
    {"digi_e2e_mission",       {"high_departure", "high_air_to_air_engage", "high_recovery"}},
    {"digi_e2e_formation",     {"high_departure", "high_formation_joinup", "high_recovery"}},
    {"digi_e2e_aar",           {"high_departure", "high_aar", "high_recovery"}},
    {"digi_e2e_ground_attack", {"high_departure", "high_air_to_ground", "high_recovery"}},
};

static const std::map<std::string, std::vector<std::string>> g_highToLow = {
    {"high_departure",          {"low_taxi", "low_takeoff", "low_climb", "low_level_hold"}},
    {"high_loiter_station",     {"low_loiter_orbit", "low_level_hold", "low_waypoint_follow"}},
    {"high_aar",                {"low_aar_vector", "low_aar_pre_contact", "low_aar_contact", "low_aar_disconnect"}},
    {"high_formation_joinup",   {"low_formation_position", "low_formation_types", "low_formation_turn"}},
    {"high_air_to_air_engage",  {"low_bvr_engage", "low_merge", "low_wvr_engage", "low_missile_engage",
                                 "low_guns_engage", "low_separate", "low_roop", "low_overb"}},
    {"high_air_to_ground",      {"low_ground_attack_low", "low_ground_attack_dive",
                                 "low_ground_attack_toss", "low_ground_attack_high"}},
    {"high_recovery",           {"low_rtb", "low_divert", "low_approach", "low_landing", "low_taxi"}},
    {"high_defensive_chain",    {"low_missile_defeat", "low_guns_jink", "low_collision_avoid"}},
};

// Public accessors that return an empty vector if the key is not found.
// Defined inside namespace f4flight_test to match the header declaration.
namespace f4flight_test {
const std::vector<std::string>& highScenariosFor(const std::string& e2eName) {
    static const std::vector<std::string> kEmpty;
    auto it = g_e2eToHigh.find(e2eName);
    return (it != g_e2eToHigh.end()) ? it->second : kEmpty;
}
const std::vector<std::string>& lowScenariosFor(const std::string& highName) {
    static const std::vector<std::string> kEmpty;
    auto it = g_highToLow.find(highName);
    return (it != g_highToLow.end()) ? it->second : kEmpty;
}
} // namespace f4flight_test

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------
struct CliOptions {
    std::string aircraftPath;
    std::vector<std::string> scenarios;  // empty => run all (in the selected tier(s))
    std::string traceDir;               // --trace-dir: write per-scenario JSON here
    std::string htmlPath;               // --html: write aggregated HTML report here
    bool doOpen{false};                 // --open: launch browser on the HTML
    bool list{false};
    bool help{false};

    // New 3-tier flags.
    //   --level {low,high,e2e,all}  Run only scenarios in the given tier.
    //                               Default "all". Mutually exclusive with
    //                               explicit --scenario lists.
    //   --cascade                   Run all E2E scenarios first; for each
    //                               failure, run the linked HighLevel
    //                               scenarios; for each HighLevel failure,
    //                               run the linked LowLevel scenarios.
    std::string levelArg{"all"};
    bool cascade{false};
};

static void printHelp() {
    std::printf(
        "Usage: f4flight_digi_scenarios <aircraft.json> [options]\n"
        "\n"
        "Scenario selection:\n"
        "  --list              List available scenarios (grouped by tier) and exit.\n"
        "  --scenario NAME     Run only the named scenario. May be repeated.\n"
        "  --all               Run every registered scenario (default).\n"
        "  --level TIER        Run all scenarios in TIER. TIER is one of:\n"
        "                        low   - Low Level (one behavior per scenario)\n"
        "                        high  - High Level (chains of behaviors)\n"
        "                        e2e   - End-to-End (full AMIS_* missions)\n"
        "                        all   - All tiers (default)\n"
        "  --cascade           Run E2E, then HighLevel for any E2E failures, then\n"
        "                      LowLevel for any HighLevel failures. Produces a\n"
        "                      drill-down report. Implies running only E2E first.\n"
        "\n"
        "Output:\n"
        "  --trace-dir DIR     Write one trace JSON per scenario to DIR.\n"
        "  --html FILE         Write an interactive HTML flight-path report to FILE.\n"
        "  --open              Open the HTML report in the default browser.\n"
        "  -h, --help          Show this help.\n"
        "\n"
        "Examples:\n"
        "  f4flight_digi_scenarios f16bk50.json                            # run all\n"
        "  f4flight_digi_scenarios f16bk50.json --level low                # just low-level\n"
        "  f4flight_digi_scenarios f16bk50.json --level e2e --html r.html  # E2E + report\n"
        "  f4flight_digi_scenarios f16bk50.json --cascade --html r.html    # drill-down\n"
        "  f4flight_digi_scenarios f16bk50.json --scenario low_takeoff     # one scenario\n");
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
            opt.levelArg = "all";
        } else if (a == "--scenario") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --scenario requires a name argument\n");
                opt.help = true;
                return opt;
            }
            opt.scenarios.push_back(argv[++i]);
        } else if (a == "--level") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --level requires an argument\n");
                opt.help = true;
                return opt;
            }
            opt.levelArg = argv[++i];
        } else if (a == "--cascade") {
            opt.cascade = true;
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

    // FULL BRAIN RESET between scenarios.
    //
    // The SteeringController (and its DigiBrain) is initialized once in
    // main() and shared across ALL scenarios. Without a reset, state from a
    // previous scenario leaks into the next:
    //
    //   - digi_groundops leaves the brain in Landing mode with
    //     groundOps.phase == Rollout. The addMode() interlock rule
    //     "LandingMode can't be bumped by WVR-family engagements" then
    //     prevents every subsequent offensive scenario (digi_guns,
    //     digi_wvr, digi_sensors target phase, digi_tactics break) from
    //     ever entering their intended mode — they all sit in Landing and
    //     descend until they fail.
    //   - digi_rtb leaves fuel.phase == Bingo and a divert airbase set.
    //   - Waypoints from ai_flightplan leak into other scenarios.
    //   - Threat/target pointers from injected entities dangle.
    //
    // sc.reset() calls DigiBrain::reset(), which:
    //   - clears curMode_ → Waypoint, forcedMode_ → NoMode
    //   - clears all threat/target pointers and auto-tracked entities
    //   - clears frameInputs_ (truth, injected*, fuel, airbases)
    //   - clears groundOps.phase → Parking (the "none" state)
    //   - PRESERVES config (cornerSpeed, maxGs, maxRoll, maxGamma) so the
    //     base tuning set in main() survives.
    //
    // Each phase's Init() re-configures what it needs (setMode, setCornerSpeed,
    // setMaxGs, etc.), so a full reset here is safe.
    sc.reset();

    if (rec) {
        rec->start(aircraftName, scenario.name());
        rec->setTestMetadata(scenario.GetTestGroup(), scenario.GetTestLevel());
        // Capture generalized geometry overlays.
        rec->setGeometry(scenario.traceGeometry());
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
        // Reset per-phase navigation state (integrators + stick commands).
        // Without this, a previous phase's wound-up GammaHold integrator and
        // smoothed stick commands carry over to the next phase, causing
        // transients (e.g. Takeoff's full-throttle + nose-up command leaks
        // into Landing, exciting the Phugoid). The mode, config, and
        // waypoints are preserved — only transient control state is cleared.
        // Each phase's Init() can still override anything it needs.
        sc.brain().resetPhaseState();
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
                std::vector<TraceGeometry> geom = rec->trace().geometry;
                std::vector<double> pathCoords;
                for (size_t i = 0; i < wps.size(); ++i) {
                    TraceGeometry tg;
                    tg.name = "WP" + std::to_string(i + 1);
                    tg.type = "waypoint";
                    tg.coords = {wps[i].x, wps[i].y, wps[i].z};
                    tg.color = "#FFFFFF";
                    geom.push_back(tg);

                    pathCoords.push_back(wps[i].x);
                    pathCoords.push_back(wps[i].y);
                    pathCoords.push_back(wps[i].z);
                }
                if (pathCoords.size() >= 6) {
                    TraceGeometry pathGeom;
                    pathGeom.name = "Flight Plan";
                    pathGeom.type = "corridor";
                    pathGeom.coords = pathCoords;
                    pathGeom.color = "#8a90a6";
                    pathGeom.width = 1.0;
                    geom.push_back(pathGeom);
                }
                rec->setGeometry(geom);
            }
        }

        const double phaseStartT = simT;
        std::string lastModeName;  // for mode-change event detection
        double lastFireEventT = -1.0;  // for gun-fire event throttling (per-phase)

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
                // Extract the RESOLVED offensive target (injected or auto-tracked
                // via SensorFusion). Without this, targets detected autonomously
                // (via truth/SensorFusion) never appear in the report — the old
                // code only drew injected targets.
                if (brain.resolvedTarget()) {
                    const auto* tgt = brain.resolvedTarget();
                    // Only add if not already covered by injectedTarget/groundTarget
                    // to avoid duplicate rendering.
                    bool alreadyHave = false;
                    for (const auto& th : threats) {
                        if (th.type == "target" &&
                            std::fabs(th.x - tgt->x) < 1.0 &&
                            std::fabs(th.y - tgt->y) < 1.0) {
                            alreadyHave = true;
                            break;
                        }
                    }
                    if (!alreadyHave) {
                        threats.push_back({"target",
                            tgt->x, tgt->y, tgt->z, tgt->speed});
                    }
                }
                // Also extract the flight lead (if injected) so it shows up
                // in the visualization as a green track.
                if (brain.frameInputs().injectedLead) {
                    const auto* lead = brain.frameInputs().injectedLead;
                    threats.push_back({"lead",
                        lead->x, lead->y, lead->z, lead->speed});
                }
                // Merge custom trace entities from the test (formation slots,
                // ghost wingmen, airbases, etc.)
                auto custom = test->traceEntities();
                threats.insert(threats.end(), custom.begin(), custom.end());
                rec->record(simT, fm.state(), input, modeName, test->name(), threats);

                // Publish per-frame samples from the test (range, heading
                // error, fuel, TTGO, etc.) for the frame readout + time-series.
                for (const auto& s : test->traceSamples()) {
                    rec->addSample(s.key, s.value, s.unit);
                }

                // Emit a mode-change event when the AI mode transitions.
                // This gives the report an event log of what the brain did
                // and when — critical for diagnosing "why didn't it enter
                // GunsEngage?" type failures.
                if (!lastModeName.empty() && modeName != lastModeName) {
                    rec->addEvent(simT, "mode",
                        "Mode: " + lastModeName + " -> " + modeName, "info");
                }
                lastModeName = modeName;

                // Emit a weapon-fire event when the gun fires.
                if (input.fireGun) {
                    // Only log the first fire frame per burst to avoid spam.
                    if (simT - lastFireEventT > 0.5) {
                        rec->addEvent(simT, "weapon", "Gun fired", "info");
                        lastFireEventT = simT;
                    }
                }
            }
            simT += dt;
        }

        const double phaseEndT = simT;
        if (rec) {
            std::string msg = "Phase [" + std::string(test->name()) + "] Finished: " +
                              (test->IsPassed() ? "PASSED" : "FAILED (" + test->failureReason() + ")");
            rec->addEvent(phaseEndT, "phase", msg, test->IsPassed() ? "info" : "fail");
            for (const auto& cond : test->conditions()) {
                std::string condMsg = "Condition [" + cond.name + "] (" + cond.description + "): " + (cond.passed ? "PASSED" : "FAILED");
                rec->addEvent(phaseEndT, "condition", condMsg, cond.passed ? "info" : "warn");
            }
            for (const auto& res : test->additionalResults()) {
                rec->addEvent(phaseEndT, "result", "Metric: " + res.text, res.color == "danger" ? "fail" : (res.color == "warning" ? "warn" : "info"));
            }
        }

        test->Finish();
        std::printf("\n");

        if (test->IsPassed()) res.passed++;
    }

    if (rec) rec->finish(simT);
    return res;
}

// ---------------------------------------------------------------------------
// Helper: print the scenario list grouped by tier
// ---------------------------------------------------------------------------
static void printScenariosByTier(const ScenarioRegistry& registry) {
    auto printTier = [&](TestTier t, const char* label) {
        const auto names = registry.listByTier(t);
        std::printf("\n=== %s (%zu) ===\n", label, names.size());
        for (const auto& name : names) {
            auto s = registry.create(name);
            std::printf("  %-30s %s\n", name.c_str(),
                        s ? s->GetDescription().c_str() : "(no description)");
        }
    };
    printTier(TestTier::LowLevel,  "Low Level  — one behavior per scenario");
    printTier(TestTier::HighLevel, "High Level — chains of related behaviors");
    printTier(TestTier::EndToEnd,  "End-to-End — full AMIS_* mission types");

    // Also print the cascade mapping so users can see the drill-down graph.
    std::printf("\n=== Cascade mapping (E2E -> High -> Low) ===\n");
    for (const auto& name : registry.listByTier(TestTier::EndToEnd)) {
        const auto& highs = highScenariosFor(name);
        if (highs.empty()) continue;
        std::printf("  %s\n", name.c_str());
        for (const auto& h : highs) {
            const auto& lows = lowScenariosFor(h);
            std::printf("    -> %s", h.c_str());
            if (!lows.empty()) {
                std::printf("  [");
                for (size_t i = 0; i < lows.size(); ++i) {
                    if (i) std::printf(", ");
                    std::printf("%s", lows[i].c_str());
                }
                std::printf("]");
            }
            std::printf("\n");
        }
    }
}

// ---------------------------------------------------------------------------
// Helper: run a single scenario by name. Returns the ScenarioResult and
// appends a trace to `traces` if `recordTraces` is true. Used by both the
// flat run-all loop and the cascade runner.
// ---------------------------------------------------------------------------
struct RunContext {
    FlightModel& fm;
    SteeringController& sc;
    const ScenarioContext& sctx;
    const std::string& aircraftName;
    const std::string& traceDir;
    bool recordTraces;
    std::vector<Trace>& traces;
    int totalPassed{0};
    int totalTests{0};
    std::vector<std::string> failedScenarios;  // populated for cascade
};

static ScenarioResult runOneScenario(const ScenarioRegistry& registry,
                                     const std::string& name,
                                     RunContext& rc,
                                     const char* banner = nullptr) {
    auto scenario = registry.create(name);
    if (!scenario) {
        std::fprintf(stderr, "Error: failed to create scenario '%s'\n", name.c_str());
        return ScenarioResult{0, 0};
    }

    if (banner) std::printf("\n%s\n", banner);
    std::printf("=== Scenario: %s  [%s] ===\n",
                scenario->name().c_str(),
                testTierName(scenario->GetTestTier()));
    std::printf("%s\n\n", scenario->GetDescription().c_str());

    TraceRecorder rec;
    ScenarioResult r = runScenario(*scenario, rc.fm, rc.sc, rc.sctx,
                                   rc.aircraftName,
                                   rc.recordTraces ? &rec : nullptr);

    std::printf("  Scenario '%s' result: %d/%d\n\n",
                scenario->name().c_str(), r.passed, r.total);

    if (rc.recordTraces) {
        if (!rc.traceDir.empty()) {
            std::string fname = rc.aircraftName + "_" + name + ".json";
            std::string path = (std::filesystem::path(rc.traceDir) / fname).string();
            if (rec.write(path)) {
                std::printf("  trace written: %s\n", path.c_str());
            } else {
                std::fprintf(stderr, "  warning: could not write trace %s\n", path.c_str());
            }
        }
        rc.traces.push_back(rec.trace());
    }

    rc.totalPassed += r.passed;
    rc.totalTests  += r.total;
    if (r.passed != r.total) {
        rc.failedScenarios.push_back(name);
    }
    return r;
}

// ---------------------------------------------------------------------------
// Cascade runner: runs all E2E scenarios; for each failure runs the linked
// HighLevel scenarios; for each HighLevel failure runs the linked LowLevel
// scenarios. Each scenario is run at most once per cascade (deduplicated).
// ---------------------------------------------------------------------------
static void runCascade(const ScenarioRegistry& registry, RunContext& rc) {
    std::set<std::string> alreadyRan;

    auto runIfNew = [&](const std::string& name, const char* banner) -> bool {
        if (alreadyRan.count(name)) return true;  // already passed (or already ran)
        if (!registry.has(name)) {
            std::fprintf(stderr, "  (cascade: scenario '%s' not registered, skipping)\n",
                         name.c_str());
            return true;
        }
        alreadyRan.insert(name);
        runOneScenario(registry, name, rc, banner);
        // Return true if the scenario PASSED.
        auto s = registry.create(name);
        if (!s) return true;
        // Re-run the scenario to get the result? No — we just stored it in
        // rc.failedScenarios. Check there.
        for (const auto& failed : rc.failedScenarios) {
            if (failed == name) return false;
        }
        return true;
    };

    // Tier 1: run all E2E scenarios.
    const auto e2eNames = registry.listByTier(TestTier::EndToEnd);
    std::printf("\n"
                "#############################################\n"
                "# CASCADE TIER 1: End-to-End scenarios     #\n"
                "#############################################\n");
    std::vector<std::string> e2eFailures;
    for (const auto& name : e2eNames) {
        alreadyRan.insert(name);
        runOneScenario(registry, name, rc,
                       "--- E2E baseline run ---");
        // Did it fail? Check rc.failedScenarios.
        bool failed = false;
        for (const auto& f : rc.failedScenarios) {
            if (f == name) { failed = true; break; }
        }
        if (failed) e2eFailures.push_back(name);
    }

    if (e2eFailures.empty()) {
        std::printf("\nAll E2E scenarios passed — cascade complete.\n");
        return;
    }

    // Tier 2: run the linked HighLevel scenarios for each E2E failure.
    std::printf("\n"
                "#############################################\n"
                "# CASCADE TIER 2: High Level drill-down    #\n"
                "# (%zu E2E scenarios failed)                #\n"
                "#############################################\n",
                e2eFailures.size());
    std::vector<std::string> highFailures;
    for (const auto& e2eName : e2eFailures) {
        const auto& highs = highScenariosFor(e2eName);
        if (highs.empty()) {
            std::printf("\n(E2E '%s' has no HighLevel mapping — skipping drill-down)\n",
                        e2eName.c_str());
            continue;
        }
        std::printf("\n--- Drill-down for E2E failure '%s' ---\n", e2eName.c_str());
        for (const auto& h : highs) {
            if (alreadyRan.count(h)) {
                std::printf("\n(skipping '%s' — already ran in this cascade)\n", h.c_str());
                continue;
            }
            alreadyRan.insert(h);
            runOneScenario(registry, h, rc,
                           "--- High Level chain (linked from E2E failure) ---");
            bool failed = false;
            for (const auto& f : rc.failedScenarios) {
                if (f == h) { failed = true; break; }
            }
            if (failed) highFailures.push_back(h);
        }
    }

    if (highFailures.empty()) {
        std::printf("\nAll HighLevel chains passed — cascade complete.\n");
        return;
    }

    // Tier 3: run the linked LowLevel scenarios for each HighLevel failure.
    std::printf("\n"
                "#############################################\n"
                "# CASCADE TIER 3: Low Level drill-down     #\n"
                "# (%zu High Level chains failed)            #\n"
                "#############################################\n",
                highFailures.size());
    for (const auto& hName : highFailures) {
        const auto& lows = lowScenariosFor(hName);
        if (lows.empty()) {
            std::printf("\n(HighLevel '%s' has no LowLevel mapping — skipping drill-down)\n",
                        hName.c_str());
            continue;
        }
        std::printf("\n--- Drill-down for HighLevel failure '%s' ---\n", hName.c_str());
        for (const auto& l : lows) {
            if (alreadyRan.count(l)) {
                std::printf("\n(skipping '%s' — already ran in this cascade)\n", l.c_str());
                continue;
            }
            alreadyRan.insert(l);
            runOneScenario(registry, l, rc,
                           "--- Low Level behavior (linked from HighLevel failure) ---");
        }
    }
    std::printf("\nCascade complete.\n");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {

    CliOptions opt = parseArgs(argc, argv);
    if (opt.help) { printHelp(); return 0; }

    auto& registry = ScenarioRegistry::instance();
    if (opt.list) {
        std::printf("Available scenarios (grouped by tier):\n");
        printScenariosByTier(registry);
        return 0;
    }

    if (opt.aircraftPath.empty()) {
        std::fprintf(stderr, "Error: no aircraft JSON specified.\n\n");
        printHelp();
        return 1;
    }

    // Validate --level argument.
    TestTier tierFilter;
    bool hasTierFilter = false;
    if (opt.levelArg != "all") {
        if (!parseTestTier(opt.levelArg, tierFilter)) {
            std::fprintf(stderr, "Error: invalid --level '%s'. Must be low|high|e2e|all.\n",
                        opt.levelArg.c_str());
            return 1;
        }
        hasTierFilter = true;
    }
    if (hasTierFilter && !opt.scenarios.empty()) {
        std::fprintf(stderr,
            "Error: --level and --scenario are mutually exclusive. Pick one.\n");
        return 1;
    }
    if (opt.cascade && (hasTierFilter || !opt.scenarios.empty())) {
        std::fprintf(stderr,
            "Error: --cascade cannot be combined with --level or --scenario.\n");
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

    // Build the list of scenarios to run.
    //   --scenario X Y    -> run X and Y
    //   --level T         -> run all scenarios in tier T
    //   --cascade         -> run E2E first, then drill down (handled below)
    //   (none of the above) -> run all scenarios
    std::vector<std::string> scenarioNames = opt.scenarios;
    if (scenarioNames.empty() && !opt.cascade) {
        if (hasTierFilter) {
            scenarioNames = registry.listByTier(tierFilter);
        } else {
            scenarioNames = registry.list();
        }
    } else if (!scenarioNames.empty()) {
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

    RunContext rc{fm, sc, sctx, aircraftName, opt.traceDir, recordTraces, traces};

    if (opt.cascade) {
        runCascade(registry, rc);
    } else {
        for (const auto& name : scenarioNames) {
            runOneScenario(registry, name, rc);
        }
    }

    int totalPassed = rc.totalPassed;
    int totalTests  = rc.totalTests;

    // Final summary
    std::printf("=== Overall ===\n");
    if (opt.cascade) {
        std::printf("Cascade run (E2E + High + Low drill-down).\n");
        std::printf("Failed scenarios: %zu\n", rc.failedScenarios.size());
        for (const auto& f : rc.failedScenarios) {
            std::printf("  - %s\n", f.c_str());
        }
    } else {
        std::printf("Scenarios run: %zu\n", scenarioNames.size());
    }
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
            const long bytes = sz ? static_cast<long>(sz.tellg()) : 0;
            std::printf("HTML report written: %s (%.1f KB, %zu trace%s)\n",
                        opt.htmlPath.c_str(), static_cast<double>(bytes) / 1024.0,
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
