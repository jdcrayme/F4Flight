// f4flight - Simplified flat conditional-driven Scenario test runner.
//
// Usage:
//   f4flight_digi_scenarios <aircraft.json> [options]
//
// Options:
//   --list                List available scenarios and exit.
//   --scenario NAME       Run only the named scenario.
//   --trace-dir DIR       Write trace JSON per scenario to DIR.
//   --html FILE           Emit self-contained interactive HTML flight-path report.
//   --open                Open the HTML report in default browser.
//   --summary FILE        Write structured, machine-parseable JSON summary of results.
//   -h, --help            Show usage.

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

struct CliOptions {
    std::string aircraftPath;
    std::vector<std::string> scenarios;
    std::string traceDir;
    std::string htmlPath;
    std::string summaryPath;
    bool doOpen{false};
    bool list{false};
    bool help{false};
};

static void printHelp() {
    std::printf(
        "Usage: f4flight_digi_scenarios <aircraft.json> [options]\n"
        "\n"
        "Scenario selection:\n"
        "  --list              List available scenarios and exit.\n"
        "  --scenario NAME     Run only the named scenario. May be repeated.\n"
        "\n"
        "Output:\n"
        "  --trace-dir DIR     Write one trace JSON per scenario to DIR.\n"
        "  --html FILE         Write an interactive HTML flight-path report to FILE.\n"
        "  --summary FILE      Write a structured, machine-parseable JSON summary to FILE.\n"
        "  --open              Open the HTML report in the default browser.\n"
        "  -h, --help          Show this help.\n");
}

static CliOptions parseArgs(int argc, char** argv) {
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            opt.help = true;
        } else if (a == "--list") {
            opt.list = true;
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
        } else if (a == "--summary") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --summary requires a path\n");
                opt.help = true;
                return opt;
            }
            opt.summaryPath = argv[++i];
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
    if (opt.doOpen && opt.htmlPath.empty()) {
        opt.htmlPath = "f4flight_report.html";
    }
    return opt;
}

struct PhaseResult {
    std::string name;
    bool passed{false};
    std::string criteria;
    std::string failureReason;
    std::vector<TestCondition> conditions;
};

struct ScenarioRunResult {
    std::string name;
    bool passed{false};
    std::vector<PhaseResult> phases;
};

struct ScenarioResult {
    int passed{0};
    int total{0};
    std::vector<PhaseResult> phaseDetails;
};

static ScenarioResult runScenario(ManeuverScenario& scenario,
                                  const std::string& aircraftPath,
                                  const std::string& aircraftName,
                                  TraceRecorder* rec) {
    ScenarioResult res;
    res.total = 1; // The scenario acts as a single unified flat run

    scenario.ClearScenarioObjects();
    scenario.SetDefaultAircraftPath(aircraftPath);
    scenario.StartScenario(aircraftPath);

    const double dt = 1.0 / 60.0;

    if (rec) {
        rec->start(scenario.aircraftList()[0]->name, scenario.name());
        rec->setTestMetadata(scenario.GetTestGroup(), scenario.GetTestLevel());
        rec->setGeometry(scenario.traceGeometry());
    }

    double simT = 0.0;
    int frameCount = 0;
    bool printedHeader = false;

    std::set<Conditional*> loggedConditionals;

    // Start all conditionals
    for (auto& cond : scenario.conditionals()) {
        if (rec) {
            auto oldPassed = cond->OnPassed;
            cond->OnPassed = [&simT, rec, cond, oldPassed, &loggedConditionals]() {
                if (loggedConditionals.find(cond.get()) == loggedConditionals.end()) {
                    loggedConditionals.insert(cond.get());
                    std::string condMsg = "Condition [" + cond->name() + "] (" + cond->criteria() + "): PASSED";
                    rec->addEvent(simT, "condition", condMsg, "info");
                }
                if (oldPassed) oldPassed();
            };

            auto oldFailed = cond->OnFailed;
            cond->OnFailed = [&simT, rec, cond, oldFailed, &loggedConditionals]() {
                if (loggedConditionals.find(cond.get()) == loggedConditionals.end()) {
                    loggedConditionals.insert(cond.get());
                    std::string condMsg = "Condition [" + cond->name() + "] (" + cond->criteria() + "): FAILED";
                    rec->addEvent(simT, "condition", condMsg, "warn");
                }
                if (oldFailed) oldFailed();
            };
        }
        cond->Start();
    }

    // Capture waypoints from the primary aircraft (if any) for tracing
    if (rec && !scenario.aircraftList().empty()) {
        const auto& brain = scenario.aircraftList()[0]->brain;
        const auto& wps = brain.waypoints();
        if (!wps.empty()) {
            std::vector<TraceGeometry> geom = rec->trace().geometry;
            std::vector<double> pathCoords;
            for (size_t i = 0; i < wps.size(); ++i) {
                TraceGeometry tg;
                tg.name = "WP" + std::to_string(i + 1);
                tg.type = "waypoint";
                tg.coords = std::vector<double>{wps[i].x, wps[i].y, wps[i].z};
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

    std::string lastModeName;

    // Main flat simulation loop
    while (true) {
        // Evaluate stop conditions
        bool allRequiredPassed = true;
        bool hasRequired = false;
        for (const auto& cond : scenario.conditionals()) {
            if (cond->isRequired()) {
                hasRequired = true;
                if (!cond->hasPassed()) {
                    allRequiredPassed = false;
                }
            }
        }

        bool shouldStop = (simT >= scenario.maxTime()) || (hasRequired && allRequiredPassed);
        if (shouldStop) break;

        // Execute dynamic scenario update logic
        scenario.UpdateScenario(dt);

        // Step all aircraft
        for (size_t idx = 0; idx < scenario.aircraftList().size(); ++idx) {
            auto& ac = scenario.aircraftList()[idx];
            ac->input = ac->brain.compute(ac->fm.state(), dt, 0.0, ac->fm.fcs(), ac->fm.state().fcs);
            ac->activeModeName = f4flight::digi::digiModeName(ac->brain.activeMode());
            ac->fm.update(dt, ac->input, 0.0, Vec3{0.0, 0.0, 1.0});
        }

        // Update telemetries
        for (auto& tel : scenario.telemetries()) {
            tel->sample();
        }

        // Evaluate conditionals
        for (auto& cond : scenario.conditionals()) {
            cond->Evaluate(dt);
        }

        // Output telemetry to console every 10 frames
        if (!scenario.telemetries().empty()) {
            if (!printedHeader) {
                std::printf("%-10s", "Frame");
                for (const auto& tel : scenario.telemetries()) {
                    std::printf(" %-15s", tel->name().c_str());
                }
                std::printf("\n");
                printedHeader = true;
            }
            if (frameCount % 10 == 0) {
                std::printf("%-10d", frameCount);
                for (const auto& tel : scenario.telemetries()) {
                    std::printf(" %-15.3f", tel->lastValue());
                }
                std::printf("\n");
            }
        }

        // Record trace frame if requested
        if (rec && !scenario.aircraftList().empty()) {
            auto& primaryAc = scenario.aircraftList()[0];
            std::vector<ThreatEntity> threats;

            // Map secondary aircraft as wingmen/moving tracks
            for (size_t idx = 1; idx < scenario.aircraftList().size(); ++idx) {
                auto& ac = scenario.aircraftList()[idx];
                ThreatEntity t;
                t.type = "wingman";
                t.name = ac->name;
                t.x = ac->fm.state().kin.x;
                t.y = ac->fm.state().kin.y;
                t.z = ac->fm.state().kin.z;
                t.speed = ac->fm.state().kin.vt;
                t.psi = ac->fm.state().kin.psi;
                threats.push_back(t);
            }

            rec->record(simT, primaryAc->fm.state(), primaryAc->input, primaryAc->activeModeName, scenario.name(), threats);

            // Record registered telemetries as trace samples
            for (const auto& tel : scenario.telemetries()) {
                rec->addSample(tel->name(), tel->lastValue(), "");
            }

            // Record radio calls from all active aircraft
            for (auto& ac : scenario.aircraftList()) {
                auto& comm = ac->brain.stateMutable().comm;
                digi::RadioCall call;
                while (comm.radioCalls.pop(call)) {
                    std::string callMsg = ac->name + ": \"" + digi::radioCallText(call.type) + "\"";
                    rec->addEvent(simT, "comm", callMsg, "info");
                }
            }

            if (!lastModeName.empty() && primaryAc->activeModeName != lastModeName) {
                rec->addEvent(simT, "mode", "Mode: " + lastModeName + " -> " + primaryAc->activeModeName, "info");
            }
            lastModeName = primaryAc->activeModeName;
        }

        simT += dt;
        frameCount++;
    }

    // Stop all conditionals
    for (auto& cond : scenario.conditionals()) {
        cond->Stop();
    }

    // Determine final scenario outcome
    bool scenarioPassed = true;
    std::string criteriaText;
    std::string failureReasonText;
    std::vector<TestCondition> conds;

    for (const auto& cond : scenario.conditionals()) {
        if (!criteriaText.empty()) criteriaText += "; ";
        criteriaText += cond->name() + " (" + cond->criteria() + ")";

        if (cond->isRequired() && !cond->hasPassed()) {
            scenarioPassed = false;
            if (!failureReasonText.empty()) failureReasonText += "; ";
            failureReasonText += cond->failureReason();
        }

        conds.push_back({cond->name(), cond->criteria() + ": " + (cond->hasPassed() ? "Passed" : "Failed"), cond->hasPassed()});

        if (rec && loggedConditionals.find(cond.get()) == loggedConditionals.end()) {
            std::string condMsg = "Condition [" + cond->name() + "] (" + cond->criteria() + "): " + (cond->hasPassed() ? "PASSED" : "FAILED");
            rec->addEvent(simT, "condition", condMsg, cond->hasPassed() ? "info" : "warn");
            loggedConditionals.insert(cond.get());
        }
    }

    if (rec) {
        std::string msg = "Scenario [" + scenario.name() + "] Finished: " +
                          (scenarioPassed ? "PASSED" : "FAILED (" + failureReasonText + ")");
        rec->addEvent(simT, "phase", msg, scenarioPassed ? "info" : "fail");
        rec->finish(simT);
    }

    // Console summary output
    std::printf("  --- Scenario Finish Summary ---\n");
    std::printf("  Name: %s\n", scenario.name().c_str());
    std::printf("  Result: %s\n", scenarioPassed ? "PASSED" : "FAILED");
    if (!scenarioPassed) {
        std::printf("  Failure Reason: %s\n", failureReasonText.c_str());
    }

    PhaseResult pr;
    pr.name = scenario.name();
    pr.passed = scenarioPassed;
    pr.criteria = criteriaText;
    pr.failureReason = failureReasonText;
    pr.conditions = conds;
    res.phaseDetails.push_back(pr);

    if (scenarioPassed) res.passed++;

    return res;
}

static void writeSummaryJson(const std::string& filepath,
                             const std::string& aircraft,
                             const std::vector<ScenarioRunResult>& results) {
    std::ofstream sf(filepath);
    if (!sf) {
        std::fprintf(stderr, "Error: cannot write JSON summary to %s\n", filepath.c_str());
        return;
    }

    int totalScenarios = static_cast<int>(results.size());
    int passedScenarios = 0;
    for (const auto& r : results) {
        if (r.passed) passedScenarios++;
    }

    sf << "{\n";
    sf << "  \"aircraft\": \"" << aircraft << "\",\n";
    sf << "  \"summary\": {\n";
    sf << "    \"total_scenarios\": " << totalScenarios << ",\n";
    sf << "    \"passed_scenarios\": " << passedScenarios << ",\n";
    sf << "    \"failed_scenarios\": " << (totalScenarios - passedScenarios) << "\n";
    sf << "  },\n";
    sf << "  \"scenarios\": [\n";

    for (size_t i = 0; i < results.size(); ++i) {
        const auto& sr = results[i];
        sf << "    {\n";
        sf << "      \"name\": \"" << sr.name << "\",\n";
        sf << "      \"passed\": " << (sr.passed ? "true" : "false") << ",\n";
        sf << "      \"phases\": [\n";

        for (size_t j = 0; j < sr.phases.size(); ++j) {
            const auto& pr = sr.phases[j];
            sf << "        {\n";
            sf << "          \"name\": \"" << pr.name << "\",\n";
            sf << "          \"passed\": " << (pr.passed ? "true" : "false") << ",\n";
            sf << "          \"criteria\": \"" << pr.criteria << "\",\n";

            std::string escapedReason = pr.failureReason;
            size_t pos = 0;
            while ((pos = escapedReason.find('"', pos)) != std::string::npos) {
                escapedReason.replace(pos, 1, "\\\"");
                pos += 2;
            }
            sf << "          \"failure_reason\": \"" << escapedReason << "\",\n";
            sf << "          \"conditions\": [\n";

            for (size_t k = 0; k < pr.conditions.size(); ++k) {
                const auto& cond = pr.conditions[k];
                sf << "            {\n";
                sf << "              \"name\": \"" << cond.name << "\",\n";
                sf << "              \"description\": \"" << cond.description << "\",\n";
                sf << "              \"passed\": " << (cond.passed ? "true" : "false") << "\n";
                sf << "            }" << (k + 1 < pr.conditions.size() ? "," : "") << "\n";
            }
            sf << "          ]\n";
            sf << "        }" << (j + 1 < sr.phases.size() ? "," : "") << "\n";
        }
        sf << "      ]\n";
        sf << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }

    sf << "  ]\n";
    sf << "}\n";
    sf.close();
    std::printf("Machine-parseable JSON summary written: %s\n", filepath.c_str());
}

int main(int argc, char** argv) {
    CliOptions opt = parseArgs(argc, argv);
    if (opt.help) { printHelp(); return 0; }

    auto& registry = ScenarioRegistry::instance();
    if (opt.list) {
        std::printf("Available scenarios:\n");
        for (const auto& name : registry.list()) {
            auto s = registry.create(name);
            std::printf("  %-30s %s\n", name.c_str(),
                        s ? s->GetDescription().c_str() : "(no description)");
        }
        return 0;
    }

    if (opt.aircraftPath.empty()) {
        std::fprintf(stderr, "Error: no aircraft JSON specified.\n\n");
        printHelp();
        return 1;
    }

    std::vector<std::string> scenarioNames = opt.scenarios;
    if (scenarioNames.empty()) {
        scenarioNames = registry.list();
    } else {
        for (const auto& name : scenarioNames) {
            if (!registry.has(name)) {
                std::fprintf(stderr, "Error: unknown scenario '%s'.\n", name.c_str());
                return 1;
            }
        }
    }

    std::string aircraftName = "aircraft";
    {
        std::error_code ec;
        auto p = std::filesystem::path(opt.aircraftPath).stem();
        if (!p.empty()) aircraftName = p.string();
    }

    const bool recordTraces = !opt.traceDir.empty() || !opt.htmlPath.empty();
    if (recordTraces && !opt.traceDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(opt.traceDir, ec);
    }

    std::vector<Trace> traces;
    std::vector<ScenarioRunResult> results;
    int grandTotalPassed = 0;
    int grandTotalPhases = 0;

    for (const auto& name : scenarioNames) {
        auto scenario = registry.create(name);
        if (!scenario) continue;

        std::printf("\n=== Scenario: %s ===\n", scenario->name().c_str());
        std::printf("%s\n\n", scenario->GetDescription().c_str());

        TraceRecorder rec;
        ScenarioResult r = runScenario(*scenario, opt.aircraftPath, aircraftName, recordTraces ? &rec : nullptr);

        std::printf("  Scenario '%s' result: %d/%d\n\n", scenario->name().c_str(), r.passed, r.total);

        ScenarioRunResult srr;
        srr.name = name;
        srr.passed = (r.passed == r.total);
        srr.phases = r.phaseDetails;
        results.push_back(srr);

        grandTotalPassed += r.passed;
        grandTotalPhases += r.total;

        if (recordTraces) {
            if (!opt.traceDir.empty()) {
                std::string fname = aircraftName + "_" + name + ".json";
                std::string path = (std::filesystem::path(opt.traceDir) / fname).string();
                rec.write(path);
            }
            traces.push_back(rec.trace());
        }
    }

    std::printf("=== Overall ===\n");
    std::printf("Scenarios run: %zu\n", scenarioNames.size());
    std::printf("Phases: %d/%d passed\n", grandTotalPassed, grandTotalPhases);

    if (!opt.summaryPath.empty()) {
        writeSummaryJson(opt.summaryPath, aircraftName, results);
    }

    if (recordTraces && !opt.htmlPath.empty()) {
        std::printf("\nGenerating HTML report...\n");
        std::ofstream hf(opt.htmlPath);
        if (hf) {
            HtmlReportOptions hopts;
            hopts.title = "F4Flight — " + aircraftName;
            generateHtmlReport(traces, hf, hopts);
            hf.close();
            std::printf("HTML report written: %s\n", opt.htmlPath.c_str());
        }
    }

    return (grandTotalPassed == grandTotalPhases && grandTotalPhases > 0) ? 0 : 1;
}
