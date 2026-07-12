// f4flight - Steering demo: AI-piloted aircraft flies a multi-leg flight
// plan with altitude and speed schedules.
//
// This is a FORWARD-LOOKING demo file. It exists separately from
// maneuver_test because:
//
//   - maneuver_test is a TEST RUNNER: it runs registered scenarios, reports
//     pass/fail, and exits. Its output is structured for parsing.
//
//   - steering_demo is an INTERACTIVE DEMO: it runs a single, hand-crafted
//     flight profile and prints a continuous state log meant for human
//     inspection. It demonstrates how to compose multiple behaviors (waypoint
//     following + altitude hold + speed hold) into a complete mission.
//
// Future scenarios that are too exploratory for the test framework (new
// approach procedures, formation flying, air-to-air refueling, etc.) should
// start life as a steering_demo variant, mature until the pass/fail criteria
// are clear, and then be promoted into a registered maneuver_test scenario.
//
// Usage:
//   steering_demo [aircraft.json]
//
// If no argument is given, the built-in F-16C notional config is used.

#include "f4flight/f4flight.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace f4flight;

int main(int argc, char** argv) {
    AircraftConfig cfg;
    if (argc >= 2) {
        auto result = json::readFile(argv[1], cfg);
        if (!result.ok) {
            std::fprintf(stderr, "Failed to load %s:\n", argv[1]);
            for (auto const& e : result.errors) std::fprintf(stderr, "  %s\n", e.c_str());
            return 1;
        }
        std::printf("Loaded aircraft from %s\n", argv[1]);
    } else {
        std::printf("Using built-in F-16C notional config\n");
        cfg = config::makeF16CConfig();
    }

    // Build a 4-waypoint square circuit in the world (NED, Z-down).
    // Side length scales to give a ~5-minute per-leg flight at the cruise
    // speed we'll use.
    const double ALT_FT = 10000.0;
    const double SIDE_FT = 30.0 * NM_TO_FT;  // 30 NM sides
    const double Z = -ALT_FT;
    std::vector<Vec3> wps = {
        { SIDE_FT, 0.0,      Z },
        { SIDE_FT, SIDE_FT,  Z },
        { 0.0,      SIDE_FT,  Z },
        { 0.0,      0.0,      Z },
    };

    FlightModel fm;
    fm.init(cfg, ALT_FT, 300.0 * KNOTS_TO_FTPSEC, 0.0, true);

    SteeringController sc;
    sc.setMaxBankAngle_deg(45.0);
    sc.setMaxGs(cfg.geometry.maxGs);

    // Compose three behaviors into a complete mission:
    //   Horizontal: SteerToWaypoint (advance through the 4-waypoint circuit)
    //   Vertical:   AltitudeHold    (climb/descend/level as needed)
    //   Throttle:   SpeedHold       (maintain 300 kts in level flight)
    const double CRUISE_KTS = 300.0;
    const double CAPTURE_FT = 3000.0;   // waypoint capture radius

    sc.setHorizontalBehavior(
        std::make_unique<SteerToWaypoint>(wps, CAPTURE_FT));
    sc.setVerticalBehavior(
        std::make_unique<AltitudeHold>(ALT_FT));
    sc.setThrottleBehavior(
        std::make_unique<SpeedHold>(CRUISE_KTS));

    std::printf("\n=== f4flight steering demo: %s ===\n", cfg.name.c_str());
    std::printf("Mission: 4-waypoint square circuit, %d ft, %d kts\n",
                (int)ALT_FT, (int)CRUISE_KTS);
    std::printf("Waypoints:\n");
    for (size_t i = 0; i < wps.size(); ++i) {
        std::printf("  %zu: (%.1f, %.1f) nm\n", i + 1,
                    wps[i].x / NM_TO_FT, wps[i].y / NM_TO_FT);
    }
    std::printf("\n");

    const double dt = 1.0 / 60.0;
    double simTime = 0.0;
    double nextPrint = 0.0;
    const double MAX_TIME = 1800.0;  // 30 minutes hard cap

    std::printf("%-7s %10s %10s %10s %8s %6s %8s %8s %8s %5s %5s\n",
                "t(s)", "x(nm)", "y(nm)", "alt(ft)", "vcas",
                "hdg", "throt", "bank", "pitch", "G", "wp");

    while (simTime < MAX_TIME) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
        simTime += dt;

        if (simTime >= nextPrint) {
            const auto& s = fm.state();
            const SteerToWaypoint* nav =
                dynamic_cast<const SteerToWaypoint*>(sc.horizontalBehavior());
            std::printf("%-7.0f %10.2f %10.2f %10.0f %8.1f %6.1f %8.2f %8.1f %8.1f %5.2f %5d\n",
                        simTime,
                        s.kin.x / NM_TO_FT, s.kin.y / NM_TO_FT,
                        -s.kin.z, s.vcas, s.kin.psi * RTD,
                        input.throttle, s.kin.phi * RTD, s.kin.theta * RTD,
                        s.loads.nzcgs,
                        nav ? static_cast<int>(nav->currentWaypoint()) + 1 : 0);
            nextPrint += 30.0;
        }

        // Stop early if all waypoints are captured.
        const SteerToWaypoint* nav =
            dynamic_cast<const SteerToWaypoint*>(sc.horizontalBehavior());
        if (nav && nav->allCaptured()) {
            std::printf("\nAll waypoints captured at t=%.0f s. Mission complete.\n", simTime);
            break;
        }
    }

    std::printf("\nFinal state: alt %.0f ft, vcas %.1f kts, fuel %.1f lbs\n",
                -fm.state().kin.z, fm.state().vcas, fm.state().fuel.fuel_lbs);
    return 0;
}
