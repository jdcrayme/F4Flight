// f4flight - Steering demo: an AI-piloted F-16 flies a 4-waypoint circuit.
//
// Usage:
//   steering_demo [aircraft.json]
//
// If no argument is given, the built-in F-16C notional config is used.
// To use real Falcon 4 data, first convert it with dat2json:
//   dat2json f16bk50.dat f16bk50.json
//   steering_demo f16bk50.json

#include "f4flight/f4flight.h"

#include <cstdio>
#include <string>

using namespace f4flight;

int main(int argc, char** argv) {
    AircraftConfig cfg;
    if (argc >= 2) {
        std::string path = argv[1];
        std::printf("Loading aircraft from %s\n", path.c_str());
        auto result = json::readFile(path, cfg);
        if (!result.ok) {
            std::fprintf(stderr, "Failed to load %s:\n", path.c_str());
            for (auto const& e : result.errors) std::fprintf(stderr, "  %s\n", e.c_str());
            return 1;
        }
    } else {
        std::printf("Using built-in F-16C notional config\n");
        cfg = config::makeF16CConfig();
    }

    // Set up a 4-waypoint circuit. Each waypoint is (x_north_ft, y_east_ft,
    // z_down_ft). Altitude = -z, so z=-10000 means 10,000 ft.
    // We'll fly a 20 nm square pattern at 10,000 ft.
    // 20 nm = 121,523 ft.
    const double R = 121523.0;
    const double ALT = -10000.0;
    std::vector<Vec3> wps = {
        {  R,       0.0,    ALT },   // north
        {  R,       R,      ALT },   // north-east corner
        {  0.0,     R,      ALT },   // east
        {  0.0,     0.0,    ALT },   // back to start
    };

    // Create the flight model at the first waypoint, 350 kts (well above stall),
    // heading North.
    FlightModel fm;
    fm.init(cfg, 10000.0, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);

    // Set up the steering controller
    SteeringController sc;
    sc.setMode(SteeringMode::Waypoint);
    sc.setWaypoints(wps);
    sc.setMaxBankAngle_deg(45.0);
    sc.setMaxPitchAngle_deg(20.0);
    sc.setWaypointCaptureRadius_ft(3000.0);  // 3000 ft capture radius

    // Configure the steering goal: hold 350 kts, follow the waypoints
    SteeringGoal goal;
    goal.hasSpeed = true;
    goal.speed_kts = 350.0;
    sc.setGoal(goal);

    std::printf("\n=== f4flight steering demo: %s ===\n", cfg.name.c_str());
    std::printf("Flying a 4-waypoint square circuit (20 nm sides, 10,000 ft)\n\n");
    std::printf("%-6s %10s %10s %8s %8s %8s %8s %8s %8s %6s %6s %6s\n",
                "t(s)", "alt(ft)", "vcas(kts)", "hdg(d)", "bank(d)", "pitch(d)",
                "pstick", "rstick", "throt", "pG", "wpN", "wpDist");

    const double dt = 1.0 / 60.0;
    double simTime = 0.0;
    double nextPrint = 0.0;
    const double totalSimTime = 1200.0;  // 20 minutes

    while (simTime < totalSimTime) {
        // Compute the steering command
        PilotInput input = sc.compute(fm.state(), dt, 0.0);

        // Update the flight model
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
        simTime += dt;

        if (simTime >= nextPrint) {
            const auto& s = fm.state();
            const double alt = -s.kin.z;
            const double heading = s.kin.psi * RTD;
            const double bank = s.kin.phi * RTD;
            const double pitch = s.kin.theta * RTD;

            // Distance to current waypoint
            double wpDist = 0.0;
            if (sc.currentWaypointIndex() < wps.size()) {
                const auto& wp = wps[sc.currentWaypointIndex()];
                const double dx = wp.x - s.kin.x;
                const double dy = wp.y - s.kin.y;
                wpDist = std::sqrt(dx * dx + dy * dy);
            }

            std::printf("%-6.0f %10.0f %10.1f %8.1f %8.1f %8.1f %8.2f %8.2f %8.2f %6.1f %6zu %6.0f\n",
                        simTime, alt, s.vcas,
                        heading, bank, pitch,
                        input.pstick, input.rstick, input.throttle,
                        s.loads.nzcgs,
                        sc.currentWaypointIndex(),
                        wpDist);
            nextPrint += 30.0;  // print every 30 seconds
        }
    }

    std::printf("\nFinal state:\n");
    const auto& s = fm.state();
    std::printf("  Position:   (%.0f, %.0f, %.0f) ft\n", s.kin.x, s.kin.y, s.kin.z);
    std::printf("  Altitude:   %.0f ft\n", -s.kin.z);
    std::printf("  Heading:    %.1f deg\n", s.kin.psi * RTD);
    std::printf("  Airspeed:   %.1f kts KCAS\n", s.vcas);
    std::printf("  Mach:       %.3f\n", s.mach);
    std::printf("  Fuel:       %.1f lbs (burned %.1f lbs)\n",
                s.fuel.fuel_lbs,
                cfg.geometry.internalFuel_lbs - s.fuel.fuel_lbs);
    return 0;
}
