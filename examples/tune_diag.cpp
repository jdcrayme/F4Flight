// f4flight - Tuning diagnostic: prints detailed state every 5 seconds to
// help diagnose the altitude hold issue and tune the steering PID gains.
//
// Usage:
//   tune_diag [aircraft.json]

#include "f4flight/f4flight.h"
#include "f4flight/json_io.h"

#include <cstdio>
#include <string>

using namespace f4flight;

int main(int argc, char** argv) {
    AircraftConfig cfg;
    if (argc >= 2) {
        auto result = json::readFile(argv[1], cfg);
        if (!result.ok) {
            std::fprintf(stderr, "Failed to load %s\n", argv[1]);
            return 1;
        }
    } else {
        cfg = config::makeF16CConfig();
    }

    // Start at 10,000 ft, 350 kts, heading North
    FlightModel fm;
    fm.init(cfg, 10000.0, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);

    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);
    sc.setMaxBankAngle_deg(30.0);
    sc.setMaxGs(cfg.geometry.maxGs);

    SteeringGoal goal;
    goal.hasHeading = true;
    goal.heading_rad = 0.0;
    goal.hasAltitude = true;
    goal.altitude_ft = 10000.0;
    goal.hasSpeed = true;
    goal.speed_kts = 350.0;
    sc.setGoal(goal);

    std::printf("=== Tuning diagnostic: %s ===\n", cfg.name.c_str());
    std::printf("Goal: alt=10000ft, hdg=0deg, spd=350kts\n\n");
    std::printf("%-6s %8s %8s %8s %8s %8s %7s %7s %7s %7s %7s %7s %8s %8s %8s\n",
                "t(s)", "alt(ft)", "altErr", "vz(fps)", "vcas", "mach",
                "alpha", "theta", "pitch", "G", "nzCmd", "throt",
                "cl", "cd", "thrst");

    const double dt = 1.0 / 60.0;
    double simTime = 0.0;
    double nextPrint = 0.0;

    while (simTime < 120.0) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
        simTime += dt;

        if (simTime >= nextPrint) {
            const auto& s = fm.state();
            const double alt = -s.kin.z;
            const double altErr = 10000.0 - alt;
            const double vz = -s.kin.zdot;  // climb rate, positive = up
            const double thrust_lbf = s.engine.thrust * s.fuel.mass_slugs;

            std::printf("%-6.0f %8.0f %8.0f %8.1f %8.1f %8.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %8.3f %8.4f %8.0f\n",
                        simTime, alt, altErr, vz, s.vcas, s.mach,
                        s.aero.alpha_deg, s.kin.theta * RTD,
                        input.pstick, s.loads.nzcgs,
                        s.fcs.aoacmd,
                        input.throttle,
                        s.aero.cl, s.aero.cd,
                        thrust_lbf);
            nextPrint += 5.0;
        }
    }

    return 0;
}
