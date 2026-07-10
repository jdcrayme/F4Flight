// f4flight - Example: a simple console simulation that prints the aircraft
// state every second for 30 seconds.
//
// Usage:
//   simple_sim [aircraft.json]
//
// If no argument is given, the built-in F-16C notional config is used.
// To use real Falcon 4 data, first convert it with dat2json:
//   dat2json f16bk50.dat f16bk50.json
//   simple_sim f16bk50.json

#include "f4flight/f4flight.h"
#include "f4flight/json_io.h"

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
        // Validate that this is a real aircraft (not e.g. vapor.dat)
        if (cfg.aero.mach.empty() || cfg.aero.clift.empty()) {
            std::fprintf(stderr, "Error: %s does not contain valid aircraft aero data\n", path.c_str());
            return 1;
        }
    } else {
        std::printf("Using built-in F-16C notional config\n");
        cfg = config::makeF16CConfig();
    }

    // Create and initialise the flight model: 5000 ft, 500 ft/s, heading North,
    // airborne.
    FlightModel fm;
    fm.init(cfg, 5000.0, 500.0, 0.0, true);

    std::printf("\n=== f4flight simulation: %s ===\n", cfg.name.c_str());
    std::printf("%-6s %10s %10s %10s %10s %8s %8s %8s %10s\n",
                "t(s)", "alt(ft)", "vt(fps)", "vcas(kts)", "mach",
                "alpha(d)", "theta(d)", "G", "thrust(lbf)");

    PilotInput input;
    input.throttle = 0.75;  // 75% MIL
    input.pstick   = 0.0;
    input.rstick   = 0.0;
    input.ypedal   = 0.0;
    input.gearHandle = -1.0;  // gear up

    const double dt = 1.0 / 60.0;
    double simTime = 0.0;
    double nextPrint = 0.0;

    while (simTime < 30.0) {
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
        simTime += dt;

        if (simTime >= nextPrint) {
            const auto& s = fm.state();
            const double alt = -s.kin.z;
            const double thrust_lbf = s.engine.thrust * s.fuel.mass_slugs;
            std::printf("%-6.1f %10.1f %10.1f %10.1f %10.3f %8.2f %8.2f %8.2f %10.0f\n",
                        simTime, alt, s.kin.vt, s.vcas, s.mach,
                        s.aero.alpha_deg,
                        s.kin.theta * RTD,
                        s.loads.nzcgs,
                        thrust_lbf);
            nextPrint += 1.0;
        }
    }

    std::printf("\nFinal fuel: %.1f lbs (burned %.1f lbs)\n",
                fm.state().fuel.fuel_lbs,
                cfg.geometry.internalFuel_lbs - fm.state().fuel.fuel_lbs);
    return 0;
}
