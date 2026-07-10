// dat2json - Convert a Falcon 4 .dat aircraft file to f4flight JSON format.
//
// Usage:
//   dat2json <input.dat> <output.json>

#include "f4flight/dat_loader.h"
#include "f4flight/json_io.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "Usage: %s <input.dat> <output.json>\n", argv[0]);
        return 1;
    }

    const std::string inputPath  = argv[1];
    const std::string outputPath = argv[2];

    std::printf("Converting %s -> %s\n", inputPath.c_str(), outputPath.c_str());

    auto result = f4flight::dat::loadFile(inputPath);
    if (!result.ok) {
        std::fprintf(stderr, "ERROR: Failed to parse %s\n", inputPath.c_str());
        for (auto const& e : result.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 2;
    }

    if (!result.warnings.empty()) {
        std::printf("Warnings (%zu):\n", result.warnings.size());
        for (auto const& w : result.warnings) std::printf("  %s\n", w.c_str());
    }

    if (!f4flight::json::writeFile(result.config, outputPath)) {
        std::fprintf(stderr, "ERROR: Failed to write %s\n", outputPath.c_str());
        return 3;
    }

    auto const& c = result.config;
    std::printf("\nConverted aircraft: %s\n", c.name.c_str());
    std::printf("  Empty weight:    %.1f lbs\n", c.geometry.emptyWeight_lbs);
    std::printf("  Wing area:       %.1f ft^2\n", c.geometry.area_ft2);
    std::printf("  Internal fuel:   %.1f lbs\n", c.geometry.internalFuel_lbs);
    std::printf("  Span:            %.1f ft\n", c.geometry.span_ft);
    std::printf("  Max Gs:          %.1f\n", c.geometry.maxGs);
    std::printf("  AOA limits:      %.1f to %.1f deg\n",
                c.geometry.aoaMin_deg, c.geometry.aoaMax_deg);
    std::printf("  Gear points:     %zu\n", c.geometry.gear.size());
    std::printf("  Engines:         %d (type %d)\n", c.aux.nEngines, c.aux.typeEngine);
    std::printf("  Aero table:      %zu mach x %zu alpha\n",
                c.aero.mach.size(), c.aero.alpha_deg.size());
    std::printf("  Engine table:    %zu alt x %zu mach\n",
                c.engine.alt_ft.size(), c.engine.mach.size());
    std::printf("  Roll table:      %zu alpha x %zu qbar\n",
                c.rollCmd.alpha_deg.size(), c.rollCmd.qbar.size());
    std::printf("  Has AB:          %s\n", c.engine.hasAB() ? "yes" : "no");

    if (!c.engine.thrust_mil.empty() && !c.engine.thrust_ab.empty()) {
        std::printf("  Sea-level MIL:   %.0f lbf\n", c.engine.thrust_mil[0]);
        std::printf("  Sea-level AB:    %.0f lbf\n", c.engine.thrust_ab[0]);
    }

    std::printf("\nWrote %s\n", outputPath.c_str());
    return 0;
}
