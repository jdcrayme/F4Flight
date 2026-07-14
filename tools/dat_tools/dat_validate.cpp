// dat_validate - Load a JSON aircraft file and run AircraftConfig::validate().
//
// Prints every issue found (does not short-circuit on the first error) so a
// host loading a data file gets a complete diagnostic in one pass.
//
// Exit codes:
//   0  validation passed (no errors; warnings may be present)
//   1  validation failed (one or more errors)
//   2  could not load the file (parse error, missing file, etc.)
//
// Usage:
//   dat_validate <input.json>
//   dat_validate <input.json> --quiet     # only print on failure
//   dat_validate <input.json> --strict    # treat warnings as errors

#include "f4flight/flight/json_io.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <input.json> [--quiet] [--strict]\n", argv[0]);
        return 2;
    }

    const std::string inputPath = argv[1];
    bool quiet = false;
    bool strict = false;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--quiet")  quiet = true;
        else if (a == "--strict") strict = true;
        else {
            std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
            return 2;
        }
    }

    f4flight::AircraftConfig cfg;
    auto io = f4flight::json::readFile(inputPath, cfg);
    if (!io.ok) {
        std::fprintf(stderr, "ERROR: Could not load %s\n", inputPath.c_str());
        for (auto const& e : io.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 2;
    }

    auto report = cfg.validate();

    if (!quiet || !report.ok() || (strict && report.hasWarnings())) {
        std::printf("Aircraft: %s\n", cfg.name.c_str());
        std::printf("  Source:   %s\n", cfg.sourceFile.c_str());
        std::printf("  Errors:   %zu\n", report.errorCount());
        std::printf("  Warnings: %zu\n", report.warningCount());
        std::printf("\n");
        if (!report.format().empty()) {
            std::printf("%s\n", report.format().c_str());
        }
    }

    if (!report.ok()) return 1;
    if (strict && report.hasWarnings()) return 1;
    return 0;
}
