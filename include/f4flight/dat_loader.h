// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// dat_loader.h
//
// Parser for the legacy FreeFalcon .dat aircraft definition file format.
//
// The .dat format is a flat sequence of whitespace-separated tokens with
// '#' line comments. Some sections (the AuxAeroData block at the bottom) use
// a key/value form ("normSpoolRate 3.0"); the top sections are positional.
//
// This parser extracts ONLY the fields that the f4flight library actually
// consumes. Everything else (sounds, vapor data, hardpoint groupings, AI
// behaviour knobs, etc.) is silently skipped. The extracted data is written
// into an AircraftConfig that can then be serialized to JSON.

#pragma once

#include "f4flight/aircraft_config.h"

#include <string>
#include <vector>

namespace f4flight::dat {

// Result of parsing a .dat file.
struct ParseResult {
    AircraftConfig config;
    std::vector<std::string> warnings;   // non-fatal issues (skipped sections, etc.)
    std::vector<std::string> errors;     // fatal issues (would not produce a usable config)
    bool ok = false;
};

// Parse a Falcon 4 .dat file from disk. Returns a ParseResult; check
// `result.ok` before using `result.config`.
ParseResult loadFile(const std::string& path);

// Parse a Falcon 4 .dat file from an in-memory string (mainly for tests).
ParseResult loadString(const std::string& contents, const std::string& sourceName = "<string>");

} // namespace f4flight::dat
