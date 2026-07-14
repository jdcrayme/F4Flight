// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// json_io.h
//
// Minimal JSON serialization for AircraftConfig.
//
// We hand-roll a small JSON writer (since we control the output format, no
// escaping concerns) and a small JSON reader (since we need to parse the
// files we wrote plus allow hand-edits).
//
// The JSON format is a direct 1:1 mapping of the AircraftConfig struct.
// Arrays of doubles are written as JSON arrays of numbers.

#pragma once

#include "f4flight/flight/aircraft_config.h"

#include <string>
#include <vector>

namespace f4flight::json {

struct IoResult {
    bool ok = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

// Serialize an AircraftConfig to a JSON string.
std::string write(const AircraftConfig& cfg);

// Write an AircraftConfig to a JSON file. Returns true on success.
bool writeFile(const AircraftConfig& cfg, const std::string& path);

// Parse a JSON string into an AircraftConfig. Returns IoResult; on success
// `result.ok` is true and `cfg` is populated.
IoResult read(const std::string& json, AircraftConfig& cfg);

// Read a JSON file into an AircraftConfig. Returns true on success.
IoResult readFile(const std::string& path, AircraftConfig& cfg);

} // namespace f4flight::json
