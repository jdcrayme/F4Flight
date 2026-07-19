// f4flight - campaign/include/f4flight/campaign/campaign_io.h
//
// CampaignIO provides serialization and deserialization functions to read/write
// the entire campaign map state and active unit status to/from structured JSON.

#pragma once

#include "campaign_state.h"
#include <string>

namespace f4flight {
namespace campaign {

class CampaignIO {
public:
    // Serializes the CampaignState to a compact JSON string.
    static std::string serialize(const CampaignState& state);

    // Writes the campaign state to a JSON file.
    static bool serializeToFile(const CampaignState& state, const std::string& filepath);

    // Parses a JSON string and populates the CampaignState.
    static bool deserialize(const std::string& json, CampaignState& state);

    // Loads campaign state from a JSON file.
    static bool deserializeFromFile(const std::string& filepath, CampaignState& state);
};

} // namespace campaign
} // namespace f4flight
