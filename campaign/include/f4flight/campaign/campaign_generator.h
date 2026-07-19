// f4flight - campaign/include/f4flight/campaign/campaign_generator.h
//
// CampaignGenerator procedurally builds a complete campaign scenario in memory,
// including coastline/mountain terrain, road graphs, objectives, runways, and units.

#pragma once

#include "campaign_state.h"

namespace f4flight {
namespace campaign {

class CampaignGenerator {
public:
    // Generates a fully populated, ready-to-run 32x32 campaign scenario
    // with Blue and Red forces ready to clash.
    static CampaignState generateDefaultTheater();
};

} // namespace campaign
} // namespace f4flight
