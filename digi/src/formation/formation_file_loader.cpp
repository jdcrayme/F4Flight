// f4flight - digi/formation/formation_file_loader.cpp
//
// FreeFalcon formdat.fil loader for FormationTable.
//
// Port of FreeFalcon's ACFormationData constructor (formdata.cpp).
// Reads the flat-text formation definition file and registers each
// formation in the FormationTable by its FreeFalcon formNum.

#include "f4flight/digi/formation/formation_geometry.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace f4flight {
namespace digi {
namespace formation {

// NM to feet conversion (matches FreeFalcon's NM_TO_FT).
static constexpr double kNmToFt = 6076.11549;

int FormationTable::loadFromFile(const char* filename) {
    std::FILE* f = std::fopen(filename, "r");
    if (!f) {
        return -1;
    }

    // Helper: read the next whitespace-separated token from the file.
    char buf[256];

    // Read numFormations
    if (std::fscanf(f, "%255s", buf) != 1) {
        std::fclose(f);
        return -1;
    }
    int numFormations = std::atoi(buf);
    if (numFormations <= 0 || numFormations > 100) {
        std::fclose(f);
        return -1;
    }

    int loaded = 0;
    for (int i = 0; i < numFormations; ++i) {
        // Read: num4Slots num2Slots formNum formationName
        if (std::fscanf(f, "%255s", buf) != 1) break;
        int num4Slots = std::atoi(buf);
        if (std::fscanf(f, "%255s", buf) != 1) break;
        int num2Slots = std::atoi(buf);
        if (std::fscanf(f, "%255s", buf) != 1) break;
        int formNum = std::atoi(buf);
        if (std::fscanf(f, "%255s", buf) != 1) break;
        // formationName — skip (not used by F4Flight, stored in buf)
        (void)buf;

        // Clamp num4Slots to kMaxFormationSlots
        int slotsToRead = num4Slots;
        if (slotsToRead > kMaxFormationSlots) {
            slotsToRead = kMaxFormationSlots;
        }

        Formation form{};
        for (int j = 0; j < slotsToRead; ++j) {
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double relAzDeg = std::atof(buf);
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double relElDeg = std::atof(buf);
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double rangeNm = std::atof(buf);

            form[j] = PositionData{
                relAzDeg * M_PI / 180.0,   // radians
                relElDeg * M_PI / 180.0,   // radians
                rangeNm * kNmToFt           // feet
            };
        }

        // Skip any extra 4-ship slots beyond kMaxFormationSlots
        for (int j = slotsToRead; j < num4Slots; ++j) {
            std::fscanf(f, "%255s", buf);  // relAz
            std::fscanf(f, "%255s", buf);  // relEl
            std::fscanf(f, "%255s", buf);  // range
        }

        // Read 2-ship data (if num2Slots > 0)
        if (num2Slots > 0) {
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double relAzDeg = std::atof(buf);
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double relElDeg = std::atof(buf);
            if (std::fscanf(f, "%255s", buf) != 1) { std::fclose(f); return loaded; }
            double rangeNm = std::atof(buf);

            // Store the 2-ship position in a separate formation entry.
            // 2-ship formations have slot 0 = lead and slot 1 = wingman.
            // We store it with formNum + 1000 offset so the host can look
            // up 2-ship formations distinctly from 4-ship formations.
            Formation twoShipForm{};
            twoShipForm[0] = PositionData{0.0, 0.0, 0.0};
            twoShipForm[1] = PositionData{
                relAzDeg * M_PI / 180.0,
                relElDeg * M_PI / 180.0,
                rangeNm * kNmToFt
            };
            registerFormationById(formNum + 1000, twoShipForm);
        }

        registerFormationById(formNum, form);
        ++loaded;
    }

    std::fclose(f);
    return loaded;
}

} // namespace formation
} // namespace digi
} // namespace f4flight
