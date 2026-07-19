// f4flight - tools/viz/cmap_viz.cpp
//
// Standalone CLI utility to run the procedural campaign simulation for several
// hours, record each frame, and write the self-contained interactive HTML dashboard.

#include "f4flight/campaign/campaign_generator.h"
#include "f4flight/campaign/campaign_io.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace f4flight {
namespace campaign {

// Forward declaration of HTML template strings
extern const char* kCampaignHtmlHead;
extern const char* kCampaignHtmlTail1;
extern const char* kCampaignHtmlTail2;
extern const char* kCampaignHtmlTail3;
extern const char* kCampaignHtmlTail4;
extern const char* kCampaignHtmlTail5;

void generateCampaignHtml(const std::string& title, const std::string& serializedFrames, std::ostream& out) {
    out << kCampaignHtmlHead;
    out << "\nconst CAMPAIGN_TITLE = \"" << title << "\";\n";
    out << "const CAMPAIGN_FRAMES = " << serializedFrames << ";\n";
    out << kCampaignHtmlTail1;
    out << kCampaignHtmlTail2;
    out << kCampaignHtmlTail3;
    out << kCampaignHtmlTail4;
    out << kCampaignHtmlTail5;
}

}
}

using namespace f4flight;
using namespace f4flight::campaign;

int main(int argc, char** argv) {
    std::string outPath = "campaign_report.html";
    std::string title = "Falcon 4 Modular Campaign Map Visualizer";
    int numFrames = 60;
    double dt = 180.0; // 3 minutes per tick (Total: 3 hours of combat simulation)

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-o" || a == "--output") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: -o requires a path\n");
                return 1;
            }
            outPath = argv[++i];
        } else if (a == "--title") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --title requires a title string\n");
                return 1;
            }
            title = argv[++i];
        } else if (a == "--frames") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: --frames requires an integer\n");
                return 1;
            }
            numFrames = std::atoi(argv[++i]);
        }
    }

    std::printf("Running procedural Falcon 4 Campaign Simulation (%d frames, %g seconds per step)...\n", numFrames, dt);

    // Generate starting campaign state
    CampaignState state = CampaignGenerator::generateDefaultTheater();

    // Time-series recording
    std::string serializedFrames = "[";

    for (int f = 0; f < numFrames; ++f) {
        if (f > 0) serializedFrames += ",\n";
        serializedFrames += CampaignIO::serialize(state);

        // Advance simulation
        state.tick(dt);
    }
    serializedFrames += "]";

    std::printf("Simulation complete. Writing HTML dashboard to: %s...\n", outPath.c_str());

    std::ofstream outFile(outPath, std::ios::binary);
    if (!outFile) {
        std::fprintf(stderr, "Error: Could not open output file: %s\n", outPath.c_str());
        return 2;
    }

    generateCampaignHtml(title, serializedFrames, outFile);
    outFile.close();

    std::printf("Campaign visualization successfully written! (Open '%s' in any web browser to view the dynamic map)\n", outPath.c_str());
    return 0;
}
