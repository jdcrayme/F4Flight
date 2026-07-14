// f4flight - tools/trace2svg.cpp
//
// Convert a trace JSON file to SVG panels.
//
// Usage:
//   trace2svg <trace.json> <output.svg>          # one SVG with all phases
//   trace2svg <trace.json> <output.svg> --split  # one SVG per phase
//
// Produces two panels per SVG:
//   1. Top-down view (ground track, colored by aircraft, phase start markers)
//   2. Side profile (altitude + speed vs time, phase labels centered on ranges)
//
// Key design decisions (informed by user feedback):
//   - Tracks are SPLIT at phase boundaries. Each phase re-initializes the
//     aircraft to a new position, so connecting phase N's end to phase N+1's
//     start produces nonsense lines. We draw each phase as a separate segment.
//   - Runway marker only appears for ground-ops scenarios, drawn as an
//     oriented line with RWY_Start / RWY_End labels (not a cross).
//   - Phase labels are centered on their time range, with alternating
//     vertical offsets to prevent overlap.
//   - Path color is based on the aircraft name (deterministic hash), not the
//     AI mode — this makes formation flying and multi-aircraft comparison
//     possible in the future.
//   - Time axis labels are auto-spaced to avoid overlap.

#include "trace.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <set>

using namespace f4flight;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Deterministic color from a string (aircraft name → consistent color).
// Uses a simple hash → HSV → RGB. Same aircraft always gets the same color.
static std::string aircraftColor(const std::string& name) {
    uint32_t hash = 5381;
    for (char c : name) hash = ((hash << 5) + hash) + (uint8_t)c;
    double hue = (hash % 360) / 360.0;
    double s = 0.7, v = 0.9;
    // HSV → RGB
    double r, g, b;
    int i = (int)(hue * 6);
    double f = hue * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);
    switch (i % 6) {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        default: r=v; g=p; b=q; break;
    }
    char buf[8];
    std::snprintf(buf, sizeof(buf), "#%02X%02X%02X",
                  (int)(r * 255), (int)(g * 255), (int)(b * 255));
    return buf;
}

// SVG-escape a string
static std::string esc(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '&': out += "&amp;"; break;
            case '"': out += "&quot;"; break;
            default: out += c;
        }
    }
    return out;
}

// Check if a phase is a ground-ops phase (takeoff/landing)
static bool isGroundOpsPhase(const std::string& phaseName) {
    return phaseName == "Takeoff" || phaseName == "Landing" ||
           phaseName.find("Takeoff") != std::string::npos ||
           phaseName.find("Landing") != std::string::npos ||
           phaseName.find("groundops") != std::string::npos;
}

// Check if any phase in the trace involves ground ops
static bool hasGroundOps(const Trace& trace) {
    for (const auto& p : trace.phases) {
        if (isGroundOpsPhase(p.name)) return true;
    }
    // Also check frames for ground-ops modes
    for (const auto& f : trace.frames) {
        if (f.mode == "Takeoff" || f.mode == "Landing") return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Compute bounds of the trace (per-phase for split mode)
// ---------------------------------------------------------------------------

struct Bounds {
    double minX, maxX, minY, maxY;
    double minAlt, maxAlt;
    double minT, maxT;
};

static Bounds computeBounds(const Trace& trace, int phaseIdx = -1) {
    Bounds b{};
    b.minX = b.maxX = 0;
    b.minY = b.maxY = 0;
    b.minAlt = b.maxAlt = 0;
    b.minT = b.maxT = 0;
    bool first = true;

    for (const auto& f : trace.frames) {
        // Filter by phase if phaseIdx specified
        if (phaseIdx >= 0 && phaseIdx < (int)trace.phases.size()) {
            if (f.t < trace.phases[phaseIdx].start_s ||
                f.t > trace.phases[phaseIdx].end_s) continue;
        }

        double alt = -f.z;
        if (first) {
            b.minX = b.maxX = f.x;
            b.minY = b.maxY = f.y;
            b.minAlt = b.maxAlt = alt;
            b.minT = b.maxT = f.t;
            first = false;
        } else {
            b.minX = std::min(b.minX, f.x);
            b.maxX = std::max(b.maxX, f.x);
            b.minY = std::min(b.minY, f.y);
            b.maxY = std::max(b.maxY, f.y);
            b.minAlt = std::min(b.minAlt, alt);
            b.maxAlt = std::max(b.maxAlt, alt);
            b.minT = std::min(b.minT, f.t);
            b.maxT = std::max(b.maxT, f.t);
        }
    }
    // Include threat positions
    for (const auto& f : trace.frames) {
        if (phaseIdx >= 0 && phaseIdx < (int)trace.phases.size()) {
            if (f.t < trace.phases[phaseIdx].start_s ||
                f.t > trace.phases[phaseIdx].end_s) continue;
        }
        for (const auto& t : f.threats) {
            b.minX = std::min(b.minX, t.x);
            b.maxX = std::max(b.maxX, t.x);
            b.minY = std::min(b.minY, t.y);
            b.maxY = std::max(b.maxY, t.y);
        }
    }
    double padX = (b.maxX - b.minX) * 0.08 + 200;
    double padY = (b.maxY - b.minY) * 0.08 + 200;
    double padAlt = (b.maxAlt - b.minAlt) * 0.08 + 200;
    b.minX -= padX; b.maxX += padX;
    b.minY -= padY; b.maxY += padY;
    b.minAlt = std::max(0.0, b.minAlt - padAlt);
    b.maxAlt += padAlt;
    if (b.maxT == b.minT) b.maxT = b.minT + 1.0;
    return b;
}

// ---------------------------------------------------------------------------
// Get frame indices for each phase (for splitting tracks)
// ---------------------------------------------------------------------------

struct PhaseRange {
    int startFrame;  // first frame index in this phase
    int endFrame;    // last frame index in this phase
};

static std::vector<PhaseRange> computePhaseRanges(const Trace& trace) {
    std::vector<PhaseRange> ranges;
    if (trace.phases.empty()) {
        if (!trace.frames.empty()) {
            ranges.push_back({0, (int)trace.frames.size() - 1});
        }
        return ranges;
    }

    int fi = 0;
    for (size_t pi = 0; pi < trace.phases.size(); ++pi) {
        const auto& p = trace.phases[pi];
        int start = fi;
        while (fi < (int)trace.frames.size() && trace.frames[fi].t <= p.end_s) {
            fi++;
        }
        int end = fi - 1;
        if (end < start) end = start;
        ranges.push_back({start, end});
    }
    return ranges;
}

// ---------------------------------------------------------------------------
// Generate a single SVG panel set for one phase (or all phases)
// ---------------------------------------------------------------------------

static void generateSVG(const Trace& trace, const std::string& outPath,
                         int phaseIdx = -1) {
    Bounds b = computeBounds(trace, phaseIdx);

    // SVG dimensions
    const int svgW = 1200;
    const int svgH = 1600;
    const int panelMargin = 50;
    const int topPanelH = 680;
    const int bottomPanelH = 680;
    const int panelW = svgW - 2 * panelMargin;

    // Coordinate transforms (equal aspect for top-down)
    double spanX = b.maxX - b.minX;
    double spanY = b.maxY - b.minY;
    double spanTD = std::max(spanX, spanY);
    if (spanTD < 1) spanTD = 100;
    double scaleTD = panelW / spanTD;
    double tdOffsetX = panelMargin + (panelW - spanX * scaleTD) / 2;
    double tdOffsetY = panelMargin + 50 + (panelW - spanY * scaleTD) / 2;

    auto toScreenTD_X = [&](double x) { return tdOffsetX + (x - b.minX) * scaleTD; };
    auto toScreenTD_Y = [&](double y) { return tdOffsetY + (y - b.minY) * scaleTD; };

    // Side profile
    double spTop = panelMargin + topPanelH + 80;
    double scaleAlt = bottomPanelH / std::max(1.0, b.maxAlt - b.minAlt);
    double scaleTime = panelW / std::max(1.0, b.maxT - b.minT);
    auto toScreenSP_X = [&](double t) { return panelMargin + (t - b.minT) * scaleTime; };
    auto toScreenSP_Y = [&](double alt) {
        return spTop + (b.maxAlt - alt) * scaleAlt;
    };

    // Phase ranges for splitting tracks
    auto phaseRanges = computePhaseRanges(trace);
    // If showing a single phase, only use that range
    if (phaseIdx >= 0 && phaseIdx < (int)phaseRanges.size()) {
        phaseRanges = {phaseRanges[phaseIdx]};
    }

    std::string aircraftCol = aircraftColor(trace.aircraft);

    std::string svg;
    svg.reserve(128 * 1024);

    // SVG header
    svg += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg += "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" + std::to_string(svgW) +
           "\" height=\"" + std::to_string(svgH) + "\" viewBox=\"0 0 " +
           std::to_string(svgW) + " " + std::to_string(svgH) + "\">\n";
    svg += "<style>text{font-family:monospace;font-size:11px} .title{font-size:16px;font-weight:bold}</style>\n";

    // Background
    svg += "<rect width=\"100%\" height=\"100%\" fill=\"#1a1a2e\"/>\n";

    // Title
    std::string title = trace.aircraft + " — " + trace.scenario;
    if (phaseIdx >= 0 && phaseIdx < (int)trace.phases.size()) {
        title += " — " + trace.phases[phaseIdx].name;
    }
    svg += "<text x=\"" + std::to_string(panelMargin) + "\" y=\"28\" class=\"title\" fill=\"white\">" +
           esc(title) + "  (" + std::to_string(trace.frames.size()) + " frames, " +
           std::to_string((int)(b.maxT - b.minT)) + "s)</text>\n";

    // ===== Top-down panel =====
    svg += "<text x=\"" + std::to_string(panelMargin) + "\" y=\"" +
           std::to_string(panelMargin + 25) + "\" class=\"title\" fill=\"#aaa\">Top-Down View (ground track — "
           + esc(trace.aircraft) + ")</text>\n";

    // Grid (auto-spaced)
    double gridSpacing = 5000;
    if (spanTD < 3000) gridSpacing = 500;
    else if (spanTD < 10000) gridSpacing = 1000;
    else if (spanTD < 50000) gridSpacing = 5000;
    else gridSpacing = 10000;
    svg += "<g stroke=\"#2a2a3e\" stroke-width=\"0.5\" fill=\"none\">\n";
    for (double gx = std::floor(b.minX / gridSpacing) * gridSpacing; gx <= b.maxX; gx += gridSpacing) {
        double sx = toScreenTD_X(gx);
        svg += "<line x1=\"" + std::to_string(sx) + "\" y1=\"" +
               std::to_string(tdOffsetY) + "\" x2=\"" + std::to_string(sx) + "\" y2=\"" +
               std::to_string(tdOffsetY + spanY * scaleTD) + "\"/>\n";
    }
    for (double gy = std::floor(b.minY / gridSpacing) * gridSpacing; gy <= b.maxY; gy += gridSpacing) {
        double sy = toScreenTD_Y(gy);
        svg += "<line x1=\"" + std::to_string(tdOffsetX) + "\" y1=\"" +
               std::to_string(sy) + "\" x2=\"" + std::to_string(tdOffsetX + spanX * scaleTD) +
               "\" y2=\"" + std::to_string(sy) + "\"/>\n";
    }
    svg += "</g>\n";

    // Runway marker — only for ground-ops scenarios, drawn as oriented line
    if (hasGroundOps(trace)) {
        // The runway is at the origin, heading 0 (north = +Y in our convention)
        // Draw as a thick line from (0, -150) to (0, +150) — 300 ft runway
        double rwLen = 300;
        double sx1 = toScreenTD_X(0);
        double sy1 = toScreenTD_Y(-rwLen);
        double sx2 = toScreenTD_X(0);
        double sy2 = toScreenTD_Y(rwLen);
        svg += "<line x1=\"" + std::to_string(sx1) + "\" y1=\"" + std::to_string(sy1) +
               "\" x2=\"" + std::to_string(sx2) + "\" y2=\"" + std::to_string(sy2) +
               "\" stroke=\"#FFD700\" stroke-width=\"5\" opacity=\"0.7\"/>\n";
        svg += "<text x=\"" + std::to_string(sx2 + 5) + "\" y=\"" +
               std::to_string(sy2 - 5) + "\" fill=\"#FFD700\" font-weight=\"bold\">RWY_End</text>\n";
        svg += "<text x=\"" + std::to_string(sx1 + 5) + "\" y=\"" +
               std::to_string(sy1 + 12) + "\" fill=\"#FFD700\" font-weight=\"bold\">RWY_Start</text>\n";
    }

    // Draw each phase as a separate track segment (no connecting lines between phases)
    for (size_t pi = 0; pi < phaseRanges.size(); ++pi) {
        const auto& pr = phaseRanges[pi];
        if (pr.endFrame <= pr.startFrame) continue;

        // Track segment
        std::string points;
        for (int fi = pr.startFrame; fi <= pr.endFrame && fi < (int)trace.frames.size(); ++fi) {
            const auto& f = trace.frames[fi];
            if (!points.empty()) points += " ";
            points += std::to_string(toScreenTD_X(f.x)) + "," + std::to_string(toScreenTD_Y(f.y));
        }
        svg += "<polyline points=\"" + points + "\" fill=\"none\" stroke=\"" +
               aircraftCol + "\" stroke-width=\"2\" opacity=\"0.85\"/>\n";

        // Phase start marker (numbered circle)
        if (pr.startFrame < (int)trace.frames.size()) {
            const auto& f0 = trace.frames[pr.startFrame];
            double sx = toScreenTD_X(f0.x);
            double sy = toScreenTD_Y(f0.y);
            std::string phaseLabel;
            if (pi < trace.phases.size()) phaseLabel = trace.phases[pi].name;
            else phaseLabel = "P" + std::to_string(pi + 1);
            svg += "<circle cx=\"" + std::to_string(sx) + "\" cy=\"" +
                   std::to_string(sy) + "\" r=\"7\" fill=\"" + aircraftCol +
                   "\" stroke=\"white\" stroke-width=\"2\"/>\n";
            svg += "<text x=\"" + std::to_string(sx + 10) + "\" y=\"" +
                   std::to_string(sy - 8) + "\" fill=\"white\" font-weight=\"bold\">" +
                   std::to_string(pi + 1) + ". " + esc(phaseLabel) + "</text>\n";
        }
    }

    // Threat markers (unique positions)
    std::set<std::string> drawnThreats;
    for (const auto& f : trace.frames) {
        for (const auto& t : f.threats) {
            std::string key = std::to_string((int)t.x) + "," + std::to_string((int)t.y);
            if (drawnThreats.count(key)) continue;
            drawnThreats.insert(key);
            if (t.x >= b.minX && t.x <= b.maxX && t.y >= b.minY && t.y <= b.maxY) {
                double sx = toScreenTD_X(t.x);
                double sy = toScreenTD_Y(t.y);
                std::string col = (t.type == "missile") ? "#F44336" :
                                  (t.type == "guns") ? "#FF9800" : "#9C27B0";
                svg += "<circle cx=\"" + std::to_string(sx) + "\" cy=\"" +
                       std::to_string(sy) + "\" r=\"6\" fill=\"" + col +
                       "\" opacity=\"0.5\" stroke=\"" + col + "\" stroke-width=\"1\"/>\n";
                svg += "<text x=\"" + std::to_string(sx + 8) + "\" y=\"" +
                       std::to_string(sy + 4) + "\" fill=\"" + col + "\">" + esc(t.type) + "</text>\n";
            }
        }
    }

    // ===== Side profile panel =====
    svg += "<text x=\"" + std::to_string(panelMargin) + "\" y=\"" +
           std::to_string(spTop - 15) + "\" class=\"title\" fill=\"#aaa\">Side Profile (altitude + speed vs time)</text>\n";

    // Altitude grid (every 1000 ft, or adaptive)
    double altSpacing = 1000;
    if (b.maxAlt - b.minAlt > 20000) altSpacing = 5000;
    else if (b.maxAlt - b.minAlt > 10000) altSpacing = 2000;
    svg += "<g stroke=\"#2a2a3e\" stroke-width=\"0.5\" fill=\"none\">\n";
    for (double alt = std::floor(b.minAlt / altSpacing) * altSpacing; alt <= b.maxAlt; alt += altSpacing) {
        double sy = toScreenSP_Y(alt);
        svg += "<line x1=\"" + std::to_string(panelMargin) + "\" y1=\"" +
               std::to_string(sy) + "\" x2=\"" + std::to_string(panelMargin + panelW) +
               "\" y2=\"" + std::to_string(sy) + "\"/>\n";
        svg += "<text x=\"" + std::to_string(panelMargin - 5) + "\" y=\"" +
               std::to_string(sy + 4) + "\" fill=\"#888\" text-anchor=\"end\">" +
               std::to_string((int)alt) + "ft</text>\n";
    }

    // Time grid (auto-spaced to avoid overlap — ~10 labels max)
    double timeSpan = b.maxT - b.minT;
    double timeSpacing = 10;
    if (timeSpan > 300) timeSpacing = 60;
    else if (timeSpan > 120) timeSpacing = 30;
    else if (timeSpan > 60) timeSpacing = 15;
    else if (timeSpan > 30) timeSpacing = 10;
    else timeSpacing = 5;
    for (double t = std::ceil(b.minT / timeSpacing) * timeSpacing; t <= b.maxT; t += timeSpacing) {
        double sx = toScreenSP_X(t);
        svg += "<line x1=\"" + std::to_string(sx) + "\" y1=\"" +
               std::to_string(spTop) + "\" x2=\"" + std::to_string(sx) + "\" y2=\"" +
               std::to_string(spTop + bottomPanelH) + "\"/>\n";
        svg += "<text x=\"" + std::to_string(sx) + "\" y=\"" +
               std::to_string(spTop + bottomPanelH + 18) + "\" fill=\"#888\" text-anchor=\"middle\">" +
               std::to_string((int)t) + "s</text>\n";
    }
    svg += "</g>\n";

    // Altitude + speed profiles (split by phase, no connecting lines)
    double maxVcas = 1;
    for (const auto& f : trace.frames) {
        if (phaseIdx >= 0) {
            if (f.t < trace.phases[phaseIdx].start_s || f.t > trace.phases[phaseIdx].end_s) continue;
        }
        maxVcas = std::max(maxVcas, f.vcas);
    }
    double spScale = bottomPanelH / maxVcas;

    for (const auto& pr : phaseRanges) {
        if (pr.endFrame <= pr.startFrame) continue;

        // Altitude line
        std::string altPts, spdPts;
        for (int fi = pr.startFrame; fi <= pr.endFrame && fi < (int)trace.frames.size(); ++fi) {
            const auto& f = trace.frames[fi];
            if (!altPts.empty()) { altPts += " "; spdPts += " "; }
            altPts += std::to_string(toScreenSP_X(f.t)) + "," + std::to_string(toScreenSP_Y(-f.z));
            spdPts += std::to_string(toScreenSP_X(f.t)) + "," +
                      std::to_string(spTop + bottomPanelH - f.vcas * spScale);
        }
        svg += "<polyline points=\"" + altPts + "\" fill=\"none\" stroke=\"#4CAF50\" stroke-width=\"2\"/>\n";
        svg += "<polyline points=\"" + spdPts + "\" fill=\"none\" stroke=\"#2196F3\" stroke-width=\"1.5\" opacity=\"0.7\"/>\n";
    }

    // Phase boundary lines + centered labels with alternating offset
    for (size_t pi = 0; pi < trace.phases.size(); ++pi) {
        const auto& p = trace.phases[pi];
        if (phaseIdx >= 0 && (int)pi != phaseIdx) continue;

        double startX = toScreenSP_X(p.start_s);
        double endX = toScreenSP_X(p.end_s);
        double midX = (startX + endX) / 2;

        // Boundary lines (dashed)
        svg += "<line x1=\"" + std::to_string(startX) + "\" y1=\"" +
               std::to_string(spTop) + "\" x2=\"" + std::to_string(startX) + "\" y2=\"" +
               std::to_string(spTop + bottomPanelH) +
               "\" stroke=\"" + (p.passed ? "#4CAF50" : "#F44336") +
               "\" stroke-width=\"1\" stroke-dasharray=\"4,4\" opacity=\"0.5\"/>\n";
        if (pi == trace.phases.size() - 1 || (phaseIdx >= 0 && (int)pi == phaseIdx)) {
            svg += "<line x1=\"" + std::to_string(endX) + "\" y1=\"" +
                   std::to_string(spTop) + "\" x2=\"" + std::to_string(endX) + "\" y2=\"" +
                   std::to_string(spTop + bottomPanelH) +
                   "\" stroke=\"" + (p.passed ? "#4CAF50" : "#F44336") +
                   "\" stroke-width=\"1\" stroke-dasharray=\"4,4\" opacity=\"0.5\"/>\n";
        }

        // Centered label with alternating vertical offset
        int yOffset = (pi % 2) * 16;  // alternate up/down
        int labelY = spTop - 5 + yOffset;
        const char* res = p.skipped ? "SKIP" : (p.passed ? "PASS" : "FAIL");
        std::string label = std::to_string(pi + 1) + ". " + p.name + " [" + res + "]";
        svg += "<text x=\"" + std::to_string(midX) + "\" y=\"" +
               std::to_string(labelY) + "\" fill=\"" +
               (p.passed ? "#8BC34A" : "#EF5350") +
               "\" text-anchor=\"middle\" font-weight=\"bold\">" + esc(label) + "</text>\n";
    }

    // Legend (only what's relevant)
    int legY = svgH - 80;
    svg += "<text x=\"" + std::to_string(panelMargin) + "\" y=\"" +
           std::to_string(legY) + "\" class=\"title\" fill=\"#aaa\">Legend</text>\n";
    legY += 20;
    // Aircraft color
    svg += "<rect x=\"" + std::to_string(panelMargin) + "\" y=\"" +
           std::to_string(legY - 10) + "\" width=\"12\" height=\"12\" fill=\"" + aircraftCol + "\"/>\n";
    svg += "<text x=\"" + std::to_string(panelMargin + 18) + "\" y=\"" +
           std::to_string(legY) + "\" fill=\"#ddd\">" + esc(trace.aircraft) + " (path)</text>\n";
    // Altitude
    svg += "<rect x=\"" + std::to_string(panelMargin + 200) + "\" y=\"" +
           std::to_string(legY - 10) + "\" width=\"12\" height=\"12\" fill=\"#4CAF50\"/>\n";
    svg += "<text x=\"" + std::to_string(panelMargin + 218) + "\" y=\"" +
           std::to_string(legY) + "\" fill=\"#ddd\">Altitude</text>\n";
    // Speed
    svg += "<rect x=\"" + std::to_string(panelMargin + 350) + "\" y=\"" +
           std::to_string(legY - 10) + "\" width=\"12\" height=\"12\" fill=\"#2196F3\"/>\n";
    svg += "<text x=\"" + std::to_string(panelMargin + 368) + "\" y=\"" +
           std::to_string(legY) + "\" fill=\"#ddd\">Speed (VCAS)</text>\n";
    // Phase start marker
    svg += "<circle cx=\"" + std::to_string(panelMargin + 530) + "\" cy=\"" +
           std::to_string(legY - 4) + "\" r=\"6\" fill=\"" + aircraftCol +
           "\" stroke=\"white\" stroke-width=\"2\"/>\n";
    svg += "<text x=\"" + std::to_string(panelMargin + 545) + "\" y=\"" +
           std::to_string(legY) + "\" fill=\"#ddd\">Phase start</text>\n";

    svg += "</svg>\n";

    std::ofstream f(outPath);
    if (!f) {
        std::fprintf(stderr, "Error: cannot write to %s\n", outPath.c_str());
        std::exit(1);
    }
    f << svg;
    std::printf("SVG written to %s (%zu bytes)\n", outPath.c_str(), svg.size());
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    bool split = false;
    std::string tracePath, outPath;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--split") {
            split = true;
        } else if (a == "-h" || a == "--help") {
            std::printf(
                "Usage: trace2svg <trace.json> <output.svg> [--split]\n\n"
                "Produces a two-panel SVG (top-down + side profile).\n"
                "  --split  Generate one SVG per phase (output_1.svg, output_2.svg, ...)\n");
            return 0;
        } else if (tracePath.empty()) {
            tracePath = a;
        } else if (outPath.empty()) {
            outPath = a;
        }
    }

    if (tracePath.empty() || outPath.empty()) {
        std::fprintf(stderr,
            "Usage: trace2svg <trace.json> <output.svg> [--split]\n");
        return 1;
    }

    Trace trace;
    std::string err;
    if (!readTrace(tracePath, trace, err)) {
        std::fprintf(stderr, "Error reading trace: %s\n", err.c_str());
        return 1;
    }

    std::printf("Loaded trace: %s / %s (%zu frames, %.1fs, %zu phases)\n",
                trace.aircraft.c_str(), trace.scenario.c_str(),
                trace.frames.size(), trace.duration_s, trace.phases.size());

    if (split) {
        // Generate one SVG per phase
        // Extract base path and extension
        std::string base = outPath;
        std::string ext = ".svg";
        auto dotPos = base.find_last_of('.');
        if (dotPos != std::string::npos) {
            ext = base.substr(dotPos);
            base = base.substr(0, dotPos);
        }
        for (size_t i = 0; i < trace.phases.size(); ++i) {
            std::string phasePath = base + "_" + std::to_string(i + 1) + ext;
            std::printf("Phase %zu: %s (%.1f-%.1fs)\n", i + 1,
                        trace.phases[i].name.c_str(),
                        trace.phases[i].start_s, trace.phases[i].end_s);
            generateSVG(trace, phasePath, (int)i);
        }
    } else {
        generateSVG(trace, outPath);
    }
    return 0;
}
