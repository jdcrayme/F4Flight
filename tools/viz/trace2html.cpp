// f4flight - tools/viz/trace2html.cpp
//
// Convert one or more trace JSON files into a self-contained interactive
// HTML report.
//
// Usage:
//   trace2html <trace.json>... -o <report.html> [--open]
//   trace2html <trace.json> -o <report.html>      # single trace, no index
//   trace2html *.json -o report.html --open       # dashboard, auto-open
//
// The output is a single .html file with all trace data embedded inline.
// No web server or external files are required — open it in any browser.
//
// --open launches the default browser:
//   Linux:   xdg-open
//   macOS:   open
//   Windows: start
//
// Exit codes: 0 OK, 1 usage error, 2 could not read a trace, 3 write error.

#include "html_report.h"
#include "trace.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#ifdef __linux__
#include <unistd.h>
#elif defined(__APPLE__)
#include <unistd.h>
#endif

using namespace f4flight;

static void printHelp() {
    std::printf(
        "Usage: trace2html <trace.json>... -o <report.html> [--open] [--title TITLE]\n"
        "\n"
        "Converts one or more f4flight trace JSON files into a self-contained\n"
        "interactive HTML report with an index dashboard and per-trace detail\n"
        "views (top-down flight path, altitude/speed/G time-series, scrubber).\n"
        "\n"
        "Options:\n"
        "  -o, --output PATH   Output HTML file (required).\n"
        "      --open          Open the report in the default browser after writing.\n"
        "      --title TITLE   Custom report title.\n"
        "  -h, --help          Show this help.\n"
        "\n"
        "Examples:\n"
        "  trace2html trace.json -o report.html --open\n"
        "  trace2html traces/*.json -o dashboard.html\n");
}

// Launch the default browser to open a file:// URL. Returns true if a launch
// was attempted (does not guarantee the browser actually opened).
static bool openInBrowser(const std::string& htmlPath) {
    // Build a file:// URL with the absolute path.
    std::string url = "file://";
    // Resolve absolute path so the browser opens the right file regardless
    // of its working directory.
    std::string absPath = htmlPath;
#ifdef __linux__
    {
        char buf[4096];
        if (realpath(htmlPath.c_str(), buf)) absPath = buf;
    }
#endif
    // Minimal URL-encoding: spaces and other unsafe chars. For typical paths
    // (no spaces) this is a no-op; for paths with spaces it avoids breakage.
    for (size_t i = 0; i < absPath.size(); ++i) {
        char c = absPath[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '/' || c == '-' || c == '_' ||
            c == '.' || c == '~') {
            url += c;
        } else {
            char hex[4];
            std::snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
            url += hex;
        }
    }

    std::string cmd;
#if defined(_WIN32) || defined(_WIN64)
    cmd = "start \"\" \"" + absPath + "\"";
#elif defined(__APPLE__)
    cmd = "open \"" + url + "\"";
#else
    cmd = "xdg-open \"" + url + "\" 2>/dev/null";
#endif
    std::printf("Opening: %s\n", url.c_str());
    int rc = std::system(cmd.c_str());
    return rc == 0;
}

int main(int argc, char** argv) {
    std::vector<std::string> tracePaths;
    std::string outPath;
    bool doOpen = false;
    std::string title;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") { printHelp(); return 0; }
        else if (a == "-o" || a == "--output") {
            if (i + 1 >= argc) { std::fprintf(stderr, "Error: -o requires a path\n"); return 1; }
            outPath = argv[++i];
        } else if (a == "--open") {
            doOpen = true;
        } else if (a == "--title") {
            if (i + 1 >= argc) { std::fprintf(stderr, "Error: --title requires an argument\n"); return 1; }
            title = argv[++i];
        } else if (a.size() > 0 && a[0] == '-') {
            std::fprintf(stderr, "Error: unknown option '%s'\n", a.c_str());
            printHelp();
            return 1;
        } else {
            tracePaths.push_back(a);
        }
    }

    if (tracePaths.empty()) {
        std::fprintf(stderr, "Error: no trace files specified.\n\n");
        printHelp();
        return 1;
    }
    if (outPath.empty()) {
        std::fprintf(stderr, "Error: no output file specified (-o).\n\n");
        printHelp();
        return 1;
    }

    // Read all traces.
    std::vector<Trace> traces;
    traces.reserve(tracePaths.size());
    for (const auto& path : tracePaths) {
        Trace t;
        std::string err;
        if (!readTrace(path, t, err)) {
            std::fprintf(stderr, "Error reading %s: %s\n", path.c_str(), err.c_str());
            return 2;
        }
        traces.push_back(std::move(t));
    }

    std::printf("Loaded %zu trace(s)\n", traces.size());
    for (const auto& t : traces) {
        std::printf("  %s / %s — %zu frames, %.1fs, %zu phases\n",
                    t.aircraft.c_str(), t.scenario.c_str(),
                    t.frames.size(), t.duration_s, t.phases.size());
    }

    // Generate the report.
    HtmlReportOptions opts;
    if (!title.empty()) opts.title = title;
    std::ofstream f(outPath);
    if (!f) {
        std::fprintf(stderr, "Error: cannot write to %s\n", outPath.c_str());
        return 3;
    }
    generateHtmlReport(traces, f, opts);
    f.close();
    if (!f.good()) {
        std::fprintf(stderr, "Error: write failed for %s\n", outPath.c_str());
        return 3;
    }

    // Report file size.
    std::ifstream sz(outPath, std::ios::ate | std::ios::binary);
    long bytes = sz ? (long)sz.tellg() : 0;
    std::printf("\nHTML report written to %s (%.1f KB)\n", outPath.c_str(), bytes / 1024.0);

    if (doOpen) {
        openInBrowser(outPath);
    }
    return 0;
}
