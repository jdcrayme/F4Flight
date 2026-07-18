// f4flight - tools/viz/html_report.cpp
//
// Self-contained interactive HTML report generator.

#include "html_report.h"
#include "trace.h"

#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>

// We can pass the compiled-in F4FLIGHT_SOURCE_DIR macro from CMake as a fallback.
#ifndef F4FLIGHT_SOURCE_DIR
#define F4FLIGHT_SOURCE_DIR ""
#endif

namespace f4flight {

// Escape a JSON string for safe embedding inside an HTML <script> tag.
static std::string escapeForScript(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 16);
    for (char c : s) {
        if (c == '<') out += "\\u003c";
        else out += c;
    }
    return out;
}

// Produce a JSON string literal (with quotes) from a C++ string
static std::string jsonString(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += "\"";
    return escapeForScript(out);
}

// Downsample a trace's frames for HTML embedding.
static const size_t kMaxFramesPerTrace = 3000;
static Trace downsampleForHtml(const Trace& in) {
    Trace out = in;
    out.frames.clear();
    const size_t n = in.frames.size();
    if (n <= kMaxFramesPerTrace) {
        out.frames = in.frames;
        return out;
    }
    const size_t stride = (n + kMaxFramesPerTrace - 1) / kMaxFramesPerTrace;
    std::vector<bool> keep(n, false);
    for (size_t i = 0; i < n; i += stride) keep[i] = true;
    keep[n - 1] = true;
    out.frames.reserve(kMaxFramesPerTrace + 2);
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) out.frames.push_back(in.frames[i]);
    }
    return out;
}

// Find and read the template.html file
static std::string readTemplateFile() {
    std::vector<std::filesystem::path> searchPaths;

    // 1. Check compiled-in F4FLIGHT_SOURCE_DIR (if defined)
    std::string srcDir = F4FLIGHT_SOURCE_DIR;
    if (!srcDir.empty()) {
        searchPaths.push_back(std::filesystem::path(srcDir) / "tools" / "viz" / "template.html");
    }

    // 2. Check current working directory and common subdirectories
    searchPaths.push_back(std::filesystem::path(".") / "template.html");
    searchPaths.push_back(std::filesystem::path(".") / "tools" / "viz" / "template.html");
    searchPaths.push_back(std::filesystem::path("..") / "tools" / "viz" / "template.html");

    // 3. Check relative build paths
    searchPaths.push_back(std::filesystem::path(".") / "tools" / "template.html");
    searchPaths.push_back(std::filesystem::path(".") / "bin" / "template.html");

    std::vector<std::string> searchedPathsStr;
    for (const auto& path : searchPaths) {
        std::error_code ec;
        std::filesystem::path absPath = std::filesystem::absolute(path, ec);
        std::string pStr = ec ? path.string() : absPath.string();
        searchedPathsStr.push_back(pStr);

        if (std::filesystem::exists(path, ec) && !std::filesystem::is_directory(path, ec)) {
            std::ifstream f(path, std::ios::in | std::ios::binary);
            if (f) {
                std::ostringstream ss;
                ss << f.rdbuf();
                return ss.str();
            }
        }
    }

    std::cerr << "Error: Could not locate 'template.html'. Checked paths:\n";
    for (const auto& p : searchedPathsStr) {
        std::cerr << "  - " << p << "\n";
    }
    std::cerr << "Please ensure 'template.html' is in one of these directories.\n";
    return "";
}

// Assemble the document by doing string replacement on the loaded template
void generateHtmlReport(const std::vector<Trace>& traces,
                        std::ostream& out,
                        const HtmlReportOptions& opts) {
    std::string templateStr = readTemplateFile();
    if (templateStr.empty()) {
        // Fall back to a very simple error HTML so the file is not empty and still opens
        out << "<!DOCTYPE html><html><body><h2>Error: template.html not found</h2>";
        out << "<p>Could not locate the required <code>template.html</code> file to generate the rich visualizer.</p>";
        out << "</body></html>\n";
        return;
    }

    // Format the Report Title
    std::string titleStr = jsonString(opts.title);

    // Format the Traces as compact JSON
    std::string tracesJson = "[";
    for (size_t i = 0; i < traces.size(); ++i) {
        if (i > 0) tracesJson += ",";
        Trace ds = downsampleForHtml(traces[i]);
        std::string json;
        traceToJson(ds, json);
        tracesJson += escapeForScript(json);
    }
    tracesJson += "]";

    // Replace the placeholders in the template string
    std::string placeholderTitle = "/* INSERT_REPORT_TITLE_HERE */";
    std::string placeholderTraces = "/* INSERT_TRACES_HERE */";

    size_t titlePos = templateStr.find(placeholderTitle);
    if (titlePos != std::string::npos) {
        templateStr.replace(titlePos, placeholderTitle.length(), titleStr);
    }

    // Recalculate position for traces since the length might have changed
    size_t tracesPos = templateStr.find(placeholderTraces);
    if (tracesPos != std::string::npos) {
        templateStr.replace(tracesPos, placeholderTraces.length(), tracesJson);
    }

    out << templateStr;
}

} // namespace f4flight
