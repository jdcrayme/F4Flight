// f4flight - tools/viz/html_report.h
//
// Self-contained interactive HTML report generator for flight test traces.
//
// A single .html file containing:
//   - An index dashboard: a grid of trace cards (one per scenario run), each
//     with a mini top-down flight-path thumbnail and a pass/fail badge.
//   - A detail view: interactive top-down flight path + altitude/speed/G
//     time-series, with a time scrubber, play/pause, and color-by controls
//     (phase / pass-fail / altitude / speed / G-load / AI mode).
//
// All trace data is embedded inline as JSON inside a <script> tag, so the
// report is fully self-contained — no web server, no external files, no
// network. Open it directly in any browser (including via file://).
//
// Usage from C++:
//   std::vector<Trace> traces = ...;
//   std::ofstream out("report.html");
//   generateHtmlReport(traces, out);
//
// Usage from the CLI tool (trace2html):
//   trace2html trace1.json trace2.json -o report.html
//   trace2html trace1.json -o report.html --open

#pragma once

#include "trace.h"

#include <ostream>
#include <string>
#include <vector>

namespace f4flight {

struct HtmlReportOptions {
    // Browser <title> and visible heading.
    std::string title = "F4Flight Flight Test Report";
};

// Generate a self-contained HTML report from one or more traces.
// Writes the full document to `out`. The HTML embeds all trace data inline
// as compact JSON and requires no external files or server.
//
// Traces may be empty — the report will render an empty index with a notice.
void generateHtmlReport(const std::vector<Trace>& traces,
                        std::ostream& out,
                        const HtmlReportOptions& opts = {});

} // namespace f4flight
