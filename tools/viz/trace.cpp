// f4flight - tools/trace.cpp
//
// Trace recording implementation — JSON serialization for maneuver traces.

#include "trace.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <cmath>

namespace f4flight {

namespace detail {
// Write a JSON string literal (with escaping) into `out`.
void writeJsonString(std::string& out, const std::string& s) {
    out += '"';
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += '"';
}
} // namespace detail

// ---------------------------------------------------------------------------
// TraceRecorder
// ---------------------------------------------------------------------------

void TraceRecorder::start(const std::string& aircraft, const std::string& scenario) {
    trace_ = Trace{};
    trace_.aircraft = aircraft;
    trace_.scenario = scenario;
    trace_.frames.reserve(60 * 120);  // pre-alloc for 120s at 60 Hz
    phaseStart_ = 0.0;
}

void TraceRecorder::record(double t, const AircraftState& as, const PilotInput& input,
                            const std::string& modeName, const std::string& phaseName) {
    TraceFrame f;
    f.t = t;
    f.x = as.kin.x;
    f.y = as.kin.y;
    f.z = as.kin.z;
    f.psi = as.kin.psi;
    f.theta = as.kin.theta;
    f.phi = as.kin.phi;
    f.vt = as.kin.vt;
    f.vcas = as.vcas;
    f.nzcgs = as.loads.nzcgs;
    f.throttle = input.throttle;
    f.pstick = input.pstick;
    f.rstick = input.rstick;
    f.mode = modeName;
    f.phase = phaseName;
    trace_.frames.push_back(std::move(f));
}

void TraceRecorder::record(double t, const AircraftState& as, const PilotInput& input,
                            const std::string& modeName, const std::string& phaseName,
                            const std::vector<ThreatEntity>& threats) {
    record(t, as, input, modeName, phaseName);
    if (!threats.empty()) {
        trace_.frames.back().threats = threats;
    }
}

void TraceRecorder::addSample(const std::string& key, double value, const std::string& unit) {
    if (!trace_.frames.empty()) {
        trace_.frames.back().samples.push_back({key, value, unit});
    }
}

void TraceRecorder::addEvent(double t, const std::string& category,
                              const std::string& message, const std::string& severity) {
    trace_.events.push_back({t, category, message, severity});
}

void TraceRecorder::markPhase(const std::string& name, double start_s, double end_s,
                               bool passed, bool skipped, bool reinitializes,
                               const std::string& criteria,
                               const std::string& failureReason) {
    trace_.phases.push_back({name, start_s, end_s, passed, skipped, reinitializes, criteria, failureReason});
}

void TraceRecorder::setWaypoints(const std::vector<Waypoint>& wps) {
    trace_.waypoints = wps;
}

void TraceRecorder::addSceneLine(const SceneLine& line) {
    trace_.sceneLines.push_back(line);
}

void TraceRecorder::finish(double duration_s) {
    trace_.duration_s = duration_s;
}

// ---------------------------------------------------------------------------
// JSON writing (compact, no whitespace)
// ---------------------------------------------------------------------------
//
// traceToJson() is the single source of truth for trace serialization. It
// produces compact JSON suitable for both file output and inline embedding
// in HTML <script> tags. TraceRecorder::write() is now a thin wrapper.

void traceToJson(const Trace& trace, std::string& out) {
    out.reserve(out.size() + trace.frames.size() * 180 + 256);
    out += '{';

    // Metadata
    out += "\"aircraft\":";
    detail::writeJsonString(out, trace.aircraft);
    out += ",\"scenario\":";
    detail::writeJsonString(out, trace.scenario);
    out += ",\"duration_s\":";
    out += std::to_string(trace.duration_s);

    // Phases
    out += ",\"phases\":[";
    for (size_t i = 0; i < trace.phases.size(); ++i) {
        if (i > 0) out += ',';
        const auto& p = trace.phases[i];
        out += "{\"name\":";
        detail::writeJsonString(out, p.name);
        out += ",\"start_s\":";
        out += std::to_string(p.start_s);
        out += ",\"end_s\":";
        out += std::to_string(p.end_s);
        out += ",\"passed\":";
        out += p.passed ? "true" : "false";
        out += ",\"skipped\":";
        out += p.skipped ? "true" : "false";
        out += ",\"reinitializes\":";
        out += p.reinitializes ? "true" : "false";
        out += ",\"criteria\":";
        detail::writeJsonString(out, p.criteria);
        out += ",\"failureReason\":";
        detail::writeJsonString(out, p.failureReason);
        out += '}';
    }
    out += ']';

    // Frames
    out += ",\"frames\":[";
    for (size_t i = 0; i < trace.frames.size(); ++i) {
        if (i > 0) out += ',';
        const auto& f = trace.frames[i];
        out += "{\"t\":";
        out += std::to_string(f.t);
        out += ",\"x\":";
        out += std::to_string(f.x);
        out += ",\"y\":";
        out += std::to_string(f.y);
        out += ",\"z\":";
        out += std::to_string(f.z);
        out += ",\"psi\":";
        out += std::to_string(f.psi);
        out += ",\"theta\":";
        out += std::to_string(f.theta);
        out += ",\"phi\":";
        out += std::to_string(f.phi);
        out += ",\"vt\":";
        out += std::to_string(f.vt);
        out += ",\"vcas\":";
        out += std::to_string(f.vcas);
        out += ",\"nzcgs\":";
        out += std::to_string(f.nzcgs);
        out += ",\"throttle\":";
        out += std::to_string(f.throttle);
        out += ",\"pstick\":";
        out += std::to_string(f.pstick);
        out += ",\"rstick\":";
        out += std::to_string(f.rstick);
        out += ",\"mode\":";
        detail::writeJsonString(out, f.mode);
        out += ",\"phase\":";
        detail::writeJsonString(out, f.phase);
        if (!f.threats.empty()) {
            out += ",\"threats\":[";
            for (size_t j = 0; j < f.threats.size(); ++j) {
                if (j > 0) out += ',';
                const auto& e = f.threats[j];
                out += "{\"type\":";
                detail::writeJsonString(out, e.type);
                out += ",\"x\":";
                out += std::to_string(e.x);
                out += ",\"y\":";
                out += std::to_string(e.y);
                out += ",\"z\":";
                out += std::to_string(e.z);
                out += ",\"speed\":";
                out += std::to_string(e.speed);
                out += '}';
            }
            out += ']';
        }
        if (!f.samples.empty()) {
            out += ",\"samples\":[";
            for (size_t j = 0; j < f.samples.size(); ++j) {
                if (j > 0) out += ',';
                const auto& s = f.samples[j];
                out += "{\"key\":";
                detail::writeJsonString(out, s.key);
                out += ",\"value\":";
                out += std::to_string(s.value);
                out += ",\"unit\":";
                detail::writeJsonString(out, s.unit);
                out += '}';
            }
            out += ']';
        }
        out += '}';
    }
    out += ']';

    // Events (discrete events: mode changes, weapon fires, etc.)
    if (!trace.events.empty()) {
        out += ",\"events\":[";
        for (size_t i = 0; i < trace.events.size(); ++i) {
            if (i > 0) out += ',';
            const auto& ev = trace.events[i];
            out += "{\"t\":";
            out += std::to_string(ev.t);
            out += ",\"category\":";
            detail::writeJsonString(out, ev.category);
            out += ",\"message\":";
            detail::writeJsonString(out, ev.message);
            out += ",\"severity\":";
            detail::writeJsonString(out, ev.severity);
            out += '}';
        }
        out += ']';
    }

    // Waypoints (scenario-level navigation waypoints)
    if (!trace.waypoints.empty()) {
        out += ",\"waypoints\":[";
        for (size_t i = 0; i < trace.waypoints.size(); ++i) {
            if (i > 0) out += ',';
            const auto& w = trace.waypoints[i];
            out += "{\"x\":";
            out += std::to_string(w.x);
            out += ",\"y\":";
            out += std::to_string(w.y);
            out += ",\"z\":";
            out += std::to_string(w.z);
            out += ",\"name\":";
            detail::writeJsonString(out, w.name);
            out += '}';
        }
        out += ']';
    }

    // Scene lines (runway, taxiways, etc.)
    if (!trace.sceneLines.empty()) {
        out += ",\"sceneLines\":[";
        for (size_t i = 0; i < trace.sceneLines.size(); ++i) {
            if (i > 0) out += ',';
            const auto& s = trace.sceneLines[i];
            out += "{\"label\":";
            detail::writeJsonString(out, s.label);
            out += ",\"x1\":";
            out += std::to_string(s.x1);
            out += ",\"y1\":";
            out += std::to_string(s.y1);
            out += ",\"z1\":";
            out += std::to_string(s.z1);
            out += ",\"x2\":";
            out += std::to_string(s.x2);
            out += ",\"y2\":";
            out += std::to_string(s.y2);
            out += ",\"z2\":";
            out += std::to_string(s.z2);
            out += ",\"color\":";
            detail::writeJsonString(out, s.color);
            out += ",\"width\":";
            out += std::to_string(s.width);
            out += '}';
        }
        out += ']';
    }

    out += '}';
}

bool TraceRecorder::write(const std::string& path) const {
    std::string json;
    traceToJson(trace_, json);
    std::ofstream f(path);
    if (!f) return false;
    f << json;
    return f.good();
}

// ---------------------------------------------------------------------------
// JSON reading (minimal parser for the trace format)
// ---------------------------------------------------------------------------

namespace {

// Skip whitespace
static const char* skipWs(const char* p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    return p;
}

// Parse a string literal. Returns pointer past closing quote, or nullptr on error.
static const char* parseString(const char* p, std::string& out) {
    p = skipWs(p);
    if (*p != '"') return nullptr;
    ++p;
    out.clear();
    while (*p && *p != '"') {
        if (*p == '\\') {
            ++p;
            if (!*p) return nullptr;
        }
        out += *p;
        ++p;
    }
    if (*p != '"') return nullptr;
    return p + 1;
}

// Parse a number. Returns pointer past the number, or nullptr on error.
static const char* parseNumber(const char* p, double& out) {
    p = skipWs(p);
    char* end = nullptr;
    out = std::strtod(p, &end);
    if (end == p) return nullptr;
    return end;
}

// Parse a boolean.
static const char* parseBool(const char* p, bool& out) {
    p = skipWs(p);
    if (p[0] == 't' && p[1] == 'r' && p[2] == 'u' && p[3] == 'e') {
        out = true;
        return p + 4;
    }
    if (p[0] == 'f' && p[1] == 'a' && p[2] == 'l' && p[3] == 's' && p[4] == 'e') {
        out = false;
        return p + 5;
    }
    return nullptr;
}

} // anonymous namespace

bool readTrace(const std::string& path, Trace& out, std::string& error_msg) {
    std::ifstream f(path);
    if (!f) {
        error_msg = "Cannot open file: " + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string content = ss.str();
    const char* p = content.c_str();

    p = skipWs(p);
    if (*p != '{') { error_msg = "Expected '{' at start"; return false; }
    ++p;

    out = Trace{};

    while (*p) {
        p = skipWs(p);
        if (*p == '}') break;
        if (*p != '"') { error_msg = "Expected key string"; return false; }

        std::string key;
        p = parseString(p, key);
        if (!p) { error_msg = "Bad key string"; return false; }

        p = skipWs(p);
        if (*p != ':') { error_msg = "Expected ':' after key"; return false; }
        ++p;
        p = skipWs(p);

        if (key == "aircraft") {
            p = parseString(p, out.aircraft);
        } else if (key == "scenario") {
            p = parseString(p, out.scenario);
        } else if (key == "duration_s") {
            p = parseNumber(p, out.duration_s);
        } else if (key == "phases") {
            if (*p != '[') { error_msg = "Expected '[' for phases"; return false; }
            ++p;
            p = skipWs(p);
            while (*p && *p != ']') {
                PhaseResult pr{};
                if (*p != '{') { error_msg = "Expected '{' in phase"; return false; }
                ++p;
                while (*p && *p != '}') {
                    p = skipWs(p);
                    std::string pk;
                    p = parseString(p, pk);
                    if (!p) { error_msg = "Bad phase key"; return false; }
                    p = skipWs(p);
                    if (*p != ':') { error_msg = "Expected ':' in phase"; return false; }
                    ++p;
                    p = skipWs(p);
                    if (pk == "name") {
                        p = parseString(p, pr.name);
                    } else if (pk == "start_s") {
                        p = parseNumber(p, pr.start_s);
                    } else if (pk == "end_s") {
                        p = parseNumber(p, pr.end_s);
                    } else if (pk == "passed") {
                        p = parseBool(p, pr.passed);
                    } else if (pk == "skipped") {
                        p = parseBool(p, pr.skipped);
                    } else if (pk == "reinitializes") {
                        p = parseBool(p, pr.reinitializes);
                    } else if (pk == "criteria") {
                        p = parseString(p, pr.criteria);
                    } else if (pk == "failureReason") {
                        p = parseString(p, pr.failureReason);
                    } else {
                        // skip unknown value
                        if (*p == '"') { std::string s; p = parseString(p, s); }
                        else { double d; p = parseNumber(p, d); }
                    }
                    if (!p) { error_msg = "Bad phase value"; return false; }
                    p = skipWs(p);
                    if (*p == ',') ++p;
                }
                if (*p != '}') { error_msg = "Expected '}' in phase"; return false; }
                ++p;
                out.phases.push_back(pr);
                p = skipWs(p);
                if (*p == ',') ++p;
                p = skipWs(p);
            }
            if (*p != ']') { error_msg = "Expected ']' after phases"; return false; }
            ++p;
        } else if (key == "frames") {
            if (*p != '[') { error_msg = "Expected '[' for frames"; return false; }
            ++p;
            p = skipWs(p);
            while (*p && *p != ']') {
                TraceFrame fr{};
                if (*p != '{') { error_msg = "Expected '{' in frame"; return false; }
                ++p;
                while (*p && *p != '}') {
                    p = skipWs(p);
                    std::string fk;
                    p = parseString(p, fk);
                    if (!p) { error_msg = "Bad frame key"; return false; }
                    p = skipWs(p);
                    if (*p != ':') { error_msg = "Expected ':' in frame"; return false; }
                    ++p;
                    p = skipWs(p);
                    if (fk == "t") p = parseNumber(p, fr.t);
                    else if (fk == "x") p = parseNumber(p, fr.x);
                    else if (fk == "y") p = parseNumber(p, fr.y);
                    else if (fk == "z") p = parseNumber(p, fr.z);
                    else if (fk == "psi") p = parseNumber(p, fr.psi);
                    else if (fk == "theta") p = parseNumber(p, fr.theta);
                    else if (fk == "phi") p = parseNumber(p, fr.phi);
                    else if (fk == "vt") p = parseNumber(p, fr.vt);
                    else if (fk == "vcas") p = parseNumber(p, fr.vcas);
                    else if (fk == "nzcgs") p = parseNumber(p, fr.nzcgs);
                    else if (fk == "throttle") p = parseNumber(p, fr.throttle);
                    else if (fk == "pstick") p = parseNumber(p, fr.pstick);
                    else if (fk == "rstick") p = parseNumber(p, fr.rstick);
                    else if (fk == "mode") p = parseString(p, fr.mode);
                    else if (fk == "phase") p = parseString(p, fr.phase);
                    else if (fk == "threats") {
                        // parse threats array
                        if (*p != '[') { error_msg = "Expected '[' for threats"; return false; }
                        ++p;
                        p = skipWs(p);
                        while (*p && *p != ']') {
                            ThreatEntity te{};
                            if (*p != '{') { error_msg = "Expected '{' in threat"; return false; }
                            ++p;
                            while (*p && *p != '}') {
                                p = skipWs(p);
                                std::string tk;
                                p = parseString(p, tk);
                                if (!p) { error_msg = "Bad threat key"; return false; }
                                p = skipWs(p);
                                if (*p != ':') { error_msg = "Expected ':' in threat"; return false; }
                                ++p;
                                p = skipWs(p);
                                if (tk == "type") p = parseString(p, te.type);
                                else if (tk == "x") p = parseNumber(p, te.x);
                                else if (tk == "y") p = parseNumber(p, te.y);
                                else if (tk == "z") p = parseNumber(p, te.z);
                                else if (tk == "speed") p = parseNumber(p, te.speed);
                                else {
                                    if (*p == '"') { std::string s; p = parseString(p, s); }
                                    else { double d; p = parseNumber(p, d); }
                                }
                                if (!p) { error_msg = "Bad threat value"; return false; }
                                p = skipWs(p);
                                if (*p == ',') ++p;
                            }
                            if (*p != '}') { error_msg = "Expected '}' in threat"; return false; }
                            ++p;
                            fr.threats.push_back(te);
                            p = skipWs(p);
                            if (*p == ',') ++p;
                            p = skipWs(p);
                        }
                        if (*p != ']') { error_msg = "Expected ']' after threats"; return false; }
                        ++p;
                    }
                    else if (fk == "samples") {
                        if (*p != '[') { error_msg = "Expected '[' for samples"; return false; }
                        ++p;
                        p = skipWs(p);
                        while (*p && *p != ']') {
                            TraceSample sm{};
                            if (*p != '{') { error_msg = "Expected '{' in sample"; return false; }
                            ++p;
                            while (*p && *p != '}') {
                                p = skipWs(p);
                                std::string sk;
                                p = parseString(p, sk);
                                if (!p) { error_msg = "Bad sample key"; return false; }
                                p = skipWs(p);
                                if (*p != ':') { error_msg = "Expected ':' in sample"; return false; }
                                ++p;
                                p = skipWs(p);
                                if (sk == "key") p = parseString(p, sm.key);
                                else if (sk == "value") p = parseNumber(p, sm.value);
                                else if (sk == "unit") p = parseString(p, sm.unit);
                                else {
                                    if (*p == '"') { std::string s; p = parseString(p, s); }
                                    else { double d; p = parseNumber(p, d); }
                                }
                                if (!p) { error_msg = "Bad sample value"; return false; }
                                p = skipWs(p);
                                if (*p == ',') ++p;
                            }
                            if (*p != '}') { error_msg = "Expected '}' in sample"; return false; }
                            ++p;
                            fr.samples.push_back(sm);
                            p = skipWs(p);
                            if (*p == ',') ++p;
                            p = skipWs(p);
                        }
                        if (*p != ']') { error_msg = "Expected ']' after samples"; return false; }
                        ++p;
                    }
                    else {
                        // skip unknown value
                        if (*p == '"') { std::string s; p = parseString(p, s); }
                        else { double d; p = parseNumber(p, d); }
                    }
                    if (!p) { error_msg = "Bad frame value"; return false; }
                    p = skipWs(p);
                    if (*p == ',') ++p;
                }
                if (*p != '}') { error_msg = "Expected '}' in frame"; return false; }
                ++p;
                out.frames.push_back(fr);
                p = skipWs(p);
                if (*p == ',') ++p;
                p = skipWs(p);
            }
            if (*p != ']') { error_msg = "Expected ']' after frames"; return false; }
            ++p;
        } else if (key == "events") {
            if (*p != '[') { error_msg = "Expected '[' for events"; return false; }
            ++p;
            p = skipWs(p);
            while (*p && *p != ']') {
                TraceEvent ev{};
                if (*p != '{') { error_msg = "Expected '{' in event"; return false; }
                ++p;
                while (*p && *p != '}') {
                    p = skipWs(p);
                    std::string ek;
                    p = parseString(p, ek);
                    if (!p) { error_msg = "Bad event key"; return false; }
                    p = skipWs(p);
                    if (*p != ':') { error_msg = "Expected ':' in event"; return false; }
                    ++p;
                    p = skipWs(p);
                    if (ek == "t") p = parseNumber(p, ev.t);
                    else if (ek == "category") p = parseString(p, ev.category);
                    else if (ek == "message") p = parseString(p, ev.message);
                    else if (ek == "severity") p = parseString(p, ev.severity);
                    else {
                        if (*p == '"') { std::string s; p = parseString(p, s); }
                        else { double d; p = parseNumber(p, d); }
                    }
                    if (!p) { error_msg = "Bad event value"; return false; }
                    p = skipWs(p);
                    if (*p == ',') ++p;
                }
                if (*p != '}') { error_msg = "Expected '}' in event"; return false; }
                ++p;
                out.events.push_back(ev);
                p = skipWs(p);
                if (*p == ',') ++p;
                p = skipWs(p);
            }
            if (*p != ']') { error_msg = "Expected ']' after events"; return false; }
            ++p;
        } else if (key == "waypoints") {
            if (*p != '[') { error_msg = "Expected '[' for waypoints"; return false; }
            ++p;
            p = skipWs(p);
            while (*p && *p != ']') {
                Waypoint wp{};
                if (*p != '{') { error_msg = "Expected '{' in waypoint"; return false; }
                ++p;
                while (*p && *p != '}') {
                    p = skipWs(p);
                    std::string wk;
                    p = parseString(p, wk);
                    if (!p) { error_msg = "Bad waypoint key"; return false; }
                    p = skipWs(p);
                    if (*p != ':') { error_msg = "Expected ':' in waypoint"; return false; }
                    ++p;
                    p = skipWs(p);
                    if (wk == "x") p = parseNumber(p, wp.x);
                    else if (wk == "y") p = parseNumber(p, wp.y);
                    else if (wk == "z") p = parseNumber(p, wp.z);
                    else if (wk == "name") p = parseString(p, wp.name);
                    else {
                        if (*p == '"') { std::string s; p = parseString(p, s); }
                        else { double d; p = parseNumber(p, d); }
                    }
                    if (!p) { error_msg = "Bad waypoint value"; return false; }
                    p = skipWs(p);
                    if (*p == ',') ++p;
                }
                if (*p != '}') { error_msg = "Expected '}' in waypoint"; return false; }
                ++p;
                out.waypoints.push_back(wp);
                p = skipWs(p);
                if (*p == ',') ++p;
                p = skipWs(p);
            }
            if (*p != ']') { error_msg = "Expected ']' after waypoints"; return false; }
            ++p;
        } else if (key == "sceneLines") {
            if (*p != '[') { error_msg = "Expected '[' for sceneLines"; return false; }
            ++p;
            p = skipWs(p);
            while (*p && *p != ']') {
                SceneLine sl{};
                if (*p != '{') { error_msg = "Expected '{' in sceneLine"; return false; }
                ++p;
                while (*p && *p != '}') {
                    p = skipWs(p);
                    std::string sk;
                    p = parseString(p, sk);
                    if (!p) { error_msg = "Bad sceneLine key"; return false; }
                    p = skipWs(p);
                    if (*p != ':') { error_msg = "Expected ':' in sceneLine"; return false; }
                    ++p;
                    p = skipWs(p);
                    if (sk == "label") p = parseString(p, sl.label);
                    else if (sk == "x1") p = parseNumber(p, sl.x1);
                    else if (sk == "y1") p = parseNumber(p, sl.y1);
                    else if (sk == "z1") p = parseNumber(p, sl.z1);
                    else if (sk == "x2") p = parseNumber(p, sl.x2);
                    else if (sk == "y2") p = parseNumber(p, sl.y2);
                    else if (sk == "z2") p = parseNumber(p, sl.z2);
                    else if (sk == "color") p = parseString(p, sl.color);
                    else if (sk == "width") p = parseNumber(p, sl.width);
                    else {
                        if (*p == '"') { std::string s; p = parseString(p, s); }
                        else { double d; p = parseNumber(p, d); }
                    }
                    if (!p) { error_msg = "Bad sceneLine value"; return false; }
                    p = skipWs(p);
                    if (*p == ',') ++p;
                }
                if (*p != '}') { error_msg = "Expected '}' in sceneLine"; return false; }
                ++p;
                out.sceneLines.push_back(sl);
                p = skipWs(p);
                if (*p == ',') ++p;
                p = skipWs(p);
            }
            if (*p != ']') { error_msg = "Expected ']' after sceneLines"; return false; }
            ++p;
        } else {
            // skip unknown key's value
            if (*p == '"') { std::string s; p = parseString(p, s); }
            else if (*p == '[') {
                int depth = 1; ++p;
                while (*p && depth > 0) {
                    if (*p == '[') ++depth;
                    else if (*p == ']') --depth;
                    ++p;
                }
            } else if (*p == '{') {
                int depth = 1; ++p;
                while (*p && depth > 0) {
                    if (*p == '{') ++depth;
                    else if (*p == '}') --depth;
                    ++p;
                }
            } else { double d; p = parseNumber(p, d); }
        }

        if (!p) { error_msg = "Parse error"; return false; }
        p = skipWs(p);
        if (*p == ',') ++p;
    }

    return true;
}

} // namespace f4flight
