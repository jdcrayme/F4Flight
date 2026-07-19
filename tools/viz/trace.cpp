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

void TraceRecorder::setTestMetadata(const std::string& testGroup, const std::string& testLevel) {
    trace_.testGroup = testGroup;
    trace_.testLevel = testLevel;
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
    out += ",\"testGroup\":";
    detail::writeJsonString(out, trace.testGroup);
    out += ",\"testLevel\":";
    detail::writeJsonString(out, trace.testLevel);
    out += ",\"duration_s\":";
    out += std::to_string(trace.duration_s);


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
                out += ",\"name\":";
                detail::writeJsonString(out, e.name);
                out += ",\"x\":";
                out += std::to_string(e.x);
                out += ",\"y\":";
                out += std::to_string(e.y);
                out += ",\"z\":";
                out += std::to_string(e.z);
                out += ",\"psi\":";
                out += std::to_string(e.psi);
                out += ",\"theta\":";
                out += std::to_string(e.theta);
                out += ",\"phi\":";
                out += std::to_string(e.phi);
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


    // Generalized static test geometry
    if (!trace.geometry.empty()) {
        out += ",\"geometry\":[";
        for (size_t i = 0; i < trace.geometry.size(); ++i) {
            if (i > 0) out += ',';
            const auto& g = trace.geometry[i];
            out += "{\"name\":";
            detail::writeJsonString(out, g.name);
            out += ",\"type\":";
            detail::writeJsonString(out, g.type);
            out += ",\"coords\":[";
            for (size_t j = 0; j < g.coords.size(); ++j) {
                if (j > 0) out += ',';
                out += std::to_string(g.coords[j]);
            }
            out += "],\"color\":";
            detail::writeJsonString(out, g.color);
            out += ",\"width\":";
            out += std::to_string(g.width);
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
