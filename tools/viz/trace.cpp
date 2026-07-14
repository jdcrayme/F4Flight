// f4flight - tools/trace.cpp
//
// Trace recording implementation — JSON serialization for maneuver traces.

#include "trace.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <cmath>

namespace f4flight {

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

void TraceRecorder::markPhase(const std::string& name, double start_s, double end_s,
                               bool passed, bool skipped) {
    trace_.phases.push_back({name, start_s, end_s, passed, skipped});
}

void TraceRecorder::finish(double duration_s) {
    trace_.duration_s = duration_s;
}

// ---------------------------------------------------------------------------
// JSON writing
// ---------------------------------------------------------------------------

static void writeString(std::string& out, const std::string& s) {
    out += '"';
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += '"';
}

static void writeFrame(std::string& out, const TraceFrame& f) {
    out += "  {\"t\":";
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
    writeString(out, f.mode);
    out += ",\"phase\":";
    writeString(out, f.phase);
    if (!f.threats.empty()) {
        out += ",\"threats\":[";
        for (size_t i = 0; i < f.threats.size(); ++i) {
            if (i > 0) out += ',';
            const auto& e = f.threats[i];
            out += "{\"type\":";
            writeString(out, e.type);
            out += ",\"x\":";
            out += std::to_string(e.x);
            out += ",\"y\":";
            out += std::to_string(e.y);
            out += ",\"z\":";
            out += std::to_string(e.z);
            out += ",\"speed\":";
            out += std::to_string(e.speed);
            out += "}";
        }
        out += "]";
    }
    out += "}";
}

bool TraceRecorder::write(const std::string& path) const {
    std::string json;
    json.reserve(trace_.frames.size() * 200);
    json += "{\n";

    // Metadata
    json += "  \"aircraft\": ";
    writeString(json, trace_.aircraft);
    json += ",\n  \"scenario\": ";
    writeString(json, trace_.scenario);
    json += ",\n  \"duration_s\": ";
    json += std::to_string(trace_.duration_s);

    // Phases
    json += ",\n  \"phases\": [\n";
    for (size_t i = 0; i < trace_.phases.size(); ++i) {
        if (i > 0) json += ",\n";
        const auto& p = trace_.phases[i];
        json += "    {\"name\":";
        writeString(json, p.name);
        json += ",\"start_s\":";
        json += std::to_string(p.start_s);
        json += ",\"end_s\":";
        json += std::to_string(p.end_s);
        json += ",\"passed\":";
        json += p.passed ? "true" : "false";
        json += ",\"skipped\":";
        json += p.skipped ? "true" : "false";
        json += "}";
    }
    json += "\n  ],\n";

    // Frames
    json += "  \"frames\": [\n";
    for (size_t i = 0; i < trace_.frames.size(); ++i) {
        if (i > 0) json += ",\n";
        writeFrame(json, trace_.frames[i]);
    }
    json += "\n  ]\n";

    json += "}\n";

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
