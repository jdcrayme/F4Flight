// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// json_io.cpp
//
// Minimal JSON serializer/deserializer for AircraftConfig.
//
// We write a strict subset of JSON: objects, arrays, strings (only for
// name/description/limiter types), numbers, booleans. The writer is trivial
// because we control the output. The reader is a small recursive-descent
// parser that handles the same subset.

#include "f4flight/json_io.h"
#include "f4flight/core/constants.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace f4flight::json {

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------
namespace {

// Format a double with enough precision to round-trip, but without trailing
// zeros or scientific notation for "normal" magnitudes.
std::string fmtNum(double v) {
    // Handle integers cleanly
    if (std::isfinite(v) && v == std::floor(v) && std::fabs(v) < 1e15) {
        std::ostringstream ss;
        ss.precision(0);
        ss << std::fixed << v;
        return ss.str();
    }
    std::ostringstream ss;
    ss.precision(15);
    ss << v;
    return ss.str();
}

std::string escapeString(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

// Indented JSON writer. Tracks indentation level.
class Writer {
public:
    void writeRaw(const std::string& s) { out_ += s; }
    void writeString(const std::string& s) {
        out_ += '"';
        out_ += escapeString(s);
        out_ += '"';
    }
    void writeNum(double v) { out_ += fmtNum(v); }
    void writeBool(bool v)  { out_ += v ? "true" : "false"; }
    void writeInt(int v)    { out_ += std::to_string(v); }

    void beginObj() { out_ += '{'; indent_++; needNL_ = true; first_ = true; }
    void endObj()   { indent_--; if (needNL_) nl(); else out_ += '\n'; pad(); out_ += '}'; first_ = false; needNL_ = true; }
    void beginArr() { out_ += '['; indent_++; needNL_ = true; first_ = true; }
    void endArr()   { indent_--; if (needNL_) nl(); else out_ += '\n'; pad(); out_ += ']'; first_ = false; needNL_ = true; }

    void key(const std::string& k) {
        if (!first_) out_ += ',';
        nl(); pad();
        out_ += '"';
        out_ += escapeString(k);
        out_ += "\": ";
        first_ = false;
        needNL_ = false; // value follows on same line
    }

    void beginObjValue() {
        out_ += '{';
        indent_++;
        first_ = true;
        needNL_ = true;
    }

    void beginArrValue() {
        out_ += '[';
        indent_++;
        first_ = true;
        needNL_ = true;
    }

    // Write a numeric value (continuing an array)
    void numValue(double v) {
        if (!first_) out_ += ", ";
        out_ += fmtNum(v);
        first_ = false;
    }

    // Write a string value (continuing an array)
    void strValue(const std::string& s) {
        if (!first_) out_ += ", ";
        out_ += '"';
        out_ += escapeString(s);
        out_ += '"';
        first_ = false;
    }

    void endValue() {
        // After a value in an object, prepare for next key
        first_ = false;
        needNL_ = true;
    }

    void nl()  { if (needNL_) { out_ += '\n'; } }
    void pad() { for (int i = 0; i < indent_; ++i) out_ += "  "; }

    void resetFirst() { first_ = true; }

    std::string str() const { return out_; }

private:
    std::string out_;
    int indent_{0};
    bool first_{true};     // true at start of obj/arr
    bool needNL_{false};   // true if next key/item should start on a new line
};

void writeDoubleArray(Writer& w, const std::vector<double>& v) {
    w.beginArr();
    w.resetFirst();
    for (double x : v) w.numValue(x);
    w.endArr();
}

[[maybe_unused]] void writeGearPoint(Writer& w, const std::string& key, const GearPoint& g) {
    w.key(key);
    w.beginObjValue();
    w.key("x");     w.writeNum(g.x);
    w.key("y");     w.writeNum(g.y);
    w.key("z");     w.writeNum(g.z);
    w.key("range"); w.writeNum(g.range);
    w.endObj();
    w.endValue();
}

void writeLimiter(Writer& w, const std::string& key, const Limiter& lim) {
    w.key(key);
    w.beginObjValue();
    std::string typeStr;
    switch (lim.type) {
        case LimiterType::Line:       typeStr = "Line"; break;
        case LimiterType::Value:      typeStr = "Value"; break;
        case LimiterType::Percent:    typeStr = "Percent"; break;
        case LimiterType::ThreePoint: typeStr = "ThreePoint"; break;
        case LimiterType::MinMax:     typeStr = "MinMax"; break;
    }
    w.key("type"); w.writeString(typeStr);
    w.key("x0"); w.writeNum(lim.x0);
    w.key("y0"); w.writeNum(lim.y0);
    w.key("x1"); w.writeNum(lim.x1);
    w.key("y1"); w.writeNum(lim.y1);
    w.key("x2"); w.writeNum(lim.x2);
    w.key("y2"); w.writeNum(lim.y2);
    w.endObj();
    w.endValue();
}

} // namespace

std::string write(const AircraftConfig& cfg) {
    Writer w;

    w.beginObj();
    w.resetFirst();

    // Metadata
    w.key("name");        w.writeString(cfg.name);
    w.key("description"); w.writeString(cfg.GetDescription);

    // Geometry
    w.key("geometry");
    w.beginObjValue();
    w.key("emptyWeight_lbs");  w.writeNum(cfg.geometry.emptyWeight_lbs);
    w.key("area_ft2");         w.writeNum(cfg.geometry.area_ft2);
    w.key("internalFuel_lbs"); w.writeNum(cfg.geometry.internalFuel_lbs);
    w.key("maxFuel_lbs");      w.writeNum(cfg.geometry.maxFuel_lbs);
    w.key("aoaMax_deg");       w.writeNum(cfg.geometry.aoaMax_deg);
    w.key("aoaMin_deg");       w.writeNum(cfg.geometry.aoaMin_deg);
    w.key("betaMax_deg");      w.writeNum(cfg.geometry.betaMax_deg);
    w.key("betaMin_deg");      w.writeNum(cfg.geometry.betaMin_deg);
    w.key("maxGs");            w.writeNum(cfg.geometry.maxGs);
    w.key("maxRoll_deg");      w.writeNum(cfg.geometry.maxRoll_deg);
    w.key("minVcas_kts");      w.writeNum(cfg.geometry.minVcas_kts);
    w.key("maxVcas_kts");      w.writeNum(cfg.geometry.maxVcas_kts);
    w.key("cornerVcas_kts");   w.writeNum(cfg.geometry.cornerVcas_kts);
    w.key("thetaMax_rad");     w.writeNum(cfg.geometry.thetaMax_rad);
    w.key("cgLoc_ft");         w.writeNum(cfg.geometry.cgLoc_ft);
    w.key("length_ft");        w.writeNum(cfg.geometry.length_ft);
    w.key("span_ft");          w.writeNum(cfg.geometry.span_ft);
    w.key("fusRadius_ft");     w.writeNum(cfg.geometry.fusRadius_ft);
    w.key("tailHt_ft");        w.writeNum(cfg.geometry.tailHt_ft);

    w.key("gear");
    w.beginArrValue();
    w.resetFirst();
    for (std::size_t i = 0; i < cfg.geometry.gear.size(); ++i) {
        if (i > 0) { w.writeRaw(", "); }
        w.beginObjValue();
        w.key("x");     w.writeNum(cfg.geometry.gear[i].x);
        w.key("y");     w.writeNum(cfg.geometry.gear[i].y);
        w.key("z");     w.writeNum(cfg.geometry.gear[i].z);
        w.key("range"); w.writeNum(cfg.geometry.gear[i].range);
        w.endObj();
    }
    w.endArr();
    w.endValue();
    w.endObj();
    w.endValue();

    // AuxAero
    w.key("aux");
    w.beginObjValue();
    w.key("fuelFlowFactorNormal"); w.writeNum(cfg.aux.fuelFlowFactorNormal);
    w.key("fuelFlowFactorAb");     w.writeNum(cfg.aux.fuelFlowFactorAb);
    w.key("minFuelFlow");          w.writeNum(cfg.aux.minFuelFlow);
    w.key("normSpoolRate");        w.writeNum(cfg.aux.normSpoolRate);
    w.key("abSpoolRate");          w.writeNum(cfg.aux.abSpoolRate);
    w.key("jfsSpoolUpRate");       w.writeNum(cfg.aux.jfsSpoolUpRate);
    w.key("jfsSpoolUpLimit");      w.writeNum(cfg.aux.jfsSpoolUpLimit);
    w.key("lightupSpoolRate");     w.writeNum(cfg.aux.lightupSpoolRate);
    w.key("flameoutSpoolRate");    w.writeNum(cfg.aux.flameoutSpoolRate);
    w.key("hasLef");               w.writeBool(cfg.aux.hasLef);
    w.key("hasTef");               w.writeBool(cfg.aux.hasTef);
    w.key("tefMaxAngle");          w.writeNum(cfg.aux.tefMaxAngle);
    w.key("lefMaxAngle");          w.writeNum(cfg.aux.lefMaxAngle);
    w.key("tefRate");              w.writeNum(cfg.aux.tefRate);
    w.key("lefRate");              w.writeNum(cfg.aux.lefRate);
    w.key("tefTakeOff");           w.writeNum(cfg.aux.tefTakeOff);
    w.key("lefGround");            w.writeNum(cfg.aux.lefGround);
    w.key("lefMaxMach");           w.writeNum(cfg.aux.lefMaxMach);
    w.key("rudderMaxAngle");       w.writeNum(cfg.aux.rudderMaxAngle);
    w.key("aileronMaxAngle");      w.writeNum(cfg.aux.aileronMaxAngle);
    w.key("airbrakeMaxAngle");     w.writeNum(cfg.aux.airbrakeMaxAngle);
    w.key("CLtefFactor");          w.writeNum(cfg.aux.CLtefFactor);
    w.key("CDtefFactor");          w.writeNum(cfg.aux.CDtefFactor);
    w.key("CDlefFactor");          w.writeNum(cfg.aux.CDlefFactor);
    w.key("CDSPDBFactor");         w.writeNum(cfg.aux.CDSPDBFactor);
    w.key("CDLDGFactor");          w.writeNum(cfg.aux.CDLDGFactor);
    w.key("dragChuteCd");          w.writeNum(cfg.aux.dragChuteCd);
    w.key("area2Span");            w.writeNum(cfg.aux.area2Span);
    w.key("rollMomentum");         w.writeNum(cfg.aux.rollMomentum);
    w.key("pitchMomentum");        w.writeNum(cfg.aux.pitchMomentum);
    w.key("yawMomentum");          w.writeNum(cfg.aux.yawMomentum);
    w.key("pitchElasticity");      w.writeNum(cfg.aux.pitchElasticity);
    w.key("sinkRate");             w.writeNum(cfg.aux.sinkRate);
    w.key("gearPitchFactor");      w.writeNum(cfg.aux.gearPitchFactor);
    w.key("rollGearGain");         w.writeNum(cfg.aux.rollGearGain);
    w.key("yawGearGain");          w.writeNum(cfg.aux.yawGearGain);
    w.key("pitchGearGain");        w.writeNum(cfg.aux.pitchGearGain);
    w.key("landingAOA");           w.writeNum(cfg.aux.landingAOA);
    w.key("rollCouple");           w.writeNum(cfg.aux.rollCouple);
    w.key("elevatorRolls");        w.writeBool(cfg.aux.elevatorRolls);
    w.key("criticalAOA");          w.writeNum(cfg.aux.criticalAOA);
    w.key("nEngines");             w.writeInt(cfg.aux.nEngines);
    w.key("typeEngine");           w.writeInt(cfg.aux.typeEngine);
    w.endObj();
    w.endValue();

    // Aero tables
    w.key("aero");
    w.beginObjValue();
    w.key("mach");     writeDoubleArray(w, cfg.aero.mach);
    w.key("alpha_deg"); writeDoubleArray(w, cfg.aero.alpha_deg);
    w.key("clift");    writeDoubleArray(w, cfg.aero.clift);
    w.key("cdrag");    writeDoubleArray(w, cfg.aero.cdrag);
    w.key("cy");       writeDoubleArray(w, cfg.aero.cy);
    w.key("clFactor"); w.writeNum(cfg.aero.clFactor);
    w.key("cdFactor"); w.writeNum(cfg.aero.cdFactor);
    w.key("cyFactor"); w.writeNum(cfg.aero.cyFactor);
    w.endObj();
    w.endValue();

    // Engine tables
    w.key("engine");
    w.beginObjValue();
    w.key("thrustFactor");  w.writeNum(cfg.engine.thrustFactor);
    w.key("fuelFlowFactor"); w.writeNum(cfg.engine.fuelFlowFactor);
    w.key("alt_ft");        writeDoubleArray(w, cfg.engine.alt_ft);
    w.key("mach");          writeDoubleArray(w, cfg.engine.mach);
    w.key("thrust_idle");   writeDoubleArray(w, cfg.engine.thrust_idle);
    w.key("thrust_mil");    writeDoubleArray(w, cfg.engine.thrust_mil);
    w.key("thrust_ab");     writeDoubleArray(w, cfg.engine.thrust_ab);
    w.key("fuelflow_idle"); writeDoubleArray(w, cfg.engine.fuelflow_idle);
    w.key("fuelflow_mil");  writeDoubleArray(w, cfg.engine.fuelflow_mil);
    w.key("fuelflow_ab");   writeDoubleArray(w, cfg.engine.fuelflow_ab);
    w.endObj();
    w.endValue();

    // Roll command table
    w.key("rollCmd");
    w.beginObjValue();
    w.key("alpha_deg"); writeDoubleArray(w, cfg.rollCmd.alpha_deg);
    w.key("qbar");      writeDoubleArray(w, cfg.rollCmd.qbar);
    w.key("rollRate");  writeDoubleArray(w, cfg.rollCmd.rollRate);
    w.key("scale");     w.writeNum(cfg.rollCmd.scale);
    w.endObj();
    w.endValue();

    // Limiters
    w.key("limiters");
    w.beginObjValue();
    for (int i = 0; i < static_cast<int>(LimiterKey::Count); ++i) {
        const Limiter& lim = cfg.limiters[i];
        // Only write non-default limiters (those with type != Line or with
        // non-zero values). For simplicity, write all of them.
        writeLimiter(w, std::to_string(i), lim);
    }
    w.endObj();
    w.endValue();

    // Top-level flags
    // NOTE: aoaCommandMode / aoaCommandMaxGs are NOT serialized -- they are
    // runtime defaults from the f4flight code, not data from the .dat file.
    // Keeping them out of the JSON ensures the JSON contains only data that
    // came from the .dat file (no "profile data").

    // Verbatim .dat capture (the "no data loss" channel).
    // rawAuxAeroData: every key/value pair from the .dat AuxAeroData section.
    w.key("rawAuxAeroData");
    w.beginObjValue();
    w.resetFirst();
    for (const auto& kv : cfg.rawAuxAeroData) {
        w.key(kv.first);
        w.writeString(kv.second);
    }
    w.endObj();
    w.endValue();

    // aeroOptions / engineOptions: literal option names from `aeropt`/`engopt`.
    w.key("aeroOptions");
    w.beginArrValue();
    w.resetFirst();
    for (const auto& opt : cfg.aeroOptions) w.strValue(opt);
    w.endArr();
    w.endValue();

    w.key("engineOptions");
    w.beginArrValue();
    w.resetFirst();
    for (const auto& opt : cfg.engineOptions) w.strValue(opt);
    w.endArr();
    w.endValue();

    // Source metadata for traceability.
    w.key("sourceTitle");     w.writeString(cfg.sourceTitle);
    w.key("sourceAuthor");    w.writeString(cfg.sourceAuthor);
    w.key("sourceRevision");  w.writeString(cfg.sourceRevision);
    w.key("sourceFile");      w.writeString(cfg.sourceFile);

    w.endObj();
    w.writeRaw("\n");
    return w.str();
}

bool writeFile(const AircraftConfig& cfg, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << write(cfg);
    return f.good();
}

// ---------------------------------------------------------------------------
// Reader: minimal recursive-descent JSON parser
// ---------------------------------------------------------------------------
namespace {

class Reader {
public:
    explicit Reader(const std::string& s) : s_(s), p_(0) {}

    void skipWs() {
        while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\t' ||
               s_[p_] == '\n' || s_[p_] == '\r')) ++p_;
    }

    char peek() { skipWs(); return (p_ < s_.size()) ? s_[p_] : '\0'; }

    char next() { skipWs(); return (p_ < s_.size()) ? s_[p_++] : '\0'; }

    void expect(char c) {
        if (next() != c) throw std::runtime_error(std::string("Expected '") + c + "'");
    }

    bool match(char c) {
        skipWs();
        if (p_ < s_.size() && s_[p_] == c) { ++p_; return true; }
        return false;
    }

    std::string readString() {
        skipWs();
        if (s_[p_] != '"') throw std::runtime_error("Expected string");
        ++p_;
        std::string out;
        while (p_ < s_.size() && s_[p_] != '"') {
            if (s_[p_] == '\\') {
                ++p_;
                if (p_ >= s_.size()) break;
                char e = s_[p_++];
                switch (e) {
                    case '"':  out += '"';  break;
                    case '\\': out += '\\'; break;
                    case '/':  out += '/';  break;
                    case 'b':  out += '\b'; break;
                    case 'f':  out += '\f'; break;
                    case 'n':  out += '\n'; break;
                    case 'r':  out += '\r'; break;
                    case 't':  out += '\t'; break;
                    case 'u': {
                        // Read 4 hex digits
                        if (p_ + 4 > s_.size()) throw std::runtime_error("Bad \\u escape");
                        int code = 0;
                        for (int i = 0; i < 4; ++i) {
                            char c = s_[p_++];
                            code <<= 4;
                            if (c >= '0' && c <= '9') code |= c - '0';
                            else if (c >= 'a' && c <= 'f') code |= c - 'a' + 10;
                            else if (c >= 'A' && c <= 'F') code |= c - 'A' + 10;
                            else throw std::runtime_error("Bad hex digit");
                        }
                        if (code < 0x80) out += static_cast<char>(code);
                        else if (code < 0x800) {
                            out += static_cast<char>(0xC0 | (code >> 6));
                            out += static_cast<char>(0x80 | (code & 0x3F));
                        } else {
                            out += static_cast<char>(0xE0 | (code >> 12));
                            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                            out += static_cast<char>(0x80 | (code & 0x3F));
                        }
                        break;
                    }
                    default: out += e; break;
                }
            } else {
                out += s_[p_++];
            }
        }
        if (p_ >= s_.size()) throw std::runtime_error("Unterminated string");
        ++p_; // consume closing quote
        return out;
    }

    double readNumber() {
        skipWs();
        std::size_t start = p_;
        if (p_ < s_.size() && (s_[p_] == '-' || s_[p_] == '+')) ++p_;
        while (p_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[p_])) ||
               s_[p_] == '.' || s_[p_] == 'e' || s_[p_] == 'E' ||
               s_[p_] == '+' || s_[p_] == '-')) ++p_;
        std::string tok = s_.substr(start, p_ - start);
        try {
            return std::stod(tok);
        } catch (...) {
            throw std::runtime_error("Bad number: " + tok);
        }
    }

    bool readBool() {
        skipWs();
        if (s_.compare(p_, 4, "true") == 0) { p_ += 4; return true; }
        if (s_.compare(p_, 5, "false") == 0) { p_ += 5; return false; }
        throw std::runtime_error("Expected true/false");
    }

    void readNull() {
        skipWs();
        if (s_.compare(p_, 4, "null") == 0) { p_ += 4; return; }
        throw std::runtime_error("Expected null");
    }

    // Read a value of unknown type and dispatch.
    // We provide overloads for the types we care about.

    void readInto(double& out) {
        skipWs();
        if (peek() == 'n') { readNull(); out = 0.0; return; }
        out = readNumber();
    }

    void readInto(bool& out) {
        skipWs();
        if (peek() == 't' || peek() == 'f') { out = readBool(); return; }
        // Accept 0/1 as bool
        out = (readNumber() != 0.0);
    }

    void readInto(int& out) {
        double d;
        readInto(d);
        out = static_cast<int>(d);
    }

    void readInto(std::string& out) {
        skipWs();
        if (peek() == '"') { out = readString(); return; }
        readNull(); out.clear();
    }

    void readInto(std::vector<double>& out) {
        skipWs();
        if (peek() == 'n') { readNull(); return; }
        expect('[');
        out.clear();
        skipWs();
        if (peek() == ']') { ++p_; return; }
        while (true) {
            out.push_back(readNumber());
            skipWs();
            if (peek() == ',') { ++p_; continue; }
            break;
        }
        expect(']');
    }

    // Skip an arbitrary value (used for unknown keys)
    void skipValue() {
        skipWs();
        char c = peek();
        if (c == '"') { readString(); return; }
        if (c == '{') {
            ++p_; // consume {
            skipWs();
            if (peek() == '}') { ++p_; return; }
            while (true) {
                readString(); // key
                skipWs();
                expect(':');
                skipValue();
                skipWs();
                if (peek() == ',') { ++p_; continue; }
                break;
            }
            expect('}');
            return;
        }
        if (c == '[') {
            ++p_;
            skipWs();
            if (peek() == ']') { ++p_; return; }
            while (true) {
                skipValue();
                skipWs();
                if (peek() == ',') { ++p_; continue; }
                break;
            }
            expect(']');
            return;
        }
        if (c == 't' || c == 'f') { readBool(); return; }
        if (c == 'n') { readNull(); return; }
        readNumber();
    }

    // Read an object's keys into a callback
    template <typename F>
    void readObject(F&& cb) {
        skipWs();
        expect('{');
        skipWs();
        if (peek() == '}') { ++p_; return; }
        while (true) {
            std::string key = readString();
            skipWs();
            expect(':');
            cb(key);
            skipWs();
            if (peek() == ',') { ++p_; continue; }
            break;
        }
        expect('}');
    }

    // Read an array of objects via callback
    template <typename F>
    void readObjectArray(F&& cb) {
        skipWs();
        expect('[');
        skipWs();
        if (peek() == ']') { ++p_; return; }
        while (true) {
            cb();
            skipWs();
            if (peek() == ',') { ++p_; continue; }
            break;
        }
        expect(']');
    }

private:
    const std::string& s_;
    std::size_t p_;
};

LimiterType parseLimiterType(const std::string& s) {
    if (s == "Line")       return LimiterType::Line;
    if (s == "Value")      return LimiterType::Value;
    if (s == "Percent")    return LimiterType::Percent;
    if (s == "ThreePoint") return LimiterType::ThreePoint;
    if (s == "MinMax")     return LimiterType::MinMax;
    return LimiterType::Line;
}

} // namespace

IoResult read(const std::string& jsonStr, AircraftConfig& cfg) {
    IoResult result;
    cfg = AircraftConfig{};

    try {
        Reader r(jsonStr);
        r.readObject([&](const std::string& key) {
            if (key == "name")        r.readInto(cfg.name);
            else if (key == "description") r.readInto(cfg.GetDescription);
            else if (key == "geometry") {
                r.readObject([&](const std::string& k) {
                    if (k == "emptyWeight_lbs")  r.readInto(cfg.geometry.emptyWeight_lbs);
                    else if (k == "area_ft2")         r.readInto(cfg.geometry.area_ft2);
                    else if (k == "internalFuel_lbs") r.readInto(cfg.geometry.internalFuel_lbs);
                    else if (k == "maxFuel_lbs")      r.readInto(cfg.geometry.maxFuel_lbs);
                    else if (k == "aoaMax_deg")       r.readInto(cfg.geometry.aoaMax_deg);
                    else if (k == "aoaMin_deg")       r.readInto(cfg.geometry.aoaMin_deg);
                    else if (k == "betaMax_deg")      r.readInto(cfg.geometry.betaMax_deg);
                    else if (k == "betaMin_deg")      r.readInto(cfg.geometry.betaMin_deg);
                    else if (k == "maxGs")            r.readInto(cfg.geometry.maxGs);
                    else if (k == "maxRoll_deg")      r.readInto(cfg.geometry.maxRoll_deg);
                    else if (k == "minVcas_kts")      r.readInto(cfg.geometry.minVcas_kts);
                    else if (k == "maxVcas_kts")      r.readInto(cfg.geometry.maxVcas_kts);
                    else if (k == "cornerVcas_kts")   r.readInto(cfg.geometry.cornerVcas_kts);
                    else if (k == "thetaMax_rad")     r.readInto(cfg.geometry.thetaMax_rad);
                    else if (k == "cgLoc_ft")         r.readInto(cfg.geometry.cgLoc_ft);
                    else if (k == "length_ft")        r.readInto(cfg.geometry.length_ft);
                    else if (k == "span_ft")          r.readInto(cfg.geometry.span_ft);
                    else if (k == "fusRadius_ft")     r.readInto(cfg.geometry.fusRadius_ft);
                    else if (k == "tailHt_ft")        r.readInto(cfg.geometry.tailHt_ft);
                    else if (k == "gear") {
                        r.readObjectArray([&]() {
                            GearPoint gp;
                            r.readObject([&](const std::string& gk) {
                                if (gk == "x")     r.readInto(gp.x);
                                else if (gk == "y") r.readInto(gp.y);
                                else if (gk == "z") r.readInto(gp.z);
                                else if (gk == "range") r.readInto(gp.range);
                                else r.skipValue();
                            });
                            cfg.geometry.gear.push_back(gp);
                        });
                    } else {
                        r.skipValue();
                    }
                });
            }
            else if (key == "aux") {
                r.readObject([&](const std::string& k) {
                    if (k == "fuelFlowFactorNormal") r.readInto(cfg.aux.fuelFlowFactorNormal);
                    else if (k == "fuelFlowFactorAb")     r.readInto(cfg.aux.fuelFlowFactorAb);
                    else if (k == "minFuelFlow")          r.readInto(cfg.aux.minFuelFlow);
                    else if (k == "normSpoolRate")        r.readInto(cfg.aux.normSpoolRate);
                    else if (k == "abSpoolRate")          r.readInto(cfg.aux.abSpoolRate);
                    else if (k == "jfsSpoolUpRate")       r.readInto(cfg.aux.jfsSpoolUpRate);
                    else if (k == "jfsSpoolUpLimit")      r.readInto(cfg.aux.jfsSpoolUpLimit);
                    else if (k == "lightupSpoolRate")     r.readInto(cfg.aux.lightupSpoolRate);
                    else if (k == "flameoutSpoolRate")    r.readInto(cfg.aux.flameoutSpoolRate);
                    else if (k == "hasLef")               r.readInto(cfg.aux.hasLef);
                    else if (k == "hasTef")               r.readInto(cfg.aux.hasTef);
                    else if (k == "tefMaxAngle")          r.readInto(cfg.aux.tefMaxAngle);
                    else if (k == "lefMaxAngle")          r.readInto(cfg.aux.lefMaxAngle);
                    else if (k == "tefRate")              r.readInto(cfg.aux.tefRate);
                    else if (k == "lefRate")              r.readInto(cfg.aux.lefRate);
                    else if (k == "tefTakeOff")           r.readInto(cfg.aux.tefTakeOff);
                    else if (k == "lefGround")            r.readInto(cfg.aux.lefGround);
                    else if (k == "lefMaxMach")           r.readInto(cfg.aux.lefMaxMach);
                    else if (k == "rudderMaxAngle")       r.readInto(cfg.aux.rudderMaxAngle);
                    else if (k == "aileronMaxAngle")      r.readInto(cfg.aux.aileronMaxAngle);
                    else if (k == "airbrakeMaxAngle")     r.readInto(cfg.aux.airbrakeMaxAngle);
                    else if (k == "CLtefFactor")          r.readInto(cfg.aux.CLtefFactor);
                    else if (k == "CDtefFactor")          r.readInto(cfg.aux.CDtefFactor);
                    else if (k == "CDlefFactor")          r.readInto(cfg.aux.CDlefFactor);
                    else if (k == "CDSPDBFactor")         r.readInto(cfg.aux.CDSPDBFactor);
                    else if (k == "CDLDGFactor")          r.readInto(cfg.aux.CDLDGFactor);
                    else if (k == "dragChuteCd")          r.readInto(cfg.aux.dragChuteCd);
                    else if (k == "area2Span")            r.readInto(cfg.aux.area2Span);
                    else if (k == "rollMomentum")         r.readInto(cfg.aux.rollMomentum);
                    else if (k == "pitchMomentum")        r.readInto(cfg.aux.pitchMomentum);
                    else if (k == "yawMomentum")          r.readInto(cfg.aux.yawMomentum);
                    else if (k == "pitchElasticity")      r.readInto(cfg.aux.pitchElasticity);
                    else if (k == "sinkRate")             r.readInto(cfg.aux.sinkRate);
                    else if (k == "gearPitchFactor")      r.readInto(cfg.aux.gearPitchFactor);
                    else if (k == "rollGearGain")         r.readInto(cfg.aux.rollGearGain);
                    else if (k == "yawGearGain")          r.readInto(cfg.aux.yawGearGain);
                    else if (k == "pitchGearGain")        r.readInto(cfg.aux.pitchGearGain);
                    else if (k == "landingAOA")           r.readInto(cfg.aux.landingAOA);
                    else if (k == "rollCouple")           r.readInto(cfg.aux.rollCouple);
                    else if (k == "elevatorRolls")        r.readInto(cfg.aux.elevatorRolls);
                    else if (k == "criticalAOA")          r.readInto(cfg.aux.criticalAOA);
                    else if (k == "nEngines")             r.readInto(cfg.aux.nEngines);
                    else if (k == "typeEngine")           r.readInto(cfg.aux.typeEngine);
                    else r.skipValue();
                });
            }
            else if (key == "aero") {
                r.readObject([&](const std::string& k) {
                    if (k == "mach")        r.readInto(cfg.aero.mach);
                    else if (k == "alpha_deg") r.readInto(cfg.aero.alpha_deg);
                    else if (k == "clift")  r.readInto(cfg.aero.clift);
                    else if (k == "cdrag")  r.readInto(cfg.aero.cdrag);
                    else if (k == "cy")     r.readInto(cfg.aero.cy);
                    else if (k == "clFactor") r.readInto(cfg.aero.clFactor);
                    else if (k == "cdFactor") r.readInto(cfg.aero.cdFactor);
                    else if (k == "cyFactor") r.readInto(cfg.aero.cyFactor);
                    else r.skipValue();
                });
            }
            else if (key == "engine") {
                r.readObject([&](const std::string& k) {
                    if (k == "thrustFactor")  r.readInto(cfg.engine.thrustFactor);
                    else if (k == "fuelFlowFactor") r.readInto(cfg.engine.fuelFlowFactor);
                    else if (k == "alt_ft")         r.readInto(cfg.engine.alt_ft);
                    else if (k == "mach")           r.readInto(cfg.engine.mach);
                    else if (k == "thrust_idle")    r.readInto(cfg.engine.thrust_idle);
                    else if (k == "thrust_mil")     r.readInto(cfg.engine.thrust_mil);
                    else if (k == "thrust_ab")      r.readInto(cfg.engine.thrust_ab);
                    else if (k == "fuelflow_idle")  r.readInto(cfg.engine.fuelflow_idle);
                    else if (k == "fuelflow_mil")   r.readInto(cfg.engine.fuelflow_mil);
                    else if (k == "fuelflow_ab")    r.readInto(cfg.engine.fuelflow_ab);
                    else r.skipValue();
                });
            }
            else if (key == "rollCmd") {
                r.readObject([&](const std::string& k) {
                    if (k == "alpha_deg") r.readInto(cfg.rollCmd.alpha_deg);
                    else if (k == "qbar") r.readInto(cfg.rollCmd.qbar);
                    else if (k == "rollRate") r.readInto(cfg.rollCmd.rollRate);
                    else if (k == "scale") r.readInto(cfg.rollCmd.scale);
                    else r.skipValue();
                });
            }
            else if (key == "limiters") {
                r.readObject([&](const std::string& k) {
                    int idx = std::atoi(k.c_str());
                    if (idx < 0 || idx >= static_cast<int>(LimiterKey::Count)) {
                        r.skipValue();
                        return;
                    }
                    Limiter lim;
                    r.readObject([&](const std::string& lk) {
                        if (lk == "type") {
                            std::string t;
                            r.readInto(t);
                            lim.type = parseLimiterType(t);
                        } else if (lk == "x0") r.readInto(lim.x0);
                        else if (lk == "y0") r.readInto(lim.y0);
                        else if (lk == "x1") r.readInto(lim.x1);
                        else if (lk == "y1") r.readInto(lim.y1);
                        else if (lk == "x2") r.readInto(lim.x2);
                        else if (lk == "y2") r.readInto(lim.y2);
                        else r.skipValue();
                    });
                    cfg.limiters[idx] = lim;
                });
            }
            else if (key == "rawAuxAeroData") {
                r.readObject([&](const std::string& k) {
                    std::string v;
                    r.readInto(v);
                    cfg.rawAuxAeroData[k] = v;
                });
            }
            else if (key == "aeroOptions") {
                r.readObjectArray([&]() {
                    std::string s;
                    r.readInto(s);
                    cfg.aeroOptions.push_back(s);
                });
            }
            else if (key == "engineOptions") {
                r.readObjectArray([&]() {
                    std::string s;
                    r.readInto(s);
                    cfg.engineOptions.push_back(s);
                });
            }
            else if (key == "sourceTitle")     r.readInto(cfg.sourceTitle);
            else if (key == "sourceAuthor")    r.readInto(cfg.sourceAuthor);
            else if (key == "sourceRevision")  r.readInto(cfg.sourceRevision);
            else if (key == "sourceFile")      r.readInto(cfg.sourceFile);
            // aoaCommandMode / aoaCommandMaxGs are intentionally NOT read
            // from JSON -- they are runtime defaults, not .dat data. Old
            // JSONs that still contain these keys will simply skip them.
            else if (key == "aoaCommandMode")  r.skipValue();
            else if (key == "aoaCommandMaxGs") r.skipValue();
            else r.skipValue();
        });

        // If maxFuel wasn't set, default to internalFuel
        if (cfg.geometry.maxFuel_lbs <= 0.0) cfg.geometry.maxFuel_lbs = cfg.geometry.internalFuel_lbs;

        // If we loaded rawAuxAeroData but the typed `aux` struct wasn't
        // populated (e.g. JSON written by an older version that only had
        // the typed aux object), that's fine -- aux keeps its defaults.
        // If both are present, the typed aux object was already populated
        // from the JSON's "aux" section above; rawAuxAeroData is the
        // authoritative full record.

        result.ok = true;
    } catch (const std::exception& e) {
        result.errors.push_back(e.what());
        result.ok = false;
    }

    return result;
}

IoResult readFile(const std::string& path, AircraftConfig& cfg) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        IoResult r;
        r.errors.push_back("Could not open file: " + path);
        r.ok = false;
        return r;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return read(ss.str(), cfg);
}

} // namespace f4flight::json
