// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// dat_loader.cpp
//
// Implementation of the .dat file parser. Direct port of the relevant
// portions of FreeFalcon's readin.cpp, limiters.cpp, and the AuxAeroDataDesc
// table, but rewritten as a clean recursive-descent parser rather than the
// legacy "GetNext() / ReadLine()" sequential reader.
//
// The parser is intentionally permissive: unknown keys and unknown sections
// are skipped silently (or recorded as warnings) rather than aborting.

#include "f4flight/dat_loader.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace f4flight::dat {

// ---------------------------------------------------------------------------
// Tokenizer
//
// The .dat format uses '#' for line comments and whitespace as the token
// separator. We read the whole file into a string, strip comments, and
// produce a flat token stream.
// ---------------------------------------------------------------------------
class TokenStream {
public:
    explicit TokenStream(const std::string& source, const std::string& sourceName)
        : sourceName_(sourceName) {
        // Strip line comments
        std::string stripped;
        stripped.reserve(source.size());
        std::size_t i = 0;
        while (i < source.size()) {
            char c = source[i];
            if (c == '#') {
                // Skip to end of line
                while (i < source.size() && source[i] != '\n') ++i;
            } else {
                stripped.push_back(c);
                ++i;
            }
        }
        // Tokenize on whitespace
        std::istringstream iss(stripped);
        std::string tok;
        while (iss >> tok) tokens_.push_back(tok);
    }

    // Peek at the next token without consuming.
    const std::string& peek() const {
        static const std::string empty;
        return (pos_ < tokens_.size()) ? tokens_[pos_] : empty;
    }

    bool eof() const { return pos_ >= tokens_.size(); }

    // Consume and return the next token. Throws if EOF.
    std::string next() {
        if (pos_ >= tokens_.size()) {
            throw std::runtime_error("Unexpected EOF in " + sourceName_);
        }
        return tokens_[pos_++];
    }

    // Consume the next token and parse as double. Throws on parse failure.
    double nextDouble() {
        std::string tok = next();
        try {
            return std::stod(tok);
        } catch (const std::exception&) {
            throw std::runtime_error("Failed to parse '" + tok + "' as double in " + sourceName_);
        }
    }

    // Consume the next token and parse as int.
    int nextInt() {
        std::string tok = next();
        try {
            return std::stoi(tok);
        } catch (const std::exception&) {
            throw std::runtime_error("Failed to parse '" + tok + "' as int in " + sourceName_);
        }
    }

    // Consume the next N tokens as doubles.
    std::vector<double> nextDoubles(std::size_t n) {
        std::vector<double> v;
        v.reserve(n);
        for (std::size_t i = 0; i < n; ++i) v.push_back(nextDouble());
        return v;
    }

    std::size_t pos() const { return pos_; }
    void setPos(std::size_t p) { pos_ = p; }
    std::size_t size() const { return tokens_.size(); }

private:
    std::string sourceName_;
    std::vector<std::string> tokens_;
    std::size_t pos_{0};
};

// ---------------------------------------------------------------------------
// Section markers. The .dat file is organised into sections delimited by
// specific keyword tokens. We search forward for each marker.
// ---------------------------------------------------------------------------

// Find the position of the next occurrence of a literal token (case-sensitive).
// Returns true if found; the stream's position is left AT that token (not
// past it). If not found, the stream position is restored to where it was
// before the call.
static bool findToken(TokenStream& ts, const std::string& needle) {
    std::size_t startPos = ts.pos();
    while (!ts.eof()) {
        if (ts.next() == needle) {
            ts.setPos(ts.pos() - 1);  // step back to the matching token
            return true;
        }
    }
    ts.setPos(startPos);  // not found -- restore position
    return false;
}

// Find the next token whose value starts with `prefix`.
[[maybe_unused]] static bool findTokenPrefix(TokenStream& ts, const std::string& prefix) {
    while (!ts.eof()) {
        std::string tok = ts.next();
        if (tok.size() >= prefix.size() && tok.compare(0, prefix.size(), prefix) == 0) {
            // Step back so the caller can read it
            ts.setPos(ts.pos() - 1);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Parse the input-data block (51 positional fields + gear points).
// Direct port of ReadData() in readin.cpp.
// ---------------------------------------------------------------------------
static void parseInputData(TokenStream& ts, AircraftConfig& cfg,
                           std::vector<std::string>& warnings) {
    // The block starts at "INPUT MASS AND GEOMETRIC PROPERTIES" -- but since
    // we already stripped comments, the section header is gone. Instead we
    // search for the first numeric token after the file title, which is the
    // empty weight. We use a heuristic: find the very first token that parses
    // as a number > 1000 (an empty weight in lbs) and consume from there.

    // Restart from the beginning.
    ts.setPos(0);

    // Skip the title line (3 tokens: "Title:", "Author:", "Revision:")
    // Actually those are comments, so they're already stripped. The first
    // tokens after stripping are the empty weight, area, fuel, etc.

    // Empty weight
    cfg.geometry.emptyWeight_lbs = ts.nextDouble();
    // Reference area
    cfg.geometry.area_ft2 = ts.nextDouble();
    // Internal fuel
    cfg.geometry.internalFuel_lbs = ts.nextDouble();
    cfg.geometry.maxFuel_lbs = cfg.geometry.internalFuel_lbs;

    // AOA / sideslip limits
    cfg.geometry.aoaMax_deg   = ts.nextDouble();
    cfg.geometry.aoaMin_deg   = ts.nextDouble();
    cfg.geometry.betaMax_deg  = ts.nextDouble();
    cfg.geometry.betaMin_deg  = ts.nextDouble();
    cfg.geometry.maxGs        = ts.nextDouble();
    cfg.geometry.maxRoll_deg  = ts.nextDouble();
    cfg.geometry.minVcas_kts  = ts.nextDouble();
    cfg.geometry.maxVcas_kts  = ts.nextDouble();
    cfg.geometry.cornerVcas_kts = ts.nextDouble();
    cfg.geometry.thetaMax_rad = ts.nextDouble() * DTR; // stored as radians
    int numGear = ts.nextInt();
    cfg.geometry.gear.resize(numGear);

    // Gear table (xPos, yPos, zPos, extent) per gear point
    for (int i = 0; i < numGear; ++i) {
        cfg.geometry.gear[i].x = ts.nextDouble();
        cfg.geometry.gear[i].y = ts.nextDouble();
        cfg.geometry.gear[i].z = ts.nextDouble();
        cfg.geometry.gear[i].range = ts.nextDouble();
    }

    // Physical data
    cfg.geometry.cgLoc_ft     = ts.nextDouble();
    cfg.geometry.length_ft    = ts.nextDouble();
    cfg.geometry.span_ft      = ts.nextDouble();
    cfg.geometry.fusRadius_ft = ts.nextDouble();
    cfg.geometry.tailHt_ft    = ts.nextDouble();

    (void)warnings;
}

// ---------------------------------------------------------------------------
// Parse a 2D Mach x alpha coefficient table (CL, CD, or CY).
// Direct port of AirframeAeroRead().
//
// The CL table is the "full" form:
//   <numMach>
//   <mach[0]> ... <mach[numMach-1]>
//   <numAlpha>
//   <alpha[0]> ... <alpha[numAlpha-1]>
//   <clFactor>           # table multiplier
//   <clift[mach=0][alpha=0..numAlpha-1]>   # laid out as mach*numAlpha + alpha
//   <clift[mach=1][...]>
//   ...
//
// The CD and CY tables reuse the same Mach/alpha breakpoints (they are NOT
// repeated in the file). Their layout is:
//   <cdFactor>           # table multiplier
//   <cdrag[mach=0][alpha=0..numAlpha-1]>
//   <cdrag[mach=1][...]>
//   ...
// ---------------------------------------------------------------------------
static void parseAeroTable(TokenStream& ts, AeroTable& table,
                           const std::string& tableName,
                           std::vector<std::string>& warnings) {

    int numMach, numAlpha;

    if (tableName == "CL") {
        // Full form: read breakpoints
        numMach = ts.nextInt();
        if (numMach <= 0 || numMach > 1000) {
            warnings.push_back(tableName + ": implausible numMach=" + std::to_string(numMach));
            return;
        }
        table.mach = ts.nextDoubles(static_cast<std::size_t>(numMach));

        numAlpha = ts.nextInt();
        if (numAlpha <= 0 || numAlpha > 1000) {
            warnings.push_back(tableName + ": implausible numAlpha=" + std::to_string(numAlpha));
            return;
        }
        table.alpha_deg = ts.nextDoubles(static_cast<std::size_t>(numAlpha));
    } else {
        // CD / CY: reuse the breakpoints from the CL table that was already
        // parsed.
        numMach  = static_cast<int>(table.mach.size());
        numAlpha = static_cast<int>(table.alpha_deg.size());
        if (numMach == 0 || numAlpha == 0) {
            warnings.push_back(tableName + ": CL table must be parsed first");
            return;
        }
    }

    // Table multiplier
    double factor = ts.nextDouble();

    // Read numMach * numAlpha values
    std::vector<double> data;
    data.reserve(static_cast<std::size_t>(numMach) * numAlpha);
    for (int i = 0; i < numMach * numAlpha; ++i) {
        data.push_back(ts.nextDouble());
    }

    if (tableName == "CL") {
        table.clift = std::move(data);
        table.clFactor = factor;
    } else if (tableName == "CD") {
        // Legacy: readin.cpp multiplies CD by 1.5 on read. We preserve that.
        for (auto& v : data) v *= 1.5;
        table.cdrag = std::move(data);
        table.cdFactor = factor;
    } else if (tableName == "CY") {
        table.cy = std::move(data);
        table.cyFactor = factor;
    } else {
        warnings.push_back("Unknown aero table name: " + tableName);
    }
}

// ---------------------------------------------------------------------------
// Parse the propulsion data block.
// Direct port of AirframeEngineRead().
//
// The engine section starts at the "PROPULSION DATA" marker. Layout:
//   <engopt...>
//   <thrustFactor>
//   <fuelFlowFactor>
//   <numMach>
//   <mach[0..numMach-1]>
//   <numAlt>
//   <alt[0..numAlt-1]>
//   <numAlpha>             # alpha breakpoints for thrust-alpha factor
//   <alpha[0..numAlpha-1]>
//   <thrustAlphaFactor[alt=0][mach=0..numMach-1]>  ... wait, actually it's
//      <thrustAlphaFactor[alt=0][alpha=0..numAlpha-1]> (alpha is inner)
//   <thrustAlphaFactor[alt=1][...]>
//   ... (numAlt rows)
//   <thrust_idle[alt=0][mach=0..numMach-1]>
//   <thrust_idle[alt=1][...]>
//   ... (numAlt rows)
//   <thrust_mil[...]>
//   <thrust_ab[...]>
//   <fuelflow_idle[...]>
//   <fuelflow_mil[...]>
//   <fuelflow_ab[...]>
//
// NOTE: the thrust tables are indexed as [alt*numMach + mach] (altitude is
// outer, mach is inner). The thrust-alpha-factor table is indexed as
// [alt*numAlpha + alpha] (altitude is outer, alpha is inner).
// ---------------------------------------------------------------------------
static void parseEngine(TokenStream& ts, AircraftConfig& cfg,
                        std::vector<std::string>& warnings) {
    // The "PROPULSION DATA" section header is a comment, so it was stripped.
    // Some files start the engine block with "engopt" tokens (modern format);
    // others go directly to the thrust multiplier (legacy format). We search
    // for "engopt" first; if not found, we try to find the engine block by
    // scanning for a pattern of two doubles (thrustFactor, fuelFlowFactor)
    // followed by a small integer (numMach) after the aero tables.

    // First, try to find "engopt" -- but only search from the CURRENT
    // position (after the aero tables), not from the beginning, to avoid
    // false positives in the aero data.
    std::size_t searchStart = ts.pos();
    bool foundEngopt = findToken(ts, "engopt");
    bool hasFuelFlowOpt = false;
    if (foundEngopt) {
        // findToken leaves the stream positioned AT the "engopt" token.
        // Skip any "engopt" tokens. Check if "fuelflow" is one of the options.
        while (ts.peek() == "engopt") {
            ts.next(); // consume "engopt"
            std::string optName = ts.next(); // option name
            if (optName == "fuelflow") hasFuelFlowOpt = true;
        }
    } else {
        // Legacy format: no engopt. Restore the position to where we started
        // searching (after the aero tables) and scan forward from there.
        ts.setPos(searchStart);
        // Scan forward looking for the engine block. We validate strictly:
        // - Two doubles near 1.0 (thrustFactor, fuelFlowFactor)
        // - An integer numMach >= 5 (engine tables always have many breakpoints)
        // - Mach breakpoints all in [0, 5]
        // - An integer numAlt >= 3
        // - Alt breakpoints all in [0, 100000]
        // - Enough remaining tokens for 3 thrust tables (3 * numAlt * numMach)
        bool found = false;
        while (!ts.eof()) {
            std::size_t saved = ts.pos();
            try {
                double f1 = ts.nextDouble();
                double f2 = ts.nextDouble();
                int nm = ts.nextInt();
                if (nm < 5 || nm > 50 ||
                    std::fabs(f1 - 1.0) > 2.0 || std::fabs(f2 - 1.0) > 2.0) {
                    ts.setPos(saved + 1);
                    continue;
                }
                // Read mach breakpoints
                std::vector<double> machs = ts.nextDoubles(static_cast<std::size_t>(nm));
                bool machsOk = true;
                for (double m : machs) {
                    if (m < 0.0 || m > 5.0) { machsOk = false; break; }
                }
                if (!machsOk) { ts.setPos(saved + 1); continue; }

                // Read numAlt
                int na = ts.nextInt();
                if (na < 3 || na > 20) { ts.setPos(saved + 1); continue; }

                // Read alt breakpoints
                std::vector<double> alts = ts.nextDoubles(static_cast<std::size_t>(na));
                bool altsOk = true;
                for (double a : alts) {
                    if (a < 0.0 || a > 100000.0) { altsOk = false; break; }
                }
                if (!altsOk) { ts.setPos(saved + 1); continue; }

                // Check there are enough tokens for at least the 3 thrust tables
                std::size_t needed = 3 * static_cast<std::size_t>(na) * nm;
                if (ts.pos() + needed > ts.size()) { ts.setPos(saved + 1); continue; }

                // Found a valid engine block. Roll back to `saved`.
                ts.setPos(saved);
                found = true;
                break;
            } catch (const std::exception&) {
                ts.setPos(saved + 1);
            }
        }
        if (!found) {
            warnings.push_back("Engine: could not locate engine block");
            return;
        }
    }

    // At this point, the stream is positioned at the thrust multiplier
    // (either just after the engopt tokens, or at the `saved` position from
    // the legacy scan).
    EngineTable& e = cfg.engine;
    e.thrustFactor = ts.nextDouble();
    e.fuelFlowFactor = ts.nextDouble();

    int numMach = ts.nextInt();
    e.mach = ts.nextDoubles(static_cast<std::size_t>(numMach));

    int numAlt = ts.nextInt();
    e.alt_ft = ts.nextDoubles(static_cast<std::size_t>(numAlt));

    // Some .dat files have an optional "Alpha BREAKPOINTS" section + a
    // thrust-alpha-factor table (numAlt * numAlpha values) between the
    // altitude breakpoints and the thrust tables. Others skip this and go
    // directly to the thrust tables. We detect which format we have by
    // peeking at the next token: if it's a small integer (2..50) followed
    // by that many alpha-like values, we have the alpha-breakpoints format.
    // Otherwise, we skip directly to the thrust tables.
    int numAlpha = 0;
    std::size_t savedPos = ts.pos();
    bool hasAlphaFactor = false;
    try {
        int maybeNumAlpha = ts.nextInt();
        if (maybeNumAlpha >= 2 && maybeNumAlpha <= 50) {
            // Try to read that many alpha values
            std::vector<double> alphas = ts.nextDoubles(static_cast<std::size_t>(maybeNumAlpha));
            // Check that the values look like angles (-90 to 90 deg)
            bool alphasOk = true;
            for (double a : alphas) {
                if (a < -90.0 || a > 90.0) { alphasOk = false; break; }
            }
            // Check that there are enough remaining tokens for the
            // thrust-alpha-factor table (numAlt * maybeNumAlpha values)
            // plus the 3 thrust tables (3 * numAlt * numMach values).
            std::size_t needed = static_cast<std::size_t>(numAlt) * maybeNumAlpha
                               + 3 * static_cast<std::size_t>(numAlt) * numMach;
            if (alphasOk && ts.pos() + needed <= ts.size()) {
                numAlpha = maybeNumAlpha;
                hasAlphaFactor = true;
                // Consume the thrust-alpha-factor table
                for (int a = 0; a < numAlt; ++a) {
                    for (int al = 0; al < numAlpha; ++al) {
                        ts.nextDouble();
                    }
                }
            } else {
                // Roll back -- this isn't an alpha-breakpoints section
                ts.setPos(savedPos);
            }
        } else {
            ts.setPos(savedPos);
        }
    } catch (const std::exception&) {
        // Roll back on any parse error
        ts.setPos(savedPos);
    }

    // Thrust tables: numAlt rows of numMach values each.
    // Order: idle, mil, AB.
    e.thrust_idle.resize(static_cast<std::size_t>(numAlt) * numMach);
    e.thrust_mil.resize (static_cast<std::size_t>(numAlt) * numMach);
    e.thrust_ab.resize  (static_cast<std::size_t>(numAlt) * numMach);

    auto readThrustTable = [&](std::vector<double>& out) {
        for (int a = 0; a < numAlt; ++a) {
            for (int m = 0; m < numMach; ++m) {
                out[static_cast<std::size_t>(a) * numMach + m] = ts.nextDouble();
            }
        }
    };
    readThrustTable(e.thrust_idle);
    readThrustTable(e.thrust_mil);
    readThrustTable(e.thrust_ab);

    // Optional fuel flow tables. Only read them if "engopt fuelflow" was
    // specified (modern format). Legacy files without engopt don't have
    // fuel flow tables.
    if (hasFuelFlowOpt && !ts.eof()) {
        e.fuelflow_idle.resize(static_cast<std::size_t>(numAlt) * numMach);
        e.fuelflow_mil .resize(static_cast<std::size_t>(numAlt) * numMach);
        e.fuelflow_ab  .resize(static_cast<std::size_t>(numAlt) * numMach);
        readThrustTable(e.fuelflow_idle);
        readThrustTable(e.fuelflow_mil);
        readThrustTable(e.fuelflow_ab);
    }

    (void)hasAlphaFactor;
    (void)warnings;
}

// ---------------------------------------------------------------------------
// Parse the roll-rate command table.
// Direct port of AirframeFcsRead().
//
// The roll section is preceded by several small "chart" tables (IdleThrottle,
// MilRPM, MaxABRPM, Nozzle Max, a Rate, Function66) that we must skip. Since
// all section headers are comments (and thus stripped), we can't search for
// a marker. Instead, we scan forward and try to parse each candidate position
// as a roll table; if it validates (numAlpha in [2..20], numQbar in [2..20],
// alpha values look like angles, qbar values look like pressures, scale is
// reasonable, and there are enough remaining tokens for numAlpha*numQbar
// values), we accept it.
// ---------------------------------------------------------------------------
static void parseRollData(TokenStream& ts, AircraftConfig& cfg,
                          std::vector<std::string>& warnings) {
    // Scan the ENTIRE token stream from the beginning looking for a valid
    // roll table. This is more robust than scanning from the current
    // position (which may be wrong if the engine parser overran).
    ts.setPos(0);

    // Scan forward looking for a valid roll table.
    while (!ts.eof()) {
        std::size_t savedPos = ts.pos();

        // Try to parse a roll table starting here
        try {
            int numAlpha = ts.nextInt();
            if (numAlpha < 2 || numAlpha > 20) { ts.setPos(savedPos + 1); continue; }

            std::vector<double> alphas = ts.nextDoubles(static_cast<std::size_t>(numAlpha));
            // Alpha values should be in a reasonable range (-30 to 90 deg)
            bool alphasOk = true;
            for (double a : alphas) {
                if (a < -30.0 || a > 90.0) { alphasOk = false; break; }
            }
            if (!alphasOk) { ts.setPos(savedPos + 1); continue; }

            int numQbar = ts.nextInt();
            if (numQbar < 2 || numQbar > 20) { ts.setPos(savedPos + 1); continue; }

            std::vector<double> qbars = ts.nextDoubles(static_cast<std::size_t>(numQbar));
            // Qbar values should be non-negative
            bool qbarsOk = true;
            for (double q : qbars) {
                if (q < 0.0 || q > 10000.0) { qbarsOk = false; break; }
            }
            if (!qbarsOk) { ts.setPos(savedPos + 1); continue; }

            double scale = ts.nextDouble();
            if (scale < 0.01 || scale > 100.0) { ts.setPos(savedPos + 1); continue; }

            // Need numAlpha * numQbar more values
            std::size_t needed = static_cast<std::size_t>(numAlpha) * numQbar;
            if (ts.pos() + needed > ts.size()) { ts.setPos(savedPos + 1); continue; }

            std::vector<double> rates = ts.nextDoubles(needed);

            // Roll rates should be in a plausible range (0..400 deg/s)
            bool ratesOk = true;
            for (double r : rates) {
                if (r < -50.0 || r > 500.0) { ratesOk = false; break; }
            }
            if (!ratesOk) { ts.setPos(savedPos + 1); continue; }

            // Accept!
            cfg.rollCmd.alpha_deg = std::move(alphas);
            cfg.rollCmd.qbar      = std::move(qbars);
            cfg.rollCmd.scale     = scale;
            cfg.rollCmd.rollRate  = std::move(rates);
            return;
        } catch (const std::exception&) {
            ts.setPos(savedPos + 1);
            continue;
        }
    }
    warnings.push_back("RollData: could not locate a valid roll table");
}

// ---------------------------------------------------------------------------
// Parse the LIMITERS block.
// Direct port of LimiterMgrClass::ReadLimiters() in limiters.cpp.
//
// The "LIMITERS" header is a comment, so it's stripped. We detect the
// limiters block by scanning for a plausible count (10..30) followed by
// a sequence of (type, key, values...) tuples where type is in [0,4] and
// key is in [0,17].
// ---------------------------------------------------------------------------
static void parseLimiters(TokenStream& ts, AircraftConfig& cfg,
                          std::vector<std::string>& warnings) {
    // Scan the ENTIRE token stream from the beginning looking for a valid
    // limiters block.
    ts.setPos(0);

    // Scan forward looking for a plausible limiters block.
    while (!ts.eof()) {
        std::size_t savedPos = ts.pos();
        try {
            int numLimiters = ts.nextInt();
            if (numLimiters < 10 || numLimiters > 30) {
                ts.setPos(savedPos + 1);
                continue;
            }

            // Try to parse numLimiters limiters
            // Save position so we can rollback if parsing fails
            bool ok = true;
            Limiter tempLimiters[static_cast<int>(LimiterKey::Count)];

            for (int i = 0; i < numLimiters && ok; ++i) {
                int type = ts.nextInt();
                int key  = ts.nextInt();
                if (type < 0 || type > 4) { ok = false; break; }
                if (key  < 0 || key  >= static_cast<int>(LimiterKey::Count)) { ok = false; break; }

                Limiter& lim = tempLimiters[key];
                switch (type) {
                    case 0: // Line: x1 y1 x2 y2
                        lim.type = LimiterType::Line;
                        lim.x1 = ts.nextDouble(); lim.y1 = ts.nextDouble();
                        lim.x2 = ts.nextDouble(); lim.y2 = ts.nextDouble();
                        break;
                    case 1: // Value
                        lim.type = LimiterType::Value;
                        lim.x1 = ts.nextDouble();
                        break;
                    case 2: // Percent
                        lim.type = LimiterType::Percent;
                        lim.x1 = ts.nextDouble();
                        break;
                    case 3: // ThreePoint: x0 y0 x1 y1 x2 y2
                        lim.type = LimiterType::ThreePoint;
                        lim.x0 = ts.nextDouble(); lim.y0 = ts.nextDouble();
                        lim.x1 = ts.nextDouble(); lim.y1 = ts.nextDouble();
                        lim.x2 = ts.nextDouble(); lim.y2 = ts.nextDouble();
                        break;
                    case 4: // MinMax: min max
                        lim.type = LimiterType::MinMax;
                        lim.x1 = ts.nextDouble();
                        lim.x2 = ts.nextDouble();
                        break;
                }
            }

            if (!ok) {
                ts.setPos(savedPos + 1);
                continue;
            }

            // Accept!
            for (int i = 0; i < static_cast<int>(LimiterKey::Count); ++i) {
                cfg.limiters[i] = tempLimiters[i];
            }
            return;
        } catch (const std::exception&) {
            ts.setPos(savedPos + 1);
            continue;
        }
    }
    warnings.push_back("Limiters: could not locate a valid limiters block");
}

// ---------------------------------------------------------------------------
// Parse the AuxAeroData block (key/value pairs).
// Direct port of the AuxAeroDataDesc table in readin.cpp.
//
// The block runs from "ADDITIONAL DATA" to the end of the file. We scan
// for known keys and apply them; unknown keys are silently skipped.
// ---------------------------------------------------------------------------
static void parseAuxAero(TokenStream& ts, AircraftConfig& cfg,
                         std::vector<std::string>& warnings) {
    // Scan the entire token stream from the beginning for known keys.
    // Unknown keys are silently skipped (along with their values, which we
    // can't always identify as numeric vs string).

    ts.setPos(0);

    auto& aux = cfg.aux;

    // Helper: try to read a double; if it fails, return the default and
    // DON'T advance the stream (so the next iteration can try the token as
    // a key).
    auto tryReadDouble = [&](double& out) -> bool {
        std::size_t saved = ts.pos();
        try {
            out = ts.nextDouble();
            return true;
        } catch (const std::exception&) {
            ts.setPos(saved);
            return false;
        }
    };
    auto tryReadInt = [&](int& out) -> bool {
        std::size_t saved = ts.pos();
        try {
            out = ts.nextInt();
            return true;
        } catch (const std::exception&) {
            ts.setPos(saved);
            return false;
        }
    };

    while (!ts.eof()) {
        std::string tok = ts.next();

        // Skip "aeropt" / "engopt" lines (option name follows)
        if (tok == "aeropt" || tok == "engopt") {
            if (!ts.eof()) ts.next(); // option name
            continue;
        }

        double dv;
        int iv;

        if (tok == "typeEngine") { if (tryReadInt(iv)) aux.typeEngine = iv; }
        else if (tok == "nEngines") { if (tryReadInt(iv)) aux.nEngines = iv; }
        else if (tok == "normSpoolRate") { if (tryReadDouble(dv)) aux.normSpoolRate = dv; }
        else if (tok == "abSpoolRate") { if (tryReadDouble(dv)) aux.abSpoolRate = dv; }
        else if (tok == "jfsSpoolUpRate") { if (tryReadDouble(dv)) aux.jfsSpoolUpRate = dv; }
        else if (tok == "jfsSpoolUpLimit") { if (tryReadDouble(dv)) aux.jfsSpoolUpLimit = dv; }
        else if (tok == "lightupSpoolRate") { if (tryReadDouble(dv)) aux.lightupSpoolRate = dv; }
        else if (tok == "flameoutSpoolRate") { if (tryReadDouble(dv)) aux.flameoutSpoolRate = dv; }
        else if (tok == "mainGenRpm") { if (tryReadDouble(dv)) aux.mainGenRpm = dv; }
        else if (tok == "stbyGenRpm") { if (tryReadDouble(dv)) aux.stbyGenRpm = dv; }
        else if (tok == "epuBurnTime") { if (tryReadDouble(dv)) aux.epuBurnTime = dv; }
        else if (tok == "fuelFlowFactorNormal") { if (tryReadDouble(dv)) aux.fuelFlowFactorNormal = dv; }
        else if (tok == "fuelFlowFactorAb") { if (tryReadDouble(dv)) aux.fuelFlowFactorAb = dv; }
        else if (tok == "minFuelFlow") { if (tryReadDouble(dv)) aux.minFuelFlow = dv; }
        else if (tok == "haslef")  { if (tryReadInt(iv)) aux.hasLef = (iv != 0); }
        else if (tok == "hasTef")  { if (tryReadInt(iv)) aux.hasTef = (iv != 0); }
        else if (tok == "lefGround") { if (tryReadDouble(dv)) aux.lefGround = dv; }
        else if (tok == "lefMaxAngle") { if (tryReadDouble(dv)) aux.lefMaxAngle = dv; }
        else if (tok == "lefMaxMach") { if (tryReadDouble(dv)) aux.lefMaxMach = dv; }
        else if (tok == "lefRate") { if (tryReadDouble(dv)) aux.lefRate = dv; }
        else if (tok == "tefMaxAngle") { if (tryReadDouble(dv)) aux.tefMaxAngle = dv; }
        else if (tok == "tefTakeoff") { if (tryReadDouble(dv)) aux.tefTakeOff = dv; }
        else if (tok == "tefRate") { if (tryReadDouble(dv)) aux.tefRate = dv; }
        else if (tok == "CLtefFactor") { if (tryReadDouble(dv)) aux.CLtefFactor = dv; }
        else if (tok == "CDtefFactor") { if (tryReadDouble(dv)) aux.CDtefFactor = dv; }
        else if (tok == "CDlefFactor") { if (tryReadDouble(dv)) aux.CDlefFactor = dv; }
        else if (tok == "CDSPDBFactor") { if (tryReadDouble(dv)) aux.CDSPDBFactor = dv; }
        else if (tok == "CDLDGFactor") { if (tryReadDouble(dv)) aux.CDLDGFactor = dv; }
        else if (tok == "dragChuteCd") { if (tryReadDouble(dv)) aux.dragChuteCd = dv; }
        else if (tok == "area2Span") { if (tryReadDouble(dv)) aux.area2Span = dv; }
        else if (tok == "pitchMomentum") { if (tryReadDouble(dv)) aux.pitchMomentum = dv; }
        else if (tok == "rollMomentum") { if (tryReadDouble(dv)) aux.rollMomentum = dv; }
        else if (tok == "yawMomentum") { if (tryReadDouble(dv)) aux.yawMomentum = dv; }
        else if (tok == "pitchElasticity") { if (tryReadDouble(dv)) aux.pitchElasticity = dv; }
        else if (tok == "gearPitchFactor") { if (tryReadDouble(dv)) aux.gearPitchFactor = dv; }
        else if (tok == "pitchGearGain") { if (tryReadDouble(dv)) aux.pitchGearGain = dv; }
        else if (tok == "rollGearGain") { if (tryReadDouble(dv)) aux.rollGearGain = dv; }
        else if (tok == "yawGearGain") { if (tryReadDouble(dv)) aux.yawGearGain = dv; }
        else if (tok == "rudderMaxAngle") { if (tryReadDouble(dv)) aux.rudderMaxAngle = dv; }
        else if (tok == "aileronMaxAngle") { if (tryReadDouble(dv)) aux.aileronMaxAngle = dv; }
        else if (tok == "airbrakeMaxAngle") { if (tryReadDouble(dv)) aux.airbrakeMaxAngle = dv; }
        else if (tok == "rollCouple") { if (tryReadDouble(dv)) aux.rollCouple = dv; }
        else if (tok == "elevatorRoll" || tok == "elevatorRolls") { if (tryReadInt(iv)) aux.elevatorRolls = (iv != 0); }
        else if (tok == "sinkRate") { if (tryReadDouble(dv)) aux.sinkRate = dv; }
        else if (tok == "landingAOA") { if (tryReadDouble(dv)) aux.landingAOA = dv; }
        // Unknown keys: skip silently. The token's value (if any) will be
        // read on the next iteration as a potential key, and skipped if it
        // doesn't match. This is robust against string-valued keys.
    }

    (void)warnings;
}

// ---------------------------------------------------------------------------
// Extract the aircraft name from the title comment.
// We need to do this BEFORE comment stripping, so it's a separate pass.
// ---------------------------------------------------------------------------
static std::string extractTitle(const std::string& contents) {
    std::istringstream iss(contents);
    std::string line;
    while (std::getline(iss, line)) {
        // Strip leading whitespace
        std::size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        if (line.compare(start, 8, "# Title:") == 0) {
            std::string title = line.substr(start + 8);
            // Trim
            std::size_t s = title.find_first_not_of(" \t");
            std::size_t e = title.find_last_not_of(" \t\r\n");
            if (s == std::string::npos) return "";
            return title.substr(s, e - s + 1);
        }
    }
    return "";
}

// ---------------------------------------------------------------------------
// Top-level parser
// ---------------------------------------------------------------------------
ParseResult loadString(const std::string& contents, const std::string& sourceName) {
    ParseResult result;
    AircraftConfig& cfg = result.config;

    cfg.name = extractTitle(contents);
    if (cfg.name.empty()) cfg.name = sourceName;

    TokenStream ts(contents, sourceName);

    try {
        // 1. Input data (positional, at top of file)
        parseInputData(ts, cfg, result.warnings);

        // 2. Aero tables (CL, CD, CY).
        while (ts.peek() == "aeropt") {
            ts.next(); ts.next();
        }
        try {
            parseAeroTable(ts, cfg.aero, "CL", result.warnings);
            parseAeroTable(ts, cfg.aero, "CD", result.warnings);
            parseAeroTable(ts, cfg.aero, "CY", result.warnings);
        } catch (const std::exception& e) {
            result.warnings.push_back(std::string("Aero tables: ") + e.what());
        }

        // 3. Engine
        try {
            parseEngine(ts, cfg, result.warnings);
        } catch (const std::exception& e) {
            result.warnings.push_back(std::string("Engine: ") + e.what());
        }

        // 4. Roll data
        try {
            parseRollData(ts, cfg, result.warnings);
        } catch (const std::exception& e) {
            result.warnings.push_back(std::string("RollData: ") + e.what());
        }

        // 5. Limiters
        try {
            parseLimiters(ts, cfg, result.warnings);
        } catch (const std::exception& e) {
            result.warnings.push_back(std::string("Limiters: ") + e.what());
        }

        // 6. AuxAeroData (key/value pairs -- scan entire stream from start)
        try {
            parseAuxAero(ts, cfg, result.warnings);
        } catch (const std::exception& e) {
            result.warnings.push_back(std::string("AuxAero: ") + e.what());
        }

        result.ok = true;
    } catch (const std::exception& e) {
        result.errors.push_back(e.what());
        result.ok = false;
    }

    return result;
}

ParseResult loadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        ParseResult r;
        r.errors.push_back("Could not open file: " + path);
        r.ok = false;
        return r;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return loadString(ss.str(), path);
}

} // namespace f4flight::dat
