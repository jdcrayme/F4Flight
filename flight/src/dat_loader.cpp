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

#include "f4flight/flight/dat_loader.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

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
            cfg.engineOptions.push_back(optName);
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
    // Try to find a valid roll table. We try TWO passes:
    //
    //   Pass 1: scan forward FROM THE CURRENT STREAM POSITION (which should
    //           be right after the engine section, per the FreeFalcon readin
    //           order: ReadData -> AirframeAeroRead -> AirframeEngineRead ->
    //           AirframeFcsRead). This is the correct location of the roll
    //           table, so a match here is authoritative.
    //
    //   Pass 2: only if Pass 1 fails, scan from the BEGINNING of the token
    //           stream. This is a fallback for files where the engine parser
    //           overran or left the stream in an unexpected position.
    //
    // The previous implementation scanned only from position 0, which caused
    // false positives: the alpha-breakpoints section of the aero table
    // (sequences like "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14.5 15 15.5 16
    // 17 18 19 20 21 22 23 24 25 26 27 28 29 30 ...") looks exactly like a
    // valid roll table signature to the heuristic, and the parser would
    // accept "2 3 4 / 5 6 7 8 9 / 10 / 11 12 13 14 14.5 15 15.5 16 17 18
    // 19" as a 2x5 roll table instead of finding the real 7x7 roll table
    // later in the file.

    auto tryParseRollAt = [&](std::size_t startPos) -> bool {
        ts.setPos(startPos);
        while (!ts.eof()) {
            std::size_t savedPos = ts.pos();

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
                return true;
            } catch (const std::exception&) {
                ts.setPos(savedPos + 1);
                continue;
            }
        }
        return false;
    };

    // Pass 1: from current position (after engine section).
    if (tryParseRollAt(ts.pos())) return;

    // Pass 2: fallback -- scan from the beginning.
    if (tryParseRollAt(0)) return;

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
// Direct port of the AuxAeroDataDesc table in readin.cpp, but with a
// critical difference: we capture EVERY key/value pair verbatim into
// `cfg.rawAuxAeroData`, not just the keys we know about. This guarantees
// no data is lost in the .dat -> JSON conversion.
//
// The block is line-based: each line is `<key> <value...>` followed by an
// optional `# comment`. We capture the value as everything after the key
// (with leading whitespace trimmed) up to the end of the line. The trailing
// `# comment` (if any) is preserved verbatim as part of the value string
// (matching what FreeFalcon's ReadLine() would see).
//
// Known keys are also dispatched to populate the typed `aux` struct for
// backward compatibility with existing f4flight code that reads
// `cfg.aux.normSpoolRate` etc.
// ---------------------------------------------------------------------------
static void parseAuxAero(const std::string& contents,
                         AircraftConfig& cfg,
                         std::vector<std::string>& warnings) {
    auto& aux = cfg.aux;

    // Helper: parse a single double from a string starting at pos; advance pos.
    auto parseDoubleAt = [](const std::string& s, std::size_t& pos, double& out) -> bool {
        while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos >= s.size()) return false;
        char* end = nullptr;
        out = std::strtod(s.c_str() + pos, &end);
        if (end == s.c_str() + pos) return false;
        pos = static_cast<std::size_t>(end - s.c_str());
        return true;
    };
    auto parseIntAt = [](const std::string& s, std::size_t& pos, int& out) -> bool {
        while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos >= s.size()) return false;
        char* end = nullptr;
        long v = std::strtol(s.c_str() + pos, &end, 10);
        if (end == s.c_str() + pos) return false;
        out = static_cast<int>(v);
        pos = static_cast<std::size_t>(end - s.c_str());
        return true;
    };

    std::istringstream iss(contents);
    std::string line;
    while (std::getline(iss, line)) {
        // Normalize line endings: strip any trailing \r (Windows CRLF).
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Find first non-whitespace (space, tab, \r, \n, \v, \f).
        std::size_t start = line.find_first_not_of(" \t\r\n\v\f");
        if (start == std::string::npos) continue;
        // Skip comment lines (we keep the raw line's content, but the key
        // extraction below wouldn't find a valid key on a comment line anyway).
        if (line[start] == '#') continue;

        // Find end of key (next whitespace)
        std::size_t keyEnd = line.find_first_of(" \t\r\n\v\f", start);
        std::string key;
        std::string value;
        if (keyEnd == std::string::npos) {
            key = line.substr(start);
            value = "";  // key with no value
        } else {
            key = line.substr(start, keyEnd - start);
            // Value is everything after the key, with leading whitespace
            // trimmed. Trailing whitespace/newline is also trimmed. Embedded
            // `# comment` is preserved (the .dat format uses `#` only at the
            // start of a line for full-line comments, so an inline `#` after
            // a value is treated as part of the value by the legacy parser
            // when it does ReadLine(); we preserve the same behavior).
            std::size_t valStart = line.find_first_not_of(" \t\r\n\v\f", keyEnd);
            if (valStart == std::string::npos) {
                value = "";
            } else {
                std::size_t valEnd = line.find_last_not_of(" \t\r\n\v\f");
                value = line.substr(valStart, valEnd - valStart + 1);
            }
        }

        // Skip option flags -- they're handled by parseEngine / parseAeroTable
        // via the TokenStream, and we capture them separately into
        // aeroOptions/engineOptions. Don't record them here.
        if (key == "aeropt" || key == "engopt") continue;

        // Skip section-end marker
        if (key == "END" ) continue;

        // Skip keys that are actually positional-table tokens that happen to
        // start with a letter. The AuxAeroData section uses key/value form
        // exclusively, but the earlier positional sections (gear coords,
        // aero/engine/roll tables, limiters) are pure numbers. If a "key"
        // parses as a number, it's not an AuxAeroData key -- skip it.
        {
            char* end = nullptr;
            std::strtod(key.c_str(), &end);
            if (end != key.c_str() && *end == '\0') continue;  // numeric key
        }

        // Capture into the raw map (authoritative record). If a key appears
        // more than once (some .dat files repeat keys), the last occurrence
        // wins, matching the legacy ParseSimlibFile behavior.
        cfg.rawAuxAeroData[key] = value;

        // Dispatch known keys to the typed `aux` struct for backward compat.
        // We parse from the captured value string.
        std::size_t pos = 0;
        double dv = 0.0;
        int iv = 0;

        auto tryD = [&](double& dst) {
            if (parseDoubleAt(value, pos, dv)) dst = dv;
        };
        auto tryI = [&](int& dst) {
            if (parseIntAt(value, pos, iv)) dst = iv;
        };

        if      (key == "typeEngine")   tryI(aux.typeEngine);
        else if (key == "nEngines")     tryI(aux.nEngines);
        else if (key == "normSpoolRate")        tryD(aux.normSpoolRate);
        else if (key == "abSpoolRate")          tryD(aux.abSpoolRate);
        else if (key == "jfsSpoolUpRate")       tryD(aux.jfsSpoolUpRate);
        else if (key == "jfsSpoolUpLimit")      tryD(aux.jfsSpoolUpLimit);
        else if (key == "lightupSpoolRate")     tryD(aux.lightupSpoolRate);
        else if (key == "flameoutSpoolRate")    tryD(aux.flameoutSpoolRate);
        else if (key == "mainGenRpm")           tryD(aux.mainGenRpm);
        else if (key == "stbyGenRpm")           tryD(aux.stbyGenRpm);
        else if (key == "epuBurnTime")          tryD(aux.epuBurnTime);
        else if (key == "fuelFlowFactorNormal") tryD(aux.fuelFlowFactorNormal);
        else if (key == "fuelFlowFactorAb")     tryD(aux.fuelFlowFactorAb);
        else if (key == "minFuelFlow")          tryD(aux.minFuelFlow);
        else if (key == "haslef" || key == "hasLef") {
            if (parseIntAt(value, pos, iv)) aux.hasLef = (iv != 0);
        }
        else if (key == "hasTef") {
            if (parseIntAt(value, pos, iv)) aux.hasTef = (iv != 0);
        }
        else if (key == "lefGround")    tryD(aux.lefGround);
        else if (key == "lefMaxAngle")  tryD(aux.lefMaxAngle);
        else if (key == "lefMaxMach")   tryD(aux.lefMaxMach);
        else if (key == "lefRate")      tryD(aux.lefRate);
        else if (key == "tefMaxAngle")  tryD(aux.tefMaxAngle);
        else if (key == "tefTakeoff" || key == "tefTakeOff") tryD(aux.tefTakeOff);
        else if (key == "tefRate")      tryD(aux.tefRate);
        else if (key == "CLtefFactor")  tryD(aux.CLtefFactor);
        else if (key == "CDtefFactor")  tryD(aux.CDtefFactor);
        else if (key == "CDlefFactor")  tryD(aux.CDlefFactor);
        else if (key == "CDSPDBFactor") tryD(aux.CDSPDBFactor);
        else if (key == "CDLDGFactor")  tryD(aux.CDLDGFactor);
        else if (key == "dragChuteCd")  tryD(aux.dragChuteCd);
        else if (key == "area2Span")    tryD(aux.area2Span);
        else if (key == "pitchMomentum")   tryD(aux.pitchMomentum);
        else if (key == "rollMomentum")    tryD(aux.rollMomentum);
        else if (key == "yawMomentum")     tryD(aux.yawMomentum);
        else if (key == "pitchElasticity") tryD(aux.pitchElasticity);
        else if (key == "gearPitchFactor") tryD(aux.gearPitchFactor);
        else if (key == "pitchGearGain")   tryD(aux.pitchGearGain);
        else if (key == "rollGearGain")    tryD(aux.rollGearGain);
        else if (key == "yawGearGain")     tryD(aux.yawGearGain);
        else if (key == "rudderMaxAngle")  tryD(aux.rudderMaxAngle);
        else if (key == "aileronMaxAngle") tryD(aux.aileronMaxAngle);
        else if (key == "airbrakeMaxAngle") tryD(aux.airbrakeMaxAngle);
        else if (key == "rollCouple")      tryD(aux.rollCouple);
        else if (key == "elevatorRoll" || key == "elevatorRolls") {
            if (parseIntAt(value, pos, iv)) aux.elevatorRolls = (iv != 0);
        }
        else if (key == "sinkRate")    tryD(aux.sinkRate);
        else if (key == "landingAOA")  tryD(aux.landingAOA);
        else if (key == "criticalAOA") tryD(aux.criticalAOA);
        // Unknown keys: already captured in rawAuxAeroData, nothing else to do.
    }

    (void)warnings;
}

// ---------------------------------------------------------------------------
// Extract source metadata (Title/Author/Revision) from the header comments.
// We need to do this BEFORE comment stripping, so it's a separate pass.
// ---------------------------------------------------------------------------
static void extractSourceMetadata(const std::string& contents,
                                  const std::string& sourceName,
                                  AircraftConfig& cfg) {
    cfg.sourceFile = sourceName;
    // Strip directory portion if present
    {
        std::size_t slash = cfg.sourceFile.find_last_of("/\\");
        if (slash != std::string::npos) cfg.sourceFile = cfg.sourceFile.substr(slash + 1);
    }

    std::istringstream iss(contents);
    std::string line;
    while (std::getline(iss, line)) {
        // Normalize line endings: strip any trailing \r (Windows CRLF).
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::size_t start = line.find_first_not_of(" \t\r\n\v\f");
        if (start == std::string::npos) continue;
        if (line.compare(start, 8, "# Title:") == 0) {
            std::string v = line.substr(start + 8);
            std::size_t s = v.find_first_not_of(" \t");
            std::size_t e = v.find_last_not_of(" \t\r\n\v\f");
            if (s != std::string::npos) cfg.sourceTitle = v.substr(s, e - s + 1);
        }
        else if (line.compare(start, 9, "# Author:") == 0) {
            std::string v = line.substr(start + 9);
            std::size_t s = v.find_first_not_of(" \t");
            std::size_t e = v.find_last_not_of(" \t\r\n\v\f");
            if (s != std::string::npos) cfg.sourceAuthor = v.substr(s, e - s + 1);
        }
        else if (line.compare(start, 11, "# Revision:") == 0) {
            std::string v = line.substr(start + 11);
            std::size_t s = v.find_first_not_of(" \t");
            std::size_t e = v.find_last_not_of(" \t\r\n\v\f");
            if (s != std::string::npos) cfg.sourceRevision = v.substr(s, e - s + 1);
        }
    }
}

// ---------------------------------------------------------------------------
// Detect BMS "Advanced Flight Model" (.dat files whose name ends in `_afm`).
//
// AFM files use a completely different format from the standard Falcon 4 .dat
// file: they start with a small integer (the FM type, 1-4) instead of the
// empty weight, and contain 6-DOF inertia tensors, fuel-tank CG positions,
// and per-control-surface derivative tables rather than the CL/CD/CY vs
// Mach/alpha tables the standard format uses.
//
// The f4flight library is a port of the *standard* FreeFalcon flight model
// and cannot consume AFM data. We detect AFM files up-front and return a
// clean error so the caller can skip them gracefully (e.g. the dat2json
// tool reports "AFM format -- skipped" instead of crashing with a confusing
// parse error deep in the engine-table reader).
//
// Detection heuristic: the file name ends in `_afm` OR the first non-comment
// numeric token is a small integer (1..4) AND there is no `aeropt` token
// anywhere in the file (standard .dat files always have aerodynamic tables
// preceded by `aeropt` or directly by the mach-breakpoints count).
// ---------------------------------------------------------------------------
static bool isAfmFile(const std::string& contents, const std::string& sourceName) {
    // Quick check: file name suffix.
    {
        std::string base = sourceName;
        std::size_t slash = base.find_last_of("/\\");
        if (slash != std::string::npos) base = base.substr(slash + 1);
        // Strip ".dat" suffix to check for "_afm"
        if (base.size() > 4 && base.compare(base.size() - 4, 4, ".dat") == 0) {
            base = base.substr(0, base.size() - 4);
        }
        if (base.size() >= 4 && base.compare(base.size() - 4, 4, "_afm") == 0) {
            return true;
        }
    }

    // Content check: scan for the first non-comment, non-whitespace token.
    // If it parses as a small integer (1..4), treat as AFM. Standard .dat
    // files start with a large floating-point empty weight (typically
    // > 1000 lbs).
    std::istringstream iss(contents);
    std::string line;
    while (std::getline(iss, line)) {
        std::size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        if (line[start] == '#') continue;
        // First non-comment token is at `start`. Extract it.
        std::size_t end = line.find_first_of(" \t", start);
        std::string tok = (end == std::string::npos)
                          ? line.substr(start)
                          : line.substr(start, end - start);
        // Try to parse as integer.
        try {
            int v = std::stoi(tok);
            if (v >= 1 && v <= 4) return true;
        } catch (const std::exception&) {
            // Not an integer -- not AFM by this check.
        }
        return false;  // first non-comment token wasn't a small int
    }
    return false;
}

// ---------------------------------------------------------------------------
// Top-level parser
// ---------------------------------------------------------------------------
ParseResult loadString(const std::string& contents, const std::string& sourceName) {
    ParseResult result;
    AircraftConfig& cfg = result.config;

    // AFM detection: fail cleanly with a recognizable error message so the
    // caller can skip these files.
    if (isAfmFile(contents, sourceName)) {
        result.errors.push_back(
            "AFM format not supported by f4flight (BMS Advanced Flight Model .dat files "
            "use a different schema with 6-DOF inertia tensors and per-surface derivative "
            "tables). Use the standard .dat file with the same base name instead.");
        result.ok = false;
        return result;
    }

    // Extract source metadata (Title/Author/Revision/file) before comment
    // stripping. Also sets cfg.name from the Title.
    extractSourceMetadata(contents, sourceName, cfg);
    cfg.name = cfg.sourceTitle;
    if (cfg.name.empty()) cfg.name = sourceName;

    // Capture aeropt options (line-based scan of the raw contents, before
    // TokenStream stripping -- we want the literal option names in file
    // order, including duplicates if any).
    {
        std::istringstream iss(contents);
        std::string line;
        while (std::getline(iss, line)) {
            // Normalize line endings: strip any trailing \r (Windows CRLF).
            if (!line.empty() && line.back() == '\r') line.pop_back();
            std::size_t start = line.find_first_not_of(" \t\r\n\v\f");
            if (start == std::string::npos) continue;
            if (line.compare(start, 6, "aeropt") != 0) continue;
            // Make sure it's a standalone token (next char is whitespace)
            if (start + 6 < line.size() && !std::isspace(static_cast<unsigned char>(line[start + 6]))) continue;
            std::size_t nameStart = line.find_first_not_of(" \t\r\n\v\f", start + 6);
            if (nameStart == std::string::npos) continue;
            std::size_t nameEnd = line.find_last_not_of(" \t\r\n\v\f");
            cfg.aeroOptions.push_back(line.substr(nameStart, nameEnd - nameStart + 1));
        }
    }

    TokenStream ts(contents, sourceName);

    try {
        // 1. Input data (positional, at top of file)
        parseInputData(ts, cfg, result.warnings);

        // 2. Aero tables (CL, CD, CY). Skip aeropt tokens (already captured
        // above into cfg.aeroOptions).
        while (ts.peek() == "aeropt") {
            ts.next(); // consume "aeropt"
            ts.next(); // consume option name
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

        // 6. AuxAeroData (key/value pairs -- scan entire raw contents, line
        // based, capturing EVERY key/value pair verbatim into rawAuxAeroData
        // for no-loss round-trip fidelity).
        try {
            parseAuxAero(contents, cfg, result.warnings);
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
