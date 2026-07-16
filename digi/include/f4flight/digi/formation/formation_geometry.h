// f4flight - digi/formation/formation_geometry.h
//
// Formation geometry primitives for the wingman / flight-lead system.
//
// Port of FreeFalcon's ACFormationData (formdata.cpp:114 LOC) and the
// PositionData struct used by DigitalBrain (digimain.cpp:41-62).
//
// FreeFalcon loads formation definitions from `formdat.fil` — a flat text
// file mapping (formationType × slotIndex) → (relAz, relEl, range). F4Flight
// doesn't ship a file loader (host supplies formation definitions via
// `FormationTable::registerFormation`), but the data structures are
// identical so a future file loader can be added without API churn.
//
// Round-2 structural fix (DIGI_AUDIT.md Rec 3 / Gap 1): this header is
// the prerequisite for any wingman-mode work. The DigiState already has
// `flightLeadId` / `isWing` / `vehicleInUnit` / `formationId` fields;
// this header defines the geometry lookup that the future `AiFollowLead`
// maneuver will consume.

#pragma once

#include <cmath>
#include <unordered_map>
#include <array>
#include <cstdint>

// M_PI is not guaranteed to be defined in <cmath> (C++11), so define it if missing.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace f4flight {
namespace digi {
namespace formation {

// Formation types — matches FreeFalcon's formation message IDs (fvorenum.h).
// Used as an index into FormationTable.
enum class FormationType : int {
    Wedge        = 0,   // 4-ship wedge (default fighter formation)
    Echelon      = 1,   // 4-ship echelon (parade formation)
    Fingertip    = 2,   // 4-ship fingertip (tight maneuvering)
    Trail        = 3,   // 4-ship trail (instrument conditions)
    Stack        = 4,   // 4-ship stack (vertical separation)
    Spread       = 5,   // 4-ship spread (combat spread)
    TwoShipLineAbreast = 6,  // 2-ship line-abreast
    TwoShipTrail = 7,         // 2-ship trail
    Custom       = 99,        // host-defined geometry
};

// PositionData — relative position of one slot in a formation.
// Direct port of FreeFalcon's PositionData struct (formdata.cpp).
//
// All angles in RADIANS, range in FEET. The geometry is relative to the
// flight lead's velocity vector:
//   relAz   : bearing CLOCKWISE from lead's nose (positive = right)
//             0     = directly ahead of lead's nose (forward)
//             +pi/2 = directly to the right of lead
//             +pi   = directly behind lead (trail)
//             -pi/2 = directly to the left of lead
//   relEl   : elevation from lead's plane (positive = above)
//   range   : straight-line distance from lead
//
// COORDINATE SYSTEM NOTE: F4Flight uses (x=east, y=north, z=down) with
// heading sigma measured CCW from +x (standard math). FreeFalcon uses
// (x=north, y=east, z=down) with sigma measured CW from +x (navigation).
// FreeFalcon's formula `cos/sin(relAz + sigma)` assumes its own convention;
// in F4Flight's convention that formula places relAz=0 FORWARD and makes
// positive relAz = LEFT (CCW), which is the opposite of the intended
// "positive = right". The wingman code therefore uses the ADAPTED formula
//   trackX = leadX + range * cos(sigma - relAz*formSide)
//   trackY = leadY + range * sin(sigma - relAz*formSide)
// which reproduces FreeFalcon's intended geometry (positive relAz = right)
// in F4Flight's coordinate system. See wingman_ai.cpp for details.
struct PositionData {
    double relAz{0.0};    // radians, CW from lead's nose (positive = right)
    double relEl{0.0};    // radians, positive = above lead's plane
    double range{1000.0}; // feet
};

// Maximum slots in any formation (4-ship is the largest standard formation).
constexpr int kMaxFormationSlots = 4;

// A Formation is an array of PositionData, one per slot. Slot 0 is always
// the lead (relAz=0, relEl=0, range=0).
using Formation = std::array<PositionData, kMaxFormationSlots>;

// Default wedge formation — 4-ship fighter wedge.
//   slot 0: lead (at center)
//   slot 1: right wing — 45° aft of line abreast, 1000 ft
//   slot 2: left wing  — 45° aft of line abreast, 1000 ft
//   slot 3: trail      — directly behind lead, 2000 ft
// relAz uses the CW-from-nose convention: +135° = behind-right, -135° = behind-left,
// +180° = directly behind. This matches the standard fighter wedge where
// wingmen are BEHIND and to the sides of the lead (not in front).
inline Formation defaultWedge() {
    Formation f{};
    f[0] = {0.0,                       0.0, 0.0};      // lead
    f[1] = {135.0 * M_PI / 180.0,      0.0, 1000.0};   // right wing (behind-right)
    f[2] = {-135.0 * M_PI / 180.0,     0.0, 1000.0};   // left wing  (behind-left)
    f[3] = {180.0 * M_PI / 180.0,      0.0, 2000.0};   // trail (directly behind)
    return f;
}

// Default 2-ship trail — wingman 1000 ft directly behind lead.
inline Formation defaultTwoShipTrail() {
    Formation f{};
    f[0] = {0.0, 0.0, 0.0};
    f[1] = {180.0 * M_PI / 180.0, 0.0, 1000.0};  // directly behind
    return f;
}

// Default 2-ship line-abreast — wingman 1000 ft to the right of lead.
inline Formation defaultTwoShipLineAbreast() {
    Formation f{};
    f[0] = {0.0, 0.0, 0.0};
    f[1] = {90.0 * M_PI / 180.0, 0.0, 1000.0};  // directly to the right
    return f;
}

// FormationTable — registry of formation definitions by type.
// Hosts register custom formations; the AI looks up the geometry by type
// when computing the wingman's desired position.
//
// DESIGN NOTE: this was previously a Meyer's singleton (static instance()).
// That prevented testing (couldn't reset between tests), prevented multiple
// tables (e.g. NATO vs Warsaw Pact formations), and prevented dependency
// injection. Now it's a regular class — hosts construct one and pass it
// where needed. For backward compatibility, a default instance is still
// available via defaultInstance() (NOT a singleton — just a shared default
// for hosts that don't want to manage their own).
class FormationTable {
public:
    FormationTable() {
        // Register the standard formations on construction.
        formations_[static_cast<int>(FormationType::Wedge)] = defaultWedge();
        formations_[static_cast<int>(FormationType::TwoShipTrail)] = defaultTwoShipTrail();
        formations_[static_cast<int>(FormationType::TwoShipLineAbreast)] = defaultTwoShipLineAbreast();
    }

    void registerFormation(FormationType type, const Formation& f) {
        formations_[static_cast<int>(type)] = f;
    }

    // Get the PositionData for a specific slot in a formation.
    // Returns a zero-relative PositionData (lead slot — relAz=0, relEl=0,
    // range=0) if the type or slot is invalid — this fails safe (the
    // wingman will try to fly to the lead's position).
    PositionData slotGeometry(FormationType type, int slot) const {
        if (slot < 0 || slot >= kMaxFormationSlots) {
            return PositionData{0.0, 0.0, 0.0};  // explicit zeros (lead slot)
        }
        auto it = formations_.find(static_cast<int>(type));
        if (it == formations_.end()) {
            return PositionData{0.0, 0.0, 0.0};
        }
        return it->second[slot];
    }

    // Convenience: get the relative position for the wingman's current
    // state. Reads formationId + slot from the digi state.
    static PositionData forWingman(int formationId, int slot) {
        return defaultInstance().slotGeometry(static_cast<FormationType>(formationId), slot);
    }

    // --- FreeFalcon formdat.fil loader ---
    //
    // FreeFalcon loads formation definitions from `formdat.fil` — a flat
    // text file. F4Flight doesn't ship the file, but hosts can supply one
    // (e.g. extracted from a Falcon 4 installation) and call loadFromFile()
    // to register all formations at once. This allows hosts to customize
    // formations without recompiling.
    //
    // File format (whitespace-separated tokens):
    //   numFormations
    //   num4Slots num2Slots formNum formationName
    //     relAz_deg relEl_deg range_NM    (repeated num4Slots times)
    //     relAz_deg relEl_deg range_NM    (repeated num2Slots times, if > 0)
    //   ... (repeated for each formation)
    //
    // relAz is in DEGREES (converted to radians internally).
    // relEl is in DEGREES (converted to radians internally).
    // range is in NAUTICAL MILES (converted to feet internally).
    //
    // The formNum is FreeFalcon's FalconWingmanMsg enum value. It's stored
    // as-is as the integer key — the host maps it to F4Flight's FormationType
    // via registerFormation(FormationType, ...) or looks it up directly via
    // slotGeometryById(formNum, slot).
    //
    // Returns the number of formations loaded, or -1 on error.
    int loadFromFile(const char* filename);

    // Look up a slot by raw integer key (e.g. FreeFalcon formNum).
    PositionData slotGeometryById(int formId, int slot) const {
        if (slot < 0 || slot >= kMaxFormationSlots) {
            return PositionData{0.0, 0.0, 0.0};
        }
        auto it = formations_.find(formId);
        if (it == formations_.end()) {
            return PositionData{0.0, 0.0, 0.0};
        }
        return it->second[slot];
    }

    // Register a formation by raw integer key (for FreeFalcon formNum).
    void registerFormationById(int formId, const Formation& f) {
        formations_[formId] = f;
    }

    // Shared default instance for hosts that don't want to manage their own
    // FormationTable. NOT a singleton — hosts are free to construct their
    // own and pass them explicitly (preferred for testing/multi-table use).
    static FormationTable& defaultInstance() {
        static FormationTable t;
        return t;
    }

private:
    std::unordered_map<int, Formation> formations_;
};

} // namespace formation
} // namespace digi
} // namespace f4flight
