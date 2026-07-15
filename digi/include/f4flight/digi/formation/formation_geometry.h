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
//   relAz   : bearing from lead's nose (positive = right)
//   relEl   : elevation from lead's plane (positive = above)
//   range   : straight-line distance from lead
//
// The wingman's desired world position is computed as:
//   leadPos + leadDcm * (range * cos(relEl) * sin(relAz),
//                         range * cos(relEl) * cos(relAz),
//                        -range * sin(relEl))
struct PositionData {
    double relAz{0.0};    // radians, positive = right of lead's nose
    double relEl{0.0};    // radians, positive = above lead's plane
    double range{1000.0}; // feet
};

// Maximum slots in any formation (4-ship is the largest standard formation).
constexpr int kMaxFormationSlots = 4;

// A Formation is an array of PositionData, one per slot. Slot 0 is always
// the lead (relAz=0, relEl=0, range=0).
using Formation = std::array<PositionData, kMaxFormationSlots>;

// Default wedge formation — 4-ship, 1000 ft trail, 30° spread.
// Matches FreeFalcon's default wedge (formdat.fil first entry).
inline Formation defaultWedge() {
    Formation f{};
    f[0] = {0.0,                       0.0, 0.0};
    f[1] = {30.0 * M_PI / 180.0,       0.0, 1000.0};
    f[2] = {-30.0 * M_PI / 180.0,      0.0, 1000.0};
    f[3] = {0.0,                       0.0, 2000.0};
    return f;
}

// Default 2-ship trail — wingman 1000 ft behind lead.
inline Formation defaultTwoShipTrail() {
    Formation f{};
    f[0] = {0.0, 0.0, 0.0};
    f[1] = {0.0, 0.0, 1000.0};
    return f;
}

// Default 2-ship line-abreast — wingman 1000 ft to the right of lead.
inline Formation defaultTwoShipLineAbreast() {
    Formation f{};
    f[0] = {0.0, 0.0, 0.0};
    f[1] = {90.0 * M_PI / 180.0, 0.0, 1000.0};
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
