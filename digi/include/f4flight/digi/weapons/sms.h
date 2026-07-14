// f4flight - digi/weapons/sms.h
//
// StoresManagementSystem — minimal AI-facing SMS.
//
// FreeFalcon's SMSClass (sms.h) is a complex per-hardpoint weapon manager
// with weapon stepping, release sequencing, jettison, etc. The digi AI only
// needs to know:
//   - What weapons are on which hardpoints
//   - Which weapon is currently selected
//   - How many rounds/missiles remain
//
// This minimal SMS provides that interface. It does NOT simulate release
// sequencing or jettison — the host (or test harness) handles that by
// decrementing counts when the AI fires.
//
// Port concept from FreeFalcon src/sim/include/sms.h.

#pragma once

#include "f4flight/digi/weapons/weapon_types.h"
#include "f4flight/digi/weapons/weapon_spec.h"

#include <vector>
#include <cstdint>

namespace f4flight {
namespace digi {

// ===========================================================================
// Hardpoint — one weapon station on the aircraft.
// ===========================================================================
struct Hardpoint {
    int         station  {0};         // hardpoint number (1-based)
    WeaponType  type     {WeaponType::None};
    WeaponClass wclass   {WeaponClass::NoWpn};
    int         count    {0};         // missiles/bombs on this station, or
                                       // rounds for guns
    bool        trainable{false};     // turret gun (AC-130, etc.)
};

// ===========================================================================
// StoresManagementSystem — minimal AI-facing SMS.
//
// The host populates this at init (from the aircraft config) and updates
// counts when weapons are fired. The AI reads it via FireControl to decide
// what to fire and when.
// ===========================================================================
class StoresManagementSystem {
public:
    StoresManagementSystem() = default;

    // --- Setup (host calls at init) ---

    void addHardpoint(int station, WeaponType type, int count,
                      bool trainable = false) {
        Hardpoint hp;
        hp.station   = station;
        hp.type      = type;
        hp.wclass    = weaponClassOf(type);
        hp.count     = count;
        hp.trainable = trainable;
        hardpoints_.push_back(hp);
    }

    // --- Query (AI calls) ---

    int numHardpoints() const {
        return static_cast<int>(hardpoints_.size());
    }

    const std::vector<Hardpoint>& hardpoints() const {
        return hardpoints_;
    }

    const Hardpoint* hardpoint(int station) const {
        for (const auto& hp : hardpoints_) {
            if (hp.station == station) return &hp;
        }
        return nullptr;
    }

    // Does the aircraft have any weapon of this class?
    bool hasWeaponClass(WeaponClass wc) const {
        for (const auto& hp : hardpoints_) {
            if (hp.wclass == wc && hp.count > 0) return true;
        }
        return false;
    }

    // Does the aircraft have any weapon of this type?
    bool hasWeaponType(WeaponType wt) const {
        for (const auto& hp : hardpoints_) {
            if (hp.type == wt && hp.count > 0) return true;
        }
        return false;
    }

    // Total remaining rounds/missiles of a given type.
    int remainingOfType(WeaponType wt) const {
        int total = 0;
        for (const auto& hp : hardpoints_) {
            if (hp.type == wt) total += hp.count;
        }
        return total;
    }

    // Total remaining A/A weapons (missiles + gun rounds).
    int remainingAirToAir() const {
        int total = 0;
        for (const auto& hp : hardpoints_) {
            if (weaponDomainOf(hp.type) == WeaponDomain::Air && hp.count > 0) {
                total += hp.count;
            }
        }
        return total;
    }

    // Get the gun hardpoint (if any). Returns nullptr if no gun.
    const Hardpoint* gun() const {
        for (const auto& hp : hardpoints_) {
            if (hp.wclass == WeaponClass::GunWpn && hp.count > 0) return &hp;
        }
        return nullptr;
    }

    // --- Mutation (host calls when weapon fired) ---

    // Decrement the count on a hardpoint (weapon fired/released).
    // Returns true if a weapon was actually expended.
    bool expend(int station, int count = 1) {
        for (auto& hp : hardpoints_) {
            if (hp.station == station && hp.count > 0) {
                hp.count = std::max(0, hp.count - count);
                return true;
            }
        }
        return false;
    }

    // Decrement the first available hardpoint of a given type.
    // Returns true if a weapon was actually expended.
    bool expendOfType(WeaponType wt, int count = 1) {
        for (auto& hp : hardpoints_) {
            if (hp.type == wt && hp.count > 0) {
                hp.count = std::max(0, hp.count - count);
                return true;
            }
        }
        return false;
    }

    // --- Currently selected weapon (AI sets this) ---

    WeaponType currentWeapon() const { return currentWeapon_; }
    int currentStation() const { return currentStation_; }

    void selectWeapon(WeaponType wt, int station) {
        currentWeapon_ = wt;
        currentStation_ = station;
    }

private:
    std::vector<Hardpoint> hardpoints_;
    WeaponType currentWeapon_ {WeaponType::None};
    int        currentStation_{0};
};

} // namespace digi
} // namespace f4flight
