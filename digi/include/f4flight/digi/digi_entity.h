// f4flight - digi/digi_entity.h
//
// DigiEntity — minimal entity representation for the digi AI.
//
// FreeFalcon's digi AI operates on SimBaseClass / MissileClass / AircraftClass
// entities with a complex reference-counting + campaign object model. For the
// F4Flight port, we use a plain struct that holds only the fields the digi
// AI actually reads. The host program populates these from its own entity
// model each frame.
//
// This struct is used for:
//   - Incoming missiles (MissileDefeat)
//   - Guns threats (GunsJink)
//   - Future: targets (WVREngage, BVR), wingmen (Formation), etc.
//
// All positions/velocities are in world frame (NED, Z-down, ft and ft/s).
// Attitude angles (yaw, pitch, roll) are in radians.

#pragma once

#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/types.h"  // for Matrix3
#include <cmath>

namespace f4flight {
namespace digi {

struct DigiEntity {
    // --- World position (ft, NED, Z-down) ---
    double x{0.0}, y{0.0}, z{0.0};

    // --- World velocity (ft/s) ---
    double vx{0.0}, vy{0.0}, vz{0.0};

    // --- Attitude (radians) ---
    double yaw{0.0};     // heading (0 = North)
    double pitch{0.0};   // pitch
    double roll{0.0};    // roll

    // --- Body-to-world DCM (3x3) ---
    // Computed from yaw/pitch/roll. Used by body-frame relative geometry
    // (e.g. collision_avoid, AutoTrack target-body-frame computations).
    // For self, the brain populates this from AircraftState.kin.dcm.
    // For injected/auto-tracked entities, the host or toDigiEntity() helper
    // populates it. If left as identity (default), only 2D yaw-based
    // geometry works — pitch/roll effects on body-frame transforms are lost.
    Matrix3 dcm;

    // --- Speed (ft/s) — convenience field for closure calculations ---
    double speed{0.0};

    // --- Entity type / state ---
    enum class SeekerType {
        None,         // not a missile (aircraft, etc.)
        Radar,        // radar-guided missile (SARH / ARH)
        IR            // infrared-guided missile
    } seekerType{SeekerType::None};

    bool   isFiring{false};   // for aircraft: currently firing guns
    bool   isDead{false};

    // For aircraft: is the fire-control radar emitting? (set by host based on
    // radar mode). RWR detects this. Defaults to true to preserve historical
    // behavior (the previous code assumed all aircraft within range were
    // emitting). Hosts that model radar modes (RWS/TWS/SAM/STT/Off) should
    // set this false when the radar is in standby or off.
    bool   isRadarEmitting{true};

    // --- Convenience accessors ---
    bool isMissile() const { return seekerType != SeekerType::None; }
    bool isAircraft() const { return seekerType == SeekerType::None && !isDead; }
};

// ===========================================================================
// Relative geometry helpers — compute the angles/distances the digi AI needs.
//
// FreeFalcon stores these in SimObjectLocalData (az, el, ata, ataFrom, droll,
// range, rangedot, etc.). We compute them on-demand from entity positions +
// velocities to avoid storing per-pair cached data.
// ===========================================================================

// RelativeGeometry — the set of relative quantities the digi AI reads.
struct RelativeGeometry {
    double range{0.0};       // 3D distance (ft)
    double rangedot{0.0};    // range rate (ft/s, negative = closing)
    double az{0.0};          // azimuth of target from self (rad, relative to self heading)
    double el{0.0};          // elevation of target from self (rad)
    double ata{0.0};         // angle off self's nose to target (rad, = az in 2D)
    double ataFrom{0.0};     // angle off target's nose to self (rad)
    double azFrom{0.0};      // azimuth of self from target (rad, relative to target heading)
    double elFrom{0.0};      // elevation of self from target (rad)
    double droll{0.0};       // roll difference: target.roll - self.roll (rad)
    double closure{0.0};     // closure rate (ft/s, positive = closing)
};

// Compute relative geometry: self observing target.
//   self   : the observing aircraft (the AI's own aircraft)
//   target : the entity being observed (missile, aircraft, etc.)
inline RelativeGeometry computeRelativeGeometry(const DigiEntity& self,
                                                 const DigiEntity& target) {
    RelativeGeometry rg;

    const double dx = target.x - self.x;
    const double dy = target.y - self.y;
    const double dz = target.z - self.z;

    rg.range = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Range rate: d(range)/dt = (relative_pos · relative_vel) / range
    // Negative = closing
    const double dvx = target.vx - self.vx;
    const double dvy = target.vy - self.vy;
    const double dvz = target.vz - self.vz;
    if (rg.range > 1.0) {
        rg.rangedot = (dx * dvx + dy * dvy + dz * dvz) / rg.range;
    }
    rg.closure = -rg.rangedot;  // positive = closing

    // Azimuth of target from self (world frame)
    const double bearingToTarget = std::atan2(dy, dx);

    // Azimuth of self from target (world frame) = bearingToTarget + PI
    const double bearingToSelf = bearingToTarget + PI;

    // Angle off self's nose (ata = az in 2D)
    rg.az = rg.ata = bearingToTarget - self.yaw;
    // Wrap to [-PI, PI]
    while (rg.az >  PI) rg.az -= 2.0 * PI;
    while (rg.az < -PI) rg.az += 2.0 * PI;
    rg.ata = rg.az;

    // Angle off target's nose (ataFrom = azFrom in 2D)
    rg.azFrom = rg.ataFrom = bearingToSelf - target.yaw;
    while (rg.azFrom >  PI) rg.azFrom -= 2.0 * PI;
    while (rg.azFrom < -PI) rg.azFrom += 2.0 * PI;
    rg.ataFrom = rg.azFrom;

    // Elevation of target from self
    const double horizDist = std::sqrt(dx * dx + dy * dy);
    if (horizDist > 1.0) {
        rg.el = std::atan2(-dz, horizDist) - self.pitch;
        rg.elFrom = std::atan2(dz, horizDist) - target.pitch;  // dz reversed: self from target
    }

    // Roll difference
    rg.droll = target.roll - self.roll;

    return rg;
}

// ===========================================================================
// toDigiEntity — convert a SensorContact to a DigiEntity.
//
// BUG FIX (background): the brain previously rebuilt DigiEntity from
// SensorContact in three nearly-identical inline blocks (missile, guns,
// target), each copying a DIFFERENT field subset. The missile path
// hardcoded seekerType=Radar (breaking IR missile detection); the guns
// path hardcoded isFiring=true (ignoring sensor data); the target path
// missed seekerType and isFiring entirely. Any new field added to
// DigiEntity would have to be added in three places.
//
// This helper centralizes the conversion so all fields are copied
// consistently. The SensorContact's `isMissile` flag maps to seekerType
// (Radar by default — a real sensor model would distinguish IR vs radar
// missiles, but for now we preserve the original behavior of treating
// auto-detected missiles as radar-guided).
//
// SensorContact is forward-declared to avoid a circular include
// (sensor_picture.h includes digi_entity.h).
// ===========================================================================
struct SensorContact;

inline DigiEntity toDigiEntity(const struct SensorContact& c);

} // namespace digi
} // namespace f4flight
