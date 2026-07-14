// f4flight - maneuver_test_assertions.h
//
// Shared assertion helpers for maneuver test phases.
//
// These helpers standardize the common pass/fail checks that every phase needs:
//   - NaN detection (comprehensive — checks all critical state fields)
//   - Ground penetration tracking
//   - Mode entry tracking
//
// Without these helpers, each phase reimplements its own ad-hoc checks with
// different field sets and thresholds, making it easy to write trivially-
// passing tests (e.g., the old LandingPhase only checked kin.vt and kin.z for
// NaN, missing divergences in theta/phi/alpha/qbar).
//
// Usage:
//   class MyPhase : public ManeuverTest {
//       PhaseAssertions a_;
//   public:
//       void Evaluate(...) override {
//           a_.checkNaN(state);
//           a_.updateGroundPenetration(state);
//           ...
//       }
//       bool IsPassed() const override {
//           return a_.noNaN() && a_.noGroundPenetration();
//       }
//   };

#pragma once

#include "f4flight/f4flight.h"
#include <cmath>
#include <limits>

namespace manuver_test {

// Standard NaN check tolerance for ground penetration. The EOM pins z to
// groundZ - minHeight, so any penetration > this indicates a physics bug.
constexpr double kGroundPenetrationTolerance = 5.0;  // ft

// PhaseAssertions — shared state tracking and pass/fail checks for a phase.
//
// Each ManeuverTest phase owns one of these and calls its update methods from
// Evaluate(). The phase's IsPassed() then delegates to the shared checks.
struct PhaseAssertions {
    bool   hasNaN{false};
    double maxGroundPenetration{0.0};  // ft (positive = below ground)

    // Comprehensive NaN check. Checks all critical kinematic + aero + atmospheric
    // fields. A NaN in any of these indicates the simulation has diverged.
    void checkNaN(const f4flight::AircraftState& as) {
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z) ||
            std::isnan(as.kin.theta) || std::isnan(as.kin.phi) ||
            std::isnan(as.kin.psi) || std::isnan(as.kin.x) || std::isnan(as.kin.y) ||
            std::isnan(as.aero.alpha_deg) || std::isnan(as.aero.beta_deg) ||
            std::isnan(as.qbar) || std::isnan(as.mach) ||
            std::isnan(as.loads.nzcgs)) {
            hasNaN = true;
        }
    }

    // Track ground penetration. Call every frame with the current state.
    // groundZ is the terrain altitude at the aircraft position (positive up).
    void updateGroundPenetration(const f4flight::AircraftState& as, double groundZ) {
        const double altAGL = -as.kin.z - groundZ;
        if (altAGL < 0.0) {
            maxGroundPenetration = std::max(maxGroundPenetration, -altAGL);
        }
    }

    // --- Pass/fail predicates ---
    bool noNaN() const { return !hasNaN; }
    bool noGroundPenetration(double tolerance = kGroundPenetrationTolerance) const {
        return maxGroundPenetration <= tolerance;
    }

    // Reset between runs (if a phase is reused).
    void reset() {
        hasNaN = false;
        maxGroundPenetration = 0.0;
    }

    // Print summary lines for the common checks. Call from Finish().
    void printSummary(const char* prefix = "  ") const {
        std::printf("%sNo NaN:              %s\n", prefix,
            noNaN() ? "[PASS]" : "[FAIL]");
        std::printf("%sNo ground penetration: %s (max %.2f ft below)\n", prefix,
            noGroundPenetration() ? "[PASS]" : "[FAIL]",
            maxGroundPenetration);
    }
};

} // namespace manuver_test
