// tune_profile - Empirically tune a PerformanceProfile for an aircraft.
//
// Usage:
//   tune_profile <aircraft.json> [--write] [--verbose]
//
// Loads an aircraft JSON file, runs short simulations at different cruise,
// climb, and descent speeds, and finds the combination that gives the
// most stable flight (smallest altitude/speed deviations). Prints the
// recommended profile and optionally writes it back into the JSON file.
//
// The tuning process:
//   1. Start with the auto-derived profile (cfg.deriveProfile()).
//   2. Test cruise: at the cruise altitude, try 5-7 speeds around the
//      derived cruise speed. Score each by how well the aircraft holds
//      altitude and speed over 60 seconds. Pick the best.
//   3. Test climb: from cruise alt to cruise+10000, try 5 speeds around
//      the derived climb speed at MIL power. Score by altitude capture
//      accuracy and speed deviation. Pick the best.
//   4. Test descent: from cruise+10000 to cruise-10000, try 5 speeds.
//      Score by altitude capture and speed deviation. Pick the best.
//   5. Print the recommended profile. If --write, update the JSON file.
//
// This takes ~2-3 minutes per aircraft (each test phase runs 60-180
// seconds of simulated time at 60 Hz).

#include "f4flight/f4flight.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace f4flight;

// ---------------------------------------------------------------------------
// Test result for a single speed trial
// ---------------------------------------------------------------------------
struct TrialResult {
    double speed_kts;
    double altRmsErr_ft;      // RMS altitude error after capture
    double spdRmsErr_kts;     // RMS speed error after capture
    double altMaxDev_ft;      // max |alt - target| after capture
    double spdMaxDev_kts;     // max |spd - target| after capture
    bool   captured;          // whether altitude was captured at all
    double captureTime_s;     // time to capture altitude
    double score;             // lower is better
};

// ---------------------------------------------------------------------------
// Run a level-flight trial at a given speed and altitude.
// Returns the trial result.
// ---------------------------------------------------------------------------
TrialResult testCruise(const AircraftConfig& cfg, double alt_ft, double speed_kts) {
    TrialResult r;
    r.speed_kts = speed_kts;
    r.altRmsErr_ft = 1e9;
    r.spdRmsErr_kts = 1e9;
    r.altMaxDev_ft = 1e9;
    r.spdMaxDev_kts = 1e9;
    r.captured = false;
    r.captureTime_s = 1e9;
    r.score = 1e9;

    FlightModel fm;
    fm.init(cfg, alt_ft, calcTasFromKcas(speed_kts, alt_ft), 0.0, true);

    SteeringController sc;
    sc.setMaxBankAngle_deg(cfg.profile.maxBank_deg);
    sc.setMaxGs(cfg.geometry.maxGs);
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(alt_ft));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(speed_kts));

    const double dt = 1.0 / 60.0;
    const double maxTime = 90.0;  // 1.5 minutes
    double t = 0.0;

    double altSumSq = 0.0, spdSumSq = 0.0;
    double altMax = 0.0, spdMax = 0.0;
    int samples = 0;
    bool captured = false;
    double captureTime = 0.0;
    const double CAPTURE_BAND = 100.0; // ft

    while (t < maxTime) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});

        double alt = -fm.state().kin.z;
        double spd = fm.state().vcas;
        double altErr = std::fabs(alt - alt_ft);
        double spdErr = std::fabs(spd - speed_kts);

        // Check for NaN (numerical instability)
        if (std::isnan(alt) || std::isnan(spd)) {
            r.captured = false;
            r.score = 1e9;
            return r;
        }

        if (!captured && altErr < CAPTURE_BAND) {
            captured = true;
            captureTime = t;
        }

        if (captured) {
            altSumSq += altErr * altErr;
            spdSumSq += spdErr * spdErr;
            altMax = std::max(altMax, altErr);
            spdMax = std::max(spdMax, spdErr);
            ++samples;
        }

        t += dt;
    }

    r.captured = captured;
    r.captureTime_s = captured ? captureTime : 1e9;
    if (samples > 0) {
        r.altRmsErr_ft = std::sqrt(altSumSq / samples);
        r.spdRmsErr_kts = std::sqrt(spdSumSq / samples);
        r.altMaxDev_ft = altMax;
        r.spdMaxDev_kts = spdMax;
        // Score: weighted combination of RMS and max deviations.
        // Lower is better. Speed is weighted higher than altitude because
        // speed oscillation is the primary failure mode.
        r.score = r.spdRmsErr_kts * 1.0 + r.altRmsErr_ft * 0.01 +
                  r.spdMaxDev_kts * 0.5 + r.altMaxDev_ft * 0.005;
        if (!captured) r.score += 1000.0; // huge penalty for no capture
    }

    return r;
}

// ---------------------------------------------------------------------------
// Run a climb trial: from start_alt to target_alt at a given climb speed.
// ---------------------------------------------------------------------------
TrialResult testClimb(const AircraftConfig& cfg, double startAlt_ft,
                      double targetAlt_ft, double climbSpeed_kts, double climbPower) {
    TrialResult r;
    r.speed_kts = climbSpeed_kts;
    r.altRmsErr_ft = 1e9;
    r.spdRmsErr_kts = 1e9;
    r.altMaxDev_ft = 1e9;
    r.spdMaxDev_kts = 1e9;
    r.captured = false;
    r.captureTime_s = 1e9;
    r.score = 1e9;

    FlightModel fm;
    fm.init(cfg, startAlt_ft, calcTasFromKcas(climbSpeed_kts, startAlt_ft), 0.0, true);

    SteeringController sc;
    sc.setMaxBankAngle_deg(cfg.profile.maxBank_deg);
    sc.setMaxGs(cfg.geometry.maxGs);
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(targetAlt_ft));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(climbSpeed_kts));

    const double dt = 1.0 / 60.0;
    const double maxTime = 300.0; // 5 minutes max for climb
    double t = 0.0;

    double altSumSq = 0.0, spdSumSq = 0.0;
    double altMax = 0.0, spdMax = 0.0;
    double spdMin = 1e9, spdMaxAll = 0.0;
    int samples = 0;
    bool captured = false;
    double captureTime = 0.0;
    const double CAPTURE_BAND = 200.0;

    while (t < maxTime) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});

        double alt = -fm.state().kin.z;
        double spd = fm.state().vcas;
        double altErr = std::fabs(alt - targetAlt_ft);
        double spdErr = std::fabs(spd - climbSpeed_kts);

        if (std::isnan(alt) || std::isnan(spd)) {
            r.captured = false;
            r.score = 1e9;
            return r;
        }

        // Track speed range during the climb (before capture)
        if (!captured) {
            spdMin = std::min(spdMin, spd);
            spdMaxAll = std::max(spdMaxAll, spd);
        }

        if (!captured && altErr < CAPTURE_BAND && t > 10.0) {
            captured = true;
            captureTime = t;
        }

        if (captured) {
            altSumSq += altErr * altErr;
            spdSumSq += spdErr * spdErr;
            altMax = std::max(altMax, altErr);
            spdMax = std::max(spdMax, spdErr);
            ++samples;
        }

        // Stop early if we've held altitude for 60s after capture
        if (captured && t > captureTime + 60.0) break;

        t += dt;
    }

    r.captured = captured;
    r.captureTime_s = captured ? captureTime : 1e9;
    if (samples > 0) {
        r.altRmsErr_ft = std::sqrt(altSumSq / samples);
        r.spdRmsErr_kts = std::sqrt(spdSumSq / samples);
        r.altMaxDev_ft = altMax;
        r.spdMaxDev_kts = spdMax;
        // Score: penalize speed deviation during climb heavily (the main
        // failure mode is speed bleeding to ~150 kts or oscillating).
        // Also penalize the speed range during the climb.
        double spdRange = spdMaxAll - spdMin;
        r.score = r.spdRmsErr_kts * 1.0 + r.altRmsErr_ft * 0.005 +
                  r.spdMaxDev_kts * 0.5 + r.altMaxDev_ft * 0.002 +
                  spdRange * 0.3;
        if (!captured) r.score += 500.0;
        // Penalize very long capture times
        if (captured && captureTime > 180.0) r.score += (captureTime - 180.0) * 2.0;
    }

    return r;
}

// ---------------------------------------------------------------------------
// Run a descent trial: from start_alt to target_alt at a given descent speed.
// ---------------------------------------------------------------------------
TrialResult testDescent(const AircraftConfig& cfg, double startAlt_ft,
                        double targetAlt_ft, double descentSpeed_kts) {
    TrialResult r;
    r.speed_kts = descentSpeed_kts;
    r.altRmsErr_ft = 1e9;
    r.spdRmsErr_kts = 1e9;
    r.altMaxDev_ft = 1e9;
    r.spdMaxDev_kts = 1e9;
    r.captured = false;
    r.captureTime_s = 1e9;
    r.score = 1e9;

    FlightModel fm;
    fm.init(cfg, startAlt_ft, calcTasFromKcas(descentSpeed_kts, startAlt_ft), 0.0, true);

    SteeringController sc;
    sc.setMaxBankAngle_deg(cfg.profile.maxBank_deg);
    sc.setMaxGs(cfg.geometry.maxGs);
    sc.setVerticalBehavior(std::make_unique<AltitudeHold>(targetAlt_ft));
    sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0.0));
    sc.setThrottleBehavior(std::make_unique<SpeedHold>(descentSpeed_kts));

    const double dt = 1.0 / 60.0;
    const double maxTime = 300.0;
    double t = 0.0;

    double altSumSq = 0.0, spdSumSq = 0.0;
    double altMax = 0.0, spdMax = 0.0;
    int samples = 0;
    bool captured = false;
    double captureTime = 0.0;
    const double CAPTURE_BAND = 200.0;

    while (t < maxTime) {
        PilotInput input = sc.compute(fm.state(), dt, 0.0);
        fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});

        double alt = -fm.state().kin.z;
        double spd = fm.state().vcas;
        double altErr = std::fabs(alt - targetAlt_ft);
        double spdErr = std::fabs(spd - descentSpeed_kts);

        if (std::isnan(alt) || std::isnan(spd)) {
            r.captured = false;
            r.score = 1e9;
            return r;
        }

        if (!captured && altErr < CAPTURE_BAND && t > 10.0) {
            captured = true;
            captureTime = t;
        }

        if (captured) {
            altSumSq += altErr * altErr;
            spdSumSq += spdErr * spdErr;
            altMax = std::max(altMax, altErr);
            spdMax = std::max(spdMax, spdErr);
            ++samples;
        }

        if (captured && t > captureTime + 60.0) break;

        t += dt;
    }

    r.captured = captured;
    r.captureTime_s = captured ? captureTime : 1e9;
    if (samples > 0) {
        r.altRmsErr_ft = std::sqrt(altSumSq / samples);
        r.spdRmsErr_kts = std::sqrt(spdSumSq / samples);
        r.altMaxDev_ft = altMax;
        r.spdMaxDev_kts = spdMax;
        r.score = r.spdRmsErr_kts * 1.0 + r.altRmsErr_ft * 0.005 +
                  r.spdMaxDev_kts * 0.5 + r.altMaxDev_ft * 0.002;
        if (!captured) r.score += 500.0;
    }

    return r;
}

// ---------------------------------------------------------------------------
// Print a trial result
// ---------------------------------------------------------------------------
void printTrial(const char* label, const TrialResult& r) {
    std::printf("  %-28s spd=%5.0f  score=%7.1f  altRMS=%5.0f  spdRMS=%5.1f  altMax=%6.0f  spdMax=%5.1f  %s\n",
        label, r.speed_kts, r.score,
        r.altRmsErr_ft, r.spdRmsErr_kts,
        r.altMaxDev_ft, r.spdMaxDev_kts,
        r.captured ? "OK" : "NO-CAPTURE");
}

// ---------------------------------------------------------------------------
// Find the best trial from a list
// ---------------------------------------------------------------------------
TrialResult findBest(std::vector<TrialResult>& trials) {
    TrialResult best = trials[0];
    for (auto& r : trials) {
        if (r.score < best.score) best = r;
    }
    return best;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    bool writeOutput = false;
    bool verbose = false;
    std::string path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--write") writeOutput = true;
        else if (a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") {
            std::printf("Usage: tune_profile <aircraft.json> [--write] [--verbose]\n");
            std::printf("  --write    Write the tuned profile back into the JSON file\n");
            std::printf("  --verbose  Print every trial, not just the best\n");
            return 0;
        }
        else path = a;
    }

    if (path.empty()) {
        std::fprintf(stderr, "Error: no aircraft JSON specified.\n");
        std::fprintf(stderr, "Usage: tune_profile <aircraft.json> [--write] [--verbose]\n");
        return 1;
    }

    // Load the aircraft
    AircraftConfig cfg;
    auto result = json::readFile(path, cfg);
    if (!result.ok) {
        std::fprintf(stderr, "Failed to load %s\n", path.c_str());
        for (auto const& e : result.errors) std::fprintf(stderr, "  %s\n", e.c_str());
        return 1;
    }

    // Start with the auto-derived profile
    cfg.deriveProfile();

    std::printf("=== Tuning profile for %s ===\n", cfg.name.c_str());
    std::printf("Category: %s  (TWR=%.3f, nEngines=%d, maxGs=%.1f, maxVcas=%.0f)\n",
        cfg.profile.category.c_str(),
        (cfg.engine.thrust_mil.empty() ? 0.0 : cfg.engine.thrust_mil[0] * cfg.aux.nEngines /
         (cfg.geometry.emptyWeight_lbs + cfg.geometry.internalFuel_lbs)),
        cfg.aux.nEngines, cfg.geometry.maxGs, cfg.geometry.maxVcas_kts);
    std::printf("Derived: cruise=%.0f climb=%.0f descent=%.0f at alt=%.0f\n\n",
        cfg.profile.cruiseSpeed_kts, cfg.profile.climbSpeed_kts,
        cfg.profile.descentSpeed_kts, cfg.profile.cruiseAlt_ft);

    // --- 1. Tune cruise speed ---
    std::printf("--- Cruise speed trials (alt=%.0f ft) ---\n", cfg.profile.cruiseAlt_ft);
    double baseCruise = cfg.profile.cruiseSpeed_kts;
    double cruiseSteps[] = {-40, -20, 0, +20, +40};
    std::vector<TrialResult> cruiseTrials;
    for (double ds : cruiseSteps) {
        double spd = baseCruise + ds;
        if (spd < cfg.geometry.minVcas_kts * 1.1) continue;
        if (spd > cfg.geometry.maxVcas_kts * 0.9) continue;
        auto r = testCruise(cfg, cfg.profile.cruiseAlt_ft, spd);
        cruiseTrials.push_back(r);
        if (verbose) printTrial("cruise", r);
    }
    if (cruiseTrials.empty()) {
        // Fallback: use the derived speed
        auto r = testCruise(cfg, cfg.profile.cruiseAlt_ft, baseCruise);
        cruiseTrials.push_back(r);
    }
    auto bestCruise = findBest(cruiseTrials);
    std::printf("  Best cruise: %.0f kts (score %.1f, altRMS %.0f, spdRMS %.1f)\n\n",
        bestCruise.speed_kts, bestCruise.score, bestCruise.altRmsErr_ft, bestCruise.spdRmsErr_kts);

    // --- 2. Tune climb speed ---
    double cruiseAlt = cfg.profile.cruiseAlt_ft;
    double climbTarget = cruiseAlt + 10000.0;
    std::printf("--- Climb speed trials (%.0f -> %.0f ft) ---\n", cruiseAlt, climbTarget);
    double baseClimb = cfg.profile.climbSpeed_kts;
    double climbSteps[] = {-40, -20, 0, +20, +40};
    std::vector<TrialResult> climbTrials;
    for (double ds : climbSteps) {
        double spd = baseClimb + ds;
        if (spd < cfg.geometry.minVcas_kts * 1.1) continue;
        if (spd > cfg.geometry.maxVcas_kts * 0.9) continue;
        auto r = testClimb(cfg, cruiseAlt, climbTarget, spd, cfg.profile.climbPower);
        climbTrials.push_back(r);
        if (verbose) printTrial("climb", r);
    }
    if (climbTrials.empty()) {
        auto r = testClimb(cfg, cruiseAlt, climbTarget, baseClimb, cfg.profile.climbPower);
        climbTrials.push_back(r);
    }
    auto bestClimb = findBest(climbTrials);
    std::printf("  Best climb: %.0f kts (score %.1f, altRMS %.0f, spdRMS %.1f, capture %.0fs)\n\n",
        bestClimb.speed_kts, bestClimb.score, bestClimb.altRmsErr_ft,
        bestClimb.spdRmsErr_kts, bestClimb.captureTime_s);

    // --- 3. Tune descent speed ---
    double descentTarget = std::max(3000.0, cruiseAlt - 10000.0);
    std::printf("--- Descent speed trials (%.0f -> %.0f ft) ---\n", climbTarget, descentTarget);
    double baseDescent = cfg.profile.descentSpeed_kts;
    double descentSteps[] = {-40, -20, 0, +20, +40};
    std::vector<TrialResult> descentTrials;
    for (double ds : descentSteps) {
        double spd = baseDescent + ds;
        if (spd < cfg.geometry.minVcas_kts * 1.1) continue;
        if (spd > cfg.geometry.maxVcas_kts * 0.95) continue;
        auto r = testDescent(cfg, climbTarget, descentTarget, spd);
        descentTrials.push_back(r);
        if (verbose) printTrial("descent", r);
    }
    if (descentTrials.empty()) {
        auto r = testDescent(cfg, climbTarget, descentTarget, baseDescent);
        descentTrials.push_back(r);
    }
    auto bestDescent = findBest(descentTrials);
    std::printf("  Best descent: %.0f kts (score %.1f, altRMS %.0f, spdRMS %.1f)\n\n",
        bestDescent.speed_kts, bestDescent.score, bestDescent.altRmsErr_ft, bestDescent.spdRmsErr_kts);

    // --- Update the profile ---
    // Enforce the climb < cruise < descent ordering. The tuner tests each
    // speed independently, so it's possible for the best climb speed to
    // end up higher than the best cruise speed (e.g., if the aircraft
    // handles better at 360 than 340 in a climb, but cruises better at
    // 340). That violates the aviation convention and breaks the test
    // scenario's speed-capture logic. Clamp to maintain the ordering.
    double cruiseSpd = bestCruise.speed_kts;
    double climbSpd = std::min(bestClimb.speed_kts, cruiseSpd - 10.0);
    double descentSpd = std::max(bestDescent.speed_kts, cruiseSpd + 10.0);
    // Clamp to aircraft limits
    const double minSafe = std::max(cfg.geometry.minVcas_kts * 1.1, 150.0);
    if (climbSpd < minSafe) climbSpd = minSafe;
    if (descentSpd > cfg.geometry.maxVcas_kts * 0.95) descentSpd = cfg.geometry.maxVcas_kts * 0.95;

    cfg.profile.cruiseSpeed_kts = cruiseSpd;
    cfg.profile.climbSpeed_kts = climbSpd;
    cfg.profile.descentSpeed_kts = descentSpd;
    cfg.profile.tuned = true;

    // Print the final profile
    std::printf("=== Tuned profile ===\n");
    std::printf("  category:         %s\n", cfg.profile.category.c_str());
    std::printf("  cruiseSpeed_kts:  %.0f\n", cfg.profile.cruiseSpeed_kts);
    std::printf("  climbSpeed_kts:   %.0f\n", cfg.profile.climbSpeed_kts);
    std::printf("  climbMach:        %.2f\n", cfg.profile.climbMach);
    std::printf("  climbPower:       %.2f\n", cfg.profile.climbPower);
    std::printf("  descentSpeed_kts: %.0f\n", cfg.profile.descentSpeed_kts);
    std::printf("  descentMach:      %.2f\n", cfg.profile.descentMach);
    std::printf("  descentPower:     %.2f\n", cfg.profile.descentPower);
    std::printf("  cruiseAlt_ft:     %.0f\n", cfg.profile.cruiseAlt_ft);
    std::printf("  climbAlt_ft:      %.0f\n", cfg.profile.climbAlt_ft);
    std::printf("  descentAlt_ft:    %.0f\n", cfg.profile.descentAlt_ft);
    std::printf("  maxBank_deg:      %.0f\n", cfg.profile.maxBank_deg);
    std::printf("  levelBand_ft:     %.0f\n", cfg.profile.levelBand_ft);
    std::printf("  tuned:            true\n");

    // Write back if requested
    if (writeOutput) {
        if (!json::writeFile(cfg, path)) {
            std::fprintf(stderr, "ERROR: Failed to write %s\n", path.c_str());
            return 3;
        }
        std::printf("\nWrote tuned profile to %s\n", path.c_str());
    } else {
        std::printf("\n(use --write to save the profile to the JSON file)\n");
    }

    return 0;
}
