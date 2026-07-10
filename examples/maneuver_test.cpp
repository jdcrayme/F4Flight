// f4flight - Maneuver test with cruise-stabilized sequencing and
// level-off-terminated climb/descent phases.
//
// Usage:
//   maneuver_test <aircraft.json>
//
// Each maneuver is preceded by a cruise stabilization period. Climb and
// descent phases terminate 60 seconds after level-off (to catch overshoots)
// or after a 1-hour timeout (for heavy aircraft).

#include "f4flight/f4flight.h"
#include "f4flight/json_io.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <limits>

using namespace f4flight;

// ---------------------------------------------------------------------------
// Aircraft category profiles
// ---------------------------------------------------------------------------
enum class AircraftCategory {
    Fighter, Attack, Bomber, Transport, Interceptor, Patrol, Trainer, Generic,
};

struct CategoryProfile {
    AircraftCategory category;
    const char* name;
    double cruiseVcas_kts;
    double cruiseAlt_ft;
    double climbVcas_kts;
    double climbMach;
    double descentVcas_kts;
    double descentMach;
    double climbPower;
    double descentPower;
    double maxBank_deg;
};

// Profiles use the convention: climb speed < cruise speed < descent speed
// This matches real aviation practice: slow climb for best climb angle,
// fast descent for efficient descent.
static const CategoryProfile PROFILES[] = {
    {AircraftCategory::Fighter,    "Fighter",    420, 15000, 350, 0.80, 460, 0.80, 1.0, 0.05, 45},
    {AircraftCategory::Attack,     "Attack",     280, 10000, 230, 0.55, 320, 0.60, 1.0, 0.10, 30},
    {AircraftCategory::Bomber,     "Bomber",     300, 5000,  250, 0.60, 340, 0.65, 1.0, 0.10, 30},
    {AircraftCategory::Transport,  "Transport",  250, 5000,  210, 0.50, 290, 0.55, 1.0, 0.15, 25},
    {AircraftCategory::Interceptor,"Interceptor",450, 20000, 380, 0.85, 500, 0.85, 1.0, 0.05, 45},
    {AircraftCategory::Patrol,     "Patrol",     250, 10000, 210, 0.55, 290, 0.60, 0.8, 0.15, 25},
    {AircraftCategory::Trainer,    "Trainer",    230, 10000, 200, 0.50, 270, 0.55, 1.0, 0.10, 30},
    {AircraftCategory::Generic,    "Generic",    300, 10000, 270, 0.70, 340, 0.70, 1.0, 0.10, 30},
};

AircraftCategory categorizeAircraft(const std::string& name) {
    if (name.find("f16") != std::string::npos || name.find("f15") != std::string::npos ||
        name.find("f18") != std::string::npos || name.find("f14") != std::string::npos ||
        name.find("f22") != std::string::npos || name.find("f4") != std::string::npos ||
        name.find("f5") != std::string::npos || name.find("f117") != std::string::npos ||
        name.find("mig29") != std::string::npos || name.find("su27") != std::string::npos ||
        name.find("su30") != std::string::npos || name.find("su33") != std::string::npos ||
        name.find("su35") != std::string::npos || name.find("ef2000") != std::string::npos ||
        name.find("rafale") != std::string::npos || name.find("mirage2k") != std::string::npos ||
        name.find("cf188") != std::string::npos || name.find("kf16") != std::string::npos ||
        name.find("tornf3") != std::string::npos || name.find("f104") != std::string::npos ||
        name.find("ja37") != std::string::npos || name.find("aj37") != std::string::npos ||
        name.find("fck1") != std::string::npos)
        return AircraftCategory::Fighter;
    if (name.find("mig25") != std::string::npos || name.find("mig31") != std::string::npos ||
        name.find("sr71") != std::string::npos || name.find("u2") != std::string::npos ||
        name.find("su15") != std::string::npos)
        return AircraftCategory::Interceptor;
    if (name.find("a10") != std::string::npos || name.find("su25") != std::string::npos ||
        name.find("su39") != std::string::npos || name.find("av8b") != std::string::npos ||
        name.find("a4") != std::string::npos || name.find("a6") != std::string::npos ||
        name.find("a7") != std::string::npos || name.find("amx") != std::string::npos ||
        name.find("q5") != std::string::npos || name.find("su7") != std::string::npos ||
        name.find("f111") != std::string::npos || name.find("su24") != std::string::npos ||
        name.find("su17") != std::string::npos || name.find("su22") != std::string::npos ||
        name.find("jaguargr3") != std::string::npos || name.find("ov10") != std::string::npos)
        return AircraftCategory::Attack;
    if (name.find("b52") != std::string::npos || name.find("b1b") != std::string::npos ||
        name.find("b2a") != std::string::npos || name.find("tu16") != std::string::npos ||
        name.find("tu22") != std::string::npos || name.find("tu95") != std::string::npos ||
        name.find("tu160") != std::string::npos || name.find("tornids") != std::string::npos)
        return AircraftCategory::Bomber;
    if (name.find("c130") != std::string::npos || name.find("c17") != std::string::npos ||
        name.find("c5") != std::string::npos || name.find("c141") != std::string::npos ||
        name.find("c160") != std::string::npos || name.find("kc135") != std::string::npos ||
        name.find("kc10") != std::string::npos || name.find("kdc10") != std::string::npos ||
        name.find("il76") != std::string::npos || name.find("il78") != std::string::npos ||
        name.find("an124") != std::string::npos || name.find("an72") != std::string::npos ||
        name.find("an24") != std::string::npos || name.find("an2") != std::string::npos ||
        name.find("a50") != std::string::npos || name.find("mv22") != std::string::npos)
        return AircraftCategory::Transport;
    if (name.find("e2c") != std::string::npos || name.find("e3a") != std::string::npos ||
        name.find("e8c") != std::string::npos || name.find("rc135") != std::string::npos ||
        name.find("ea6b") != std::string::npos || name.find("ea18g") != std::string::npos ||
        name.find("ef111") != std::string::npos || name.find("ac130") != std::string::npos)
        return AircraftCategory::Patrol;
    if (name.find("mb339") != std::string::npos)
        return AircraftCategory::Trainer;
    return AircraftCategory::Generic;
}

const CategoryProfile& getProfile(AircraftCategory cat) {
    for (const auto& p : PROFILES)
        if (p.category == cat) return p;
    return PROFILES[sizeof(PROFILES)/sizeof(PROFILES[0]) - 1];
}

// ---------------------------------------------------------------------------
// Maneuver types
// ---------------------------------------------------------------------------
enum class ManeuverType {
    Cruise,       // hold altitude + speed + heading (stabilization)
    Climb,        // climb to target altitude
    Descent,      // descend to target altitude
    Turn,         // turn to target heading
    Orbit,        // orbit (constant bank)
    Accel,        // accelerate to target speed
    Decel,        // decelerate to target speed
};

struct Maneuver {
    ManeuverType type;
    const char* name;
    double targetAlt_ft;
    double targetSpeed_kts;
    double targetHeading_rad;
    // For climb/descent: terminate 60s after level-off or after maxTime_s
    double maxTime_s;
};

// ---------------------------------------------------------------------------
// Phase tracker
// ---------------------------------------------------------------------------
struct PhaseTracker {
    bool altCaptured{false};
    double altCaptureTime_s{-1.0};
    double maxAltOvershoot_ft{0.0};
    double maxAltErrAfter_ft{0.0};
    bool spdCaptured{false};
    double spdCaptureTime_s{-1.0};
    double maxSpdErrAfter_kts{0.0};
    double maxBank_deg{0.0};
    double maxHdgRate_dps{0.0};

    void reset() {
        altCaptured = false; altCaptureTime_s = -1.0; maxAltOvershoot_ft = 0.0;
        maxAltErrAfter_ft = 0.0;
        spdCaptured = false; spdCaptureTime_s = -1.0; maxSpdErrAfter_kts = 0.0;
        maxBank_deg = 0.0; maxHdgRate_dps = 0.0;
    }
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <aircraft.json>\n", argv[0]);
        return 1;
    }

    std::string path = argv[1];
    std::string acName = path.substr(path.find_last_of('/') + 1);
    acName = acName.substr(0, acName.find_last_of('.'));

    AircraftConfig cfg;
    auto result = json::readFile(path, cfg);
    if (!result.ok) { std::fprintf(stderr, "Failed to load %s\n", path.c_str()); return 1; }

    AircraftCategory cat = categorizeAircraft(acName);
    const CategoryProfile& prof = getProfile(cat);

    double cruiseAlt = prof.cruiseAlt_ft;
    double climbAlt = cruiseAlt + 10000.0;
    double descentAlt = std::max(3000.0, cruiseAlt - 10000.0);

    std::printf("=== Maneuver test: %s ===\n", cfg.name.c_str());
    std::printf("Category: %s, cruise %d kts @ %d ft\n", prof.name,
                (int)prof.cruiseVcas_kts, (int)cruiseAlt);
    std::printf("Capture spec: +/- 100 ft, +/- 10 kts\n\n");

    FlightModel fm;
    fm.init(cfg, cruiseAlt, prof.cruiseVcas_kts * KNOTS_TO_FTPSEC, 0.0, true);

    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);
    sc.setMaxBankAngle_deg(prof.maxBank_deg);
    sc.setMaxGs(cfg.geometry.maxGs);

    SteeringGoal goal;
    goal.hasHeading = true;
    goal.heading_rad = 0.0;
    goal.hasAltitude = true;
    goal.hasSpeed = true;
    goal.speed_kts = prof.cruiseVcas_kts;
    goal.climbPower = prof.climbPower;
    goal.descentPower = prof.descentPower;
    goal.climbVcas_kts = prof.climbVcas_kts;
    goal.climbMach = prof.climbMach;
    goal.descentVcas_kts = prof.descentVcas_kts;
    goal.descentMach = prof.descentMach;
    goal.levelBand_ft = 200.0;

    // Build maneuver sequence: cruise → climb → cruise → descent → cruise →
    // turn → cruise → orbit → cruise → accel → cruise → decel → cruise
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise stable",     cruiseAlt,  prof.cruiseVcas_kts, 0.0, 60});
    maneuvers.push_back({ManeuverType::Climb,    "Climb",             climbAlt,   prof.cruiseVcas_kts, 0.0, 3600});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after climb",climbAlt,   prof.cruiseVcas_kts, 0.0, 30});
    maneuvers.push_back({ManeuverType::Descent,  "Descend",           descentAlt, prof.cruiseVcas_kts, 0.0, 3600});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after desc", descentAlt, prof.cruiseVcas_kts, 0.0, 30});
    maneuvers.push_back({ManeuverType::Turn,     "Turn 90 right",     descentAlt, prof.cruiseVcas_kts, 90.0*DTR, 60});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after turn", descentAlt, prof.cruiseVcas_kts, 90.0*DTR, 20});
    maneuvers.push_back({ManeuverType::Orbit,    "Orbit 360 right",   descentAlt, prof.cruiseVcas_kts, 90.0*DTR, 90});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after orbit",descentAlt, prof.cruiseVcas_kts, 90.0*DTR, 20});
    // Use descent speed for accel target, climb speed for decel target.
    // These are realistic speeds the aircraft can actually achieve.
    maneuvers.push_back({ManeuverType::Accel,    "Accelerate",        descentAlt, prof.descentVcas_kts, 90.0*DTR, 60});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after accel",descentAlt, prof.descentVcas_kts, 90.0*DTR, 20});
    maneuvers.push_back({ManeuverType::Decel,    "Decelerate",        descentAlt, prof.climbVcas_kts, 90.0*DTR, 60});
    maneuvers.push_back({ManeuverType::Cruise,   "Cruise after decel",descentAlt, prof.cruiseVcas_kts, 90.0*DTR, 20});

    const double ALT_BAND = 100.0;
    const double SPD_BAND = 10.0;
    const double dt = 1.0 / 60.0;
    double simTime = 0.0;

    std::printf("%-22s %6s %8s %8s %7s %7s %7s %6s %6s %5s\n",
                "Phase", "t(s)", "alt(ft)", "altErr", "vcas", "spdErr",
                "throt", "hdg(d)", "bank(d)", "G");

    for (const auto& man : maneuvers) {
        // Set goal for this maneuver
        goal.altitude_ft = man.targetAlt_ft;
        goal.speed_kts = man.targetSpeed_kts;
        goal.climbVcas_kts = prof.climbVcas_kts;
        if (man.type != ManeuverType::Orbit)
            goal.heading_rad = man.targetHeading_rad;
        sc.setGoal(goal);

        PhaseTracker tracker;
        tracker.reset();
        double phaseTime = 0.0;
        double nextPrint = 0.0;
        bool firstFrame = true;
        double prevHeading = fm.state().kin.psi;
        bool levelOffDetected = false;
        double levelOffTime = -1.0;

        // For climb/descent: terminate 60s after level-off or at maxTime
        // For other types: terminate at maxTime
        while (phaseTime < man.maxTime_s) {
            PilotInput input;
            if (man.type == ManeuverType::Orbit) {
                input = sc.compute(fm.state(), dt, 0.0);
                double orbitBank = prof.maxBank_deg * 0.6 * DTR;
                input.rstick = limit((orbitBank - fm.state().kin.phi) * 2.0, -1.0, 1.0);
            } else {
                input = sc.compute(fm.state(), dt, 0.0);
            }
            fm.update(dt, input, 0.0, Vec3{0.0, 0.0, 1.0});
            simTime += dt;
            phaseTime += dt;

            const auto& s = fm.state();
            const double alt = -s.kin.z;
            const double altErr = man.targetAlt_ft - alt;
            const double spdErr = man.targetSpeed_kts - s.vcas;

            // Track overshoot
            if (man.type == ManeuverType::Climb && altErr < 0)
                tracker.maxAltOvershoot_ft = std::max(tracker.maxAltOvershoot_ft, -altErr);
            if (man.type == ManeuverType::Descent && altErr > 0)
                tracker.maxAltOvershoot_ft = std::max(tracker.maxAltOvershoot_ft, altErr);

            // Track capture
            if (!tracker.altCaptured && std::fabs(altErr) < ALT_BAND && !firstFrame) {
                tracker.altCaptured = true;
                tracker.altCaptureTime_s = phaseTime;
            }
            if (!tracker.spdCaptured && std::fabs(spdErr) < SPD_BAND && !firstFrame) {
                tracker.spdCaptured = true;
                tracker.spdCaptureTime_s = phaseTime;
            }
            if (tracker.altCaptured)
                tracker.maxAltErrAfter_ft = std::max(tracker.maxAltErrAfter_ft, std::fabs(altErr));
            if (tracker.spdCaptured)
                tracker.maxSpdErrAfter_kts = std::max(tracker.maxSpdErrAfter_kts, std::fabs(spdErr));

            // Track turn data
            tracker.maxBank_deg = std::max(tracker.maxBank_deg, std::fabs(s.kin.phi * RTD));
            double hdgDelta = (s.kin.psi - prevHeading) * RTD;
            while (hdgDelta > 180.0) hdgDelta -= 360.0;
            while (hdgDelta < -180.0) hdgDelta += 360.0;
            tracker.maxHdgRate_dps = std::max(tracker.maxHdgRate_dps, std::fabs(hdgDelta / dt));
            prevHeading = s.kin.psi;

            // Detect level-off for climb/descent
            if ((man.type == ManeuverType::Climb || man.type == ManeuverType::Descent) &&
                !levelOffDetected && std::fabs(altErr) < ALT_BAND) {
                levelOffDetected = true;
                levelOffTime = phaseTime;
            }

            // Print every 10s
            if (phaseTime >= nextPrint) {
                std::printf("%-22s %6.0f %8.0f %8.0f %7.1f %7.1f %7.2f %6.1f %6.1f %5.2f\n",
                            man.name, simTime, alt, altErr,
                            s.vcas, spdErr, input.throttle,
                            s.kin.psi * RTD, s.kin.phi * RTD,
                            s.loads.nzcgs);
                nextPrint += 10.0;
            }

            // Termination: for climb/descent, end 60s after level-off
            if (levelOffDetected && (phaseTime - levelOffTime) > 60.0)
                break;

            firstFrame = false;
        }

        // Print summary
        std::printf("  --- Summary ---\n");
        if (tracker.altCaptured) {
            std::printf("  ALT: cap t+%.1fs, ovrsht %.0f ft, maxErrAft %.0f ft %s\n",
                        tracker.altCaptureTime_s, tracker.maxAltOvershoot_ft,
                        tracker.maxAltErrAfter_ft,
                        tracker.maxAltErrAfter_ft < ALT_BAND ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  ALT: NOT captured [FAIL]\n");
        }
        if (tracker.spdCaptured) {
            std::printf("  SPD: cap t+%.1fs, maxErrAft %.1f kts %s\n",
                        tracker.spdCaptureTime_s, tracker.maxSpdErrAfter_kts,
                        tracker.maxSpdErrAfter_kts < SPD_BAND ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  SPD: NOT captured [FAIL]\n");
        }
        if (tracker.maxBank_deg > 5.0) {
            std::printf("  TRN: maxBank %.1f deg, maxHdgRate %.1f dps\n",
                        tracker.maxBank_deg, tracker.maxHdgRate_dps);
        }
        std::printf("\n");
    }

    std::printf("=== Maneuver test complete ===\n");
    return 0;
}
