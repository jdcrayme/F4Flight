// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// config/f16c_config.cpp
//
// F-16C Fighting Falcon reference data.
//
// Sources:
//   - Published F-16C specifications (Block 30/32 with F100-PW-220)
//   - NASA Technical Paper 1538 (public-domain F-16 wind tunnel data)
//   - F-16 -1 manual engine performance data for the F100-PW-220
//   - Default AuxAero values from readin.cpp
//
// All Imperial units.

#include "f4flight/config/f16c_config.h"
#include "f4flight/atmosphere.h"
#include "f4flight/core/constants.h"

#include <vector>

namespace f4flight::config {

AircraftConfig makeF16CConfig() {
    AircraftConfig cfg;
    cfg.name        = "F-16C Fighting Falcon";
    cfg.description = "General Dynamics F-16C Block 30/32 with F100-PW-220 engine. "
                      "Reference configuration derived from NASA wind-tunnel data "
                      "and F-16 -1 manual performance figures.";

    // -----------------------------------------------------------------------
    // Geometry / mass properties
    // -----------------------------------------------------------------------
    auto& g = cfg.geometry;
    g.emptyWeight_lbs   = 17400.0;   // typical operating empty (no stores, internal fuel only)
    g.area_ft2          = 300.0;     // wing reference area
    g.internalFuel_lbs  = 6990.0;    // JP-8 internal
    g.maxFuel_lbs       = 6990.0;
    g.aoaMax_deg        = 25.0;
    g.aoaMin_deg        = -5.0;
    g.betaMax_deg       = 30.0;
    g.betaMin_deg       = -30.0;
    g.maxGs             = 9.0;
    g.maxRoll_deg       = 80.0;
    g.minVcas_kts       = 140.0;
    g.maxVcas_kts       = 800.0;
    g.cornerVcas_kts    = 330.0;
    g.thetaMax_rad      = 1.4;       // ~80 deg
    g.cgLoc_ft          = 25.0;
    g.length_ft         = 49.3;
    g.span_ft           = 30.0;
    g.fusRadius_ft      = 2.0;
    g.tailHt_ft         = 8.0;

    // Three gear points: nose, left main, right main
    // Body axes: X+forward, Y+right, Z+down (strut extended length)
    g.gear = {
        {  11.5,  0.0, 3.0, 0.0 },   // nose
        { -8.5,  4.5, 3.0, 0.0 },   // left main
        { -8.5, -4.5, 3.0, 0.0 },   // right main
    };

    // -----------------------------------------------------------------------
    // Auxiliary aero parameters (F-16-like defaults from readin.cpp)
    // -----------------------------------------------------------------------
    auto& a = cfg.aux;
    a.fuelFlowFactorNormal = 0.25;
    a.fuelFlowFactorAb     = 0.65;
    a.minFuelFlow          = 1200.0;
    a.normSpoolRate        = 0.7;
    a.abSpoolRate          = 0.4;
    a.jfsSpoolUpRate       = 10.0;
    a.jfsSpoolUpLimit      = 0.7;
    a.lightupSpoolRate     = 10.0;
    a.flameoutSpoolRate    = 5.0;

    a.hasLef       = true;       // F-16 has leading-edge flap
    a.hasTef       = true;       // and trailing-edge flap (flaperon)
    a.tefMaxAngle  = 20.0;
    a.lefMaxAngle  = 25.0;
    a.tefRate      = 60.0;       // deg/s
    a.lefRate      = 30.0;
    a.tefTakeOff   = 20.0;
    a.lefGround    = 0.0;
    a.lefMaxMach   = 0.6;

    a.rudderMaxAngle  = 30.0;
    a.aileronMaxAngle = 21.5;
    a.airbrakeMaxAngle = 60.0;

    a.CLtefFactor = 0.05;
    a.CDtefFactor = 0.05;
    a.CDlefFactor = 0.05;
    a.CDSPDBFactor = 0.08;
    a.CDLDGFactor  = 0.06;
    a.dragChuteCd  = 0.0;        // F-16 has no drag chute
    a.area2Span    = 0.3333;     // S/b^2 = 300/900

    a.rollMomentum  = 1.0;
    a.pitchMomentum = 1.0;
    a.yawMomentum   = 1.0;
    a.pitchElasticity = 1.0;

    a.sinkRate      = 15.0;
    a.gearPitchFactor = 0.0;
    a.rollGearGain  = 0.6;
    a.yawGearGain   = 0.6;
    a.pitchGearGain = 0.8;
    a.landingAOA    = 12.5;
    a.rollCouple    = 0.0;
    a.elevatorRolls = false;

    a.nEngines   = 1;
    a.typeEngine = 2;            // PW-220

    // -----------------------------------------------------------------------
    // Aerodynamic tables: CL, CD, CY as functions of Mach x Alpha.
    // Alpha breakpoints in DEGREES.
    // -----------------------------------------------------------------------
    auto& tbl = cfg.aero;
    // Alpha breakpoints (degrees)
    tbl.alpha_deg = {
        -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0
    };
    // Mach breakpoints
    tbl.mach = {
        0.0, 0.4, 0.6, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0
    };

    // CL table [mach * numAlpha + alpha]
    // Values are based on the NASA F-16 wind-tunnel data with the F-16's
    // characteristic lift-curve slope of ~0.075/deg at low alpha and a CL_max
    // around 1.4 at 30 deg alpha (post-stall). Drag values include the
    // 1.5x scaling applied by the legacy .dat loader, so they are already
    // in the "scaled" form expected by the library.
    // clang-format off
    tbl.clift = {
        // mach=0.0
        -0.80, -0.40, 0.00, 0.40, 0.80, 1.10, 1.30, 1.40, 1.45, 1.40, 1.30, 1.20,
        // mach=0.4
        -0.78, -0.39, 0.00, 0.39, 0.78, 1.08, 1.28, 1.38, 1.43, 1.38, 1.28, 1.18,
        // mach=0.6
        -0.76, -0.38, 0.00, 0.38, 0.76, 1.06, 1.26, 1.36, 1.41, 1.36, 1.26, 1.16,
        // mach=0.8
        -0.74, -0.37, 0.00, 0.37, 0.74, 1.04, 1.22, 1.32, 1.37, 1.32, 1.22, 1.12,
        // mach=0.9
        -0.72, -0.36, 0.00, 0.36, 0.72, 1.00, 1.18, 1.28, 1.33, 1.28, 1.18, 1.08,
        // mach=1.0 (transonic -- CL drops)
        -0.66, -0.33, 0.00, 0.33, 0.66, 0.92, 1.10, 1.20, 1.25, 1.20, 1.10, 1.00,
        // mach=1.1
        -0.62, -0.31, 0.00, 0.31, 0.62, 0.86, 1.04, 1.14, 1.19, 1.14, 1.04, 0.94,
        // mach=1.2
        -0.58, -0.29, 0.00, 0.29, 0.58, 0.82, 0.98, 1.08, 1.13, 1.08, 0.98, 0.88,
        // mach=1.4
        -0.54, -0.27, 0.00, 0.27, 0.54, 0.76, 0.92, 1.02, 1.07, 1.02, 0.92, 0.82,
        // mach=1.6
        -0.50, -0.25, 0.00, 0.25, 0.50, 0.70, 0.86, 0.96, 1.01, 0.96, 0.86, 0.76,
        // mach=1.8
        -0.46, -0.23, 0.00, 0.23, 0.46, 0.64, 0.80, 0.90, 0.95, 0.90, 0.80, 0.70,
        // mach=2.0
        -0.42, -0.21, 0.00, 0.21, 0.42, 0.58, 0.74, 0.84, 0.89, 0.84, 0.74, 0.64,
    };
    // clang-format on

    // CD table -- values are the *scaled* drag coefficient (already x1.5
    // as the legacy .dat loader would do). Note the transonic drag rise
    // around Mach 1.0.
    // clang-format off
    tbl.cdrag = {
        // mach=0.0
        0.030, 0.018, 0.015, 0.018, 0.030, 0.060, 0.110, 0.180, 0.270, 0.360, 0.450, 0.540,
        // mach=0.4
        0.030, 0.018, 0.015, 0.018, 0.030, 0.060, 0.110, 0.180, 0.270, 0.360, 0.450, 0.540,
        // mach=0.6
        0.032, 0.020, 0.016, 0.020, 0.032, 0.062, 0.112, 0.182, 0.272, 0.362, 0.452, 0.542,
        // mach=0.8
        0.036, 0.024, 0.020, 0.024, 0.036, 0.066, 0.116, 0.186, 0.276, 0.366, 0.456, 0.546,
        // mach=0.9
        0.050, 0.040, 0.036, 0.040, 0.052, 0.082, 0.132, 0.202, 0.292, 0.382, 0.472, 0.562,
        // mach=1.0 (peak drag rise)
        0.090, 0.080, 0.076, 0.080, 0.092, 0.122, 0.172, 0.242, 0.332, 0.422, 0.512, 0.602,
        // mach=1.1
        0.085, 0.075, 0.071, 0.075, 0.087, 0.117, 0.167, 0.237, 0.327, 0.417, 0.507, 0.597,
        // mach=1.2
        0.078, 0.068, 0.064, 0.068, 0.080, 0.110, 0.160, 0.230, 0.320, 0.410, 0.500, 0.590,
        // mach=1.4
        0.066, 0.056, 0.052, 0.056, 0.068, 0.098, 0.148, 0.218, 0.308, 0.398, 0.488, 0.578,
        // mach=1.6
        0.058, 0.048, 0.044, 0.048, 0.060, 0.090, 0.140, 0.210, 0.300, 0.390, 0.480, 0.570,
        // mach=1.8
        0.052, 0.042, 0.038, 0.042, 0.054, 0.084, 0.134, 0.204, 0.294, 0.384, 0.474, 0.564,
        // mach=2.0
        0.048, 0.038, 0.034, 0.038, 0.050, 0.080, 0.130, 0.200, 0.290, 0.380, 0.470, 0.560,
    };
    // clang-format on

    // CY table -- side force coefficient. The F-16 has relatively small side
    // force due to its slender fuselage; CY is near zero at low beta and
    // grows roughly linearly.
    // clang-format off
    tbl.cy = {
        // mach=0.0
        -1.20, -0.60, 0.00, 0.60, 1.20, 1.50, 1.70, 1.80, 1.85, 1.80, 1.70, 1.60,
        // mach=0.4
        -1.18, -0.59, 0.00, 0.59, 1.18, 1.48, 1.68, 1.78, 1.83, 1.78, 1.68, 1.58,
        // mach=0.6
        -1.16, -0.58, 0.00, 0.58, 1.16, 1.46, 1.66, 1.76, 1.81, 1.76, 1.66, 1.56,
        // mach=0.8
        -1.14, -0.57, 0.00, 0.57, 1.14, 1.44, 1.64, 1.74, 1.79, 1.74, 1.64, 1.54,
        // mach=0.9
        -1.10, -0.55, 0.00, 0.55, 1.10, 1.40, 1.60, 1.70, 1.75, 1.70, 1.60, 1.50,
        // mach=1.0
        -1.00, -0.50, 0.00, 0.50, 1.00, 1.30, 1.50, 1.60, 1.65, 1.60, 1.50, 1.40,
        // mach=1.1
        -0.94, -0.47, 0.00, 0.47, 0.94, 1.24, 1.44, 1.54, 1.59, 1.54, 1.44, 1.34,
        // mach=1.2
        -0.88, -0.44, 0.00, 0.44, 0.88, 1.18, 1.38, 1.48, 1.53, 1.48, 1.38, 1.28,
        // mach=1.4
        -0.80, -0.40, 0.00, 0.40, 0.80, 1.10, 1.30, 1.40, 1.45, 1.40, 1.30, 1.20,
        // mach=1.6
        -0.74, -0.37, 0.00, 0.37, 0.74, 1.04, 1.24, 1.34, 1.39, 1.34, 1.24, 1.14,
        // mach=1.8
        -0.70, -0.35, 0.00, 0.35, 0.70, 1.00, 1.20, 1.30, 1.35, 1.30, 1.20, 1.10,
        // mach=2.0
        -0.66, -0.33, 0.00, 0.33, 0.66, 0.96, 1.16, 1.26, 1.31, 1.26, 1.16, 1.06,
    };
    // clang-format on

    tbl.clFactor = 1.0;
    tbl.cdFactor = 1.0;
    tbl.cyFactor = 1.0;

    // -----------------------------------------------------------------------
    // Engine tables: F100-PW-220
    // Thrust in lbf at altitude (ft) x Mach.
    // -----------------------------------------------------------------------
    auto& e = cfg.engine;
    e.alt_ft = { 0.0, 5000.0, 10000.0, 15000.0, 20000.0, 25000.0, 30000.0,
                 35000.0, 40000.0, 45000.0, 50000.0 };
    e.mach  = { 0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0 };
    e.thrustFactor = 1.0;
    e.fuelFlowFactor = 1.0;

    // Sea-level static thrust values for the F100-PW-220:
    //   idle ~3,000 lbf
    //   mil  ~14,670 lbf
    //   AB   ~23,770 lbf
    // Thrust decays with altitude via the density ratio (rsigma). We add a
    // small "ram effect" mach boost.
    e.thrust_idle.clear();
    e.thrust_idle.reserve(e.alt_ft.size() * e.mach.size());
    e.thrust_mil.clear();
    e.thrust_mil.reserve(e.alt_ft.size() * e.mach.size());
    e.thrust_ab.clear();
    e.thrust_ab.reserve(e.alt_ft.size() * e.mach.size());
    e.fuelflow_idle.clear();
    e.fuelflow_idle.reserve(e.alt_ft.size() * e.mach.size());
    e.fuelflow_mil.clear();
    e.fuelflow_mil.reserve(e.alt_ft.size() * e.mach.size());
    e.fuelflow_ab.clear();
    e.fuelflow_ab.reserve(e.alt_ft.size() * e.mach.size());

    for (std::size_t ia = 0; ia < e.alt_ft.size(); ++ia) {
        const double alt = e.alt_ft[ia];
        double ttheta, rsigma;
        (void)ttheta;
        calcPressureRatio(alt, ttheta, rsigma);

        for (std::size_t im = 0; im < e.mach.size(); ++im) {
            const double m = e.mach[im];

            // Idle thrust
            double ramIdle = 1.0 + 0.10 * m - 0.02 * m * m;
            if (ramIdle < 0.4) ramIdle = 0.4;
            e.thrust_idle.push_back(3000.0 * std::pow(rsigma, 0.85) * ramIdle);

            // Mil thrust
            double ramMil = 1.0 + 0.25 * m - 0.06 * m * m;
            if (ramMil < 0.5) ramMil = 0.5;
            e.thrust_mil.push_back(14670.0 * std::pow(rsigma, 0.80) * ramMil);

            // AB thrust (drops faster with altitude)
            double ramAb = 1.0 + 0.30 * m - 0.05 * m * m;
            if (ramAb < 0.5) ramAb = 0.5;
            e.thrust_ab.push_back(23770.0 * std::pow(rsigma, 0.70) * ramAb);

            // Fuel flow (lb/hr) -- scales with sqrt(rho/rho0)
            const double altFF = std::pow(rsigma, 0.6);
            const double machFF = 1.0 + 0.1 * m;
            e.fuelflow_idle.push_back(1200.0  * altFF * machFF);
            e.fuelflow_mil .push_back(4800.0  * altFF * machFF);
            e.fuelflow_ab  .push_back(18000.0 * altFF * machFF);
        }
    }

    // -----------------------------------------------------------------------
    // Roll-rate command table: max roll rate (deg/s) as function of
    // alpha x qbar. The F-16 can sustain ~240 deg/s at low alpha / high Q,
    // falling off dramatically at high alpha.
    // -----------------------------------------------------------------------
    auto& r = cfg.rollCmd;
    r.alpha_deg = { 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0 };
    r.qbar      = { 0.0, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0 };
    r.scale     = 1.0;
    // clang-format off
    r.rollRate = {
        // alpha=0
          0.0,  60.0, 120.0, 200.0, 240.0, 240.0, 240.0,
        // alpha=5
          0.0,  60.0, 120.0, 200.0, 240.0, 240.0, 240.0,
        // alpha=10
          0.0,  55.0, 110.0, 180.0, 220.0, 230.0, 230.0,
        // alpha=15
          0.0,  45.0,  90.0, 150.0, 200.0, 220.0, 220.0,
        // alpha=20
          0.0,  35.0,  70.0, 120.0, 170.0, 190.0, 200.0,
        // alpha=25
          0.0,  25.0,  50.0,  90.0, 130.0, 150.0, 160.0,
        // alpha=30
          0.0,  15.0,  30.0,  60.0,  90.0, 110.0, 120.0,
    };
    // clang-format on

    // -----------------------------------------------------------------------
    // FCS limiters
    // -----------------------------------------------------------------------
    // Negative G limiter (LineLimiter on vcas): -3 G above 250 kts, -1 G below
    auto& negG = cfg.limiters[static_cast<int>(LimiterKey::NegGLimiter)];
    negG.type = LimiterType::Line;
    negG.x1 = 0.0;   negG.y1 = -1.0;
    negG.x2 = 250.0; negG.y2 = -3.0;

    // Positive G limiter: 9 G always
    auto& posG = cfg.limiters[static_cast<int>(LimiterKey::PosGLimiter)];
    posG.type = LimiterType::MinMax;
    posG.x1 = -3.0; posG.x2 = 9.0;

    // Roll-rate limiter (vs alpha): full authority below 15 deg, fading to
    // 0.4 at 25 deg.
    auto& rl = cfg.limiters[static_cast<int>(LimiterKey::RollRateLimiter)];
    rl.type = LimiterType::Line;
    rl.x1 = 0.0;  rl.y1 = 1.0;
    rl.x2 = 25.0; rl.y2 = 0.4;

    // Pitch/yaw control damper (vs qbar): full at qbar >= 25, ramping down
    // below
    auto& pyd = cfg.limiters[static_cast<int>(LimiterKey::PitchYawControlDamper)];
    pyd.type = LimiterType::Line;
    pyd.x1 = 0.0;  pyd.y1 = 0.4;
    pyd.x2 = 25.0; pyd.y2 = 1.0;

    auto& rld = cfg.limiters[static_cast<int>(LimiterKey::RollControlDamper)];
    rld.type = LimiterType::Line;
    rld.x1 = 0.0;  rld.y1 = 0.4;
    rld.x2 = 25.0; rld.y2 = 1.0;

    // AOA limiter: 25 deg max
    auto& aoaL = cfg.limiters[static_cast<int>(LimiterKey::AOALimiter)];
    aoaL.type = LimiterType::MinMax;
    aoaL.x1 = -5.0; aoaL.x2 = 25.0;

    cfg.aoaCommandMode = false;     // F-16 uses G-command mode by default
    cfg.aoaCommandMaxGs = 9.0;

    return cfg;
}

} // namespace f4flight::config
