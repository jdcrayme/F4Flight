// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aircraft_state.h
//
// Runtime state of an aircraft. Owned by the FlightModel. This is the
// equivalent of the legacy AirframeClass member variables, trimmed down to
// just the flight-model-relevant fields.
//
// All quantities are in Imperial units to match the original coefficient
// tables. Coordinate frames:
//   World: NED, Z-down, in feet (altitude = -z)
//   Body : X-forward, Y-right, Z-down
//   alpha / beta are stored in DEGREES (matches legacy convention)
//   sigma, gmma, mu, psi, theta, phi are stored in RADIANS
//   body rates p, q, r are in rad/s

#pragma once

#include "f4flight/core/math.h"
#include "f4flight/core/types.h"

#include <vector>

namespace f4flight {

// Pilot / AI input. All values in [-1, +1] except throttle which is in
// [0, 1.5] where 1.0 = MIL and 1.5 = full AB.
struct PilotInput {
    double pstick{0.0};     // pitch stick, -1 (full forward / nose down) .. +1 (full back / nose up)
    double rstick{0.0};     // roll stick,  -1 (full left) .. +1 (full right)
    double ypedal{0.0};     // rudder pedal, -1 (full left) .. +1 (full right)
    double throttle{0.0};   // 0..1.5 (1.0 = MIL, 1.5 = full AB)
    double speedBrake{0.0}; // -1 (retract) .. +1 (extend)
    double gearHandle{1.0}; // -1 (up) .. +1 (down) -- default down
    double hookHandle{0.0}; // -1 (up) .. +1 (down)

    bool   wheelBrakes{false};
    bool   parkingBrake{false};
    bool   noseSteerOn{true};
    bool   refueling{false};

    // --- Weapon fire commands (set by digi AI, read by host) ---
    // Port of FreeFalcon BaseBrain::GunFireFlag / MslFireFlag
    // (simbrain.h:17-21). Cleared at the top of each compute() frame.
    bool   fireGun{false};         // fire the internal gun
    bool   releaseConsent{false};  // release weapon (missile/bomb)
    int    weaponStation{0};       // which hardpoint to release from (0 = auto)
};

// Position + orientation in world (NED, Z-down) frame.
struct KinematicState {
    // Position (ft) -- altitude = -z
    double x{0.0}, y{0.0}, z{0.0};
    // Velocity (ft/s) along world axes
    double xdot{0.0}, ydot{0.0}, zdot{0.0};

    // Quaternion (body-to-world). scalar-first Hamilton.
    Quaternion quat;

    // Body angular rates (rad/s)
    double p{0.0}, q{0.0}, r{0.0};

    // Euler angles (radians), recovered from the quaternion
    double sigma{0.0};   // velocity-vector heading
    double gmma{0.0};    // flight path angle
    double mu{0.0};      // velocity-vector roll
    double psi{0.0};     // body yaw
    double theta{0.0};   // body pitch
    double phi{0.0};     // body roll

    // Trigonometry cache (computed every frame by Trigenometry())
    double sinalp{0.0}, cosalp{1.0};
    double sinbet{0.0}, cosbet{1.0};
    double singam{0.0}, cosgam{1.0};
    double sinsig{0.0}, cossig{1.0};
    double sinmu{0.0},  cosmu{1.0};
    double sinthe{0.0}, costhe{1.0};
    double sinphi{0.0}, cosphi{1.0};
    double sinpsi{0.0}, cospsi{1.0};

    // Body-to-world DCM (3x3)
    Matrix3 dcm;

    // True airspeed (ft/s)
    double vt{0.0};
};

// Aerodynamic state.
struct AeroState {
    double alpha_deg{0.0};    // angle of attack, degrees
    double beta_deg{0.0};     // sideslip angle, degrees
    double alpha_dot{0.0};    // alpha rate, deg/s
    double beta_dot{0.0};     // beta rate, deg/s

    // Coefficients (dimensionless)
    double cl{0.0}, cd{0.0}, cy{0.0};
    double clalpha{0.0};      // dCL/dalpha (per radian)
    double clalph0{0.0};      // static slope at alpha=0..10deg
    double clift0{0.0};       // CL at alpha=0
    double cnalpha{0.0};      // normal-force slope

    // Forces (accelerations, ft/s^2) in body axes
    double xaero{0.0}, yaero{0.0}, zaero{0.0};
    // Stability axes
    double xsaero{0.0}, ysaero{0.0}, zsaero{0.0};
    // Wind axes
    double xwaero{0.0}, ywaero{0.0}, zwaero{0.0};

    // Lift / drag (ft/s^2, i.e. force / mass)
    double lift{0.0}, drag{0.0};

    // Surface positions
    double tefPos{0.0};        // trailing edge flap, normalized 0..1
    double lefPos{0.0};        // leading edge flap, normalized 0..1
    double dbrake{0.0};        // speed brake, 0..1
    double gearPos{1.0};       // gear, 0 (up) .. 1 (down)
    double hookPos{0.0};       // hook, 0..1
    double dragChutePos{0.0};  // 0..1

    // Trim
    double aoabias{0.0};

    // Stall
    bool   stalled{false};
    double stallSpeed{0.0};    // kcas

    // Stores drag (incremental CD from external stores)
    double cdStores{0.0};
};

// Engine state.
struct EngineState {
    double rpm{0.0};           // 0..1+ (1 = MIL, >1 = AB)
    double rpmCmd{0.0};
    double thrust{0.0};        // thrust acceleration (ft/s^2) = thrust_lbf / mass
    double thrust2{0.0};       // second engine (if present)
    double fuelFlow{0.0};      // lb/hr
    double fuelFlow2{0.0};
    double ftit{0.0};          // 0..10 normalized
    double nozzlePos{0.0};     // 0..1
    bool   aburnLit{false};
    bool   engLit{true};       // engine running
    bool   flameout{false};

    // Spool filter state
    LagFilter rpmLag;
    bool   rpmLagInitialized{false};
};

// Fuel state.
struct FuelState {
    double fuel_lbs{0.0};
    double externalFuel_lbs{0.0};
    double epuFuel_lbs{0.0};
    double weight_lbs{0.0};    // gross weight (empty + fuel + stores)
    double mass_slugs{0.0};    // mass = weight / g
    double emptyWeight_lbs{0.0};
    double loadingFraction{1.0}; // weight / emptyWeight
};

// Flight-control-system state (filters + integrators).
struct FcsState {
    // Pitch
    LagFilter pitchRateLag;     // for q from qptchc
    AdamsBash2 pitchIntegral;   // NZ error integrator
    LeadLagFilter pitchAlphaLag; // F7Tust lead-lag (tau1=tp01, tau2=tp02, tau3=tp03)
    double oldp02[6]{};
    double oldp03[6]{};
    double kp01{1.0}, kp02{1.0}, kp03{2.0}, kp05{1.0};
    double tp01{0.2}, tp02{0.2}, tp03{0.2};
    double zp01{0.9};
    double pshape{0.0};
    double ptcmd{0.0};          // commanded pitch (alpha or G)
    double aoacmd{0.0};
    bool   aoaCmdModeRuntime{false};  // set by computeGains, read by runPitch
                                       // (matches FreeFalcon AOACmdMode flag)

    // Roll
    LagFilter  rollRateLag;
    double kr01{1.0}, kr02{1.0};
    double tr01{0.25};
    double rshape{0.0};
    double pscmd{0.0};          // commanded roll rate, rad/s
    double pstab{0.0};          // filtered roll rate
    // FreeFalcon airframe.h:596 — used by Roll() / RollIt() to limit pscmd
    // when |phi| > maxRoll, and to scale pscmd by (1 - startRoll/maxRollDelta)
    // as the aircraft approaches the target bank. Set by the steering layer
    // (HeadingAndAltitudeHold, LevelTurn, etc.) via SetMaxRoll / SetMaxRollDelta.
    double maxRoll{80.0};       // deg — roll limit (FreeFalcon af->maxRoll)
    double maxRollDelta{5.0};   // deg — roll-rate damping window (af->maxRollDelta)
    double startRoll{0.0};      // rad — integrated roll (af->startRoll, eom.cpp:810)

    // Yaw
    LagFilter  yawBetaLag;
    AdamsBash2 yawIntegral;
    double ky02{1.0}, ky03{2.0}, ky05{1.0};
    double ty02{0.3};
    double yshape{0.0};
    double betcmd{0.0};

    // Damper gains
    double plsdamp{1.0}, rlsdamp{1.0}, ylsdamp{1.0};

    // Stall mode (0 none, 1 entering deep stall, 2 deep stall, 3 recovering,
    // 4 spinning, 5 flat spin)
    int stallMode{0};
};

// Load-factor state (computed by Accelerometers).
struct LoadFactorState {
    double nxcgb{0.0}, nycgb{0.0}, nzcgb{0.0}; // body
    double nxcgs{0.0}, nycgs{0.0}, nzcgs{0.0}; // stability
    double nxcgw{0.0}, nycgw{0.0}, nzcgw{0.0}; // wind
};

// Gear / ground state.
struct GearState {
    struct Wheel {
        double strutCompression_ft{0.0}; // current compression
        double strutVel_fps{0.0};        // compression rate
        double wheelAngle_rad{0.0};      // visual rotation
        bool   onGround{false};
        bool   broken{false};
        bool   stuck{false};
    };
    std::vector<Wheel> wheels;           // sized to AircraftConfig.gear.size()
    bool   inAir{true};
    bool   planted{false};               // stationary on ground
    double groundZ_ft{0.0};              // terrain altitude at aircraft position
    Vec3   groundNormal{0.0, 0.0, 1.0};  // up vector
    double muFric{0.04};                 // current friction coefficient
    double minHeight_ft{0.0};            // minimum body clearance
    double nwsAngle_rad{0.0};            // nose-wheel steering angle
    bool   onObject{false};              // carrier deck / hard surface
    bool   overRunway{true};
};

// All state combined. Owned by FlightModel.
struct AircraftState {
    KinematicState    kin;
    AeroState         aero;
    EngineState       engine;
    EngineState       engine2;
    FuelState         fuel;
    FcsState          fcs;
    LoadFactorState   loads;
    GearState         gear;

    // Atmospheric outputs at current altitude/airspeed
    double rho{0.0};
    double pa{0.0};
    double mach{0.0};
    double qbar{0.0};
    double qsom{0.0};
    double qovt{0.0};
    double vcas{0.0};
    double sound{0.0};

    // Wind (world frame, ft/s)
    double windX{0.0}, windY{0.0};

    // Misc flags
    bool   simplified{false};     // use simple model (for AI)
    bool   trimming{false};
    double netAccel{0.0};         // last frame's net accel (ft/s^2)
    double vtDot{0.0};            // true airspeed rate (ft/s^2) — set by EOM
    double vRot{0.0};             // rotation speed

    // -----------------------------------------------------------------------
    // Reset every field to its default-constructed value.
    //
    // This is equivalent to `*this = AircraftState{}` but:
    //   - It is a named method, so the intent is explicit at the call site
    //     (the value-init idiom is easy to misread as "construct a temporary").
    //   - It clears the gear.wheels vector to size 0, which is the same as
    //     what value-init does. The FlightModel::init() path re-sizes the
    //     wheel vector via GearModel::init() immediately after reset, so this
    //     is safe.
    //   - It does NOT touch the EngineState lag filters' "initialized" flag
    //     differently from value-init (both leave it false).
    //
    // Hosts that want to "rewind" a simulation to a clean state should call
    // this and then call FlightModel::init() again with the desired initial
    // conditions.
    // -----------------------------------------------------------------------
    void reset() noexcept {
        *this = AircraftState{};
    }
};

} // namespace f4flight
