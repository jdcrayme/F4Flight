# f4flight

A modern, self-contained C++17 flight-model library extracted from the
**Falcon 4 / FreeFalcon** combat flight simulator source code.

The library preserves the high-fidelity aerodynamics, propulsion, atmosphere,
and 6-DOF equations-of-motion of the original Falcon 4 flight model, but
refactors the legacy god-object `AirframeClass` (~12,000 lines across 21 files)
into a clean, modular, dependency-free C++17 library that can be dropped into
any game or simulation project.

## Current Status

**Version 2.2.0** — the library is functional and tested but has known
limitations (see below). It is suitable for integration into game projects
as a flight-model and AI-steering backend.

### What Works

- **Flight model** — 6-DOF rigid-body simulation with quaternion integration,
  3-layer standard atmosphere, 2-D Mach × alpha aerodynamic coefficient
  tables, F-16 FLCS-style G-command flight control system, per-engine-type
  RPM schedules (PW-100/220/229, GE-110/129), landing gear, and fuel burn.
- **Aircraft data loading** — parses all 182 BMS `.dat` aircraft files via
  the `dat2json` converter tool. The JSON format is version-independent and
  extensible.
- **AI steering / autopilot** — heading/altitude/speed hold, waypoint
  following, formation flying, terrain following, ILS-style approach, and
  orbit patterns. Uses PID controllers with VVI-capped level-off for smooth
  altitude transitions.
- **Multi-aircraft validation** — tested across fighters (F-16, F-15, F/A-18,
  F-14, Su-27, MiG-29, Rafale, EF2000), attack (A-10), bombers (B-52),
  transports (C-130), and others. Level-flight, turn, orbit, acceleration,
  and deceleration maneuvers pass ±100 ft / ±10 kts for most fighter and
  attack aircraft.

### Known Limitations

1. **Climb phase speed control** — during climbs at MIL power, some aircraft
   accelerate beyond the target climb speed because the FCS speed-protection
   logic is not aggressive enough. The F-16 climb overshoots altitude by
   ~130 ft (target: <100 ft).

2. **Heavy aircraft (B-52, C-130)** — these have very low thrust-to-weight
   ratios and cannot maintain the default cruise speeds at altitude. They
   need lower cruise speeds (near their min Vcas) and lower starting
   altitudes. The category profiles need further tuning.

3. **Some fighter .dat files** — a few aircraft (F-15, F-14, MiG-29,
   Mirage 2000, EF2000) have difficulty maintaining 420 kts at 15,000 ft.
   This may be a thrust-to-weight issue or a drag-model discrepancy.

4. **No moment coefficient tables** — the original Falcon 4 model uses
   CL/CD/CY tables plus FCS closed-loop feedback to synthesize pitch/roll/yaw
   moments, rather than separate Cm/Cl/Cn tables. This is a simplification
   that may not capture all aircraft dynamics.

5. **Forward Euler integration** — the rigid-body state is integrated with
   forward Euler, which is less stable than RK4 at large time steps. The
   model sub-steps (6 minor frames per major frame) to mitigate this.

6. **No combat steering** — the `Combat` steering mode is a placeholder.
   Target tracking, intercept geometry, and weapon-envelope management are
   future work.

7. **No multi-engine support** — the engine model supports a single engine.
   Multi-engine aircraft (F-15, F/A-18, B-52) use a single combined thrust
   table.

## Key Features

- **High-fidelity physics** — 3-layer standard atmosphere, 2-D Mach × alpha
  coefficient tables, F-16 FLCS-style G-command FCS, quaternion-based 6-DOF
  equations of motion, VVI-capped level-off with power-law altitude error.
- **AI steering / autopilot** — PID-based heading/altitude/speed hold with
  turn compensation (1/cos(bank)), waypoint following, formation flying,
  terrain following, and orbit patterns. Climb/descent uses the standard
  aviation procedure: throttle for altitude, pitch for speed.
- **Falcon 4 .dat file loader** — convert real BMS aircraft data to JSON
  with the `dat2json` tool, then load at runtime. All 182 BMS aircraft
  files parse successfully.
- **Zero external dependencies** — only the C++17 standard library.
- **Compiled static/shared library** — clean public API in `f4flight/`.
- **126 GoogleTest unit tests** — all passing.
- **CMake build** — generates Visual Studio 2022 projects (and works with
  GCC/Clang). GoogleTest is fetched automatically via `FetchContent`.

## Quick Start

### Building with CMake (Visual Studio 2022)

```bat
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

### Building with CMake (GCC / Clang)

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Converting BMS aircraft data

```bash
# Convert a .dat file to JSON
dat2json f16bk50.dat f16bk50.json

# Run a simulation
simple_sim f16bk50.json

# Run the maneuver test suite
maneuver_test f16bk50.json
```

### Minimal usage example

```cpp
#include <f4flight/f4flight.h>

int main() {
    using namespace f4flight;

    // Load an aircraft
    AircraftConfig cfg;
    json::readFile("f16bk50.json", cfg);

    // Create and initialise the flight model
    FlightModel fm;
    fm.init(cfg, 15000.0, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);

    // Set up the autopilot
    SteeringController sc;
    sc.setMode(SteeringMode::HeadingAltitude);
    sc.setMaxGs(cfg.geometry.maxGs);

    SteeringGoal goal;
    goal.hasHeading = true;  goal.heading_rad = 0.0;
    goal.hasAltitude = true; goal.altitude_ft = 15000.0;
    goal.hasSpeed = true;    goal.speed_kts = 420.0;
    goal.climbVcas_kts = 350.0;   // slower than cruise
    goal.descentVcas_kts = 460.0; // faster than cruise
    goal.climbMach = 0.80;
    goal.climbPower = 1.0;        // MIL
    goal.descentPower = 0.05;     // near-idle
    goal.levelBand_ft = 200.0;
    sc.setGoal(goal);

    // Simulation loop
    const double dt = 1.0 / 60.0;
    while (running) {
        PilotInput input = sc.compute(fm.state(), dt, groundZ);
        fm.update(dt, input, groundZ, groundNormal);
        // Read state for rendering / AI...
    }
}
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    FlightModel                               │
│  (orchestrator — runs the per-frame update loop)             │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│  Atmos   │   Aero   │  Engine  │   FCS    │      EOM        │
│ (header) │  (forces)│ (thrust) │ (limits) │  (integrate)    │
└──────────┴──────────┴──────────┴──────────┴─────────────────┘
       │         │          │          │           │
       └─────────┴──────────┴──────────┴───────────┘
                         │
                   AircraftState
              (shared mutable state)

┌─────────────────────────────────────────────────────────────┐
│               SteeringController                             │
│  (autopilot — translates goals into PilotInput)              │
├──────────────────────────────────────────────────────────────┤
│  WaypointFollower │ HeadingAltitude │ Formation │ TerrainFollow│
├──────────────────────────────────────────────────────────────┤
│  PID controllers (pitch, roll, throttle, yaw, speed)         │
│  VVI cap: maxVVI = K × |altErr|^P                            │
└──────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│               Data Pipeline                                  │
│  .dat file → dat_loader → JSON → json_io → AircraftConfig    │
└─────────────────────────────────────────────────────────────┘
```

| Module | Header | Source | Legacy equivalent |
|--------|--------|--------|-------------------|
| Math primitives | `core/types.h`, `core/math.h` | — | `vector.h`, `simmath.h` |
| Lookup tables | `core/lookup.h` | — | `lookuptable.h` |
| Atmosphere | `atmosphere.h` | — (header-only) | `atmos.cpp` |
| Aerodynamics | `aerodynamics.h` | `aerodynamics.cpp` | `aero.cpp` |
| Engine | `engine.h` | `engine.cpp` | `engine.cpp` |
| Gear | `gear.h` | `gear.cpp` | `gear.cpp` |
| Flight control | `fcs.h` | `fcs.cpp` | `fcs.cpp`, `gain.cpp`, `pitch.cpp`, `roll.cpp`, `yaw.cpp` |
| Equations of motion | `eom.h` | `eom.cpp` | `eom.cpp` |
| Aircraft config | `aircraft_config.h` | `aircraft_config.cpp` | `readin.cpp`, `arfrmdat.h` |
| Aircraft state | `aircraft_state.h` | — | `AirframeClass` members |
| Flight model | `flight_model.h` | `flight_model.cpp` | `airframe.cpp` |
| AI steering | `steering.h` | `steering.cpp` | `sim/digi/` |
| .dat loader | `dat_loader.h` | `dat_loader.cpp` | `readin.cpp` |
| JSON I/O | `json_io.h` | `json_io.cpp` | (new) |
| F-16C config | `config/f16c_config.h` | `config/f16c_config.cpp` | (new) |

## Coordinate Frames and Units

The library uses **Imperial units** throughout to match the original Falcon 4
coefficient tables:

| Quantity | Unit |
|----------|------|
| Length | feet (ft) |
| Mass | slugs |
| Force | pounds-force (lbf) |
| Pressure | lb/ft² |
| Density | slugs/ft³ |
| Speed | ft/s (true airspeed), knots (calibrated) |
| Temperature | degrees Rankine (°R) |
| Angles | degrees (alpha, beta), radians (euler, body rates) |

Coordinate frames:
- **World**: NED (North-East-Down), Z-down. Altitude = `-z`.
- **Body**: X-forward, Y-right, Z-down.
- **Stability**: X-along-velocity-projected-into-body-XY.
- **Wind**: X-along-velocity.

## Tools

| Tool | Purpose |
|------|---------|
| `dat2json` | Convert Falcon 4 `.dat` files to JSON |
| `simple_sim` | Run a simple 30-second simulation |
| `steering_demo` | AI-piloted waypoint circuit |
| `maneuver_test` | Full maneuver profile with capture/overshoot reporting |
| `tune_diag` | Detailed frame-by-frame diagnostic output |
| `perf_check` | Performance validation against published specs |

## Testing

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

126 unit tests covering math, atmosphere, aerodynamics, engine, gear, FCS,
EOM, steering, .dat loading, and JSON I/O.

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `F4FLIGHT_BUILD_TESTS` | `ON` | Build the unit tests |
| `F4FLIGHT_BUILD_EXAMPLES` | `ON` | Build example programs and tools |
| `F4FLIGHT_BUILD_SHARED` | `OFF` | Build as a shared library (DLL) |
| `F4FLIGHT_ENABLE_WERROR` | `OFF` | Treat warnings as errors |
| `F4FLIGHT_USE_MSVC_UTF8` | `ON` | Force `/utf-8` on MSVC |

## Consuming the Library

```cmake
add_subdirectory(f4flight)
target_link_libraries(my_game PRIVATE f4flight::f4flight)
```

Or after installation:

```cmake
find_package(f4flight REQUIRED)
target_link_libraries(my_game PRIVATE f4flight::f4flight)
```

## Adding a Custom Aircraft

### Option 1: Load from a Falcon 4 .dat file (recommended)

```bash
dat2json f16bk50.dat f16bk50.json
```

```cpp
AircraftConfig cfg;
f4flight::json::readFile("f16bk50.json", cfg);
```

### Option 2: Build an AircraftConfig in code

See `src/config/f16c_config.cpp` for a complete worked example.

## Aircraft Category Profiles

The maneuver test categorizes aircraft and selects appropriate speeds:

| Category | Cruise kts | Climb kts | Descent kts | Max bank |
|----------|-----------|-----------|-------------|---------|
| Fighter | 420 | 350 | 460 | 45° |
| Attack | 280 | 230 | 320 | 30° |
| Bomber | 300 | 250 | 340 | 30° |
| Transport | 250 | 210 | 290 | 25° |
| Interceptor | 450 | 380 | 500 | 45° |

The convention is: **climb speed < cruise speed < descent speed**, matching
real aviation practice (slow climb for best angle, fast descent for efficiency).

## Steering Controller Design

### Level Flight
- **Pitch** for altitude (with turn compensation: G = 1/cos(bank))
- **Throttle** for speed

### Climb
- **Throttle** = climbPower, **reduced by VVI cap** as climb rate exceeds
  `K × |altErr|^P`. This bleeds excess thrust to control the climb rate
  without causing speed runaway.
- **Pitch** for speed (hold climb speed schedule: vcas until Mach transition,
  then Mach). G stays at a moderate climb setting (~1.15).

### Descent
- **Throttle** chases **cruise speed** (keeps engine spooled for level-off).
- **Pitch** chases descent speed + VVI cap. As the aircraft approaches target
  altitude, the VVI cap increases pitch (toward 1.0 G), which slows the
  descent. As pitch increases, speed drops, and the throttle adds power.

### VVI Cap
```
maxVVI = 10.0 × |altErr|^0.7  (fpm)
```
- At 5000 ft from target: maxVVI = 7,740 fpm (full climb allowed)
- At 500 ft: maxVVI = 1,580 fpm
- At 50 ft: maxVVI = 324 fpm
- At 5 ft: maxVVI = 65 fpm

The power < 1 means the cap is lenient far from target (allowing high climb
rates) but tight near the target (forcing gentle level-off).

## Roadmap

### Short Term
- Fix climb-phase speed control (the aircraft accelerates beyond the climb
  target speed at MIL power)
- Tune category profiles for heavy aircraft (B-52, C-130) and fighters that
  can't maintain 420 kts at 15,000 ft
- Add aircraft-specific maneuver profile data to the JSON format

### Medium Term
- Implement combat steering (target tracking, intercept geometry, weapon
  envelopes)
- Add multi-engine support (separate thrust tables per engine, asymmetric
  thrust yaw coupling)
- Add a proper trim solver (iterative alpha + throttle for steady-state)
- Add RK4 integration option for better numerical stability

### Long Term
- Add separate moment coefficient tables (Cm, Cl, Cn) for more accurate
  rotational dynamics
- Add weather effects (wind shear, turbulence, icing)
- Add stores drag and weight modelling (external weapons, fuel tanks)
- Add carrier landing mode (ACLs, arresting gear)

## Acknowledgements

- **FreeFalcon team** — for maintaining the open-source Falcon 4 derivative
- **Spectrum HoloByte / MicroProse** — the original Falcon 4.0 development team
- **NASA** — for the public-domain F-16 wind-tunnel data (TP-1538)
- **BMS team** — for the aircraft data files used for validation

## License

BSD 2-Clause License (same as the original FreeFalcon source code).
See `LICENSE.md` for details.
