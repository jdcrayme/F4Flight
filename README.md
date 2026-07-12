# f4flight

A modern, self-contained C++17 flight-model library extracted from the
**Falcon 4 / FreeFalcon** combat flight simulator source code.

The library preserves the high-fidelity aerodynamics, propulsion, atmosphere,
and 6-DOF equations-of-motion of the original Falcon 4 flight model, but
refactors the legacy god-object `AirframeClass` (~12,000 lines across 21 files)
into a clean, modular, dependency-free C++17 library that can be dropped into
any game or simulation project.

## Current Status

**Version 3.3.0** — the library is functional and tested but has known
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
- **Strongly typed units (opt-in)** — `core/units.h` provides `Quantity<Tag>`
  wrappers (`Radians`, `Degrees`, `Feet`, `Knots`, `Slugs`, ...) that make
  unit-mixing bugs a compile error. Existing code is untouched; new code can
  adopt the typed aliases incrementally.
- **Configuration validation** — `AircraftConfig::validate()` returns a
  report of every problem found (empty tables, dimension mismatches, NaN
  values, backwards limits, ...) so a host loading a data file gets a
  complete diagnostic in one pass.
- **Clean state reset** — `AircraftState::reset()` replaces the fragile
  `state = AircraftState{}` value-init idiom with a named method.

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
- **158 GoogleTest unit tests** — all passing (125 original + 33 covering the
  new units, validation, reset, trig-cache, and limiter-accessor facilities).
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

### Converting and validating BMS aircraft data

```bash
# Convert a .dat file to JSON
dat2json f16bk50.dat f16bk50.json

# Validate the result
 dat_validate f16bk50.json

# Run the maneuver test suite
maneuver_test f16bk50.json
```

See `tools/README.md` for the full tool set (`dat2json`, `dat_validate`,
`json_diff`).

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

## Strongly Typed Units (opt-in)

The library's existing API uses raw `double` for every physical quantity and
relies on naming conventions (`alpha_deg`, `vt_ftps`, `alt_ft`) to distinguish
units. This works but is fragile — nothing prevents
`std::sin(state.aero.alpha_deg)` where `sin` expects radians.

`core/units.h` provides `Quantity<Tag>` strong-type wrappers that make such
mistakes a **compile error**. The wrappers are ABI-compatible with `double`
(a single `double` field) and zero-cost at runtime. Existing code is
untouched; new code can adopt the typed aliases incrementally.

```cpp
#include <f4flight/core/units.h>
using namespace f4flight;

Radians alpha_rad = toRadians(Degrees(state.aero.alpha_deg));
Feet    alt       = feet(15000.0);
Knots   cruise    = knots(420.0);
FeetPerSec vt     = toFeetPerSec(cruise);

double mach = vt / toFeetPerSec(knots(AASLK));   // ratio -> dimensionless double

// Mixing tags is a compile error:
// Radians r(1.0); Degrees d(90.0); r + d;   // ERROR
```

Design mirrors `std::chrono::duration`:
- Construction from `double` is **explicit** (no implicit `double -> Quantity`)
- Conversion back to `double` is via `count()` or `static_cast<double>` (no
  implicit `Quantity -> double` that would silently drop the unit)
- Arithmetic between same-tag quantities returns the same tag
- Division of two same-tag quantities yields a plain `double` (dimensionless ratio)
- Cross-unit conversion requires an explicit free function (`toRadians`,
  `toKnots`, `toFeet`, ...) so the conversion is visible at the call site

Provided aliases: `Radians`, `Degrees`, `Feet`, `Meters`, `FeetPerSec`,
`Knots`, `Seconds`, `Slugs`, `PoundsMass`, `PoundsForce`, `LbPerFt2`,
`SlugsPerFt3`, `AreaFt2`, `FtPerSec2`, `Rankine`, `LbPerHour`. Lowercase
factory functions (`radians()`, `feet()`, `knots()`, ...) make call sites
self-documenting.

## Configuration Validation

`AircraftConfig::validate()` returns a `ConfigValidationReport` listing every
problem found (it does not short-circuit on the first error, so a host loading
a malformed data file gets a complete diagnostic in one pass):

```cpp
AircraftConfig cfg;
f4flight::json::readFile("f16bk50.json", cfg);

auto report = cfg.validate();
if (!report.ok()) {
    std::cerr << report.format();   // multi-line "E: [field] message" listing
    return 1;
}
```

Checks performed:
- Aero tables non-empty and dimensionally consistent (`mach.size() * alpha.size() == clift.size()`)
- Engine thrust tables non-empty and dimensionally consistent
- Roll-command table dimensions (if present)
- Geometry: positive area, weight, span; non-negative fuel, length
- AOA/beta limits sane (`min < 0 < max`, max not absurd)
- Performance envelope: `maxGs`, `maxRoll`, `minVcas < maxVcas`, etc.
- No NaN/Inf in critical scalar fields (sampled for large tables)
- Gear points: finite coordinates, non-negative strut range

## Other API Additions

- **`AircraftState::reset()`** — named re-initialization replacing the fragile
  `state = AircraftState{}` value-init idiom. Equivalent behavior, clearer intent.
- **`AircraftConfig::limiter(LimiterKey)` / `setLimiter(LimiterKey, Limiter)`** —
  typed accessor methods for the limiter array. Preferred over raw
  `limiters[static_cast<int>(key)]` indexing. The legacy `limiters[]` array
  remains public so existing code keeps compiling.
- **`recomputeKinematicTrig(KinematicState&, alpha_deg, beta_deg)`** — shared
  helper in `core/trig.h` that fills all 16 sin/cos fields and the
  velocity-vector euler angles (sigma, gamma, mu). Both `FlightModel::init()`
  and `EquationsOfMotion::trigonometry()` now call this helper, eliminating the
  drift between the init path (which used to compute a partial subset) and the
  per-frame update path (which computed all of it).

## Tools

See `tools/README.md` for full documentation. Summary:

| Tool | Purpose |
|------|---------|
| `dat2json` | Convert Falcon 4 `.dat` files to JSON |
| `dat_validate` | Load a JSON and run `AircraftConfig::validate()` |
| `json_diff` | Field-by-field diff of two JSON aircraft files |

(`maneuver_test` is built from `tests/maneuver/` and lives with the test
tree, not `tools/`.)

## Maneuver Test Scenarios

`maneuver_test` is a scenario-based test runner. Each scenario is a
self-contained subclass of `ManeuverScenario` that builds an ordered list of
test phases (climb, turn, waypoint leg, etc.). Scenarios self-register at
startup, so adding a new scenario is a matter of dropping a new `.cpp` file
in `tests/maneuver/scenarios/` and rebuilding — the runner never needs editing.

### Built-in Scenarios

| Scenario | Description | Status |
|----------|-------------|--------|
| `basic` | Level flight, climb, descent, turn, orbit, accelerate, decelerate | Complete (default) |
| `flightplan` | 4-waypoint square circuit using `SteerToWaypoint` | Complete |
| `approach` | Precision approach: GS intercept + 3-degree glideslope | Scaffold (full ILS coupling is future work) |
| `combat` | ACM and weapons delivery | Scaffold (target tracking, weapon envelopes are future work) |

### Usage

```bash
# List available scenarios
maneuver_test f16bk50.json --list

# Run all scenarios (default)
maneuver_test f16bk50.json

# Run a specific scenario
maneuver_test f16bk50.json --scenario basic

# Run several scenarios back-to-back
maneuver_test f16bk50.json --scenario basic --scenario flightplan
```

### Adding a New Scenario

1. Create `tests/maneuver/scenarios/scenario_<name>.cpp`.
2. Define a `ManeuverScenario` subclass that builds its test sequence in
   `StartScenario()`.
3. Self-register with `static RegisterScenario reg("name", []{ ... });`.
4. Add the new `.cpp` to `F4FLIGHT_MANEUVER_TEST_SOURCES` in `CMakeLists.txt`.

See `tests/maneuver/scenarios/scenario_basic.cpp` for a complete worked
example. The `flightplan` and `combat` scenarios show how to build on top
of the framework with custom `ManeuverTest` subclasses.

### Test fixtures

The maneuver tests run against the curated JSON set in `tests/fixtures/`.
The set is intentionally small but representative:

| Aircraft | Category |
|----------|----------|
| `f16bk50` | Fighter (F-16, the primary reference) |
| `f15c` | Twin-engine fighter |
| `f18c` | Carrier fighter |
| `f14b` | Variable-sweep wing fighter |
| `mig29a`, `su27` | Russian fighters |
| `ef2000`, `rafalec`, `mirage2k5` | European delta-wing fighters |
| `a10a` | Attack (low TWR) |
| `b52h` | Bomber (heavy, multi-engine) |
| `c130` | Transport |
| `sr71` | Interceptor (high speed) |

Adding a JSON to `tests/fixtures/` automatically registers it for every
maneuver scenario (see `file(GLOB F4FLIGHT_FIXTURES ...)` in
`CMakeLists.txt`) — no need to edit any file.

## Testing

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

The test tree is split into:

- **`tests/unit/`** — gtest unit tests for each library module (math,
  atmosphere, aerodynamics, engine, FCS, EOM, JSON I/O, .dat loading,
  units, validation, trig cache, limiter accessors). 170 tests, all
  passing. Quick correctness checks, no simulation.

- **`tests/maneuver/`** — scenario-based integration tests that drive the
  full `FlightModel` + `SteeringController` through multi-phase maneuvers
  (climb, turn, orbit, waypoint circuit, ...). Each scenario
  self-registers; the runner picks up scenarios by name.

- **`tests/fixtures/`** — curated JSON aircraft files used by the maneuver
  tests. 13 aircraft covering fighters, attack, bomber, transport, and
  interceptor categories.

For a quick build without cmake (uses a generated GoogleTest stub), see
`scripts/build_and_run_tests.sh` in the project root.

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `F4FLIGHT_BUILD_TESTS` | `ON` | Build the unit + maneuver tests |
| `F4FLIGHT_BUILD_TOOLS` | `ON` | Build `dat2json`, `dat_validate`, `json_diff` |
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
