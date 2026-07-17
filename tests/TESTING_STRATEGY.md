# F4Flight Testing Strategy & Action Plan

This document defines the architecture, organization, and action plan for clean and well-organized testing in the F4Flight repository. It serves as a guide for maintainers, developers, and AI agents to keep our test framework clean, high-signal, and tightly aligned with the capabilities of the FreeFalcon digi AI.

---

## 1. Test Suite Architecture & Categories

The test suite is organized into three distinct categories to separate concerns and ensure tests run at the correct level of abstraction:

### A. Unit Tests (GoogleTest)
- **Files Location**: `tests/flight/unit/`, `tests/digi/unit/`
- **Scope**: Lower-level, stateless, or deterministic algorithms (e.g., math helpers, coordinate conversions, atmosphere lookups, flight control limits, visual/RWR sensor models, and formation file parsers).
- **Goal**: Absolute correctness, fast execution (<5 ms per test), and zero flakiness.

### B. Flight Scenario Tests
- **Files Location**: `tests/flight/scenarios/`
- **Scope**: Testing physical flight-model characteristics and low-level control laws without AI dependency.
- **Approach**: Uses `inputOverride()` to bypass the steering controller and feed raw/direct pilot inputs (e.g., G-limit steps, roll commands).
- **Goal**: Confirm flight physics and control laws perform according to aerodynamic limits.

### C. Digi AI Scenario Tests (Integration Tests)
- **Files Location**: `tests/digi/scenarios/`
- **Scope**: Closed-loop testing of the Digi AI brain coupled with the Flight Model.
- **Approach**: Exercises the `SteeringController` and `DigiBrain` over a prolonged period (e.g., 20–150 seconds) in high-fidelity missions (e.g., Takeoff, Landing, Formations, Intercepts, Air-to-Air Refueling).
- **Goal**: Replicate FreeFalcon digi AI behaviors, tracking mode transitions, and ensuring stable maneuvers.

---

## 2. Test Execution Tiers

To keep testing fast and focused, we establish a three-tiered execution strategy:

| Tier | Name | Target Target(s) | Description | Execution Frequency |
|---|---|---|---|---|
| **Tier 1** | **Fast Unit Tests** | `f4flight_flight_tests`, `f4flight_digi_tests` | Pure GoogleTest binaries covering mathematical and logic correctness. | On every code change / pre-commit. |
| **Tier 2** | **F-16 Baseline Scenarios** | `f4flight_digi_scenarios` with `f16bk50.json` | Running the full suite of 25+ high-fidelity scenarios specifically on the primary reference fighter. | Before opening a Pull Request. |
| **Tier 3** | **Semantically-Selective Cross-Aircraft** | `ctest` with filtered scenario/aircraft groups | Running scenarios on a broad fleet of aircraft configurations to catch regression across dynamics. | Nightly build / CI system. |

---

## 3. Semantically-Selective Cross-Aircraft Registration

Previously, the test runner registered every scenario against every aircraft config blindly (resulting in over 3,300 ctest definitions). This led to:
- Cargo/transport aircraft (e.g., `C-130`, `E-3A`) being evaluated in high-aspect air-to-air dogfights (`digi_guns`, `digi_missile_engage`) where they failed or exhibited highly unstable behavior, generating test noise.
- Excessive execution times and diluted test signals.

### Selection Strategy:
We filter test combinations by matching scenarios with semantically appropriate aircraft categories:

1. **Autopilot & Recovery Scenarios** (`ai_basic`, `ai_cruise`, `ai_flightplan`, `digi_rtb`, `digi_loiter_orbit`, `digi_groundops`):
   - **Target**: Run on **ALL** aircraft classes (Fighters, Heavies, Transports, Attack).
2. **Combat Scenarios** (`digi_guns`, `digi_guns_rear`, `digi_wvr`, `digi_wvr_defensive`, `digi_bvr`, `digi_missile_engage`, `digi_defensive`, `digi_sensors`, `digi_merge`, `digi_separate`):
   - **Target**: Only run on **Fighters** and **Attack** aircraft.
3. **Formation Flying Scenarios** (`digi_formation`, `digi_formation_types`, `digi_formation_maneuver`, `digi_e2e_formation`):
   - **Target**: Only run on **Fighters** (and select high-maneuverability aircraft like `A-10`).
4. **Refueling & Strike Scenarios** (`digi_aar`, `digi_e2e_aar`, `digi_ground_attack`, `digi_ground_attack_profiles`, `digi_e2e_ground_attack`):
   - **Target**: Run on aircraft with appropriate hardware configurations (e.g. receivers, bombers, attack).

---

## 4. Guidelines for Defining Test Criteria

To prevent "too loose" parameters and ensure we catch oscillations or instabilities (such as unstable wingman tracking), developers must adhere to the following rules when defining pass/fail criteria in new or modified scenarios:

### Avoid Momentary Thresholds
- Checking `minDistToSlot_ < 300.0` alone is insufficient. An unstable, oscillating wingman might sweep past the slot at high speed and trigger a PASS, even though it immediately departs or diverges.
- **Solution**: Always couple momentary thresholds with **sustained proximity checks** and **final stability windows**:
  - *Example*: Require `timeInProximity_ >= 15.0 s` (total time within 500 ft of slot) AND `finalWindowInPos_ / finalWindowTime_ >= 50%` (stable in the last 10 seconds).

### Clean, Synced Console Summaries
- The console text printed in `Finish()` **must** be driven by the exact same Boolean expressions evaluated in `IsPassed()`.
- Avoid hardcoding different numbers in `IsPassed()` (e.g., checking `minDistToBoom_ > 600.0`) and `Finish()` (e.g., printing `minDistToBoom_ < 200.0 ? "[PASS]" : "[FAIL]"`). Such discrepancies create confusion and false-fail or false-pass reports.

### Class-Aware Tolerances
- Do not lower Fighter requirements to allow Heavy/Transport aircraft to pass. Instead, define class-aware thresholds:
```cpp
if (isHeavy(fm.config())) {
    // Relaxed criteria for cargo planes
    minClose = 1000.0;
} else {
    // Tight criteria for fighters
    minClose = 300.0;
}
```

---

## 5. HTML Visualizer Integration

Interactive HTML visualizer reports (generated via `--html` or `--open`) are our main tool for analyzing complex dynamic behaviors like formation tracking, landings, and intercepts. Maintainers should ensure:

1. **Expose Key Variables via Trace Samples**:
   - Every scenario should override `traceSamples()` to publish high-value, scenario-specific quantities (e.g., target range `d_boom`, along-track error `spd_err`, active flag `in_pos`). These are rendered in the live scrubber readout and can be graphed in time-series plots.
2. **Expose Key Milestones via Trace Events**:
   - Use `rec->addEvent()` to mark key moments (e.g., refueling contact made, weapon release consent, engine restart). These are shown as vertical markers on the plots and logged in the interactive event panel.
3. **Keep Scene Geometry Accurate**:
   - Scenario classes should override `sceneGeometry()` to draw static landmarks (e.g., the gold runway centerline in `digi_groundops` or divert base in `digi_rtb`). This gives immediate visual context to the aircraft's path.
