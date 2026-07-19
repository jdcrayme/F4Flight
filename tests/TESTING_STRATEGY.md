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

### C. Digi AI Scenario Tests (3-Tier Hierarchy)
- **Files Location**: `tests/digi/scenarios/{low_level,high_level,e2e}/`
- **Scope**: Closed-loop testing of the Digi AI brain coupled with the Flight Model.
- **Approach**: Exercises the `SteeringController` and `DigiBrain` over a prolonged period (e.g., 20–150 seconds) in high-fidelity missions.
- **Goal**: Replicate FreeFalcon digi AI behaviors, tracking mode transitions, and ensuring stable maneuvers.

The Digi AI scenarios are further subdivided into three tiers (see §2 below).

---

## 2. Digi AI 3-Tier Test Hierarchy

The DIGI AI test suite is organized into three tiers that form a drill-down
hierarchy. The cascade workflow (§3) uses these tiers to quickly identify the
root cause of a failure: when an End-to-End mission fails, run the linked
High-Level chains; when a High-Level chain fails, run the linked Low-Level
behaviors.

### Tier 1 — Low Level (one behavior per scenario)

- **Files Location**: `tests/digi/scenarios/low_level/`
- **Scope**: Each scenario tests ONE behavior in isolation (Takeoff, Climb,
  BVR Engage, AAR Pre-Contact, Missile Defeat, ...).
- **Naming**: New scenarios use the `low_*` prefix (e.g., `low_taxi`,
  `low_takeoff`, `low_aar_vector`). Pre-existing single-behavior scenarios
  keep their original names (`digi_bvr`, `digi_wvr`, `ai_cruise`, ...).
- **Coverage goal**: Every entry in FreeFalcon's `DigiMode` enum should have
  at least one low-level test. See `tests/COVERAGE.md` for the matrix.
- **Pass criteria**: Relaxed — verify the AI enters the right mode + makes
  meaningful progress. The point is "does the behavior work at all", not
  "does it meet tight tolerances".

### Tier 2 — High Level (chains of behaviors)

- **Files Location**: `tests/digi/scenarios/high_level/`
- **Scope**: Each scenario chains 3-5 related behaviors into a realistic
  operational sequence (Taxi → Takeoff → Climb → Level-off, or
  Missile Defeat → Guns Jink → Collision Avoid → Re-engage).
- **Naming**: New scenarios use the `high_*` prefix (e.g., `high_departure`,
  `high_aar`, `high_air_to_air_engage`). Pre-existing chain scenarios keep
  their original names (`digi_groundops`, `digi_aar`, `digi_defensive`,
  `ai_basic`, ...).
- **Pass criteria**: Each phase passes if the AI enters the right mode for
  that segment + makes meaningful progress. The point is "does the chain
  complete in the right order with the right modes".

### Tier 3 — End-to-End (full AMIS_* missions)

- **Files Location**: `tests/digi/scenarios/e2e/`
- **Scope**: Each scenario is a full mission analogous to one of FreeFalcon's
  `AMIS_*` campaign mission types (BARCAP, TARCAP, SWEEP, INTERCEPT, ESCORT,
  ...). They cover Takeoff → Navigate → Mission execution → RTB.
- **Naming**: New scenarios use the `e2e_*` prefix (e.g., `e2e_barcap`,
  `e2e_intercept`). Pre-existing E2E scenarios keep their original names
  (`digi_e2e_mission`, `digi_e2e_aar`, `digi_e2e_formation`,
  `digi_e2e_ground_attack`).
- **Coverage goal**: Every `AMIS_*` mission type should have an E2E test.
  The current pass covers the 5 core fighter missions; see
  `tests/COVERAGE.md` for the gap analysis.
- **Pass criteria**: Each phase passes if the AI enters the right mode + the
  mission makes meaningful progress through its expected sequence.

---

## 3. Cascade Execution Workflow

The cascade workflow is the primary post-commit test strategy:

1. **Run all E2E tests** — `f4flight_digi_scenarios ac.json --level e2e`
2. **If any E2E fails**, run the linked High-Level chains for those failures
3. **If any High-Level fails**, run the linked Low-Level behaviors for those
   failures
4. **The Low-Level failure** points you at the specific behavior that broke

The cascade is implemented in the binary itself via the `--cascade` flag:

```bash
# Run the full cascade (E2E -> High -> Low) in one invocation
./build/tests/f4flight_digi_scenarios tests/fixtures/aircraft/fighters/f16bk50.json \
    --cascade --html cascade_report.html --open
```

The mapping tables are in `tests/framework/scenario_framework.cpp`:
- `g_e2eToHigh` — maps each E2E scenario name to its composing High-Level chains
- `g_highToLow` — maps each High-Level chain to its composing Low-Level behaviors

Use `--list` to print the cascade mapping at runtime:

```bash
./build/tests/f4flight_digi_scenarios --list
```

### Cascade deduplication

Each scenario is run at most once per cascade invocation, even if multiple
E2E failures would trigger it. This means a Low-Level behavior that breaks
and is shared by multiple High-Level chains (e.g., `low_takeoff` is used by
both `high_departure` and `high_recovery`) is only run once.

---

## 4. Test Execution Tiers (CI / pre-commit / nightly)

| Tier | Name | Target | Description | Execution Frequency |
|---|---|---|---|---|
| **Tier 1** | **Fast Unit Tests** | `f4flight_flight_tests`, `f4flight_digi_tests` | Pure GoogleTest binaries covering mathematical and logic correctness. | On every code change / pre-commit. |
| **Tier 2** | **Low-Level Digi Scenarios** | `f4flight_digi_scenarios --level low` | All 35 low-level DIGI AI behavior tests on the F-16 baseline. | On every code change / pre-commit. |
| **Tier 3** | **High-Level Digi Scenarios** | `f4flight_digi_scenarios --level high` | All 14 high-level DIGI AI chain tests on the F-16 baseline. | Before opening a Pull Request. |
| **Tier 4** | **End-to-End Digi Scenarios** | `f4flight_digi_scenarios --level e2e` | All 9 E2E mission tests (5 AMIS_* + 4 legacy) on the F-16 baseline. | Before opening a Pull Request. |
| **Tier 5** | **Cascade** | `f4flight_digi_scenarios --cascade` | Full drill-down: E2E → High → Low. Use when Tier 4 has failures to identify the root cause. | On CI failures / debugging. |
| **Tier 6** | **Cross-Aircraft** | `ctest -L fighters` / `ctest -L attack` / `ctest -L bomber` / `ctest -L transport` | Run scenarios across the full fleet of aircraft configurations. | Nightly build / CI system. |

### CTest labels

Each ctest is tagged with multiple labels via `set_tests_properties(... LABELS ...)`:

- **Tier label**: `LowLevel` | `HighLevel` | `EndToEnd`
- **Aircraft category label**: `fighters` | `attack` | `bomber` | `transport`
- **Suite label**: `digi` | `flight`

Examples:

```bash
ctest -L LowLevel                    # run only low-level digi tests
ctest -L EndToEnd                    # run only E2E digi tests
ctest -L fighters                    # run only fighter-aircraft tests
ctest -L 'LowLevel;fighters'         # run low-level + fighters (intersection)
ctest -L digi                        # run all digi tests (any tier)
```

---

## 5. Semantically-Selective Cross-Aircraft Registration

Previously, the test runner registered every scenario against every aircraft config blindly (resulting in over 3,300 ctest definitions). This led to:
- Cargo/transport aircraft (e.g., `C-130`, `E-3A`) being evaluated in high-aspect air-to-air dogfights (`digi_guns`, `digi_missile_engage`) where they failed or exhibited highly unstable behavior, generating test noise.
- Excessive execution times and diluted test signals.

### Selection Strategy:
We filter test combinations by matching scenarios with semantically appropriate aircraft categories:

1. **Autopilot & Recovery Scenarios** (`ai_basic`, `ai_flightplan`, `ai_cruise`, `digi_rtb`, `digi_loiter_orbit`, `digi_groundops`, `low_taxi`, `low_takeoff`, `low_landing`, `low_approach`, `high_departure`, `high_loiter_station`, `high_recovery`):
   - **Target**: Run on **ALL** aircraft classes (Fighters, Heavies, Transports, Attack).
2. **Combat Scenarios** (`digi_guns`, `digi_guns_rear`, `digi_wvr`, `digi_wvr_defensive`, `digi_bvr`, `digi_missile_engage`, `digi_defensive`, `digi_sensors`, `digi_merge`, `digi_separate`, `digi_collision`, `digi_tactics`, `digi_e2e_mission`, `low_roop`, `low_overb`, `low_missile_defeat`, `low_guns_jink`, `high_air_to_air_engage`, `high_defensive_chain`, `e2e_barcap`, `e2e_tarcap`, `e2e_sweep`, `e2e_intercept`, `e2e_escort`):
   - **Target**: Only run on **Fighters** and **Attack** aircraft.
3. **Formation Flying Scenarios** (`digi_formation`, `digi_formation_types`, `digi_formation_maneuver`, `digi_e2e_formation`, `low_formation_turn`, `high_formation_joinup`):
   - **Target**: Only run on **Fighters** (and select high-maneuverability aircraft like `A-10`).
4. **Refueling & Strike Scenarios** (`digi_aar`, `digi_e2e_aar`, `digi_ground_attack`, `digi_ground_attack_profiles`, `digi_e2e_ground_attack`, `low_aar_*`, `low_ground_attack_dive`, `low_ground_attack_toss`, `low_ground_attack_high`, `high_air_to_ground`):
   - **Target**: Run on aircraft with appropriate hardware configurations (e.g. receivers, bombers, attack).

---

## 6. Guidelines for Defining Test Criteria

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

### Tier-Appropriate Tolerances
- **Low-Level tests**: relaxed criteria — verify the AI enters the right mode + makes meaningful progress. The point is "does the behavior work at all".
- **High-Level tests**: relaxed per-phase criteria — verify the chain completes in the right order with the right modes.
- **E2E tests**: relaxed per-phase criteria — verify the mission completes in the right order.
- **Tight tolerances** are for specialized tests (e.g., `digi_loiter_orbit` for orbit geometry) where the precise metric is the point of the test.

---

## 7. HTML Visualizer Integration

Interactive HTML visualizer reports (generated via `--html` or `--open`) are our main tool for analyzing complex dynamic behaviors like formation tracking, landings, and intercepts. Maintainers should ensure:

### Tier Tabs
The HTML report has a 4-tab bar above the card grid:
- **All** — show all traces regardless of tier
- **Low Level** — only low-level traces
- **High Level** — only high-level traces
- **End-to-End** — only E2E traces

Each tab shows the total count and (if non-zero) a red failure pill. Clicking
a tab filters the card grid and updates the summary cards (Traces / Passing /
Failing / Phases passed) to reflect only that tier.

### Per-Trace Tier Badge
Every card in the "All" view shows a small badge under the title indicating
which tier the trace belongs to (`Low Level` / `High Level` / `End-to-End`),
so you can see at a glance which tier each card is in.

### Expose Key Variables via Trace Samples
- Every scenario should override `traceSamples()` to publish high-value, scenario-specific quantities (e.g., target range `d_boom`, along-track error `spd_err`, active flag `in_pos`). These are rendered in the live scrubber readout and can be graphed in time-series plots.

### Expose Key Milestones via Trace Events
- Use `rec->addEvent()` to mark key moments (e.g., refueling contact made, weapon release consent, engine restart). These are shown as vertical markers on the plots and logged in the interactive event panel.

### Keep Scene Geometry Accurate
- Scenario classes should override `sceneGeometry()` to draw static landmarks (e.g., the gold runway centerline in `digi_groundops` or divert base in `digi_rtb`). This gives immediate visual context to the aircraft's path.

---

## 8. Coverage Matrix

See `tests/COVERAGE.md` for the authoritative mapping of:
- Every FreeFalcon `DigiMode` enum entry → F4Flight low-level test
- Every FreeFalcon `AMIS_*` mission type → F4Flight E2E test (with gap analysis)
- Every high-level chain → its composing low-level tests
- The cascade mapping graph (E2E → High → Low)

This matrix is the source of truth for "what is covered" and "what is
missing". When adding a new test, update the matrix.
