# F4Flight Digi AI — Audit & Fixes (2026-07-14)

Bottom line: 10 bugs found and fixed, 8 tests added (1224/1224 passing).
The critical Bug A fix means autonomous defensive modes now actually
execute when SensorFusion detects a threat — previously they were silent
no-ops because `resolveMode` save/restored the threat pointer.

## Bugs Fixed

| # | Severity | File | Fix |
|---|----------|------|-----|
| A | Critical | `src/digi/digi_brain.cpp` | Removed save/restore in `resolveMode`; commit threat pointer so `runMissileDefeat`/`runGunsJink` can see it. Added threat-recovery when SensorFusion loses the threat. |
| B | High | `include/f4flight/digi/defensive/missile_defeat.h` | `kDragClosureThreshold = 400.0 * KNOTS_TO_FTPSEC` (was 400 ft/s, should be 400 kts → 675 ft/s). |
| C | High | `src/digi/maneuvers/maneuver_primitives.cpp` | Removed `digi.maxRoll = 0.0` in `LevelTurn` Phase 1 — was clobbering host's `setMaxBank()` config permanently. |
| D | Medium | `src/digi/digi_brain.cpp`, `digi_state.h`, `missile_defeat.cpp` | Track `incomingMissileId` in `DigiState`; reset per-missile state on swap. |
| E | Low | `include/f4flight/digi/comms/mailbox.h` | `Mailbox::push` now returns `false` when full (was always `true`). |
| F | Low | `src/digi/offensive/roll_and_pull.cpp` | Use `headingError()` instead of raw `target.yaw - self.yaw` (wrap-around at ±π). |
| G | Medium | `src/digi/sensors/sensor_fusion.cpp` | `bestTarget` now includes Bomber + Helicopter (was Fighter-only). |
| H | Medium | `src/digi/sensors/sensor_fusion.cpp` | Sticky-track `incomingMissile` by `entityId`; swap only if new missile is <0.5× range. Also fixed dangling-pointer bug from `ageAndPurge`. |
| I | Low | `src/digi/defensive/missile_defeat.cpp` | Use `digi.skill.evadeHoldSec` instead of inline `6.0 - skill.level` formula. |
| L | Low | `include/f4flight/digi/atc/taxi_graph.h` | `TaxiGraph::addNode` returns `bool` (false on duplicate/negative id). |

## Bugs Deferred

- **Bug J** — `TaxiGraph::findPath` uses BFS hop count, not Dijkstra. Adequate for typical airbase graphs; add Dijkstra when routing matters.
- **Bug K** — `RunTaxi` is dead code. Wiring it is a feature (full ATC ground ops), not a bug fix.

## Tests Added (8)

- `DigiBrainSensorTest.AutonomousMissileDefeatCommandsNonzeroStick` — verifies Bug A fix
- `DigiBrainSensorTest.AutonomousMissileDefeatTurnsBeamToMissile` — verifies turn direction
- `DigiBrainSensorTest.AutonomousWVREngageCommandsNonzeroStick` — verifies WVR maneuver output
- `DigiBrainSensorTest.AutonomousThreatRecoveryWhenMissileLeavesTruth` — verifies threat recovery
- `DigiBrainSensorTest.MissileSwapReinitializesDefeatState` — verifies Bug D fix
- `RollAndPullTest.OffsetTargetCommandsTurn` — directional roll assertion
- `MailboxTest.FullDropsOldest` (strengthened) — verifies Bug E return value
- `TaxiGraphTest.AddNodeReturnsFalseOnDuplicate` / `AddNodeRejectsNegativeId` — verifies Bug L

## Structural Recommendations (before continuing the port)

1. ~~**Refactor `DigiBrain` host API** — split into `configure()` / `setFrameInputs()` / `commandXxx()` / const-only `state()`. Remove deprecated `setIncomingMissile`/`setGunsThreat`. Unblocks everything else.~~ **DONE (2026-07-14)** — see below.

2. **Add weapon model + fire control** — no `WeaponClass`, no `RMax`, no SMS, no `FireControl`. Required for BVR.

3. **Add formation geometry + lead concept** — `MessageBus` exists but no handler. No `FormationGeometry`, no `leadId`. Required for wingman.

4. **Wire `TaxiGraph` into `RunTaxi`** — `RunTaxi` is dead code. Required for full ATC ground ops.

5. **Adopt `Quantity<Tag>` for units in new digi code** — `core/units.h` has a strong-type system that the digi/ subsystem never uses. Bug B was a direct symptom. Adopt at function signatures before BVR porting.

6. **Port `DigiMode` enum to full 25-mode stack** — currently 7 of 25 modes. Preserve priority ordering exactly (FreeFalcon's `AddMode` uses `newMode < nextMode`).

---

## DigiBrain Host API Refactor (completed 2026-07-14)

The `DigiBrain` host-facing API has been refactored per recommendation §3.1.
The old flat `set*` interface is replaced with four clear categories:

### New API

```cpp
// 1. Configuration (set once at init)
struct DigiConfig {
    SkillLevel skillLevel;
    double cornerSpeedKts, maxGs, maxBankDeg, maxGammaDeg, turnLoadFactor;
};
void configure(const DigiConfig& cfg);
DigiConfig config() const;

// 2. Navigation setup
void setWaypoints(std::vector<Vec3>);
void setCaptureRadius(double r_ft);
void setHeading(double rad);
void setAltitude(double ft);

// 3. Per-frame inputs
struct FrameInputs {
    const TruthState* truth;              // SensorFusion input (production)
    const DigiEntity* selfEntity;         // null = auto-build from AircraftState
    const DigiEntity* injectedTarget;     // offensive target (host-injected)
    const DigiEntity* injectedMissile;    // testing: bypass SensorFusion
    const DigiEntity* injectedGunsThreat; // testing: bypass SensorFusion
};
void setFrameInputs(const FrameInputs& inputs);

// 4. Commands (asynchronous)
void commandTakeoff(RunwayId, ...);
void commandLanding(RunwayId, ...);
void clearTarget();

// 5. Mode override (testing)
void forceMode(DigiMode);
void clearForcedMode();

// 6. Read-only state (const only)
const DigiState& state() const;
DigiMode activeMode() const;

// 7. Mutable state — TESTING ONLY
DigiState& stateMutable();
```

### Backward compatibility

All old `set*` methods are retained as `[[deprecated]]` inline shims that
delegate to the new internal storage (`frameInputs_`). Existing code
(tests, `SteeringController`, host programs) continues to compile and run
without modification. The shims will be removed in a future release once
all callers migrate.

The mutable `DigiState& state()` overload is also deprecated; use
`stateMutable()` for write access or `state()` (const) for read access.

### What this fixes

- **Bug A root cause eliminated**: the dual-path confusion between
  `setIncomingMissile()` and `SensorFusion` is now explicit —
  `FrameInputs` has both `truth` (production) and `injectedMissile`
  (testing) fields, and `resolveMode()` clearly prioritizes injected
  over auto-tracked.
- **Encapsulation**: `state()` is const-only; production code cannot
  accidentally mutate `DigiState` fields directly.
- **Scalability**: each new mode adds a `commandXxx()` method + internal
  handler, not 2-5 new `set*` fields.

### Tests

11 new tests added covering the new API:
`ConfigureSetsAllConfigFields`, `ConfigReadsBackCurrentValues`,
`SetFrameInputsStoresTruthAndSelfEntity`,
`SetFrameInputsAutoBuildsSelfEntityWhenNull`,
`SetFrameInputsWithTruthRunsSensorFusion`,
`SetFrameInputsInjectedMissileEntersMissileDefeat`,
`CommandTakeoffSetsGroundOpsPhase`, `CommandLandingSetsGroundOpsPhase`,
`ForceModeAndClearForcedMode`, `StateMutableAllowsWriteForTesting`,
`ResetClearsFrameInputsAndAutoEntities`.

**Test count:** 1224 → 1235 (all passing).

## Recommended Porting Order

1. Refactor `DigiBrain` host API
2. Weapon model + fire control
3. BVR engagement (`bvrengage.cpp` — 3,238 LOC, 18 tactical profiles)
4. CollisionAvoid + Separate (small, quick wins)
5. Formation geometry + wingman (`wingai.cpp` + `wingmnvers.cpp` + `formdata.cpp`)
6. Full ATC ground ops (`landme.cpp` — 4,778 LOC, port in chunks)
7. Refueling (`refuel.cpp` + `tankbrn.cpp`)
8. A/G attack (`gndattck.cpp` — 4,929 LOC, largest file, needs SMS + FCC + IP + doctrine)

See `download/f4flight_digi_analysis.docx` for the full report.
