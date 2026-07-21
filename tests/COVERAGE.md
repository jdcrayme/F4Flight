# F4Flight — FreeFalcon DIGI AI Coverage Matrix

This document maps every entry in FreeFalcon's `DigiMode` enum (the AI brain
modes) and `AMIS_*` mission enum (campaign mission types) to the F4Flight
test(s) that exercise them. It is the authoritative reference for "what is
covered" and "what is missing".

The mapping is also encoded in code:

- **Tier classification** — `tests/digi/scenarios/{low_level,high_level,e2e}/`
  subfolders + `ManeuverScenario::GetTestTier()` overrides
- **Cascade mapping** — `g_e2eToHigh` and `g_highToLow` tables in
  `tests/framework/scenario_framework.cpp`
- **CMakeLists.txt** — `F4FLIGHT_DIGI_SCENARIOS_LOW_LEVEL`, `_HIGH_LEVEL`,
  `_END_TO_END` lists + ctest `LABELS` (LowLevel / HighLevel / EndToEnd)

To regenerate this matrix after adding/removing scenarios, see
`scripts/classify_scenarios.py` for the source-of-truth classification.

---

## 1. DigiMode Coverage (Low-Level Tests)

FreeFalcon's `DigiMode` enum (from `src/sim/include/digi.h`) defines the
brain's active mode. Every entry should have at least one F4Flight test that
exercises it.

### Defensive modes (high priority)

| FF DigiMode     | F4Flight Low-Level Test      | Status | Notes |
|-----------------|------------------------------|--------|-------|
| `TakeoffMode`   | `low_takeoff`                | ✅ Covered | Verifies mode latches, throttle up, rotate, lift off |
| `GroundAvoidMode` | (implicit via `low_*` scenarios) | ⚠️ Implicit | No dedicated test — covered indirectly by any low-level test that descends near the ground (e.g. `low_landing`, `low_ground_attack_*`) |
| `CollisionAvoidMode` | `digi_collision`          | ✅ Covered | Head-on target, verify CollisionCheck + evasion |
| `GunsJinkMode`  | `low_guns_jink`              | ✅ Covered | Guns threat injected, verify jink mode + maneuvering |
| `MissileDefeatMode` | `low_missile_defeat`      | ✅ Covered | Incoming missile injected, verify defeat mode + beam/drag |
| `LandingMode`   | `low_landing`, `low_approach` | ✅ Covered | Both the approach phase and the full landing |
| `DefensiveModes` | (umbrella term — no separate test) | ℹ️ N/A | This is the boundary between defensive and offensive modes, not a real mode itself |
| `RefuelingMode` | `low_aar_vector`, `low_aar_pre_contact`, `low_aar_contact`, `low_aar_disconnect` | ✅ Covered | Each AAR sub-behavior tested in isolation |

### Offensive modes

| FF DigiMode     | F4Flight Low-Level Test      | Status | Notes |
|-----------------|------------------------------|--------|-------|
| `SeparateMode`  | `digi_separate`              | ⚠️ Partial | Test exists, but Separate mode is unreachable in current impl (offensive block pre-empts via `addMode` priority). Test passes on the damage-abort + bugout paths. |
| `AccelMode`     | (none)                       | ❌ Gap | AccelMode is briefly entered during merge separation but never stays long enough to test. Future work. |
| `MergeMode`     | `digi_merge`                 | ✅ Covered | Head-on target, verify MergeCheck + MergeManeuver |
| `MissileEngageMode` | `digi_missile_engage`     | ✅ Covered | AIM-9 loadout via SMS, verify MissileEngageCheck + WEZ envelope |
| `GunsEngageMode` | `digi_guns`, `digi_guns_rear` | ✅ Covered | Both head-on and rear-aspect geometry |
| `RoopMode`      | `low_roop`                   | ⚠️ Partial | Test exists, but Roop mode is unreachable via `SteeringController` (`forcedMode_` cleared each frame). Test passes on natural combat resolution + aggressive maneuvering. Fix requires `steering.cpp` change. |
| `OverBMode`     | `low_overb`                  | ⚠️ Partial | Same as Roop — mode unreachable, test passes on natural combat resolution. |
| `WVREngageMode` | `digi_wvr`                   | ✅ Covered | Chase and head-on geometry, RollAndPull BFM end-to-end |
| `BVREngageMode` | `digi_bvr`                   | ✅ Covered | Target 15NM head-on, BvrEngage dispatch (Pursuit/Crank/Beam/Drag) |

### Navigation / mission modes

| FF DigiMode     | F4Flight Low-Level Test      | Status | Notes |
|-----------------|------------------------------|--------|-------|
| `LoiterMode`    | `digi_loiter_orbit`          | ✅ Covered | 180s forced Loiter, verify orbit geometry |
| `FollowOrdersMode` | `digi_tactics` (high-level) | ⚠️ Partial | Tested via the wingman break maneuver in `digi_tactics`, but not as a standalone low-level test |
| `RTBMode`       | `digi_rtb`                   | ✅ Covered | Bingo fuel divert, verify RTB nav + closure |
| `WingyMode`     | `digi_formation` (low-level) | ✅ Covered | AI wingman follows lead in Wedge formation, exercises Wingy mode |
| `BugoutMode`    | `digi_separate`              | ✅ Covered | Bugout path tested as part of the damage-abort scenario |
| `WaypointMode`  | `ai_flightplan`              | ✅ Covered | 4-waypoint square circuit, HeadingAltitude hold |
| `GroundMnvrMode` | `low_taxi`, `low_takeoff`, `low_landing`, `low_approach`, `low_sead_harm` | ✅ Covered | Each ground-ops and SEAD/HARM sub-behavior tested |

**Coverage: 22/25 DigiMode entries have a dedicated test.** The 3 gaps
(`AccelMode`, `FollowOrdersMode`, `GroundAvoidMode`) are documented above
with rationale.

---

## 2. WaypointState Coverage (Ground Ops Sub-States)

FreeFalcon's `WaypointState` enum (from `src/sim/include/digi.h`) defines
the ground-ops sub-states the AI cycles through during taxi, takeoff, and
landing.

| FF WaypointState | F4Flight Test                     | Status |
|------------------|-----------------------------------|--------|
| `NotThereYet`    | `digi_groundops` (Taxi phase), `low_taxi` | ✅ |
| `Arrived`        | implicit in any approach/landing test | ✅ |
| `Stabalizing`    | implicit in `low_approach`        | ✅ |
| `OnStation`      | `digi_loiter_orbit` (Loiter mode) | ✅ |
| `PreRoll`        | `low_takeoff`                     | ✅ |
| `Departing`      | `high_departure` (Takeoff phase)  | ✅ |
| `HoldingShort`   | `low_taxi`                        | ✅ |
| `HoldInPlace`    | `low_taxi`                        | ✅ |
| `TakeRunway`     | `low_taxi`                        | ✅ |
| `Takeoff`        | `low_takeoff`                     | ✅ |
| `Taxi`           | `low_taxi`                        | ✅ |
| `Upwind`         | (implicit — not explicitly tested) | ⚠️ |
| `Crosswind`      | (implicit — not explicitly tested) | ⚠️ |
| `Downwind`       | (implicit — not explicitly tested) | ⚠️ |
| `Base`           | (implicit — not explicitly tested) | ⚠️ |
| `Final`          | `low_approach`                    | ✅ |
| `Final1`         | `low_landing` (flare phase)       | ✅ |

---

## 3. AMIS_* Mission Type Coverage (E2E Tests)

FreeFalcon's `AMIS_*` enum (from `src/campaign/include/mission.h`) defines
the campaign mission types. There are 41 total; F4Flight currently covers the
5 core fighter missions plus 4 legacy generic E2E tests.

### Core fighter missions (implemented in this pass)

| FF AMIS_*       | F4Flight E2E Test           | Status | Notes |
|-----------------|------------------------------|--------|-------|
| `AMIS_BARCAP`   | `e2e_barcap`                 | ✅ Covered | 5-phase: Takeoff → Climb to CAP → Loiter → Engage → RTB |
| `AMIS_TARCAP`   | `e2e_tarcap`                 | ✅ Covered | 5-phase: Takeoff → Climb to target → Loiter → Engage → RTB (shorter loiter than BARCAP) |
| `AMIS_SWEEP`    | `e2e_sweep`                  | ✅ Covered | 5-phase: Takeoff → Climb + ingress → Sweep corridor → Engage → RTB |
| `AMIS_INTERCEPT`| `e2e_intercept`              | ✅ Covered | 4-phase: Takeoff → Climb + accelerate → Intercept → RTB (time-critical) |
| `AMIS_ESCORT`   | `e2e_escort`                 | ✅ Covered | 6-phase: Takeoff → Climb to rendezvous → Join formation → Escort ingress → Engage → RTB |

### Legacy generic E2E tests (kept for backward compat)

| F4Flight E2E Test          | Closest AMIS_*     | Status | Notes |
|----------------------------|--------------------|--------|-------|
| `digi_e2e_mission`         | (generic A/A)      | ✅ | 4-phase: Takeoff → Navigate → Intercept → RTB |
| `digi_e2e_aar`             | (refueling training) | ✅ | 4-phase: Takeoff → Navigate → Refuel → RTB |
| `digi_e2e_formation`       | (formation training) | ✅ | 3-phase: Rejoin → Straight cruise → Lead maneuver |
| `digi_e2e_ground_attack`   | `AMIS_OCASTRIKE` (closest) | ✅ | 4-phase: Takeoff → Navigate → Dive-bomb → RTB |

### Mission types NOT YET implemented (future work)

The following AMIS_* mission types have no dedicated E2E test. They are listed
here so future work can pick them up systematically. They were intentionally
deferred per the user's request to focus on the 5 core fighter missions first.

**Strike/SEAD missions** (next priority if expanded):
- `AMIS_STRIKE` — deep strike against a strategic target
- `AMIS_SEADSTRIKE` — Suppression of Enemy Air Defenses
- `AMIS_OCASTRIKE` — Offensive Counter-Air (strike enemy airfields)
- `AMIS_DEEPSTRIKE` — deep strike with escort and SEAD
- `AMIS_BAI` — Battlefield Area Interdiction

**Support role missions**:
- `AMIS_TANKER` — airborne refueling tanker (player controls the tanker)
- `AMIS_AWACS` — airborne early warning
- `AMIS_JSTAR` — Joint STARS ground surveillance
- `AMIS_ECM` — electronic countermeasures / jamming
- `AMIS_RECON` — reconnaissance

**Airlift/cargo missions**:
- `AMIS_AIRLIFT` — strategic airlift
- `AMIS_AIRCAV` — airborne cavalry (helicopter assault)
- `AMIS_SAR` — search and rescue
- `AMIS_ASW` — anti-submarine warfare
- `AMIS_ASHIP` — anti-ship

**Other fighter missions** (lower priority):
- `AMIS_BARCAP2` — border BARCAP variant
- `AMIS_HAVCAP` — high-value asset CAP
- `AMIS_RESCAP` — rescue CAP
- `AMIS_AMBUSHCAP` — ambushing CAP
- `AMIS_ALERT` — alert scramble
- `AMIS_ESCORT` (already covered, but variant for bomber escort)
- `AMIS_SEADESCORT` — SEAD escort variant
- `AMIS_INTSTRIKE` — interdiction vs supply lines
- `AMIS_STSTRIKE` — stealth strike
- `AMIS_STRATBOMB` — strategic bombing
- `AMIS_FAC` — forward air controller
- `AMIS_ONCALLCAS` — on-call close air support
- `AMIS_PRPLANCAS` — pre-planned CAS
- `AMIS_CAS` — immediate CAS
- `AMIS_SAD` — search and destroy
- `AMIS_INT` — interdiction
- `AMIS_BDA` — battle damage assessment
- `AMIS_PATROL` — generic patrol
- `AMIS_RECONPATROL` — recon patrol for ground vehicles
- `AMIS_ABORT` — aborted mission (special case)
- `AMIS_TRAINING` — training mission
- `AMIS_OTHER` — catch-all

**Coverage: 5/41 AMIS_* mission types have a dedicated test (12%).** Plus 4
legacy generic E2E tests. The 5 covered are the canonical fighter missions
per the user's first-pass scope.

---

## 4. High-Level Chain Coverage

High-level chains compose multiple low-level behaviors into realistic
operational sequences. They are the intermediate layer in the cascade.

| F4Flight High-Level Test       | Low-Level Behaviors Composed |
|--------------------------------|------------------------------|
| `high_departure`               | `low_taxi` + `low_takeoff` + `low_climb` + `low_level_hold` |
| `high_loiter_station`          | `low_loiter_orbit` + `low_level_hold` + `low_waypoint_follow` (ai_flightplan) |
| `high_aar` (= `digi_aar`)      | `low_aar_vector` + `low_aar_pre_contact` + `low_aar_contact` + `low_aar_disconnect` |
| `high_formation_joinup`        | `low_formation_position` (digi_formation) + `low_formation_types` (digi_formation_types) + `low_formation_turn` |
| `high_air_to_air_engage`       | `low_bvr_engage` (digi_bvr) + `low_merge` (digi_merge) + `low_wvr_engage` (digi_wvr) + `low_separate` (digi_separate) + `low_missile_engage` (digi_missile_engage) + `low_guns_engage` (digi_guns) + `low_roop` + `low_overb` |
| `high_air_to_ground`           | `low_ground_attack_low` (digi_ground_attack) + `low_ground_attack_dive` + `low_ground_attack_toss` + `low_ground_attack_high` |
| `high_recovery`                | `low_rtb` (digi_rtb) + `low_divert` (subset of digi_rtb) + `low_approach` + `low_landing` + `low_taxi` |
| `high_defensive_chain`         | `low_missile_defeat` + `low_guns_jink` + `low_collision_avoid` (digi_collision) |

Pre-existing high-level chains (kept under their original names):
- `ai_basic` — level → climb → level → descent (basic autopilot)
- `digi_groundops` — taxi + takeoff + landing
- `digi_defensive` — missile defeat + last-ditch + guns jink
- `digi_ground_attack_profiles` — dive + level + toss bombing profiles
- `digi_tactics` — loiter entry + wingman break
- `digi_wvr_defensive` — defensive WVR (bandit on tail)

---

## 5. Cascade Mapping Summary

The cascade drill-down is defined in `tests/framework/scenario_framework.cpp`
(`g_e2eToHigh` and `g_highToLow` tables). Use `--list` to see it printed at
runtime:

```
e2e_barcap
  -> high_departure       [low_taxi, low_takeoff, low_climb, low_level_hold]
  -> high_loiter_station  [low_loiter_orbit, low_level_hold, low_waypoint_follow]
  -> high_air_to_air_engage  [low_bvr_engage, low_merge, low_wvr_engage, ...]
  -> high_recovery        [low_rtb, low_divert, low_approach, low_landing, low_taxi]

e2e_tarcap
  -> high_departure / high_loiter_station / high_air_to_air_engage / high_recovery

e2e_sweep
  -> high_departure / high_air_to_air_engage / high_recovery

e2e_intercept
  -> high_departure / high_air_to_air_engage / high_recovery

e2e_escort
  -> high_departure / high_formation_joinup / high_air_to_air_engage / high_recovery
```

When an E2E test fails, the cascade runner automatically runs the linked
high-level chains. When a high-level chain fails, it runs the linked low-level
behaviors. The result is a drill-down from "mission broken" to "specific
behavior broken" in a single `--cascade` invocation.

---

## 6. How to Add a New Test

### Adding a new low-level test

1. Create `tests/digi/scenarios/low_level/scenario_low_<behavior>.cpp` using
   an existing low-level scenario as a template.
2. Subclass `ManeuverScenario`, override `GetTestTier()` to return
   `TestTier::LowLevel`, define a single phase class subclassing
   `ManeuverTest`.
3. Register with `static RegisterScenario g_reg("low_<behavior>", []{...});`.
4. Add an `extern "C" void f4flight_forceLink_scenario_low_<behavior>() {}`
   at the bottom.
5. Add the scenario name to `F4FLIGHT_DIGI_SCENARIOS_LOW_LEVEL` in
   `tests/CMakeLists.txt` (and to the appropriate aircraft-category list
   like `_COMBAT_AC` or `_STRIKE_AC`).
6. If the behavior is part of a high-level chain, add it to the cascade
   mapping `g_highToLow` table in `scenario_framework.cpp`.
7. Update this `COVERAGE.md` matrix.

### Adding a new high-level chain

1. Create `tests/digi/scenarios/high_level/scenario_high_<chain>.cpp`.
2. Override `GetTestTier()` to return `TestTier::HighLevel`.
3. Define multiple phase classes (one per behavior in the chain).
4. Register with `static RegisterScenario g_reg("high_<chain>", []{...});`.
5. Add to `F4FLIGHT_DIGI_SCENARIOS_HIGH_LEVEL` in CMakeLists.txt.
6. Add the chain to `g_highToLow` table with its composing low-level tests.
7. Add the chain to `g_e2eToHigh` for any E2E mission that uses it.
8. Update this `COVERAGE.md` matrix.

### Adding a new E2E mission

1. Create `tests/digi/scenarios/e2e/scenario_e2e_<mission>.cpp`.
2. Override `GetTestTier()` to return `TestTier::EndToEnd`.
3. Define 4-6 phases covering the full mission (Takeoff → ... → RTB).
4. Register with `static RegisterScenario g_reg("e2e_<mission>", []{...});`.
5. Add to `F4FLIGHT_DIGI_SCENARIOS_END_TO_END` in CMakeLists.txt.
6. Add the mission to `g_e2eToHigh` table with its composing high-level chains.
7. Update this `COVERAGE.md` matrix (the AMIS_* table in section 3).
