# F4Flight Digi AI — Round-3 Audit

**Date:** 2026-07-14
**Scope:** Verify F4Flight faithfully replicates FreeFalcon digi AI capabilities;
fix broken/stupid code; determine structural changes needed before continuing
the FreeFalcon digi port.

**Sources compared:**
- FreeFalcon: `https://github.com/FreeFalcon/freefalcon-central` → `src/sim/digi/*` (~40K LOC) + `src/sim/include/digi.h`
- F4Flight: `https://github.com/jdcrayme/F4Flight` → `digi/` (~9.6K LOC headers+sources)

**Build/test status at audit time:** clean build (gcc 14.2, C++17, `-Wall -Wextra -Wpedantic`),
**1369/1369 tests pass** (was 1368 before this round; +1 regression test added).

---

## 1. Fixes applied this round (Round-3)

### 1.1 BVR engage-range bug + dead `BvrEngageCheck()`  *(behavioral fix)*

**File:** `digi/src/digi_brain.cpp` (`resolveMode`)

**Problem.** `BvrEngageCheck()` was defined in `digi/src/offensive/bvr_engage.cpp`
(with the correct FreeFalcon entry logic `engageRange = max(maxAAWpnRange×1.3, 45 NM)`)
but **never called** by the brain. `resolveMode` used an inline
`if (range > 8 NM) { BVREngage; }` nested inside `if (range < maxAAWpnRangeFt)`.
Two consequences:

1. **Dead code** — `BvrEngageCheck` had zero callers (verified by grep). Every
   other offensive `*Check` function (`MissileEngageCheck`, `MergeCheck`,
   `GunsEngageCheck`, `CollisionCheck`) IS called by `resolveMode`; BVR was the
   lone inconsistency.
2. **Behavioral bug** — BVR was silently capped at `maxAAWpnRange` (default
   35 NM) instead of FreeFalcon's `max(45 NM, maxAAWpnRange×1.3)` ≈ 45.5 NM.
   A target at 40 NM stayed in Waypoint navigation instead of entering BVR.

**Fix.** Replaced the inline check with a call to `BvrEngageCheck()`, moved
*before* the `range < maxAAWpnRangeFt` guard so BVR can engage out to the full
`engageRange`. `BvrEngageCheck` returns `false` inside 8 NM (RAP distance), so
the WVR-family checks (`MissileEngage`/`Merge`/`GunsEngage`/`WVREngage`) below
still own the <8 NM regime unchanged.

**Test impact.** `DigiBrainWVRTest.TargetBeyondBVRStaysWaypoint` asserted the
*buggy* behavior (40 NM → Waypoint). Updated it to 50 NM (genuinely beyond
engageRange) and added `TargetAtFortyNmEntersBVR` to lock in the corrected
boundary. Net: +1 test, 1369/1369 pass.

### 1.2 Dead `DigiBrain::runGroundAvoid()` method  *(dead-code removal)*

**Files:** `digi/include/f4flight/digi/digi_brain.h`, `digi/src/digi_brain.cpp`

**Problem.** `runGroundAvoid()` was declared + defined but **never called**.
Ground avoidance is not a dispatched mode — it runs as a concurrent overlay in
`compute()` (`RunGroundAvoid` sets `pullingUp`, and the per-mode `switch` is
skipped entirely while `pullingUp` is true). The `case DigiMode::GroundAvoid:`
in the switch was an empty `break;`, reachable only via `forceMode(GroundAvoid)`
when there is no terrain danger (in which case a no-op is correct). The method
also hardcoded `groundZ = 0.0`, a latent bug that never triggered because the
method was dead.

**Fix.** Removed the declaration and definition. Added a comment block above
the per-mode action declarations and at the `case DigiMode::GroundAvoid:` site
documenting that the no-op is intentional (overlay handles it).

### 1.3 Deprecation warning in `scenario_framework.cpp:249`  *(warning fix)*

**File:** `tests/framework/scenario_framework.cpp`

**Problem.** `const auto& ds = sc.brain().state();` — `sc.brain()` returns a
non-const `DigiBrain&`, so overload resolution picked the `[[deprecated]]`
non-const `state()` overload. This was the only build warning. It was hidden
by `-Wno-deprecated-declarations` on the scenario targets.

**Fix.** Bind to a const reference first so the const (non-deprecated) overload
is selected: `const auto& brain = sc.brain(); const auto& ds = brain.state();`.
The `-Wno-deprecated-declarations` flag is retained for now because the unit
tests intentionally exercise the deprecated `set*` shims during the ongoing API
migration — removing the flag would surface ~30 expected deprecation usages
that are tracked separately.

### 1.4 Misleading comments in `bvr_engage.cpp`  *(doc fix)*

**File:** `digi/src/offensive/bvr_engage.cpp`

Three comments described behavior that the code does not implement or described
the wrong axis:

- `CrankManeuver`: comment said "If target is behind our 3/9 line" but the
  condition checks `rg.ataFrom > 90°` (we are behind the *target's* 3/9 line).
  Rewrote to correctly describe the ataFrom semantics.
- `DragManeuver`: comment said "turn to match the target's heading" but the
  code flies directly *away* from the target (along the target→self bearing).
  Rewrote to describe the actual cold-turn kinematics.
- `StickandThrottle`: the "Need speed — sacrifice altitude" comment advertised
  energy management that the code does not perform (the `speedDiff > 50` branch
  was a no-op). Rewrote to honestly describe the placeholder and added a TODO
  referencing the FF energy-management loop. Collapsed the empty `if` branch.

### 1.5 Dead `initialMissileBearing_` test member  *(dead-code removal)*

**File:** `tests/digi/scenarios/scenario_digi_defensive.cpp`

The field was written in `Init()` (comment: "retained for diagnostic printing")
but never read anywhere. Removed the write and the declaration.

---

## 2. Implementation status — what's REAL vs STUB

Cross-referenced against the FreeFalcon capability surface (~160 items across
26 modes). "Real" = ported with faithful behavior; "Simplified" = ported but
reduced in scope; "Stub" = mode resolves but falls through to Waypoint nav.

| Subsystem | Status | F4Flight LOC | FF LOC | Notes |
|---|---|---|---|---|
| Flight model (aero/EOM/FCS/engine/gear) | Real | — | — | Solid foundation, out of scope here |
| Maneuver primitives | Real | 811 | ~1000 | 18 primitives incl. PullToCollisionPoint, OverBank, RollOutOfPlane, WvrBugOut |
| GroundAvoid | Real | 140 | 304 | PullUp overlay + pullupTimer anti-jerk |
| MissileDefeat | Simplified | 290 | 732 | beam/drag/last-ditch; no full drag-trackpoint smoothing (missileShouldDrag/missileFinishedBeam placeholders retained) |
| GunsJink | Real | 245 | 331 | aspect-based roll offset, max-G pull |
| CollisionAvoid | Real | 112 | 139 | time-to-impact prediction + maneuver-plane dodge |
| WVREngage (RollAndPull) | Simplified | 259 | 612 | offensive/neutral/defensive branches; no EagManage/PullToControlPoint/MaintainClosure sub-controllers |
| GunsEngage | Real | 290 | 579 | FineGunsTrack with pipper/gravity-drop/fire-when-aligned |
| MissileEngage | Real | 267 | 488 | check + engage + FireControl shoot-shoot/shoot-look doctrine |
| BVREngage | **Simplified** | 308 | 3238 | 6 profiles (FF:18), 4 tactics (FF:24); no Notch/Pince/SSO/Grind/MAR/DOR/chooseRadarMode/spike/AiFlyBvrFormation |
| Merge | Simplified | 166 | 304 | no maneuverData table / CombatClass — picks 1/2-circle without class flags |
| Accel | Real | (in merge.cpp) | — | invert-and-pull |
| Takeoff/Landing | Real | 351 | 4778 | simplified ATC state machine; no preflight/canopy/taxi-collision/parking |
| ATC controller + TaxiGraph | Real | 179 + hdr | — | queue/clear/pathfind |
| Comms (MessageBus/Mailbox) | Real | hdr | — | FIFO, broadcast, group; **no wingman commands yet** |
| SensorFusion (radar/rwr/visual) | Real | 256 | 551 | threat scoring, best-target, sticky missile track |
| Weapons (SMS/FireControl/WeaponSpec) | Real | — | — | envelopes, off-bore limits, gimbal, shoot doctrines |
| Formation geometry (data only) | Real | 151 hdr | 114 | FormationTable + 3 defaults; **no AiFollowLead maneuver** |
| **Refueling** | Stub | 0 | 1030+2395 | falls through to Waypoint |
| **Separate** (full) | Partial | uses WvrBugOut | 471 | no RangeAtTailChase / bugoutTimer |
| **FollowOrders / Wingy** | Stub | 0 | ~9000 | entire wingman system unported |
| **RTB** (AirbaseCheck/bingo) | Stub | 0 | (in actions/separate) | falls through to Waypoint |
| **GroundMnvr** (A/G attack) | Stub | 0 | 4929 | entire gndattck.cpp unported |
| FAC brain / Tanker brain | n/a | 0 | 185+2395 | not present |

**Net:** ~13 of 26 modes are faithfully implemented, ~7 simplified, ~6 stubs.
The flight model + maneuver-primitive layer is excellent; the wingman, A/G, and
refueling subsystems are the big remaining gaps.

---

## 3. Structural changes needed before continuing the port

Ordered by priority. Items marked **[refactor]** are pure restructure with no
behavior change; items marked **[additive]** introduce new types/fields.

### 3.1 [refactor] Replace the flat `resolveMode` if-else chain with an explicit `AddMode`/`nextMode`/`curMode` priority stack  — **HIGH**

Currently `resolveMode` is a linear chain of `if (...) { activeMode_ = X; return; }`
checks. FreeFalcon's `AddMode` (dlogic.cpp:729-762) is a 3-slot stack
(`curMode`/`nextMode`/`lastMode`) with **interlock rules**:

- `BugoutMode` is sticky — cannot be bumped except by `MissileDefeatMode`.
- `LandingMode` cannot be bumped by WVR engagements once set.
- `LandingMode` ↔ `WVREngageMode` interlock (don't alternate each frame).
- A lower-priority mode already queued cannot be pre-empted by a higher one
  that arrives later in the same frame's check sequence.

The flat chain cannot express these interlocks. As `Separate`, `Bugout`,
`Refueling`, and the full `Landing` interactions land, the chain will either
grow special-case `if` guards (brittle, FF-smell) or silently drop interlocks
(behavior drift).

**Action:** introduce `DigiMode curMode_`/`nextMode_`/`lastMode_` members and a
private `addMode(DigiMode)` helper mirroring FF's interlock rules. Rewrite
`resolveMode` as a sequence of `*Check` calls that `addMode()` their result,
then a single `resolveModeConflicts()` that copies `nextMode_ → curMode_`.
`activeMode_` becomes `curMode_`. No behavior change for the currently-ported
modes (their checks don't trigger the interlocks); unblocks clean porting of
the interlocked modes.

### 3.2 [additive] `WingmanCommand` enum + `ReceiveOrders` dispatch  — **HIGH**

The `MessageBus`/`Mailbox` carry only ATC messages. FreeFalcon's wingman system
is message-driven (`FalconWingmanMsg` with ~92 commands → `ReceiveOrders` →
`mpActionFlags[7]`/`mpSearchFlags[3]`/`mDesignatedObject`/`mFormation`/
`mCurrentManeuver`). Without this, `Wingy`/`FollowOrders` cannot be ported.

**Action:** define `WingmanCommand` (start with the ~25 most-used: formations,
RTB, rejoin, weapons hold/free, engage, break, posthole, chainsaw, pince,
designate target, ECM/smoke/lights). Add a `WingmanOrder` message type to the
existing `MessageBus` and a `DigiBrain::receiveOrders(const WingmanOrder&)`
handler that populates a new `WingmanState` block in `DigiState`
(`actionFlags`, `searchFlags`, `designatedTargetId`, `formationId`,
`currentManeuver`, `weaponsAction`). This is the prerequisite for all wingman
work and the BVR profile commands (`WMPlevel1a`..`WMPDefensive`).

### 3.3 [additive] Dedicated `AgAttackPhase` enum — **HIGH, one-time decision**

FreeFalcon reuses the `WaypointState onStation` enum for **both** the landing
pattern **and** the A/G attack phases (NotThereYet/Crosswind/Downwind/Base/
Final/Final1/Stabalizing/HoldInPlace). This is FF quirk #19 — one variable,
three semantic roles, and it's a known source of confusion.

F4Flight currently has a clean `GroundOpsPhase` enum for takeoff/landing. Before
porting `gndattck.cpp`, decide the A/G phase representation.

**Action:** add a **separate** `AgAttackPhase` enum
(`NotThereYet, HoldInPlace, Crosswind, Downwind, Base, Final, Final1,
Stabalizing`) and an `agAttackPhase` field to `DigiState`. Do **not** overload
`GroundOpsPhase`. This avoids repeating FreeFalcon's mistake and keeps each
state machine legible. One-time structural decision; do it before any A/G code.

### 3.4 [additive] `CombatClass` + maneuverData table for Merge/BVR  — **MEDIUM**

FreeFalcon's `MergeManeuver` selects 1-circle/2-circle/slice/vertical/level-turn
based on `maneuverClassData[CombatClass].flags` (`CanLevelTurn`, `CanSlice`,
`CanUseVertical`, `CanOneCircle`, `CanTwoCircle`, …), indexed by a 9-class enum
(F4/F5/F14/F15/F16/Mig25/Mig27/A10/Bomber). F4Flight's `DigiEntity` has no
`combatClass`; the current `MergeManeuver` picks geometry without class flags.

**Action:** add `CombatClass combatClass` to `DigiEntity` (host populates from
its aircraft type table) and a small `maneuverClassData` table (9 entries,
hardcoded — no `mnvrdata.dat` file loader needed initially). Wire
`MergeManeuver` to read the flags. Also enables the 9×9 `maneuverData[][]`
BVR-intercept/merge-type table later.

### 3.5 [additive] Team/stance + spike tracking  — **MEDIUM**

FreeFalcon BVR depends on "spiked" detection (RWR says a radar is tracking us)
and team stance for IFF (hostile/friendly/neutral). F4Flight's `SensorFusion`
identifies missile/guns threats but has no spike concept and `DigiEntity` has
no team. This blocks faithful `Notch`, `Crank`-with-radar-support, and
`chooseRadarMode`.

**Action:** add `int team` (or `TeamId`) to `DigiEntity` + a stance helper. Add
a `SensorContact::Type::Spike` (RWR radar tracking self) to `SensorPicture` and
a `spiked`/`spikeEntity` accessor. Then `chooseRadarMode` and `Notch` can be
ported against the real RWR picture.

### 3.6 [additive] Per-target rate tracking (`atadot`, `rangedot` low-pass)  — **MEDIUM**

FreeFalcon maintains `atadot`/`rangedot` per-target in `SimObjectLocalData`,
used by `FineGunsTrack` fire control (the `atadot < 50°/s` fire gate),
`Stagnated` Luffberry detection, and merge geometry. F4Flight computes
`RelativeGeometry` on-demand (no rate terms). `DigiState` has `ataDot`/
`pastAta` but they're updated only for the single resolved target.

**Action:** add a small rate tracker to `SensorContact` (or a
`TrackedTargetRates` struct in `DigiState` updated each frame for the current
target): `atadot`, `rangedot`, low-passed. Prerequisite for `Stagnated`
detection and tighter guns fire control.

### 3.7 [refactor] Reduce AI-scenario parameterization inflation  — **MEDIUM**

The 91-fixture × 11-scenario matrix = 1001 ctest entries, ~77 s. The test audit
found ~10× coverage inflation: 63 fighter JSONs (mostly F-16 variants) run the
same class-aware predicates, so a regression that breaks only F-16BK50 (not
BK52) is not caught — both pass or both fail identically.

**Action:** add a CMake option `F4FLIGHT_FULL_AI_MATRIX` (default OFF). When
OFF, the 8 AI scenarios run against a 4-aircraft **representative class matrix**
(F-16BK50 fighter, B-52H bomber, A-10C attack, C-130 transport); the 3
flight-model scenarios keep the full 91-fixture matrix (they're cheap and
genuinely benefit from aero-table breadth). When ON, run the full 91×11 matrix
for nightly/CI deep runs. Cuts default ctest time ~6× as more AI scenarios are
added, without losing meaningful coverage.

### 3.8 [refactor] Wire up or delete `scenario_assertions.h`  — **LOW**

`tests/framework/scenario_assertions.h` is built into `f4flight_test_framework`
but `#include`d by zero scenario files; each scenario reimplements `hasNaN_`/
`minAlt_` checks inline (~6 copies). Either refactor the 5 scenario files to use
`PhaseAssertions::checkNaN()` + `noGroundPenetration()`, or delete the header.

### 3.9 [test] Tighten `RollAndPull` unit tests  — **MEDIUM**

5 of 7 `RollAndPullTest.*` tests use the vacuous
`hasCommand = (|pStick|>0.01 || |rStick|>0.01 || throttle>0.01)` pattern — any
non-trivial state passes (MachHold alone produces `throttle>0.01`). Replace with
direction-aware assertions like `scenario_digi_wvr.cpp` already uses (target
offset east → `rstick > +0.1`; target behind → `|pstick| > 0.3`). Important
before porting more WVR tactics so regressions are actually caught.

### 3.10 [test] Add scenario coverage for the next porting wave  — **HIGH, test-driven**

Per the test audit, these capabilities have **no** or smoke-only test coverage
and should get a scenario **before or alongside** their port:

- **BVR-intercept-to-missile-fire**: target at 25 NM head-on, AIM-120 loaded,
  assert BVREngage → MissileEngage transition ~15 NM → `releaseConsent` set at
  the right range.
- **Merge → Roop/OverB**: target at 8 NM head-on closing to 1 NM, assert Merge
  entry and an Over-Bank or Roll-Out-of-Plane maneuver.
- **MissileEngage end-to-end**: target at 5 NM, AIM-9 loaded, assert
  `releaseConsent` set ≥ 2 frames; SARH support-gate doesn't break lock.
- **CollisionAvoid**: two aircraft on a 45° beam collision course at 3 NM,
  assert separation increases, no NaN.
- **Formation/wingman-following**: lead at constant heading, AI in slot 1,
  assert formation geometry (range 1000 ± 200 ft, az ± 10°) over 60 s.
- **GroundAttack** (once 3.3 lands): IP → crosswind → final → weapon release.

---

## 4. Recommended porting order for the next phase

| Phase | Work | Unblocks |
|---|---|---|
| **3a (structural prep)** | 3.1 AddMode refactor, 3.2 WingmanCommand enum, 3.3 AgAttackPhase enum | All subsequent phases |
| **3b (wingman)** | Port `AiFollowLead` (Wingy mode), `FollowOrders` dispatch, 15 formation types from `formdata.cpp` | Formation flying, BVR formation tactics, player commandable formations |
| **3c (BVR depth)** | 3.4 CombatClass+maneuverData, 3.5 team/spike, `chooseRadarMode`, `Notch` tactic, MAR/DOR, remaining 12 BVR profiles | Faithful BVR playbook |
| **3d (A/G attack)** | Port `gndattck.cpp` using 3.3's `AgAttackPhase`: `SetupAGMode`, NotThereYet→Final1 state machine, per-weapon delivery (bomb/GBU/Maverick/HARM/rocket/gun strafe) | Full ground-attack missions |
| **3e (support modes)** | Refueling (`refuel.cpp`+`tankbrn.cpp`), RTB (`AirbaseCheck`+bingo), full Separate (`RangeAtTailChase`+`bugoutTimer`) | Mission completeness |

Phase 3a is pure refactor + enum additions — zero behavior change, low risk,
and unblocks the most downstream work. Recommend doing 3a + 3.10's first two
scenarios (BVR-to-missile-fire, Merge) as the immediate next step, since those
scenarios will exercise the AddMode refactor and reveal any interlock issues
early.

---

## 5. Things deliberately NOT replicated from FreeFalcon (FF quirks to avoid)

Carried forward from the capability map's "stupid things" list. F4Flight
should NOT port these:

- **`GroundMnvrMode` dead enum** — FF declares it but never dispatches. F4Flight
  already has it as a stub; keep it stubbed or remove the enum value.
- **`threatTime`/`targetTime` stale fields** — FF computes them but
  `TargetSelection` only reads `threatScore`. F4Flight's `SensorFusion` is
  already score-only; stay that way.
- **`HoldCorner` always-true hack** — FF's `return true;` disables corner-speed
  protection. Don't port the hack; implement the real check if needed.
- **`onStation` enum overload** (landing + A/G) — see 3.3; use a separate enum.
- **`agmergeTimer` triple-role** (loiter time + needs-init + mission-complete) —
  split into distinct fields when porting A/G.
- **`missileShotTimer` triple-role** (next shot time + weapons-hold release +
  GroundAttack hack) — split into distinct fields.
- **`AI_EXECUTE_MANEUVER == TRUE+1` magic sentinel** — use an explicit enum.
- **`HandleThreat` trivial wrapper** — FF's just calls `WvrEngage`; inline it.
- **`TailChaseRMaxNe` always-returns-6-NM** — implement the real stern-chase
  geometry or omit the function.
- **`F4IsBadReadPtr` defensive guards** — these mask real bugs; rely on clean
  pointer ownership (F4Flight's host-owned `DigiEntity*` model already does).
- **`=` vs `==` debug-label assignment bugs** (FF actions.cpp:380, wvrengage.cpp:436)
  — don't copy-paste.

---

## 6. Round-3 fix summary

| # | Fix | Files | Type |
|---|---|---|---|
| 1.1 | Wire `BvrEngageCheck` into `resolveMode`; fix 35→45.5 NM engage range | `digi/src/digi_brain.cpp`, `tests/digi/unit/test_digi_offensive.cpp` | behavioral + dead-code |
| 1.2 | Remove dead `runGroundAvoid`; document `GroundAvoid` no-op case | `digi/include/f4flight/digi/digi_brain.h`, `digi/src/digi_brain.cpp` | dead-code |
| 1.3 | Use const `state()` overload in scenario framework | `tests/framework/scenario_framework.cpp` | warning |
| 1.4 | Correct misleading BVR comments (Crank/Drag/StickandThrottle) | `digi/src/offensive/bvr_engage.cpp` | doc |
| 1.5 | Remove dead `initialMissileBearing_` test member | `tests/digi/scenarios/scenario_digi_defensive.cpp` | dead-code |

**Verification:** clean build (no warnings), **1369/1369 tests pass**.
