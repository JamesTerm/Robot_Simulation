# 2026-04-03 — Generic_CompositeGoal AutoActivate ping-pong fix

## Summary

Fixed an infinite ping-pong bug in the autonomous goal system where the robot endlessly alternated between waypoint 1 and (0,0) without progressing through its full 16-subgoal sequence.  Two root causes were identified and fixed:

1. **Tolerance mismatch** — the goal layer's waypoint tolerance was not passed to the motion controller, so the two independent `HitWayPoint()` checks could disagree.
2. **AutoActivate=false on composite subgoals** — `ArmGrabSequence` and `ClawGrabSequence` used the default `AutoActivate=false`, causing `Generic_CompositeGoal::Process()` to return `eInactive` forever, which propagated up and triggered infinite re-Activation of the parent goal.

Root cause #2 was the actual ping-pong bug.  Root cause #1 was a contributing factor that made waypoint hits unreliable.

## Root cause analysis: the AutoActivate bug

### How Generic_CompositeGoal works

`Generic_CompositeGoal` (Goal_Types.h) has an `m_AutoActivate` flag (default `false`).  Its `Process()` method:

```cpp
virtual Goal_Status Process(double dTime_s) {
    if (m_AutoActivate)
        ActivateIfInactive();       // <-- only if AutoActivate=true
    if (m_Status == eInactive)
        return m_Status;            // <-- returns eInactive if never activated
    if (m_Status == eActive)
        m_Status = ProcessSubgoals(dTime_s);
    return m_Status;
}
```

When `AutoActivate=false`, the class expects the **client** (whoever created the goal) to call `Activate()` explicitly.  But when a `Generic_CompositeGoal` is used as a **subgoal** inside another composite, `ProcessSubgoals()` only calls `Process()` on the front subgoal — it never calls `Activate()`.  So the subgoal stays `eInactive` forever.

### The chain of failure

1. `ArmGrabSequence` extends `Generic_CompositeGoal` with default `AutoActivate=false`
2. `ArmGrabSequence::Activate()` pushes 6 arm subgoals and sets `m_Status = eActive` — but it's never called
3. `ArmGrabSequence::Process()` (inherited from `Generic_CompositeGoal`) returns `eInactive` every frame
4. Parent `AutonGrabAndReturn::Process()` does `m_Status = ProcessSubgoals(dTime_s)` → gets `eInactive`
5. Next frame: `AutonGrabAndReturn::Process()` calls `ActivateIfInactive()`, sees `eInactive`, calls `Activate()`
6. `Activate()` pushes all 16 subgoals AGAIN on top of existing ones
7. Subgoal count jumps: 10→25→34→... Robot ping-pongs between WP1 and (0,0)

### Evidence from diagnostic dump

The `OutputDebugStringA` trace captured in `D:\temp\SmartDashboard\AutonDumpTest01.txt` proved the bug conclusively:

- Lines 103-104: `subgoals=10` → `subgoals=25` (jump of +15 = 16 new subgoals minus 1 just completed)
- Lines 320-321: `subgoals=19` → `subgoals=34` (another +15 push)
- Pattern: Activate WP1 → COMPLETED → Activate (0,0) → COMPLETED → Activate WP1... only ever alternating between these two coordinates

### The fix

Pass `AutoActivate=true` to the `Generic_CompositeGoal` base constructor:

```cpp
// ArmGrabSequence constructor
ArmGrabSequence(...)
    : Generic_CompositeGoal(true)   // <-- was default (false)
    , ...
```

Applied to both `ArmGrabSequence` and `ClawGrabSequence` in `ExcavatorGoals.h`.

### Audit of all Generic_CompositeGoal subclasses

| Class | File | AutoActivate | Status |
|-------|------|-------------|--------|
| `Goal_ArmHoldStill` | ExcavatorGoals.h | `true` | Already correct |
| `Goal_SetArmPosition` | ExcavatorGoals.h | `true` | Already correct |
| `ArmGrabSequence` | ExcavatorGoals.h | `true` | **FIXED** |
| `ClawGrabSequence` | ExcavatorGoals.h | `true` | **FIXED** |
| `AutonGrabAndReturn` | ExcavatorGoals.h | `false` | OK — has own `ActivateIfInactive()` in `Process()` override |
| `MoveForward` | AI_Input_Example.cpp | parameterized | OK — sets `eActive` in ctor when `false` |
| `RotateWithWait` | AI_Input_Example.cpp | parameterized | OK — sets `eActive` in ctor when `false` |
| `TestMoveRotateSequence` | AI_Input_Example.cpp | `false` | OK — sets `eActive` in ctor |

### The rule

Any `Generic_CompositeGoal` subclass that:
1. Overrides `Activate()` to push subgoals, AND
2. Does NOT override `Process()` to call `ActivateIfInactive()`, AND
3. Is used as a subgoal inside another composite

**MUST** pass `AutoActivate=true` to the base constructor.  Otherwise `Activate()` will never be called and `Process()` will return `eInactive` forever.

The legacy Curivator's `SetArmWaypoint` (Curivator_Goals.cpp:661) worked around this by calling `Activate()` directly in its constructor.  The modern fix uses `AutoActivate=true` which is cleaner — it lets the goal framework handle activation timing correctly.

## Root cause #1: tolerance mismatch (supporting fix)

The goal layer's `safestop_tolerance` parameter was not passed through the 7-layer `DriveToLocation` call chain to the motion controller's `DriveTo_Controller`.  The goal checked `HitWayPoint()` with its own tolerance (e.g. `Feet2Meters(3.0)`), but the controller used a hardcoded default of `Feet2Meters(1.0)`.

Fixed by threading `double safestop_tolerance` through all 7 layers:
- `AI_BaseController_goals.h` → `AI_Input.h/.cpp` → `SwerveRobot.h/.cpp` → `MotionControl2D.h` → `MotionControl2D_physics.cpp`

## Files changed

| File | Change |
|------|--------|
| `ExcavatorGoals.h` | `ArmGrabSequence`: `Generic_CompositeGoal(true)`; `ClawGrabSequence`: same fix; removed DIAG OutputDebugString from `AutonGrabAndReturn::Process()` |
| `AI_BaseController_goals.h` | Tolerance threading to DriveToLocation; removed DIAG OutputDebugString blocks and `<Windows.h>` include |
| `Goal_Types.h` | Removed temporary `GetSubgoalCount()`/`GetFrontSubgoal()` diagnostic accessors |
| `TeleAutonV2.cpp` | Fixed misleading indentation at Start() lines 869-872 |
| `AI_Input.h/.cpp` | Tolerance parameter in DriveTo_proto and DriveToLocation |
| `SwerveRobot.h/.cpp` | Tolerance parameter pass-through |
| `MotionControl2D.h` (physics) | Tolerance parameter in DriveToLocation |
| `MotionControl2D_physics.cpp` | Tolerance parameter in DriveTo_Controller |
| `TeleAutonV1.cpp` | Parallel tolerance parameter |
| `MotionControl2D.h/.cpp` (simple) | Interface parity — param accepted, ignored |

## Verification

- DriverStation.exe builds clean (Release)
- 29/30 unit tests pass (1 pre-existing TCP flake)
- User confirmed: auton runs to completion, subgoal count decreases monotonically, no ping-pong
