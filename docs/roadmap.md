# Robot_Simulation Roadmap

Actionable future work for this repository.

- Items move here from `Agent_Session_Notes.md` (in-progress/deferred).
- When an item is completed, move it to `docs/project_history.md` and remove it from this file.
- Cross-repo items that involve SmartDashboard are marked with **(cross-repo)**.

---

## Active: Autonomous goal system — remaining items

### Write a "cool real autonomous goal" for the Auton chooser

The Auton chooser currently only has "Do Nothing" and "Just Move Forward". A more interesting autonomous sequence should be written as a fun task (user's words). This could be a multi-waypoint path, a timed scoring sequence, or any creative autonomous routine.

### Runtime verification of Test chooser workflow

The SetIntendedPosition-once oscillation fix is confirmed working. The first-enable initialization issue has been fixed: Ship1D_Legacy.h now initializes all members with C++11 default member initializers and clamps `dTime_s` to 50ms in `TimeChange()`. Remaining verification:
1. ~~Fix and verify the first-enable oscillation in Debug builds (Ship1D_Legacy.h)~~ **DONE** — member initializers + dTime_s clamp applied, all 12 unit tests pass
2. Selecting Test mode populates the Test chooser widget immediately
3. Picking a drive test (e.g. "Just Rotate") and pressing Enable runs that test
4. With ExcavatorArm enabled, arm goals appear in the chooser and execute correctly
5. Switching manipulator to "None" and relaunching correctly hides arm goals

---

## Active: Command/Subsystem status display (Need #6)

### Phase 2 — Command/Subsystem scheduler layer (COMPLETED)

Publish WPILib-compatible Command/Subsystem NT keys so SmartDashboard (ours and official) can display subsystem/command status.

**Completed work:**

- **Layer 1 — NT4Server topic properties**: Added `propertiesJson` to `TopicInfo`, updated all 3 announce JSON construction sites (`BuildAnnounceJson`, `SendMatchingTopicsToClient`, publish handler) to emit topic properties, added `SetTopicProperties()` public API with `NT4TypeHint` enum, plumbed through SmartDashboard→sink→NT4Backend→NT4Server chain.
- **Layer 2 — Manipulator CommandScheduler**: Embedded lightweight scheduler in ExcavatorArm (per ManipulatorPlugin.h line 8-9 note). `CommandBehavior` struct with callback-based commands. Three commands adapted from Curivator goals: MoveArmToPosition, ArmGrabSequence (4-phase hover→approach→scoop→retract), HomeArm. Publishes Scheduler/Subsystem/Command NT keys each frame via `PublishSchedulerTelemetry()`. Wired into ExcavatorArm lifecycle (Init→InitScheduler, TimeSlice→UpdateScheduler, PublishTelemetry→PublishSchedulerTelemetry).

**NT key patterns published** (under `/SmartDashboard/`):
- **Scheduler**: `.type`=`"Scheduler"`, `Names`=string[]  (Ids/Cancel deferred — need PublishIntArray plumbing)
- **Subsystem**: `.type`=`"Subsystem"`, `.hasDefault`=bool, `.default`=string, `.hasCommand`=bool, `.command`=string
- **Command**: `.type`=`"Command"`, `/running`=bool
- `.type` topics carry property JSON `{"SmartDashboard":"<type>"}` set via `SetTopicProperties()` once in `InitScheduler()`

**Key discoveries:**
- No `PublishIntArray` exists — `NT4Type::IntArr=18` defined but no publish method or retained storage. Scheduler Ids/Cancel deferred.
- ExcavatorArm.cpp is not in any vcxproj — compiled indirectly via header inclusion from TeleAutonV2.cpp.
- Pre-existing C++17 `const inline` issue fixed (ExcavatorArm.h lines 83-84).

### Phase 3 — Test against official SmartDashboard 2026 **(cross-repo)**

Verify that the Command/Subsystem NT keys are correctly consumed by the official WPILib SmartDashboard. Fix any protocol gaps.

### Phase 4 — Implement Command/Subsystem widgets in our SmartDashboard **(cross-repo)**

Build the visual widgets in our SmartDashboard that display subsystem status, active commands, and the scheduler state.

### Runtime verification (outstanding)

Build `RobotAssembly_static` and `DriverStation`, verify j/k/;/l/i/u/o/p keys move the arm correctly in the side-view visualization. Unit tests pass but runtime visual test has not been done.

---

## Deferred work

Lower-priority items parked for future consideration.

- Expand smoke test published keys from ~6 + chooser to full TeleAutonV2 (~49 keys)
- Debug builds: manual carrier picker in DriverStation dialog for SHM vs TCP comparisons
