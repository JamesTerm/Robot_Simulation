# Agent session notes

- Keep this file short and handoff-focused.
- For remaining work and deferred items: see `docs/roadmap.md`.
- Move durable milestone history to `docs/project_history.md`.
- Use `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md` as the canonical long-term Native Link rollout note.

## Workflow

- `apply_patch` expects workspace-relative paths with forward slashes.
- **Use CRLF line endings** for all source files (`.cpp`, `.h`, `.cmake`, `.ps1`, `.py`, `.md`, `.gitignore`, `.json`). Both repos standardized CRLF as of the NT4 transport merge.
- **Do NOT CRLF-normalize `.rc` or `.aps` files** — they are UTF-16 LE encoded. `.gitattributes` marks them `binary`. See "Key invariants" for the full story.
- Read nearby `Ian:` comments before editing and add new ones where session, ownership, carrier, protocol, or runtime-mode lessons would be easy to lose.
- Process detection: use `Get-Process -Name <name> -ErrorAction SilentlyContinue` (NOT `tasklist | findstr` which breaks in Git Bash due to flag mangling).
- Killing processes: use PowerShell `Stop-Process` (NOT `taskkill` through Git Bash).

## Build

```bash
# Configure (uses existing build-vcpkg directory)
cmake -G "Visual Studio 17 2022" -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE="D:/Git/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows

# Build
cmake --build build-vcpkg --target DriverStation --config Release
cmake --build build-vcpkg --target DriverStation_TransportSmoke --config Release

# Run DriverStation in NetworkTables V4 mode (serves Shuffleboard, Glass, any NT4 dashboard)
Start-Process -FilePath 'D:\code\Robot_Simulation\build-vcpkg\bin\Release\DriverStation.exe' -ArgumentList 'nt4' -WorkingDirectory 'D:\code\Robot_Simulation\build-vcpkg\bin\Release'

# Run smoke test
build-vcpkg\bin\Release\DriverStation_TransportSmoke.exe --mode nt4
```

Note: The ixwebsocket overlay port only matters at `vcpkg install` time, not at CMake configure time. The patched library is already in vcpkg's `installed/` directory.

**Windows Firewall:** The first time DriverStation.exe listens on a new port, Windows will prompt to allow it. This is a one-time approval per port.

## Key invariants (do not break)

- `Core::PublishInternal` auto-register only applies when `allowServerOnly=true`. Client writes on unknown topics must still be rejected.
- `RegisterDefaultTopics` must not grow to list every TeleAutonV2 key — the auto-register path handles those.
- `TeleAutonV2` publishes ~26 keys per loop. `Heading` (line 324) and `Travel_Heading` (line 307) are unconditional.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.
- **`HasDirectTransport()` and `UsesNetworkTablesTransport()` in SmartDashboard.cpp must include every mode that uses a `DirectPublishSink`/`DirectQuerySource`.** Missing a mode causes `PutNumber`/etc. to fall through to `NetworkTable::GetTable()` which starts the legacy NT2 server on port 1735 alongside the intended backend. This was the root cause of the Shuffleboard port bug (port 1735 instead of 5810).
- **`.rc` files must never be CRLF-normalized by git.** They are UTF-16 LE (BOM `FF FE`). Git's CRLF normalization treats them as single-byte text, mangling `0D 00 0A 00` to `0D 0A 00 0D 0A 00`, destroying UTF-16 alignment. `rc.exe` then silently produces an empty `.res` (32 bytes), and `CreateDialogW` returns NULL with `ERROR_RESOURCE_TYPE_NOT_FOUND` (1813). `.gitattributes` now marks `*.rc` and `*.aps` as `binary`.
- **`NT4Server::SetTopicProperties` uses a `propertiesJson` string**, not `nlohmann::json`, in the header to avoid pulling nlohmann into `NT4Server.h`. The `.cpp` parses it with `json::parse(topic.propertiesJson, nullptr, false)`. The `NT4TypeHint` public enum maps to the private `NT4Type` enum internally.
- **ExcavatorArm scheduler state is per-instance** — `m_activeCommandIndex`, `m_commandRunning`, `m_grabPhase` etc. are private members. The scheduler is initialized once in `Init()` and publishes each frame in `PublishTelemetry()`. Commands are registered in `InitScheduler()` with `std::function` callbacks, not virtual methods.
- **"Reset fresh" config self-cleaning (April 2026).** Both DriverStation and SmartDashboard validate persisted settings at startup. If the registry `ConnectionMode` is missing or out of the valid enum range, DriverStation logs a warning, writes the clean default (`eDirectConnect`) back to the registry, and starts with DirectConnect. SmartDashboard validates `transportKind` (must be 0–2) and `pluginSettingsJson` (must be parseable JSON if non-empty). If either is corrupt, SmartDashboard nukes ALL connection settings from the registry, resets to DirectConnect defaults, and persists the clean state. The operator sees DirectConnect in the UI, signaling "something was wrong — reconfigure." Root cause: mangled JSON from PowerShell registry writes silently broke the NativeLink plugin; stale `ConnectionMode=1` silently prevented the NativeLink TCP server from starting. Neither failure was visible. Now both are caught and cleaned up automatically.
- **Two choosers, two lifecycles.** Auton chooser (`Autonomous/Auton_Selection/AutoChooser`) is seeded by `PublishAutonChooserOptions()` when `SetGameMode(eAuton)` fires (and by TransportSmoke for smoke tests). Test chooser (`Test/Test_Selection/TestChooser`) is seeded by `PublishTestChooser()` when `SetGameMode(eTest)` fires. `ActivateTest()` reads the user's `selected` key later at Enable time. Never publish `selected=true` in `ActivateTest` or it will overwrite the user's pick.
- **ManipulatorPlugin test goal coupling** — plugins contribute test goals via `GetTestGoals()`/`CreateTestGoal()` virtual methods. The AI goal system never hard-codes manipulator-specific enums. Plugin goal indices start at `eNoTestDriveTypes` (6). When `manipulatorPlugin` is null, only drive goals appear and stale plugin labels fall back to "Do Nothing" via `ResolveAutonSelectionFromChooser`.
- **ManipulatorPlugin auton goal coupling** — same loose-coupling pattern as test goals, but for the Auton chooser. Plugins contribute auton routines via `GetAutonGoals()`/`CreateAutonGoal()` virtual methods. `CreateAutonGoal()` receives an `AI_Input*` controller so plugins can create drive goals (waypoints) combined with arm sequences. Plugin auton indices start at `eNoAutonTypes` (2). Both choosers are now dynamic: `BuildAutonChooserOptions(plugin)` and `BuildTestChooserOptions(plugin)` merge drive + plugin options at runtime. ExcavatorArm contributes "Grab and Return" — a triangle-pattern drive+grab autonomous.
- **SetIntendedPosition-once pattern** — Position goals on Ship_1D must call `SetIntendedPosition()` **ONCE** in `Activate()` and only monitor convergence in `Process()`. Calling it every frame resets `m_LockShipToPosition = false`, which restarts the position-tracking physics and prevents convergence. This matches the Curivator's `Goal_Rotary_MoveToPosition` (Common/Rotary_System.cpp:1210-1266). Our analog is `Goal_Ship1D_MoveToPosition` in `ExcavatorGoals.h`.
- **Ship_1D first-enable burst (Debug, FIXED)** — On first test goal activation in Debug builds, the arm could oscillate for a few seconds due to three factors: (1) uninitialized `m_SimFlightMode` causing UB in the `SetSimFlightMode()` guard, (2) no `dTime_s` clamping allowing Euler physics overshoot from large timesteps, (3) uninitialized member variables (`m_LockShipToPosition`, `m_Last_RequestedVelocity`, `m_Mass`, etc.). All three are now fixed via C++11 member initializers and a 50ms `dTime_s` clamp at the top of `Ship_1D::TimeChange()`. See Ship1D_Legacy.h lines 245-271 (initializers) and 310-320 (clamp). All 12 ExcavatorArm unit tests pass.
- **`#include` inside a namespace block** is a latent hazard in this codebase. ExcavatorArm.cpp opens `namespace Module { namespace Robot {` at line 20. Any `#include` below that point pulls standard headers inside the namespace, causing `Module::Robot::std::ratio` etc. Always include goal/framework headers at the top of the file, outside any namespace.
- **Generic_CompositeGoal AutoActivate requirement (FIXED)** — Any `Generic_CompositeGoal` subclass that overrides `Activate()` to push subgoals AND is used as a subgoal inside another composite MUST pass `AutoActivate=true` to the base constructor. Without it, `ProcessSubgoals()` calls `Process()` on the front subgoal, but `Generic_CompositeGoal::Process()` only calls `ActivateIfInactive()` when `m_AutoActivate` is true. An unactivated subgoal returns `eInactive`, which propagates up and kills the entire goal sequence. This was the root cause of the auton waypoint ping-pong bug — `ArmGrabSequence` and `ClawGrabSequence` used default `AutoActivate=false`, causing the parent `AutonGrabAndReturn` to re-Activate and re-push all 16 subgoals every frame. See `docs/journal/2026-04-03-autoactivate-ping-pong-fix.md` for the full investigation.
- **ArmGrabSequence behavior parity (ExcavatorGoals.h)** — Four fixes ported from the legacy Curivator to close the gap between modern and legacy arm behavior:
  - **Per-axis tolerance (0.15):** Replaced hardcoded `tolerance=0.60` with per-axis constants `kDefaultTolerance_XPos/YPos/BucketAngle/ClaspAngle = 0.15`, matching CurivatorRobot.lua v1.14. The legacy `tolerance` key in lua maps to `Rotary_Props::PrecisionTolerance` (Rotary_System.cpp:1059). V1.0 used 0.6 from `arm_pos_common`; v1.14 tightened all four to 0.15. `Move_ArmAndBucket` now accepts separate tolerances per axis.
  - **RobotArmHoldStill (0.6s settle):** Created `Goal_ArmNotify` (fires a callback) and `Goal_ArmHoldStill` (composite: LockPosition→Wait(0.4)→FreezeArm→Wait(0.2)→UnfreezeArm→UnlockPosition). `Goal_SetArmPosition::Activate()` now adds `Goal_ArmHoldStill` AFTER `Move_ArmAndBucket` in LIFO order, matching legacy `SetArmWaypoint::Activate()` (Curivator_Goals.cpp:691-693). Callbacks capture `ExcavatorArm::m_FreezeArm` and `m_LockPosition` booleans via lambdas from `CreateTestGoal()`.
  - **Bucket speed override (0.5):** ArmGrabSequence step 5 (scoop: bucket 90°→40°) now passes `bucketSpeed=0.5`, matching legacy line 825 `bucket_Angle_deg_speed=0.5`.
  - **BucketAngleContinuity UI warning:** Added `BucketAngleContinuity` field to `ManipulatorArm_UI::ArmState`, computed in `SetupUI` callback as `|desired_bucket_angle - FK_actual_bucket_angle|` (matches legacy `GetBucketAngleContinuity()`, Curivator_Robot.cpp:995). `LinesUpdate::update()` in OSG_Viewer.cpp now blends vertex colors [7] (bucket tip: steel blue→red) and [8] (bucket angle: peach→red) with `errorRatio = min(continuity / 3.0, 1.0)`, matching legacy toleranceScale=3.0 degrees.

## Strategy

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.
- **Support, not imitate** — give teams enough compatibility to try SmartDashboard with each dashboard tool, then encourage Native Link for full support.

## Transport architecture

See `Transport.h` for the `Ian:` comment listing all files that must be updated when adding a new transport mode.

Current modes:
| Mode | Enum | Backend | Port | Status |
|---|---|---|---|---|
| Legacy SmartDashboard | `eLegacySmartDashboard` | NT v2 TCP | 1735 | Stable |
| Direct | `eDirectConnect` | Shared memory | N/A | Stable |
| NetworkTables V4 | `eNetworkTablesV4` | NT4 WebSocket | 5810 | Stable (serves Shuffleboard, Glass, any NT4 dashboard) |
| Native Link | `eNativeLink` | Custom TCP/SHM | 5810 | Stable |

### NT4 server

`NT4Server.h/.cpp` (~1200 lines) — full NT4 WebSocket server: subscription-driven architecture, custom MsgPack encoder/decoder, JSON announce, retained-value cache, per-client subscription tracking, client value write-back with pubuid->topicId resolution and re-broadcast to all subscribers (including sender).

### DriverStation automation

- `dsctl.ps1` — sends `WM_COMMAND` to DriverStation dialog. Commands: `start`, `stop`, `auton`, `tele`, `test`, `auton-enable`.
- Hotkeys: F5=Legacy, F6=Direct, F7=NetworkTables V4, F8=NativeLink.
- Control IDs: `IDC_Start`=1006, `IDC_Stop`=1005, `IDC_Auton`=1003, `IDC_Tele`=1002, `IDC_Test`=1004.

### Video source modes

| Mode | Enum | Description |
|---|---|---|
| Off | `VideoSourceMode::eOff` | No camera stream; MJPEG server torn down |
| Camera | `VideoSourceMode::eCamera` | USB webcam via VFW (Video for Windows) |
| Synthetic Radar | `VideoSourceMode::eSyntheticRadar` | Existing SimCameraSource radar sweep (default) |
| The Grid | `VideoSourceMode::eVirtualField` | Tron-style first-person virtual field camera (TronGridSource) |

Video source is selected via "Video" combo in DriverStation dialog. Persisted to `[Video] Source=<int>` in DriverStation.ini.

## Cross-repo sync

- This repo and `D:\code\SmartDashboard` share the Native Link contract, carrier implementations, and plugin boundary.
- When either repo's session notes change, check the other side for consistency.

## Remaining work

See `docs/roadmap.md` for all remaining work and deferred items.

## BroncBotz Legacy Simulation Build (broncbotz_code)

**Status: FULLY BUILDABLE** as of 2026-04-01.

The legacy BroncBotz FRC robot simulation (originally VS2008) has been ported to a modern CMake build in `D:\code\broncbotz_code`. Purpose: debug the Ship_1D first-enable oscillation bug documented in "Key invariants" above.

### Build commands

```bash
# Configure (one-time, or after CMakeLists.txt changes)
cmake -S D:\code\broncbotz_code -B D:\code\broncbotz_code\build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake

# Build all (Debug x64)
cmake --build D:\code\broncbotz_code\build --config Debug

# Build just Robot_Tester
cmake --build D:\code\broncbotz_code\build --config Debug --target Robot_Tester
```

Output: `D:\code\broncbotz_code\build\Debug\Robot_Tester.exe` (console app) with .lua scripts copied alongside.

### Architecture

- 9 targets: xmlParser, GG_Framework_Base, GG_Framework_UI_OSG, GG_Framework_UI_Audio, GG_Framework_Logic_Scripting, GG_Framework_UI, Robot_Common, Robot_Drive, Robot_Tester
- All static libraries except Robot_Tester (console EXE)
- vcpkg: OSG 3.6.5 only. Lua 5.1 is embedded (LUA.h/LUA.cpp amalgamation)
- FMOD stubs in stubs/fmod/ (Audio compiles but is mock/no-op at runtime)
- SmartDashboard: **NT4 server enabled** via `NT4SmartDashboard.h` (self-contained, header-only, ~1150 lines). Replaces the old `__DisableSmartDashboard__` stub. Robot_Tester acts as the **robot** (server on port 5810); dashboard clients (SmartDashboard, Shuffleboard, Glass) connect to it. Uses ixwebsocket (vcpkg). See "NT4 SmartDashboard Server" section below.

### Source modifications from legacy (summary)

1. `GG_Framework.UI.Audio.h` — Added `#include "fmod.hpp"` for FMOD stub visibility
2. `Action.h` — Removed `const` from `m_actorScene` and `m_eventMap` pointers (C++17 std::list requires copy-assignable elements)
3. `JoystickBinder.cpp` — Replaced MSVC-internal `_Mynode()` with `end()` comparison; initialize iterator array to `end()`
4. `TestText.cpp` — `getBound().xMax()` → `getBound().center().x() + getBound().radius()` (OSG 3.x BoundingSphere API)
5. `Curivator_Robot.cpp` — `BIND_PER_PRIMITIVE` → `BIND_PER_PRIMITIVE_SET` (removed in OSG 3.x)
6. `CMakeLists.txt` — Removed 5 dead Producer-dependent .cpp files from GG_Framework_UI (SceneRunner, PostDrawCallback, KeyboardMouseCallback, Viewer, ViewPortReference — not included by GG_Framework.UI.h, no Robot code references them)
7. Earlier fixes: LoadSave.h/cpp windows.h+std::max, GG_Framework.Base.h missing include, osg::CameraNode→Camera, UI.OSG.h missing includes, Thread.h AtomicT template, Misc.cpp dynamic_cast, Common.h SmartDashboard stub, Logic.Scripting.h standalone Lua

### Dead code not built

5 files in GG_Framework/UI/ use the removed `Producer::` windowing API (pre-OSG 2.x). They are NOT included by `GG_Framework.UI.h` and no Robot code references them. They remain in the source tree but are excluded from CMake:
- `PostDrawCallback.cpp/.h` (Producer::Camera::Callback)
- `KeyboardMouseCallback.cpp/.h` (Producer::KeyboardMouseCallback)
- `SceneRunner.cpp/.h` (Producer::Trackball, KeyChar_Escape)
- `Viewer.cpp/.h` (in UI dir — distinct from Robot_Tester/Viewer.cpp)
- `ViewPortReference.cpp/.h` (Producer::Matrix)

### NT4 SmartDashboard Server — COMPLETE (2026-04-01)

**Goal:** Replace the disabled SmartDashboard stub with a live NT4 (NetworkTables v4.1) WebSocket **server**, so Robot_Tester acts as the robot (like a real FRC robot running ntcore) and dashboard clients (SmartDashboard, Shuffleboard, Glass, OutlineViewer) can connect to it on port 5810.

**Architecture (CORRECTED):**
- **Robot_Tester = ROBOT = NT4 SERVER** listening on `ws://0.0.0.0:5810/nt/<clientname>`
- **SmartDashboard / Shuffleboard / Glass = DASHBOARD = CLIENT** connecting to Robot_Tester on port 5810
- This matches how real FRC robots work: the robot runs ntcore (server), dashboards connect to it.
- Previous implementation was backwards (Robot_Tester as client connecting to DriverStation). Corrected 2026-04-01.

**Design choice:** Self-contained header-only implementation (`NT4SmartDashboard.h`, ~1150 lines) embedded directly in broncbotz_code. Only new external dependency: ixwebsocket (already compiled in shared vcpkg at `D:\code\vcpkg\installed\x64-windows\`). No nlohmann_json — JSON is hand-crafted with snprintf. Uses `ix::WebSocketServer` (not `ix::WebSocket`).

**Key files changed:**
- `Source/Robot_Simulation/Common/NT4SmartDashboard.h` — **CREATED then REWRITTEN**: Complete NT4 server (~1150 lines)
- `Source/Robot_Simulation/Common/Common.h` — `__DisableSmartDashboard__` commented out, `#include "NT4SmartDashboard.h"` added
- `CMakeLists.txt` — `find_package(ixwebsocket)`, linked to Robot_Common (PUBLIC); `bcrypt` linked to Robot_Tester (mbedtls dep); `/FI"winsock2.h"` force-include to resolve winsock conflicts globally

**Server implementation details:**
- `ix::WebSocketServer` listens on `0.0.0.0:5810`
- **Subscription-driven** — server does NOT send announces until client sends subscribe (LESSON LEARNED: ntcore silently ignores pre-subscribe announces)
- **RTT ping handling** — server MUST respond to client RTT pings by echoing the client timestamp back, or ntcore blocks ALL outgoing messages
- **Server-assigned topic IDs** in announce messages (sequential, starting from 0)
- **Retained value cache** — newly subscribing clients get full snapshot of all matching topics
- **Per-client state** — subscription tracking (prefix patterns), announced topics set, pubuid→topicId mapping
- **Client value write-back** — handles client "publish" JSON + binary value frames, updates retained cache, re-broadcasts to other subscribers
- **Hand-parsed JSON** for subscribe/publish messages using `string::find`-based extraction (no nlohmann dependency)
- MsgPack encoder (WriteArrayHeader, WriteUInt, WriteInt, WriteDouble, WriteBool, WriteString) — copied from Robot_Simulation's NT4Server.cpp
- MsgPack reader (ReadCursor) — for decoding incoming binary value frames from clients
- SmartDashboard singleton class matching the original stub API: init(), shutdown(), SetClientMode(), SetIPAddress() (both no-ops in server mode), PutNumber/GetNumber, PutBoolean/GetBoolean, PutString/GetString, SetDefaultNumber, SetDefaultBoolean

**Build issues resolved:**
- **winsock.h / winsock2.h conflict**: OSG headers pull in `windows.h` → `winsock.h`; ixwebsocket includes `winsock2.h`. Resolved with `/FI"winsock2.h"` MSVC force-include (ensures winsock2.h is always first, globally).
- **bcrypt.lib**: ixwebsocket → mbedtls → `BCryptGenRandom`. Added `bcrypt` to Robot_Tester link libraries.

**Build status:** Compiles and links cleanly (2026-04-01). No errors, only pre-existing legacy warnings (format string `%d` vs `size_t`).

**Runtime testing:** NOT YET TESTED. Next step: start Robot_Tester, load Curivator (`Test Curivator`), connect a dashboard client (Shuffleboard, Glass, or OutlineViewer) to `localhost:5810`, verify telemetry appears with grouped key names AND the TestChooser dropdown appears with all 12 goals.

### TestChooser Wiring — COMPLETE (2026-04-01)

**Goal:** Wire the legacy Curivator goal selection system to the WPILib String Chooser protocol so dashboard clients show a dropdown of all 12 test goals, matching the modern Robot_Simulation's chooser layout exactly.

**What was added:**

1. **NT4SmartDashboard.h** — `NT4_StringArray` type (wire code 20): `PutStringArray()`, `WriteStringArray()` MsgPack encoder, binary decode for string arrays, `"string[]"` in type string mapping, `stringArrayVal` in RetainedValue.

2. **Curivator_Goals.cpp** — Full chooser integration:
   - `AutonTypeLabels[12]` — human-readable labels for all AutonType values, matching modern names where the same goal exists (indices 0-4 drive, 5 arm, 6 grab, 9 claw; 7-8/10-11 legacy-only)
   - `PublishTestChooser(activeLabel)` — publishes WPILib String Chooser sub-keys under `Test/Test_Selection/TestChooser` (`.type`, `options`, `default`, `active`)
   - `ResolveAutonFromLabel(label)` — maps label string back to AutonType index
   - `SeedTestChooser()` — public static, seeds chooser with initial "Do Nothing" selection so dashboard sees dropdown immediately on connect
   - Modified `Curivator_Goals_Impl::Activate()` — reads `selected` key first (written by dashboard client), falls back to numeric `Test/AutonTest`, re-publishes chooser with `active` label

3. **Curivator_Robot.h** — Added `SeedTestChooser()` declaration to `Curivator_Goals` namespace.

4. **Curivator_Robot.cpp** — Calls `Curivator_Goals::SeedTestChooser()` in `Initialize()` (inside `#ifdef Robot_TesterCode`).

**Chooser key paths (matches modern):**
- `Test/Test_Selection/TestChooser/.type` = `"String Chooser"`
- `Test/Test_Selection/TestChooser/options` = string[] of all 12 labels
- `Test/Test_Selection/TestChooser/default` = `"Do Nothing"`
- `Test/Test_Selection/TestChooser/active` = currently running goal label
- `Test/Test_Selection/TestChooser/selected` = written by dashboard client (user's pick)

**Build status:** Compiles and links cleanly. Committed as `48c94ec`.

**Runtime testing:** NOT YET TESTED. See runtime testing note above.

### Dashboard Layout Update — Curivatorlayout.json (2026-04-01)

Layout expanded from ~105 widgets to ~137 widgets. New variables added by user from runtime observation of legacy simulator:

**New disable controls (Manipulator, bool.led):**
- `Manipulator/ArmDisable`, `Manipulator/BoomDisable`, `Manipulator/BucketDisable`, `Manipulator/ClaspDisable`, `Manipulator/TurretDisable`
- `Manipulator/Disable_Setpoints` — master disable for all setpoint goals
- `Manipulator/SafetyLock_Arm` — arm safety interlock

**New drive disable controls (bool.checkbox):**
- `Drive/Wheel_clDisable`, `Drive/Wheel_crDisable` — center wheel disables (legacy 6-wheel swerve)

**New safety/autonomous:**
- `Autonomous/SafetyLock_Drive` — drive safety interlock

**New vision variables (from legacy camera tracking):**
- `Vision/X Position`, `Vision/Y Position`, `Vision/Z Position` (double.numeric)
- `Vision/Main_Is_Targeting` (bool.led)

**New test variable:**
- `Test/AutonTest` (double.numeric) — legacy numeric goal index, now visible alongside the String Chooser dropdown

**TestChooser options updated:** Removed "Smart Waypoints" (modern-only drive goal), added legacy-only goals: "Test Turret", "Arm And Turret", "Turret Tracking", "Drive Tracking" — now shows all 12 legacy goals.

### SmartDashboard Key Renaming — COMPLETE (2026-04-01)

**Goal:** Rename all SmartDashboard key names in the legacy simulator to match the modern Robot_Simulation's grouped `/`-separated naming convention (e.g., flat `"Velocity"` → `"Drive/Velocity"`, `"arm_xpos"` → `"Manipulator/arm_xpos"`).

**Approach chosen:** Add group prefixes directly at call sites (string literals), NOT a mapping layer in NT4SmartDashboard.h. The NT4 server already prepends `/SmartDashboard/` on the wire (line 473), so call sites only need the group prefix (e.g., `"Drive/"`, `"Swerve/"`, etc.).

**Group prefix mapping:**
| Group | Applies to |
|---|---|
| `Drive/` | Velocity, position, heading, wheel encoders/voltages, tank PID, waypoints, slide/field-centric modes, autopilot, high gear, accel limits, driver override |
| `Swerve/` | Swivel voltages, swivel raw/pot values |
| `Manipulator/` | Arm positions/angles, boom/bucket/clasp, rotary system PID debug, arm voltage/raw/pot, safety locks, catapult, intake, winch, roller, fork, deployment, claw, wrist, height, tote |
| `Autonomous/` | Timer, safety lock drive, test auton, freeze arm, sequence, auton params (ball count, position, goal selection, move/side/height) |
| `Test/` | Target arm/bucket/clasp values, test arm dimensions, test move/rotate, auton test, turret test |
| `Vision/` | Camera positions, averages, yaw, drive distance, targeting enables, tracking params, scale factors, tolerances, ball tracking, hot target, land reticle, camera transform |

**Files edited (13 total, all active SmartDashboard calls prefixed):**
1. `UI_Controller.cpp` — 9 keys → `Drive/`
2. `Swerve_Robot.cpp` — All keys → `Drive/`, `Swerve/`, `Autonomous/`; dynamic construction uses index-based branching
3. `Tank_Robot.cpp` — 44 keys → `Drive/`
4. `Ship.cpp` — 25 keys → `Drive/`
5. `Rotary_System.cpp` — 70 keys → `Manipulator/`
6. `Curivator_Robot.cpp` — ~30+ edits → `Manipulator/`, `Drive/`, `Vision/`, `Autonomous/`; `CheckDisableSafety_Curivator()` got `groupPrefix` parameter for dynamic key construction
7. `Curivator_Goals.cpp` — All keys → `Test/`, `Vision/`, `Autonomous/`, `Manipulator/`
8. `FRC2014_Robot.cpp` — ~59 keys → `Manipulator/`, `Vision/`, `Autonomous/`, `Drive/`
9. `FRC2015_Robot.cpp` — ~48 keys → `Vision/`, `Drive/`, `Manipulator/`, `Autonomous/`
10. `HikingVIking_Robot.cpp` — 7 keys → `Manipulator/`
11. `Ship_1D.cpp` — 4 keys → `Drive/`
12. `Calibration_Testing.cpp` — 7 keys → `Drive/`, `Test/`
13. `UDP_Listener.cpp` — 6 keys → `Vision/`

**Files verified no edits needed:** `Vehicle_Drive.cpp` (all calls commented out), `Entity2D.cpp` (behind `#undef __SetRobotRemote__`), `AI_Base_Controller.cpp` (all calls commented out).

**Disabled code:** ~52 unprefixed calls remain in `#if 0` blocks, `#else` of `#undef` guards, and `//`-commented lines. These are intentionally left as-is since they are not compiled.

**Build status:** Compiles and links cleanly (2026-04-01). Zero errors, only pre-existing format string warnings.

**Architecture context:**
- GUI loop (Viewer::Start in Viewer.cpp:108-127) runs on GUIThread. Each frame calls `timer.FireTimer()` to get wall-clock `dTime_s`, then `GameClient::UpdateData(dTime_s)` → each entity's `TimeChange(dTime_s)`.
- Console CLI (Robot_Tester.cpp main loop) runs on the main thread. Test commands call `goal->Activate()` which calls `SetIntendedPosition()` directly on Ship_1D — **no synchronization** between threads.
- `dTime_s` comes from `osg::Timer` (QueryPerformanceCounter). **No clamping exists anywhere.** A debugger pause, window drag, or any stall causes the next frame to receive the full accumulated wall-clock delta as a single physics step.

### Ship_1D oscillation bug analysis (broncbotz_code legacy)

**Three identified contributing factors:**

**Factor 1: Uninitialized `m_SimFlightMode` in Ship_1D constructor (minor)**
- `m_SimFlightMode` (Ship_1D.h:183) has no initializer and is not in the member init list.
- `SetSimFlightMode(true)` at Ship_1D.cpp:21 has a guard: `if (m_SimFlightMode != SimFlightMode)`.
- In MSVC Debug, uninitialized memory is `0xCC`. Comparing `0xCC != 0x01 (true)` → guard passes → body executes → `m_SimFlightMode` is set correctly. So this works by **coincidence** in Debug.
- In Release, garbage could be `0x01`, making the guard skip, leaving `m_SimFlightMode` as garbage. The subsequent `ResetPosition()` → `SetSimFlightMode(true)` would hit the same guard.
- **Impact:** Low — `m_SimFlightMode` ends up true either way, but the code reads uninitialized memory which is UB.

**Factor 2: No `dTime_s` clamping (primary suspect)**
- `OSG_Timer::FireTimer()` (Timer.cpp:16-23) returns raw wall-clock delta with no cap.
- `GG_Framework::Base::Timer::IncTime_s()` (EventMap.cpp:187-195) accumulates without filtering.
- The physics integrator (`ApplyFractionalForce`) does simple Euler: `velocity += (force/mass) * dt`. With a large `dt`, velocity overshoots the target position, then the next frame computes a large correction in the opposite direction → **oscillation**.
- `GetVelocityFromDistance_Linear` (Physics_1D.cpp:162-197) computes a safe stopping velocity, but its math assumes small `dt`: `Time = sqrt(2 * Distance / Acceleration)`, then `MaxSpeed = Acceleration * max(Time - dt, 0)`. With `dt >> Time`, `MaxSpeed` goes to zero, but `IdealSpeed = Distance / dt` may still be large, and `min(IdealSpeed, MaxSpeed)` picks `MaxSpeed` which is zero → ship stops → next frame distance is still large → cycle.
- **Scenario:** Debug builds are slower (disabled optimizations, runtime checks). If the first frame after construction/loading takes >100ms (plausible), the physics gets a single `dTime_s` of 0.1-0.5s instead of the expected ~0.016s. This produces a force overshoot.

**Factor 3: Thread race between goal activation and physics loop (aggravating)**
- `SetIntendedPosition()` (Ship_1D.h:102-104) writes `m_LockShipToPosition` and `m_IntendedPosition` from the console thread with no lock.
- `TimeChange()` reads these on the GUI thread. A torn read could see `m_IntendedPosition` updated but `m_LockShipToPosition` still true (or vice versa), producing one frame of wrong-branch physics.
- **Impact:** Medium — could cause a single frame of bad force, which then damps out over subsequent frames ("oscillates for a few seconds").

**Recommended debugging approach (set breakpoints in VS2022):**
1. Set breakpoint at Ship_1D.cpp:157 (TimeChange entry) — log `dTime_s` for the first ~20 frames after goal activation. Look for any frame with `dTime_s > 0.05`.
2. Set breakpoint at Ship_1D.cpp:300 (GetVelocityFromDistance_Linear call in the else branch) — log `DistanceToUse`, `Vel`, and `ForceToApply`.
3. Set breakpoint at Ship_1D.cpp:330 (ApplyFractionalForce) — log `ForceToApply` and resulting velocity after the call.
4. Alternatively: enable `m_UseSyntheticTimeDeltas = true` before goal activation. If the oscillation disappears with fixed 10ms steps, the root cause is confirmed as unclamped `dTime_s`.

**Fixes applied (in Robot_Simulation Ship1D_Legacy.h, not in broncbotz_code legacy):**
- **Fix A (APPLIED):** Clamp `dTime_s` in `Ship_1D::TimeChange()`: `dTime_s = min(dTime_s, 0.05);` — prevents physics explosion from large time steps.
- **Fix B (APPLIED):** Initialize `m_SimFlightMode = false;` in Ship_1D.h declaration — eliminates UB.
- **Fix C (NOT APPLIED):** For the Robot_Simulation (non-legacy) project, ensure `SetIntendedPosition()` is only called from the physics thread, or add a mutex.

**Key files for breakpoint debugging:**
- `Ship_1D.cpp:157` — TimeChange entry, inspect `dTime_s`
- `Ship_1D.cpp:282-322` — unlocked branch (after SetIntendedPosition), inspect computed `Vel` and `ForceToApply`
- `Ship_1D.cpp:330` — force application, inspect velocity before/after
- `Physics_1D.cpp:162` — GetVelocityFromDistance_Linear, inspect `Distance`, `Time`, `MaxSpeed`, `IdealSpeed`
- `Viewer.cpp:111` — timer.FireTimer() return value
