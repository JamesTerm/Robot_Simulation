# Agent session notes

- Edit this file for short, high-signal context that helps the next session start quickly.
- Keep this file lean; move long milestone history to `docs/project_history.md`.
- Commit-note convention: when the user says "update notes", keep this file to handoff-critical context only; put durable feature/change history in `docs/project_history.md`.

## Workflow note

- `apply_patch` expects workspace-relative paths (forward slashes). Avoid absolute Windows paths to prevent separator errors.
- Code style uses ANSI/Allman indentation; keep brace/indent alignment consistent with existing blocks to avoid drift.
- Use Windows CRLF line endings for C++ source files in this repo.
- Read nearby `Ian:` comments before editing a file. They mark intentional boundaries or historical lessons that should be preserved unless you deliberately mean to change behavior.

## Documentation and teaching comments rule

- Treat this codebase as both working simulator code and a carry-forward engineering notebook.
- Add concise `Ian:` comments in `.cpp` files when behavior depends on non-obvious reasoning, ordering, cross-process/state lifetime lessons, or semantics learned by trial and error.
- Focus those comments on *why* a path is shaped the way it is, especially where future sessions might otherwise repeat an old mistake.
- Good targets include transport/session behavior, retained vs live state flow, multi-client fan-out, ownership/lease decisions, reconnect ordering, and any place we are intentionally following Direct as a temporary parity baseline.
- Keep comments high-signal; avoid narrating obvious code.

## Quick context for next session

- Primary app target is `DriverStation`; legacy apps/testers are optional CMake groups.
- CMake + vcpkg build path is working in Debug (including OSG-based targets).
- Durable details moved to `docs/project_history.md`.
- Long-form repository orientation moved to `docs/overview.md`.
- GoogleTest integration is now in place (`find_package(GTest CONFIG REQUIRED)` + `gtest_discover_tests`).
- New transport feature direction is documented in `docs/transport_modes_strategy.md`.
- Current working vcpkg path on this machine is `D:/Git/vcpkg`, not the older `D:/code/vcpkg` examples that still appear in some docs/notes.
- Transport iteration progress:
  - Iteration 1: runtime mode selection plumbing added (cmdline + hotkeys) with legacy behavior preserved.
  - Iteration 2: extracted connection backend/router (`Transport.h/.cpp`) and routed `RobotTester` through it.
  - Current behavior: legacy + shuffle use SmartDashboard path; direct mode now starts a shared-memory/event publisher (`Local\\SmartDashboard.Direct.*`) compatible with SmartDashboard direct subscriber expectations.
  - Stability patch: added defensive guard in legacy NT entry store to handle rare null-entry-type update windows during Auton test transitions.
  - Deeper hardening: removed implicit `std::map::operator[]` insertions in NT entry stores to avoid accidental null entry publication.
  - Startup/reconnect stability: direct command startup now preserves dashboard-owned `AutonTest` and related test keys across repeated simulator restarts without restarting SmartDashboard.
  - Added `DriverStation_TransportSmoke` target and `ResolveAutonIndex` unit-tested helper to reproduce startup sequencing in a fast harness.
	  - Direct chooser contract is now in place for simulator work: `Test/AutoChooser/.type`, `options`, `default`, `active`, `selected`, with string-array `options` on the direct path.
	  - DriverStation now has an in-app connection dropdown wired to legacy/direct/shuffle modes in addition to existing hotkeys.
	  - Native Link first simulator-owned slice is now in place behind its own selectable backend using local shared memory + named events as the v1 carrier.
	  - Preserve Direct as the known-good single-dashboard reference path; use it as the rough template when Native Link behavior is surprising during early 1:1 validation.
	  - First Native Link simulator topics are aligned with the proven Direct harness keys: `Test/Auton_Selection/AutoChooser/selected`, `TestMove`, `Timer`, and `Y_ft`.
	  - Native Link session behavior intentionally treats the simulator as the authority: a simulator restart advances the server session even if the local mapping/channel name stays the same.
	  - Native Link fan-out intentionally uses per-client read progress instead of the old shared-consumer pattern that previously caused Direct watcher/observer interference.
	  - Current v1 lease behavior opportunistically claims a lease on first client writer contact for lease-controlled topics to keep the initial 1:1 operator experience close to Direct while still exercising explicit ownership in the core.
	  - Latest IPC protocol carry-forward change: the shared carrier no longer relies on packed structs around atomics, and both repos now include a dedicated `snapshotCompleteSessionId` field so snapshot completion is not overloaded onto write-ack state.
	  - `robot_unit_tests.exe` currently passes with the new Native Link tests when run directly from `build-vcpkg/bin/Debug`; if `ctest` disagrees, suspect stale test-discovery metadata in the build directory before assuming a code regression.
  - Current direct status checkpoint:
    - simulator and dashboard both now use independent consumer-cursor semantics on the telemetry ring instead of one shared consumed cursor
    - `DriverStation_TransportSmoke` is a reliable harness entry point and now explicitly runs in `Direct Connect`
    - SmartDashboard process control helper exists at `D:/code/SmartDashboard/tools/smartdashboard_process.py`
    - repeated robot restart stress improved after fixing publisher free-space accounting against the active consumer cursor rather than obsolete shared `readIndex`
    - direct command subscriber instance ids now include process/time entropy, matching the SmartDashboard-side hardening against multi-process collisions
    - publisher retained-command replay no longer waits for a consumer-instance-id change; it keys off the inactive->active heartbeat transition so dashboard restarts still receive the retained command snapshot
    - paired validation with `D:/code/SmartDashboard/tools/survive_sequence.py` now passes again for dashboard survive, chooser survival, `TestMove=3.5`, and the robot-survive handoff
  - Remaining blocker/reference point:
    - direct mode is good again for the real single-dashboard path, but passive extra observers/watchers still destabilize repeated runs, so transport is not yet truly multi-observer safe
    - the short immediate post-dashboard-restart probe window can still under-report early telemetry even when the later robot-survive phase succeeds
    - for the real dashboard path, setup-state tiles (`TestMove`, chooser) can appear visually stale even when robot behavior proves command values were applied; use `Timer` / `Y_ft` as live paint indicators in current harness layout

## Active constraints

- `SmartDashboard/networktables2/util/System.cpp` remains excluded in CMake (VxWorks path); Windows build uses `OSAL/System.cpp`.
- `_CRT_SECURE_NO_WARNINGS` macro redefinition warnings remain non-blocking cleanup work.

## Current known issues / follow-up log

- Direct chooser manual validation now passes for basic operator flow: selecting `Just Move Forward` and enabling auton works, and dashboard restart no longer overwrites robot chooser state.
- Paired survive validation now also passes for remembered numeric control recovery: `TestMove=3.5` survives dashboard restart and is read correctly on auton activation.
- Remaining reconnect gap: extra concurrent observers still expose race/session weaknesses, so the current Direct path is still best treated as single-real-client.
- Current manual interpretation: when smoke behavior is correct but chooser/TestMove look static, that may be a visibility/expectation issue because those are setup-state values; live telemetry keys like `Timer` and `Y_ft` are better paint verification signals.
- Keep monitoring for any remaining control keys that may require scoped alias support (`Test/<key>` fallback) when dashboards mix flat and scoped naming.
- Official SmartDashboard historically supported `SendableChooser`; use that as compatibility guidance rather than keeping long-term numeric-only fallback in this feature branch.
- Current cross-repo blocker: SmartDashboard's real IPC client startup/restart handshake is still flaky even after the carrier/layout cleanup. Robot_Simulation-side Native Link tests are green, but the dashboard-side combined slice still needs more ordering work before paired validation should be treated as deterministic.
- Follow-on roadmap note: keep the current shared-memory + named-events authority path available as the simpler diagnostic/reference carrier even after a future TCP carrier is added. The longer-term plan is carrier parity under one Native Link semantic contract, not a one-way delete-and-replace of the current IPC path.

## Next-session checklist

1. Decide whether to keep current single-real-client direct assumptions or invest in true multi-observer broadcast semantics for tooling/watchers.
2. Reduce remaining chooser/status republish churn if it becomes a practical performance or readability issue during longer runs.
3. Clean up harness instrumentation/logging once behavior is stable, but keep the process-control + smoke/probe workflow documented.
4. Once Direct behavior is stable enough, compare against local Shuffleboard rather than relying on official SmartDashboard localhost behavior.
5. Before ending a Native Link session, scan changed tricky paths for missing `Ian:` comments and capture the same rationale in these notes if it would help a future handoff.
