# Agent session notes

- Edit this file for short, high-signal context that helps the next session start quickly.
- Keep this file lean; move long milestone history to `docs/project_history.md`.
- Commit-note convention: when the user says "update notes", keep this file to handoff-critical context only; put durable feature/change history in `docs/project_history.md`.

## Workflow note

- `apply_patch` expects workspace-relative paths (forward slashes). Avoid absolute Windows paths to prevent separator errors.
- Code style uses ANSI/Allman indentation; keep brace/indent alignment consistent with existing blocks to avoid drift.
- Use Windows CRLF line endings for C++ source files in this repo.

## Quick context for next session

- Primary app target is `DriverStation`; legacy apps/testers are optional CMake groups.
- CMake + vcpkg build path is working in Debug (including OSG-based targets).
- Durable details moved to `docs/project_history.md`.
- Long-form repository orientation moved to `docs/overview.md`.
- GoogleTest integration is now in place (`find_package(GTest CONFIG REQUIRED)` + `gtest_discover_tests`).
- New transport feature direction is documented in `docs/transport_modes_strategy.md`.
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
  - Current direct status checkpoint:
    - simulator and dashboard both now use independent consumer-cursor semantics on the telemetry ring instead of one shared consumed cursor
    - `DriverStation_TransportSmoke` is a reliable harness entry point and now explicitly runs in `Direct Connect`
    - SmartDashboard process control helper exists at `D:/code/SmartDashboard/tools/smartdashboard_process.py`
    - repeated robot restart stress improved after fixing publisher free-space accounting against the active consumer cursor rather than obsolete shared `readIndex`
  - Remaining blocker/reference point:
    - real single-dashboard runs are much healthier, but robot-survive stress is still somewhat race-sensitive over repeated cycles
    - passive extra observers (debug watchers) still destabilize repeated runs, so transport is not yet truly multi-observer safe
    - for the real dashboard path, setup-state tiles (`TestMove`, chooser) can appear visually stale even when robot behavior proves command values were applied; use `Timer` / `Y_ft` as live paint indicators in current harness layout

## Active constraints

- `SmartDashboard/networktables2/util/System.cpp` remains excluded in CMake (VxWorks path); Windows build uses `OSAL/System.cpp`.
- `_CRT_SECURE_NO_WARNINGS` macro redefinition warnings remain non-blocking cleanup work.

## Current known issues / follow-up log

- Direct chooser manual validation now passes for basic operator flow: selecting `Just Move Forward` and enabling auton works, and dashboard restart no longer overwrites robot chooser state.
- Remaining reconnect gap: repeated robot restart stress is improved but still not fully deterministic across long sequences; extra concurrent observers still expose race/session weaknesses.
- Current manual interpretation: when smoke behavior is correct but chooser/TestMove look static, that may be a visibility/expectation issue because those are setup-state values; live telemetry keys like `Timer` and `Y_ft` are better paint verification signals.
- Keep monitoring for any remaining control keys that may require scoped alias support (`Test/<key>` fallback) when dashboards mix flat and scoped naming.
- Official SmartDashboard historically supported `SendableChooser`; use that as compatibility guidance rather than keeping long-term numeric-only fallback in this feature branch.

## Next-session checklist

1. Finish hardening robot-survive stress for repeated restart cycles with a single real SmartDashboard client.
2. Decide whether to keep current single-real-client direct assumptions or invest in true multi-observer broadcast semantics for tooling/watchers.
3. Clean up harness instrumentation/logging once behavior is stable, but keep the process-control + smoke/probe workflow documented.
4. Once Direct behavior is stable enough, compare against local Shuffleboard rather than relying on official SmartDashboard localhost behavior.
