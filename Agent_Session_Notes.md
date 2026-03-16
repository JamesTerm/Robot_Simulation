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
  - Current blocker/reference point: dashboard restart no longer clobbers robot-owned chooser selection, but full bidirectional retained-state parity is still incomplete:
    - dashboard restarted second: chooser selection survives behaviorally, but some dashboard-owned cosmetic/control values (for example `TestMove` display refresh) still appear stale
    - robot restarted second: chooser/control state handoff still behaves like previous runs rather than fully restoring both ownership directions

## Active constraints

- `SmartDashboard/networktables2/util/System.cpp` remains excluded in CMake (VxWorks path); Windows build uses `OSAL/System.cpp`.
- `_CRT_SECURE_NO_WARNINGS` macro redefinition warnings remain non-blocking cleanup work.

## Current known issues / follow-up log

- Direct chooser manual validation now passes for basic operator flow: selecting `Just Move Forward` and enabling auton works, and dashboard restart no longer overwrites robot chooser state.
- Remaining reconnect gap: we do not yet have both directions at once (robot-owned chooser retention plus dashboard-owned control refresh/replay parity).
- Keep monitoring for any remaining control keys that may require scoped alias support (`Test/<key>` fallback) when dashboards mix flat and scoped naming.
- Official SmartDashboard historically supported `SendableChooser`; use that as compatibility guidance rather than keeping long-term numeric-only fallback in this feature branch.

## Next-session checklist

1. Finish direct retained-state parity so both reconnect directions work cleanly:
   - dashboard restarted second should repaint dashboard-owned values correctly
   - robot restarted second should receive remembered dashboard-owned controls without regressing chooser ownership
2. Keep robot-owned chooser state separate from dashboard-owned controls when refining replay/reconnect logic.
3. Once Direct behavior is correct, compare against local Shuffleboard rather than relying on official SmartDashboard localhost behavior.
4. Research/plan NT library update separately if adapter work later shows protocol/version gaps in legacy mode.
