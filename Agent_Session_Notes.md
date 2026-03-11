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

## Active constraints

- `SmartDashboard/networktables2/util/System.cpp` remains excluded in CMake (VxWorks path); Windows build uses `OSAL/System.cpp`.
- `_CRT_SECURE_NO_WARNINGS` macro redefinition warnings remain non-blocking cleanup work.

## Next-session checklist

1. Expand gtest coverage by migrating practical checks from `_Tester.cpp` paths into focused suites.
2. Keep optional tester apps buildable for exploratory/manual scenarios.
3. Consider non-blocking cleanup of `_CRT_SECURE_NO_WARNINGS` macro redefinition warnings.
