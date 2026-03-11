# Agent session notes

- Edit this file for short, high-signal context that helps the next session start quickly.
- Keep this file lean; move long milestone history to `docs/project_history.md`.
- Commit-note convention: when the user says "update notes", keep this file to handoff-critical context only; put durable feature/change history in `project_history.md`.

## Workflow note

- `apply_patch` expects workspace-relative paths (forward slashes). Avoid absolute Windows paths to prevent separator errors.
- Code style uses ANSI/Allman indentation; keep brace/indent alignment consistent with existing blocks to avoid drift.
- Use Windows CRLF line endings for C++ source files in this repo.

## Documentation and teaching comments rule

- Treat this codebase as both production code and a learning reference.
- Add concise, high-value comments in `.cpp` files when logic is non-trivial (timing behavior, concurrency, transport semantics, state handling, etc.).
- For advanced algorithms/patterns, include the concept name directly in comments where implemented (for example: ring buffer, round-robin, coalescing/latest-value cache, debounce, backoff).
- Keep comments practical and instructional: explain *why* a pattern is used and what trade-off it makes, not just what the line does.
- Avoid noisy comments on obvious code paths; focus comments on places likely to confuse first-time readers.

## Quick context for next session

Goal for first session: “Create non-invasive CMake scaffold + build one vertical slice (e.g., SmartDashboard lib + one app) with vcpkg OSG.”

Status (2026-03-11): initial vertical slice is now building with CMake + vcpkg using `DriverStation` as the primary app target.

- Added top-level `CMakeLists.txt` and kept existing `.sln/.vcxproj` untouched.
- CMake now builds `SmartDashboard` (static), `dx_Joystick_Controller` (static), `OSG_Viewer` (shared), `RobotAssembly_static` (static), and `DriverStation` (exe).
- Switched OSG discovery to vcpkg package config (`find_package(unofficial-osg CONFIG REQUIRED COMPONENTS plugins)`), with explicit target links.
- Removed hardcoded `#pragma comment(lib, "...OSG_Viewer...")` from `TeleAutonV1.cpp`, `TeleAutonV2.cpp`, `TeleOpV2.cpp`, and `TeleOpV3.cpp`.
- Added missing `#include <locale.h>` in `Source/Base/LUA.cpp` for CMake build compatibility.
- Build proof: `cmake --build build-vcpkg --config Debug --target DriverStation` succeeds.
- Run proof: `build-vcpkg/bin/Debug/DriverStation.exe` launches and starts NetworkTables threads.

Additional progress (same day):

- Added `BaseCore` CMake static library for shared Base sources (`EventMap.cpp`, `LUA.cpp`, `Misc.cpp`, `Script.cpp`, `Time_Type.cpp`) to avoid duplicate object linkage.
- Added CMake options and target grouping:
  - `BUILD_DRIVER_STATION` (default ON)
  - `BUILD_LEGACY_APPS` (default OFF)
  - `BUILD_TESTER_APPS` (default OFF)
  - `BUILD_UNIT_TESTS` (default ON)
- Added optional legacy app targets under `BUILD_LEGACY_APPS`:
  - `RobotAssembly`
  - `OutputViewer`
- Added optional tester app targets under `BUILD_TESTER_APPS`:
  - `AI_Input_Tester`, `OSG_Viewer_Tester`, `OSG_ViewTester`, `dx_Joystick_Controller_App`,
    `Entity1D_Tester`, `Entity2D_Tester`, `MotionControl2D_simple_Tester`,
    `MotionControl2D_physics_Tester`, `SwerveRobot_Tester`
- Added initial CTest-based unit test executable `robot_unit_tests` (`tests/robot_unit_tests.cpp`) covering basic Entity1D/Entity2D/DriveKinematics sanity checks.
- Standardized Windows macro checks from `_Win32` to `_WIN32` in:
  - `Source/Base/Base_Includes.h`
  - `Source/Base/LUA.h`
  - `Source/Base/LUA.cpp`
  - `Source/Base/Time_Type.cpp`
  - `Source/Base/Misc.cpp`
- Removed remaining `#pragma comment(lib, ...)` usage across `.cpp` sources, including:
  - `Source/Application/DriverStation/DriverStation/DriverStation.cpp`
  - `Source/Libraries/SmartDashboard/OSAL/System.cpp`
  - `Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.cpp`
  - `Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.cpp`
  - `Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot_Tester.cpp`

Build status after enabling optional targets:

- All primary + optional legacy apps + tester apps build in Debug with vcpkg toolchain.
- `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (`robot_unit_tests`).

Known follow-ups:

- `CMakeLists.txt` currently defines `_Win32` globally to preserve legacy header behavior (`Base_Includes.h` / `LUA.h` use `_Win32` instead of `_WIN32`).
- `CMakeLists.txt` excludes `Source/Libraries/SmartDashboard/networktables2/util/System.cpp` and uses `OSAL/System.cpp` as the Windows implementation.
- Macro redefinition warnings remain (`_CRT_SECURE_NO_WARNINGS`) from legacy headers; non-blocking.

Updated follow-ups:

- `_Win32` workaround is no longer needed after macro cleanup to `_WIN32`.
- Macro redefinition warnings (`_CRT_SECURE_NO_WARNINGS`) still remain in legacy headers; non-blocking cleanup candidate.

## Next-session checklist

Start a new branch in Robot_Simulation (done), add top-level CMake, keep existing .sln/.vcxproj untouched initially.
Phase 1: build core libraries as CMake targets (no behavior change).
Phase 2: convert _Tester.cpp binaries into GoogleTest-style tests where practical.
Phase 3: define one primary app target (your final executable), demote legacy apps.
Phase 4: replace hardcoded OSG paths with find_package(OpenSceneGraph) via vcpkg toolchain.
Phase 5: remove #pragma comment(lib, ...) in favor of target_link_libraries.
