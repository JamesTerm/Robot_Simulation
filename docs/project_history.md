# Project history

## 2026-03-17 - Direct survive replay simplification and paired restart recovery

- Simplified robot-side Direct retained-command replay during dashboard reconnects:
  - retained replay now keys off the inactive->active consumer heartbeat transition instead of waiting for a changed `consumerInstanceId`
  - this avoids missing dashboard restarts where the dashboard preserves command intent but the subscriber-side identity logic no longer produces the old change signal.
- Hardened direct command subscriber identity generation:
  - subscriber instance ids now mix process id, steady-clock time, and local counter state instead of using only a simple in-process increment.
- Paired result with the SmartDashboard survive/startup fixes:
  - remembered `TestMove=3.5` survives dashboard restart
  - chooser selection survives dashboard restart
  - subsequent robot-survive activation reads the remembered value correctly and passes the paired stress flow.
- Practical takeaway:
  - the final remaining survive fix was mostly startup/reconnect ordering and reconnect detection, not a major transport rewrite.

## 2026-03-17 - Direct numeric recovery and chooser-safe fallback

- Added non-destructive SmartDashboard `TryGet*` reads so Direct-mode numeric lookups do not misinterpret missing values as valid zeroes.
- Restored reliable Direct autonomous execution by fixing robot-side `TestMove` reads at activation time.
- Kept chooser support available for Direct mode while preserving numeric fallback when chooser state is absent.
- Added focused transport/auton tracing that made it easier to distinguish command-path bugs from harness/reporting bugs.
- Expanded documentation structure with `docs/learning/` and `docs/journal/` for more student-friendly engineering context.

## 2026-03-16 - Direct transport stress harness and consumer-cursor hardening

- Continued the Direct-mode dashboard interoperability push with a stronger harness around SmartDashboard pairing:
  - `DriverStation_TransportSmoke` now serves as a repeatable deterministic smoke runner for chooser + `TestMove` setup and 10-second auton execution
  - SmartDashboard process/probe/watch tooling on the companion repo is now part of the active debug loop for repeated restart stress
- Updated Robot_Simulation Direct transport to better match the SmartDashboard-side cursor model:
  - direct telemetry replay now tracks consumer instance changes
  - publisher free-space accounting now uses the active consumer cursor instead of the obsolete shared `readIndex`
- Resulting status checkpoint:
  - real single-dashboard Direct runs are healthier and smoke telemetry is visible again after repeated restarts
  - repeated robot-restart stress is improved but still not considered fully deterministic over long sequences
  - passive extra observers still perturb the transport, so true multi-observer robustness remains future work rather than the current Direct-mode assumption.

## 2026-03-16 - Shared chooser contract alignment for simulator and dashboards

- Began converging Robot_Simulation autonomous selection around the same chooser semantics used by current SmartDashboard/Shuffleboard compatibility work:
  - chooser base publishes `.type`, `options`, `default`, `active`, and `selected`
  - robot-side autonomous selection now prefers chooser labels first and keeps legacy `AutonTest` string fallback during transition
- Updated legacy SmartDashboard sendable publish path to emit `.type` alongside historical `~TYPE~` so documentation and compatibility adapters can share one chooser contract.
- Updated transport smoke startup coverage to seed chooser-style autonomous keys instead of numeric-only `AutonTest`.
- Documentation now records that modern NT string-array chooser options remain the preferred long-term encoding. SmartDashboard direct transport now supports them end-to-end, while Robot_Simulation still bridges chooser options as comma-strings until its direct protocol is upgraded to match.

## 2026-03-15 - Direct startup/reconnect command retention stabilization

- Added direct transport smoke harness target `DriverStation_TransportSmoke` to exercise startup path quickly against live dashboard state.
- Added autonomous selection resolver helper (`Source/Modules/Input/AI_Input/AutonSelection.h`) and gtest coverage (`tests/auton_selection_tests.cpp`) for retry/default/clamp behavior.
- Fixed dashboard-owned command startup race behavior in robot-side activation path:
  - `AI_Input_Example` now uses retry-based selection resolve for first auton activation.
  - avoids robot-side overwrite of operator-owned startup values.
- Hardened direct/legacy command reads for operator keys by supporting scoped and flat aliases in helper functions:
  - prefers `Test/<key>` then falls back to `<key>`
  - supports number-from-string fallback for mixed widget/control publishing paths
  - removed robot-side default writeback in RobotTester helper paths.
- Hardened NT entry/type handling for legacy path stability during mode and startup transitions.
- Validation:
  - `DriverStation`, `DriverStation_TransportSmoke`, and `robot_unit_tests` build successfully.
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (10/10).
  - Manual direct-mode stress validated startup/reconnect behavior for `AutonTest` and `TestMove` when paired with updated SmartDashboard.

## 2026-03-11 - CMake migration milestone (DriverStation vertical slice)

- Added top-level `CMakeLists.txt` to begin non-invasive CMake migration while preserving legacy Visual Studio solution/project files.
- Set `DriverStation` as the primary application target for the first end-to-end milestone.
- Added CMake targets for core runtime:
  - `SmartDashboard` (static)
  - `BaseCore` (static)
  - `dx_Joystick_Controller` (static)
  - `OSG_Viewer` (shared)
  - `RobotAssembly_static` (static)
  - `DriverStation` (exe)
- Added optional CMake target families:
  - Legacy apps (`BUILD_LEGACY_APPS`): `RobotAssembly`, `OutputViewer`
  - Tester apps (`BUILD_TESTER_APPS`): `AI_Input_Tester`, `OSG_Viewer_Tester`, `OSG_ViewTester`, `dx_Joystick_Controller_App`, `Entity1D_Tester`, `Entity2D_Tester`, `MotionControl2D_simple_Tester`, `MotionControl2D_physics_Tester`, `SwerveRobot_Tester`
  - Unit tests (`BUILD_UNIT_TESTS`): `robot_unit_tests` with CTest integration
- Integrated OSG through vcpkg package config (`unofficial-osg`) and linked OSG/OpenThreads via `target_link_libraries`.
- Installed OSG and dependencies in local vcpkg (`osg:x64-windows`).
- Removed hardcoded OSG `#pragma comment(lib, ...)` links from:
  - `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV1.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV2.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleOpV2.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleOpV3.cpp`
- Removed remaining `#pragma comment(lib, ...)` from `.cpp` sources and moved linkage to CMake targets:
  - `Source/Application/DriverStation/DriverStation/DriverStation.cpp`
  - `Source/Libraries/SmartDashboard/OSAL/System.cpp`
  - `Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.cpp`
  - `Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.cpp`
  - `Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot_Tester.cpp`
- Added `#include <locale.h>` in `Source/Base/LUA.cpp` to resolve CMake/MSVC compile failure.
- Normalized Windows macro checks from `_Win32` to `_WIN32` in Base files to remove global compile-definition workaround.
- CMake build validation:
  - Configure: `cmake -S . -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE=/d/code/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows`
  - Build: `cmake --build build-vcpkg --config Debug --target DriverStation`
  - Run: `build-vcpkg/bin/Debug/DriverStation.exe` (launches and initializes NetworkTables threads)
- Additional build validation:
  - Configure with optional targets: `-DBUILD_LEGACY_APPS=ON -DBUILD_TESTER_APPS=ON`
  - Build optional apps/testers succeeds in Debug
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (1/1)

### Notes

- CMake excludes `Source/Libraries/SmartDashboard/networktables2/util/System.cpp` (VxWorks implementation) and uses Windows `OSAL/System.cpp`.
- Non-blocking warnings remain from legacy `_CRT_SECURE_NO_WARNINGS` macro redefinitions.

## 2026-03-11 - Documentation organization and GoogleTest integration

- Reorganized docs to match SmartDashboard-style layout:
  - Added `docs/overview.md` for long-form project orientation.
  - Moved durable history to `docs/project_history.md`.
  - Kept `Agent_Session_Notes.md` lean and handoff-focused.
  - Slimmed top-level readme to quick-start + documentation map.
- Integrated GoogleTest with CMake unit-test flow:
  - Added `find_package(GTest CONFIG REQUIRED)` and `include(GoogleTest)`.
  - Replaced custom test `main()` with gtest-based suites.
  - Added `gtest_discover_tests(robot_unit_tests)` for CTest/Visual Studio discovery.
- Split tests into focused files:
  - `tests/entity1d_tests.cpp`
  - `tests/entity2d_tests.cpp`
  - `tests/drivekinematics_tests.cpp`
- Added `docs/testing.md` with configure/build/run and Visual Studio test-discovery guidance.
- Validation:
  - `cmake --build build-vcpkg --config Debug --target robot_unit_tests` succeeds.
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (6/6).

## 2026-03-15 - Transport mode feature planning handoff

- Added `docs/transport_modes_strategy.md` to capture next major feature direction.
- Documented explicit three-mode goal:
  - `Direct Connect`
  - `Legacy SmartDashboard`
  - `Shuffleboard`
- Captured sequencing requirement: establish Direct Connect before Shuffleboard work.
- Captured compatibility rule: preserve legacy SmartDashboard behavior as baseline/oracle.
- Linked transport strategy in top-level `ReadMe.md` documentation map.
- Updated `Agent_Session_Notes.md` checklist to prioritize transport mode selection and Direct/Legacy contract implementation.

## 2026-03-15 - Safe cleanup while transport work continues

- Applied no-risk warning hygiene for `_CRT_SECURE_NO_WARNINGS` macro definitions by guarding local defines:
  - `Source/Application/DriverStation/DriverStation/stdafx.h`
  - `Source/Base/Base_Includes.h`
  - `Source/Base/LUA.cpp`
  - `Source/Base/Misc.cpp`
- Performed no-risk include deduplication in `Source/Application/DriverStation/DriverStation/stdafx.h` and `Source/Base/Misc.cpp`.
- Added and verified startup transport smoke harness target (`DriverStation_TransportSmoke`) to exercise Direct mode startup sequence quickly.
- Added unit tests for autonomous selection retry/default/clamp behavior (`tests/auton_selection_tests.cpp`) and integrated into `robot_unit_tests`.
