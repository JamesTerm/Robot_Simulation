# Project history

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
