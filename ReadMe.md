# Robot Simulation

C++ robot simulation project with `DriverStation` as the primary app target.

## Quick start (Windows + MSVC + vcpkg)

1. Ensure vcpkg is available (example path used below: `D:/code/vcpkg`).
2. Install OSG once:
   - `D:/code/vcpkg/vcpkg install osg:x64-windows --recurse`
3. Install GoogleTest once:
   - `D:/code/vcpkg/vcpkg install gtest:x64-windows`
4. Configure:
   - `cmake -S . -B build-vcpkg -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows`
5. Build primary app:
   - `cmake --build build-vcpkg --config Debug --target DriverStation`
6. Run:
   - `build-vcpkg/bin/Debug/DriverStation.exe`

## Optional build groups

- Enable legacy apps: `-DBUILD_LEGACY_APPS=ON`
- Enable tester apps: `-DBUILD_TESTER_APPS=ON`
- Unit tests are on by default: `-DBUILD_UNIT_TESTS=ON`

Run tests:

- `ctest --test-dir build-vcpkg -C Debug --output-on-failure`

Visual Studio (CMake) note:

- With CMake mode, discovered GoogleTests are registered via CTest and should appear in Test Explorer after configure/build.

## Documentation map

- `docs/overview.md` - project structure and architecture overview
- `docs/transport_modes_strategy.md` - transport-mode plan (Direct, Legacy SmartDashboard, NetworkTables V4)
- `docs/testing.md` - test targets and test commands
- `docs/project_history.md` - durable migration and milestone history
- `Agent_Session_Notes.md` - lean handoff notes for next coding session
