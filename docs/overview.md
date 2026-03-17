# Robot Simulation Overview

This document preserves the longer-form project orientation that previously lived in the repository readme.

## High-level structure

- `Source/Application` - top-level app assemblies (including `DriverStation` and legacy app entry points)
- `Source/Base` - shared foundational framework utilities
- `Source/Libraries` - third-party and shared library code (`SmartDashboard`)
- `Source/Modules` - module implementations grouped by role:
  - `Input` - joystick and AI goal input flows
  - `Robot` - drive/motion/odometry behavior
  - `Output` - simulation viewers and display adapters
- `Source/Properties` - script-driven property loading and calibration data plumbing

## Application model

- `DriverStation` is the primary app target and main launcher.
- `RobotAssembly` is a command-driven assembly app useful for integration exercises.
- Many module folders also include standalone tester executables for focused behavior checks.

## Module intent

- Modules are designed to be composable and independently testable.
- Callback-driven boundaries are used to reduce coupling between input, robot, and output layers.
- Simulation and real-robot pathways are intended to share common robot-side logic where practical.

## Input / Robot / Output flow

- **Input** selects teleop controls or autonomous goals and emits desired behavior.
- **Robot** translates desired behavior to motion/kinematics and internal state updates.
- **Output** visualizes and/or publishes resulting robot state (simulation and dashboard-style consumers).

## Properties system

- Property loading provides a script/configuration bridge for robot tuning and calibration.
- The design goal is to keep robot logic decoupled from any one scripting format.
- Asset/registry style key-value exchange is used so modules consume stable property keys.

## Historical note on OSG

- Older workflows sometimes used prebuilt OSG binaries dropped into module folders.
- Current CMake migration path prefers package-managed OSG via vcpkg.

## Documentation map

- `docs/project_history.md` - lean milestone log
- `docs/journal/` - longer-form dated engineering stories and debugging writeups
- `docs/learning/` - reusable lessons and teaching-oriented engineering notes
- `docs/testing.md` - build/test commands and validation flow
- `docs/transport_modes_strategy.md` - transport direction and compatibility planning
