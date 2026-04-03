# Curivator Manipulator — Context & Architecture Reference

This document captures everything needed for session handoff regarding the excavator arm
manipulator plugin (Phases 1–1.5 + Layer 2 of the Command/Subsystem status display work).

## Overview

The manipulator is based on the **Curivator excavator arm** — a 4-year NASA SRR project
(2015–2019) with 5 joints (Turret, BigArm, Boom, Bucket, Clasp).  We port 4 joints
(omitting Turret) into the modern Robot_Simulation framework as an optional, additive
plugin behind the `ManipulatorPlugin` base class.

Reference source: `D:\code\Curivator\Curivator\src\Curivator_Robot.cpp`

## Two-Layer Architecture

### Layer 1 — Physical dart joints (direct mode)

Four `RotarySystem_Position` joints (BigArm, Boom, Bucket, Clasp) with PID, potentiometer
feedback, and voltage integrator.  Keys 1-8 directly drive individual darts.  Active when
`enable_arm_auto_position=false`.

### Layer 2 — Virtual 3D-position joints (auto-position mode)

Four `Ship_1D` entities (arm_xpos, arm_ypos, bucket_angle, clasp_angle) whose `GetPos_m()`
returns the desired bucket tip XY position and angle.  When `enable_arm_auto_position=true`,
these are the MASTER: each frame their positions are read, IK computes shaft lengths, and
`SetIntendedPosition` drives the physical joints.  FK feedback writes actual bucket position
back.  Keys j/k, ;/l, i/u, o/p drive virtual joints.

A SmartDashboard checkbox (or `SetEnableArmAutoPosition(bool)` for tests) toggles between
layers.

## Application Integration

```
ManipulatorPlugin (physics base, RobotAssembly_static)
  ├── CreateUI() → unique_ptr<ManipulatorUI_Plugin>
  ├── SetupUI(ManipulatorUI_Plugin*) — wires FK state callback
  ├── SetVoltageDetached(bool) / GetVoltageDetached()
  ├── SetJointInput(jointIndex, normalizedVelocity) — Layer 1
  ├── Set3DPositionInput(jointIndex, normalizedVelocity) — Layer 2
  ├── SetEnableArmAutoPosition(bool) — layer toggle
  ├── GetJointCount()
  └── ExcavatorArm (concrete implementation)

ManipulatorUI_Plugin (viz base, OSG_Viewer DLL)
  └── ManipulatorArm_UI (pImpl → ManipulatorArm_UI_Internal inside OSG_Viewer.cpp)

TeleAutonV2 (application layer):
  m_manipulator (unique_ptr<ManipulatorPlugin>)
  m_manipulatorUI (unique_ptr<ManipulatorUI_Plugin>)
  UpdateScene(rootNode, geode) calls m_manipulatorUI->UpdateScene()
  UpdateVariables() reads SmartDashboard voltage detach booleans
  GetInputSlice() translates keyboard keys into SetJointInput() or Set3DPositionInput()
```

### Hot-swappable design

- `m_manipulator == nullptr` → no manipulator; simulation runs exactly as before.
- Each `ManipulatorPlugin` creates its own visualization via `CreateUI()` and wires the
  state callback via `SetupUI()`.  Swapping the plugin automatically swaps the renderer.
- `TeleAutonV2` never needs to know the concrete UI type.

## Joint configuration

### Physical joints (Layer 1)

| Joint   | Index | Min (in) | Max (in) | Starting (in) | Layer 1 Keys |
|---------|-------|----------|----------|----------------|--------------|
| BigArm  | 0     | 1.0      | 10.0     | 6.0            | 1/2          |
| Boom    | 1     | 1.0      | 10.0     | 6.0            | 3/4          |
| Bucket  | 2     | 0.0      | 12.0     | 6.0            | 5/6          |
| Clasp   | 3     | 0.0      | 7.0      | 3.5            | 7/8          |

Ranges ported from CurivatorRobot.lua (lines 237-373) with `pot_offset`.

### Virtual joints (Layer 2)

| Joint         | Index | min_range | max_range | starting_pos | max_speed | accel/brake | Layer 2 Keys |
|---------------|-------|-----------|-----------|--------------|-----------|-------------|--------------|
| arm_xpos      | 0     | 12.79     | 55.29     | 32.80        | 6.0       | 10/10       | k(+) / j(-)  |
| arm_ypos      | 1     | -20       | 40        | -0.976       | 6.0       | 10/10       | ;(+) / l(-)  |
| bucket_angle  | 2     | 0         | 180       | 78.07        | 36.66     | 50/50       | i(+) / u(-)  |
| clasp_angle   | 3     | -7        | 100       | 13.19        | 36.66     | 5/5         | o(+) / p(-)  |

All virtual joints have `MaxAccelFwd/Rev=10000` (effectively unlimited).
`arm_ypos` and `clasp_angle` have `using_range=true`; `arm_xpos` and `bucket_angle` have
`using_range=false`.

### Physical joint PID properties (all from CurivatorRobot.lua)

| Property                | BigArm    | Boom      | Bucket    | Clasp     |
|-------------------------|-----------|-----------|-----------|-----------|
| PID_Up/Down             | 100/0/25  | 100/0/25  | 100/0/25  | 100/0/25  |
| UsePID_Up_Only          | true      | true      | true      | true      |
| UseAggressiveStop       | true      | true      | true      | true      |
| ToleranceConsecutiveCount | 1       | 1         | 1         | 1         |
| MAX_SPEED               | 6.0       | 6.0       | 3.0       | 3.0       |
| MaxAccelFwd/Rev         | 5/5       | 25/25     | 500/500   | 500/500   |
| PrecisionTolerance      | 0.3       | 0.15      | 0.0125    | 0.0125    |
| Positive_DeadZone       | 0.23      | 0.37      | 0.0       | 0.0       |
| Negative_DeadZone       | -0.23     | -0.37     | 0.0       | 0.0       |
| VelocityPredictUp/Down  | 0.2/0.2   | 0.2/0.2   | 0.0/0.0   | 0.0/0.0   |
| Range                   | 1-10      | 1-10      | 0-12      | 0-7       |
| MinLimitRange           | -10       | -10       | -12       | -7        |
| MaxLimitRange           | 10        | 10        | 12        | 7         |
| Mass                    | 1.5       | 1.5       | 1.5       | 1.5       |

## TimeSlice ordering (CRITICAL — matches Curivator)

**Curivator order (lines 686-740):**
1. `Robot_Control_TimeChange(dTime_s)` — simulators tick
2. `Swerve_Robot::TimeChange(dTime_s)` — drive wheels only
3. `mp_Arm[i]->AsEntity1D().TimeChange(dTime_s)` — ALL arm entities (physical AND virtual)
4. Read virtual joint positions → IK → `SetIntendedPosition` on physical joints

**Our order (matches):**
1. Set virtual joint velocities from keyboard
2. Physical joints TimeSlice (processes PREVIOUS frame's SetIntendedPosition)
3. Virtual joints TimeChange
4. Read virtual joint positions → IK → SetIntendedPosition (for NEXT frame)

## Root cause: Layer 2 zero-voltage bug (FIXED — commit 3e27df2)

**The bug**: ALL physical joints produced zero voltage when driven by Layer 2
(SetIntendedPosition from IK).

**Root cause chain**:
1. Robot_Simulation's `RotarySystem.cpp` (line ~297) uses `MinLimitRange`/`MaxLimitRange`
   to set PID output ranges: `SetOutputRange(MinLimitRange * 0.99, MaxLimitRange * 0.99)`.
   This is a `#else` path that **differs from Curivator** — Curivator uses symmetric
   `±MaxSpeed` for PID output range (line 192 of Curivator's Rotary_System.cpp).
2. `MinLimitRange` and `MaxLimitRange` default to `0.0` in `RotaryProperties_Legacy.h`
   (line 107) and were **never set** by ExcavatorArm.
3. This produced `SetOutputRange(0.0, 0.0)` — both min and max output = 0.
4. In `PIDController2::operator()` (PIDController.h line 246-263), when `m_I = 0` and
   `m_minimumOutput = 0`: `TotalErrorCheck = 0`, which is NOT `> 0` (m_minimumOutput),
   so it enters the else branch at line 251 and computes `MinError = (0 - 0) / 0 = NaN`.
5. `m_totalError` becomes NaN. Even though `m_I = 0`, the computation
   `m_I * m_totalError = 0 * NaN = NaN` in IEEE 754.
6. `m_result = P*error + I*totalError + D*(error-prev) = NaN` → `m_ErrorOffset = NaN`.
7. `Voltage = (Velocity + NaN) / MaxSpeed = NaN` → clamped to 0 at line 728
   (`//is nan case`).

**Why Curivator didn't have this bug**: Curivator's `Rotary_System.cpp` line 188-192 uses
`MaxSpeedReference` (symmetric ±MaxSpeed) for PID output range, NOT
MinLimitRange/MaxLimitRange. So `m_minimumOutput` was always negative (e.g., -2.97), and
the `0 > -2.97` check passed safely.

**Fix**: Set `MinLimitRange = -maxRange` and `MaxLimitRange = maxRange` for symmetric PID
output range. This ensures `m_minimumOutput < 0`, avoiding the 0/0 division.

## Previous bugs fixed

### LoopState: eClosed required for feedback (FIXED — commit 729cd3f)

Originally we used `LoopState=eNone` which was WRONG.  With `eNone`, `GetActualPos()`
returns `GetPos_m()` (Ship_1D internal) instead of `GetRotaryCurrentPorV()` (our odometry
callback).  The PID saw its own internally-integrated position, never the "real" simulated
dart position in `m_simulated_positions`.

**Fix**: Changed to `LoopState=eClosed`.  Now `GetActualPos()` calls the odometry callback
→ `m_simulated_positions[i]`, closing the loop.

### Actuator drift at startup (FIXED — commit 729cd3f)

Race condition: `Ship_1D` constructor sets `m_Position = 0.0`, but `m_simulated_positions[i]
= startingPos`.  If TimeSlice fires before Reset(), PID error = startingPos → max voltage
→ drift.

**Fix**: Added `Reset()` at end of `ExcavatorArm::Init()`.

### RotaryPosition_Internal::Reset() hardcoded position (FIXED — commit 729cd3f)

`m_Position = 0.0` was hardcoded instead of `m_Position = position`.  Fixed.

### IK constant error (FIXED — commit 2ce1993)

Used wrong geometry constant in IK calculation.

### MaxSpeed_Reverse positive instead of negative (FIXED — commit 2ce1993)

`MaxSpeed_Reverse` was set to a positive value, causing negative velocities to be clamped
to a positive number, breaking retraction for all joints.

### Voltage callback dt fix (FIXED — commit 2ce1993)

Voltage callback used wrong time delta.

### UI smearing (FIXED — commit 2ce1993)

OSG visualization accumulated geometry without clearing.

## SmartDashboard in unit tests

`SmartDashboard` defaults to `eDirectConnect` mode where `PutBoolean`/`TryGetBoolean` are
no-ops.  `SetEnableArmAutoPosition(bool)` bypasses SmartDashboard for tests.

## Curivator Robot_TesterCode path — how potentiometer simulation works

The original Curivator source uses `#define Robot_TesterCode` for simulation. When defined,
the feedback loop works as follows:

### Voltage output chain (UpdateVoltage)

```
Curivator_Robot_Control::UpdateVoltage(index, Voltage)
  → m_Potentiometer[index].UpdatePotentiometerVoltage(SafetyLock ? 0.0 : Voltage)
    → Potentiometer_Tester2::UpdatePotentiometerVoltage(Voltage)
      → Ship_1D::SetRequestedVelocity(Voltage * MaxSpeed)
  → m_Potentiometer[index].TimeChange()
    → Ship_1D::TimeChange(m_Time_s)  // integrates position
```

`Potentiometer_Tester2` inherits from `Ship_1D`.  `UpdatePotentiometerVoltage()` converts
voltage to a velocity request, and `TimeChange()` integrates that into position.

### Odometry/feedback chain (GetRotaryCurrentPorV)

```
Curivator_Robot_Control::GetRotaryCurrentPorV(index)
  → result = m_Potentiometer[index].GetPotentiometerCurrentPosition()
    → Potentiometer_Tester2::GetPotentiometerCurrentPosition()
      → return GetPos_m()  // Ship_1D's current position
  → NormalizedResult = (result - MinRange) / (MaxRange - MinRange)
  → SmartDashboard::PutNumber(prefix + "_Raw", result)       // position in ship units
  → SmartDashboard::PutNumber(prefix + "Pot_Raw", normalized) // 0-1 normalized
```

### Key insight

The `Potentiometer_Tester2` IS both the physics model AND the feedback source.  It
receives voltage, integrates it into position, and returns that position as the sensor
reading.  `GetRotaryCurrentPorV` feeds back into `Rotary_Position_Control::GetActualPos()`,
closing the loop.

### Time delta management

```
Curivator_Robot_Control::Robot_Control_TimeChange(dTime_s)
  → for i in 0..4: m_Potentiometer[i].SetTimeDelta(dTime_s)
```

Each frame, the time delta is cached in `Potentiometer_Tester2::m_Time_s`, then used by
`TimeChange()` for position integration.

## Our implementation vs. Curivator's

### What we replicate

- Voltage → position integration (our voltage callback lambda in ExcavatorArm::Init)
- Position feedback via odometry callback → `m_simulated_positions`
- `_Raw` and `Pot_Raw` telemetry publication
- Forward kinematics (faithful port of the geometry math)
- Inverse kinematics (faithful port of ComputeArmPosition)
- Safety lock equivalent: `VoltageDetached` boolean toggle
- Full two-layer control (virtual 3D-position joints → IK → physical joints)
- Match-tuned PID gains, tolerances, dead zones from CurivatorRobot.lua

## Ship_1D / Entity1D architecture

- **Ship_1D** inherits from legacy **Entity1D** (in `Ship1D_Legacy.h`), which holds
  `m_Position` (double) and `m_Physics` (PhysicsEntity_1D).
- `GetPos_m()` returns `m_Position`.  `SetPos_m()` sets it directly.
- `ResetPosition(pos)` zeros velocity and sets `m_Position = pos`, `m_IntendedPosition = pos`.
- `Initialize()` sets `m_StartingPosition` from props and `m_IntendedPosition = 0.0` but
  does NOT call `ResetPosition()`.
- `TimeChange()` is a semi-implicit Euler integrator: computes velocity from intended
  position vs current, applies force, integrates position.
- With `LoopState=eClosed` and `m_PotentiometerState=eActive`, `GetActualPos()` returns
  the odometry callback (our simulated position).  The PID compares this against
  `GetPos_m()` (the Ship_1D's projected position) to compute error offset.

## RotarySystem_Position internal wiring

```
RotaryPosition_Internal : public Rotary_Control_Interface
  m_rotary_legacy  = unique_ptr<Rotary_Position_Control>
  m_VoltageCallback  = client's voltage output lambda
  m_OdometryCallack  = client's position feedback lambda
  m_OpenLoop_position  = internal position tracker (unused when odometry callback set)

  GetRotaryCurrentPorV()  → m_OdometryCallack ? m_OdometryCallack() : m_OpenLoop_position
  UpdateRotaryVoltage()   → m_VoltageCallback(Voltage)
  Init()                  → creates Rotary_Position_Control, copies props, calls Initialize()
  SetPosition()           → stores position, calls SetIntendedPosition()
  SetIntendedPosition()   → sets PID's intended position without modifying m_Position
  TimeSlice()             → calls m_rotary_legacy->TimeChange()
  Reset(position)         → sets m_Position, calls ResetPosition()
```

## Key RotarySystem.cpp code paths (for debugging reference)

- **Lines 291-304**: PID output range setup uses `MinLimitRange`/`MaxLimitRange` (differs
  from Curivator which uses `±MaxSpeed`).  This is the code that caused the NaN bug.
- **Lines 369-784**: `Rotary_Position_Control::TimeChange` — main PID loop.
- **Lines 766-775**: Limit switch checks (DidHitMaxLimit/DidHitMinLimit).
- **Line 728**: NaN→0 voltage clamp (`//is nan case`).
- **Line 538**: `SetPos_m(NewPosition)` resets Ship_1D's position to actual position.

## Voltage detach

### Drive

- `SwerveRobot_Internal` has `m_VoltageDetached` bool.
- After all rotary loops complete in `TimeSlice()`, if detached, `m_Voltage` is zeroed.
- Exposed via `SwerveRobot::SetVoltageDetached(bool)` / `GetVoltageDetached()`.

### Manipulator

- `ExcavatorArm` has `m_voltageDetached` bool.
- Voltage callback returns early (no delta applied) when true.
- Exposed via `ManipulatorPlugin::SetVoltageDetached(bool)` / `GetVoltageDetached()`.

### SmartDashboard wiring

- `TeleAutonV2::UpdateVariables()` reads `"Drive/VoltageDetached"` and
  `"Manipulator/VoltageDetached"` booleans each frame.
- Values are NOT pre-published on init — SmartDashboard client sets them first.
  If no key exists, `GetBoolean` defaults to `false` (voltage active).

## Keyboard bindings

### Drive (existing)

| Key(s)              | Axis | Action              |
|---------------------|------|---------------------|
| W / Up arrow        | m_Y  | forward (negative)  |
| S / Down arrow      | m_Y  | reverse (positive)  |
| Left / Right arrows | m_X  | strafe              |
| A / D               | m_Z  | turn                |
| X                   | all  | zero all drive axes |
| Space               | all  | full reset          |

### Layer 1 — Manipulator (direct dart control)

| Keys | Joint   | Action            |
|------|---------|-------------------|
| 1/2  | BigArm  | extend / retract  |
| 3/4  | Boom    | extend / retract  |
| 5/6  | Bucket  | extend / retract  |
| 7/8  | Clasp   | extend / retract  |

### Layer 2 — 3D Position (auto-position mode)

| Key | Event                  | Direction      |
|-----|------------------------|----------------|
| k   | arm_xpos_Advance       | positive (+)   |
| j   | arm_xpos_Retract       | negative (-)   |
| ;   | arm_ypos_Advance       | positive (+)   |
| l   | arm_ypos_Retract       | negative (-)   |
| i   | bucket_angle_Advance   | positive (+)   |
| u   | bucket_angle_Retract   | negative (-)   |
| o   | clasp_angle_Advance    | positive (+)   |
| p   | clasp_angle_Retract    | negative (-)   |

## Telemetry keys published

### Per-joint (4 joints)

- `Manipulator/<Joint>_ShaftPos` — shaft extension in inches
- `Manipulator/<Joint>_Raw` — same as ShaftPos (matches TesterCode `_Raw` pattern)
- `Manipulator/<Joint>Pot_Raw` — normalized 0–1 (matches TesterCode `Pot_Raw` pattern)
- `Manipulator/<Joint>Voltage` — PID output voltage (normalized -1 to 1)

### Forward kinematics

- `Manipulator/BigArm_Angle`, `_Length`, `_Height`
- `Manipulator/Boom_Angle`, `_Length`, `_Height`
- `Manipulator/Bucket_Angle`, `_Length`, `_TipHeight`, `_RoundEndHeight`
- `Manipulator/Clasp_Angle`, `_Distance`, `_Height`, `_MinHeight`

## Test coverage

12 unit tests in `tests/excavator_arm_tests.cpp`:

| Test | Layer | Description |
|------|-------|-------------|
| InitialFKIsStable | 1 | FK produces consistent angles from starting positions |
| Layer1_PositiveInputExtendsBigArm | 1 | +1.0 input extends BigArm dart |
| Layer1_NegativeInputRetractsBigArm | 1 | -1.0 input retracts BigArm dart |
| Layer1_ExtendAndRetractAreOpposite | 1 | Extend and retract produce opposite deltas |
| Layer1_AllJointsRespond | 1 | All 4 joints produce nonzero FK delta |
| Layer2_StableWithNoInput | 2 | No drift when auto-position enabled with no input |
| Layer2_ActualMatchesDesiredOnReset | 2 | FK actual matches virtual joint desired on reset |
| Layer2_AutoOn_NoInputNoMovement | 2 | No movement with auto-position on and no input |
| Layer2_AutoOff_NoInputNoMovement | 2 | No movement with auto-position off and no input |
| IK_FK_RoundTrip | - | IK→FK round trip produces consistent results |
| Layer2_NoDriftOver10Seconds | 2 | No drift over 625 frames (10 seconds) |
| Layer2_Diagnostic_BucketAngleChain | 2 | Full Layer 2 chain: input→virtual→IK→PID→voltage→FK |

## Files

### Created

| File | Purpose |
|------|---------|
| `Source/Modules/Robot/Manipulator/Manipulator/ManipulatorPlugin.h` | Base class interface |
| `Source/Modules/Robot/Manipulator/Manipulator/ExcavatorArm.h` | Geometry constants, FK/IK structs, class declaration |
| `Source/Modules/Robot/Manipulator/Manipulator/ExcavatorArm.cpp` | FK, IK, telemetry, voltage/odometry callbacks, two-layer control |
| `Source/Modules/Output/OSG_Viewer/OSG_Viewer/ManipulatorUI_Plugin.h` | Abstract UI base class |
| `Source/Modules/Output/OSG_Viewer/OSG_Viewer/ManipulatorArm_UI.h` | Concrete UI class (pImpl) |
| `tests/excavator_arm_tests.cpp` | 12 unit tests covering Layer 1, Layer 2, FK/IK |

### Modified

| File | Changes |
|------|---------|
| `Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.cpp` | `ManipulatorArm_UI_Internal` class, wrapper methods |
| `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV2.cpp` | Manipulator init/UI/timeslice wiring, voltage detach reads, keyboard bindings (both layers) |
| `Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot.h` | `SetVoltageDetached(bool)`, `GetVoltageDetached()` |
| `Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot.cpp` | `m_VoltageDetached`, voltage zeroing, `RotaryPosition_Internal::Reset()` bug fix, `SetIntendedPosition` public API |
| `Source/Properties/RegistryV1.h` | `Rg_(Build_enable_manipulator)` |
| `Source/Properties/script_loader_example.cpp` | `put_bool(csz_Build_enable_manipulator, true)` |
| `CMakeLists.txt` | Added `ExcavatorArm.cpp` and `excavator_arm_tests.cpp` |

### Legacy files referenced (DO NOT modify permanently)

| File | Relevant sections |
|------|-------------------|
| `RotarySystem.cpp` | Lines 291-304 (PID output range), 369-784 (TimeChange), 728 (NaN clamp) |
| `Ship1D_Legacy.h` | Lines 303-483 (TimeChange), 559-572 (SetIntendedPosition/SetRequestedVelocity) |
| `PIDController.h` | Lines 246-263 (I-term clamping, the 0/0 NaN source) |
| `Physics.h` | GetVelocityFromDistance_Linear, ApplyFractionalForce |
| `RotaryProperties_Legacy.h` | Line 107 (default MinLimitRange=MaxLimitRange=0.0) |

## Phase plan

| Phase | Status | Description |
|-------|--------|-------------|
| 1     | DONE   | Standalone manipulator module (FK, IK, telemetry, 4 joints) |
| 1.5   | DONE   | Side-view arm visualization (OSG_Viewer) |
| Stab. | DONE   | Actuator drift fix, voltage detach, LoopState=eClosed, keyboard bindings |
| L2    | DONE   | Two-layer arm control (virtual 3D-position joints + IK), PID zero-voltage fix |
| 2     | TODO   | Command/Subsystem scheduler layer (NT key patterns) |
| 3     | TODO   | Test against official SmartDashboard 2026 |
| 4     | TODO   | Implement widgets in our SmartDashboard |

## Remaining runtime test

Build `RobotAssembly_static` and `DriverStation`, verify j/k/;/l/i/u/o/p keys move the arm
correctly in the side-view visualization.  This was not done during the Layer 2 sessions
because the unit tests provided sufficient coverage of the control chain.

## WPILib Command/Subsystem NT protocol (for Phase 2)

NT key conventions under `/SmartDashboard/`:

- **Scheduler**: `.type` = `"Scheduler"`, `Names` = string[], `Ids` = int[], `Cancel` = int[]
- **Subsystem**: `.type` = `"Subsystem"`, `.hasDefault`, `.default`, `.hasCommand`, `.command`
- **Command**: `.type` = `"Command"`, `/running` = bool
- The `.type` topic uses `publishEx` with property JSON `{"SmartDashboard":"<type>"}`

### NT4 Server gap

`BuildAnnounceJson` publishes empty `"properties": {}`.  Needs
`{"SmartDashboard":"Scheduler"}` etc. for Phase 2.

## Geometry constants

All geometry constants are in `ExcavatorArm.h` namespace `ExcavatorGeometry`, ported
verbatim from `Curivator_Robot.cpp`.  Values are in inches matching the original CAD.
Do NOT convert to meters — the FK/IK operate entirely in inches.
