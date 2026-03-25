# Transport Modes Strategy

This document is the implementation handoff for transport-mode work in `Robot_Simulation`.

## Primary product goal

Support explicit runtime selection between three connection modes:

1. `Direct Connect` (native protocol, primary near-term path)
2. `Legacy SmartDashboard` (legacy NT-compatible baseline)
3. `NetworkTables V4` (NT4 WebSocket — serves Shuffleboard, Glass, and any WPILib NT4 dashboard)

## Sequencing rule

Before starting NetworkTables V4 behavior, establish Direct Connect first.

- Direct Connect is the first-class implementation target.
- Legacy SmartDashboard compatibility is preserved as a known-good oracle.
- NT4 support is added later and should translate to/from the native Direct model where practical.

## Architecture intent

- Keep existing SmartDashboard/legacy behavior stable and swappable.
- Introduce mode-selection boundaries so transport changes do not require robot-behavior rewrites.
- Treat native Direct payloads as the internal canonical shape where possible.
- Add compatibility adapters for legacy/NT4-facing contracts.

## Required mode behavior

### Direct Connect

- Lowest-friction local integration mode.
- Publish and consume bool/double/string telemetry + commands.
- Include chooser-style topics and selection writeback support.

### Legacy SmartDashboard

- Maintain legacy NT-compatible behavior.
- Keep this path deterministic for regression comparison.
- Do not regress existing SmartDashboard interoperability.

### NetworkTables V4 (later phase)

- Implement after Direct mode contract is stable.
- Additive behavior only; avoid breaking legacy profile semantics.
- Translate NT4-facing topics into internal native representation.
- Serves any NT4-compatible dashboard: Shuffleboard, Glass, AdvantageScope, etc.

## Chooser contract baseline

Use chooser base key like `Test/AutonTest/AutoChooser` with:

- `<base>/.type` = `String Chooser`
- `<base>/options`
- `<base>/default`
- `<base>/active`
- `<base>/selected`

Preferred dashboard-native encoding for options is string array on modern NT paths. Robot_Simulation still publishes comma-string chooser options on its current Direct bridge until the simulator-side direct protocol is upgraded to first-class arrays.

## Key naming policy (scoped vs flat)

To align with modern dashboard behavior (including hierarchical keys used by Shuffleboard, Glass, and other NT4 dashboards), robot-side reads should prefer scoped keys while keeping legacy compatibility aliases.

- Canonical control/input keys should be scoped (for example `Test/AutonTest`).
- Legacy flat aliases (for example `AutonTest`) should remain readable during migration.
- Robot code should not rewrite operator-owned keys during normal read paths.
- If both scoped and flat forms exist, prefer scoped value.

Current migration example:

- `AutonTest` control is treated as dashboard-owned.
- Robot read path checks both `AutonTest` and `Test/AutonTest` to bridge legacy and scoped dashboards.

## References

Protocol and implementation references in `../SmartDashboard`:

- `docs/robot_simulation_transport_guide.md`
- `src/transport/dashboard_transport.cpp`
- `src/app/main_window.cpp`
- `ClientInterface_direct/tests/direct_publisher_tests.cpp`
- chooser widget tests under `SmartDashboard/tests/`

## Implementation phases

1. Add explicit mode switch (`Direct`, `Legacy SmartDashboard`, `NetworkTables V4`) with startup logging.
2. Implement shared scalar publish/read contract in Direct + Legacy modes.
3. Implement chooser publish/read contract in Direct + Legacy modes, including shared `.type/default/options/active/selected` semantics.
4. Add mode-scoped tests for scalar and chooser roundtrip.
5. Add NetworkTables V4 path once Direct and Legacy are stable.

## Session handoff note

Use this file as the planning source for the next feature branch. Keep milestone outcomes in `docs/project_history.md` and short handoff context in `Agent_Session_Notes.md`.
