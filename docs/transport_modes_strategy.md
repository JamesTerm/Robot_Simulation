# Transport Modes Strategy

This document is the implementation handoff for transport-mode work in `Robot_Simulation`.

## Primary product goal

Support explicit runtime selection between three connection modes:

1. `Direct Connect` (native protocol, primary near-term path)
2. `Legacy SmartDashboard` (legacy NT-compatible baseline)
3. `Shuffleboard` (future mode built on top of stable baseline behavior)

## Sequencing rule

Before starting Shuffleboard behavior, establish Direct Connect first.

- Direct Connect is the first-class implementation target.
- Legacy SmartDashboard compatibility is preserved as a known-good oracle.
- Shuffleboard support is added later and should translate to/from the native Direct model where practical.

## Architecture intent

- Keep existing SmartDashboard/legacy behavior stable and swappable.
- Introduce mode-selection boundaries so transport changes do not require robot-behavior rewrites.
- Treat native Direct payloads as the internal canonical shape where possible.
- Add compatibility adapters for legacy/shuffleboard-facing contracts.

## Required mode behavior

### Direct Connect

- Lowest-friction local integration mode.
- Publish and consume bool/double/string telemetry + commands.
- Include chooser-style topics and selection writeback support.

### Legacy SmartDashboard

- Maintain legacy NT-compatible behavior.
- Keep this path deterministic for regression comparison.
- Do not regress existing SmartDashboard interoperability.

### Shuffleboard (later phase)

- Implement after Direct mode contract is stable.
- Additive behavior only; avoid breaking legacy profile semantics.
- Translate shuffleboard-facing topics into internal native representation.

## Chooser contract baseline

Use chooser base key like `Test/AutoChooser` with:

- `<base>/.type` = `String Chooser`
- `<base>/options`
- `<base>/default`
- `<base>/active`
- `<base>/selected`

Preferred NT encoding for options is string array (comma-string fallback acceptable while bridging).

## References

Protocol and implementation references in `../SmartDashboard`:

- `docs/robot_simulation_transport_guide.md`
- `src/transport/dashboard_transport.cpp`
- `src/app/main_window.cpp`
- `ClientInterface_direct/tests/direct_publisher_tests.cpp`
- chooser widget tests under `SmartDashboard/tests/`

## Implementation phases

1. Add explicit mode switch (`Direct`, `Legacy SmartDashboard`, `Shuffleboard`) with startup logging.
2. Implement shared scalar publish/read contract in Direct + Legacy modes.
3. Implement chooser publish/read contract in Direct + Legacy modes.
4. Add mode-scoped tests for scalar and chooser roundtrip.
5. Add Shuffleboard path once Direct and Legacy are stable.

## Session handoff note

Use this file as the planning source for the next feature branch. Keep milestone outcomes in `docs/project_history.md` and short handoff context in `Agent_Session_Notes.md`.
