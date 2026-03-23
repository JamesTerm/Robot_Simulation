# Agent session notes

- Keep this file short and handoff-focused.
- Move durable milestone history to `docs/project_history.md`.
- Use `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md` as the canonical long-term Native Link rollout note.

## Workflow note

- `apply_patch` expects workspace-relative paths with forward slashes.
- Use CRLF line endings for C++ source files in this repo.
- Read nearby `Ian:` comments before editing and add new ones where session, ownership, carrier, or runtime-mode lessons would be easy to lose.

## Active Native Link context

- `Source/Application/DriverStation/DriverStation/NativeLink.h` exposes the active carrier/config boundary for the simulator-owned authority and test client.
- Current working backends: SHM + named events for diagnostics; localhost TCP for the reference/runtime path.
- `Robot_Simulation` is the first reference authority/example for Native Link semantics but should not become the permanent home of all reusable authority-side logic.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.
- Debug builds now expose a manual Native Link carrier picker in the DriverStation dialog for SHM vs TCP manual comparisons.
- `DriverStation_TransportSmoke` supports `--startup-delay-ms` and `--test-move` for longer manual observe runs without rebuilding.

## Transport status — COMPLETE

- SHM transport declared stable as of 2026-03-21.
- TCP authority work completed and merged 2026-03-21. Three blockers fixed: bind 0.0.0.0, INI default from `GetDefaultNativeLinkCarrier()`, carrier combo bounce.
- Both carriers sit under the same Native Link semantic contract. Near-term roadmap items 1-5 from `native_link_rollout_strategy.md` are done.

## Key invariants (do not break)

- `Core::PublishInternal` auto-register only applies when `allowServerOnly=true`. Client writes on unknown topics must still be rejected.
- `RegisterDefaultTopics` must not grow to list every TeleAutonV2 key — the auto-register path handles those.
- `TeleAutonV2` publishes ~26 keys per loop. `Heading` (line 324) and `Travel_Heading` (line 307) are unconditional — both are now delivered to the dashboard.

## Strategy reminders

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- Product stance: TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier and should stay reachable through developer overrides, not as a normal team-facing mode toggle.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.

## Cross-repo sync rule

- This repo and `D:\code\SmartDashboard` share the Native Link contract, carrier implementations, and plugin boundary.
- When either repo's session notes or strategy docs change, check the other side for consistency — especially around invariants, carrier defaults, and plugin support iterations.
- The canonical long-term rollout strategy lives in `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md`.

## Active feature: Shuffleboard transport (`feature/shuffleboard-transport`)

### Goal

Make the simulator talk to the official Shuffleboard app (`D:\code\Shuffleboard`) over NT4 (WebSocket). The simulator takes the lead — once it can publish to Shuffleboard, SmartDashboard gets a plugin on its own `feature/shuffleboard-transport` branch later.

### Principles

- **Simulation-first:** prove the protocol here before involving SmartDashboard code.
- **Support, not imitate:** give teams enough Shuffleboard compatibility to try SmartDashboard, then encourage Native Link for full support.
- **Keep it simple:** only implement what we currently support in our widget set. Skip advanced widgets (Compass, Camera, Multi-plot, Field2d, Mechanism2d, command/subsystem panels) — add those incrementally later.
- **Drop what doesn't align:** if a Shuffleboard convention conflicts with our vision, we can choose not to support it.

### Existing scaffolding

- `ConnectionMode::eShuffleboard` enum is wired through UI combo, F7 shortcut, command-line `shuffle`, and INI persistence.
- `ShuffleboardBackend` in `Transport.cpp` is a stub that currently delegates to the legacy NT2 path.
- Shuffleboard is classified in `UsesLegacyTransportPath()` — this will need to change once a real NT4 backend exists.
- No NT4 protocol code exists anywhere in the codebase yet.

### What to publish (iteration 1 — current widget types only)

| Category | Keys | Type |
|---|---|---|
| Motion | `Velocity`, `Rotation Velocity`, `X_ft`, `Y_ft`, `Heading`, `Travel_Heading` | double |
| Wheel velocity | `Wheel_{fl,fr,rl,rr}_Velocity` | double |
| Wheel voltage | `Wheel_{fl,fr,rl,rr}_Voltage` | double |
| Wheel encoder | `wheel_{fl,fr,rl,rr}_Encoder` | double |
| Swivel voltage | `Swivel_{fl,fr,rl,rr}_Voltage` | double |
| Swivel angle | `swivel_{fl,fr,rl,rr}_Raw` | double |
| AI / control | `Timer`, `TestMove`, `AutonTest` | double |
| Bool controls | `SafetyLock_Drive`, `TestVariables_set` | bool |
| Chooser | `Test/Auton_Selection/AutoChooser/{.type,options,default,active,selected}` | string / string[] |

### Skipped for now (future iterations)

- Predicted odometry (`predicted_*`)
- PID monitor keys
- Tank drive keys (test-only)
- Advanced Shuffleboard widget types: Compass, Camera, Multi-plot, Field2d, Mechanism2d, command/subsystem panels

### Technical approach

1. Study the NT4 WebSocket protocol (Shuffleboard 2026.2.2 uses NT4 over `ws://host:5810/nt/...`).
2. Implement a minimal NT4 server or use the existing NT server with NT4 framing.
3. Replace `ShuffleboardBackend` stub with a real backend that publishes the iteration-1 keys.
4. Verify by launching `run_shuffleboard_local.bat` against the running DriverStation.
5. Document what Shuffleboard expects vs what we choose to support, to inform the SmartDashboard plugin later.

### Immediate next-session focus

1. Research the NT4 WebSocket wire protocol (message types, topic announcement, value publish).
2. Determine whether to implement a standalone NT4 server or adapt the existing NT infrastructure.
3. Begin the minimal NT4 server implementation in the DriverStation transport layer.
