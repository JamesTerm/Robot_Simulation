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

### Implementation status — NT4 server working, Shuffleboard connects

**Completed:**
- `NT4Server.h/.cpp` (~860 lines) — full NT4 WebSocket server on port 5810 with custom MessagePack encoder, JSON announce messages, retained-value cache, broadcast to all clients, client message handling.
- `ShuffleboardBackend` in `Transport.cpp` replaced with real NT4 backend implementing `SmartDashboardDirectPublishSink`. Shuffleboard removed from `UsesLegacyTransportPath()`.
- `TransportSmoke.cpp` rewritten for headless shuffleboard mode: flexible `--mode` arg, skip OSG viewer/TeleAuton, SEH crash handler, continuous 10Hz publish loop.
- IXWebSocket overlay port at `overlay-ports/ixwebsocket/` fixes `Sec-WebSocket-Protocol` negotiation (see critical bug below).
- Shuffleboard connected successfully, stable 30+ seconds, 300+ value updates sent.

### Critical discoveries

**IXWebSocket subprotocol bug (FIXED via overlay port):**
- IXWebSocket v11.4.6 server-side does NOT handle `Sec-WebSocket-Protocol` at all. The `serverHandshake()` doesn't read or echo the header.
- Initial fix echoed back the entire comma-separated header, which caused Shuffleboard to connect then immediately disconnect in a loop (RFC 6455 requires exactly ONE selected protocol).
- Final fix: parse comma-separated header, pick first protocol, echo only that one. Applied as vcpkg overlay port.
- Install with: `"D:/code/vcpkg/vcpkg.exe" install ixwebsocket:x64-windows --overlay-ports="D:/code/Robot_Simulation/overlay-ports"`
- Patch file MUST use LF line endings and git-style `a/` `b/` path prefixes.

**Shuffleboard server address (NOT command-line args):**
- `localhost` passed to `java -jar Shuffleboard.jar localhost` is completely ignored — Shuffleboard has no argument parsing.
- Server address stored via Java Preferences API → Windows Registry: `HKCU\Software\JavaSoft\Prefs\edu\wpi\first\shuffleboard\plugin\networktables` (key: `server`).
- Default in code is `"localhost"` so it works without registry changes, but can be set explicitly:
  ```powershell
  Set-ItemProperty -Path 'HKCU:\Software\JavaSoft\Prefs\edu\wpi\first\shuffleboard\plugin\networktables' -Name 'server' -Value 'localhost' -Type String
  ```

**Headless crash workaround:**
- `RobotTester_init()` → `TeleAuton_V2::init()` creates an OSG 3D viewer, which crashes (0xC0000005) in headless/RDP sessions.
- Smoke test in shuffleboard mode skips `RobotTester_init()` entirely; transport is already initialized by `SetConnectionMode()`.

### Build commands

```bash
# Configure (fresh build_nt4 directory)
cmake -G "Visual Studio 17 2022" -B build_nt4 -DCMAKE_TOOLCHAIN_FILE="D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows -DVCPKG_OVERLAY_PORTS="D:/code/Robot_Simulation/overlay-ports"

# Build
cmake --build build_nt4 --target DriverStation --config Release
cmake --build build_nt4 --target DriverStation_TransportSmoke --config Release

# Run smoke test
build_nt4\bin\Release\DriverStation_TransportSmoke.exe --mode shuffle
```

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

### Next steps

1. **Visual verification** — With Shuffleboard GUI visible (interactive session, not headless RDP), verify topics appear in Sources panel and values update live.
2. **Test with full DriverStation** — `DriverStation.exe` in shuffleboard mode (requires display for OSG viewer).
3. **Bidirectional support** — Handle Shuffleboard writing back chooser selections (incoming `publish` + value updates from clients).
4. **Expand published keys** — Smoke test currently publishes 6 keys + chooser. Full TeleAutonV2 publishes ~49 keys.
