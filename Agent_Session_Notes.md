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

### Implementation status — NT4 server working, Shuffleboard shows topics

**Completed:**
- `NT4Server.h/.cpp` (~1050 lines) — full NT4 WebSocket server on port 5810 with subscription-driven architecture, custom MessagePack encoder, JSON announce messages, retained-value cache, per-client subscription tracking, client message handling.
- `ShuffleboardBackend` in `Transport.cpp` — real NT4 backend implementing `SmartDashboardDirectPublishSink` (PublishBoolean, PublishNumber, PublishString, PublishStringArray). Shuffleboard removed from `UsesLegacyTransportPath()`.
- `TransportSmoke.cpp` rewritten for headless shuffleboard mode: flexible `--mode` arg, skip OSG viewer/TeleAuton, SEH crash handler, continuous 10Hz publish loop.
- IXWebSocket overlay port at `overlay-ports/ixwebsocket/` fixes `Sec-WebSocket-Protocol` negotiation (see critical bug below).
- Topics appear in Shuffleboard's Sources panel, values update live.
- All diagnostic trace logging removed (TraceLog, RobotTrace, TransportTrace).

### Critical discoveries

**NT4 is subscription-driven (FIXED in NT4Server.cpp):**
- The server must NOT send `announce` messages until the client sends a `subscribe` message with matching topic patterns.
- Our first implementation sent announces immediately on connect. ntcore silently ignores unsolicited announces, which is why Shuffleboard showed nothing despite receiving all the data.
- The correct flow is: (1) client connects, (2) client sends subscribe with topic patterns, (3) server sends announce for matching topics, (4) server sends cached values, (5) future publishes only go to clients with matching subscriptions.
- Per-client tracking: `ClientSubscription` (subuid, topic patterns, prefix flag) and `ClientState` (subscriptions list, announced topic IDs) stored in `m_clientState` map keyed by WebSocket pointer.

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
# Configure (uses existing build-vcpkg directory)
cmake -G "Visual Studio 17 2022" -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE="D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows

# Build
cmake --build build-vcpkg --target DriverStation --config Release
cmake --build build-vcpkg --target DriverStation_TransportSmoke --config Release

# Run smoke test
build-vcpkg\bin\Release\DriverStation_TransportSmoke.exe --mode shuffle
```

Note: The ixwebsocket overlay port only matters at `vcpkg install` time, not at CMake configure time. The patched library is already in vcpkg's `installed/` directory.

**Windows Firewall:** The first time DriverStation.exe listens on port 5810, Windows will prompt to allow it through the firewall. This is a one-time approval.

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

1. **SmartDashboard plugin** — Start the `feature/shuffleboard-transport` branch in `D:\code\SmartDashboard`. The plugin will be an NT4 WebSocket *client* connecting to port 5810, subscribing to topics, and feeding values into SmartDashboard's display pipeline. Follow the existing plugin ABI pattern (`sd_transport_plugin_descriptor_v1`). Phase 1 is receive-only (display telemetry); phase 2 adds write-back for chooser selections.
2. **Bidirectional support** — Handle Shuffleboard writing back chooser selections (incoming `publish` + value updates from clients). This requires implementing `SmartDashboardDirectQuerySource` on the Robot_Simulation side.
3. **Expand published keys** — Smoke test currently publishes 6 keys + chooser. Full TeleAutonV2 publishes ~49 keys.

### NT4 protocol reference (for SmartDashboard plugin authors)

- **Transport:** WebSocket on port 5810, resource path `/nt/<clientname>`
- **Subprotocols:** `v4.1.networktables.first.wpi.edu` (preferred), `networktables.first.wpi.edu` (v4.0 fallback)
- **Control messages:** JSON text frames — each frame is a JSON array of message objects with `method` and `params` keys
- **Server→client methods:** `announce`, `unannounce`, `properties`
- **Client→server methods:** `subscribe`, `unsubscribe`, `publish`, `unpublish`, `setproperties`
- **Value updates:** MessagePack binary frames — each message is 4-element array: `[topicID, timestamp_us, dataType, value]`
- **Data type codes:** boolean=0, double=1, int=2, float=3, string=4, raw/msgpack/protobuf=5, boolean[]=16, double[]=17, int[]=18, float[]=19, string[]=20
- **Timestamp sync:** Topic ID -1, client sends local time, server responds with server time + echoed client time
- **Subscribe triggers announces:** Server sends announce ONLY after client subscribes with matching patterns. `prefix=true` with empty string `""` matches all non-meta topics.
- **Full spec:** https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
