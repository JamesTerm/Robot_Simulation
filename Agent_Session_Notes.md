# Agent session notes

- Keep this file short and handoff-focused.
- Move durable milestone history to `docs/project_history.md`.
- Use `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md` as the canonical long-term Native Link rollout note.

## Workflow

- `apply_patch` expects workspace-relative paths with forward slashes.
- **Use CRLF line endings** for all source files (`.cpp`, `.h`, `.cmake`, `.ps1`, `.py`, `.md`, `.rc`, `.gitignore`, `.json`). Both repos standardized CRLF as of the Shuffleboard merge.
- Read nearby `Ian:` comments before editing and add new ones where session, ownership, carrier, protocol, or runtime-mode lessons would be easy to lose.
- Process detection: use `Get-Process -Name <name> -ErrorAction SilentlyContinue` (NOT `tasklist | findstr` which breaks in Git Bash due to flag mangling).
- Killing processes: use PowerShell `Stop-Process` (NOT `taskkill` through Git Bash).

## Build

```bash
# Configure (uses existing build-vcpkg directory)
cmake -G "Visual Studio 17 2022" -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE="D:/code/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows

# Build
cmake --build build-vcpkg --target DriverStation --config Release
cmake --build build-vcpkg --target DriverStation_TransportSmoke --config Release

# Run DriverStation in Shuffleboard mode
Start-Process -FilePath 'D:\code\Robot_Simulation\build-vcpkg\bin\Release\DriverStation.exe' -ArgumentList 'shuffle' -WorkingDirectory 'D:\code\Robot_Simulation\build-vcpkg\bin\Release'

# Run smoke test
build-vcpkg\bin\Release\DriverStation_TransportSmoke.exe --mode shuffle
```

Note: The ixwebsocket overlay port only matters at `vcpkg install` time, not at CMake configure time. The patched library is already in vcpkg's `installed/` directory.

**Windows Firewall:** The first time DriverStation.exe listens on a new port, Windows will prompt to allow it. This is a one-time approval per port.

## Key invariants (do not break)

- `Core::PublishInternal` auto-register only applies when `allowServerOnly=true`. Client writes on unknown topics must still be rejected.
- `RegisterDefaultTopics` must not grow to list every TeleAutonV2 key — the auto-register path handles those.
- `TeleAutonV2` publishes ~26 keys per loop. `Heading` (line 324) and `Travel_Heading` (line 307) are unconditional.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.

## Strategy

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.
- **Support, not imitate** — give teams enough compatibility to try SmartDashboard with each dashboard tool, then encourage Native Link for full support.

## Transport architecture

See `Transport.h` for the `Ian:` comment listing all files that must be updated when adding a new transport mode.

Current modes:
| Mode | Enum | Backend | Port | Status |
|---|---|---|---|---|
| Legacy SmartDashboard | `eLegacySmartDashboard` | NT v2 TCP | 1735 | Stable |
| Direct | `eDirectConnect` | Shared memory | N/A | Stable |
| Shuffleboard | `eShuffleboard` | NT4 WebSocket | 5810 | Stable, merged |
| Native Link | `eNativeLink` | Custom TCP/SHM | 5810 | Stable |

### NT4 server

`NT4Server.h/.cpp` (~1200 lines) — full NT4 WebSocket server: subscription-driven architecture, custom MsgPack encoder/decoder, JSON announce, retained-value cache, per-client subscription tracking, client value write-back with pubuid→topicId resolution and re-broadcast to all subscribers (including sender).

### DriverStation automation

- `dsctl.ps1` — sends `WM_COMMAND` to DriverStation dialog. Commands: `start`, `stop`, `auton`, `tele`, `test`, `auton-enable`.
- Hotkeys: F5=Legacy, F6=Direct, F7=Shuffleboard, F8=NativeLink.
- Control IDs: `IDC_Start`=1006, `IDC_Stop`=1005, `IDC_Auton`=1003, `IDC_Tele`=1002, `IDC_Test`=1004.

## Cross-repo sync

- This repo and `D:\code\SmartDashboard` share the Native Link contract, carrier implementations, and plugin boundary.
- When either repo's session notes change, check the other side for consistency.

## Completed milestones

| Feature | Branch | Status |
|---|---|---|
| Native Link TCP carrier | `feature/native-link-tcpip-carrier` | Merged to master |
| Shuffleboard NT4 transport | `feature/shuffleboard-transport` | Merged to master |

## Preparing for Glass transport

Glass is the next dashboard integration target. It uses the same NT4 protocol as Shuffleboard. The process:

1. **Pull Glass source** into `D:\code\Glass`
2. **Make Robot_Simulation work with Glass** (may need NT4 server adjustments)
3. **Create SmartDashboard Glass plugin** under `plugins/GlassTransport/`

### Checklist: adding a new transport mode (e.g. Glass)

See the `Ian:` comment on `Transport.h` for the full file list. Summary:

1. Add `ConnectionMode` enum value in `Transport.h`
2. Create `IConnectionBackend` subclass in `Transport.cpp` (like `ShuffleboardBackend`)
3. Add factory case in `EnsureBackend()` in `Transport.cpp`
4. Update `UsesLegacyTransportPath()` — return false for NT4-style transports
5. Update `IsChooserEnabledForCurrentConnection()` in `AI_Input_Example.cpp`
6. Add hotkey and menu entry in `DriverStation.cpp`
7. If the NT4 server port differs, make it configurable or support multi-port

### Lessons from Shuffleboard to apply to Glass

**NT4 protocol gotchas (all fixed, but will bite again if forgotten):**
- NT4 is subscription-driven. The server must NOT send `announce` until the client sends `subscribe`. Silent failure otherwise.
- Client binary frames use **pubuid** (client-chosen), not server-assigned topicId. Server needs per-client `pubuid → topicId` map.
- Echo values back to sender — the server is the single source of truth.
- MsgPack frames may contain multiple concatenated messages per WebSocket frame.
- IXWebSocket's `Sec-WebSocket-Protocol` handling needed a patch (overlay port) to echo exactly ONE selected protocol per RFC 6455. Glass may use a different subprotocol string.

**Simulator-side gotchas:**
- `ShuffleboardBackend` implements both `SmartDashboardDirectPublishSink` (server→client) AND `SmartDashboardDirectQuerySource` (client→server read-back). Both are needed for the simulator to read values written by dashboard clients.
- The query source normalizes flat keys (e.g. `TestMove`) to NT4 paths (`/SmartDashboard/TestMove`).
- `IsChooserEnabledForCurrentConnection()` must include the new mode or the auton AI falls back to the numeric `AutonTest` key and ignores the chooser.
- Headless crash: `TeleAuton_V2::init()` creates an OSG 3D viewer which crashes in headless sessions. Smoke test skips this for shuffleboard mode.
- WSAStartup: `ix::initNetSystem()` must be called before any IXWebSocket server operations.

**Chooser protocol:**
Base key `Test/Auton_Selection/AutoChooser`, all prefixed with `/SmartDashboard/`:
- Server publishes: `.type`="String Chooser", `options`=[array], `default`, `active`
- Client writes back: `selected` = user's choice
- Read-back priority: `selected` → `active` → `default`, with 20-retry × 10ms loop
- See `AutonChooser.h` for the `Ian:` comment with full protocol description.

**Port differences:**
- Shuffleboard default: port 5810
- Glass default: port 1735 (same as legacy NT — potential conflict!)
- Legacy SmartDashboard: port 1735

**No value persistence by design:**
TestMove and chooser selection start at 0/"Do Nothing" each launch. Values must be published by a client after the NT4 server is running.

### NT4 protocol quick reference

- **Transport:** WebSocket, resource path `/nt/<clientname>`
- **Subprotocols:** `v4.1.networktables.first.wpi.edu` (preferred), `networktables.first.wpi.edu` (v4.0)
- **Control messages:** JSON text frames — array of `{method, params}` objects
- **Server→client:** `announce`, `unannounce`, `properties`
- **Client→server:** `subscribe`, `unsubscribe`, `publish`, `unpublish`, `setproperties`
- **Value updates:** MsgPack binary — `[topicID, timestamp_us, dataType, value]`
- **Data types:** boolean=0, double=1, int=2, float=3, string=4, raw=5, boolean[]=16, double[]=17, int[]=18, float[]=19, string[]=20
- **Full spec:** https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc

## What to publish (current widget types)

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

## Deferred work (not blocking Glass)

- Expand smoke test published keys from ~6 + chooser to full TeleAutonV2 (~49 keys)
- Debug builds: manual carrier picker in DriverStation dialog for SHM vs TCP comparisons
