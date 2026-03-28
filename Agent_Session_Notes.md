# Agent session notes

- Keep this file short and handoff-focused.
- For remaining work and deferred items: see `docs/roadmap.md`.
- Move durable milestone history to `docs/project_history.md`.
- Use `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md` as the canonical long-term Native Link rollout note.

## Workflow

- `apply_patch` expects workspace-relative paths with forward slashes.
- **Use CRLF line endings** for all source files (`.cpp`, `.h`, `.cmake`, `.ps1`, `.py`, `.md`, `.gitignore`, `.json`). Both repos standardized CRLF as of the NT4 transport merge.
- **Do NOT CRLF-normalize `.rc` or `.aps` files** — they are UTF-16 LE encoded. `.gitattributes` marks them `binary`. See "Key invariants" for the full story.
- Read nearby `Ian:` comments before editing and add new ones where session, ownership, carrier, protocol, or runtime-mode lessons would be easy to lose.
- Process detection: use `Get-Process -Name <name> -ErrorAction SilentlyContinue` (NOT `tasklist | findstr` which breaks in Git Bash due to flag mangling).
- Killing processes: use PowerShell `Stop-Process` (NOT `taskkill` through Git Bash).

## Build

```bash
# Configure (uses existing build-vcpkg directory)
cmake -G "Visual Studio 17 2022" -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE="D:/Git/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows

# Build
cmake --build build-vcpkg --target DriverStation --config Release
cmake --build build-vcpkg --target DriverStation_TransportSmoke --config Release

# Run DriverStation in NetworkTables V4 mode (serves Shuffleboard, Glass, any NT4 dashboard)
Start-Process -FilePath 'D:\code\Robot_Simulation\build-vcpkg\bin\Release\DriverStation.exe' -ArgumentList 'nt4' -WorkingDirectory 'D:\code\Robot_Simulation\build-vcpkg\bin\Release'

# Run smoke test
build-vcpkg\bin\Release\DriverStation_TransportSmoke.exe --mode nt4
```

Note: The ixwebsocket overlay port only matters at `vcpkg install` time, not at CMake configure time. The patched library is already in vcpkg's `installed/` directory.

**Windows Firewall:** The first time DriverStation.exe listens on a new port, Windows will prompt to allow it. This is a one-time approval per port.

## Key invariants (do not break)

- `Core::PublishInternal` auto-register only applies when `allowServerOnly=true`. Client writes on unknown topics must still be rejected.
- `RegisterDefaultTopics` must not grow to list every TeleAutonV2 key — the auto-register path handles those.
- `TeleAutonV2` publishes ~26 keys per loop. `Heading` (line 324) and `Travel_Heading` (line 307) are unconditional.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.
- **`HasDirectTransport()` and `UsesNetworkTablesTransport()` in SmartDashboard.cpp must include every mode that uses a `DirectPublishSink`/`DirectQuerySource`.** Missing a mode causes `PutNumber`/etc. to fall through to `NetworkTable::GetTable()` which starts the legacy NT2 server on port 1735 alongside the intended backend. This was the root cause of the Shuffleboard port bug (port 1735 instead of 5810).
- **`.rc` files must never be CRLF-normalized by git.** They are UTF-16 LE (BOM `FF FE`). Git's CRLF normalization treats them as single-byte text, mangling `0D 00 0A 00` to `0D 0A 00 0D 0A 00`, destroying UTF-16 alignment. `rc.exe` then silently produces an empty `.res` (32 bytes), and `CreateDialogW` returns NULL with `ERROR_RESOURCE_TYPE_NOT_FOUND` (1813). `.gitattributes` now marks `*.rc` and `*.aps` as `binary`.

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
| NetworkTables V4 | `eNetworkTablesV4` | NT4 WebSocket | 5810 | Stable (serves Shuffleboard, Glass, any NT4 dashboard) |
| Native Link | `eNativeLink` | Custom TCP/SHM | 5810 | Stable |

### NT4 server

`NT4Server.h/.cpp` (~1200 lines) — full NT4 WebSocket server: subscription-driven architecture, custom MsgPack encoder/decoder, JSON announce, retained-value cache, per-client subscription tracking, client value write-back with pubuid->topicId resolution and re-broadcast to all subscribers (including sender).

### DriverStation automation

- `dsctl.ps1` — sends `WM_COMMAND` to DriverStation dialog. Commands: `start`, `stop`, `auton`, `tele`, `test`, `auton-enable`.
- Hotkeys: F5=Legacy, F6=Direct, F7=NetworkTables V4, F8=NativeLink.
- Control IDs: `IDC_Start`=1006, `IDC_Stop`=1005, `IDC_Auton`=1003, `IDC_Tele`=1002, `IDC_Test`=1004.

### Video source modes

| Mode | Enum | Description |
|---|---|---|
| Off | `VideoSourceMode::eOff` | No camera stream; MJPEG server torn down |
| Camera | `VideoSourceMode::eCamera` | USB webcam via VFW (Video for Windows) |
| Synthetic Radar | `VideoSourceMode::eSyntheticRadar` | Existing SimCameraSource radar sweep (default) |
| The Grid | `VideoSourceMode::eVirtualField` | Tron-style first-person virtual field camera (TronGridSource) |

Video source is selected via "Video" combo in DriverStation dialog. Persisted to `[Video] Source=<int>` in DriverStation.ini.

## Cross-repo sync

- This repo and `D:\code\SmartDashboard` share the Native Link contract, carrier implementations, and plugin boundary.
- When either repo's session notes change, check the other side for consistency.

## Remaining work

See `docs/roadmap.md` for all remaining work and deferred items.
