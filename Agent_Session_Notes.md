# Agent session notes

- Keep this file short and handoff-focused.
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

`NT4Server.h/.cpp` (~1200 lines) — full NT4 WebSocket server: subscription-driven architecture, custom MsgPack encoder/decoder, JSON announce, retained-value cache, per-client subscription tracking, client value write-back with pubuid→topicId resolution and re-broadcast to all subscribers (including sender).

### DriverStation automation

- `dsctl.ps1` — sends `WM_COMMAND` to DriverStation dialog. Commands: `start`, `stop`, `auton`, `tele`, `test`, `auton-enable`.
- Hotkeys: F5=Legacy, F6=Direct, F7=NetworkTables V4, F8=NativeLink.
- Control IDs: `IDC_Start`=1006, `IDC_Stop`=1005, `IDC_Auton`=1003, `IDC_Tele`=1002, `IDC_Test`=1004.

## Cross-repo sync

- This repo and `D:\code\SmartDashboard` share the Native Link contract, carrier implementations, and plugin boundary.
- When either repo's session notes change, check the other side for consistency.

## Completed milestones

| Feature | Branch | Status |
|---|---|---|
| Native Link TCP carrier | `feature/native-link-tcpip-carrier` | Merged to master |
| NT4 transport (originally "Shuffleboard") | `feature/shuffleboard-transport` | Merged to master |
| Glass verification + Shuffleboard→NT4 rename | `feature/glass-transport` | Merged to master |

## In progress: Camera MJPEG server (`feature/camera-widget`)

MJPEG camera stream server for SmartDashboard's camera viewer dock.  This is
Phase 3 of the camera widget feature (Phase 1 = SmartDashboard MJPEG client,
Phase 2 = targeting reticle overlay, Phase 3 = simulator MJPEG server).

### Architecture

- **MjpegServer** (`MjpegServer.h/.cpp`): MJPEG-over-HTTP streaming server on
  port 1181.  Subclasses `ix::SocketServer` directly (NOT `ix::HttpServer`, which
  is request-response only and can't do streaming).  Each client connection gets its
  own thread.  `handleConnection()` parses the HTTP request, sends MJPEG response
  headers (`multipart/x-mixed-replace`), then loops pushing frames from a shared
  buffer via condition variable.

  Ian: LESSON LEARNED — ix::HttpServer is strictly request-response.  The
  OnConnectionCallback must return a complete HttpResponsePtr.  MJPEG requires an
  infinite streaming response, so we bypass HttpServer and subclass SocketServer.

- **SimCameraSource** (`SimCameraSource.h/.cpp`): Frame generator running on a
  dedicated worker thread.  Renders a synthetic test pattern (radar sweep, crosshair,
  frame counter, "SIM CAM" label) into an RGB buffer, JPEG-encodes via
  `stb_image_write`, and pushes to MjpegServer.  320x240 @ 15fps default.

- **Integration** in `NT4Backend::Initialize()` / `Shutdown()` in Transport.cpp:
  Creates MjpegServer + SimCameraSource, publishes CameraPublisher discovery keys
  via NT4 (`/CameraPublisher/SimCamera/streams`, `/source`, `/connected`).

### New files

| File | Purpose |
|---|---|
| `Source/Application/DriverStation/DriverStation/MjpegServer.h` | MJPEG HTTP server header |
| `Source/Application/DriverStation/DriverStation/MjpegServer.cpp` | MJPEG HTTP server implementation |
| `Source/Application/DriverStation/DriverStation/SimCameraSource.h` | Frame source header |
| `Source/Application/DriverStation/DriverStation/SimCameraSource.cpp` | Frame generation + JPEG encoding |
| `Source/ThirdParty/stb/stb_image_write.h` | Single-header JPEG encoder (v1.16) |

### Modified files

| File | Change |
|---|---|
| `Source/Application/DriverStation/DriverStation/Transport.cpp` | Added `#include` for MjpegServer/SimCameraSource; NT4Backend now owns MjpegServer + SimCameraSource; StartCameraStream()/StopCameraStream() methods; CameraPublisher NT4 key publishing |
| `CMakeLists.txt` | Added MjpegServer.cpp + SimCameraSource.cpp to DriverStation and TransportSmoke targets; added stb include path |

### Key protocol details

- MJPEG boundary: `mjpegstream` (sent without leading dashes in Content-Type header)
- CameraPublisher NT4 key: `/CameraPublisher/SimCamera/streams` = `["mjpg:http://127.0.0.1:1181/?action=stream"]`
- SmartDashboard's `MjpegStreamSource` strips the `mjpg:` prefix and connects to the URL
- SmartDashboard's `CameraPublisherDiscovery` watches `/CameraPublisher/*/streams` for auto-discovery

### Status

- [x] stb_image_write.h downloaded
- [x] MjpegServer.h/.cpp created
- [x] SimCameraSource.h/.cpp created
- [x] Transport.cpp integration (NT4Backend owns MjpegServer + SimCameraSource)
- [x] CMakeLists.txt updated
- [x] Build verified (both DriverStation + TransportSmoke clean, 17 unit tests pass)
- [x] MJPEG stream verified via curl (correct multipart headers, valid JPEG frames, ~15fps)
- [x] SmartDashboard auto-connect wiring complete — discovery→combo→auto-connect pipeline verified end-to-end (SmartDashboard repo, `feature/camera-widget` branch, 28 new tests)

## Glass support (complete — no separate plugin needed)

Glass uses the same NT4 protocol as Shuffleboard — same WebSocket transport, same MsgPack binary frames, same JSON control messages, same port 5810. It connects to the existing NT4 server with zero changes (beyond the RTT ping fix that was already committed). No separate Glass backend or SmartDashboard Glass plugin is needed.

The transport mode was renamed from Shuffleboard-specific naming to generic NT4 naming: `eShuffleboard` → `eNetworkTablesV4`, `ShuffleboardBackend` → `NT4Backend`, CLI `--mode nt4` (with `shuffle`/`shuffleboard` kept as backward-compatible aliases).

### Glass installation

Glass 2026.2.2 is installed at `D:\code\Glass` (same portable-directory pattern as Shuffleboard):

| File | Purpose |
|---|---|
| `glass.exe` | Glass application (native C++ binary — no JRE needed) |
| `glass.pdb` | Debug symbols |
| `run_glass.bat` | Launch Glass (default config from `%APPDATA%`) |
| `run_glass_local.bat` | Launch Glass pre-configured for localhost:5810 |
| `config_local\glass.json` | Pre-configured: NT4 client mode, `localhost`, port 5810 |

**How local config works:** Glass accepts one CLI argument — a save directory for its JSON config files. `run_glass_local.bat` passes `config_local\` which contains a `glass.json` pre-seeded with `mode="Client (NT4)"`, `serverTeam="localhost"`, port 5810, dsClient=false. Glass reads this on startup and auto-connects. The default `run_glass.bat` uses `%APPDATA%` and requires manual configuration via the GUI.

**Source:** Downloaded from WPILib Maven — `https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/tools/Glass/2026.2.2/Glass-2026.2.2-windowsx86-64.zip`

### Plan

1. ~~Pull Glass into `D:\code\Glass`~~ — Done
2. ~~Make Robot_Simulation work with Glass~~ — Done: zero server changes needed since Glass speaks the same NT4 protocol on port 5810. RTT ping fix was required.
3. ~~SmartDashboard Glass plugin~~ — Not needed. Glass connects to the existing NT4Transport plugin with zero changes.

### Checklist: adding a new transport mode

See the `Ian:` comment on `Transport.h` for the full file list. Summary:

1. Add `ConnectionMode` enum value in `Transport.h`
2. Create `IConnectionBackend` subclass in `Transport.cpp` (like `NT4Backend`)
3. Add factory case in `EnsureBackend()` in `Transport.cpp`
4. Update `UsesLegacyTransportPath()` — return false for NT4-style transports
5. **Update `HasDirectTransport()` and `UsesNetworkTablesTransport()` in `SmartDashboard.cpp`** — any mode that uses `DirectPublishSink`/`DirectQuerySource` must be listed, or `PutNumber`/etc. will also start the legacy NT2 server on port 1735
6. Update `IsChooserEnabledForCurrentConnection()` in `AI_Input_Example.cpp`
7. Add hotkey and menu entry in `DriverStation.cpp`
8. If the NT4 server port differs, make it configurable or support multi-port

### NT4 protocol lessons (from Shuffleboard/Glass integration)

**NT4 protocol gotchas (all fixed, but will bite again if forgotten):**
- NT4 is subscription-driven. The server must NOT send `announce` until the client sends `subscribe`. Silent failure otherwise.
- **RTT ping response is a hard gate.** The ntcore client sends `[-1, 0, type=2(Integer), clientTimestamp]` on connect. The server MUST respond with `[-1, serverTimestamp, type=2(Integer), clientTimestamp]`. If the type code is wrong (e.g. 1/Double instead of 2/Integer) or the client's timestamp isn't echoed back, the client never sets `m_haveTimeOffset` and `SendOutgoing()` blocks ALL outgoing messages (subscribe, publish, etc.) forever. This was the root cause of Shuffleboard/Glass connecting but appearing silent.
- Client binary frames use **pubuid** (client-chosen), not server-assigned topicId. Server needs per-client `pubuid → topicId` map.
- Echo values back to sender — the server is the single source of truth.
- MsgPack frames may contain multiple concatenated messages per WebSocket frame.
- IXWebSocket's `Sec-WebSocket-Protocol` handling needed a patch (overlay port) to echo exactly ONE selected protocol per RFC 6455. Glass and Shuffleboard use the same subprotocol (`v4.1.networktables.first.wpi.edu`).

**Simulator-side gotchas:**
- `NT4Backend` implements both `SmartDashboardDirectPublishSink` (server→client) AND `SmartDashboardDirectQuerySource` (client→server read-back). Both are needed for the simulator to read values written by dashboard clients.
- The query source normalizes flat keys (e.g. `TestMove`) to NT4 paths (`/SmartDashboard/TestMove`).
- `IsChooserEnabledForCurrentConnection()` must include the new mode or the auton AI falls back to the numeric `AutonTest` key and ignores the chooser.
- Headless crash: `TeleAuton_V2::init()` creates an OSG 3D viewer which crashes in headless sessions. Smoke test skips this for NT4 mode.
- WSAStartup: `ix::initNetSystem()` must be called before any IXWebSocket server operations.

**Chooser protocol:**
Base key `Test/Auton_Selection/AutoChooser`, all prefixed with `/SmartDashboard/`:
- Server publishes: `.type`="String Chooser", `options`=[array], `default`, `active`
- Client writes back: `selected` = user's choice
- Read-back priority: `selected` → `active` → `default`, with 20-retry × 10ms loop
- See `AutonChooser.h` for the `Ian:` comment with full protocol description.

**Port situation (no conflict):**
- Shuffleboard NT4 client: port 5810
- Glass NT4 client: port 5810 (same as Shuffleboard — only one NT4 dashboard connects at a time)
- Legacy SmartDashboard (NT2 TCP): port 1735

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

## Deferred work

- Expand smoke test published keys from ~6 + chooser to full TeleAutonV2 (~49 keys)
- Debug builds: manual carrier picker in DriverStation dialog for SHM vs TCP comparisons

## In progress: Video Source selector (`feature/camera-widget`)

Video Source dropdown in the DriverStation dialog that lets the user switch between
video sources feeding the MJPEG server on port 1181.

### Modes

| Mode | Enum | Description |
|---|---|---|
| Off | `VideoSourceMode::eOff` | No camera stream; MJPEG server torn down |
| Camera | `VideoSourceMode::eCamera` | USB webcam via VFW (Video for Windows) |
| Synthetic Radar | `VideoSourceMode::eSyntheticRadar` | Existing SimCameraSource radar sweep (default) |
| The Grid | `VideoSourceMode::eVirtualField` | Tron-style first-person virtual field camera (TronGridSource) |

### Architecture

- **WebCameraSource** (`WebCameraSource.h/.cpp`): VFW-based USB webcam capture.
  Creates a hidden capture window on a worker thread with its own message pump.
  Uses `capGrabFrameNoStop` in a timed polling loop to request frames at the
  target framerate — each grab triggers the `capSetCallbackOnFrame` callback
  synchronously, which converts the frame to top-down RGB (from YUY2 or
  bottom-up BGR), JPEG-encodes via `stbi_write_jpg_to_func`, pushes to
  `MjpegServer::PushFrame()`.

  Ian: VFW capture windows MUST be created on a thread with a message pump.
  The worker thread runs the pump; the main thread signals shutdown via atomic flag.

  Ian: LESSON LEARNED — capPreview(TRUE) + capSetCallbackOnFrame relies on the
  window receiving WM_PAINT messages, which never arrive for hidden windows.
  capGrabFrameNoStop bypasses this — it synchronously triggers the frame callback
  regardless of window visibility.

  Ian: CRITICAL LESSON — Most USB cameras on Windows deliver YUY2 (packed YCbCr
  4:2:2) through VFW, NOT BI_RGB.  capSetVideoFormat to request RGB24 often fails
  because the VFW driver doesn't support format conversion.  The frame callback
  must handle YUY2→RGB conversion using BT.601 color space coefficients
  (fixed-point integer arithmetic for speed).  YUY2 format: every 4 bytes encode
  2 pixels as [Y0, U, Y1, V], where each pixel pair shares U,V chrominance.

  Ian: No-camera fallback — WebCameraSource::WaitForStartup() lets the caller
  block until capDriverConnect succeeds or fails.  If no camera is found,
  NT4Backend::SetVideoSource() tears down the MJPEG server and falls back to Off.
  ApplyVideoSource() in DriverStation.cpp reads back GetVideoSource() after
  SetVideoSource() to detect the fallback and update the UI combo + INI file.

- **NT4Backend refactoring** in Transport.cpp: Replaced monolithic
  `StartCameraStream()`/`StopCameraStream()` with `SetVideoSource()` override
  plus helper methods `StartMjpegServer()`, `StopMjpegServer()`, `StopCurrentSource()`,
  `PublishCameraConnected()`, `PublishCameraDisconnected()`. The MJPEG server stays
  alive when switching between Camera and Radar sources; only torn down for Off.

- **DriverStation.cpp UI**: Always-visible "Video" label + combo box
  (control IDs 1012/1013). Persisted to `[Video] Source=<int>` in DriverStation.ini.
  Applied after connection mode on startup via `ApplyVideoSource(LoadPersistedVideoSource())`.

  Layout: Video row is positioned dynamically below the Connection combo by
  querying its actual pixel position at runtime (`GetDlgItem` + `GetWindowRect` +
  `ScreenToClient`). In Debug builds, the NativeLink Carrier combo is on the
  same row as the Connection combo (to its right, 10px gap), keeping it clear
  of the Video dropdown.

- **`stb_image_write`**: `STB_IMAGE_WRITE_IMPLEMENTATION` defined exactly once in
  `SimCameraSource.cpp`. `WebCameraSource.cpp` and `TronGridSource.cpp` only include
  the header for declarations.

- **TronGridSource** (`TronGridSource.h/.cpp`): "The Grid" — first-person Tron-style
  virtual field camera.  Pure software 3D renderer using the same pixel-buffer +
  Bresenham drawing pattern as SimCameraSource.  Zero OSG/OpenGL dependency — avoids
  crash risk from GPU context creation on worker threads.

  Renders a 54×27 ft FRC field as a glowing cyan wireframe grid on black background,
  viewed from the robot's perspective (camera at robot position, 2 ft height, looking
  in heading direction).  Features:
  - 3D perspective projection via software pinhole camera model (90° HFOV)
  - 1-foot-spaced floor grid with depth fog (cyan fades to dark blue with distance)
  - Field boundary walls as bright cyan-white wireframe rectangles (1.5 ft tall)
  - Squid Games FIRST emblem at field center: red circle, white triangle, blue square
  - Background clouds: distant dim wireframe rectangles floating in the void (Flynn
    falling into the digital world aesthetic)
  - MCP tower: tall red wireframe monolith far to the north with glowing band stripes
  - CRT scanline effect for retro aesthetic (no HUD text — pure black background)
  - Off-field arrow: orange arrow pointing back toward field center when robot
    wanders outside field bounds
  - Near-plane line clipping to prevent projection artifacts from behind-camera geometry
  - Safety limits on Bresenham line length to prevent frame lockup from bad projections

  Ian: LESSON LEARNED — OSG offscreen FBO rendering on a worker thread crashes on
  many Windows GPU drivers.  The pure-software approach is immune — only needs CPU
  and std::vector<uint8_t>.  At 320×240 @ 15fps the CPU cost is negligible.

  Ian: Robot position is read each frame via a callback that queries the NT4 retained
  value cache for `/SmartDashboard/Drive/X_ft`, `Y_ft`, and `Heading`.  The callback
  is set by NT4Backend when creating the source, capturing `this` pointer — safe because
  StopCurrentSource() is always called before backend shutdown.

  Ian: Field coordinate mapping — the robot starts at (0,0) in the simulator which is
  mapped to field position (27, 1) — center of the near alliance wall, 1 ft from the
  wall.  This places the robot naturally on the field.

  Ian: HEADING INVERSION — The published Drive/Heading uses atan2(x,y) which gives
  CW-positive from +Y.  But WorldToCamera() expects CCW-positive (standard math
  convention).  The heading must be negated: `m_headingDeg = -heading;`.  This also
  fixed the grid lines and walls rendering in wrong positions — they all flow through
  the same WorldToCamera() rotation matrix.

  Ian: CENTERED COORDINATE SYSTEM — Robot (0,0) from the simulator maps directly to
  field center (0,0).  The field spans X ∈ [-27, +27], Y ∈ [-13.5, +13.5].  No offset
  mapping needed.  The old system offset to (27, 1) which was wrong — when the robot
  reported negative Y it triggered the off-field arrow prematurely.

  Ian: SCREEN-SPACE CLIPPING — Lines outside the camera FOV were still being drawn
  because the old code used a generous 200px margin check.  Replaced with proper
  Cohen-Sutherland 2D line clipping against the screen rectangle.  This clips line
  segments to exactly the visible area before Bresenham, eliminating wasted pixel
  traversals and incorrect rendering of off-FOV geometry.

### New files

| File | Purpose |
|---|---|
| `Source/Application/DriverStation/DriverStation/WebCameraSource.h` | VFW webcam capture header |
| `Source/Application/DriverStation/DriverStation/WebCameraSource.cpp` | VFW webcam capture implementation |
| `Source/Application/DriverStation/DriverStation/TronGridSource.h` | "The Grid" Tron-style first-person virtual field camera header |
| `Source/Application/DriverStation/DriverStation/TronGridSource.cpp` | "The Grid" implementation: software 3D renderer, perspective grid, Squid Games emblem, background clouds, MCP tower |

### Modified files

| File | Change |
|---|---|
| `Transport.h` | Added `VideoSourceMode` enum, `GetVideoSourceModeName()`, virtual `SetVideoSource()`/`GetVideoSource()` on `IConnectionBackend`, forwarding on `DashboardTransportRouter`; updated eVirtualField comment to "The Grid" |
| `Transport.cpp` | Refactored NT4Backend camera management: `SetVideoSource()` override with helper methods; added `#include "TronGridSource.h"`; `eVirtualField` case creates TronGridSource with position callback reading NT4 cache; `StopCurrentSource()` handles TronGridSource cleanup; display name "The Grid"; router forwarding implementations |
| `Robot_Tester.h` | Added `SetVideoSource()`/`GetVideoSource()` declarations |
| `Robot_Tester.cpp` | Added `SetVideoSource()`/`GetVideoSource()` implementations forwarding through router |
| `DriverStation.cpp` | Added video source UI controls, persistence, `ApplyVideoSource()`, WM_INITDIALOG/WM_COMMAND wiring |
| `CMakeLists.txt` | Added `WebCameraSource.cpp` and `TronGridSource.cpp` to DriverStation + TransportSmoke targets; added `vfw32` to link libs |

### Status

- [x] WebCameraSource.h/.cpp created
- [x] Transport.h — VideoSourceMode enum, virtual methods
- [x] Transport.cpp — NT4Backend refactored with SetVideoSource
- [x] Robot_Tester.h/.cpp — forwarding methods
- [x] DriverStation.cpp — UI controls, persistence, event handling, startup wiring
- [x] DriverStation.cpp — Layout fix: Video row below Connection, Carrier beside Connection (Debug)
- [x] CMakeLists.txt — WebCameraSource.cpp + vfw32
- [x] Build verified (Debug + Release DriverStation, Release TransportSmoke — all clean)
- [x] WebCameraSource rewritten to use capGrabFrameNoStop polling (fixes hidden-window issue)
- [x] CameraCapture prototype: discovered camera delivers YUY2, not BI_RGB
- [x] YUY2→RGB conversion (BT.601, fixed-point) added to prototype and verified — valid JPEG output
- [x] YUY2 support ported to WebCameraSource.cpp — handles both YUY2 and BI_RGB (24/32 bpp)
- [x] DriverStation rebuilt (Debug + Release) with YUY2 support — clean
- [x] SmartDashboard: auto-connect removed, URL on its own row, diagnostic logging added
- [x] End-to-end test: Synthetic Radar verified (320x240, ~7KB frames)
- [x] End-to-end test: Off mode verified (port 1181 connection refused)
- [x] End-to-end test: Camera mode verified (640x480, ~22KB real webcam frames)
- [x] No-camera fallback: WaitForStartup() + SetVideoSource falls back to Off, UI combo + INI updated
- [x] TronGridSource.h/.cpp created — "The Grid" first-person Tron-style virtual field camera
- [x] Transport.cpp — eVirtualField wired to TronGridSource with NT4 position callback
- [x] Transport.h — eVirtualField comment updated to "The Grid"
- [x] GetVideoSourceModeName returns "The Grid" for eVirtualField
- [x] CMakeLists.txt — TronGridSource.cpp added to both targets
- [x] Build verified (Debug + Release DriverStation, Release TransportSmoke — all clean, 17 unit tests pass)
- [x] Heading inversion fixed: `m_headingDeg = -heading` — fixes rotation direction AND grid/wall rendering
- [x] HUD text removed: "THE GRID" title, position, heading, FPS, frame counter all stripped; CRT scanlines kept
- [x] Background clouds added: 12 distant dim wireframe rectangles floating in the void with gentle drift
- [x] MCP tower added: tall red wireframe monolith at Y=40 with 5 glowing band stripes
- [x] Build verified after all fixes (Debug + Release DriverStation, Release TransportSmoke — all clean)
- [x] Coordinate system fixed: field centered at origin, X ∈ [-27,+27], Y ∈ [-13.5,+13.5], no offset mapping
- [x] Cohen-Sutherland screen-space line clipping added — lines outside FOV properly culled
- [x] Clouds/MCP repositioned for centered coords and brought within fog range (~40 ft from center)
- [x] Build verified after centering fixes (Debug + Release DriverStation, Release TransportSmoke — all clean)

### SmartDashboard changes (E:\code\SmartDashboard)

| File | Change |
|---|---|
| `SmartDashboard/src/widgets/camera_viewer_dock.h` | Updated doc: auto-connect removed, auto-reconnect preserved |
| `SmartDashboard/src/widgets/camera_viewer_dock.cpp` | Removed auto-connect (TryAutoConnect is no-op); URL edit moved to own row with label; visibility-changed handler no longer auto-connects |
| `SmartDashboard/src/camera/mjpeg_stream_source.cpp` | Added qDebug logging: connect URL, first readyRead Content-Type, first frame decode, stream errors |
| `SmartDashboard/tests/camera_viewer_dock_tests.cpp` | Updated 3 auto-connect tests to verify auto-connect does NOT fire; all 168 runnable tests pass |
