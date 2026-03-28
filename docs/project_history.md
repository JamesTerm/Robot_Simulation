# Project history

## 2026-03-28 - Camera Phase 4 closed, manual e2e verified

### Phase 4 — Backup camera guide lines (closed, superseded)

The original Phase 4 concept — OSG-side overlays baked into the MJPEG stream (Honda-style curved path lines driven by velocity/angular velocity) — was dropped. The design decision:

1. **Overlays belong on the dashboard side (QPainter), not baked into the stream.** SmartDashboard's `CameraDisplayWidget` already draws a targeting reticle via QPainter. Any future guide-line or annotation overlay should follow the same pattern. This keeps the MJPEG server simple, the stream reusable by any client, and aligns with how FRC teams handle camera overlays today.
2. **"The Grid" (TronGridSource)** already provides first-person spatial awareness driven by robot position/heading, which serves the purpose backup camera guide lines were intended to address.

### Manual end-to-end test — verified

SmartDashboard camera viewer dock receives and displays MJPEG streams from Robot_Simulation across all video source modes (Synthetic Radar, The Grid, USB Camera). Auto-discovery via `/CameraPublisher/SimCamera/streams` works end-to-end over NT4.

### H.264 — dropped (cross-repo decision)

Research concluded FRC teams are not using H.264 for dashboard camera streams. FMS bandwidth standards are adequate for MJPEG. No implementation planned.

---

## 2026-03-28 - Video Source selector complete (`feature/camera-widget`)

Video Source dropdown in the DriverStation dialog that lets the user switch between video sources feeding the MJPEG server on port 1181.

### Modes

| Mode | Enum | Description |
|---|---|---|
| Off | `VideoSourceMode::eOff` | No camera stream; MJPEG server torn down |
| Camera | `VideoSourceMode::eCamera` | USB webcam via VFW (Video for Windows) |
| Synthetic Radar | `VideoSourceMode::eSyntheticRadar` | Existing SimCameraSource radar sweep (default) |
| The Grid | `VideoSourceMode::eVirtualField` | Tron-style first-person virtual field camera (TronGridSource) |

### Architecture

- **WebCameraSource** (`WebCameraSource.h/.cpp`): VFW-based USB webcam capture. Creates a hidden capture window on a worker thread with its own message pump. Uses `capGrabFrameNoStop` in a timed polling loop to request frames at the target framerate — each grab triggers the `capSetCallbackOnFrame` callback synchronously, which converts the frame to top-down RGB (from YUY2 or bottom-up BGR), JPEG-encodes via `stbi_write_jpg_to_func`, pushes to `MjpegServer::PushFrame()`.

  Ian: VFW capture windows MUST be created on a thread with a message pump. The worker thread runs the pump; the main thread signals shutdown via atomic flag.

  Ian: LESSON LEARNED — capPreview(TRUE) + capSetCallbackOnFrame relies on the window receiving WM_PAINT messages, which never arrive for hidden windows. capGrabFrameNoStop bypasses this — it synchronously triggers the frame callback regardless of window visibility.

  Ian: CRITICAL LESSON — Most USB cameras on Windows deliver YUY2 (packed YCbCr 4:2:2) through VFW, NOT BI_RGB. capSetVideoFormat to request RGB24 often fails because the VFW driver doesn't support format conversion. The frame callback must handle YUY2->RGB conversion using BT.601 color space coefficients (fixed-point integer arithmetic for speed). YUY2 format: every 4 bytes encode 2 pixels as [Y0, U, Y1, V], where each pixel pair shares U,V chrominance.

  Ian: No-camera fallback — WebCameraSource::WaitForStartup() lets the caller block until capDriverConnect succeeds or fails. If no camera is found, NT4Backend::SetVideoSource() tears down the MJPEG server and falls back to Off. ApplyVideoSource() in DriverStation.cpp reads back GetVideoSource() after SetVideoSource() to detect the fallback and update the UI combo + INI file.

- **TronGridSource** (`TronGridSource.h/.cpp`): "The Grid" — first-person Tron-style virtual field camera. Pure software 3D renderer using the same pixel-buffer + Bresenham drawing pattern as SimCameraSource. Zero OSG/OpenGL dependency — avoids crash risk from GPU context creation on worker threads.

  Renders a 54x27 ft FRC field as a glowing cyan wireframe grid on black background, viewed from the robot's perspective (camera at robot position, 2 ft height, looking in heading direction). Features:
  - 3D perspective projection via software pinhole camera model (90 deg HFOV)
  - 1-foot-spaced floor grid with depth fog (cyan fades to dark blue with distance)
  - Field boundary walls as bright cyan-white wireframe rectangles (1.5 ft tall)
  - Squid Games FIRST emblem at field center: red circle, white triangle, blue square
  - Background clouds: distant dim wireframe rectangles floating in the void (Flynn falling into the digital world aesthetic)
  - MCP tower: tall red wireframe monolith far to the north with glowing band stripes
  - CRT scanline effect for retro aesthetic
  - Off-field arrow: orange arrow pointing back toward field center when robot wanders outside field bounds
  - Near-plane line clipping to prevent projection artifacts from behind-camera geometry
  - Safety limits on Bresenham line length to prevent frame lockup from bad projections
  - Cohen-Sutherland screen-space line clipping — lines outside FOV properly culled

  Ian: LESSON LEARNED — OSG offscreen FBO rendering on a worker thread crashes on many Windows GPU drivers. The pure-software approach is immune — only needs CPU and std::vector<uint8_t>. At 320x240 @ 15fps the CPU cost is negligible.

  Ian: Robot position is read each frame via a callback that queries the NT4 retained value cache for `/SmartDashboard/Drive/X_ft`, `Y_ft`, and `Heading`. The callback is set by NT4Backend when creating the source, capturing `this` pointer — safe because StopCurrentSource() is always called before backend shutdown.

  Ian: Field coordinate mapping — CENTERED COORDINATE SYSTEM: Robot (0,0) from the simulator maps directly to field center (0,0). The field spans X in [-27, +27], Y in [-13.5, +13.5]. No offset mapping needed.

  Ian: HEADING INVERSION — The published Drive/Heading uses atan2(x,y) which gives CW-positive from +Y. But WorldToCamera() expects CCW-positive (standard math convention). The heading must be negated: `m_headingDeg = -heading;`.

- **NT4Backend refactoring** in Transport.cpp: Replaced monolithic `StartCameraStream()`/`StopCameraStream()` with `SetVideoSource()` override plus helper methods `StartMjpegServer()`, `StopMjpegServer()`, `StopCurrentSource()`, `PublishCameraConnected()`, `PublishCameraDisconnected()`. The MJPEG server stays alive when switching between Camera and Radar sources; only torn down for Off.

- **DriverStation.cpp UI**: Always-visible "Video" label + combo box (control IDs 1012/1013). Persisted to `[Video] Source=<int>` in DriverStation.ini. Applied after connection mode on startup via `ApplyVideoSource(LoadPersistedVideoSource())`. Layout: Video row is positioned dynamically below the Connection combo by querying its actual pixel position at runtime.

- **`stb_image_write`**: `STB_IMAGE_WRITE_IMPLEMENTATION` defined exactly once in `SimCameraSource.cpp`. `WebCameraSource.cpp` and `TronGridSource.cpp` only include the header for declarations.

### New files

| File | Purpose |
|---|---|
| `WebCameraSource.h/.cpp` | VFW webcam capture |
| `TronGridSource.h/.cpp` | "The Grid" Tron-style first-person virtual field camera |

### Modified files

| File | Change |
|---|---|
| `Transport.h` | Added `VideoSourceMode` enum, virtual `SetVideoSource()`/`GetVideoSource()` |
| `Transport.cpp` | Refactored NT4Backend camera management with SetVideoSource + helpers |
| `Robot_Tester.h/.cpp` | Added forwarding methods |
| `DriverStation.cpp` | Video source UI controls, persistence, event handling, startup wiring |
| `CMakeLists.txt` | Added sources + vfw32 link |

### SmartDashboard changes made during this work

| File | Change |
|---|---|
| `camera_viewer_dock.h` | Updated doc: auto-connect removed, auto-reconnect preserved |
| `camera_viewer_dock.cpp` | Removed auto-connect (TryAutoConnect is no-op); URL edit moved to own row |
| `mjpeg_stream_source.cpp` | Added qDebug logging for diagnostics |
| `camera_viewer_dock_tests.cpp` | Updated 3 auto-connect tests; all 168 runnable tests pass |

---

## 2026-03-27 - NT4 keys reorganized into logical groups

Reorganized all published NT4 keys from flat `/SmartDashboard/<key>` into hierarchical groups (`ddc90e1`):

| Group | Keys |
|---|---|
| `Drive/` | `Velocity`, `Rotation Velocity`, `X_ft`, `Y_ft`, `Heading`, `Travel_Heading` |
| `Swerve/` | `Wheel_{fl,fr,rl,rr}_Velocity`, `Wheel_{fl,fr,rl,rr}_Voltage`, `wheel_{fl,fr,rl,rr}_Encoder`, `Swivel_{fl,fr,rl,rr}_Voltage`, `swivel_{fl,fr,rl,rr}_Raw` |
| `Manipulator/` | (future expansion) |
| `Autonomous/` | `Timer`, `TestMove`, `AutonTest` |

This gives SmartDashboard's Run Browser dock meaningful tree structure instead of a flat list of ~49 keys. Key paths are now `/SmartDashboard/Drive/Velocity`, etc.

---

## 2026-03-27 - Graceful MJPEG shutdown fix

Fixed zombie process issue (`e7aeb27`): when the DriverStation was closed while the MJPEG server was running, the server's client threads could prevent clean process exit. Added `SignalStop()` + `connectionState->setTerminated()` to the shutdown path so all MJPEG client connections are cleanly torn down and the process exits without hanging.

---

## 2026-03-27 - Camera MJPEG server complete (`feature/camera-widget`)

MJPEG camera stream server for SmartDashboard's camera viewer dock. This is Phase 3 of the camera widget feature.

### Architecture

- **MjpegServer** (`MjpegServer.h/.cpp`): MJPEG-over-HTTP streaming server on port 1181. Subclasses `ix::SocketServer` directly (NOT `ix::HttpServer`, which is request-response only and can't do streaming). Each client connection gets its own thread. `handleConnection()` parses the HTTP request, sends MJPEG response headers (`multipart/x-mixed-replace`), then loops pushing frames from a shared buffer via condition variable.

  Ian: LESSON LEARNED — ix::HttpServer is strictly request-response. The OnConnectionCallback must return a complete HttpResponsePtr. MJPEG requires an infinite streaming response, so we bypass HttpServer and subclass SocketServer.

- **SimCameraSource** (`SimCameraSource.h/.cpp`): Frame generator running on a dedicated worker thread. Renders a synthetic test pattern (radar sweep, crosshair, frame counter, "SIM CAM" label) into an RGB buffer, JPEG-encodes via `stb_image_write`, and pushes to MjpegServer. 320x240 @ 15fps default.

- **Integration** in `NT4Backend::Initialize()` / `Shutdown()` in Transport.cpp: Creates MjpegServer + SimCameraSource, publishes CameraPublisher discovery keys via NT4 (`/CameraPublisher/SimCamera/streams`, `/source`, `/connected`).

### Key protocol details

- MJPEG boundary: `mjpegstream` (sent without leading dashes in Content-Type header)
- CameraPublisher NT4 key: `/CameraPublisher/SimCamera/streams` = `["mjpg:http://127.0.0.1:1181/?action=stream"]`
- SmartDashboard's `MjpegStreamSource` strips the `mjpg:` prefix and connects to the URL
- SmartDashboard's `CameraPublisherDiscovery` watches `/CameraPublisher/*/streams` for auto-discovery

### New files

| File | Purpose |
|---|---|
| `MjpegServer.h/.cpp` | MJPEG HTTP server |
| `SimCameraSource.h/.cpp` | Frame generation + JPEG encoding |
| `stb/stb_image_write.h` | Single-header JPEG encoder (v1.16) |

### Verification

- Build verified (DriverStation + TransportSmoke clean, 17 unit tests pass)
- MJPEG stream verified via curl (correct multipart headers, valid JPEG frames, ~15fps)
- SmartDashboard auto-connect pipeline verified end-to-end (28 new tests on SmartDashboard side)

---

## 2026-03-25 - Glass verified, RTT ping fix, Shuffleboard→NetworkTables V4 rename

### Glass support

- Glass (WPILib's native C++ dashboard) confirmed working with the existing NT4 server — zero server-side protocol changes needed.
- Glass uses the same NT4 WebSocket subprotocol (`v4.1.networktables.first.wpi.edu`), same subscribe pattern, same chooser `.type` follow-up subscribe as Shuffleboard.
- Both Shuffleboard and Glass verified: 33 topics announced, data visible and updating.

### RTT ping fix (root cause of Shuffleboard/Glass silence)

- The ntcore client gates ALL outgoing messages (subscribe, publish, etc.) on `m_haveTimeOffset`. The client sends an RTT ping on connect: `[-1, 0, type=2(Integer), clientTimestamp]`. The server must respond with `[-1, serverTimestamp, type=2(Integer), clientTimestamp]`.
- Our server was sending type code 1 (Double) instead of 2 (Integer) and 0 instead of echoing the client's timestamp. Fixed in NT4Server.cpp.

### Rename: Shuffleboard → NetworkTables V4

Since Glass and Shuffleboard use the identical NT4 protocol, the transport mode was renamed from Shuffleboard-specific to dashboard-agnostic:
- Enum: `eShuffleboard` → `eNetworkTablesV4`
- Backend class: `ShuffleboardBackend` → `NT4Backend`
- Display name: `L"Shuffleboard"` → `L"NetworkTables V4"`
- CLI: `--mode nt4` is primary; `shuffle`/`shuffleboard` kept as backward-compatible aliases
- DriverStation hotkey F7 label updated to "NetworkTables V4"
- All code references, comments, and documentation updated across 12+ files

### Debug cleanup

- Removed 7 blocks of verbose debug printf from NT4Server.cpp (added during RTT investigation)
- Kept 8 production-useful log lines (server start/stop, client connect/disconnect, errors, warnings)

### Glass installation

Glass 2026.2.2 is installed at `D:\code\Glass` (same portable-directory pattern as Shuffleboard):

| File | Purpose |
|---|---|
| `glass.exe` | Glass application (native C++ binary — no JRE needed) |
| `run_glass.bat` | Launch Glass (default config from `%APPDATA%`) |
| `run_glass_local.bat` | Launch Glass pre-configured for localhost:5810 |
| `config_local\glass.json` | Pre-configured: NT4 client mode, `localhost`, port 5810 |

Source: `https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/tools/Glass/2026.2.2/Glass-2026.2.2-windowsx86-64.zip`

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
- **RTT ping response is a hard gate.** The ntcore client sends `[-1, 0, type=2(Integer), clientTimestamp]` on connect. The server MUST respond with `[-1, serverTimestamp, type=2(Integer), clientTimestamp]`. If the type code is wrong or the client's timestamp isn't echoed back, the client never sets `m_haveTimeOffset` and `SendOutgoing()` blocks ALL outgoing messages forever.
- Client binary frames use **pubuid** (client-chosen), not server-assigned topicId. Server needs per-client `pubuid -> topicId` map.
- Echo values back to sender — the server is the single source of truth.
- MsgPack frames may contain multiple concatenated messages per WebSocket frame.
- IXWebSocket's `Sec-WebSocket-Protocol` handling needed a patch (overlay port) to echo exactly ONE selected protocol per RFC 6455.

**Simulator-side gotchas:**
- `NT4Backend` implements both `SmartDashboardDirectPublishSink` (server->client) AND `SmartDashboardDirectQuerySource` (client->server read-back). Both are needed for the simulator to read values written by dashboard clients.
- The query source normalizes flat keys (e.g. `TestMove`) to NT4 paths (`/SmartDashboard/TestMove`).
- `IsChooserEnabledForCurrentConnection()` must include the new mode or the auton AI falls back to the numeric `AutonTest` key and ignores the chooser.
- Headless crash: `TeleAuton_V2::init()` creates an OSG 3D viewer which crashes in headless sessions. Smoke test skips this for NT4 mode.
- WSAStartup: `ix::initNetSystem()` must be called before any IXWebSocket server operations.

**Chooser protocol:**
Base key `Test/Auton_Selection/AutoChooser`, all prefixed with `/SmartDashboard/`:
- Server publishes: `.type`="String Chooser", `options`=[array], `default`, `active`
- Client writes back: `selected` = user's choice
- Read-back priority: `selected` -> `active` -> `default`, with 20-retry x 10ms loop
- See `AutonChooser.h` for the `Ian:` comment with full protocol description.

**Port situation (no conflict):**
- NT4 (Shuffleboard/Glass): port 5810
- Legacy SmartDashboard (NT2 TCP): port 1735

### NT4 protocol quick reference

- **Transport:** WebSocket, resource path `/nt/<clientname>`
- **Subprotocols:** `v4.1.networktables.first.wpi.edu` (preferred), `networktables.first.wpi.edu` (v4.0)
- **Control messages:** JSON text frames — array of `{method, params}` objects
- **Server→client:** `announce`, `unannounce`, `properties`
- **Client→server:** `subscribe`, `unsubscribe`, `publish`, `unpublish`, `setproperties`
- **Value updates:** MsgPack binary — `[topicID, timestamp_us, dataType, value]`
- **Data types:** boolean=0, double=1, int=2, float=3, string=4, raw=5, boolean[]=16, double[]=17, int[]=18, float[]=19, string[]=20
- **Full spec:** https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc

### What to publish (current widget types)

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

## 2026-03-21 - Native Link SHM transport declared stable; live telemetry auto-register fix

### Live telemetry root cause

- `Core::PublishInternal` in `NativeLink.cpp` rejected writes on any topic not found in the registry with `WriteRejectReason::UnknownTopic`. `RegisterDefaultTopics` only pre-registered 7 topics. `TeleAutonV2` publishes ~26 keys per loop cycle (Velocity, Heading, Travel_Heading, all wheel velocities/voltages/encoders, swivel voltages/raws, predicted_*). All of these were silently dropped — no crash, no error surface, just missing values on the dashboard.

### Fix

- `Core::PublishInternal`: when `allowServerOnly=true` (server-originated write) and the topic is not found, auto-register it as `TopicKind::State`, `RetentionMode::LatestValue`, `replayOnSubscribe=true`, `WriterPolicy::ServerOnly`, value type inferred from the incoming value. Client writes on unknown topics are still rejected.
- `RegisterDefaultTopics` kept minimal (only topics that need special writer policies pre-declared). Added `Ian:` comment explaining that robot-code telemetry keys must NOT be added there — the auto-register path is the correct home.

### Verification

- `tools/native_link_live_telemetry_verify.py` (SmartDashboard repo): **PASS** — Velocity (57 distinct values), Y_ft (56 distinct), Wheel_fl_Velocity (12 distinct), all 7 required keys, 410+ updates each.
- `tools/native_link_shm_disconnect_stress.py 50 400`: **50/50 PASS, zero warnings** after 4 synchronization fixes in the stress script.

### Declaration

- SHM transport declared **stable**. Next session target: Native Link TCP carrier.

## 2026-03-20 - Native Link manual carrier toggle and observe helper alignment

- Added a debug-only Native Link carrier picker to the `DriverStation` dialog so local manual validation can switch the simulator authority between `Shared Memory (SHM)` and `TCP/IP` without editing environment variables by hand.
- The picker persists the selected carrier in the local `DriverStation.ini` settings file and reapplies the matching `NATIVE_LINK_*` environment before Native Link backend initialization.
- Added runtime knobs to `DriverStation_TransportSmoke` so manual/paired Native Link observe flows can:
  - delay the initial chooser/`TestMove` seed after startup
  - override `TestMove` (for example `10`) without rebuilding.
- This keeps Robot_Simulation aligned with the current SmartDashboard-side manual debugging workflow, where both apps now need explicit carrier agreement to compare SHM and TCP behavior cleanly.

## 2026-03-19 - Native Link IPC carrier alignment checkpoint

- Updated the simulator-owned Native Link shared-memory carrier to stay in lockstep with the SmartDashboard checkpoint work:
  - removed the packed-struct dependency around shared atomics
  - added explicit layout/alignment assertions
  - added `snapshotCompleteSessionId` so snapshot completion is tracked independently from client write acknowledgements.
- Kept the simulator authority semantics unchanged while making the transport contract safer for continued cross-process startup debugging.
- Validation checkpoint:
  - `DriverStation` and `robot_unit_tests` still build
  - `robot_unit_tests.exe --gtest_filter=*NativeLink*` still passes
  - the remaining blocker is still on the SmartDashboard client startup/restart side, not the simulator-owned carrier update itself.

## 2026-03-17 - Direct survive replay simplification and paired restart recovery

- Simplified robot-side Direct retained-command replay during dashboard reconnects:
  - retained replay now keys off the inactive->active consumer heartbeat transition instead of waiting for a changed `consumerInstanceId`
  - this avoids missing dashboard restarts where the dashboard preserves command intent but the subscriber-side identity logic no longer produces the old change signal.
- Hardened direct command subscriber identity generation:
  - subscriber instance ids now mix process id, steady-clock time, and local counter state instead of using only a simple in-process increment.
- Paired result with the SmartDashboard survive/startup fixes:
  - remembered `TestMove=3.5` survives dashboard restart
  - chooser selection survives dashboard restart
  - subsequent robot-survive activation reads the remembered value correctly and passes the paired stress flow.
- Practical takeaway:
  - the final remaining survive fix was mostly startup/reconnect ordering and reconnect detection, not a major transport rewrite.

## 2026-03-17 - Direct numeric recovery and chooser-safe fallback

- Added non-destructive SmartDashboard `TryGet*` reads so Direct-mode numeric lookups do not misinterpret missing values as valid zeroes.
- Restored reliable Direct autonomous execution by fixing robot-side `TestMove` reads at activation time.
- Kept chooser support available for Direct mode while preserving numeric fallback when chooser state is absent.
- Added focused transport/auton tracing that made it easier to distinguish command-path bugs from harness/reporting bugs.
- Expanded documentation structure with `docs/learning/` and `docs/journal/` for more student-friendly engineering context.

## 2026-03-16 - Direct transport stress harness and consumer-cursor hardening

- Continued the Direct-mode dashboard interoperability push with a stronger harness around SmartDashboard pairing:
  - `DriverStation_TransportSmoke` now serves as a repeatable deterministic smoke runner for chooser + `TestMove` setup and 10-second auton execution
  - SmartDashboard process/probe/watch tooling on the companion repo is now part of the active debug loop for repeated restart stress
- Updated Robot_Simulation Direct transport to better match the SmartDashboard-side cursor model:
  - direct telemetry replay now tracks consumer instance changes
  - publisher free-space accounting now uses the active consumer cursor instead of the obsolete shared `readIndex`
- Resulting status checkpoint:
  - real single-dashboard Direct runs are healthier and smoke telemetry is visible again after repeated restarts
  - repeated robot-restart stress is improved but still not considered fully deterministic over long sequences
  - passive extra observers still perturb the transport, so true multi-observer robustness remains future work rather than the current Direct-mode assumption.

## 2026-03-16 - Shared chooser contract alignment for simulator and dashboards

- Began converging Robot_Simulation autonomous selection around the same chooser semantics used by current SmartDashboard/Shuffleboard compatibility work:
  - chooser base publishes `.type`, `options`, `default`, `active`, and `selected`
  - robot-side autonomous selection now prefers chooser labels first and keeps legacy `AutonTest` string fallback during transition
- Updated legacy SmartDashboard sendable publish path to emit `.type` alongside historical `~TYPE~` so documentation and compatibility adapters can share one chooser contract.
- Updated transport smoke startup coverage to seed chooser-style autonomous keys instead of numeric-only `AutonTest`.
- Documentation now records that modern NT string-array chooser options remain the preferred long-term encoding. SmartDashboard direct transport now supports them end-to-end, while Robot_Simulation still bridges chooser options as comma-strings until its direct protocol is upgraded to match.

## 2026-03-15 - Direct startup/reconnect command retention stabilization

- Added direct transport smoke harness target `DriverStation_TransportSmoke` to exercise startup path quickly against live dashboard state.
- Added autonomous selection resolver helper (`Source/Modules/Input/AI_Input/AutonSelection.h`) and gtest coverage (`tests/auton_selection_tests.cpp`) for retry/default/clamp behavior.
- Fixed dashboard-owned command startup race behavior in robot-side activation path:
  - `AI_Input_Example` now uses retry-based selection resolve for first auton activation.
  - avoids robot-side overwrite of operator-owned startup values.
- Hardened direct/legacy command reads for operator keys by supporting scoped and flat aliases in helper functions:
  - prefers `Test/<key>` then falls back to `<key>`
  - supports number-from-string fallback for mixed widget/control publishing paths
  - removed robot-side default writeback in RobotTester helper paths.
- Hardened NT entry/type handling for legacy path stability during mode and startup transitions.
- Validation:
  - `DriverStation`, `DriverStation_TransportSmoke`, and `robot_unit_tests` build successfully.
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (10/10).
  - Manual direct-mode stress validated startup/reconnect behavior for `AutonTest` and `TestMove` when paired with updated SmartDashboard.

## 2026-03-11 - CMake migration milestone (DriverStation vertical slice)

- Added top-level `CMakeLists.txt` to begin non-invasive CMake migration while preserving legacy Visual Studio solution/project files.
- Set `DriverStation` as the primary application target for the first end-to-end milestone.
- Added CMake targets for core runtime:
  - `SmartDashboard` (static)
  - `BaseCore` (static)
  - `dx_Joystick_Controller` (static)
  - `OSG_Viewer` (shared)
  - `RobotAssembly_static` (static)
  - `DriverStation` (exe)
- Added optional CMake target families:
  - Legacy apps (`BUILD_LEGACY_APPS`): `RobotAssembly`, `OutputViewer`
  - Tester apps (`BUILD_TESTER_APPS`): `AI_Input_Tester`, `OSG_Viewer_Tester`, `OSG_ViewTester`, `dx_Joystick_Controller_App`, `Entity1D_Tester`, `Entity2D_Tester`, `MotionControl2D_simple_Tester`, `MotionControl2D_physics_Tester`, `SwerveRobot_Tester`
  - Unit tests (`BUILD_UNIT_TESTS`): `robot_unit_tests` with CTest integration
- Integrated OSG through vcpkg package config (`unofficial-osg`) and linked OSG/OpenThreads via `target_link_libraries`.
- Installed OSG and dependencies in local vcpkg (`osg:x64-windows`).
- Removed hardcoded OSG `#pragma comment(lib, ...)` links from:
  - `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV1.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV2.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleOpV2.cpp`
  - `Source/Application/RobotAssembly/RobotAssembly/TeleOpV3.cpp`
- Removed remaining `#pragma comment(lib, ...)` from `.cpp` sources and moved linkage to CMake targets:
  - `Source/Application/DriverStation/DriverStation/DriverStation.cpp`
  - `Source/Libraries/SmartDashboard/OSAL/System.cpp`
  - `Source/Modules/Input/dx_Joystick_Controller/dx_Joystick_Controller/dx_Joystick.cpp`
  - `Source/Modules/Output/OSG_Viewer/OSG_Viewer/OSG_Viewer.cpp`
  - `Source/Modules/Robot/SwerveRobot/SwerveRobot/SwerveRobot_Tester.cpp`
- Added `#include <locale.h>` in `Source/Base/LUA.cpp` to resolve CMake/MSVC compile failure.
- Normalized Windows macro checks from `_Win32` to `_WIN32` in Base files to remove global compile-definition workaround.
- CMake build validation:
  - Configure: `cmake -S . -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE=/d/code/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows`
  - Build: `cmake --build build-vcpkg --config Debug --target DriverStation`
  - Run: `build-vcpkg/bin/Debug/DriverStation.exe` (launches and initializes NetworkTables threads)
- Additional build validation:
  - Configure with optional targets: `-DBUILD_LEGACY_APPS=ON -DBUILD_TESTER_APPS=ON`
  - Build optional apps/testers succeeds in Debug
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (1/1)

### Notes

- CMake excludes `Source/Libraries/SmartDashboard/networktables2/util/System.cpp` (VxWorks implementation) and uses Windows `OSAL/System.cpp`.
- Non-blocking warnings remain from legacy `_CRT_SECURE_NO_WARNINGS` macro redefinitions.

## 2026-03-11 - Documentation organization and GoogleTest integration

- Reorganized docs to match SmartDashboard-style layout:
  - Added `docs/overview.md` for long-form project orientation.
  - Moved durable history to `docs/project_history.md`.
  - Kept `Agent_Session_Notes.md` lean and handoff-focused.
  - Slimmed top-level readme to quick-start + documentation map.
- Integrated GoogleTest with CMake unit-test flow:
  - Added `find_package(GTest CONFIG REQUIRED)` and `include(GoogleTest)`.
  - Replaced custom test `main()` with gtest-based suites.
  - Added `gtest_discover_tests(robot_unit_tests)` for CTest/Visual Studio discovery.
- Split tests into focused files:
  - `tests/entity1d_tests.cpp`
  - `tests/entity2d_tests.cpp`
  - `tests/drivekinematics_tests.cpp`
- Added `docs/testing.md` with configure/build/run and Visual Studio test-discovery guidance.
- Validation:
  - `cmake --build build-vcpkg --config Debug --target robot_unit_tests` succeeds.
  - `ctest --test-dir build-vcpkg -C Debug --output-on-failure` passes (6/6).

## 2026-03-15 - Transport mode feature planning handoff

- Added `docs/transport_modes_strategy.md` to capture next major feature direction.
- Documented explicit three-mode goal:
  - `Direct Connect`
  - `Legacy SmartDashboard`
  - `Shuffleboard`
- Captured sequencing requirement: establish Direct Connect before Shuffleboard work.
- Captured compatibility rule: preserve legacy SmartDashboard behavior as baseline/oracle.
- Linked transport strategy in top-level `ReadMe.md` documentation map.
- Updated `Agent_Session_Notes.md` checklist to prioritize transport mode selection and Direct/Legacy contract implementation.

## 2026-03-15 - Safe cleanup while transport work continues

- Applied no-risk warning hygiene for `_CRT_SECURE_NO_WARNINGS` macro definitions by guarding local defines:
  - `Source/Application/DriverStation/DriverStation/stdafx.h`
  - `Source/Base/Base_Includes.h`
  - `Source/Base/LUA.cpp`
  - `Source/Base/Misc.cpp`
- Performed no-risk include deduplication in `Source/Application/DriverStation/DriverStation/stdafx.h` and `Source/Base/Misc.cpp`.
- Added and verified startup transport smoke harness target (`DriverStation_TransportSmoke`) to exercise Direct mode startup sequence quickly.
- Added unit tests for autonomous selection retry/default/clamp behavior (`tests/auton_selection_tests.cpp`) and integrated into `robot_unit_tests`.
