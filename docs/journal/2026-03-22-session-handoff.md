# Agent Session Handoff - 2026-03-22

## What This Document Is

This is a technical handoff for the next agent session continuing the
`feature/debug-remote-stability` investigation. Read this document completely before
touching any code.

---

## The Bug

**Symptom:** The DriverStation app window appears for ~1-2 seconds then disappears
silently on a remote machine when running the Debug build. Release works. NativeLink
(mode 3) works. Does not reproduce locally.

**Branch:** `feature/debug-remote-stability` (last commit `af2e192`)
**Master:** `d7aa4ea` -- untouched

---

## Constraints (Do Not Violate)

- Remote machine has **no development environment**. No debugger. No assert dialogs
  (`assert()` calls `abort()` silently). Only `OutputDebugStringA` via DebugView is
  available.
- Working on `feature/debug-remote-stability` only. Do NOT touch master.
- Fix strategy is **surgical**. Identify exact cause, fix only that.
- For any DIAG instrumentation: use `OutputDebugStringA` with plain ASCII only
  (no em-dashes, no Unicode escapes -- they garble in DebugView on remote machines).
- All DIAG commits must be squash-merged into master as a single clean commit when done.
  No DIAG history in master.
- User builds Debug in Visual Studio, copies all binaries (EXE + DLLs) to remote,
  confirms by timestamp.

---

## What Is Confirmed True

### The triggering call
Commenting out this single line in `DriverStation.cpp:631` makes the window stay up:
```cpp
ApplyConnectionMode(s_InitialConnectionMode);
```
This call happens **before** `_robot_tester.RobotTester_init()`.

### The call chain
`ApplyConnectionMode(eDirectConnect)`
-> `s_pRobotTester->SetConnectionMode(eDirectConnect)`
-> `m_transport_router.SetMode(eDirectConnect)`
-> router not yet initialized -> `Initialize(eDirectConnect)`
-> `EnsureBackend()` -> creates `DirectConnectBackend`
-> `m_backend->Initialize()`
-> `m_publisher->Start()` -- starts publisher background thread
-> `m_commandSubscriber->Start()` -- starts subscriber background thread

### Timing
The process dies approximately **1 second** after `DirectConnectBackend::Initialize()`
returns successfully. Both threads start and run at least one iteration before death.

### No WM_DESTROY
The crash produces no `WM_DESTROY`, no exception log, no assert dialog. `GetMessage`
simply stops returning. This is consistent with `abort()` being called on a background
thread -- e.g., from an uncaught exception hitting `std::terminate`, or from a
Debug-CRT `_ASSERTE` that calls `abort()` when no debugger is present.

### Heap was clean
`_CrtCheckMemory()` passed immediately after init.

### `TeleAutonV2::init()` is NOT the cause
All inits in `TeleAutonV2::init()` were suppressed during earlier binary search.
Window still died. So the viewer, robot, script loader, etc. are not involved.

---

## What Was Fixed (Keep These)

### Fix 1: `s_populatingCombo` re-entrancy guard (commit `d8b084e`)
`PopulateConnectionModeCombo` fires `CBN_SELCHANGE` as it adds items. Without a guard,
this called `ApplyConnectionMode` recursively during startup. Fixed with a bool flag
`s_populatingCombo` in `DriverStation.cpp`.

### Fix 2: `AppendTransportLogLine` hardcoded path (commit `af2e192`)
The function used a magic static `std::ofstream` opening `D:\code\Robot_Simulation\.debug\`.
On the remote machine this path does not exist. Fixed to use `GetTempPathA()` instead.
**Note:** This fix did NOT stop the 1-second crash. The crash predates the log file
write. But the path fix is real and correct -- keep it.

---

## What Was Ruled Out

- `TeleAutonV2::init()` -- all inits suppressed, still crashed
- `AppendTransportLogLine` blocking on bad path -- fixed, still crashed
- Heap corruption -- `_CrtCheckMemory()` passed

---

## Current Suspicion

The 1-second death is almost certainly a **Debug-CRT `_ASSERTE` or
`std::terminate` on one of the background threads**, triggered by something the
threads do after their first loop iteration (~16ms sleep, then second loop).

The publisher thread's `RunLoop` and subscriber thread's `RunLoop` both have
`try/catch` blocks. But `_ASSERTE` is NOT a C++ exception -- it calls `_CRT_ASSERT`
which calls `abort()` directly. `std::terminate` from an uncaught exception in a
`std::thread` also calls `abort()` directly. Both are invisible without a debugger.

### The most likely suspects in the second loop iteration

1. **Publisher `MaybeReplayRetainedSnapshot()`** -- accesses `m_header` atomics.
   If `m_header` pointer is stale, corrupt, or the atomic was not properly
   placement-new'd, the MSVC Debug atomic implementation may `_ASSERTE` on internal
   state checks.

2. **Subscriber `DrainPendingValues()` -> `ReadNextValue()`** -- reads from the SHM
   ring. Same concern about stale/corrupt ring state.

3. **The gap-check block in publisher `RunLoop`** -- computes
   `loopDeltaUs = nowUs - previousLoopUs`. On the very first real iteration
   (second pass through the loop after ~16ms sleep), `previousLoopUs` is set to the
   time at first entry. The delta will be ~16000us, well under 250000us, so the gap
   log is NOT triggered. This path is safe.

4. **`std::thread` destructor on a running thread** -- if something destroys the
   `DirectConnectBackend` while threads are running, `std::thread::~thread()` calls
   `std::terminate()`. Could this happen if the `DirectConnectBackend` is destroyed
   before the threads are stopped? Check `DashboardTransportRouter::SetMode` -- it
   calls `m_backend->Shutdown()` then immediately replaces `m_backend`. If `Shutdown`
   doesn't join the threads, the destructor would terminate the process.

---

## Recommended Next Steps

### Step 1 -- Confirm transport-only vs. transport+viewer interaction

The code currently runs both `ApplyConnectionMode` AND `RobotTester_init()` (which
calls `TeleAutonV2::init()` including the viewer). We already know `ApplyConnectionMode`
alone causes the crash, but the viewer running simultaneously could be masking or
compounding it. To get a clean signal:

Add `return;` at the very top of `RobotTester_Internal::init()` in `Robot_Tester.cpp`
line 29, so `TeleAutonV2::init()` is suppressed. Keep `ApplyConnectionMode` enabled.
If window still dies: crash is purely in the transport threads, no viewer involved.

### Step 2 -- Isolate publisher vs. subscriber

If Step 1 still crashes, add a DIAG flag. In `DirectConnectBackend::Initialize()`:
- Try commenting out `m_commandSubscriber->Start()` (subscriber only, keep publisher).
  Does the crash persist?
- Try commenting out `m_publisher->Start()` (publisher only, keep subscriber).
  Does the crash persist?

This identifies which thread is dying.

### Step 3 -- Look at `DashboardTransportRouter::SetMode` for premature destroy

There are TWO calls that could start the transport:
1. `ApplyConnectionMode(s_InitialConnectionMode)` at line 631 (before init)
2. `_robot_tester.RobotTester_init()` at line 634, which calls
   `InitializeConnectionMode(m_ConnectionMode)` -> `m_transport_router.Initialize(mode)`

In `DashboardTransportRouter::Initialize()`:
```cpp
if (m_is_initialized)
{
    SetMode(initial_mode);  // Already initialized -- calls SetMode
    return;
}
```
And `SetMode` does:
```cpp
if (m_mode == mode && m_backend)
    return;  // Same mode, early out -- GOOD
```
So the second call should be a no-op. Confirm this is actually the path taken by adding
a single `OutputDebugStringA` to the early-return branch.

### Step 4 -- Add a second-iteration tombstone log

If transport threads are the sole suspect, add a single log line as the very LAST thing
each thread does at the BOTTOM of each loop (after `sleep_for`). If this log appears,
the thread completed the second iteration cleanly. If it doesn't, the thread died during
the second iteration.

```cpp
// In publisher RunLoop, at the bottom of the while loop, after sleep_for:
OutputDebugStringA("[Publisher] loop bottom\n");

// In subscriber RunLoop, at the bottom of the while loop (after the catch blocks):
OutputDebugStringA("[Subscriber] loop bottom\n");
```

---

## Key Files

| File | Role |
|------|------|
| `Source/Application/DriverStation/DriverStation/DriverStation.cpp` | Main entry point; `ApplyConnectionMode` at line 403; triggering call at line 631 |
| `Source/Application/DriverStation/DriverStation/Transport.cpp` | All transport: `DashboardTransportRouter`, `DirectConnectBackend`, `DirectPublisherStub`, `DirectCommandSubscriber` |
| `Source/Application/DriverStation/DriverStation/Robot_Tester.cpp` | `RobotTester_Internal::init()` calls `m_tele.init()` (TeleAutonV2) |
| `Source/Application/RobotAssembly/RobotAssembly/TeleAutonV2.cpp` | `Test_Swerve_Properties::init()` -- viewer, robot, scripts |

---

## Squash Plan (When Root Cause Is Fixed)

Do NOT cherry-pick or preserve DIAG history.

```
git checkout master
git merge --squash feature/debug-remote-stability
git commit -m "fix: DriverStation Debug crash on remote machine -- <one-line root cause>"
```

The commit message should state the actual root cause precisely once it is known.
