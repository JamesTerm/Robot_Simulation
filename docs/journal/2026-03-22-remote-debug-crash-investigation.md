# 2026-03-22 - Remote Debug Crash Investigation

## The Problem

The DriverStation app window appears for 1-2 seconds then disappears silently on a
remote machine when running the Debug build. Release mode works fine. NativeLink
(mode 3) also works. The bug never reproduces locally on the development machine.

This entry documents the full investigation process as a teaching example. Not all of
the hypotheses turned out to be correct -- and that is the point. Real debugging rarely
follows a straight line.

---

## Why This Bug Is Hard to Reproduce Locally

The remote machine has no development environment:

- No debugger can be attached.
- No `assert()` dialog boxes appear (they call `abort()` on Windows when no debugger is
  present, silently killing the process).
- `OutputDebugStringA` output IS visible using DebugView (Sysinternals), which was our
  primary instrument throughout.

The development machine has the full project tree at `D:\code\Robot_Simulation\`. The
remote machine does not.

**Lesson:** Before instrumenting a bug, understand what tools you have available on the
target machine. An `assert()` that shows a dialog locally is a silent process killer
remotely.

---

## Investigation Technique: Binary Search

When a complex system crashes and you don't know where, the most reliable strategy is
**binary search**. You suppress half the code, deploy, and observe whether the crash
still occurs. This tells you which half contains the bug. Repeat until the crashing line
is isolated.

This is the same principle as bisecting a sorted list -- each test eliminates half the
remaining candidates.

**The key discipline:** commit each suppression step separately, with a clear DIAG
label. This lets you:
- Revert to any checkpoint if a hypothesis is wrong.
- Squash all DIAG commits into one clean fix commit at the end.
- Show collaborators (or a fresh agent session) exactly what was tested.

---

## Commit History (Chronological)

The following commits were made on branch `feature/debug-remote-stability`:

| Commit | What it did |
|--------|-------------|
| `40e25a8` | Added `WM_DESTROY` log and GetMessage exit log to distinguish clean exit from crash |
| `366d6f9` | Binary search step 1: suppressed lower half of `TeleAutonV2::init()` |
| `8e55fc3` | Binary search step 2: suppressed `InitControllers` and `m_Goal.Initialize` |
| `bd132b7` | Suppressed ALL inits in `TeleAutonV2::init()` -- window still died |
| `d8b084e` | **Real fix:** guarded `PopulateConnectionModeCombo` against re-entrant `CBN_SELCHANGE` |
| `1cc8675` | Suppressed entire `ApplyConnectionMode` body -- window stayed up (key isolation) |
| `7203975` | Re-enabled only `SetConnectionMode` inside `ApplyConnectionMode` -- window died again |
| `57d46ac` | Added checkpoint logs inside `DirectConnectBackend::Initialize()` |
| `3b4f321` | Added per-step logs in publisher `RunLoop` to find the death point |
| `af2e192` | Fixed `AppendTransportLogLine` hardcoded path + restored all suppressed code + removed DIAG |

---

## Hypotheses Tested (In Order)

### Hypothesis 1: The crash is inside `TeleAutonV2::init()`

**Rationale:** `TeleAutonV2::init()` starts the OSG viewer, loads scripts, and
initializes several subsystems. Any of these could fail on a machine with a different
configuration.

**Test:** Suppressed all inits in `TeleAutonV2::init()` progressively (steps 1-3).

**Result:** Even with ALL inits suppressed, the window still died. The crash is not
inside `TeleAutonV2::init()`.

**Lesson:** The most obvious candidate is not always the right one. Prove it before
assuming it.

---

### Hypothesis 2: The crash is triggered by `ApplyConnectionMode()`

**Test:** Commented out the single call `ApplyConnectionMode(s_InitialConnectionMode)`
at line 631 of `DriverStation.cpp`. Window stayed up permanently.

**Result:** Confirmed. The crash is inside the call chain of `ApplyConnectionMode`.

**Lesson:** When you find the smallest possible change that makes the bug disappear,
that is your anchor point. Everything else is now inside that scope.

---

### Hypothesis 3: The crash is inside `SetConnectionMode`

**Rationale:** `ApplyConnectionMode` has several side effects. Binary search: restore
only `SetConnectionMode(mode)`, suppress everything else.

**Test:** Replaced `ApplyConnectionMode` body with only `s_pRobotTester->SetConnectionMode(mode)`.

**Result:** Window still died. The crash is inside `SetConnectionMode`'s call chain:
`SetMode` -> `Initialize` -> `EnsureBackend` -> `DirectConnectBackend::Initialize()` ->
starts publisher and subscriber threads.

---

### Hypothesis 4: The crash is inside `DirectConnectBackend::Initialize()`

**Test:** Added step-by-step `OutputDebugStringA` checkpoint logs inside
`DirectConnectBackend::Initialize()`. All checkpoints printed successfully. Both threads
started. `RobotTester_init()` completed. `_CrtCheckMemory()` passed.

**Result:** The process died approximately **1 second after** `Initialize()` returned.
`GetMessage` never returned -- the process terminated, not hung. No `WM_DESTROY` was
received, which means the process was killed externally (i.e., by the CRT or OS), not
by a clean DestroyWindow path.

---

### Hypothesis 5: The publisher thread is blocking inside `MaybeReplayRetainedSnapshot`

**Test:** Added per-step `OutputDebugStringA` logs inside the publisher `RunLoop`.

**Result:** Last log line was `[Publisher] before MaybeReplay`. The thread never reached
`[Publisher] before FlushNow`. This meant the thread was blocking somewhere in
`MaybeReplayRetainedSnapshot()`, or more precisely in the gap check code immediately
before it -- the code that calls `AppendTransportLogLine`.

---

### Hypothesis 6 (accepted at the time, later disproved): `AppendTransportLogLine` was blocking

**Rationale:** `AppendTransportLogLine` contained a magic static `std::ofstream`
initialized with a hardcoded path: `D:\code\Robot_Simulation\.debug\`. On the remote
machine, `D:\code\` does not exist. `CreateDirectoryW` on a nonexistent parent path
can stall for several seconds (especially if `D:\` is a network drive or an absent
drive letter), and the subsequent `std::ofstream` open attempt would fail too.

**Fix applied:** Changed `AppendTransportLogLine` to use `GetTempPathA()` to resolve
a portable `%TEMP%\direct_transport_debug_log.txt` path.

**Result on remote:** Window **still died**. The path fix was a real bug (it would have
caused issues), but it was not the root cause of the 1-second crash.

**Lesson:** Fixing a real bug is not the same as fixing the bug you are looking for.
Both can coexist. Be careful not to declare victory before confirming on the target
machine.

---

## Current State (Session End)

The bug is confirmed to be inside the `ApplyConnectionMode` call chain. The path fix
and `CBN_SELCHANGE` re-entrancy fix are real improvements and should be kept. But the
root cause of the 1-second silent crash on the remote machine is still unidentified.

### What is known for certain

- Disabling `ApplyConnectionMode(s_InitialConnectionMode)` (line 631 of DriverStation.cpp)
  prevents the crash.
- The crash occurs approximately 1 second after `DirectConnectBackend::Initialize()`
  completes successfully.
- The publisher thread starts and runs at least one iteration before the crash.
- `_CrtCheckMemory()` passed after init (heap was clean).
- The crash produces no `WM_DESTROY`, no exception log, no assert dialog.

### What has NOT been ruled out

- Whether the OSG viewer (`m_viewer.init()` inside `TeleAutonV2::init()`, called
  AFTER `ApplyConnectionMode`) is involved. At session end, `TeleAutonV2::init()`
  was fully restored, so both are running simultaneously.
- Whether the crash is in the publisher thread, the subscriber thread, or the main
  thread.
- Whether the crash is a Debug-CRT assert (e.g., an invalid heap pointer, a
  `_ASSERTE` inside MSVC runtime code, a `std::terminate` from a background thread
  exception) that calls `abort()` silently on the remote machine.

### Recommended next binary search steps

1. **Isolate transport vs. viewer:** Comment out only `_robot_tester.RobotTester_init()`
   (which calls `TeleAutonV2::init()` including the OSG viewer). Keep
   `ApplyConnectionMode` running. If the crash disappears, the viewer is involved.
   If it persists, the transport threads alone are sufficient to trigger it.

2. **Alternatively:** Keep `RobotTester_init()` but comment out only
   `ApplyConnectionMode`. Already known to fix it. This confirms the split cleanly.

3. **If transport is isolated:** Binary search inside `DirectConnectBackend::Initialize()`.
   Does commenting out `m_publisher->Start()` fix it? Does commenting out
   `m_commandSubscriber->Start()` fix it?

4. **Watch for `std::terminate`:** A background thread that throws an uncaught exception
   calls `std::terminate`, which calls `abort()`, which kills the process silently with
   no WM_DESTROY, no exception log, no dialog -- exactly what we see. The publisher and
   subscriber `RunLoop` methods have try/catch blocks, but those only catch C++
   exceptions. A Debug-CRT assert or a structured exception (SEH) from, e.g., a bad
   heap access or a `_ASSERTE` failure, bypasses them entirely.

---

## The `CBN_SELCHANGE` Re-entrancy Fix (Real Bug, Keep)

While doing the binary search, a separate real bug was identified and fixed:
`PopulateConnectionModeCombo` sends `CBN_SELCHANGE` notifications as it adds items to
the combo box. Without a guard, this fired `ApplyConnectionMode` recursively during
startup. The fix (`s_populatingCombo` flag) is in commit `d8b084e` and should be kept
in the final squash.

---

## Branch State at Session End

- Branch: `feature/debug-remote-stability`
- Last commit: `af2e192`
- All suppressed DIAG code has been restored.
- All checkpoint `OutputDebugStringA` logs have been removed.
- The `s_populatingCombo` re-entrancy fix and `AppendTransportLogLine` path fix are
  in the codebase and working.
- Master is untouched at `d7aa4ea`.

The plan when the root cause is found: squash-merge `feature/debug-remote-stability`
into master as a single clean commit, no DIAG history.
