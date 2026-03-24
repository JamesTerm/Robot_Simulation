# 2026-03-17 - Direct debugging journey

## Why this entry exists

This session produced a useful debugging lesson: several end-to-end failures were real, but some were caused by the test harness rather than the robot command path. The value of the session was learning how to separate those two cases quickly.

## Main sequence

1. Direct chooser work exposed too many moving parts at once.
2. The autonomous path was split so numeric `AutonTest` stayed available as a stable baseline.
3. We discovered the probe itself was part of the problem because a short burst did not match the behavior of a real running dashboard session.
4. After matching the probe to the working streaming test pattern, Direct numeric control population became reliable again.
5. The remaining robot-side bug was that missing numeric values could look like a successful read of `0`.
6. Adding non-destructive `TryGet*` APIs to the simulator-side SmartDashboard bridge fixed that issue.
7. Once `TestMove` read correctly, the robot moved again under Direct mode.

## Important takeaways

- Keep one stable baseline path available while testing a new feature path.
- Rename overlapping experimental keys early so one control system cannot mask another.
- A helper tool must mimic the real ownership model, not just the desired values.
- Strategic commits are not just bookkeeping; they are part of the debugging method.

## State at the end of the session

- Direct numeric autonomous selection works again.
- Direct chooser selection also reaches the robot when chooser state is present.
- Harness cleanup is still important, but it is now clearly separate from the original command-delivery bug.
