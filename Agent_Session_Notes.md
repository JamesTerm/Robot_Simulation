# Agent session notes

- Keep this file short and handoff-focused.
- Move durable milestone history to `docs/project_history.md`.
- Use `D:\code\SmartDashboard\docs\native_link_rollout_strategy.md` as the canonical long-term Native Link rollout note.

## Workflow note

- `apply_patch` expects workspace-relative paths with forward slashes.
- Use CRLF line endings for C++ source files in this repo.
- Read nearby `Ian:` comments before editing and add new ones where session, ownership, carrier, or runtime-mode lessons would be easy to lose.

## Active Native Link context

- `Source/Application/DriverStation/DriverStation/NativeLink.h` now exposes the active carrier/config boundary for the simulator-owned authority and test client.
- Current working backends are SHM + named events for diagnostics and localhost TCP for the reference/runtime path.
- `Robot_Simulation` is the first reference authority/example for Native Link semantics, but should not become the permanent home of all reusable authority-side logic.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.
- Debug builds now expose a manual Native Link carrier picker in the DriverStation dialog so the simulator side can match SmartDashboard's SHM vs TCP manual comparisons.
- `DriverStation_TransportSmoke` now supports `--startup-delay-ms` and `--test-move` for longer manual observe runs without rebuilding.

## Current manual-debug checkpoint

- SmartDashboard and DriverStation now both have debug-only Native Link carrier overrides for manual SHM vs TCP parity checks.
- The latest one-dashboard SHM observe flow can connect reliably when the authority starts first.
- Current paired gap still under investigation: Native Link retained/default state arrives, but live motion telemetry like `Velocity` still does not yet match Direct Connect behavior in the latest manual observe pass.

## Strategy reminders

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- Preserve SHM as the diagnostic/reference carrier while TCP authority work is added.
- Product stance: TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier and should stay reachable through developer overrides, not as a normal team-facing mode toggle.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.

## Immediate next-session focus

1. Compare the same manual Native Link flow on SHM vs TCP with both apps explicitly aligned on carrier.
2. Trace why live motion telemetry still falls short of Direct Connect after Native Link connect succeeds.
3. If follow-up fixes land, rerun focused Native Link tests plus the SmartDashboard SHM and TCP runtime probes.
