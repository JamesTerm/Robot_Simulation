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

## SHM transport status — STABLE

- SHM transport declared stable as of 2026-03-21.
- Live telemetry fix landed in `NativeLink.cpp` `Core::PublishInternal`: unknown topics on server-originated writes are now auto-registered instead of silently rejected.
- `RegisterDefaultTopics` stays minimal — only special-policy topics are pre-declared there. See `Ian:` comment in `NativeLinkAuthorityHelpers.cpp`.
- `DriverStation_TransportSmoke` was rebuilt and verified with `native_link_live_telemetry_verify.py` (SmartDashboard repo tools).

## Key invariants (do not break)

- `Core::PublishInternal` auto-register only applies when `allowServerOnly=true`. Client writes on unknown topics must still be rejected.
- `RegisterDefaultTopics` must not grow to list every TeleAutonV2 key — the auto-register path handles those.
- `TeleAutonV2` publishes ~26 keys per loop. `Heading` (line 324) and `Travel_Heading` (line 307) are unconditional — both are now delivered to the dashboard.

## Strategy reminders

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- Preserve SHM as the diagnostic/reference carrier while TCP authority work is added.
- Product stance: TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier and should stay reachable through developer overrides, not as a normal team-facing mode toggle.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.

## Immediate next-session focus

1. Begin Native Link TCP authority work — SHM is stable and serves as the reference comparison backend.
2. For any TCP fix, verify with `native_link_live_telemetry_verify.py` (switch `NATIVE_LINK_CARRIER` to `tcp`) and the SmartDashboard TCP runtime probe.
3. Rebuild `DriverStation_TransportSmoke` after any authority-side changes and rerun the SmartDashboard stress scripts to confirm no regression.
