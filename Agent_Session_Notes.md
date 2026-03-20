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
- Current working backend is still SHM + named events; explicit `tcp` selection currently fails cleanly as a placeholder.
- `Robot_Simulation` is the first reference authority/example for Native Link semantics, but should not become the permanent home of all reusable authority-side logic.
- Important runtime lesson: `eNativeLink` must stay out of `DashboardTransportRouter::UsesLegacyTransportPath()` or the UI can claim Native Link while the old backend is still running underneath.

## Strategy reminders

- Keep simulator-side cleanup aligned with the one-contract / many-carriers / many-adapters roadmap.
- Preserve SHM as the diagnostic/reference carrier while TCP authority work is added.
- Product stance: TCP is the intended normal runtime carrier. SHM remains the internal diagnostic/reference carrier and should stay reachable through developer overrides, not as a normal team-facing mode toggle.
- Favor extraction that keeps this repo a clean teaching example of server-authoritative session/snapshot/lease behavior.

## Immediate next-session focus

1. Keep runtime TCP integration environment-driven/internal first; do not add a public SHM/TCP selector unless there is a clear product need.
2. Keep the example/use-case story adapter-friendly for future SmartDashboard / Shuffleboard / Elastic / bridge integrations.
3. After the next cleanup slice, rerun focused Native Link tests plus the SmartDashboard SHM and TCP runtime probes.
