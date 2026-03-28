# Robot_Simulation Roadmap

Actionable future work for this repository.

- Items move here from `Agent_Session_Notes.md` (in-progress/deferred).
- When an item is completed, move it to `docs/project_history.md` and remove it from this file.
- Cross-repo items that involve SmartDashboard are marked with **(cross-repo)**.

---

## Active: Camera widget remaining phases (`feature/camera-widget`)

Camera MJPEG server (Phase 3), Video Source selector, and "The Grid" TronGridSource are all **complete**. Full details in `docs/project_history.md` (2026-03-28 and 2026-03-27 entries).

- [ ] **Phase 4:** Backup camera guide lines — OSG-side overlay baked into MJPEG frames (Honda-style curved path lines driven by velocity/angular velocity). See `D:\code\SmartDashboard\docs\camera_widget_design.md` for the design note on two-source-mode support (OSG framebuffer vs USB camera + OSG composite). **(cross-repo)**
- [ ] Manual end-to-end test with SmartDashboard camera viewer dock **(cross-repo)**

---

## Deferred work

Lower-priority items parked for future consideration.

- Expand smoke test published keys from ~6 + chooser to full TeleAutonV2 (~49 keys)
- Debug builds: manual carrier picker in DriverStation dialog for SHM vs TCP comparisons
