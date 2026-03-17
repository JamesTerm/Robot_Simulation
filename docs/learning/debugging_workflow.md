# Debugging Workflow

This folder is for durable engineering lessons that help students and contributors debug the simulator systematically.

## Regression-first transport debugging

Use a ladder of tests instead of jumping straight to the largest scenario:

1. Verify a single value can be published and observed.
2. Verify the real owning process keeps that value alive.
3. Verify the robot reads the value at the moment it matters.
4. Only then move to DriverStation enable/disable loops and stress runs.

## Why this matters

- Shared-state systems can make a missing value look like a valid default.
- A short-lived helper process may not behave like the real dashboard session.
- Small commits make it practical to compare today against a known-good checkpoint.

## Practical rule

When debugging Direct mode, prove one truth at a time and keep a stable baseline path available.
