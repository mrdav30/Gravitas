# Feature Work Issue Tracker

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before implementing fixes, use
> superpowers:test-driven-development for runtime behavior changes, and use
> superpowers:verification-before-completion before claiming an issue is fixed.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** Active

**Goal:** Keep bugs, correctness risks, documentation defects, and
feature-work-discovered issues separate from feature design plans so each fix
can be triaged, tested, and committed independently.

**Architecture:** This document is intentionally undated. Each tracked item has
its own discovery date, source, status, affected files, and recommended
verification. Feature plans may reference this tracker instead of carrying bug
fixes inside API or design phases.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit,
BenchmarkDotNet when performance evidence is needed, FixedMathSharp core
runtime and tests.

---

## Tracker Rules

- Add new items when feature work uncovers a suspected bug, stale doc, test
  smell, performance anomaly, or correctness risk.
- Keep each item scoped tightly enough to fix and verify independently.
- Record the date on the item, not in this filename.
- Move an item to `Resolved Issues` only after the fix has tests or documented
  verification evidence.
- Do not use this tracker as a substitute for tests, benchmarks, or release
  notes.
- Performance issues should stay in
  [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
  unless they become a confirmed runtime defect. Do not add performance issues
  here until they have been investigated and confirmed as runtime defects.

## Active Issues

### Mixed Discrete Response Can Reverse Restitution-Heavy Kinematic CCD Handoff Velocity

**Discovered:** 2026-06-23  
**Source:** CCD service-level island solver validation  
**Status:** Needs investigation  
**Affected area:** `CollisionResponseMixed`, mixed CCD handoff tests,
`GravitasMixedCollisionService` full-frame response ordering

During kinematic active-source CCD validation, the isolated pure-service
handoff from a kinematic 2D source into a dynamic 3D target correctly transferred
positive target velocity. When the same setup used a full mixed `LateSimulate`
with high restitution, the later mixed discrete response could cancel or reverse
the final observable 3D target velocity after the target settled against the
stopped 2D source. Inelastic resting contact can legitimately zero the velocity,
but restitution-heavy reversal should be investigated as a possible mixed normal
orientation or resting-response issue.

Recommended verification:

- Add a focused full-frame mixed test for kinematic 2D source versus dynamic 3D
  target with restitution enabled.
- Compare the post-CCD pre-mixed-response handoff velocity against the final
  mixed response velocity.
- Verify the symmetric 3D source versus 2D target case so the fix does not
  create asymmetric mixed response behavior.

## Resolved Issues

- None currently.
