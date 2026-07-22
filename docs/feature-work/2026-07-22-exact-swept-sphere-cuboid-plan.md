# Exact Swept-Sphere Cuboid Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:test-driven-development` while implementing each behavior and
> `superpowers:verification-before-completion` before reporting completion.

**Goal:** Replace Gravitas's sharp expanded-cuboid proxy with the exact first
intersection between a segment and the spherical dilation of a box.

**Architecture:** FixedMathSharp will own one reusable, public
`FixedSegment.TryGetSweptSphereBoxIntersectionDistance(...)` query. Its internal
solver will walk the at-most-six exact box-plane crossings of the segment and
solve the point-to-box squared-distance quadratic on each resulting interval
with existing wide rational/root machinery. Gravitas will only transform the
authored endpoints into cuboid-local space, delegate, and reconstruct the world
hit from the original segment.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp fixed-width wide
arithmetic, xUnit v3, BenchmarkDotNet.

## Global Constraints

- Keep FixedMathSharp engine agnostic and expose no raw wide arithmetic.
- Add only the first-distance API required by current callers; no speculative
  interval overload, OBB type, generic convex interface, or compatibility shim.
- Preserve exact closed-boundary contact, round-half-to-even distance output,
  deterministic ordering, `netstandard2.1`, and `net8.0` support.
- Keep the warmed query path allocation-free and constant-time.
- Leave all changes unstaged and uncommitted for user review.

---

### Task 1: Capture Current Behavior And Performance

**Files:**
- Modify: `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Queries/RadialRaycastBenchmarks.cs`

**Interfaces:**
- Consumes: `SweptSphereQueryWorker.TrySweep(...)`.
- Produces: failing edge/corner regressions and a representative rounded-cuboid
  benchmark with warmed allocation coverage.

- [x] Add the ordinary edge false-positive regression from the issue record and
  assert `false` plus default outputs.
- [x] Run the focused Gravitas test and confirm it fails because the sharp proxy
  reports a hit.
- [x] Add edge/corner hit fixtures to `RadialRaycastBenchmarks`, validate the
  fixture during setup, and capture the current proxy baseline before changing
  production code.

### Task 2: Add The Minimal FixedMathSharp Contract

**Files:**
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedSegment.Distance.cs`
- Create: `src/FixedMathSharp/Geometry/Wide/WideFiniteAxisIntersection.RoundedBox.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedSegment.RoundedBox.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/FiniteAxisIntersectionBenchmarks.cs`

**Interfaces:**
- Consumes: `FixedBoundBox`, existing exact rational bounds, bounded wide
  quadratic solving, and physical-distance rounding.
- Produces:

```csharp
public readonly bool TryGetSweptSphereBoxIntersectionDistance(
    FixedBoundBox box,
    Fixed64 sphericalExpansion,
    Fixed64 totalDistance,
    out Fixed64 distance)
```

- [x] Add tests that express the desired API and cover face entry, edge and
  corner miss/hit/tangency, starting overlap, endpoint-only contact, zero-radius
  box intersection, validation, and extreme coordinates.
- [x] Run the focused tests and confirm they fail because the API is absent.
- [x] Implement the smallest solver: sort/group at most six rational plane
  crossings, derive the active outside-axis quadratic on each interval, reuse
  existing exact bounded-root and distance-rounding code, and stop at the first
  closed interval hit.
- [x] Run focused tests, both target frameworks, and the focused benchmark.

### Task 3: Delegate Gravitas Cuboid Sweeps

**Files:**
- Modify: `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- Modify: `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`

**Interfaces:**
- Consumes: the FixedMathSharp API from Task 2.
- Produces: exact sphere-center first distance for cuboids and cuboid compound
  parts through the existing worker dispatch.

- [x] Transform both authored endpoints into cuboid-local coordinates and call
  `TryGetSweptSphereBoxIntersectionDistance`; do not retain `TrySweepLocalBox`.
- [x] Reconstruct the hit through the original world `FixedSegment` so small
  representable components are not lost through normalized direction math.
- [x] Add ordinary, rotated, compound, edge/corner, exact-entry, extreme atomic
  rejection, static CCD, and warmed allocation regressions around the shared
  worker and its public consumers.
- [x] Run focused query, raycast, grounding, and CCD tests until green.

### Task 4: Close Evidence And Documentation

**Files:**
- Modify: `F:/gamedevrepos/FixedMathSharp/docs/wiki/bounds-and-geometry.md`
- Modify: `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`
- Modify: `docs/feature-work/issue-tracker.md`
- Modify: `docs/feature-work/2026-07-22-exact-swept-sphere-cuboid-plan.md`

**Interfaces:**
- Consumes: verified source, tests, coverage, and benchmark output.
- Produces: accurate public API documentation and resolved issue history.

- [x] Document the typed rounded-box query without exposing implementation-only
  wide types.
- [x] Re-run FixedMathSharp `Release`, `ReleaseLean`, coverage, and relevant
  benchmarks; require 100% line and branch coverage for hand-authored source.
- [x] Re-run locally linked Gravitas `Release`, `ReleaseLean`, coverage, replay,
  warmed allocation, and relevant benchmarks; require 100% line and branch
  coverage for hand-authored source.
- [x] Compare the same benchmark before/after without claiming an improvement
  unless the measurements demonstrate one.
- [x] Request an independent final code review, resolve every material finding,
  move the issue to resolved history, and record the verification evidence here.

## Current Status

- [x] Root cause and caller/parity audits complete.
- [x] Direct piecewise solver design approved with a strict no-bloat caveat.
- [x] Tasks 1-3 complete.
- [x] Task 4 complete. The independent review found no material issue, and both
  locally linked Gravitas configurations pass their full solution gates.

## Evidence To Date

- FixedMathSharp Release: `1,753/1,753` core and `8/8` Chronicler tests.
- FixedMathSharp ReleaseLean: `1,732/1,732` core and `8/8` Chronicler tests.
- FixedMathSharp coverage excluding generated serializers: `14,421/14,421`
  lines, `4,493/4,493` branches, and `2,001/2,001` methods.
- Gravitas affected query/raycast/grounding/CCD slice: `438/438`; the independent
  reviewer reran the final query/raycast/CCD slice at `432/432` with no material
  findings.
- Gravitas local-linked Release: `3,252/3,252` tests, including the warmed
  allocation gates.
- Gravitas local-linked ReleaseLean: `3,197/3,197` tests.
- Gravitas coverage excluding generated serializers: `33,706/33,706` lines,
  `12,129/12,129` branches, and `4,192/4,192` methods.
- FixedMathSharp exact rounded-box benchmark: `6.635 us` at scale 1 and
  `8.558 us` at scale 100,000, both zero allocation.
- Gravitas exact rounded-cuboid worker: `7.595 us` at scale 1 and `9.511 us` at
  scale 100,000, both zero allocation. The old sharp proxy measured `628.0 ns`
  and `664.8 ns`; the existing exact capsule worker measured `11.479 us` and
  `12.980 us` at the same scales.
