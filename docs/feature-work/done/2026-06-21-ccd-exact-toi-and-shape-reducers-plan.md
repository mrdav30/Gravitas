# CCD Exact TOI And Shape Reducers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the remaining conservative CCD reductions with exact fixed-point TOI solvers where they provide meaningful correctness or precision gains without unacceptable hot-path cost.

**Architecture:** Keep conservative bounds as the broad candidate stage, then add shape-specific exact reducers with deterministic tie-breakers, fallback policy, and benchmark attribution before expanding each family.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry primitives, Gravitas collision/query/narrow-phase code.

---

**Date:** 2026-06-21
**Status:** Done - completed 2026-06-23
**Owner:** Gravitas collision/query hardening

## Purpose

The completed CCD depth work intentionally kept several precision gaps out of
the runtime path until they had their own tests and benchmark signal. Remaining
conservative areas include shape-exact angular time-of-impact, exact 3D
reducers against non-sphere primitive targets, exact dynamic-vs-dynamic
relative-motion reducers, mixed-dimension shape reducers, and exact
mesh/compound relative TOI decisions.

These are not wiki footnotes for a first-class physics engine. They need an
explicit plan because each exact solver can improve physical quality while also
adding nontrivial math and runtime cost.

## Current Baseline

- 2D translational static-style CCD refines circle, AABB, convex polygon, and
  compound movers with exact translated-shape sweeps.
- 3D translational static-style CCD refines sphere targets by sweeping the
  target sphere backward against the source shape, and supported convex 3D
  source families now use support-mapped source sweeps against non-sphere
  static-style targets.
- Pure 2D and pure 3D dynamic relative CCD keep conservative proxy sweeps as the
  broad candidate stage, then reject false positives with exact mover-shape
  validation where the source family is supported.
- Angular CCD uses bounded deterministic samples to bracket the first hit, then
  refines the accepted time of impact with fixed-iteration bisection over the
  exact narrow phase.
- Mixed dynamic CCD remains conservative and is owned by the service-level CCD
  island plan because exact mixed velocity handoff requires advancing
  cross-dimension participants together.

## Guiding Rules

- Conservative broad candidates remain the no-false-negative safety net.
- Exact reducers must never introduce tunneling for unsupported or degenerate
  shape pairs.
- Each promoted reducer needs a false-positive rejection test and a true-hit
  no-tunneling test.
- Stable ordering is distance/TOI first, then closing speed where applicable,
  then collider ID or stable part/triangle identity.
- Mesh and compound reducers must preserve authored owner identity and stable
  private part/triangle order.
- Runtime reducers must be benchmarked before and after promotion.

## Workstream 1: Exact Reducer Inventory And Benchmark Attribution

**Tasks**

- [x] Inventory every CCD path that still returns conservative proxy hits.
- [x] Add benchmark attribution for candidate count, exact reducer attempts,
  accepted hits, and rejected false positives where current rows cannot explain
  cost.
- [x] Rank exact reducer families by correctness gain, expected use frequency,
  and measured cost.
- [x] Record unsupported pairs explicitly in collision pipeline docs.

**Result:** Public query reducers already supplied the strongest reusable 3D
convex-source shape casts, so CCD now reuses them after proxy gathering instead
of adding separate static or dynamic sweep math. Mixed dynamic CCD remained out
of reducer scope and stays in the service-level island plan.

## Workstream 2: Shape-Exact Angular TOI

**Problem**

Bounded angular pose sampling is deterministic and safe, but it can stop at the
previous safe sample rather than the exact angular time of impact.

**Tasks**

- [x] Add red tests where angular sampling produces visibly earlier clamping
  than an exact 2D convex/AABB angular TOI should.
- [x] Prototype exact 2D convex polygon and AABB angular TOI against circles
  and convex targets.
- [x] Evaluate 3D cuboid and capsule angular TOI against spheres before
  expanding to other targets.
- [x] Keep deterministic fallback to conservative sampling for unsupported
  pairs.

**Result:** The runtime did not adopt separate closed-form angular solvers.
Instead, it now brackets contacts with deterministic samples and refines the
first hit with fixed-iteration bisection over the exact narrow phase. This gives
the practical precision gain while preserving one narrow-phase source of truth.

## Workstream 3: Exact 3D Static-Style Reducers

**Problem**

Current 3D exact reduction is strongest for sphere targets. Primitive
non-sphere targets can still cause conservative early stops.

**Tasks**

- [x] Add false-positive and true-hit tests for cuboid, capsule, cylinder, and
  convex mesh target families.
- [x] Prefer reusable fixed-point geometric primitives over bespoke math.
- [x] Add cheap deterministic rejection before expensive exact work when
  benchmark attribution proves it helps.
- [x] Preserve fallback behavior for concave mesh targets until mesh evidence
  justifies a more exact path.

**Result:** Supported convex 3D sources reuse `ConvexSweepQueryWorker`,
including convex mesh and supported compound sources. Concave mesh targets are
handled by bounded triangle candidates; concave mesh sources are intentionally
unsupported and should be authored as convex compound parts.

## Workstream 4: Exact Dynamic Relative Reducers

**Problem**

Dynamic-vs-dynamic CCD currently uses sphere/circle proxy relative sweeps. This
is safe but can over-clamp elongated or compound dynamic shapes.

**Tasks**

- [x] Add dynamic-vs-dynamic false-positive tests for elongated 2D and 3D shape
  pairs.
- [x] Implement exact 2D relative reducers for convex/AABB pairs first, reusing
  existing swept SAT where possible.
- [x] Evaluate 3D primitive relative reducers only after attribution shows the
  proxy false positives are worth the added cost.
- [x] Preserve deterministic tie-breakers when two exact relative hits share
  the same TOI.

**Result:** Pure 2D dynamic relative CCD now validates proxy candidates through
`QueryDetection2D.TrySweepMoverShape`. Pure 3D dynamic relative CCD now supports
source spheres, target spheres, convex primitive sources, convex mesh sources,
and supported compound sources against dynamic targets. Unsupported source
families keep conservative proxy behavior.

## Workstream 5: Mesh, Compound, And Mixed Reducer Policy

**Problem**

Mesh, compound, and mixed-dimension pairs can explode in candidate count if
exact reduction scans too much private geometry.

**Tasks**

- [x] Add compound part-order TOI tests where multiple private parts hit at
  different times.
- [x] Add mesh reducer benchmarks before any exact concave mesh relative TOI
  implementation.
- [x] Decide whether convex mesh exact TOI belongs in runtime or offline
  authored convex decomposition.
- [x] Add mixed-dimension reducer tests that preserve finite slab/prism
  embedding and plane-constrained normal orientation.

**Result:** Convex mesh and supported compound source casts belong in runtime
because they share the public exact query worker. Concave mesh source casts do
not belong in runtime; hosts should use offline convex decomposition into
stable `LSCompoundCollider` parts. Mixed public/static reducers are exact for
supported slab families, while mixed dynamic CCD remains conservative until the
service-level island solver owns cross-dimension advancement and velocity
handoff.

## Completion Notes

- Added red/green coverage for 3D static false-positive rejection, 2D dynamic
  relative false-positive rejection, 3D dynamic relative false-positive
  rejection, and angular refinement in pure 2D and 3D.
- Added benchmark rows for static shape-exact false positives, dynamic 2D
  relative shape-exact false positives, and dynamic 3D relative shape-exact
  false positives.
- Updated `COLLISION_PIPELINE.md`, `QUERY_SERVICES.md`,
  `RUNTIME_ARCHITECTURE.md`, benchmark docs, and the service-level CCD island
  plan baseline.
- Recorded the ambiguous 3D dynamic shape-exact BenchmarkDotNet allocation
  signal in `benchmark-signal-hardening-backlog.md`; the focused xUnit
  allocation guard for the reducer path passes with zero managed allocation.
- No new deferred reducer work remains in this plan. The remaining alpha CCD
  work is service-level island solving for chained/same-TOI groups and mixed
  dynamic velocity handoff.

## Done Criteria

- Every supported exact reducer has red/green correctness tests and benchmark
  evidence.
- Unsupported shape families remain conservative and explicitly documented.
- Exact reducers improve false-positive behavior without introducing
  false-negative risk or hidden allocation.
- Mesh/compound expansion has a written policy backed by benchmark data.
