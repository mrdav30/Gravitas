# CCD Exact TOI And Shape Reducers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the remaining conservative CCD reductions with exact fixed-point TOI solvers where they provide meaningful correctness or precision gains without unacceptable hot-path cost.

**Architecture:** Keep conservative bounds as the broad candidate stage, then add shape-specific exact reducers with deterministic tie-breakers, fallback policy, and benchmark attribution before expanding each family.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry primitives, Gravitas collision/query/narrow-phase code.

---

**Date:** 2026-06-21
**Status:** Pre-alpha release blocker
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
  target sphere backward against the source shape.
- 3D non-sphere targets, dynamic-vs-dynamic relative shape pairs, mixed
  dimension pairs, and mesh/compound dynamic relative pairs still use
  conservative proxy reduction.
- Angular CCD uses bounded deterministic pose samples, not closed-form or
  iterative exact angular TOI.

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

- [ ] Inventory every CCD path that still returns conservative proxy hits.
- [ ] Add benchmark attribution for candidate count, exact reducer attempts,
  accepted hits, and rejected false positives where current rows cannot explain
  cost.
- [ ] Rank exact reducer families by correctness gain, expected use frequency,
  and measured cost.
- [ ] Record unsupported pairs explicitly in collision pipeline docs.

## Workstream 2: Shape-Exact Angular TOI

**Problem**

Bounded angular pose sampling is deterministic and safe, but it can stop at the
previous safe sample rather than the exact angular time of impact.

**Tasks**

- [ ] Add red tests where angular sampling produces visibly earlier clamping
  than an exact 2D convex/AABB angular TOI should.
- [ ] Prototype exact 2D convex polygon and AABB angular TOI against circles
  and convex targets.
- [ ] Evaluate 3D cuboid and capsule angular TOI against spheres before
  expanding to other targets.
- [ ] Keep deterministic fallback to conservative sampling for unsupported
  pairs.

## Workstream 3: Exact 3D Static-Style Reducers

**Problem**

Current 3D exact reduction is strongest for sphere targets. Primitive
non-sphere targets can still cause conservative early stops.

**Tasks**

- [ ] Add false-positive and true-hit tests for cuboid, capsule, cylinder, and
  convex mesh target families.
- [ ] Prefer reusable fixed-point geometric primitives over bespoke math.
- [ ] Add cheap deterministic rejection before expensive exact work when
  benchmark attribution proves it helps.
- [ ] Preserve fallback behavior for concave mesh targets until mesh evidence
  justifies a more exact path.

## Workstream 4: Exact Dynamic Relative Reducers

**Problem**

Dynamic-vs-dynamic CCD currently uses sphere/circle proxy relative sweeps. This
is safe but can over-clamp elongated or compound dynamic shapes.

**Tasks**

- [ ] Add dynamic-vs-dynamic false-positive tests for elongated 2D and 3D shape
  pairs.
- [ ] Implement exact 2D relative reducers for convex/AABB pairs first, reusing
  existing swept SAT where possible.
- [ ] Evaluate 3D primitive relative reducers only after attribution shows the
  proxy false positives are worth the added cost.
- [ ] Preserve deterministic tie-breakers when two exact relative hits share
  the same TOI.

## Workstream 5: Mesh, Compound, And Mixed Reducer Policy

**Problem**

Mesh, compound, and mixed-dimension pairs can explode in candidate count if
exact reduction scans too much private geometry.

**Tasks**

- [ ] Add compound part-order TOI tests where multiple private parts hit at
  different times.
- [ ] Add mesh reducer benchmarks before any exact concave mesh relative TOI
  implementation.
- [ ] Decide whether convex mesh exact TOI belongs in runtime or offline
  authored convex decomposition.
- [ ] Add mixed-dimension reducer tests that preserve finite slab/prism
  embedding and plane-constrained normal orientation.

## Done Criteria

- Every supported exact reducer has red/green correctness tests and benchmark
  evidence.
- Unsupported shape families remain conservative and explicitly documented.
- Exact reducers improve false-positive behavior without introducing
  false-negative risk or hidden allocation.
- Mesh/compound expansion has a written policy backed by benchmark data.
