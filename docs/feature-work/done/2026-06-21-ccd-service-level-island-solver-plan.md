# CCD Service-Level Island Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move beyond body-owned bounded CCD TOI iterations where needed by adding a deterministic service-level TOI handoff model for chained and mixed-dimension continuous contacts.

**Architecture:** Keep the current body-owned bounded solver as the simple path, then introduce context-service CCD handoff queues for cases where a dynamic target has already passed its service turn or belongs to another dimensional service.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp, SwiftCollections, GridForge, Gravitas physics/collision services.

---

**Date:** 2026-06-21
**Status:** Done
**Owner:** Gravitas physics-service hardening

## Purpose

Workstream 4 of the CCD depth plan upgraded `Continuous` and `Auto` to consume
bounded same-frame TOI iterations. That closes the worst first-hit clamp behavior
for a single moving body, but it is still body-owned. Dense scenes with chained
fast bodies, same-TOI groups, kinematic active sources, and mixed-dimensional
responses may require a service-level island model so multiple participants are
advanced, resolved, and continued under one deterministic ordering contract.

## Current Baseline

- Each body resolves its own continuous hits during `LateSimulate`.
- Dynamic target prediction is segment-aware for body-owned substeps.
- Supported pure 2D and pure 3D dynamic candidates use exact relative
  mover-shape reducers after conservative proxy candidate gathering.
- Same-frame sliding through multiple static contacts is supported within a
  bounded per-body TOI iteration budget.
- Mixed candidates can be compared, but mixed-specific island response is not
  modeled.
- Kinematic active sources wake and position-correct dynamic targets during the
  body-owned pass; velocity handoff remains an island concern so targets are
  not integrated twice when source and target service phases differ.
- Ordinary discrete pair response remains the resting-contact and manifold
  authority after CCD clamps/velocity changes.

## Guiding Rules

- Do not replace the body-owned bounded solver unless a service-level path is
  measurably needed.
- Island building must be deterministic: stable body/collider IDs, stable pair
  keys, stable contact ordering, and explicit tie-breakers for equal TOI.
- Cost must be bounded per frame with explicit diagnostics when the cap is hit.
- Sleeping and wake behavior must stay deterministic and match existing wake
  reasons unless this plan deliberately changes them.
- Mixed islands must preserve the finite 2D slab embedding and vertical
  constraint rules.
- No hidden allocations in recurring island solve paths after warmup.

## Workstream 1: Island Trigger Criteria And Data Model

**Tasks**

- [x] Add tests showing body-owned TOI iterations are insufficient for a chained
  dynamic scenario.
- [x] Add tests for same-TOI candidate groups that must resolve in stable
  collider-ID or pair-key order.
- [x] Define when the service-level path engages versus staying on the simpler
  body-owned path.
- [x] Design fixed-capacity or pooled island buffers with explicit ownership and
  reset behavior.

## Workstream 2: Pure 2D And Pure 3D Service-Level Islands

**Tasks**

- [x] Collect CCD candidates for all eligible bodies before any participant
  commits its frame end pose.
- [x] Find earliest TOI per island or connected candidate group.
- [x] Advance affected bodies to TOI under a stable ordering rule.
- [x] Resolve or clamp continuous contacts, then continue through bounded
  remaining frame time.
- [x] Add deterministic cap diagnostics equivalent to body-owned TOI iteration
  diagnostics.

## Workstream 3: Sleep, Wake, Kinematic, And Immovable Semantics

**Tasks**

- [x] Specify how sleeping bodies join or ignore CCD islands.
- [x] Preserve existing wake stimuli unless tests prove an island-specific wake
  rule is required.
- [x] Treat immovable and kinematic participants as infinite mass in response,
  including active-source velocity handoff from kinematic movers to dynamic
  targets once all island participants advance at shared TOI.
- [x] Add replay tests proving repeated runs produce identical island outcomes.

## Workstream 4: Mixed-Dimension Island Response

**Problem**

Mixed CCD has constrained impulse rules that cannot be copied blindly from pure
3D or pure 2D islands.

**Tasks**

- [x] Add mixed chain tests involving 3D and embedded 2D dynamic participants.
- [x] Preserve plane-constrained 2D linear and angular response.
- [x] Define vertical Y impulse/correction behavior at island TOI.
- [x] Keep `PhysicsRuntimeMode.Both` isolated from mixed island work.

## Workstream 5: Benchmarks, Diagnostics, And Docs

**Tasks**

- [x] Benchmark sparse no-hit overhead, dense many-hit overhead, and chained
  island scenes.
- [x] Add deterministic diagnostics for island count, TOI iterations, cap hits,
  and participants if host-visible counters are accepted.
- [x] Update collision pipeline, runtime architecture, and diagnostics docs.
- [x] Validate full `Release`, full `ReleaseLean`, and benchmark smoke rows.

## Done Criteria

- Chained and simultaneous CCD cases resolve deterministically without depending
  on per-body iteration order.
- Pure 2D and pure 3D island behavior has focused correctness tests and
  benchmark evidence.
- Mixed island behavior is implemented or explicitly blocked by a documented
  response-model decision.
- Island caps are deterministic and visible through tests/diagnostics.

## Completion Notes

- Implemented service-owned processed-body handoff queues in
  `GravitasPhysicsService` and `GravitasPhysics2DService` instead of a
  heavyweight persistent island graph. A body that has already completed its
  service turn is queued and re-entered with bounded remaining-time CCD; a body
  that has not run yet consumes its pending handoff at the start of its own
  `LateSimulate`.
- Renamed the public bounded CCD cap from
  `ContinuousCollisionMaxSubsteps` to
  `ContinuousCollisionMaxToiIterations` because the cap now covers repeated TOI
  consumption and service handoff continuation, not only geometric substeps.
- Added pure 3D, pure 2D, and mixed dynamic chain tests, queue-cap tests, and
  kinematic active-source handoff tests. Kinematic source handoffs ignore the
  initiating source collider during the target continuation segment so a target
  does not collide with the same final-pose kinematic source twice.
- Added service counters for handoff batch count, TOI iteration count, and cap
  hits, plus body counters for bounded TOI iteration status.
- Added chained CCD rows to the benchmark project under
  `continuous-collision-toi-iteration` and kept the older static two-contact
  signal under the renamed TOI terminology.
- Split oversized body source files into focused partials for continuous
  collision, motion, grounding, and serialization while keeping authoritative
  state ownership on `SolidBody` and `SolidBody2D`.

No deferred work was left in this plan. Broader future CCD/query/response work
should live in the existing feature-work overview, benchmark signal backlog, or
issue tracker rather than this completed plan.
