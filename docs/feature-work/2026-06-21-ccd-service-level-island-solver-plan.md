# CCD Service-Level Island Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move beyond body-owned bounded CCD substeps where needed by adding a deterministic service-level TOI island model for chained, simultaneous, and mixed-dimension continuous contacts.

**Architecture:** Keep the current body-owned bounded solver as the simple path, then introduce context-service CCD island processing for cases that need multiple bodies advanced and resolved together at the same time of impact.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp, SwiftCollections, GridForge, Gravitas physics/collision services.

---

**Date:** 2026-06-21
**Status:** Pre-alpha release blocker
**Owner:** Gravitas physics-service hardening

## Purpose

Workstream 4 of the CCD depth plan upgraded `Continuous` and `Auto` to consume
bounded same-frame TOI substeps. That closes the worst first-hit clamp behavior
for a single moving body, but it is still body-owned. Dense scenes with chained
fast bodies, same-TOI groups, kinematic active sources, and mixed-dimensional
responses may require a service-level island model so multiple participants are
advanced, resolved, and continued under one deterministic ordering contract.

## Current Baseline

- Each body resolves its own continuous hits during `LateSimulate`.
- Dynamic target prediction is segment-aware for body-owned substeps.
- Same-frame sliding through multiple static contacts is supported within a
  bounded per-body substep budget.
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

- [ ] Add tests showing body-owned substeps are insufficient for a chained
  dynamic scenario.
- [ ] Add tests for same-TOI candidate groups that must resolve in stable
  collider-ID or pair-key order.
- [ ] Define when the service-level path engages versus staying on the simpler
  body-owned path.
- [ ] Design fixed-capacity or pooled island buffers with explicit ownership and
  reset behavior.

## Workstream 2: Pure 2D And Pure 3D Service-Level Islands

**Tasks**

- [ ] Collect CCD candidates for all eligible bodies before any participant
  commits its frame end pose.
- [ ] Find earliest TOI per island or connected candidate group.
- [ ] Advance affected bodies to TOI under a stable ordering rule.
- [ ] Resolve or clamp continuous contacts, then continue through bounded
  remaining frame time.
- [ ] Add deterministic cap diagnostics equivalent to body-owned substep
  diagnostics.

## Workstream 3: Sleep, Wake, Kinematic, And Immovable Semantics

**Tasks**

- [ ] Specify how sleeping bodies join or ignore CCD islands.
- [ ] Preserve existing wake stimuli unless tests prove an island-specific wake
  rule is required.
- [ ] Treat immovable and kinematic participants as infinite mass in response,
  including active-source velocity handoff from kinematic movers to dynamic
  targets once all island participants advance at shared TOI.
- [ ] Add replay tests proving repeated runs produce identical island outcomes.

## Workstream 4: Mixed-Dimension Island Response

**Problem**

Mixed CCD has constrained impulse rules that cannot be copied blindly from pure
3D or pure 2D islands.

**Tasks**

- [ ] Add mixed chain tests involving 3D and embedded 2D dynamic participants.
- [ ] Preserve plane-constrained 2D linear and angular response.
- [ ] Define vertical Y impulse/correction behavior at island TOI.
- [ ] Keep `PhysicsRuntimeMode.Both` isolated from mixed island work.

## Workstream 5: Benchmarks, Diagnostics, And Docs

**Tasks**

- [ ] Benchmark sparse no-hit overhead, dense many-hit overhead, and chained
  island scenes.
- [ ] Add deterministic diagnostics for island count, TOI iterations, cap hits,
  and participants if host-visible counters are accepted.
- [ ] Update collision pipeline, runtime architecture, and diagnostics docs.
- [ ] Validate full `Release`, full `ReleaseLean`, and benchmark smoke rows.

## Done Criteria

- Chained and simultaneous CCD cases resolve deterministically without depending
  on per-body iteration order.
- Pure 2D and pure 3D island behavior has focused correctness tests and
  benchmark evidence.
- Mixed island behavior is implemented or explicitly blocked by a documented
  response-model decision.
- Island caps are deterministic and visible through tests/diagnostics.
