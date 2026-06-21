# Discrete Response And Contact Quality Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade Gravitas discrete contact response from alpha-solid behavior to a first-class deterministic solver for resting stacks, cached impulse application, contact islands, cylinder contacts, mesh clipping, and mixed-dimension response.

**Architecture:** Keep the current narrow-phase/contact-pair ownership model, then harden the solver around explicit contact identity, stable island ordering, and measurable response quality. CCD-specific island time stepping remains in the CCD service-level island plan; this plan owns ordinary discrete/resting response after contacts exist.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp, SwiftCollections, Gravitas collision detection/response services.

---

**Date:** 2026-06-21
**Status:** Pre-alpha release blocker
**Owner:** Gravitas response and contact hardening

## Purpose

The current response layer is much stronger than the early prototype: 3D and
pure 2D contacts have deterministic manifolds, friction impulses, material
coefficients, and pair-local warm-start storage. Pure 2D now applies cached
normal and tangent impulses before the fresh solve. 3D still records cached
impulses without applying them as a true warm-started solve, resting-stack
static friction is still shallow, discrete island solving is not explicit, and
mixed response is intentionally constrained.

Those are not acceptable as loose wiki caveats for a first-class deterministic
physics engine. They should be driven by tests, benchmark signal, and explicit
solver invariants.

## Relationship To Existing Plans

- [`2026-06-21-ccd-service-level-island-solver-plan.md`](2026-06-21-ccd-service-level-island-solver-plan.md)
  owns continuous-collision TOI islands and same-frame advancement.
- This plan owns ordinary discrete contact islands, resting response,
  warm-start application, and island-wide sleep/wake after CCD has produced or
  handed off contact state.
- [`2026-06-21-query-and-mixed-swept-shape-hardening-plan.md`](2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  owns public query and sweep exactness. This plan consumes the resulting
  contacts and improves response quality.

## Current Baseline

- 3D contacts store pair-local cached normal and tangent impulses by contact
  identity.
- 3D response applies fresh normal/friction impulses, but does not yet apply
  cached impulses as a true warm-started iterative solve.
- Pure 2D response applies pair-local cached impulses and supports two-contact
  manifolds.
- Discrete pair response is flat over active pairs rather than island-built.
- Cylinder collision/query support exists, but docs still call out edge-case
  hardening for the finite-cylinder model.
- Mesh narrow phase supports triangle-level contacts, but richer mesh contact
  clipping remains future work.
- Mixed response applies planar X/Z impulse and angular yaw impulse to 2D
  bodies while constraining vertical response to the 3D participant.

## Guiding Rules

- Contact ordering, island ordering, warm-start application, and sleep/wake
  propagation must be deterministic and testable.
- The solver should fail visibly through diagnostics or iteration caps rather
  than hiding instability behind silent clamps.
- Static and dynamic friction should remain physically explainable Coulomb-style
  behavior with explicit thresholds.
- 3D warm-starting must not inject energy when cached contacts are stale or
  reordered.
- Cylinder and mesh contact improvements must preserve finite-cylinder and
  triangle identity semantics.
- No recurring hot-path allocation after warmup.

## Workstream 1: Solver Invariant Inventory And Evidence

**Tasks**

- [ ] Inventory current 3D, 2D, and mixed response paths, including cached
  impulse storage, tangent basis selection, contact IDs, pair ordering, and
  sleep/wake hooks.
- [ ] Add or update tests that expose the current limitations: resting-stack
  drift, stale 3D cached impulses, same-island wake propagation, cylinder
  edge-touching, and mesh clipping ambiguity.
- [ ] Add benchmark rows only where runtime cost is expected to change:
  dense resting contacts, cylinder-heavy scenes, mesh-contact scenes, and mixed
  response scenes.
- [ ] Document the baseline before changing solver behavior.

## Workstream 2: 3D Warm-Start And Resting Friction

**Problem**

3D response stores pair-local cached impulses, but the cached values are not yet
applied as the first step of an iterative solve. Resting friction is currently
dynamic-friction oriented and does not fully stabilize resting stacks.

**Tasks**

- [ ] Add tests where a 3D resting stack settles faster or avoids jitter only
  when cached impulses are applied safely.
- [ ] Add tests where stale cached impulses unwind instead of injecting energy
  after contact normals or IDs change.
- [ ] Apply cached normal and tangent impulses before the fresh 3D solve,
  following the pure 2D accumulated-impulse rules where they fit 3D.
- [ ] Introduce explicit static-friction behavior for near-resting tangential
  motion, with deterministic thresholds and material combination rules.
- [ ] Re-run existing restitution, friction, and replay tests after each solver
  change.

## Workstream 3: Deterministic Discrete Island Model

**Problem**

Flat pair iteration is predictable, but it does not model connected contact
islands, island-wide wake, or multi-iteration stabilization.

**Tasks**

- [ ] Add tests for connected bodies where solving pairs independently produces
  order-sensitive drift or incomplete wake propagation.
- [ ] Introduce an island builder keyed by stable body/collider IDs and pair
  keys, with explicit ordering for bodies, contacts, and constraints.
- [ ] Add a bounded multi-iteration solve path for islands that need
  stabilization.
- [ ] Connect island-wide wake/sleep to existing wake reasons without changing
  pure query or trigger-only behavior.
- [ ] Keep single-pair scenes on a low-overhead path when island solving is not
  needed.

## Workstream 4: Cylinder And Mesh Contact Geometry

**Problem**

Finite-cylinder support exists, but cylinder edge cases and mesh clipping still
need hardening so contacts are stable, physically plausible, and deterministic
across edge/cap/side transitions.

**Tasks**

- [ ] Add cylinder edge-case tests for sphere, capsule, cuboid, cylinder, mesh,
  and mixed embedded-2D slab contacts.
- [ ] Add mesh contact clipping tests where a triangle-level hit should produce
  a stable contact surface rather than a noisy point or ambiguous normal.
- [ ] Harden finite-cylinder cap, side, and rim contact selection without
  treating cylinders as capsules.
- [ ] Preserve mesh triangle candidate ordering and authored collider identity
  when multiple clipped contacts reduce to an external contact surface.
- [ ] Add benchmarks for cylinder-heavy and mesh-contact-heavy scenes before
  expanding clipping complexity.

## Workstream 5: Mixed-Dimension Response Quality

**Problem**

Mixed response correctly constrains vertical 2D motion, but richer mixed solver
behavior remains future hardening.

**Tasks**

- [ ] Add tests for mixed resting contacts where planar impulse, yaw torque,
  vertical constraint, and 3D participant motion interact.
- [ ] Decide whether mixed response should participate in discrete islands with
  pure 3D contacts, pure 2D contacts, or a dedicated mixed island type.
- [ ] Preserve `PhysicsRuntimeMode.Both` isolation from mixed response work.
- [ ] Add diagnostics for mixed response iteration count and cap hits if mixed
  islands become iterative.
- [ ] Document unsupported mixed solver behavior explicitly instead of leaving
  it as an implementation accident.

## Done Criteria

- 3D cached impulses are either applied safely or documented with evidence for
  why they remain storage-only.
- Resting friction behavior has correctness tests, replay coverage, and clear
  thresholds.
- Discrete islands have stable ordering, bounded iteration, and wake/sleep
  tests.
- Cylinder and mesh contact edge cases have regression coverage and benchmark
  signal where hot-path cost changes.
- Mixed response limitations are resolved or documented as explicit policy.
