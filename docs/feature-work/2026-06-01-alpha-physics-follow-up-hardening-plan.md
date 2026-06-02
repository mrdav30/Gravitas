# Alpha Physics Follow-Up Hardening Plan

**Date:** 2026-06-01
**Status:** Draft
**Owner:** Gravitas runtime/collision hardening

## Purpose

Phase 10 established the alpha mixed 2D/3D physics path. This plan captures
follow-up hardening items discovered during that review without continuing to
expand the completed alpha hardening plan.

These are not compatibility tasks. If investigation proves the current shape is
wrong for deterministic accuracy, complexity, or allocations, prefer the clean
redesign with focused tests and benchmarks.

## Phase 1: Shared GridForge Traversal Helpers

**Goal:** Remove repeated GridForge voxel/partition scanning shape from pure 3D,
pure 2D, mixed collision, and query paths where a reusable helper would reduce
complexity without hiding physics semantics.

**Tasks**

- [ ] Inventory duplicated GridForge scan patterns in:
  - `GravitasCollisionService`
  - `GravitasCollision2DService`
  - `GravitasMixedCollisionService`
  - `GravitasQuery2DService`
  - `GravitasQuery3DService`
- [ ] Decide whether the extraction belongs in Gravitas support code or as a
  small reusable GridForge helper. Push the primitive into GridForge if
  Gravitas is hand-rolling generic grid traversal.
- [ ] Preserve deterministic voxel ordering, partition identity, and caller
  ownership of temporary buffers.
- [ ] Add regression tests for sparse, dense, edge, negative-coordinate, and
  retained-partition traversal cases.
- [ ] Add or update benchmarks before changing the hot paths, then compare the
  same benchmark selections after the extraction.

**Exit Criteria**

- Shared traversal logic is easier to audit than the duplicated code it
  replaces.
- Collision/query ordering remains deterministic in 2D, 3D, and mixed modes.
- Benchmarks show no meaningful regression in sparse or dense scenarios.

## Phase 2: Mixed Swept-Circle Precision

**Goal:** Revisit the current mixed 2D-circle vs 3D sweep policy where
`SweepCircleAgainst3D` uses `max(radius, halfThickness)` as a conservative
swept-sphere proxy.

**Context**

The current policy is deterministic, simple, and intentionally conservative. It
can over-report near tall slab corners because it is not a full swept
prism/capsule-like solver. The current tests pin this alpha behavior; do not
pretend it is physically exact.

**Tasks**

- [ ] Add targeted tests that demonstrate current over-report behavior at slab
  corners and tall thickness values.
- [ ] Design a deterministic swept-circle/slab or swept-prism solver that keeps
  stable ordering and explicit failure behavior.
- [ ] Compare the exact solver against the current swept-sphere proxy for:
  - correctness on corner/edge cases.
  - false-positive rate.
  - steady-state allocation.
  - sparse and dense query cost.
- [ ] Keep the proxy path only if it remains the better alpha tradeoff and is
  clearly documented as conservative.

**Exit Criteria**

- Mixed swept-circle behavior is either made more exact or the conservative
  proxy is retained with explicit tests, docs, and benchmark justification.

## Phase 3: Retained Partition Reset Semantics

**Goal:** Define whether context reset should detach retained empty partitions
from GridForge voxels or keep retained partition payloads available for reuse,
then apply the rule consistently across 3D, 2D, and mixed services.

**Tasks**

- [ ] Audit retained partition cleanup in:
  - `PhysicsPartition`
  - `PhysicsPartition2D`
  - `PhysicsMixedPartition`
  - `GravitasCollisionService`
  - `GravitasCollision2DService`
  - `GravitasMixedCollisionService`
- [ ] Decide the reset contract for long-running contexts, context reuse, and
  deterministic replay setup.
- [ ] If reset detaches retained partitions, ensure voxel payload removal is
  stable and does not break partition reuse after the next registration.
- [ ] If reset keeps retained partitions, document why this is intentional and
  ensure retained payloads cannot leak stale collider IDs, pair keys, or
  version state.
- [ ] Add tests for context reset after sparse, dense, and mixed partition
  usage.
- [ ] Benchmark reset plus re-registration churn before and after any change.

**Exit Criteria**

- Reset semantics are explicit and uniform for 3D, 2D, and mixed paths.
- No stale collider IDs, stale pair keys, or orphaned partition state survives
  reset.
- Long-running simulation cleanup behavior is documented and benchmarked.

