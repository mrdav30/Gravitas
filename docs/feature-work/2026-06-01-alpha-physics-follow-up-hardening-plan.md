# Alpha Physics Follow-Up Hardening Plan

**Date:** 2026-06-01
**Status:** Draft
**Owner:** Gravitas runtime/collision hardening

## Purpose

The alpha hardening plan established the mixed 2D/3D physics path, serialization
contract, and diagnostic stream. This plan captures follow-up hardening items
discovered during that review without continuing to expand the completed alpha
hardening plan.

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

## Phase 4: Mesh Decomposition And Closed-Volume Policy

**Goal:** Revisit host/offline decomposed convex-piece support and any
Gravitas-owned deterministic convex decomposition only if evidence shows the
raw local-BVH triangle path is not enough for alpha-scale concave mesh
collision, closed-volume mass/inertia work, or contact-heavy scenes.

**Context**

Phase 7B made `MeshColliderMode.Concave` work through raw triangle-set
narrow-phase using local-BVH candidate gathering. That path is the alpha
baseline. Decomposed convex pieces are not required for current concave mesh
collision, and they must not leak internal collider identities or masquerade as
`LSCompoundCollider` parts.

**Tasks**

- [ ] Build comparison fixtures for raw triangle-BVH concave collision versus
  decomposed convex pieces across:
  - dense concave meshes.
  - dynamic concave bodies.
  - contact-heavy inside corners and U-channels.
  - closed-volume inertia and mass scenarios.
- [ ] Evaluate host/offline decomposed convex-piece support as an optional
  `LSMeshCollider` data path only if benchmarks or solver-quality tests justify
  it. The owning mesh must still expose one collider ID, one body binding, one
  event surface, and one broad-phase identity.
- [ ] Evaluate Gravitas-owned deterministic convex decomposition as explicit
  preprocessing R&D only if Gravitas needs an engine-agnostic asset-prep path.
- [ ] If decomposition is attempted, require deterministic ordering,
  deterministic tie-breakers, bounded failure/result codes, pathological mesh
  tests, and benchmarks against the raw local-BVH triangle path.
- [ ] Document whether decomposition improves collision quality, inertia
  quality, query cost, or merely adds complexity.

**Exit Criteria**

- Raw triangle-BVH remains the documented baseline unless decomposition has
  measurable correctness or complexity value.
- Any decomposition path preserves single-collider external identity.
- No runtime implicit decomposition mutates mesh collision truth behind the
  developer's back.

## Phase 5: Dynamic CCD And Swept Mesh Families

**Goal:** Define the next continuous-collision slice beyond the current static
or kinematic target clipping so fast dynamic bodies, mesh targets, and mixed
queries have physically explainable deterministic policy.

**Context**

Current CCD support is opt-in/auto and intentionally bounded. 3D and 2D body
movement can use swept primitive proxies against static or kinematic targets,
and mixed sweeps include alpha mesh/compound support. Ordinary dynamic-vs-
dynamic CCD, full swept mesh query families, and richer relative-velocity
ordering remain future hardening.

**Tasks**

- [ ] Specify deterministic dynamic-vs-dynamic CCD ordering for 3D, pure 2D,
  and mixed contact paths.
- [ ] Define how relative velocity, pair priority, body IDs, hierarchy keys,
  and contact normals break ties.
- [ ] Add fixtures for tunneling dynamic bodies, opposing high-speed bodies,
  thin static geometry, and mixed 2D slab interactions.
- [ ] Investigate shape-specific swept mesh behavior before adding public APIs:
  ray/segment vs mesh, swept sphere/circle vs mesh, and mesh-as-moving-source.
- [ ] Benchmark CCD candidate gathering, clip resolution, and false-positive
  rates before replacing any current conservative proxy.

**Exit Criteria**

- CCD behavior remains explicit and opt-in/auto, not a silent global cost.
- Dynamic-vs-dynamic CCD has deterministic tie-breakers and tests before it is
  enabled.
- Swept mesh APIs are added only with allocation tests and benchmark evidence.

## Phase 6: Typed Diagnostic Views

**Goal:** Keep `GravitasDiagnosticEvent` compact while reducing host adapter
mistakes if generic fields become difficult to decode.

**Context**

Phase 12 kept the alpha diagnostic event stream generic. `ScalarA`, `ScalarB`,
`DataA`, and `DataB` are sufficient while every event kind has documented field
meaning and adapters decode by `GravitasDiagnosticEventKind`. Typed views are a
tooling convenience, not a reason to bloat the capture hot path.

**Tasks**

- [ ] Inventory repeated event-decoding switch logic in host adapters, samples,
  or future tooling.
- [ ] If repetition becomes error-prone, design typed read-only view helpers
  over existing `GravitasDiagnosticEvent` payloads without changing capture
  storage.
- [ ] Add tests for each typed view's field mapping, including mixed-dimension
  payloads.
- [ ] Keep helpers outside authoritative runtime loops and benchmark any
  observable/tooling projection that fans diagnostics out to subscribers.

**Exit Criteria**

- Generic diagnostic capture remains compact and allocation-conscious.
- Host adapters can decode events without ambiguous field meanings.
- Any typed helpers are proven by tests and do not alter deterministic event
  ordering.
