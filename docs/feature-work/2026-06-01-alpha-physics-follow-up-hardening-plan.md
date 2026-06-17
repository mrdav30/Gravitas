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

- [x] Inventory duplicated GridForge scan patterns in:
  - `GravitasCollisionService`
  - `GravitasCollision2DService`
  - `GravitasMixedCollisionService`
  - `GravitasQuery2DService`
  - `GravitasQuery3DService`
- [x] Decide whether the extraction belongs in Gravitas support code or as a
  small reusable GridForge helper. Push the primitive into GridForge if
  Gravitas is hand-rolling generic grid traversal.
- [x] Preserve deterministic voxel ordering, partition identity, and caller
  ownership of temporary buffers.
- [x] Add regression tests for sparse, dense, edge, negative-coordinate, and
  retained-partition traversal cases.
- [x] Capture a post-migration benchmark baseline for the hot paths so future
  changes can compare the same benchmark selections.

**Exit Criteria**

- Shared traversal logic is easier to audit than the duplicated code it
  replaces.
- Collision/query ordering remains deterministic in 2D, 3D, and mixed modes.
- Benchmarks show no meaningful regression in sparse or dense scenarios.

**Progress - 2026-06-17**

Implemented a Gravitas-local `GridForgeTraversalState` and `GridForgeTraversal`
helper for shared duplicate voxel suppression, topology cell-edge lookup, typed
partition lookup, and padded-bounds predicates. Direct `GridTracer` calls remain
at the call sites so the helper does not hide forwarding-only methods in hot
paths.

`GravitasQuery2DService` does not directly traverse GridForge voxels. It
delegates candidate gathering to `GravitasCollision2DService`, which owns 2D
partition state, deferred partition refresh, duplicate query candidate versions,
partition sorting, layer filtering, and final collider ID ordering.

New focused tests cover topology padding mode selection, duplicate suppression,
retained empty partition lookup, and negative-coordinate padded-bound edges.
Existing 2D/mixed broad-phase tests cover sparse, dense, deterministic ordering,
and retained partition retirement behavior.

Post-migration benchmark baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-partition partition-culling mixed-broad-phase query-service physics-2d --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase1-baseline
```

BenchmarkDotNet ShortRun baseline on Windows 11, .NET 8.0.28, Intel Core
i7-9700K. Generated artifacts are under ignored `artifacts/`, so the compact
baseline is recorded here for future comparison.

### CollisionPartitionBenchmarks

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| CreateAndRegisterDynamicSpheres | 64 | 2,371.1 us | 2665388 B |
| CreateAndPartitionStaticSpheres | 64 | 2,083.6 us | 2585585 B |
| SimulatePartitionedDynamicSpheres | 64 | 186.3 us | 0 B |

### MixedBroadPhaseBenchmarks

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| SparseCandidateGathering | 64 | 727.1 us | 64 B |
| DenseCandidateGathering | 64 | 1,731.6 us | 42688 B |
| RetainedPartitionCleanupAfterChurn | 64 | 1,228.0 us | 64 B |
| SparseCandidateGathering | 1024 | 50,536.5 us | 64 B |
| DenseCandidateGathering | 1024 | 77,166.6 us | 480448 B |
| RetainedPartitionCleanupAfterChurn | 1024 | 63,522.5 us | 64 B |
| SparseCandidateGathering | 4096 | 617,662.2 us | 64 B |
| DenseCandidateGathering | 4096 | 214,682.5 us | 962368 B |
| RetainedPartitionCleanupAfterChurn | 4096 | 600,213.1 us | 64 B |

### PartitionCullingBenchmarks

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| RepartitionTeleportedDynamicSpheres | 64 | 679,202.86 ns | 72640 B |
| RemoveAndReAddDynamicPartitionMembers | 64 | 1,094.55 ns | 0 B |
| DistributeSleepingOnlyDynamicPartition | 64 | 12.20 ns | 0 B |
| RecheckCulledPairAfterColliderMove | 64 | 469.51 ns | 0 B |

### Physics2DBenchmarks

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| IntegrateDynamicBodies | 64 | 20,406.6 ns | 0 B |
| ResolveOverlappingCirclePairs_SweepBaseline | 64 | 21,740.1 ns | 0 B |
| ResolveOverlappingCirclePairs | 64 | 127,667.1 ns | 0 B |
| SimulateUnchangedColliders | 64 | 899.0 ns | 0 B |
| CheckRequiredShapePairs | 64 | 70,889.3 ns | 0 B |
| OverlapCircleAll_SweepBaseline | 64 | 23,664.9 ns | 0 B |
| OverlapCircleAll | 64 | 190,633.7 ns | 0 B |
| RaycastAll_SweepBaseline | 64 | 7,856.1 ns | 0 B |
| RaycastAll | 64 | 11,401.8 ns | 0 B |
| SweepCircleAll_NoHit | 64 | 2,111.0 ns | 0 B |
| SweepCircleAll_SparseHit | 64 | 29,430.5 ns | 0 B |
| SweepCircleAll_DenseHit | 64 | 338,467.9 ns | 0 B |
| DeactivateOverlappingPairOwners | 64 | 275,750.0 ns | 0 B |
| IntegrateDynamicBodies | 1024 | 357,359.2 ns | 0 B |
| ResolveOverlappingCirclePairs_SweepBaseline | 1024 | 1,699,518.2 ns | 0 B |
| ResolveOverlappingCirclePairs | 1024 | 4,838,581.2 ns | 0 B |
| SimulateUnchangedColliders | 1024 | 33,061.7 ns | 0 B |
| CheckRequiredShapePairs | 1024 | 1,283,779.0 ns | 0 B |
| OverlapCircleAll_SweepBaseline | 1024 | 261,118.5 ns | 0 B |
| OverlapCircleAll | 1024 | 229,527.2 ns | 0 B |
| RaycastAll_SweepBaseline | 1024 | 237,793.7 ns | 0 B |
| RaycastAll | 1024 | 11,181.0 ns | 0 B |
| SweepCircleAll_NoHit | 1024 | 2,047.7 ns | 0 B |
| SweepCircleAll_SparseHit | 1024 | 30,524.5 ns | 0 B |
| SweepCircleAll_DenseHit | 1024 | 498,968.2 ns | 0 B |
| DeactivateOverlappingPairOwners | 1024 | 6,544,433.3 ns | 0 B |

### QueryServiceBenchmarks

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| RaycastAllAcrossPopulatedContext | 64 | 132.845 us | 672 B |
| OverlapCircleAllAcrossPopulatedContext | 64 | 9.712 us | 0 B |
| DirectionalOverlapCircleAcrossPopulatedContext | 64 | 9.922 us | 0 B |
| RaycastAcrossTwoOverlappingContexts | 64 | 272.285 us | 1344 B |
| SweepSphereAllAcrossPopulatedContext | 64 | 262.301 us | 0 B |

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
