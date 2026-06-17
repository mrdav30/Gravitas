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

**Goal:** Revisit the mixed 2D-circle vs 3D sweep policy that originally used
`max(radius, halfThickness)` as a conservative swept-sphere proxy.

**Context**

The original policy was deterministic, simple, and intentionally conservative,
but it could over-report near tall slab corners because it was not a full swept
prism/capsule-like solver. Sphere targets now use an exact finite-slab
projection; other target families still retain the conservative fallback until
shape-specific solvers are justified.

**Tasks**

- [x] Add targeted tests that demonstrate current over-report behavior at slab
  corners and tall thickness values.
- [x] Design a deterministic swept-circle/slab or swept-prism solver that keeps
  stable ordering and explicit failure behavior.
- [x] Compare the exact solver against the current swept-sphere proxy for:
  - correctness on corner/edge cases.
  - false-positive rate.
  - steady-state allocation.
  - sparse and dense query cost.
- [x] Keep the proxy path only if it remains the better alpha tradeoff and is
  clearly documented as conservative.

**Exit Criteria**

- Mixed swept-circle behavior is either made more exact or the conservative
  proxy is retained with explicit tests, docs, and benchmark justification.

**Progress - 2026-06-17**

Implemented the first exact finite-slab solver for
`SweepCircleAgainst3D` sphere targets. The sphere solver keeps the 2D source as
a finite vertical slab: vertical overlap determines the sphere's effective
planar reach, then a deterministic 2D point sweep produces the time of impact.
This removes false positives where the old `max(radius, halfThickness)`
swept-sphere proxy inflated horizontal reach for tall slabs or rounded slab
corner cases.

The broad-phase query bounds now use the swept circle-slab volume instead of the
proxy sphere radius. Capsule, cuboid, finite cylinder, mesh, and compound
targets still use the existing conservative swept-sphere worker fallback. That
fallback remains documented as an alpha tradeoff in `docs/wiki/QUERY_SERVICES.md`
until shape-specific finite-slab solvers replace it.

New tests:

- `SweepCircleAgainst3D_WithTallSlabAndPlanarSeparation_ShouldRejectProxyOnlySphereHit`
- `SweepCircleAgainst3D_NearSlabCorner_ShouldUseVerticalOverlapToReducePlanarSphereReach`

New benchmark selection:

- `mixed-query` -> `MixedQueryBenchmarks`

No preserved pre-change mixed-query benchmark existed for the old proxy path, so
the correctness comparison is covered by red/green false-positive tests and the
following ShortRun numbers are the forward performance baseline.

Post-change mixed-query benchmark baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll mixed-query --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase2-mixed-query
```

BenchmarkDotNet ShortRun baseline on Windows 11, .NET 8.0.28, Intel Core
i7-9700K:

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| SweepCircleAgainst3DAll_SparseSphereTargets | 64 | 549.2 us | 0 B |
| SweepCircleAgainst3DAll_DenseSphereTargets | 64 | 243.5 us | 0 B |
| SweepCircleAgainst3DAll_CornerProxyMissSphereTargets | 64 | 143.3 us | 0 B |
| SweepCircleAgainst3DAll_SparseSphereTargets | 1024 | 17,070.5 us | 0 B |
| SweepCircleAgainst3DAll_DenseSphereTargets | 1024 | 6,885.9 us | 0 B |
| SweepCircleAgainst3DAll_CornerProxyMissSphereTargets | 1024 | 4,016.7 us | 0 B |

## Phase 3: Retained Partition Reset Semantics

**Goal:** Define whether context reset should detach retained empty partitions
from GridForge voxels or keep retained partition payloads available for reuse,
then apply the rule consistently across 3D, 2D, and mixed services.

**Tasks**

- [x] Audit retained partition cleanup in:
  - `PhysicsPartition`
  - `PhysicsPartition2D`
  - `PhysicsMixedPartition`
  - `GravitasCollisionService`
  - `GravitasCollision2DService`
  - `GravitasMixedCollisionService`
- [x] Decide the reset contract for long-running contexts, context reuse, and
  deterministic replay setup.
- [x] If reset detaches retained partitions, ensure voxel payload removal is
  stable and does not break partition reuse after the next registration.
- [x] Do not keep retained payloads attached across reset; document reset as a
  session boundary and keep normal retained-partition reuse on runtime churn.
- [x] Add tests for context reset after sparse, dense, and mixed partition
  usage.
- [x] Benchmark reset plus re-registration churn before and after any change.

**Exit Criteria**

- Reset semantics are explicit and uniform for 3D, 2D, and mixed paths.
- No stale collider IDs, stale pair keys, or orphaned partition state survives
  reset.
- Long-running simulation cleanup behavior is documented and benchmarked.

**Progress - 2026-06-17**

Decision: `GravitasWorldContext.Reset()` is a reusable-session boundary, not a
normal runtime churn step. Empty partitions are still retained during ordinary
movement and retired by TTL, but reset now detaches every owned
`PhysicsPartition`, `PhysicsPartition2D`, and `PhysicsMixedPartition` payload
from GridForge voxels through `TryRemovePartition<T>()`, clears retained
tracking, clears inactive partition pools, and then allows collider IDs to be
reused from clean service state.

New reset tests cover:

- 3D retained partition detach plus successful next registration.
- dense 3D retained partition detach across many covered voxel coordinates.
- pure 2D retained partition detach plus successful next registration.
- mixed retained partition detach plus successful next mixed registration.

The reset contract is documented in `docs/wiki/COLLISION_PIPELINE.md`,
`docs/wiki/HOST_INTEGRATION.md`, and `docs/wiki/RUNTIME_ARCHITECTURE.md`.
`CollisionPartitionBenchmarks` now includes
`ResetAndReRegisterDynamicSpheres` as the forward reset/re-registration churn
baseline.

Pre-change benchmark artifacts:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll partition-culling mixed-broad-phase collision-partition world-context --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase3-reset-baseline
```

Post-change benchmark artifacts:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll partition-culling mixed-broad-phase collision-partition world-context --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase3-reset-after
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll mixed-broad-phase --filter "*RetainedPartitionCleanupAfterChurn*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase3-reset-after-mixed-retry
```

The broad post-change run exported artifacts but printed a post-cleanup
`AccessViolationException` and one `NA` row for
`MixedBroadPhaseBenchmarks.RetainedPartitionCleanupAfterChurn` at 64 colliders.
The focused retry completed cleanly and supplies the retained-cleanup comparison
rows below.

| Benchmark | Before | After | Allocated |
| --- | ---: | ---: | ---: |
| CreateAndRegisterDynamicSpheres(64) | 2,546.9 us | 2,541.3 us | 2,665,388 B |
| CreateAndPartitionStaticSpheres(64) | 2,258.1 us | 2,233.8 us | 2,585,585 B |
| SimulatePartitionedDynamicSpheres(64) | 194.4 us | 195.2 us | 0 B |
| ResetAndReRegisterDynamicSpheres(64) | n/a | 1,126.8 us | 1,013,891 B |
| RepartitionTeleportedDynamicSpheres(64) | 723,079.10 ns | 721,618.31 ns | 72,640 B |
| RemoveAndReAddDynamicPartitionMembers(64) | 1,107.14 ns | 1,109.58 ns | 0 B |
| Mixed RetainedPartitionCleanupAfterChurn(64) | 1,692.0 us | 1.736 ms | 64 B |
| Mixed RetainedPartitionCleanupAfterChurn(1024) | 71,638.4 us | 74.961 ms | 64 B |
| Mixed RetainedPartitionCleanupAfterChurn(4096) | 651,880.1 us | 669.843 ms | 64 B |

ShortRun variance is high for the large mixed retained-cleanup rows, but the
existing steady-state partitioning selections did not show a meaningful
regression from reset-only lifecycle cleanup.

## Phase 4: Mesh Volume, Dense Concavity, And Authored Collision Assets

**Goal:** Hardening mesh physics for alpha without turning runtime collision
into an implicit asset-processing pipeline. Runtime mesh collision should keep
the raw local-BVH triangle path for simple concave geometry, require meaningful
closed-volume mass properties for dynamic mesh bodies by default, and give
users an explicit authored convex-piece path for complex collision assets.

**Context**

Phase 7B made `MeshColliderMode.Concave` work through raw triangle-set
narrow-phase using local-BVH candidate gathering. The first Phase 4 evaluation
kept that baseline: the `SwiftFixedBVH<int>` stores triangle bounds in local
mesh space, rigid movement updates transform-derived state without rebuilding
topology, and existing tests pin open-channel and inside-corner behavior so
concave meshes do not collapse into accidental convex hulls.

The current baseline also shows the real pressure point. ShortRun
`collision-detection` and `collider-shape` artifacts were captured before any
Phase 4 implementation:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collider-shape collision-detection --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4-mesh-policy-baseline
```

Important rows:

| Method | Size | Mean | Allocated |
| --- | ---: | ---: | ---: |
| BuildValidatedMeshTriangleBVH | 64 | 2.031 us | 2944 B |
| MoveMeshRuntimeShapeStateAndQueryTriangles | 64 | 9.282 us | 0 B |
| MoveDynamicConcaveMeshAndQueryTriangles | 64 | 41.145 us | 0 B |
| MoveCompoundRuntimeShapeStateAcrossPartitions | 64 | 28.086 us | 0 B |
| CheckMeshMeshPairs | 64 | 449.94 us | 0 B |
| CheckConcaveMeshCuboidPairs | 64 | 1,529.12 us | 0 B |
| CheckConcaveMeshMeshPairs | 64 | 6,667.72 us | 0 B |

Focused mesh, mixed, and swept-query tests passed under `ReleaseLean`:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration ReleaseLean --nologo --filter "FullyQualifiedName~ConcaveMesh|FullyQualifiedName~PhysicsMesh|FullyQualifiedName~MeshColliderMode|FullyQualifiedName~LSMeshColliderQuery|FullyQualifiedName~MeshTriangleQueryAllocation|FullyQualifiedName~MixedNarrowPhase|FullyQualifiedName~GravitasQuery3DServiceSweep"
```

Result: 58 passed, 0 failed.

**Phase 4A: Closed-Volume Mesh Inertia**

**Goal:** Make dynamic mesh mass properties physically meaningful by default.
Closed-volume inertia is an alpha release requirement; dynamic open/surface
meshes may remain possible only through an explicit opt-in approximation policy.

**Tasks**

- [x] Define mesh volume policy:
  - dynamic mesh bodies default to requiring a validated closed volume.
  - static, bodyless, immovable, and kinematic surface meshes remain legal for
    collision where existing behavior is correct.
  - dynamic open/surface meshes require an explicit surface-inertia
    approximation opt-in or caller-supplied inertia policy.
- [x] Add deterministic mesh topology validation for closed-volume eligibility:
  - every undirected edge has exactly two incident triangles.
  - triangle winding is consistently oriented or deterministically normalized.
  - zero-area, non-manifold, boundary, duplicate, and disconnected shell cases
    return explicit failure reasons.
- [x] Implement or specify closed-polyhedron mass properties using fixed-point
  signed tetrahedral integration over triangle faces.
- [x] Add reference tests for cube, rectangular prism, tetrahedron or simple
  wedge, translated mesh, rotated mesh, reversed winding, open plane, open
  U-channel, non-manifold edge, and disconnected shells.
- [x] Add benchmarks for validation and mass-property generation on small,
  medium, and dense closed meshes.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` and public XML docs so users
  understand when mesh inertia is solid-volume truth, explicit approximation,
  or rejected.

**Exit Criteria**

- Dynamic closed mesh inertia is deterministic, tested against fixed expected
  values, and independent of rigid movement.
- Dynamic open/surface mesh inertia is never a silent default.
- Alpha docs clearly explain the runtime distinction between surface collision
  data and solid mass properties.

**Progress - 2026-06-17**

Implemented `MeshInertiaPolicy.RequireClosedVolume` as the default for mesh
inertia and `MeshInertiaPolicy.SurfaceApproximation` as the explicit legacy
surface-area approximation path. `StiffBody` now asks colliders for inertia only
when angular dynamics are active, so bodyless/static, immovable, kinematic, and
angular-force-disabled mesh surfaces do not get forced through volume
validation.

`PhysicsMesh.TryGetClosedVolumeMassProperties(...)` validates closed-volume
eligibility by sorting deterministic triangle and edge uses: triangles cannot be
duplicated, every undirected edge must have exactly two incident triangles,
adjacent triangle edge directions must oppose each other, and all triangles must
belong to one connected shell. Whole-mesh reversed winding is accepted and
normalized through signed volume. Boundary, duplicate-triangle, non-manifold,
inconsistent-winding, disconnected-shell, and zero-volume failure states are
surfaced through `MeshVolumeValidationResult`.

Closed-volume inertia is integrated with fixed-point signed tetrahedra over the
triangle faces and cached on the immutable mesh topology. Because Gravitas does
not yet have a body center-of-mass offset model, the runtime tensor remains
diagonal about the collider reference center (`PhysicsMesh.LocalBounds.Center`);
`MeshMassProperties.CenterOfMass` is exposed for a future COM-offset hardening
pass. While touching inertia setup, `StiffBody` now correctly rotates a nonzero
inverse inertia tensor into the body's initial orientation.

Focused tests cover closed unit-cube inertia, rigid movement invariance,
reversed winding, explicit open-surface approximation, open plane rejection,
duplicate-face rejection, non-manifold edge rejection, disconnected
closed-shell rejection, default dynamic open-mesh rejection,
bodyless/immovable/kinematic legality, and rotated non-uniform cuboid
inverse-inertia orientation.

Phase 4A pre-change mesh baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collider-shape --filter "*Mesh*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4a-closed-volume-inertia-baseline
```

Forward mass-property benchmark baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll mesh-mass-property --filter "*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4a-closed-volume-inertia-after-duplicate-validation
```

| Method | Subdivision | Mean | Allocated |
| --- | ---: | ---: | ---: |
| BuildValidatedMeshTriangleBVH | n/a | 1.913 us | 2944 B |
| MoveMeshRuntimeShapeStateAndQueryTriangles | n/a | 9.023 us | 0 B |
| MoveDynamicConcaveMeshAndQueryTriangles | n/a | 40.513 us | 0 B |
| BuildAndValidateClosedVolume | 1 | 9.252 us | 10176 B |
| BuildAndValidateClosedVolume | 8 | 708.406 us | 529327 B |
| BuildAndValidateClosedVolume | 16 | 2.589 ms | 2111592 B |
| CalculateCachedClosedVolumeInertiaTensor | 1 | 49.06 ns | 0 B |
| CalculateCachedClosedVolumeInertiaTensor | 8 | 47.20 ns | 0 B |
| CalculateCachedClosedVolumeInertiaTensor | 16 | 47.07 ns | 0 B |
| CalculateSurfaceApproximationInertiaTensor | 1 | 973.95 ns | 0 B |
| CalculateSurfaceApproximationInertiaTensor | 8 | 66.770 us | 0 B |
| CalculateSurfaceApproximationInertiaTensor | 16 | 279.786 us | 0 B |

**Captured Phase 4A Follow-Ups**

These are solver/API boundaries observed during Phase 4A and intentionally not
folded into the closed-volume inertia slice:

- `PhysicsMesh.CalculateInertiaTensor(mass)` is still a shape/topology API; it
  does not know whether a body is movable, kinematic, immovable, or angular
  disabled. Phase 4A moved that decision to `StiffBody.RefreshInertiaTensor()`.
  Future mesh inertia API work should keep body mobility policy at the body or
  collider-binding boundary rather than making `PhysicsMesh` infer runtime
  ownership.
- Gravitas still uses diagonal local inertia tensors and `InvertDiagonal()`.
  Full tensor inversion, product-of-inertia support, and/or deterministic
  principal-axis diagonalization are a deeper angular-solver upgrade.
- Gravitas does not yet model body center-of-mass offsets. Phase 4A exposes
  `MeshMassProperties.CenterOfMass`, but runtime mesh inertia is still computed
  about the collider reference center because contact relative points, body
  transforms, serialization, and parallel-axis behavior all need an explicit COM
  model before the solver can consume arbitrary mesh COM safely.
- `StiffBody.InverseMass` is the raw reciprocal of `Mass`; immovable and
  kinematic bodies are mapped to zero effective inverse mass by response-layer
  wrappers such as `ResponseBody` and mixed response helpers. Future body/mass
  cleanup should decide whether to add an explicit effective inverse-mass API or
  keep every caller responsible for applying mobility gates.

**Phase 4B: Concave Mesh-Mesh Hotspot**

**Goal:** Reduce the dense concave mesh-mesh cost without losing exact triangle
collision truth for simple meshes where raw BVH remains the right answer.

**Tasks**

- [ ] Add comparison fixtures for simple concave, dense concave, contact-heavy
  U-channel, inside-corner, closed dense shell, and dynamic concave cases.
- [ ] Benchmark current triangle-gather mesh-mesh behavior before changing the
  algorithm.
- [ ] Evaluate direct BVH-vs-BVH paired traversal for mesh-mesh candidate
  generation so repeated per-triangle queries are reduced.
- [ ] Preserve deterministic candidate order, contact identity, manifold
  reduction, and zero-allocation steady-state behavior.
- [ ] Compare raw triangle BVH, BVH-vs-BVH traversal, and authored convex-piece
  collision assets for:
  - candidate count.
  - contact correctness.
  - manifold quality.
  - dense mesh-mesh cost.
  - simple mesh overhead.
- [ ] Document whether the final recommendation is raw triangle BVH, paired BVH
  traversal, authored decomposition, or a thresholded combination.

**Exit Criteria**

- Concave mesh-mesh has a measured alpha policy instead of an unexamined
  hotspot.
- Simple concave meshes keep the exact triangle-BVH path unless a replacement is
  measurably better without added complexity.
- Dense/complex collision assets have a documented alternative path.

**Phase 4C: Authored Convex Collision Assets**

**Goal:** Let users choose authored/offline decomposed convex collision data for
complex meshes while preserving one host-facing collider identity.

**Tasks**

- [ ] Decide whether authored convex pieces should use existing
  `LSCompoundCollider`, a mesh-owned internal piece path, or both:
  - `LSCompoundCollider` is already one collider ID, one body binding, one
    broad-phase identity, one event surface, and stable part order.
  - A future mesh-owned piece path may be justified only if public compound
    semantics do not fit baked mesh assets.
- [ ] Add tests that prove decomposed/authored assets do not leak internal
  collider IDs, pair ownership, events, diagnostics, hierarchy bindings, or
  broad-phase identities.
- [ ] Add benchmark fixtures comparing raw concave triangle BVH against authored
  convex/compound proxies on dense meshes.
- [ ] Document the tradeoff:
  - raw triangle BVH is exact and strong for simple concave physics meshes.
  - dense rendered meshes should not be used as physics meshes.
  - complex collision assets should be simplified, decomposed, or authored as
    convex pieces offline.
- [ ] Keep automatic runtime decomposition out of the simulation path.

**Exit Criteria**

- Authored convex-piece collision is clear, tested, and externally represented
  as one collider/body surface.
- Docs teach when to choose raw concave mesh collision versus authored compound
  pieces.
- Runtime never silently decomposes or simplifies authoritative mesh geometry.

**Phase 4D: Mesh Simplification And Decomposition Tooling Plan**

Future Gravitas-owned mesh simplification and decomposition should live in a
separate solution project/package, not in runtime simulation code. Track that
effort in
[`2026-06-17-mesh-tooling-simplification-and-decomposition-plan.md`](2026-06-17-mesh-tooling-simplification-and-decomposition-plan.md).

Research context from CGAL and decomposition literature should inform the
tooling design, but not create a runtime dependency. Exact convex decomposition
of closed polyhedra can produce `O(r^2)` convex pieces in the number of reflex
edges, while approximate convex decomposition can produce fewer, tighter
runtime shapes by allowing controlled volume over-coverage. Any future Gravitas
tool should expose deterministic failure/result codes, stable ordering, bounded
settings, and benchmarked quality metrics before its output becomes a
recommended alpha asset path.

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
