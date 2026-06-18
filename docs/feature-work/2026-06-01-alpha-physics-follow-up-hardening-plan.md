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

Earlier mesh work made `MeshColliderMode.Concave` work through raw triangle-set
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

- [x] Add comparison fixtures for simple concave, dense concave, contact-heavy
  U-channel, inside-corner, closed dense shell, and dynamic concave cases.
- [x] Benchmark current triangle-gather mesh-mesh behavior before changing the
  algorithm.
- [x] Evaluate direct BVH-vs-BVH paired traversal for mesh-mesh candidate
  generation so repeated per-triangle queries are reduced.
- [x] Preserve deterministic candidate order, contact identity, manifold
  reduction, and zero-allocation steady-state behavior.
- [x] Compare raw triangle BVH, BVH-vs-BVH traversal, and authored convex-piece
  collision assets for:
  - candidate count.
  - contact correctness.
  - manifold quality.
  - dense mesh-mesh cost.
  - simple mesh overhead.
- [x] Document whether the final recommendation is raw triangle BVH, paired BVH
  traversal, authored decomposition, or a thresholded combination.

**Exit Criteria**

- Concave mesh-mesh has a measured alpha policy instead of an unexamined
  hotspot.
- Simple concave meshes keep the exact triangle-BVH path unless a replacement is
  measurably better without added complexity.
- Dense/complex collision assets have a documented alternative path.

**Progress - 2026-06-17**

Expanded the mesh-mesh comparison fixtures to include dense concave
U-channel/inside-corner pairs, contact-heavy dense U-channel pairs, and dense
closed-shell pairs. Focused tests now cover dense same-pair contact identity,
reversed dense dispatch validity, dynamic concave mesh movement, and
zero-allocation dense mesh-mesh checks after warmup.

The original raw triangle-gather path remains the alpha runtime policy for
simple concave meshes. It preserves exact triangle collision truth, stable
same-pair contact IDs, and zero steady-state allocations. Direct BVH-vs-BVH
paired traversal was implemented and measured, then rejected: with the current
SwiftFixedBVH node API it repeatedly transformed internal node bounds and
expanded too many conservative node pairs, regressing every measured mesh-mesh
row.

The retained runtime optimization is narrower and safer: triangle-triangle SAT
now projects onto raw axes first, exits on separation, and only normalizes an
axis when it can update the stored penetration depth/normal. This keeps the
same triangle candidates and contact generation while avoiding unnecessary
fixed-point vector normalization on non-winning axes.

Expanded pre-change baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*MeshMesh*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4b-meshmesh-expanded-baseline
```

Rejected paired-BVH traversal measurement:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*MeshMesh*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4b-meshmesh-paired-bvh-after
```

Final SAT-axis optimization measurement:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*MeshMesh*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4b-meshmesh-sat-axis-after
```

| Method | Before | Paired BVH | Final SAT Axis | Allocated |
| --- | ---: | ---: | ---: | ---: |
| CheckMeshMeshPairs | 457.2 us | 470.3 us | 475.1 us | 0 B |
| CheckConcaveMeshMeshPairs | 7,079.4 us | 10,062.7 us | 5,977.8 us | 0 B |
| CheckDenseConcaveMeshMeshPairs | 43,289.2 us | 133,113.7 us | 41,422.0 us | 0 B |
| CheckContactHeavyConcaveMeshMeshPairs | 62,379.6 us | 179,498.2 us | 60,230.7 us | 0 B |
| CheckClosedDenseMeshMeshPairs | 302,322.0 us | 730,064.6 us | 296,628.3 us | 0 B |

The tiny `CheckMeshMeshPairs` regression is on the convex mesh path and inside
ShortRun noise for this phase; the optimized code is only used by the
concave/concave-or-concave/convex triangle manifold path. The main simple
concave row improved by about 15.6%; dense rows improved modestly because their
cost is dominated by candidate count and contact-heavy narrow-phase work, not
axis normalization alone.

**Captured Phase 4B Follow-Ups**

- Dense reversed mesh-mesh dispatch can produce a different reduced set of four
  contact IDs than the forward pair because contact point generation is
  directional and manifold reduction keeps only the deepest four contacts. The
  same pair remains deterministic across repeated checks. Fixing reversed
  reduced-manifold symmetry should be treated as a manifold-quality/solver
  policy change, not as a mesh candidate-generation optimization.
- Direct BVH-vs-BVH traversal should not be reintroduced without a different
  data path, such as cached transformed node bounds for one mesh pair or a
  SwiftCollections-level paired traversal API that can avoid repeated
  conservative bounds transforms.
- Dense closed-shell mesh-mesh remains far too expensive for complex runtime
  assets even after the SAT-axis optimization. Phase 4C authored convex
  collision assets are still the right alpha answer for dense or rendered-mesh
  collision content.

**Phase 4C: Authored Convex Collision Assets**

**Goal:** Let users choose authored/offline decomposed convex collision data for
complex meshes while preserving one host-facing collider identity.

**Tasks**

- [x] Decide whether authored convex pieces should use existing
  `LSCompoundCollider`, a mesh-owned internal piece path, or both:
  - `LSCompoundCollider` is already one collider ID, one body binding, one
    broad-phase identity, one event surface, and stable part order.
  - A future mesh-owned piece path may be justified only if public compound
    semantics do not fit baked mesh assets.
- [x] Add tests that prove decomposed/authored assets do not leak internal
  collider IDs, pair ownership, events, diagnostics, hierarchy bindings, or
  broad-phase identities.
- [x] Add benchmark fixtures comparing raw concave triangle BVH against authored
  convex/compound proxies on dense meshes.
- [x] Document the tradeoff:
  - raw triangle BVH is exact and strong for simple concave physics meshes.
  - dense rendered meshes should not be used as physics meshes.
  - complex collision assets should be simplified, decomposed, or authored as
    convex pieces offline.
- [x] Keep automatic runtime decomposition out of the simulation path.

**Exit Criteria**

- Authored convex-piece collision is clear, tested, and externally represented
  as one collider/body surface.
- Docs teach when to choose raw concave mesh collision versus authored compound
  pieces.
- Runtime never silently decomposes or simplifies authoritative mesh geometry.

**Progress - 2026-06-17**

Decision: authored/offline convex collision assets should use
`LSCompoundCollider` for alpha. It already has the exact runtime semantics this
phase needs: one collider ID, one body binding, one broad-phase identity, one
contact/event surface, stable internal part order, and explicit rejection of
concave mesh parts. A dedicated mesh-owned internal piece path remains a future
option only if baked mesh assets need public semantics that compound colliders
cannot express.

Implementation notes:

- `CompoundColliderPart` originally grew an explicit local offset alongside
  local rotation and local scale so generated/offline data could author the
  whole part transform atomically.
- The Phase 4C collider-wrapper authoring surface has since been superseded by
  Phase 5 data-only shape definitions. `LSCompoundCollider` now materializes
  private runtime part colliders from immutable authored part data.
- `LSCompoundCollider` reapplies the immutable authored part transform during
  shape rebuilds so private runtime collider mutation cannot silently change the
  baked compound layout.
- `LSMeshCollider` now applies `LocalOffset` to its `PhysicsMesh` transform by
  translating the mesh origin relative to `PhysicsMesh.LocalBounds.Center`.
  This keeps mesh vertices, mesh bounds, collider center, compound aggregation,
  diagnostics, and triangle queries aligned when convex mesh pieces are
  offset inside an authored compound.
- Tests cover one public collider identity, private internal part IDs,
  broad-phase owner membership, contact/event ownership, and diagnostics that
  draw convex mesh parts through the compound owner ID.
- Runtime automatic decomposition remains intentionally absent. Future
  simplification/decomposition belongs to the separate Phase 7 tooling plan.

Baseline artifacts captured before source changes:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*CollisionDetectionBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4c-authored-convex-baseline
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collider-shape --filter "*ColliderShapeBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4c-authored-convex-shape-baseline-rerun
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*Authored*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4c-authored-proxy-rows-baseline
```

The first broad `collider-shape` run emitted a BenchmarkDotNet child-process
`AccessViolationException` after two measured iterations of
`MoveCompoundRuntimeShapeStateAcrossPartitions`. A focused retry of that row
and a full rerun completed cleanly, so the usable baseline is the rerun
artifact above.

Post-change authored proxy artifact:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*Authored*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase4c-authored-proxy-rows-after
```

Important rows:

| Benchmark | Before | After | Allocated |
| --- | ---: | ---: | ---: |
| CheckDenseConcaveMeshMeshPairs | 41,212.1 us | n/a | 0 B |
| CheckClosedDenseMeshMeshPairs | 298,802.6 us | n/a | 0 B |
| CheckAuthoredCompoundProxyPairs | 555.1 us | 540.2 us | 0 B |
| GenerateAuthoredCompoundProxyManifolds | 575.9 us | 573.6 us | 0 B |
| CheckDenseConcaveMeshAuthoredCompoundProxyPairs | 29,003.2 us | 28,744.0 us | 0 B |

The benchmark signal matches the policy: authored compound proxies are orders of
magnitude cheaper when both sides avoid dense triangle sets. If a dense concave
mesh remains in the pair, the raw triangle cost is still visible, so the docs
should continue steering complex rendered meshes toward simplified/decomposed
physics assets rather than runtime concave triangle collision.

## Phase 5: Collider Shape Definitions And Authored Compound Data

**Goal:** Split authored collision shape data from runtime collider lifecycle
state so standalone colliders, compound parts, and future mesh tooling can share
a compact deterministic shape description without exposing fake child colliders.

**Context**

Phase 4C proved that `LSCompoundCollider` is the right alpha runtime surface for
authored convex-piece assets: one collider ID, one body binding, one broad-phase
identity, one event surface, stable part order, and no runtime decomposition.
The remaining API weakness is that compound pieces are still authored by
constructing `LSCollider` instances that are then forbidden from normal collider
lifecycle operations. That shape works internally, but it is overkill for
serialization, generated/offline data, and the future decomposition toolchain.

Introduce a public data-only `ColliderShapeDefinition` layer for authoritative
shape input:

```csharp
ColliderShapeDefinition
  - shape kind
  - radius / size / height
  - convex mesh vertices + triangles + inertia policy
  - no body, context, id, partition, events, parent, pairs, runtime buffers

CompoundColliderPart
  - ColliderShapeDefinition Shape
  - Vector3d LocalOffset
  - FixedQuaternion LocalRotation
  - Vector3d LocalScale
```

The runtime can still materialize private `LSCollider` instances from those
definitions if that remains the simplest way to reuse existing narrow-phase,
query, diagnostics, and inertia code. The public compound API should describe
authored shape data and transforms, not child collider lifecycle objects.

**Tasks**

- [x] Capture a pre-change benchmark baseline for the relevant Phase 4C rows:
  authored compound proxy collision, compound manifold generation, dense mesh
  vs authored compound, and compound runtime shape/partition movement.
- [x] Add a deterministic `ColliderShapeDefinition` API with factory helpers
  for supported alpha runtime shapes:
  - sphere.
  - capsule.
  - cuboid.
  - finite cylinder.
  - convex mesh with vertices, triangles, and mesh inertia policy.
- [x] Keep concave mesh definitions out of compound parts unless a later phase
  proves they have coherent one-identity compound semantics.
- [x] Add constructor/factory paths from `ColliderShapeDefinition` to standalone
  runtime colliders, for example `new LSCuboidCollider(definition)` or a focused
  factory if constructor overloads become ambiguous.
- [x] Redesign `CompoundColliderPart` so public authored parts own
  `ColliderShapeDefinition`, local offset, local rotation, and local scale.
- [x] Make `LSCompoundCollider` materialize any internal runtime part colliders
  privately and deterministically, preserving stable part order and existing
  collision semantics.
- [x] Remove or make internal the public API that exposes compound child
  collider lifecycle objects, unless a real host-facing use case remains.
- [x] Add tests that prove shape definitions:
  - contain no context/body/id/partition/pair/event state.
  - can build equivalent standalone colliders.
  - can build equivalent compound colliders.
  - keep one public compound collider identity in collision, diagnostics,
    queries, events, and broad-phase partitions.
  - keep nested compound and concave mesh out of the definition surface and
    reject default parts.
- [x] Do not add built-in serialization transport support for shape definitions
  yet; they are a data-only runtime construction API that host/tooling asset
  formats can serialize explicitly.
- [x] Update docs so Phase 7 tooling knows its runtime export target is
  `ColliderShapeDefinition[]` plus stable part transforms, not instantiated
  runtime colliders.

**Exit Criteria**

- Authored compound assets are represented as data-first shape definitions plus
  transforms.
- Runtime collider lifecycle state remains private to runtime colliders.
- Existing Phase 4C authored-compound correctness and benchmark behavior is
  preserved or improved.
- Future mesh simplification/decomposition tooling has a clean deterministic
  output shape before implementation starts.

**Progress - 2026-06-17**

Implemented `ColliderShapeDefinition` and `ColliderShapeDefinitionKind` as the
data-only 3D authoring layer for sphere, capsule, cuboid, finite cylinder, and
convex mesh shapes. Shape definitions snapshot mesh vertex/index arrays,
validate primitive dimensions and mesh index ranges, expose stable mesh element
accessors, and contain no runtime body, context, ID, partition, hierarchy, pair,
or event state.

Concrete runtime colliders now have definition-based construction paths:
`LSSphereCollider`, `LSCapsuleCollider`, `LSCuboidCollider`,
`LSCylinderCollider`, and `LSMeshCollider`. `ColliderShapeDefinition` also has a
`CreateCollider()` factory for callers that need to materialize an unbound
runtime collider from data.

`CompoundColliderPart` is now the public authored descriptor:
`ColliderShapeDefinition Shape`, `LocalOffset`, `LocalRotation`, and
`LocalScale`. Convenience factories cover sphere, capsule, cuboid, cylinder,
and convex mesh parts. `LSCompoundCollider` keeps public parts as authored data
and privately materializes runtime part colliders in stable declaration order so
existing narrow-phase, query, diagnostics, and inertia behavior is reused
without exposing fake child-collider lifecycle objects.

Built-in serialization transport support was intentionally not added to
`ColliderShapeDefinition` in this slice. The type is the deterministic runtime
construction target; host asset pipelines and future Phase 7 tooling can
serialize their own asset format into shape definitions and part transforms
before creating runtime shells.

Baseline artifacts captured before source changes:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*Authored*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase5-shape-definition-authored-baseline
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collider-shape --filter "*MoveCompoundRuntimeShapeStateAcrossPartitions*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase5-shape-definition-compound-shape-baseline
```

Post-change artifacts:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collision-detection --filter "*Authored*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase5-shape-definition-authored-after
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll collider-shape --filter "*MoveCompoundRuntimeShapeStateAcrossPartitions*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-17-phase5-shape-definition-compound-shape-after
```

Important rows:

| Benchmark | Before | After | Allocated |
| --- | ---: | ---: | ---: |
| CheckAuthoredCompoundProxyPairs | 582.8 us | 600.0 us | 0 B |
| GenerateAuthoredCompoundProxyManifolds | 541.2 us | 544.5 us | 0 B |
| CheckDenseConcaveMeshAuthoredCompoundProxyPairs | 29,683.4 us | 28,513.2 us | 0 B |
| MoveCompoundRuntimeShapeStateAcrossPartitions | 28.83 us | 28.29 us | 0 B |

The authored compound proxy rows are within ShortRun variance and keep zero
managed allocation. The shape-movement row is unchanged to slightly better in
this run, which matches the intended implementation: the new data layer affects
construction/authoring, while steady-state compound runtime traversal still
uses the existing materialized part path.

Verification:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --nologo --no-restore --filter "FullyQualifiedName~ColliderShapeDefinitionTests|FullyQualifiedName~AuthoredConvexCollisionAssetTests|FullyQualifiedName~CompoundColliderTests|FullyQualifiedName~CompoundColliderCollisionTests"
dotnet test Gravitas.slnx --configuration Release --nologo
dotnet test Gravitas.slnx --configuration ReleaseLean --nologo
```

Results: focused Release tests passed 16/16, full Release passed 406/406, and
full ReleaseLean passed 401/401.

## Phase 6: 2D Shape Definitions And Compound Collider Data

**Goal:** Bring the Phase 5 authored-shape model to pure 2D so 2D standalone
colliders, 2D compound collision assets, mixed slabs, queries, diagnostics, and
future tooling do not bake in a primitive-only 2D assumption.

**Context**

Phase 5 hardened 3D authored data by splitting `ColliderShapeDefinition` and
`CompoundColliderPart` away from runtime `LSCollider` lifecycle state. Pure 2D
currently has first-class `LSCircleCollider2D`, `LSAABBoxCollider2D`, and
`LSPolygonCollider2D`, but no data-only shape definition layer and no
`LSCompoundCollider2D` equivalent. That asymmetry is an alpha API gap: authored
2D collision assets would still need to instantiate runtime colliders as data,
and later CCD/diagnostic work could accidentally assume every 2D collider is a
single primitive.

The 2D model should be a sibling, not a reuse of the 3D definition type. Pure
2D uses X/Z projection with `Vector2d`, scalar yaw, `FixedBoundArea`, 2D
collision priorities, and optional mixed slab metadata. A separate
`ColliderShapeDefinition2D` keeps that contract explicit.

Proposed public data shape:

```csharp
ColliderShapeDefinition2D
  - shape kind
  - radius / size
  - convex polygon vertices
  - no body, context, id, partition, events, parent, pairs, runtime buffers

CompoundColliderPart2D
  - ColliderShapeDefinition2D Shape
  - Vector2d LocalOffset
  - Fixed64 LocalRotation
  - Vector2d LocalScale
```

**Tasks**

- [x] Capture focused 2D collision/query/partition benchmarks before source
  edits.
- [x] Add a deterministic `ColliderShapeDefinition2D` API with factory helpers
  for circle, AABB, and convex polygon shapes.
- [x] Add definition-based constructors/factories for existing standalone 2D
  colliders without adding runtime lifecycle state to definitions.
- [x] Add `CompoundColliderPart2D` as data-only authored part input with stable
  local offset, rotation, and scale.
- [x] Add `LSCompoundCollider2D` with one public 2D collider ID, one body
  binding, one event surface, one broad-phase identity, and private runtime part
  colliders materialized in deterministic declaration order.
- [x] Extend pure 2D collision settings, narrow-phase dispatch, partitioning,
  query services, and result ordering to handle compound 2D colliders without
  exposing child collider lifecycle.
- [x] Define and test mixed embedding behavior for 2D compound colliders so
  mixed 2D/3D contacts and swept queries use the owning 2D compound identity.
- [x] Add diagnostics/debug draw coverage for authored 2D compound parts while
  preserving owner collider IDs in emitted events.
- [x] Update docs and tests so future CCD and diagnostics phases know that pure
  2D includes compound authored data, not only primitive shapes.

**Exit Criteria**

- Pure 2D authored collision data has the same lifecycle/data separation that
  Phase 5 gave 3D.
- `LSCompoundCollider2D` behaves as one collider for registration, broad-phase
  membership, events, queries, diagnostics, hierarchy filtering, and mixed
  identity.
- 2D compound parts never leak public runtime child collider lifecycle objects.
- Benchmarks show no steady-state allocation regression in 2D collision,
  query, and partition hot paths.

**Progress - 2026-06-18**

Implemented `ColliderShapeDefinition2D` and
`ColliderShapeDefinition2DKind` as the pure 2D data-only authoring surface for
circle, axis-aligned box, and convex polygon shapes. Definitions snapshot
polygon vertices, validate dimensions and convexity, can materialize standalone
runtime colliders, and contain no body, context, collider ID, partition, pair,
hierarchy, query, event, or buffer state.

`CompoundColliderPart2D` now mirrors the 3D authored part model with
`ColliderShapeDefinition2D`, `Vector2d LocalOffset`, `Fixed64 LocalRotation`,
and `Vector2d LocalScale`. `LSCompoundCollider2D` owns one public 2D collider
identity and privately materializes deterministic part colliders from those
definitions. Internal part colliders cannot run standalone lifecycle operations,
cannot be registered independently, and are scanned in declaration order for
collision, queries, mixed embedding, CCD proxy radius, and diagnostics.

Pure 2D collision dispatch now includes compound owners through
`ColliderType2D.Compound` and `CollisionType2D.Compound`. Queries return the
owning compound collider, not the private part collider, and continue to use
caller-owned hit buffers and deterministic ordering. Mixed narrow phase,
mixed swept-sphere queries, and mixed debug draw now treat a 2D compound as an
embedded slab aggregate while preserving the owner collider ID in emitted hits
and diagnostic commands.

One benchmark side finding was fixed while measuring Phase 6: the primitive
2D raycast sweep-baseline benchmark exposed a fixed-point divide-by-zero on
parallel segment intersection. `QueryDetection2D.TryIntersectSegments` now has
an explicit zero denominator guard, with a regression test for repeated
horizontal raycasts through polygon edges.

Baseline artifacts captured before source changes:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll physics2d --filter "*Physics2DBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-18-phase6-2d-compound-baseline
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll mixed-query --filter "*MixedQueryBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-18-phase6-2d-compound-mixed-query-baseline
```

Post-change artifacts:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll Physics2DBenchmarks --filter "*Physics2DBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-18-phase6-2d-comparable-after-final
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll Physics2DCompoundBenchmarks --filter "*Physics2DCompoundBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-18-phase6-2d-compound-after-final
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll MixedQueryCompound2DBenchmarks --filter "*MixedQueryCompound2DBenchmarks*" --job short --exporters json --artifacts artifacts\benchmarks\2026-06-18-phase6-mixed-compound2d-after-final
```

The compound benchmark rows live in separate benchmark classes so existing
primitive 2D and mixed-query selections remain comparable to their pre-change
baselines.

Important rows:

| Benchmark | Before | After | Allocated |
| --- | ---: | ---: | ---: |
| SimulateUnchangedColliders(64) | 0.922 us | 1.052 us | 0 B |
| SimulateUnchangedColliders(1024) | 30.454 us | 32.837 us | 0 B |
| CheckRequiredShapePairs(64) | 79.638 us | 75.334 us | 0 B |
| CheckRequiredShapePairs(1024) | 1,288.722 us | 1,191.208 us | 0 B |
| OverlapCircleAll(64) | 200.109 us | 191.973 us | 0 B |
| OverlapCircleAll(1024) | 234.015 us | 225.887 us | 0 B |
| SweepCircleAll_SparseHit(64) | 29.716 us | 30.663 us | 0 B |
| SweepCircleAll_DenseHit(1024) | 493.625 us | 460.360 us | 0 B |
| SimulateUnchangedCompoundColliders(64) | n/a | 1.094 us | 0 B |
| CheckCompoundShapePairs(64) | n/a | 146.264 us | 0 B |
| OverlapCircleAll_CompoundTargets(64) | n/a | 218.421 us | 0 B |
| SweepCircleAll_CompoundTargets(64) | n/a | 39.014 us | 0 B |
| SimulateUnchangedCompoundColliders(1024) | n/a | 37.338 us | 0 B |
| CheckCompoundShapePairs(1024) | n/a | 2,716.276 us | 0 B |
| OverlapCircleAll_CompoundTargets(1024) | n/a | 275.366 us | 0 B |
| SweepCircleAll_CompoundTargets(1024) | n/a | 39.179 us | 0 B |
| SweepSphereAgainst2DAll_Compound2DTargets(64) | n/a | 435.0 us | 0 B |
| SweepSphereAgainst2DAll_Compound2DTargets(1024) | n/a | 16,803.7 us | 0 B |

Verification:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --nologo --no-restore --filter "FullyQualifiedName~Physics2DQueryTests|FullyQualifiedName~CompoundCollider2D|FullyQualifiedName~ColliderShapeDefinition2DTests|FullyQualifiedName~ContinuousMode_ShouldUseCompoundOwnerProxyRadius"
dotnet test Gravitas.slnx --configuration Release --nologo
dotnet test Gravitas.slnx --configuration ReleaseLean --nologo
```

Results: focused Release tests passed 21/21, full Release passed 423/423, and
full ReleaseLean passed 418/418.

## Phase 7: Mesh Simplification And Decomposition Tooling Plan

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

**Progress - 2026-06-18**

The Phase 7 tooling plan was captured in a separate document linked above. The plan includes:

- A research summary of exact and approximate convex decomposition methods,
  including their theoretical complexity, practical performance, and quality
  tradeoffs.
- A proposed API design for a Gravitas-owned mesh simplification and decomposition
  tool, including input/output formats, configuration settings, and deterministic
  result codes.
- A roadmap for implementation, testing, and benchmarking before the tool's output is recommended for alpha asset workflows.

## Phase 8: Dynamic CCD And Swept Mesh Families

**Goal:** Define the next continuous-collision slice beyond the current static
or kinematic target clipping so fast dynamic bodies, mesh targets, and mixed
queries have physically explainable deterministic policy.

**Context**

Current CCD support is opt-in/auto and intentionally bounded. 3D and 2D body
movement can use swept primitive proxies against static or kinematic targets,
and mixed sweeps include alpha mesh/compound support. Dynamic-vs-dynamic CCD
must be deterministic and physically explainable before alpha: simultaneous
fast movers should clamp at a shared time of impact instead of depending on
body iteration order. Moving mesh and compound bodies may use conservative
proxy radii until exact shape-specific swept-source solvers have stronger
evidence.

**Tasks**

- [x] Specify deterministic dynamic-vs-dynamic CCD ordering for 3D, pure 2D,
  and mixed contact paths.
- [x] Define how relative velocity, pair priority, body IDs, hierarchy keys,
  and contact normals break ties.
- [x] Add fixtures for tunneling dynamic bodies, opposing high-speed bodies,
  thin static geometry, and mixed 2D slab interactions.
- [x] Investigate shape-specific swept mesh behavior before adding public APIs:
  ray/segment vs mesh, swept sphere/circle vs mesh, and mesh-as-moving-source.
- [x] Benchmark CCD candidate gathering, clip resolution, and false-positive
  rates before replacing any current conservative proxy.

**Exit Criteria**

- CCD behavior remains explicit and opt-in/auto, not a silent global cost.
- Dynamic-vs-dynamic CCD has deterministic tie-breakers and tests before it is
  enabled.
- Swept mesh APIs are added only with allocation tests and benchmark evidence.

**Progress - 2026-06-18**

Implemented deterministic dynamic-vs-dynamic CCD for 3D, pure 2D, and mixed
3D/2D bodies. `GravitasWorldContext` now owns a late-simulation token so each
physics service can cache frame-start position and predicted displacement for
every movable body before sequential body integration mutates any one body. CCD
then compares static/kinematic query hits with dynamic relative-motion hits and
chooses the earliest distance, then higher closing speed, then stable collider
ID order.

The dynamic path intentionally uses conservative moving proxies:

- 3D uses relative sphere/sphere TOI over each collider's continuous proxy
  radius.
- 2D uses relative circle/circle TOI over each 2D collider's proxy radius.
- mixed 3D/2D maps 2D slabs into finite 3D proxy spheres using the larger of
  planar radius and mixed half-thickness.
- dynamic mesh and compound bodies are supported as moving proxy bodies, while
  static/kinematic mesh and compound targets still use the exact existing query
  workers where available.

No new public swept-mesh API was added. While validating mesh CCD, a query-level
normal issue surfaced: swept sphere hits against two-sided mesh surfaces could
return the authored triangle normal even when it pointed with the sweep
direction. `GravitasQuery3DService` now orients mesh sweep normals against the
sweep direction so CCD removes closing velocity from both mesh faces
deterministically.

New tests cover:

- resting dynamic 3D target CCD.
- opposing dynamic 3D spheres clamping at the shared time of impact.
- opposing dynamic 2D circles clamping at the shared time of impact.
- mixed dynamic 3D sphere vs 2D circle CCD.
- fast sphere vs immovable mesh CCD.
- mesh swept-sphere normal orientation from both sides.

Phase 8 baseline captured before implementation:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll continuous-collision --filter "*" --artifacts artifacts\benchmarks\2026-06-18-phase8-ccd-baseline --warmupCount 3 --iterationCount 8
```

Post-change benchmark run:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll continuous-collision --filter "*" --artifacts artifacts\benchmarks\2026-06-18-phase8-ccd-after --warmupCount 3 --iterationCount 8
```

BenchmarkDotNet on Windows 11, .NET 8.0.28, Intel Core i7-9700K:

| Method | Baseline Mean | After Mean | Allocated |
| --- | ---: | ---: | ---: |
| DiscreteFastMove | 2.925 us | 2.882 us | 672 B |
| ContinuousFastMoveAgainstThinWall | 13.164 us | 12.909 us | 672 B |
| ContinuousOpposingDynamicSpheres | n/a | 26.144 us | 1344 B |
| ContinuousFastMoveAgainstImmovableMesh | n/a | 21.704 us | 672 B |

Captured follow-up: if real alpha scenarios enable `Continuous` on many movable
bodies at once, add a size-parameter CCD benchmark and evaluate a GridForge
swept-volume candidate prefilter for dynamic relative-motion checks. The current
implementation favors deterministic correctness and simple tie-breaking; it is
not yet a claim that large all-continuous dynamic crowds have optimal candidate
gathering cost.

## Phase 8B: Dynamic CCD Candidate Prefilter Scaling

**Goal:** Make dynamic-vs-dynamic CCD scale for large deterministic lockstep
sims without weakening Phase 8 correctness, determinism, or the explicit
opt-in/auto CCD contract.

**Context**

Phase 8 deliberately chose the simplest deterministic dynamic CCD model: each
continuous moving source compares against every registered movable target using
frame-start relative-motion TOI. That is correct and easy to audit, but the
dynamic target scan can become the dominant cost when many important dynamic
bodies are CCD-capable in the same frame. LSF's target is larger than ordinary
local-player-centric games, so Gravitas should not rely on hosts manually
avoiding CCD at scale.

The prefilter must be conservative over both sides of motion. A target that
starts outside the source's swept bounds but moves into it during the frame must
remain a candidate. Candidate ordering must stay explicit and stable; no hash
or grid iteration order can become observable physics behavior.

**Tasks**

- [x] Add size-parameter dynamic CCD benchmarks before runtime changes:
  sparse 3D, dense 3D, sparse 2D, dense 2D, and mixed 3D/2D.
- [x] Measure current candidate-scan scaling and allocations for representative
  body counts.
- [x] Evaluate lower-stack spatial assets first (`GridForge`, then
  `SwiftCollections.FixedMathSharp`) before introducing Gravitas-specific
  structures.
- [x] Implement the smallest deterministic prefilter that materially improves
  measured scaling:
  - cache frame-start position and predicted displacement as Phase 8 already
    does.
  - build conservative swept proxy bounds for dynamic CCD targets.
  - gather only target candidates whose swept proxy bounds intersect the source
    swept proxy bounds.
  - traverse candidates in stable sorted order before relative TOI checks.
- [x] Add correctness tests for targets moving into a source sweep from outside
  the source's start bounds, deterministic tie ordering, disabled layers,
  triggers, siblings, and mixed dimensional candidates.
- [x] Re-run the same benchmark selection and document before/after results.

**Exit Criteria**

- Dynamic CCD candidate gathering is proven with size-scaling benchmarks.
- Sparse scenes avoid scanning unrelated dynamic bodies.
- Dense scenes preserve Phase 8 correctness and deterministic tie-breaking.
- Hot-path allocations do not increase after warmup.
- Docs describe the broad-phase/prefilter policy and remaining limits.

**Progress - 2026-06-18**

Added `DynamicCcdScalingBenchmarks` with sparse/dense 3D, sparse/dense 2D,
and sparse/dense mixed 3D/2D rows at 64 and 256 total bodies. An initial
64/256/1024 matrix timed out on the prefilter baseline, so the committed
benchmark keeps 64/256 as the repeatable comparison set and leaves 1024 as a
future stress run once the broader mixed path is cheaper.

Lower-stack review found `SwiftFixedSpatialHash<T>` is available and suitable
for persistent spatial indexing, but Phase 8B's data is a single-frame set of
already-computed swept proxy bounds. The implemented runtime path therefore
uses a Gravitas-owned sweep-and-prune candidate index instead of rebuilding a
hash every late frame. The index stores one swept AABB per eligible movable
dynamic target, sorts entries by fixed-point bounds and dynamic ID with an
allocation-free internal heap sort, and queries a conservative X-window
expanded by the largest target extent before exact relative sphere/circle TOI.
This preserves deterministic ordering without depending on hash or GridForge
traversal order.

`GravitasWorldContext.LateSimulate()` now prepares 3D and 2D dynamic CCD
candidate indices before either dimension moves, which keeps mixed 3D/2D CCD
from seeing stale opposite-dimension candidates. Service-level lazy preparation
remains for direct service/body paths such as immediate impulse tests.

Mixed CCD also now uses internal static-only mixed sweep variants before the
dynamic relative-motion pass. Public mixed queries still include dynamic
colliders; continuous-collision resolution avoids doing exact static-style
mixed sweeps against movable dynamics that will be handled by the dynamic CCD
candidate path.

Baseline:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll dynamic-ccd-scaling --filter "*" --artifacts artifacts\benchmarks\2026-06-18-phase8b-dynamic-ccd-scaling-baseline-corrected --launchCount 1 --warmupCount 1 --iterationCount 5 --unrollFactor 1
```

Final measured comparison:

```powershell
dotnet tests\Gravitas.Benchmarks\bin\Release\net8.0\Gravitas.Benchmarks.dll dynamic-ccd-scaling --filter "*" --artifacts artifacts\benchmarks\2026-06-18-phase8b-dynamic-ccd-scaling-post-heap-sort --launchCount 1 --warmupCount 1 --iterationCount 5 --unrollFactor 1
```

| Method | Bodies | Baseline | Final | Allocated |
| --- | ---: | ---: | ---: | ---: |
| Sparse3DDynamicCcd | 64 | 5.736 ms | 2.796 ms | 43008 B |
| Dense3DDynamicCcd | 64 | 6.583 ms | 3.447 ms | 43008 B |
| Sparse2DDynamicCcd | 64 | 3.983 ms | 3.191 ms | 0 B |
| Dense2DDynamicCcd | 64 | 4.419 ms | 3.132 ms | 0 B |
| SparseMixedDynamicCcd | 64 | 26.147 ms | 25.001 ms | 21504 B |
| DenseMixedDynamicCcd | 64 | 27.603 ms | 27.892 ms | 21504 B |
| Sparse3DDynamicCcd | 256 | 30.433 ms | 11.862 ms | 172032 B |
| Dense3DDynamicCcd | 256 | 32.377 ms | 13.673 ms | 172032 B |
| Sparse2DDynamicCcd | 256 | 25.111 ms | 13.886 ms | 0 B |
| Dense2DDynamicCcd | 256 | 29.026 ms | 13.564 ms | 0 B |
| SparseMixedDynamicCcd | 256 | 130.307 ms | 102.397 ms | 86016 B |
| DenseMixedDynamicCcd | 256 | 144.508 ms | 132.923 ms | 86016 B |

The pure 3D/2D paths now show the intended scaling improvement without adding
managed allocations after warmup. Mixed dynamic CCD improves at the larger
comparison size but remains the next visible hotspot and is noisy in the short
single-frame benchmark: even with dynamic relative checks prefiltered, mixed
CCD still pays for opposite-dimension query collection and broader mixed
embedding work. A partition-level static-only collector experiment was tried
and rejected for this phase because it pushed kinematic/static classification
into partition membership without a complete state-transition model. That
should be revisited only as part of a dedicated mixed broad-phase
classification pass.

## Phase 8C: Mixed CCD Signal And Shared Query Cost

**Goal:** Make mixed 3D/2D CCD measurements reliable enough to optimize from,
then reduce mixed CCD cost without weakening Phase 8/8B correctness,
deterministic ordering, or conservative swept-volume candidate policy.

**Context**

Phase 8B removed the obvious dynamic-target scan from 3D, 2D, and mixed CCD.
The pure 3D/2D benchmark rows now show a strong scaling win, but mixed CCD is
still the visible hotspot. The 256-body mixed rows improved, while 64-body
dense mixed was effectively noise, and BenchmarkDotNet still warned that some
single-frame iterations were too short for a stable signal. That makes Phase
8C both a measurement-hardening phase and an optimization phase.

Do not start by adding a clever mixed shortcut. First isolate where time is
actually going:

- mixed benchmark harness variance and short-iteration warning behavior.
- opposite-dimension candidate collection through the mixed broad phase.
- static-only CCD paths collecting movable dynamic colliders and filtering
  after collection.
- exact mixed sweep dispatch and shape-specific narrow-phase cost.
- duplicated swept-bound, proxy-radius, and candidate-buffer work across
  3D, 2D, and mixed continuous collision paths.

The rejected Phase 8B partition-level static-only collector experiment is a
useful warning, not a dead end. Filtering during collection may be a good
optimization. Persistently splitting partition membership by static/dynamic
state is only acceptable if the state-transition model is explicit and tested
for body activation, deactivation, immovable/kinematic changes, trigger
changes, layer changes, and repartitioning.

**Tasks**

- [ ] Capture a fresh mixed CCD baseline before runtime changes. If benchmark
  methodology changes, re-run the baseline under the new methodology before
  comparing runtime changes.
- [ ] Stabilize the mixed CCD benchmark signal:
  - add batched or microbenchmark variants that raise minimum iteration time
    without hiding per-frame reset/setup cost.
  - keep sparse/dense mixed rows and add 1024-body stress rows once the run
    time is reasonable.
  - split full mixed `LateSimulate` cost from candidate collection and exact
    sweep cost so the bottleneck is observable.
  - document BenchmarkDotNet warnings, outliers, and confidence intervals
    alongside mean timings.
- [ ] Add benchmark-visible internal counters where they improve attribution:
  mixed 2D/3D candidates collected, candidates rejected by static/dynamic
  policy, exact mixed sweeps attempted, dynamic CCD candidates returned, and
  final hits.
- [ ] Review lower-stack assets before adding new structures:
  `GridForge` traversal/partition APIs, `SwiftFixedBVH<T>`,
  `SwiftFixedSpatialHash<T>`, existing mixed broad-phase benchmarks, and any
  reusable FixedMathSharp bounds helpers.
- [ ] Implement only the measured highest-impact optimization. Candidate
  directions to evaluate:
  - mixed static-only collectors that filter during broad-phase collection
    without unsafe static/dynamic partition membership.
  - a per-frame or retained static mixed candidate index if repeated static
    sweep collection dominates many-CCD-body scenes.
  - shared swept-bound/proxy helpers if duplicated 3D/2D/mixed CCD prep shows
    up in profiles or allocation counters.
  - shape-specific mixed sweep fast paths if exact mixed narrow phase dominates.
- [ ] If a proven change also benefits pure 2D or 3D CCD/query paths, apply it
  there with separate before/after measurements instead of leaving the shared
  win on the table.
- [ ] Add correctness tests for any changed mixed broad-phase or query policy:
  public mixed queries still include dynamic targets, CCD static-only queries
  include bodyless/immovable/kinematic targets only, triggers/layers/sibling
  filters still apply, and hit ordering remains deterministic.
- [ ] Re-run focused CCD/mixed tests, full `Release`, full `ReleaseLean`, and
  the same benchmark set. Update this section with before/after results and
  rejected experiments.

**Exit Criteria**

- Mixed CCD has a benchmark signal that is reliable enough to act on, or the
  remaining noise is explicitly explained and bounded.
- The retained optimization is supported by measured cost attribution, not just
  by code-shape suspicion.
- Mixed CCD improves materially at the representative higher body counts
  without regressing public mixed query behavior.
- Pure 2D/3D shared-query improvements are included when the same measured
  change clearly helps them.
- Hot-path managed allocations do not increase after warmup.
- Any new broad-phase classification or indexing state has explicit lifecycle
  tests for activation, deactivation, state changes, and partition cleanup.

## Phase 9: Typed Diagnostic Views

**Goal:** Keep `GravitasDiagnosticEvent` compact while reducing host adapter
mistakes if generic fields become difficult to decode.

**Context**

Earlier diagnostic work kept the alpha diagnostic event stream generic.
`ScalarA`, `ScalarB`, `DataA`, and `DataB` are sufficient while every event kind
has documented field meaning and adapters decode by
`GravitasDiagnosticEventKind`. Typed views are a tooling convenience, not a
reason to bloat the capture hot path.

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
