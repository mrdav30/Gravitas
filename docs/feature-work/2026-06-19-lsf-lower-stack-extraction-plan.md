# LSF Lower-Stack Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move reusable deterministic math, traversal, and save/apply infrastructure out of Gravitas into the correct lower LSF libraries without weakening Gravitas' physics-facing APIs.

**Architecture:** FixedMathSharp should own reusable fixed-point transform and mesh geometry math. GridForge should own grid traversal/topology helpers that reason over `GridWorld`, `VoxelGrid`, and voxel partitions. Chronicler should own the reusable default save/apply phase base. Gravitas keeps physics-specific wrappers, collider ownership, body mobility policy, and runtime service integration.

**Tech Stack:** C# 11, FixedMathSharp `Fixed64`/`Vector3d`/`FixedQuaternion`/`Fixed3x3`, GridForge `GridWorld`/`VoxelGrid`/`IVoxelPartition`, Chronicler.Core, SwiftCollections, xUnit, Gravitas Release/ReleaseLean validation.

---

**Date:** 2026-06-19
**Status:** In progress / Workstreams 1-2 complete
**Owner:** LSF lower-stack hardening

## Purpose

Recent Gravitas hardening produced several utilities that are useful across the
LSF stack:

- `FixedTransform` is a deterministic transform shell used by host-agent
  bridges. Trailblazer and future engine-agnostic libraries should not need to
  copy Gravitas to get it.
- Closed-volume mesh mass-property math contains reusable fixed-point
  barycentric product helpers and mesh integral formulas. Gravitas should keep
  the `PhysicsMesh` collider wrapper, but the pure math belongs in
  FixedMathSharp.
- `GridForgeTraversal` and `GridTopologyMetricUtility` reason about GridForge
  topology and voxel traversal. Gravitas should consume those helpers from
  GridForge instead of owning them locally.
- `DefaultSaver` is a general save/apply lifecycle pattern that future LSF
  libraries can reuse through Chronicler.

This plan is deliberately split from physics solver work. The goal is cleaner
stack ownership and better end-user development experience, not runtime physics
behavior changes.

## Current Baseline

- Before Workstream 2, Gravitas owned
  `src/Gravitas/Support/FixedTransform.cs`; FixedMathSharp now owns the shared
  `FixedMathSharp.FixedTransform` type.
- Gravitas owns closed-volume mesh mass-property integration inside
  `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`, including
  `SumSquaredBarycentricProducts(...)` and `SumBarycentricProducts(...)`.
- FixedMathSharp already exposes `FixedMath.BarycentricCoordinate(...)` and
  `Vector3d.BarycentricCoordinates(...)`.
- Gravitas owns `src/Gravitas/Support/GridForgeTraversal.cs` and
  `src/Gravitas/Support/GridTopologyMetricUtility.cs`.
- Gravitas owns `src/Gravitas/Support/DefaultSaver.cs`; `PhysicsSettingsSaver`
  derives from it.
- Gravitas has `Release` and `ReleaseLean` package paths. Lean validation must
  remain clean after changing references or package boundaries.

## Design Decisions

- Move `FixedTransform` as a shared lower-stack transform shell while preserving
  current reference-style mutation semantics. Do not silently convert it to a
  value struct unless all consumers are redesigned to pass and store it by
  reference or by explicit `ref` APIs; copied transform values would break host
  publication semantics.
- Keep any parent/host hierarchy semantics minimal and deterministic. If
  `Parent` remains, it should be a simple transform reference, not a scene graph
  or engine object bridge.
- Move reusable mesh math, not the Gravitas `PhysicsMesh` runtime wrapper.
  Gravitas still owns collider limits, mesh collider modes, runtime bounds,
  query acceleration, shape definitions, and body/collider integration.
- Prefer FixedMathSharp APIs that operate on fixed-point primitives and explicit
  buffers. Avoid APIs that allocate hidden collections in hot paths unless the
  allocation is already part of the current Gravitas caller contract.
- Move GridForge traversal helpers into GridForge because they depend on
  `GridWorld`, `VoxelGrid`, topology metrics, and `IVoxelPartition`.
- Move `DefaultSaver` into Chronicler.Core with the same explicit phase names:
  `Save`, `EarlyApply`, `Apply`, and `LateApply`.
- During this implementation, keep local project references to sibling
  repositories. The owner will restore package references as each lower-stack
  package is released.

## File Map

### FixedMathSharp

- Added `../FixedMathSharp/src/FixedMathSharp/Numerics/Matrices/FixedTransform.cs`.
- Modify `../FixedMathSharp/src/FixedMathSharp/Core/FixedMath.cs` for scalar
  barycentric product helpers if they fit the existing `FixedMath` surface.
- Create or modify files under
  `../FixedMathSharp/src/FixedMathSharp/Geometry` for reusable closed-volume
  triangle-mesh validation and mass-property math.
- Add tests under `../FixedMathSharp/tests/FixedMathSharp.Tests`.

### GridForge

- Create or modify GridForge traversal/topology helper files under
  `../GridForge/src/GridForge`.
- Add tests under `../GridForge/tests`.

### Chronicler

- Create or modify the `DefaultSaver` base in
  `../Chronicler/src/Chronicler.Core` or the current Chronicler.Core source
  path.
- Add tests under `../Chronicler/tests` when the repo has a matching test
  project.

### Gravitas

- Delete or replace local wrappers:
  - `src/Gravitas/Support/FixedTransform.cs`
  - `src/Gravitas/Support/GridForgeTraversal.cs`
  - `src/Gravitas/Support/GridTopologyMetricUtility.cs`
  - `src/Gravitas/Support/DefaultSaver.cs`
- Modify `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs` to call
  FixedMathSharp mesh mass-property helpers.
- Modify usages in `src/Gravitas/Core`, `src/Gravitas/Queries`,
  `src/Gravitas/CollisionHandling`, `src/Gravitas/Settings`, and tests.
- Update docs in `docs/wiki` and active `docs/feature-work` plans where stack
  ownership is described.

## Workstream 1: Cross-Repo Baseline And API Shape

**Goal:** Lock the exact lower-stack API boundaries before moving files.

Tasks:

- [x] Verify sibling repositories are available:

```bash
Test-Path ..\FixedMathSharp; Test-Path ..\GridForge; Test-Path ..\Chronicler
```

- [x] Read the sibling repository contributor guides and package targets:

```bash
Get-Content ..\FixedMathSharp\AGENTS.md
Get-Content ..\GridForge\AGENTS.md
Get-Content ..\Chronicler\AGENTS.md
```

- [x] Inspect current target frameworks and package references for all four
  repositories so moved APIs compile for the lowest supported targets.

- [x] Decide the exact FixedMathSharp transform namespace and type shape before
  implementation. The default recommendation is a mutable reference type that
  preserves current Gravitas host-publication semantics.

- [x] Decide the mesh math API names. Recommended starting surface:
  `FixedMath.SumSquaredBarycentricProducts(...)`,
  `FixedMath.SumBarycentricProducts(...)`, and a geometry-level helper that
  computes closed-volume mesh mass properties from vertices and triangle
  indices.

- [x] Record any API name changes directly in this plan before implementation
  starts.

Expected result: the lower-stack type names, namespaces, and reference strategy
are explicit enough that implementation can proceed without guesswork.

Recorded decisions:

- `FixedTransform` lives in namespace `FixedMathSharp` as a mutable reference
  type backed by `Fixed4x4`.
- Scalar barycentric product helpers will be added to `FixedMath`.
- Closed-volume mesh validation and mass-property integration should move under
  `FixedMathSharp.Geometry.Meshes`; Gravitas keeps `PhysicsMesh`,
  `MeshInertiaPolicy`, collider limits, bounds/BVH ownership, and runtime
  integration.

## Workstream 2: FixedMathSharp Transform Extraction

**Goal:** Make deterministic transform state available to Trailblazer,
Gravitas, and future LSF libraries through FixedMathSharp.

Tasks:

- [x] Add FixedMathSharp tests covering construction from position/rotation/
  scale, construction from matrix, position mutation, rotation mutation, scale
  mutation, Euler angle round trip, and optional parent assignment.

- [x] Move or recreate `FixedTransform` in FixedMathSharp using only
  FixedMathSharp dependencies.

- [x] Preserve deterministic matrix-backed behavior:
  - `Position` maps to matrix translation.
  - `Rotation` maps to matrix rotation.
  - `Scale` maps to matrix global scale.
  - `LossyScale` remains a read-only view of matrix scale.
  - `EulerAngles` uses `FixedQuaternion.FromEulerAnglesInDegrees(...)`.

- [x] Update Gravitas to consume the FixedMathSharp transform type and remove
  the local Gravitas definition.

- [x] Run FixedMathSharp tests for the new transform coverage.

- [x] Run Gravitas focused host/transform tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~FixedTransform|FullyQualifiedName~HostContract|FullyQualifiedName~RuntimeMode"
```

Expected result: Gravitas and FixedMathSharp share one deterministic transform
type without changing host-agent semantics.

Notes:

- `Fixed4x4.SetGlobalScale(...)` was strengthened while extracting the type so
  scale mutation preserves translation and rotation instead of inheriting the
  old diagonal-reset behavior.
- Local project references were added through Gravitas, GridForge,
  SwiftCollections, and child test/benchmark projects to keep the sibling build
  chain coherent until package releases catch up.
- FixedMathSharp and SwiftCollections now use the same platform-level NuGet
  assets layout as GridForge so cross-repo solution builds do not look for
  missing configuration-specific restore assets.
- `Gravitas.slnx` temporarily includes the linked lower-stack projects so
  `Release` and `ReleaseLean` solution builds compile the local graph in the
  requested configuration.

## Workstream 3: FixedMathSharp Mesh Geometry Math Extraction

**Goal:** Move reusable fixed-point barycentric and closed-volume mesh
mass-property math into FixedMathSharp while keeping Gravitas' `PhysicsMesh`
as the physics/collider wrapper.

Tasks:

- [ ] Add FixedMathSharp tests for:
  - `SumSquaredBarycentricProducts(a, b, c)`.
  - `SumBarycentricProducts(firstA, firstB, firstC, secondA, secondB, secondC)`.
  - closed-volume cube volume, center of mass, and inertia tensor.
  - zero-volume, boundary, duplicate-triangle, non-manifold, inconsistent
    winding, and disconnected-shell validation results.

- [ ] Move scalar barycentric product helpers into FixedMathSharp beside
  existing barycentric helpers.

- [ ] Move reusable closed-volume mesh validation and mass-property formulas
  into FixedMathSharp geometry APIs.

- [ ] Keep Gravitas-specific mesh policy in Gravitas:
  - deterministic vertex/triangle limits.
  - `PhysicsMesh` bounds and query acceleration.
  - `MeshColliderMode`.
  - `MeshInertiaPolicy`.
  - collider shape definition boundaries.

- [ ] Update `PhysicsMesh.TryGetClosedVolumeMassProperties(...)` and
  `PhysicsMesh.CalculateInertiaTensor(...)` to call the FixedMathSharp helper.

- [ ] Run FixedMathSharp mesh/math tests.

- [ ] Run Gravitas mesh mass-property tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PhysicsMeshTests|FullyQualifiedName~ColliderRuntimeStateTests"
```

- [ ] Run mesh mass-property benchmarks after the code compiles:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll mesh-mass-property --list flat
```

Expected result: FixedMathSharp owns reusable mesh math, while Gravitas behavior
and tests remain unchanged.

## Workstream 4: GridForge Traversal And Topology Extraction

**Goal:** Move voxel traversal uniqueness and topology cell-edge helpers into
GridForge.

Tasks:

- [ ] Add GridForge tests equivalent to current
  `tests/Gravitas.Tests/Support/GridForgeTraversalTests.cs`.

- [ ] Move `GridTopologyMetricUtility` behavior into GridForge with public or
  internal APIs appropriate for GridForge consumers.

- [ ] Move `GridForgeTraversalState`, traversal padding mode, unique-partition
  lookup, and padded-bounds tests into GridForge.

- [ ] Preserve the distinction between 3D max-cell-edge padding and X/Z planar
  max-cell-edge padding.

- [ ] Update Gravitas services and queries to consume GridForge-owned helpers.

- [ ] Remove local Gravitas traversal/topology files after all usages compile.

- [ ] Run GridForge traversal tests.

- [ ] Run Gravitas partition/query/CCD focused tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Partition|FullyQualifiedName~Query|FullyQualifiedName~ContinuousCollision|FullyQualifiedName~Mixed"
```

Expected result: traversal behavior is unchanged, but GridForge owns the grid
topology knowledge.

## Workstream 5: Chronicler DefaultSaver Extraction

**Goal:** Make the save/apply phase base reusable from Chronicler.Core.

Tasks:

- [ ] Add Chronicler tests or compile coverage for a derived saver that observes
  `Save`, `EarlyApply`, `Apply`, and `LateApply` calling the matching protected
  hooks.

- [ ] Move `DefaultSaver` into Chronicler.Core using a namespace that future LSF
  libraries can consume without referencing Gravitas.

- [ ] Update `PhysicsSettingsSaver` to inherit the Chronicler-owned base.

- [ ] Delete the local Gravitas `DefaultSaver` file.

- [ ] Run Chronicler tests or build validation.

- [ ] Run Gravitas settings/serialization tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Settings|FullyQualifiedName~Serialization"
```

Expected result: Gravitas settings save/apply behavior is unchanged, and
Chronicler owns the common lifecycle base.

## Workstream 6: Gravitas Cleanup, Docs, And Release Validation

**Goal:** Finish integration and prove both standard and Lean package paths are
clean.

Tasks:

- [ ] Remove obsolete Gravitas namespaces/usings from source and tests.

- [ ] Update `README.md`, `AGENTS.md`, and `docs/wiki` where they describe
  source ownership for transforms, mesh math, GridForge traversal, or saver
  bases.

- [ ] Verify no stale local type references remain:

```bash
rg -n "Gravitas.Support.FixedTransform|class FixedTransform|GridForgeTraversal|GridTopologyMetricUtility|class DefaultSaver" src tests docs
```

- [ ] Run Gravitas full Release validation:

```bash
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

- [ ] Run Gravitas full ReleaseLean validation:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

- [ ] Keep local project references until the owner restores package references
  during the lower-stack release sequence.

Expected result: lower-stack APIs are the source of truth, Gravitas has no
duplicate utility implementations, and Release/ReleaseLean are clean.

## Exit Criteria

- FixedMathSharp exposes the shared deterministic transform shell needed by
  Gravitas and Trailblazer.
- FixedMathSharp owns reusable barycentric product and closed-volume mesh
  mass-property math.
- Gravitas `PhysicsMesh` remains the collider/runtime wrapper and no longer
  owns duplicated reusable mesh formulas.
- GridForge owns traversal and topology metric helpers used by Gravitas.
- Chronicler owns `DefaultSaver`.
- Gravitas docs identify the new source-of-truth libraries.
- Gravitas Release and ReleaseLean builds/tests pass.
