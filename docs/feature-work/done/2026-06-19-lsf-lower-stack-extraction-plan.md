# LSF Lower-Stack Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move reusable deterministic math, traversal, and save/apply
infrastructure out of Gravitas into the correct lower LSF libraries without
weakening Gravitas' physics-facing APIs.

**Architecture:** FixedMathSharp should own reusable fixed-point transform and
geometry algebra. GridForge should own grid traversal/topology helpers that
reason over `GridWorld`, `VoxelGrid`, and voxel partitions. Chronicler should
own the reusable default save/apply phase base. Gravitas keeps physics-specific
wrappers, collider ownership, body mobility policy, mesh mass-property
integration, and runtime service integration.

**Tech Stack:** C# 11, FixedMathSharp
`Fixed64`/`Vector3d`/`FixedQuaternion`/`Fixed3x3`, GridForge
`GridWorld`/`VoxelGrid`/`IVoxelPartition`, Chronicler.Core, SwiftCollections,
xUnit, Gravitas Release/ReleaseLean validation.

---

**Date:** 2026-06-19 **Status:** Done / all workstreams complete **Owner:** LSF
lower-stack hardening

## Purpose

Recent Gravitas hardening produced several utilities that are useful across the
LSF stack:

- `FixedTransform` is a deterministic transform shell used by host-agent
  bridges. Trailblazer and future engine-agnostic libraries should not need to
  copy Gravitas to get it.
- Closed-volume mesh mass-property math contains a small amount of reusable
  fixed-point barycentric product algebra. Gravitas should keep `PhysicsMesh`,
  mesh volume validation, center-of-mass calculation, and inertia formulas
  because those are physics mass-property concerns.
- GridForge traversal/topology helpers reason about GridForge topology and voxel
  traversal. Gravitas now consumes those helpers from GridForge instead of
  owning them locally.
- `DefaultSaver` is a general save/apply lifecycle pattern that future LSF
  libraries can reuse through Chronicler.

This plan is deliberately split from physics solver work. The goal is cleaner
stack ownership and better end-user development experience, not runtime physics
behavior changes.

## Current Baseline

- Before Workstream 2, Gravitas owned `src/Gravitas/Support/FixedTransform.cs`;
  FixedMathSharp now owns the shared `FixedMathSharp.FixedTransform` type.
- Gravitas owns closed-volume mesh mass-property integration inside
  `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`. Only the
  barycentric product algebra is reusable enough for FixedMathSharp.
- FixedMathSharp already exposes `FixedMath.BarycentricCoordinate(...)` and
  `Vector3d.BarycentricCoordinates(...)`.
- GridForge owns traversal helpers in `GridForge.Utility` and topology metrics
  in `GridForge.Grids.Topology`; Gravitas no longer owns local traversal or
  topology helper files.
- Chronicler owns the shared `DefaultSaver` base in its root `Chronicler`
  namespace; `PhysicsSettingsSaver` derives from that lower-stack type.
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
- Move reusable barycentric product algebra, not the Gravitas closed-volume mesh
  mass-property model. Gravitas still owns mesh volume validation,
  center-of-mass calculation, inertia formulas, collider limits, mesh collider
  modes, runtime bounds, query acceleration, shape definitions, and
  body/collider integration.
- Prefer FixedMathSharp APIs that operate on fixed-point primitives and explicit
  buffers. Avoid APIs that allocate hidden collections in hot paths unless the
  allocation is already part of the current Gravitas caller contract.
- Move GridForge traversal helpers into GridForge because they depend on
  `GridWorld`, `VoxelGrid`, topology metrics, and `IVoxelPartition`.
- Move `DefaultSaver` into Chronicler.Core with the same explicit phase names:
  `Save`, `EarlyApply`, `Apply`, and `LateApply`.
- During implementation, use local project references to sibling repositories
  when lower-stack APIs have not been released yet. For release validation,
  Gravitas should return to package references for the released lower-stack
  packages.

## File Map

### FixedMathSharp

- Added
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Matrices/FixedTransform.cs`.
- Modify `../FixedMathSharp/src/FixedMathSharp/Core/FixedMath.cs` for scalar
  barycentric product helpers if they fit the existing `FixedMath` surface.
- Modify `../FixedMathSharp/src/FixedMathSharp/Numerics/Matrices/Fixed3x3.cs`
  for the symmetric barycentric product-sum matrix factory.
- Add tests under `../FixedMathSharp/tests/FixedMathSharp.Tests`.

### GridForge

- Added `../GridForge/src/GridForge/Utility/GridTraversal.cs`.
- Added
  `../GridForge/src/GridForge/Grids/Topology/GridTopologyMetricUtility.cs`.
- Added traversal/topology tests under `../GridForge/tests/GridForge.Tests`.

### Chronicler

- Added `../Chronicler/src/Chronicler/Support/DefaultSaver.cs`.
- Added `../Chronicler/tests/Chronicler.Tests/Support/DefaultSaverTests.cs`.

### Gravitas

- Delete or replace local wrappers:
  - `src/Gravitas/Support/FixedTransform.cs`
  - `src/Gravitas/Support/GridForgeTraversal.cs` (removed in Workstream 4)
  - `src/Gravitas/Support/GridTopologyMetricUtility.cs` (removed in
    Workstream 4)
  - `src/Gravitas/Support/DefaultSaver.cs`
- Modify `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs` to call
  FixedMathSharp barycentric product matrix helpers.
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
      implementation. The default recommendation is a mutable reference type
      that preserves current Gravitas host-publication semantics.

- [x] Decide the reusable geometry API names:
      `FixedMath.SumSquaredBarycentricProducts(...)`,
      `FixedMath.SumBarycentricProducts(...)`, and
      `Fixed3x3.CreateBarycentricProductSums(...)`.

- [x] Record any API name changes directly in this plan before implementation
      starts.

Expected result: the lower-stack type names, namespaces, and reference strategy
are explicit enough that implementation can proceed without guesswork.

Recorded decisions:

- `FixedTransform` lives in namespace `FixedMathSharp` as a mutable reference
  type backed by `Fixed4x4`.
- Scalar barycentric product helpers will be added to `FixedMath`.
- Symmetric barycentric product-sum matrix construction will be added to
  `Fixed3x3`.
- Closed-volume mesh validation and mass-property integration should stay in
  Gravitas. The only Workstream 3 extraction is barycentric product algebra.

## Workstream 2: FixedMathSharp Transform Extraction

**Goal:** Make deterministic transform state available to Trailblazer, Gravitas,
and future LSF libraries through FixedMathSharp.

Tasks:

- [x] Add FixedMathSharp tests covering construction from position/rotation/
      scale, construction from matrix, position mutation, rotation mutation,
      scale mutation, Euler angle round trip, and optional parent assignment.

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
- Local project references were added temporarily through Gravitas, GridForge,
  SwiftCollections, and child test/benchmark projects to keep the sibling build
  chain coherent until package releases caught up. Final validation restored
  package references.
- FixedMathSharp and SwiftCollections now use the same platform-level NuGet
  assets layout as GridForge so cross-repo solution builds do not look for
  missing configuration-specific restore assets.
- `Gravitas.slnx` temporarily included the linked lower-stack projects so
  `Release` and `ReleaseLean` solution builds compiled the local graph in the
  requested configuration. Final validation restored the Gravitas-only solution
  graph.

## Workstream 3: FixedMathSharp Barycentric Product Helper Extraction

**Goal:** Move only reusable fixed-point barycentric product algebra into
FixedMathSharp while keeping Gravitas' `PhysicsMesh`, mesh validation,
center-of-mass calculation, and inertia formulas in Gravitas.

Tasks:

- [x] Add FixedMathSharp tests for:
  - `SumSquaredBarycentricProducts(a, b, c)`.
  - `SumBarycentricProducts(firstA, firstB, firstC, secondA, secondB, secondC)`.
  - `Fixed3x3.CreateBarycentricProductSums(a, b, c)`.

- [x] Move scalar barycentric product helpers into FixedMathSharp beside
      existing barycentric helpers, and add the symmetric product-sum factory to
      `Fixed3x3`.

- [x] Keep Gravitas-specific mesh policy in Gravitas:
  - closed-volume mesh validation.
  - volume, center-of-mass, and inertia tensor formulas.
  - deterministic vertex/triangle limits.
  - `PhysicsMesh` bounds and query acceleration.
  - `MeshColliderMode`.
  - `MeshInertiaPolicy`.
  - collider shape definition boundaries.

- [x] Update `PhysicsMesh` closed-volume integration to call the FixedMathSharp
      symmetric barycentric product-sum matrix helper.

- [x] Run FixedMathSharp scalar and matrix math tests.

- [x] Run Gravitas mesh mass-property tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PhysicsMeshTests|FullyQualifiedName~ColliderRuntimeStateTests"
```

- [x] Run mesh mass-property benchmarks after the code compiles:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll mesh-mass-property --list flat
```

Expected result: FixedMathSharp owns only reusable barycentric product algebra,
while Gravitas behavior, mesh mass-property ownership, and tests remain
unchanged.

Notes:

- Full mesh mass-property extraction was intentionally rejected as too
  physics-specific for FixedMathSharp. Gravitas continues to own closed-volume
  validation, center-of-mass, volume, inertia tensor, collider policy, and mesh
  runtime integration.
- FixedMathSharp now owns only the barycentric product algebra duplicated by
  Gravitas' closed-volume integration: scalar primitives on `FixedMath` and the
  symmetric product-sum matrix factory on `Fixed3x3`.

## Workstream 4: GridForge Traversal And Topology Extraction

**Goal:** Move voxel traversal uniqueness and topology cell-edge helpers into
GridForge.

Tasks:

- [x] Add GridForge tests equivalent to current
      `tests/Gravitas.Tests/Support/GridForgeTraversalTests.cs`.

- [x] Move `GridTopologyMetricUtility` behavior into GridForge with public or
      internal APIs appropriate for GridForge consumers.

- [x] Move `GridForgeTraversalState`, traversal padding mode, unique-partition
      lookup, and padded-bounds tests into GridForge.

- [x] Preserve the distinction between 3D max-cell-edge padding and X/Z planar
      max-cell-edge padding.

- [x] Update Gravitas services and queries to consume GridForge-owned helpers.

- [x] Remove local Gravitas traversal/topology files after all usages compile.

- [x] Run GridForge traversal tests.

- [x] Run Gravitas partition/query/CCD focused tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Partition|FullyQualifiedName~Query|FullyQualifiedName~ContinuousCollision|FullyQualifiedName~Mixed"
```

Expected result: traversal behavior is unchanged, but GridForge owns the grid
topology knowledge.

Notes:

- GridForge owns the public traversal surface as `GridTraversal`,
  `GridTraversalState`, and `GridTraversalPaddingMode` under
  `GridForge.Utility`. This avoids the stuttered `GridForgeTraversal` name once
  the helper lives inside GridForge.
- GridForge owns `GridTopologyMetricUtility` under `GridForge.Grids.Topology`,
  including full 3D, planar X/Z, and representative world cell-edge
  measurements.
- Gravitas no longer carries local traversal/topology helper files or local
  helper-specific tests; equivalent behavior is covered in GridForge.

## Workstream 5: Chronicler DefaultSaver Extraction

**Goal:** Make the save/apply phase base reusable from Chronicler.Core.

Tasks:

- [x] Add Chronicler tests or compile coverage for a derived saver that observes
      `Save`, `EarlyApply`, `Apply`, and `LateApply` calling the matching
      protected hooks.

- [x] Move `DefaultSaver` into Chronicler.Core using a namespace that future LSF
      libraries can consume without referencing Gravitas.

- [x] Update `PhysicsSettingsSaver` to inherit the Chronicler-owned base.

- [x] Delete the local Gravitas `DefaultSaver` file.

- [x] Run Chronicler tests or build validation.

- [x] Run Gravitas settings/serialization tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Settings|FullyQualifiedName~Serialization"
```

Expected result: Gravitas settings save/apply behavior is unchanged, and
Chronicler owns the common lifecycle base.

Notes:

- `DefaultSaver` now lives in the root `Chronicler` namespace to match
  Chronicler's existing public namespace guidance.
- SwiftCollections is the lowest LSF library in this extraction chain that
  references Chronicler, so local Chronicler project references were added to
  SwiftCollections, SwiftCollections tests/benchmarks, Gravitas, and Gravitas
  tests/benchmarks until package releases caught up. Final Gravitas validation
  uses package references again.
- Serial Release and ReleaseLean validation stayed clean after the extraction.

## Workstream 6: Gravitas Cleanup, Docs, And Release Validation

**Goal:** Finish integration and prove both standard and Lean package paths are
clean.

Tasks:

- [x] Remove obsolete Gravitas namespaces/usings from source and tests.

- [x] Update `README.md`, `AGENTS.md`, and `docs/wiki` where they describe
      source ownership for transforms, mesh math, GridForge traversal, or saver
      bases.

- [x] Verify no stale local type references remain:

```bash
rg -n "Gravitas.Support.FixedTransform|class FixedTransform|class GridForgeTraversal|class GridTopologyMetricUtility|class DefaultSaver" src tests
```

- [x] Run Gravitas full Release validation:

```bash
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

- [x] Run Gravitas full ReleaseLean validation:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

- [x] Confirm package references are restored after the lower-stack release
      sequence.

Expected result: lower-stack APIs are the source of truth, Gravitas has no
duplicate utility implementations, and Release/ReleaseLean are clean.

Notes:

- Removed the duplicate Gravitas-side `FixedTransformTests`; FixedMathSharp owns
  transform behavior coverage.
- `README.md`, `AGENTS.md`, and the wiki now state that FixedMathSharp owns the
  shared transform and barycentric product helpers, GridForge owns traversal and
  topology metrics, and Chronicler owns `DefaultSaver`.
- `Gravitas.slnx` contains only Gravitas projects, while the main package uses
  `FixedMathSharp`/`SwiftCollections` `5.0.2`, `GridForge` `7.1.2`, and
  `Chronicler.Core` `0.2.1` package references plus matching Lean packages.
- Validation completed with
  `dotnet build Gravitas.slnx --configuration Release`,
  `dotnet test Gravitas.slnx --configuration Release`,
  `dotnet build Gravitas.slnx --configuration ReleaseLean`, and
  `dotnet test Gravitas.slnx --configuration ReleaseLean`.

## Exit Criteria

- FixedMathSharp exposes the shared deterministic transform shell needed by
  Gravitas and Trailblazer.
- FixedMathSharp owns reusable barycentric product algebra.
- Gravitas owns closed-volume mesh validation and mesh mass-property math.
- Gravitas `PhysicsMesh` remains the collider/runtime wrapper and no longer owns
  duplicated reusable barycentric product formulas.
- GridForge owns traversal and topology metric helpers used by Gravitas.
- Chronicler owns `DefaultSaver`.
- Gravitas docs identify the new source-of-truth libraries.
- Gravitas Release and ReleaseLean builds/tests pass.
