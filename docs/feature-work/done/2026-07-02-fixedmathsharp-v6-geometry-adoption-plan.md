# FixedMathSharp v6 Geometry Adoption Implementation Plan

**Date:** 2026-07-02  
**Status:** Done  
**Owner:** Gravitas lower-stack geometry migration hardening

---

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [x]`) syntax for tracking.

**Goal:** Adopt FixedMathSharp v6 geometry primitives in Gravitas where they
reduce duplicated geometry logic without weakening deterministic physics
behavior or hot-path performance.

**Architecture:** Use `FixedTriangle` as the canonical ordered 3D triangle value
in mesh and mixed collision paths, while keeping Gravitas-owned cached normals
and query bounds where profiling proves they matter. Treat `FixedBoundCircle`,
`FixedSegment`, `FixedSegment2d`, and `FixedRay2d` as evidence-gated cleanup
targets rather than a broad mechanical rewrite.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp v6 geometry
primitives, SwiftCollections query buffers, GridForge-backed physics partitions,
Gravitas 3D/pure 2D/mixed collision and query services.

## Purpose

FixedMathSharp v6 introduced true reusable geometry primitives that were shaped
partly by Gravitas' needs:

- `FixedTriangle`
- `FixedTriangle2d`
- `FixedSegment`
- `FixedSegment2d`
- `FixedRay2d`
- `FixedBoundCircle`
- true `FixedBoundArea` for `Vector2d` planar bounds

The first Gravitas package-reference migration handled compile-time API breaks
and obvious bounds fixes. This plan captures the next cleanup layer: removing
local triangle/segment geometry duplication where the new lower-stack primitives
are the better long-term owner.

The migration should improve code navigation and reduce duplicated math without
introducing hidden allocations, looser contact behavior, or extra per-candidate
normal/bounds recomputation in mesh-heavy hot paths.

## Source Context

Read these before implementation:

- FixedMathSharp plan:
  `F:/gamedevrepos/FixedMathSharp/docs/feature-work/done/2026-06-28-bounds-and-2d-geometry-hardening-plan.md`
- FixedMathSharp migration notes:
  `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`
- Gravitas active release scope: `docs/feature-work/feature-work-overview.md`
- Gravitas benchmark guide: `tests/Gravitas.Benchmarks/README.md`

Important FixedMathSharp v6 guidance from the completed plan:

- Local Gravitas `TriangleData` and `MixedTriangle` can migrate to
  `FixedTriangle`.
- Physics-specific mesh behavior, cached normals, cached query bounds, and
  collider-specific state should stay in Gravitas.
- Migration is justified only where recomputing derived normal/bounds is
  acceptable or profiling proves a Gravitas-owned cache is still justified.

## Current Baseline

- `src/Gravitas/CollisionHandling/Detection/3D/Mesh/MeshTriangleContactGenerator.cs`
  contains a private `TriangleData` struct with vertices, cached normal,
  `FixedBoundVolume` bounds, centroid, and edge-vector access.
- `src/Gravitas/CollisionHandling/Detection/Mixed/MixedTriangle.cs` contains a
  similar local struct for mixed mesh-vs-slab checks.
- Both structs duplicate pure triangle geometry now owned by `FixedTriangle`,
  but both also carry cached mesh normals and SwiftCollections query bounds.
- `src/Gravitas/Colliders/Mesh/MeshUtils.cs` still owns triangle closest-point,
  in-plane containment, and edge closest-point helpers. Some of those overlap
  with `FixedTriangle` and `FixedSegment`.
- `PhysicsMesh` still uses `FixedBoundVolume` for SwiftCollections BVH queries.
  That is expected because the lower-stack query structures currently use
  `FixedBoundVolume`.
- Pure 2D collider bounds already use `FixedBoundArea` after the v6 migration.

## Non-Goals

- Do not replace physics-specific mesh, collider, material, contact, CCD, or
  query behavior with FixedMathSharp types.
- Do not force `FixedBoundBox` into SwiftCollections BVH call sites that still
  require `FixedBoundVolume`.
- Do not remove cached mesh normals from Gravitas unless tests and benchmarks
  show recomputing `FixedTriangle.Normal` is not a regression.
- Do not mechanically replace every segment calculation with `FixedSegment`.
  Two-segment closest-point routines and algorithm-specific simplex state should
  remain in Gravitas unless FixedMathSharp gains matching primitives.
- Do not migrate `SweepTriangleCandidate`, `TriangleWeights`, or
  `PhysicsMesh.TriangleUse`; those are algorithm/topology state, not geometry
  primitives.
- Do not add compatibility wrappers for old local triangle names if the new
  shared type is cleaner.

## Guiding Rules

- Keep deterministic ordering and fixed-point math unchanged.
- Prefer one shared internal Gravitas collision-triangle wrapper over two
  duplicated local triangle structs.
- Preserve hot-path shape:
  - no managed allocations after warmup.
  - no avoidable array creation.
  - no LINQ.
  - no reflection.
  - no hidden sorting or collection churn.
- Treat lower-stack primitives as value math, not ownership containers for
  runtime physics state.
- Add equivalence tests before replacing geometry helper behavior that can
  change contact points.
- Capture benchmark/allocation evidence before and after any hot-path cleanup.

## Proposed Internal Shape

Create one shared internal mesh/collision triangle value under collision
detection, for example:

```csharp
internal readonly struct CollisionTriangle
{
    public CollisionTriangle(FixedTriangle triangle, Vector3d normal, FixedBoundVolume queryBounds)
    {
        Triangle = triangle;
        Normal = normal;
        QueryBounds = queryBounds;
    }

    public FixedTriangle Triangle { get; }

    public Vector3d Normal { get; }

    public FixedBoundVolume QueryBounds { get; }

    public Vector3d A => Triangle.A;

    public Vector3d B => Triangle.B;

    public Vector3d C => Triangle.C;

    public Vector3d Center => Triangle.Centroid;

    public Vector3d GetEdgeVector(int index) => Triangle.GetEdge(index).Delta;
}
```

The exact filename and namespace should follow the existing collision folder
organization. A good first target is:

- Create:
  `src/Gravitas/CollisionHandling/Detection/3D/Mesh/CollisionTriangle.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Detection/3D/Mesh/MeshTriangleContactGenerator.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Detection/Mixed/CollisionDetectionMixed.Complex.cs`
- Delete: `src/Gravitas/CollisionHandling/Detection/Mixed/MixedTriangle.cs`

If mixed detection should not depend on a file physically under `3D/Mesh`, move
the new type to a shared detection folder such as:

- `src/Gravitas/CollisionHandling/Detection/Geometry/CollisionTriangle.cs`

Prefer the shared folder if it avoids namespace confusion.

## Workstream 1: Inventory And Baseline Evidence

**Problem**

The v6 migration should be measured before refactoring hot collision paths. The
first pass needs to lock down which local geometry types are safe to migrate and
which are algorithm-specific.

**Tasks**

- [x] Confirm the worktree only contains the user's intended package-reference
      migration state before changing files.
- [x] Inventory local triangle and segment-like helpers with:

```bash
rg -n "TriangleData|MixedTriangle|ClosestPointOnTriangle|ClosestPointOnEdge|ClosestPointOnLineSegment|FixedSegment|FixedTriangle|FixedBoundCircle|FixedRay2d" src tests
```

- [x] Record the no-change decision for these algorithm-specific types in this
      plan's completion notes when finished:
  - `SweepTriangleCandidate`
  - `TriangleWeights`
  - `PhysicsMesh.TriangleUse`
- [x] Run focused tests before source changes:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mesh|FullyQualifiedName~Mixed"
```

- [x] Build the benchmark project before source changes:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
```

- [x] Capture short allocation/perf smoke for affected benchmark families:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection collision-response query-service collider-shape --filter "*" -j Short -i --exporters json
```

**Done Criteria**

- Local geometry migration candidates are documented.
- Baseline tests pass before refactoring.
- Baseline benchmark smoke exists for mesh/mixed/query-sensitive paths.

## Workstream 2: Shared Collision Triangle Wrapper

**Problem**

`TriangleData` and `MixedTriangle` duplicate the same pure geometry but also
carry physics-specific cached state that should not move into FixedMathSharp.
The right migration is a shared Gravitas wrapper backed by `FixedTriangle`.

**Tasks**

- [x] Add focused tests that construct a mesh triangle through the same vertices
      used by mesh contact generation and verify:
  - `A`, `B`, and `C` preserve ordered vertices.
  - `Center` matches `(A + B + C) / 3`.
  - `GetEdgeVector(0/1/2)` matches the old `AB`, `BC`, `CA` vectors.
  - `QueryBounds` preserves the same min/max as the old `FixedBoundVolume`.
  - cached `Normal` is the normal supplied by `PhysicsMesh`, not recomputed from
    `FixedTriangle.Normal`.
- [x] Create the shared internal collision triangle wrapper using
      `FixedMathSharp.Geometry.FixedTriangle`.
- [x] Replace the nested `TriangleData` in `MeshTriangleContactGenerator.cs`
      with the shared wrapper.
- [x] Replace `MixedTriangle` in mixed mesh-vs-slab logic with the shared
      wrapper.
- [x] Delete `MixedTriangle.cs` after all references are removed.
- [x] Keep helper names explicit:
  - `Normal` for cached physics normal.
  - `QueryBounds` for `FixedBoundVolume`.
  - `Triangle.Bounds` only for derived `FixedBoundBox` when truly needed.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mesh|FullyQualifiedName~Mixed"
```

**Done Criteria**

- There is one Gravitas-owned collision triangle wrapper.
- Both 3D mesh and mixed mesh/slab paths use `FixedTriangle` for ordered
  vertices and deterministic edge access.
- Cached mesh normals and query bounds remain explicit and local to Gravitas.
- `MixedTriangle.cs` no longer exists.

## Workstream 3: Triangle Helper Equivalence And MeshUtils Cleanup

**Problem**

`MeshUtils` overlaps with `FixedTriangle` and `FixedSegment`, but it is used by
collision, query, and cone/mesh paths. Replacing it blindly can move contact
points because the current helper projects onto a supplied cached normal before
falling back to edges.

**Tasks**

- [x] Add equivalence tests for `MeshUtils.ClosestPointOnTriangle(...)` and
      `FixedTriangle.ClosestPoint(...)` using:
  - point above triangle interior.
  - point outside each edge.
  - point nearest each vertex.
  - degenerate triangle with a repeated vertex.
  - triangle whose supplied cached normal matches the vertex winding.
- [x] Add a regression test for zero-length edge handling. The current
      `MeshUtils.ClosestPointOnEdge(...)` divides by segment length squared;
      `FixedSegment.ClosestPoint(...)` deterministically returns `Start` for
      zero-length segments.
- [x] Decide from the tests whether `MeshUtils.ClosestPointOnTriangle(...)` can
      delegate to `FixedTriangle.ClosestPoint(...)`.
- [x] If behavior matches or improves correctness, update `MeshUtils` to route
      edge closest-point work through `FixedSegment`.
- [x] If triangle closest-point behavior differs in a way that affects mesh
      contacts, keep the current normal-projection triangle helper but replace
      only the zero-length unsafe edge helper with `FixedSegment`.
- [x] Run focused query and collision tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mesh|FullyQualifiedName~Cone|FullyQualifiedName~Query|FullyQualifiedName~Mixed"
```

**Done Criteria**

- Mesh closest-point behavior is either preserved with evidence or deliberately
  improved with regression tests.
- Zero-length edge behavior is deterministic.
- Any retained `MeshUtils` behavior is documented by tests rather than inertia.

## Workstream 4: Evidence-Gated Segment And Circle Primitive Adoption

**Problem**

`FixedSegment`, `FixedSegment2d`, `FixedRay2d`, and `FixedBoundCircle` may clean
up query and collider helpers, but a broad rewrite would add churn without
guaranteed runtime value.

**Tasks**

- [x] Inventory pure 2D circle query helpers that pass `center + radius` as a
      conceptual circular bound.
- [x] Use `FixedBoundCircle` only where it removes duplicated bound/containment
      math without increasing per-candidate construction cost.
- [x] Inventory 3D and 2D finite segment call sites.
- [x] Use `FixedSegment` or `FixedSegment2d` where the domain object is a finite
      segment and the replacement removes unsafe or duplicated edge math.
- [x] Do not replace two-segment closest-point routines unless a lower-stack
      primitive exists for that exact operation.
- [x] Leave `FixedRay2d` adoption to pure 2D query code only if it simplifies
      current raycast request handling without changing public query semantics.
- [x] Run focused 2D/query tests after any adoption:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Physics2D|FullyQualifiedName~Query|FullyQualifiedName~Capsule|FullyQualifiedName~Grounding2D"
```

**Done Criteria**

- New FixedMathSharp geometry primitives are used only where they make code
  clearer or safer.
- No public Gravitas query semantics change as a side effect.
- Any skipped migration candidates are listed in completion notes with a reason.

## Workstream 5: Benchmarks, Allocation Guardrails, Docs, And Closure

**Problem**

This plan touches mesh and mixed collision paths that are performance-sensitive.
The final pass must prove the cleanup did not trade duplicated code for hidden
runtime cost.

**Tasks**

- [x] Run full release tests:

```bash
dotnet test Gravitas.slnx --configuration Release
```

- [x] Run lean release tests because package-reference migration and geometry
      cleanup can expose conditional build issues:

```bash
dotnet test Gravitas.slnx --configuration ReleaseLean
```

- [x] Re-run affected benchmark smoke:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection collision-response query-service collider-shape --filter "*" -j Short -i --exporters json
```

- [x] Compare the post-change benchmark JSON/console output against the
      Workstream 1 baseline:
  - no new managed allocations in steady-state rows.
  - no obvious mesh/mixed timing regression.
  - any noisy timing deltas are called out honestly rather than overclaimed.
- [x] Update docs only if public behavior, developer guidance, or lower-stack
      migration guidance changes:
  - `docs/wiki/COLLISION_PIPELINE.md`
  - `docs/wiki/QUERY_SERVICES.md`
  - `docs/wiki/RUNTIME_ARCHITECTURE.md`
  - `tests/Gravitas.Benchmarks/README.md`
- [x] Update this plan with completion notes:
  - adopted primitives.
  - intentionally skipped candidates.
  - test commands and results.
  - benchmark/allocation evidence.
- [x] Mark this plan `Done` and move it to `docs/feature-work/done` after
      review.
- [x] Update `docs/feature-work/feature-work-overview.md` to move the plan from
      active release scope to recently completed.

**Done Criteria**

- The migration has tests and benchmark evidence.
- Triangle duplication is removed or explicitly justified if any local wrapper
  remains.
- The final codebase has no stale `TriangleData` or `MixedTriangle` local
  geometry models.
- Any remaining FixedMathSharp v6 geometry adoption opportunities are captured
  as explicit completion notes, not hidden in memory.

## Review Checklist

- [x] `rg -n "TriangleData|MixedTriangle" src tests` returns no active source
      references.
- [x] `rg -n "ClosestPointOnEdge" src tests` shows either no local helper or a
      tested reason for keeping one.
- [x] Cached mesh normal behavior remains explicit in Gravitas.
- [x] `FixedBoundVolume` remains at SwiftCollections BVH/query boundaries where
      required.
- [x] No new public APIs duplicate an existing FixedMathSharp primitive without
      a Gravitas-specific reason.
- [x] No hot path uses LINQ, iterator blocks, or temporary arrays.
- [x] Release and ReleaseLean tests pass.
- [x] Benchmark smoke preserves the baseline allocation profile for affected
      rows.

## Completion Notes

Completed 2026-07-02.

Adopted:

- Added `CollisionTriangle`, a shared Gravitas-owned wrapper backed by
  `FixedTriangle` while preserving cached mesh normals and `FixedBoundVolume`
  query bounds at SwiftCollections BVH boundaries.
- Replaced the 3D mesh `TriangleData` and mixed `MixedTriangle` local geometry
  models with `CollisionTriangle`; deleted `MixedTriangle.cs`.
- Routed `MeshUtils.ClosestPointOnEdge(...)` through `FixedSegment` and added a
  zero-length edge regression test.
- Centralized duplicated tolerant 2D closest-point-on-segment math in
  `PlanarSegmentGeometry`. `FixedSegment2d` was evaluated but not used in those
  hot paths because Gravitas intentionally collapses near-zero segments with
  `Fixed64.Epsilon`, while `FixedSegment2d` only collapses exactly zero-length
  segments.

Intentionally skipped:

- `MeshUtils.ClosestPointOnTriangle(...)` remains Gravitas-owned because the
  current helper is normal-aware and projects against cached mesh normals before
  edge fallback. Equivalence tests cover the non-degenerate and repeated-vertex
  cases used for the decision.
- `FixedBoundCircle` was not introduced into 2D overlap query bounds. Gravitas'
  public query API rejects negative radii explicitly, while `FixedBoundCircle`
  normalizes negative radii by design; the current direct min/max candidate
  bounds keep the public invariant clear without extra construction.
- `FixedRay2d` was not adopted because pure 2D raycasts are finite segment
  queries with segment-length hit ordering, not unbounded ray requests.
- Two-segment closest-point, sweep reducer, simplex, and slab routines remain in
  Gravitas because they carry collision/query-specific ordering and time-of-hit
  semantics.
- `SweepTriangleCandidate`, `TriangleWeights`, and `PhysicsMesh.TriangleUse`
  remain algorithm/topology state rather than geometry primitives.

Validation:

- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mesh|FullyQualifiedName~Mixed"`:
  passed 222 tests before refactor.
- `dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0`:
  passed before and after refactor.
- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mesh|FullyQualifiedName~Cone|FullyQualifiedName~Query|FullyQualifiedName~Mixed"`:
  passed 315 tests.
- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Physics2D|FullyQualifiedName~Query|FullyQualifiedName~Capsule|FullyQualifiedName~Grounding2D|FullyQualifiedName~Mixed"`:
  passed 402 tests.
- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PlanarSegmentGeometryTests|FullyQualifiedName~CollisionTriangleTests|FullyQualifiedName~MeshUtilsTests"`:
  passed 13 tests.
- `dotnet test Gravitas.slnx --configuration Release`: passed 917 tests.
- `dotnet test Gravitas.slnx --configuration ReleaseLean`: passed 901 tests.

Benchmark smoke:

- Command:
  `dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection collision-response query-service collider-shape --filter "*" -j Short -i --exporters json`
- Result: completed 69 benchmarks successfully. Baseline JSON artifacts were
  preserved under
  `BenchmarkDotNet.Artifacts/results/fixedmathsharp-v6-geometry-baseline` before
  the post-change run.
- Key comparisons, baseline to post-change:
  - `MoveMeshRuntimeShapeStateAndQueryTriangles`: 9.046 us to 8.566 us, 0 B to 0
    B.
  - `MoveDynamicConcaveMeshAndQueryTriangles`: 35.753 us to 36.480 us, 0 B to 0
    B.
  - `CheckMeshCylinderPairs`: 476.226 us to 480.236 us, 0 B to 0 B.
  - `GenerateMeshCylinderManifolds`: 476.439 us to 480.925 us, 0 B to 1 B. The 1
    B value is a MemoryDiagnoser rounding artifact from 960 B benchmark
    accounting over 1024 operations; no GC collections were reported.
  - `CheckMeshMeshPairs`: 492.486 us to 487.514 us, 0 B to 0 B.
  - `CheckClosedDenseMeshMeshPairs`: 303081.867 us to 294214.667 us, unchanged
    480 B MemoryDiagnoser artifact.
  - `SweepSphereAllAcrossMeshTargetContext`: 880.878 us to 876.831 us, unchanged
    1 B artifact.
  - `SweepConvexMeshAllAcrossSphereTargets_HighVertexSource`: 2667.851 us to
    2641.299 us, unchanged 4 B artifact.
- Conclusion: no obvious mesh/mixed/query timing regression. Allocation profiles
  are unchanged for the relevant steady-state rows aside from documented
  ShortRun/MemoryDiagnoser artifacts.
