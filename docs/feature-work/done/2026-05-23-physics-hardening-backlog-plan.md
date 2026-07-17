# Physics Hardening Backlog Action Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Status:** Done. Superseded by
`docs/feature-work/2026-05-25-alpha-physics-hardening-plan.md`.

**Goal:** Convert the former inline maintenance notes and current wiki prototype
edges into a prioritized alpha-hardening backlog.

**Architecture:** Work from contracts and tests outward: layer/query semantics
first, then collider shape state, narrow-phase detection, response,
body/grounding behavior, broad-phase culling, swept queries, mesh completion,
collision allocation cleanup, and diagnostics. Each phase should leave source
comments clean, docs/wiki current, and the affected hot paths covered by tests
plus benchmarks where complexity or allocation risk changes.

**Tech Stack:** C# 11, `FixedMathSharp`, `SwiftCollections`, `GridForge`, xUnit
v3, BenchmarkDotNet, Chronicler.

---

## Source Context Reviewed

- `src/Gravitas/Core/SolidBody.cs`
- `src/Gravitas/Support/PhysicsLayer.cs`
- `src/Gravitas/Settings/PhysicsSettings.cs`
- `src/Gravitas/Partitions/PhysicsPartition.cs`
- `src/Gravitas/Colliders/LSCollider.cs`
- `src/Gravitas/Colliders/Primitives/LSCapsuleCollider.cs`
- `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- `src/Gravitas/CollisionHandling/CollisionPair.cs`
- `src/Gravitas/CollisionHandling/CollisionResponse.cs`
- `docs/wiki/OVERVIEW.md`
- `docs/wiki/QUERY_SERVICES.md`
- `docs/wiki/HOST_INTEGRATION.md`
- `docs/wiki/COLLISION_PIPELINE.md`
- `docs/wiki/RUNTIME_ARCHITECTURE.md`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Bounds/BoundingFrustum.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/BoundingVolume/SwiftFixedBVH.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/BoundingVolume/Volume/FixedBoundVolume.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/Octree/SwiftFixedOctree.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/SpatialHash/SwiftFixedSpatialHash.cs`
- `../SwiftCollections/src/SwiftCollections/Observable`
- `../SwiftCollections/src/SwiftCollections/Query`
- `../GridForge/src/GridForge/Grids/Nodes/Voxel.cs`
- `../GridForge/src/GridForge/Spatial/PartitionProvider.cs`

## Cleanup Completed While Creating This Plan

- Removed the captured inline maintenance markers from `src/Gravitas`.
- Removed dead commented scaffolding around `LSCollider.IsInCollision`.
- Removed stale engine-specific debug draw comment blocks from body, detection,
  pair, and response code.
- Updated wiki docs that were stale after the allocation hardening refactor:
  - `docs/wiki/RUNTIME_ARCHITECTURE.md` no longer lists query hit buffers owned
    by query services.
  - `docs/wiki/COLLISION_PIPELINE.md` now describes direct spatial-cell/voxel
    scanning instead of `GridTracer.GetCoveredVoxels(...)`.
  - `docs/wiki/OVERVIEW.md` points to the completed allocation plan under
    `docs/feature-work/done`.

## Downstream Library Notes

- FixedMathSharp v4.0.0 includes deterministic `FixedRay`, `FixedPlane`,
  `BoundingFrustum`, typed containment/intersection APIs for bounds, expanded
  `Fixed4x4`, and `Vector4d`. Query and collider refactors should review these
  before adding custom ray, plane, bounds, or matrix logic.
- SwiftCollections already provides a FixedMathSharp query module:
  `SwiftFixedBVH<T>`, `SwiftFixedOctree<T>`, `SwiftFixedSpatialHash<T>`, and
  `FixedBoundVolume`. Prefer that module before using the generic
  `SwiftCollections.Query` numerics-backed convenience types or building a
  Gravitas-local fixed-point adapter.
- `FixedBoundVolume` is min/max based. Phase 3 verified mesh BVH triangle bounds
  against that contract; keep future query bounds min/max based.
- `SwiftCollections.Observable` may be useful for host-facing diagnostics,
  editor tooling, or presentation binding. Keep observable notifications out of
  authoritative simulation hot paths unless deterministic ordering, allocation
  behavior, and cost are covered by tests and benchmarks.

## Captured Inline Note Inventory

| Area                          | Captured concern                                                                                      | Priority | Destination                                        |
| ----------------------------- | ----------------------------------------------------------------------------------------------------- | -------: | -------------------------------------------------- |
| `SingleLayer`                 | Type name and semantics blur layer index, bitmask, and host-layer metadata.                           |       P0 | Phase 1 complete                                   |
| `PhysicsSettings`             | Collision matrix uses `bool[,]`; ground-check mask is legacy and not clearly configured.              |       P0 | Phase 1 complete                                   |
| `docs/wiki/QUERY_SERVICES.md` | Query layer parameter behaves like include mask despite `ignoreLayers` naming.                        |       P0 | Phase 1 complete                                   |
| `docs/wiki/QUERY_SERVICES.md` | Horizontal raycasts are rejected by the height-slope path.                                            |       P0 | Phase 2 complete                                   |
| `docs/wiki/QUERY_SERVICES.md` | Former circle-cast behavior was a proximity query, not a true swept shape query.                      |       P0 | Phase 2 complete; true sweep Phase 8               |
| `LSCollider`                  | Mesh rotation and bounds are not physically trustworthy enough.                                       |       P0 | Phase 3 complete                                   |
| `LSCapsuleCollider`           | Capsule derived points need deterministic invalidation/rebuild when size inputs change.               |       P0 | Phase 3 complete                                   |
| `LSCapsuleCollider`           | Default capsule dimensions can produce a zero cylinder-height inertia tensor diagonal.                |       P0 | Phase 3 complete                                   |
| `PhysicsMesh`                 | Mesh input validation is missing.                                                                     |       P0 | Phase 3 complete                                   |
| `PhysicsMesh`                 | Fixed query BVH bounds construction needs min/max verification.                                       |       P0 | Phase 3 complete                                   |
| `LSMeshCollider`              | Mesh collider limits, dynamic support, convexity, and ray overlap need explicit policy.               |       P0 | Phase 3 and Phase 4 foundation; Phase 8 completion |
| `docs/wiki/OVERVIEW.md`       | Cylinder collider behavior was documented as unimplemented after Phase 4.                             |       P1 | Phase 4 complete; docs cleanup Phase 8 complete    |
| `CollisionDetection`          | Detection needs engine-agnostic tests and instrumentation, not Unity debug draw leftovers.            |       P1 | Phase 4 complete; Phase 9 and Phase 10             |
| `CollisionResponse`           | Response is prototype-level and needs physical solver hardening.                                      |       P1 | Phase 5 complete                                   |
| `CollisionPair`               | Time-spaced culling can miss behavior; culling should account for distance, velocity, and pair state. |       P1 | Phase 7 complete                                   |
| `LSCollider`                  | Teleports should invalidate culling assumptions.                                                      |       P1 | Phase 7 complete                                   |
| `PhysicsPartition`            | Dynamic-object removal is linear.                                                                     |       P2 | Phase 7 complete                                   |
| `SolidBody`                   | Initialization assumes grounded instead of deriving it from the world.                                |       P1 | Phase 6 complete                                   |
| `SolidBody`                   | Rotation visualization interpolation needs a clear speed/time contract.                               |       P2 | Phase 6 complete                                   |
| `SolidBody`                   | Force/ground debug visualization should become engine-agnostic diagnostics.                           |       P2 | Phase 10                                           |
| `PhysicsMesh`                 | Edge cache may be removable if face normals and on-demand edge normals cover all callers.             |       P2 | Phase 8 reviewed; retained for SAT                 |

## Recommendations

- Phase 1 resolved the `SingleLayer` hazard by splitting collider layer identity
  from query/ground-check bitmask membership. Keep new code on
  `PhysicsLayer`/`PhysicsLayerMask` rather than reintroducing ambiguous layer
  helpers.
- Do not begin a broad collision-response rewrite until narrow-phase shape-pair
  tests exist. Response bugs are difficult to diagnose if contact normals and
  depths are not already pinned.
- Split collider responsibilities deliberately. `LSCollider` currently owns
  identity, host binding, shape state, partition state, pair references,
  hierarchy filtering, query versions, and events. That is workable for a
  prototype but too dense for alpha hardening.
- Keep mesh collider dynamic support behind tests and benchmarks. Mesh work can
  become the whole project if it is not boxed into validation, limits, bounds,
  and query/collision behavior.
- Treat the current circle query service as overlap/proximity behavior only.
  Phase 8 added true deterministic swept-sphere queries; keep shape sweeps on
  that path rather than routing them through `OverlapCircleInDirection`.
- Keep ground probe selection explicit. Phase 8 added `GroundProbeMode.Ray`,
  `GroundProbeMode.SweptSphere`, and `GroundProbeMode.Auto`; future grounding
  changes should preserve documented shape/size rules or update tests and docs
  with the new policy.
- Mesh ray overlap, `Mesh/Cylinder` narrow phase, mesh triangle-buffer
  ownership, and mesh contact normals were closed before diagnostics. Mesh
  swept-sphere targets remain deliberately unsupported until a triangle sweep
  policy and acceleration strategy are designed.
- Prefer downstream deterministic primitives before adding local equivalents.
  FixedMathSharp geometry and `SwiftCollections.FixedMathSharp` query structures
  should be the default starting point; if a custom Gravitas structure is
  better, prove it with tests, benchmarks, and a short design note.
- GridForge voxel partition providers are part of Gravitas' broad-phase cost
  model. Phase 7 exposed allocation churn in the packaged GridForge 6.0.4
  `PartitionProvider.TryRemove(...)` behavior when a voxel repeatedly loses and
  regains a partition. The sibling GridForge source now has a regression test
  and retention fix; Gravitas should consume the next fixed GridForge package
  before treating end-to-end repartition allocation as solved.
- Do not start diagnostics on top of unexplained collision allocation. Phase 9
  now isolates the remaining `CheckPreparedPrimitivePairs` allocation baseline
  before Phase 10 records collision events.
- Use engine-agnostic diagnostics rather than adding editor hooks to runtime
  classes. Diagnostics should report deterministic values and let hosts decide
  how to draw them.
- Maintain `docs/wiki/` with each phase. The wiki is now useful enough that
  stale pages will mislead the next implementation pass.

## Phase 0: Baseline Test And Benchmark Harness

**Purpose:** Create enough harness coverage to safely refactor collider and
collision internals.

**Files:**

- Create: `tests/Gravitas.Tests/Support/PhysicsScenarioBuilder.cs`
- Create:
  `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Create:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- Create:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Create:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionResponseBenchmarks.cs`
- Modify: `tests/Gravitas.Benchmarks/Support/BenchmarkCatalog.cs` only if alias
  discovery needs new namespace handling.

**Tasks:**

- [x] Add a scenario builder that creates a `GravitasWorldContext`, grid
      coverage, `TestMatterAgent`, body, and collider combinations with fixed
      positions and fixed rotations.
- [x] Add detection tests for supported shape pairs using separated,
      edge-touching, overlapping, degenerate, and rotated cases.
- [x] Add response invariant tests for immovable-vs-dynamic, dynamic-vs-dynamic,
      trigger-vs-solid, restitution zero, and nonzero angular velocity cases.
- [x] Add short benchmarks for narrow-phase dispatch and response solver paths
      using 64 deterministic pairs.
- [x] Run
      `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionDetectionShapePairTests|FullyQualifiedName~CollisionResponseInvariantTests"`.
- [x] Run
      `dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collision-detection collision-response --filter "*" -j Short -i --exporters json`.

**Phase 0 completion notes:**

- Added `PhysicsScenarioBuilder`, primitive narrow-phase tests, response
  invariant tests, and `collision-detection`/`collision-response` benchmark
  aliases.
- Fixed three small issues exposed by the new baseline:
  - `SolidBody.Setup(...)` no longer evaluates stateful collider `Shape` before
    the collider has a transform/body.
  - capsule/capsule detection now handles degenerate capsule line segments
    deterministically.
  - trigger pairs now skip physical response in both pair dispatch and direct
    `CollisionResponse.CalculateImpulse(...)` calls.
- Short benchmark smoke on this machine:
  - `CollisionDetectionBenchmarks.CheckPreparedPrimitivePairs`: 64 pairs, mean
    about `53.44 us`, allocated about `3 KB`.
  - `CollisionResponseBenchmarks.CalculateImpulseForPreparedPairs`: 64 pairs,
    mean about `475.4 us`, allocated about `1.27 KB`.
- BenchmarkDotNet could not raise process priority in this sandbox and warned
  that the response benchmark iteration time is short. Treat these as smoke
  baselines, not canonical performance numbers.

## Phase 1: Layer And Settings Contract

**Purpose:** Remove ambiguity between layer index, layer mask, collision matrix,
and ground-check filtering before those concepts spread into more tests.

**Files:**

- Delete: `src/Gravitas/Support/SingleLayer.cs`
- Create: `src/Gravitas/Support/PhysicsLayer.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Modify: `src/Gravitas/Raycasting/GravitasCircleQueryService.cs`
- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Create: `tests/Gravitas.Tests/Settings/PhysicsLayerTests.cs`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`

**Tasks:**

- [x] Introduce explicit types for layer index and layer mask. Recommended
      names: `PhysicsLayer` for one index and `PhysicsLayerMask` for bitmask
      queries.
- [x] Replace query parameters named `ignoreLayers` with names that match
      observed behavior: `includedLayers` or `layerMask`.
- [x] Replace the legacy hard-coded `IgnoreForGroundCheck` default with a
      settings-owned mask that defaults to a documented value.
- [x] Decide whether the collision matrix remains `bool[,]` or moves to a
      SwiftCollections-backed bitset. If it changes, add benchmarks for layer
      lookups during pair filtering.
- [x] Add tests for single layer inclusion, multi-layer inclusion, include-all,
      include-none, collision matrix allow/deny, and ground-check layer
      filtering.
- [x] Update wiki examples to use the new names and semantics.
- [x] Run the focused Phase 1 layer/query/settings test slice before the Phase 2
      circle-query rename.

**Phase 1 completion notes:**

- Deleted `SingleLayer` and added `PhysicsLayer`/`PhysicsLayerMask` so collider
  layer identity is separate from include-mask filtering.
- Query APIs now use `layerMask` names while preserving the existing include
  semantics.
- Replaced `IgnoreForGroundCheck` with `PhysicsSettings.GroundCheckLayerMask`.
  The default keeps the old prototype exclusions only as a documented example;
  hosts should configure the mask explicitly for their own layer model.
- Kept the collision matrix as `bool[,]` for now because this phase did not
  produce evidence that a bitset migration would improve the pair-filtering hot
  path enough to justify the churn.

## Phase 2: Query Semantics

**Purpose:** Make raycasts and circlecasts physically named and deterministic
before collision and grounding work depends on them.

**Files:**

- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Rename/modify: `src/Gravitas/Raycasting/GravitasCircleQueryService.cs`
- Rename/modify: `src/Gravitas/Raycasting/RaycastSegmentWorker.cs`
- Modify: `tests/Gravitas.Tests/Raycasting/GravitasRaycastServiceTests.cs`
- Rename/modify:
  `tests/Gravitas.Tests/Raycasting/GravitasCircleQueryServiceTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Raycasting/QueryServiceBenchmarks.cs`
- Modify: `docs/wiki/QUERY_SERVICES.md`

**Tasks:**

- [x] Replace the raycast height-slope rejection with deterministic 3D segment
      handling so horizontal, vertical, and diagonal rays all have defined
      behavior.
- [x] Evaluate `FixedRay` and FixedMathSharp bounds/plane intersection APIs
      before keeping or expanding custom ray intersection workers.
- [x] Add raycast tests for horizontal rays, vertical rays, diagonal rays,
      starting-inside-collider rays, no-hit rays, multi-hit ordering, and
      cross-context isolation.
- [x] Rename or split the current circlecast behavior. Recommended contract:
      keep the current proximity scan as an overlap query, then implement true
      swept sphere or swept circle only after the dimensional model is explicit.
- [x] Add tests proving circle/proximity hit distance, normal, point, and
      ordering.
- [x] Keep caller-owned hit buffers and the allocation-free sorter in all
      all-hit paths.
- [x] Re-run the `query-service` allocation benchmark and confirm the all-hit
      paths remain at 0 B/op in the short smoke run.

**Phase 2 completion notes:**

- Replaced the raycast height-slope path with a 3D `RaycastSegmentWorker`.
  `Raycast` normalizes direction before applying max distance, and `RaycastAll`
  now accepts horizontal, vertical, diagonal, and starting-inside segments.
- Reviewed FixedMathSharp `FixedRay` and bounds intersections. They remain a
  good future primitive for first-hit infinite-ray checks, but Gravitas keeps a
  custom segment worker here because this service needs bounded segment scans,
  all intersection points, caller-owned buffers, and deterministic hit sorting.
- Renamed the context service from the old circle-cast property to
  `CircleQueries` and the service type to `GravitasCircleQueryService`. Current
  circle behavior is now exposed as `OverlapCircle`, `OverlapCircleAll`, and
  `OverlapCircleInDirection`; true swept sphere casts are covered in Phase 8.
- Circle overlap hit data now uses the closest collider surface point, collider
  surface normal, and surface distance for ordering.
- Query docs, host examples, README/AGENTS references, and benchmark docs were
  updated with the new terminology.
- Short `query-service` benchmark smoke reported no managed allocation in the
  `Allocated` column for `RaycastAll`, `OverlapCircleAll`,
  `OverlapCircleInDirection`, and overlapping-context raycasts. BenchmarkDotNet
  still warned that this sandbox cannot raise process priority, so these are
  smoke numbers rather than canonical timing evidence.
- Also reran the `simulation-allocation` smoke because grounding used the query
  service; its scenarios also reported no managed allocation in the `Allocated`
  column. Phase 6 later moved body grounding to sorted raycast probes.

## Phase 3: Collider Shape And Runtime State Refactor

**Purpose:** Reduce `LSCollider` responsibility density and make shape-derived
state explicit, invalidatable, and testable.

**Files:**

- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSCapsuleCollider.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- Create: `tests/Gravitas.Tests/Colliders/ColliderRuntimeStateTests.cs`
- Create: `tests/Gravitas.Tests/Colliders/PhysicsMeshTests.cs`
- Create: `tests/Gravitas.Benchmarks/Colliders/ColliderShapeBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Split shape-derived data from collider identity and partition/pair state.
      Recommended first step: extract a focused internal shape-state helper
      before renaming public collider types.
- [x] Add tests that changing offset, scale, radius, height, or rotation
      invalidates and rebuilds bounds, radius, area, and capsule segment data
      exactly once per simulate step.
- [x] Fix capsule height/radius rebuild behavior and add tests for short, tall,
      scaled, and rotated capsules.
- [x] Fix capsule inertia for degenerate/default dimensions or define a
      sphere-fallback policy for zero cylinder height.
- [x] Add `PhysicsMesh` validation for null arrays, triangle-count multiples of
      three, out-of-range triangle indices, duplicate triangle indices, and
      degenerate triangles.
- [x] Fix or confirm mesh `FixedBoundVolume` construction so triangle BVH
      entries and query bounds use min/max coordinates, not center/size values.
- [x] Reuse FixedMathSharp typed bounds helpers where behavior matches tests,
      especially for closest point, containment, and intersection checks.
- [x] Define mesh collider limits in settings or collider construction.
      Recommended first policy: fail fast when vertex or triangle counts exceed
      explicit deterministic limits.
- [x] Prove whether `_edges` and cached edge normals are still required. Remove
      them only after mesh/cuboid and mesh/mesh SAT tests prove equivalent
      detection behavior.
- [x] Fix mesh bounds under rotation or make mesh collider rotation limitations
      explicit in API and tests.
- [x] Run collider tests and the new collider shape benchmark in Release.

**Phase 3 completion notes:**

- Added `ColliderRuntimeShapeState`/`ColliderShapeSnapshot` so collider
  identity, pair ownership, and partition state no longer hide the rebuild
  boundary for bounds and shape-derived caches.
- Added `LocalOffset`, `Radius`, and `Size` mutation paths that mark shape state
  dirty. Scale, position, and rotation are detected from the snapshot so several
  edits before `Simulate()` rebuild derived state once.
- Rebuilt capsule derived state as one unit: hemisphere centers, cylinder
  height, area, and segment endpoints. Short capsules now collapse the segment
  and use a sphere inertia fallback for zero cylinder height.
- Added `PhysicsMesh` input validation, deterministic vertex/triangle limits,
  local bounds, defensive source-array copies, triangle-ordinal access, and
  min/max `FixedBoundVolume` construction for triangle BVH/query bounds.
- Fixed mesh collider construction so it no longer reads runtime `Center` before
  binding, and fixed rotated mesh bounds to refresh from transformed vertices.
- Reviewed the mesh `_edges`/edge-normal cache. It is retained for now because
  mesh SAT still uses the existing context data; Phase 8 revisited the cache
  after mesh ray and mesh/cylinder work and kept it pending broader mesh
  manifold proof.
- Added the `collider-shape` benchmark selection for capsule runtime rebuilds
  and mesh validation/BVH construction.
- Short `collider-shape` benchmark smoke on this machine:
  - `RebuildCapsuleRuntimeShapeState`: 64 colliders, mean about `1.003 ms`,
    allocated about `198.46 KB` while intentionally forcing repartitioning.
  - `BuildValidatedMeshTriangleBVH`: two-triangle mesh construction plus BVH
    query, mean about `2.268 us`, allocated about `2.84 KB`.
  - BenchmarkDotNet could not raise process priority in this sandbox, so these
    are smoke numbers rather than canonical timing evidence.

## Phase 4: Narrow-Phase Collision Detection Coverage

**Purpose:** Complete and harden shape-pair detection before replacing response
behavior.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSCylinderCollider.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/Support/ColliderType.cs`
- Modify: `src/Gravitas/Colliders/Support/CollisionType.cs`
- Modify: `src/Gravitas/Colliders/ColliderSettings.cs`
- Modify: `src/Gravitas/Support/FixedTransform.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Modify: `tests/Gravitas.Tests/Raycasting/GravitasRaycastServiceTests.cs`
- Create: `tests/Gravitas.Tests/Support/FixedTransformTests.cs`
- Modify:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Build a shape-pair matrix that lists every supported, unsupported, and
      intentionally deferred pair.
- [x] Review `FixedPlane`, `FixedPlaneIntersectionType`, and FixedMathSharp
      typed bounds APIs before adding or rewriting SAT, plane classification,
      frustum, or mesh support helpers.
- [x] Implement cylinder collision behavior or remove cylinder from active
      dispatch until its tests define support.
- [x] Restore mesh ray overlap only after `PhysicsMesh` validation, bounds, and
      triangle acceleration tests exist.
- [x] Decide non-convex mesh policy. Recommended alpha policy: preprocess into
      convex sub-meshes offline or at initialization, never during per-frame
      collision.
- [x] Add tests for contact normal orientation, penetration depth sign, and
      point ordering for supported primitive pairs, with mesh contact hardening
      called out as deferred policy work.
- [x] Add tests for pair dispatch stability so collider priority changes cannot
      silently flip contact data.
- [x] Run collision detection benchmarks before and after each shape-pair
      algorithm change.

**Phase 4 completion notes:**

- Added an explicit shape-pair matrix in tests and docs. `Cylinder/Mesh` was
  intentionally deferred in Phase 4 instead of being exposed through an untested
  compatibility path, then implemented as `Mesh_Cylinder` in Phase 8.
- Reviewed FixedMathSharp deterministic geometry primitives before extending
  local SAT helpers. Phase 4 kept direct fixed-point projection helpers because
  the new cylinder support needs finite capped-cylinder projections, pair-order
  contact data, and caller-owned hot-path behavior.
- Implemented `LSCylinderCollider` as a finite flat-capped cylinder with shape
  rebuild state, cap centers, axis segment, surface area, frontal area, solid
  cylinder inertia, closest-surface, surface-normal, and ray segment overlap.
- Added cylinder narrow-phase support for cylinder/sphere, cylinder/capsule,
  cylinder/cylinder, and cuboid/cylinder. Cylinder/capsule, cylinder/cylinder,
  and cuboid/cylinder use projection tests that preserve flat cap separation.
- Hardened older primitive contact normals that could return zero or flipped
  normals when the tested point was already on or inside another shape.
- Fixed `FixedTransform.LossyScale` to use basis-vector scale extraction so
  rotated transforms do not collapse collider shape state by reading near-zero
  matrix diagonals.
- Removed the dormant mesh ray helper from `RaycastSegmentWorker`; mesh ray
  overlap remains disabled until mesh validation, acceleration, and contact
  policy tests justify restoring it.
- Documented the alpha mesh policy: non-convex meshes should be decomposed
  offline or during initialization, never during per-frame collision.
- Expanded the collision detection benchmark mix to include cylinder/sphere,
  cylinder/capsule, cylinder/cylinder, and cuboid/cylinder primitive pairs.
- Short `collision-detection` benchmark smoke on this machine:
  - `CollisionDetectionBenchmarks.CheckPreparedPrimitivePairs`: 64 pairs, mean
    about `325.2 us`, allocated about `1.5 KB`.
  - BenchmarkDotNet could not raise process priority in this sandbox, so this is
    a smoke result rather than canonical timing evidence.

## Phase 5: Collision Response Solver Redesign

**Purpose:** Replace prototype response behavior with a physically explainable
deterministic solver.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/CollisionResponse.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Modify: `src/Gravitas/Core/SolidBody.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Modify: `tests/Gravitas.Tests/Support/PhysicsScenarioBuilder.cs`
- Modify:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionResponseBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Define units and invariants for mass, inverse mass, inertia tensor,
      angular velocity, restitution, friction, drag, damping, and penetration
      correction.
- [x] Replace single-point response assumptions with an explicit contact model.
      Recommended first milestone: stable single-contact solver with room for
      contact manifolds.
- [x] Add tests for conservation expectations, restitution thresholds, immovable
      body behavior, kinematic body behavior, angular impulse direction, trigger
      exclusion, and stable resting contact.
- [x] Revisit `ContactPoint` depth clamping. Keep it only if tests prove it is a
      solver invariant rather than a hidden correction.
- [x] Add deterministic replay tests that run the same body pair for many fixed
      frames and compare final state.
- [x] Run response benchmarks and add allocation checks for active-pair
      processing after solver changes.

**Phase 5 completion notes:**

- Replaced the prototype response path with a deterministic single-contact
  solver that builds an explicit contact from pair bodies, contact points,
  relative contact arms, depth, and A-to-B normal.
- Treats `Immovable` and `IsKinematic` bodies as infinite mass for response.
  Movable bodies receive direct collision velocity deltas instead of the
  time-scaled `SolidBody.AddLinearImpulse(...)` host API.
- Removed the hidden `ContactPoint` depth floor. Contact data now stores the
  narrow-phase depth directly, including zero-depth touching contacts.
- Added `ContactPoint.HasContact` so zero-valued contact fields are not used as
  implicit valid data, and response now skips pairs whose contact has not been
  populated by narrow phase.
- Moved stabilization into response constants: `PenetrationSlop`,
  `PenetrationCorrectionPercent`, and `RestitutionVelocityThreshold`.
- Restitution is clamped to `[0, 1]`, combined by the lower participant
  coefficient, and suppressed below the resting-contact threshold to avoid
  deterministic micro-bounce.
- Position correction now applies immediately through body collision correction
  helpers and is distributed by inverse mass.
- Added response tests for exact elastic normal-velocity exchange, zero
  restitution damping, restitution-threshold resting contacts, kinematic
  infinite-mass behavior, trigger exclusion, immovable behavior, angular impulse
  direction, contact-depth storage, unset-contact response exclusion, correction
  slop, and deterministic replay.
- Updated collision detection shape tests so touching contacts require
  non-negative depth rather than the removed artificial penetration margin.
- Documented response units and the current deferral of tangential friction
  impulses, warm starting, manifolds, island solving, and continuous collision
  detection.
- Short `collision-response` benchmark smoke on this machine:
  - `CollisionResponseBenchmarks.CalculateImpulseForPreparedPairs`: 64 pairs,
    mean about `915.7 us`, allocated `0 B`.
  - BenchmarkDotNet could not raise process priority and warned that the
    iteration time is very small, so this is a smoke result rather than
    canonical timing evidence.
- Short `simulation-allocation` smoke still reported `0 B` allocated for
  active-pair processing:
  - `ActivePairProcessingLateSimulate`: 64 colliders, mean about `58.72 us`.
  - Other included smoke paths also reported `0 B` allocated:
    `SolidBodyLateSimulateOnly`, the then-current grounding query path, and
    `CollisionPartitionDistributionOnly`.

## Phase 6: Body, Grounding, And Visualization Semantics

**Purpose:** Make body state transitions deterministic and physically named
instead of relying on prototype assumptions.

**Files:**

- Modify: `src/Gravitas/Core/SolidBody.cs`
- Modify: `src/Gravitas/Runtime/GravitasClock.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Create: `tests/Gravitas.Tests/Core/SolidBodyGroundingTests.cs`
- Create: `tests/Gravitas.Tests/Core/SolidBodyIntegrationTests.cs`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`

**Tasks:**

- [x] Replace grounded-on-initialize with an explicit initial grounding probe or
      a documented default of not grounded until the first simulation step.
- [x] Rework ground checks to use the Phase 1 layer mask contract and the Phase
      2 query contract.
- [x] Add tests for grounded initialization, airborne initialization, moving
      platforms, skipped ground checks, slope normals, and layer-filtered
      ground.
- [x] Define visual rotation interpolation as either frame accumulation or
      speed-limited interpolation. Add tests for both reset accumulation and
      steady visualize frames.
- [x] Add integration tests for force, velocity, drag, friction, torque, angular
      damping, and rest-state transitions using fixed expected values.

**Phase 6 completion notes:**

- `SolidBody.Initialize(...)` now starts airborne, registers/partitions its
  collider, and then performs an explicit ground probe. Bodies only start
  grounded when the configured ground mask produces a non-self hit.
- Ground checks now use `PhysicsSettings.GroundCheckLayerMask` with
  `GravitasRaycastService.RaycastAll(...)`, write into a body-owned
  `SwiftList<LSRaycastHit>`, and ignore the body's own collider before accepting
  the closest ground hit.
- Manual `CheckGround()` forces a fresh probe. Simulation grounding still keeps
  the stationary-frame guard, but movement of the last hit platform invalidates
  that guard so moving platforms refresh ground point and height.
- `SkipGrounding(...)` now clears grounding immediately and keeps ground data
  reset during the skip window.
- Visual rotation interpolation is explicitly split:
  - frame-accumulated interpolation clamps `ExpectedAccumulation` to one
    simulation frame.
  - speed-limited interpolation advances from the current presentation rotation
    toward the authoritative target each visualize call.
- Added grounding tests for grounded/airborne initialization, moving platforms,
  skipped checks, slope normals, and layer-filtered ground.
- Added integration tests for force, velocity, linear drag, ground friction,
  torque, angular damping, rest reset, and visual rotation behavior.
- Updated `simulation-allocation` benchmark coverage to exercise the current
  grounding raycast-probe path.
- Verification:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~SolidBodyGroundingTests|FullyQualifiedName~SolidBodyIntegrationTests|FullyQualifiedName~GravitasClockTests"`:
    17 passed.
  - `dotnet build Gravitas.slnx --configuration Release`: succeeded with 0
    warnings and 0 errors.
  - `dotnet test Gravitas.slnx --configuration Release --no-build`: 98 passed.
  - `dotnet build Gravitas.slnx --configuration ReleaseLean`: succeeded with 0
    warnings and 0 errors.
  - `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build`: 98
    passed.
  - `simulation-allocation` short benchmark smoke reported no managed allocation
    in the summary `Allocated` column. Timings on this machine:
    `SolidBodyLateSimulateOnly` about `206.3 us`, `GroundingRaycastProbeOnly`
    about `137.7 us`, `CollisionPartitionDistributionOnly` about `128.7 us`, and
    `ActivePairProcessingLateSimulate` about `145.1 us`. BenchmarkDotNet could
    not raise process priority in this sandbox, so these are smoke numbers.

## Phase 7: Broad-Phase Culling And Partition Performance

**Purpose:** Keep collision candidate management low-complexity without hiding
missed contacts behind culling.

**Files:**

- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Modify: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Create:
  `tests/Gravitas.Tests/Partitions/PhysicsPartitionPerformanceShapeTests.cs`
- Create: `tests/Gravitas.Tests/CollisionHandling/CollisionPairCullingTests.cs`
- Create: `tests/Gravitas.Benchmarks/Core/PartitionCullingBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Add tests proving teleporting a collider invalidates pair culling and
      repartitions before the next collision distribution pass.
- [x] Replace dynamic-list removal only if benchmarks show churn is meaningful.
      Recommended implementation if needed: a sparse-set style structure owned
      by `PhysicsPartition`.
- [x] Capture the current GridForge partitioning baseline and do not replace
      broad-phase structures without a dedicated comparison against
      `SwiftFixedSpatialHash<T>`, `SwiftFixedOctree<T>`, or `SwiftFixedBVH<T>`.
- [x] Add tests for partition activation/deactivation transitions after dynamic
      object add/remove churn.
- [x] Redesign cull countdown using distance, relative velocity, frame count
      since collision, and partition movement state.
- [x] Add tests for fast-moving objects, large objects, recently collided pairs,
      and culling-disabled colliders.
- [x] Re-run `simulation-allocation`, `collision-partition`, and the new culling
      benchmark after every data-structure change.

**Phase 7 completion notes:**

- Added a pre-distribution collider refresh in
  `GravitasPhysicsService.Simulate()` so host-command teleports and body
  repositioning update dynamic collider bounds/partitions before collision
  distribution.
- Added collider broad-phase versions and pair-side version tracking so culling
  invalidates on position, rotation, partition, or shape/bounds movement even
  when a collider remains inside the same snapped voxel bounds.
- Reworked cull scoring so distance and frames since last contact can increase
  the delay, while relative velocity reduces it. Disabled zero-valued cull
  thresholds now skip their contribution instead of dividing by zero.
- Replaced `PhysicsPartition` dynamic/static ID membership checks and removals
  with `SwiftSparseMap<byte>` used as a sparse-set style container. Direct
  partition remove/re-add churn now reports no managed allocation in the short
  benchmark smoke and no longer needs separate list/index-map bookkeeping.
  `SwiftSparseSet` would be an even cleaner downstream fit when that collection
  exists.
- Replaced the `PhysicsPartition` inactive pool's `SwiftObjectPool` backing with
  a context-local `SwiftStack`. The old pool used `ConcurrentStack`, which was
  safe but allocated nodes during release-heavy repartition waves.
- Updated active-partition traversal to use the `SwiftBucket` enumerator path.
- Added Phase 7 tests for teleported dynamic-body repartitioning,
  cull-invalidation after movement, shape-only active-contact rechecks,
  fast-relative-velocity culling, disabled cull thresholds, and partition
  activation/deactivation churn.
- Resolved the older `CollisionPair.AssignPriority(...)` fallthrough before
  Phase 8. Pair ordering now applies shape priority first, same-priority linear
  speed second, and original candidate order as the deterministic tie-breaker.
  Added pair-order/contact-normal tests so this cannot silently flip
  narrow-phase contact data again.
- Added the `partition-culling` benchmark alias covering dynamic-sphere
  repartitioning, direct partition member churn, and culled-pair rechecks.
- Short benchmark smoke on this machine:
  - `simulation-allocation`: still reported `0 B` allocated for
    `SolidBodyLateSimulateOnly`, `GroundingRaycastProbeOnly`,
    `CollisionPartitionDistributionOnly`, and
    `ActivePairProcessingLateSimulate`.
  - `collision-partition`: `SimulatePartitionedDynamicSpheres` reported `0 B`
    allocated; registration/construction scenarios still allocate by design.
  - `partition-culling`: all three paths reported no managed allocation in the
    summary `Allocated` column. After the sparse-map partition change,
    `RemoveAndReAddDynamicPartitionMembers` reported about `619.6 ns`;
    `RecheckCulledPairAfterColliderMove` reported about `493.2 ns`; after the
    local GridForge provider fix and the context-local partition stack,
    `RepartitionTeleportedDynamicSpheres` reported about `778 us`.
- Issue surfaced and fixed across the stack: full repartition churn was first
  dominated by GridForge voxel partition-provider storage being released when
  the last partition was removed. The sibling GridForge repo now has a targeted
  regression test and fix. During hardening, `Gravitas.csproj` conditionally
  links the sibling GridForge project when it is present, while still keeping
  `GridForge`/`GridForge.Lean` 6.0.4 package dependencies in generated package
  metadata. Remove the temporary local-link default after the next fixed
  GridForge package is consumed.
- Verification:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~CollisionPairPriorityTests`:
    first failed against the old fallthrough behavior, then passed after the
    priority fix.
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionPairPriorityTests|FullyQualifiedName~CollisionPairCullingTests|FullyQualifiedName~CollisionDetectionShapePairTests|FullyQualifiedName~CollisionResponseInvariantTests"`:
    30 passed.
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionPairCullingTests|FullyQualifiedName~PhysicsPartitionPerformanceShapeTests|FullyQualifiedName~GravitasCollisionServiceTests|FullyQualifiedName~PhysicsPartitionTests|FullyQualifiedName~GravitasPhysicsServiceTests"`:
    19 passed.
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PhysicsPartitionPerformanceShapeTests|FullyQualifiedName~PhysicsPartitionTests|FullyQualifiedName~GravitasRaycastServiceTests|FullyQualifiedName~GravitasCircleQueryServiceTests|FullyQualifiedName~GravitasCollisionServiceTests"`:
    18 passed after migrating partition membership to `SwiftSparseMap<byte>`.
  - `dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- partition-culling --filter "*" -j Short -i --exporters json`:
    all three benchmark paths reported no managed allocation in the summary
    `Allocated` column.
  - `dotnet build Gravitas.slnx --configuration Release`: succeeded with 0
    warnings and 0 errors, and built the local GridForge reference as Release.
  - `dotnet test Gravitas.slnx --configuration Release --no-build`: 106 passed.
  - `dotnet build Gravitas.slnx --configuration ReleaseLean`: succeeded with 0
    warnings and 0 errors, and built the local GridForge reference as
    ReleaseLean.
  - `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build`: 106
    passed.
  - Generated package metadata was inspected: standard packages still depend on
    `GridForge 6.0.4`, and lean packages still depend on `GridForge.Lean 6.0.4`.
  - `dotnet test GridForge.slnx --configuration Debug` in the sibling GridForge
    repo after the provider-retention fix: 203 passed.

## Phase 8: Swept Queries, Ground Probe Modes, And Mesh Completion

**Purpose:** Close the query, grounding, and mesh deferrals before diagnostics
start reporting those paths as if they were final.

**Files:**

- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Create: `src/Gravitas/Raycasting/SweptSphereQueryWorker.cs`
- Modify: `src/Gravitas/Raycasting/LSRaycastHit.cs` only if hit metadata needs a
  query-shape flag or swept-radius value.
- Modify: `src/Gravitas/Core/SolidBody.cs`
- Create: `src/Gravitas/Core/GroundProbeMode.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs` if a context default
  ground-probe mode belongs in settings.
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- Modify: `src/Gravitas/Raycasting/RaycastSegmentWorker.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/Colliders/ColliderSettings.cs`
- Modify: `src/Gravitas/Colliders/Support/CollisionType.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/Context/MeshObjectInfo.cs`
- Create: `tests/Gravitas.Tests/Raycasting/GravitasSweptSphereQueryTests.cs`
- Create: `tests/Gravitas.Tests/Core/SolidBodyGroundProbeModeTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Create: `tests/Gravitas.Tests/Colliders/LSMeshColliderQueryTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Raycasting/QueryServiceBenchmarks.cs`
- Modify:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Modify: `tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/wiki/OVERVIEW.md`

**Tasks:**

- [x] Define the true swept sphere query contract before implementation. The
      contract should cover origin/end or direction/max-distance overloads,
      radius, layer mask, self-exclusion support for body-owned probes,
      starting-overlap behavior, hit normal semantics, and closest/all-hit
      ordering.
- [x] Implement swept sphere candidate gathering through the same context-owned
      partition path as raycasts and circle queries. Keep result buffers
      caller-owned and sort all-hit results deterministically by time of impact,
      then collider ID or another explicit stable tie-breaker.
- [x] Support swept sphere intersection against the primitive shapes already
      considered active for queries: sphere, capsule, cuboid, and cylinder.
      Avoid routing this through `OverlapCircleInDirection`; that method remains
      an X/Z proximity filter.
- [x] Decide and implement the mesh side of swept sphere queries only after mesh
      ray overlap and triangle acceleration are validated. If mesh sweep support
      is not ready in the same pass, document the unsupported state explicitly
      and keep mesh hits out of the swept-sphere API.
- [x] Add swept sphere tests for direct hits, grazing hits, starting overlap,
      no-hit separation, layer filtering, duplicate suppression, self-exclusion,
      deterministic all-hit sorting, horizontal/vertical/diagonal sweeps, and
      rotated collider targets.
- [x] Add swept sphere benchmarks beside the existing ray/circle query benchmark
      aliases, including an all-hit path with caller-owned buffers.
- [x] Add `GroundProbeMode` with at least `Ray`, `SweptSphere`, and `Auto`.
      Prefer body-level selection with an optional settings default rather than
      a hidden global switch.
- [x] Update `SolidBody.CheckGround()` so `Ray` preserves the Phase 6 sorted
      raycast/self-exclusion behavior, `SweptSphere` uses the new query, and
      `Auto` chooses from collider shape/size using explicit rules. A likely
      alpha policy is swept sphere for sphere/capsule/wide finite bodies and ray
      for small or intentionally point-like probes.
- [x] Add grounding tests proving ray and swept-sphere modes differ on edge and
      slope cases, while both preserve layer masks, skipped grounding, moving
      platform invalidation, and deterministic state after repeated runs.
- [x] Restore mesh ray overlap with triangle-level tests before exposing it
      through `LSMeshCollider.ColliderOverlapsRay(...)`. Use the mesh BVH/query
      data instead of brute-forcing every triangle in normal query paths.
- [x] Add `Mesh_Cylinder` collision support and map both `Mesh/Cylinder` and
      `Cylinder/Mesh` dispatch entries, including narrow-phase tests, contact
      normal orientation, penetration-depth sign, and reversed-pair stability.
- [x] Remove mesh hot-path allocations while the mesh code is open. Current
      review targets include per-call `SwiftList<int>` allocation in
      `LSMeshCollider.GetNearbyTriangles(...)` and `ClosestPointOnSurface(...)`,
      per-triangle `Vector3d[]` allocation in `ClosestPointToTriangles(...)`,
      and `MeshObjectInfo` triangle/vertex ownership.
- [x] Verify mesh contact normals. `LSMeshCollider.GetNormalAtPoint(...)`
      currently derives a normal from the closest point vector, which is
      unlikely to be a correct triangle/contact normal for arbitrary meshes.
- [x] Revisit the `PhysicsMesh` edge cache once mesh ray, mesh/cylinder, and
      contact-normal callers are known. Remove it only if face normals and
      on-demand edge normals cover all supported behavior with tests.
- [x] Update stale docs after this phase. In particular, remove the old
      `docs/wiki/OVERVIEW.md` claim that cylinder behavior is unimplemented,
      update mesh ray and mesh/cylinder limitations, document true swept sphere
      queries in `docs/wiki/QUERY_SERVICES.md`, and document `GroundProbeMode`
      in host/runtime wiki pages.
- [x] Run the focused query, grounding, mesh, and collision shape-pair tests.
- [x] Re-run `query-service`, `simulation-allocation`, and `collision-detection`
      benchmark smoke tests after the implementation is stable.

**Phase 8 notes:**

- Swept-sphere queries are true 3D segment sweeps against sphere, capsule,
  cuboid, and finite-cylinder targets. Mesh targets remain deliberately
  unsupported by swept-sphere queries until a triangle sweep policy and
  acceleration strategy are designed.
- `GroundProbeMode` is body-level. `Ray` preserves sorted raycast behavior,
  `SweptSphere` uses the new query with self-exclusion, and `Auto` uses swept
  spheres for sphere/capsule/cylinder/wide cuboid bodies. Ground-hit filtering
  rejects ordinary movable dynamic bodies so swept probes do not treat nearby
  peers as terrain.
- Mesh ray overlap now uses the triangle BVH and no-alloc triangle access.
  Mesh/cylinder dispatch is mapped in both directions and normalized to
  mesh-to-cylinder contact data by pair priority.
- Mesh hot-path allocation cleanup covered caller-owned triangle buffers, pooled
  `MeshObjectInfo` triangle ownership, and no-alloc triangle vertex access. The
  mesh edge cache is retained for now because mesh SAT still uses existing
  context data and removal needs broader mesh manifold proof.
- Final review found and fixed a mesh query edge case: `LSMeshCollider` now
  seeds collider bounds from `PhysicsMesh.Bounds`, uses the dominant mesh bounds
  axis for triangle query windows, and computes closest mesh points against the
  original query point rather than the snapped bounds point.
- Benchmark smoke still reports allocation in the broad collision-detection
  benchmark (`CheckPreparedPrimitivePairs`, about 1.53 KB/op in the short local
  run). Review points to the older SAT/context object-info path rather than the
  new swept query path; Phase 9 handles that before diagnostics treat collision
  events as allocation baselines.

## Phase 9: Collision SAT Allocation Baseline

**Purpose:** Isolate and remove the remaining broad collision-detection
allocation before diagnostics make collision events more expensive to reason
about.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/Context/CollisionContext.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Support/Context/CollisionObjectInfo.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/Context/CuboidObjectInfo.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/Context/MeshObjectInfo.cs`
- Create:
  `src/Gravitas/CollisionHandling/Support/Context/CollisionSatScratch.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSCuboidCollider.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Modify:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Split the `collision-detection` benchmark so it reports primitive non-SAT
      pairs, cuboid/cuboid SAT, mesh/cylinder, mesh/cuboid, and mesh/mesh paths
      separately. Keep the current mixed `CheckPreparedPrimitivePairs` benchmark
      as an aggregate smoke case.
- [x] Confirm the allocation source with the split benchmark before changing
      implementation. The current hypothesis is that allocation comes from
      per-check SAT/context object-info construction and pooled scratch
      container churn, not from the Phase 8 swept query or mesh/cylinder path.
- [x] Replace per-check SAT object-info/context allocations with reusable,
      deterministic scratch state. Prefer context-owned or physics-service-owned
      scratch over static state so concurrent worlds and test contexts remain
      isolated.
- [x] Preserve pair-order contact data, contact normal orientation, and
      penetration-depth behavior for cuboid/cuboid, mesh/cuboid, mesh/mesh, and
      mesh/cylinder checks.
- [x] Add or update tests for reversed pair ordering, rotated cuboid SAT,
      mesh/cuboid triangle selection, mesh/mesh triangle selection, and
      mesh/cylinder contact stability after the scratch refactor.
- [x] Re-run `collision-detection` benchmark smoke and document which paths
      report `0 B/op` and which paths intentionally still allocate.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` if SAT scratch ownership,
      mesh-contact selection, or benchmark expectations change.

**Status:**

- Split benchmark baseline confirmed allocations were concentrated in SAT and
  mesh candidate paths:
  - `CheckPreparedPrimitivePairs`: about 1.53 KB/op.
  - `CheckCuboidCuboidSatPairs`: about 64.5 KB/op.
  - `CheckMeshCylinderPairs`: about 2 KB/op.
  - `CheckMeshCuboidPairs`: about 24 KB/op.
  - `CheckMeshMeshPairs`: about 28.5 KB/op.
- Implemented context-owned `CollisionSatScratch` on `GravitasWorldContext`. The
  scratch owns reusable SAT context/object-info state and mesh/cylinder triangle
  buffers for one world context, avoiding static scratch and keeping concurrent
  worlds isolated.
- `CollisionContext`, `CollisionObjectInfo`, `CuboidObjectInfo`, and
  `MeshObjectInfo` now reuse owned `SwiftCollections` buffers instead of
  constructing object-info wrappers and renting pooled collections per check.
- `LSCuboidCollider.ClosestPointOnSurface(...)` now walks cached face index data
  directly for rotated cuboids instead of allocating temporary face vertex
  arrays through `GetFace(i)`.
- Added allocation guardrails for axis-aligned cuboids, rotated cuboid SAT,
  mesh/cylinder, mesh/cuboid SAT, and mesh/mesh SAT after warmup.
- Short `collision-detection` benchmark smoke after the refactor reported no
  managed allocation in the summary `Allocated` column for all split paths:
  aggregate prepared primitives, non-SAT primitives, cuboid/cuboid SAT,
  mesh/cylinder, mesh/cuboid, and mesh/mesh. Timings in that one-iteration smoke
  are not canonical performance evidence.

## Phase 10: Engine-Agnostic Diagnostics

**Purpose:** Replace old debug draw intentions with deterministic diagnostic
events that hosts can visualize however they want.

**Files:**

- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticEvent.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticSink.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDebugDrawCommand.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticColor.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticEventKind.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDebugDrawKind.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/SolidBody.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionResponse.cs`
- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Modify: `src/Gravitas/Raycasting/GravitasCircleQueryService.cs`
- Create: `tests/Gravitas.Tests/Diagnostics/GravitasDiagnosticSinkTests.cs`
- Create: `tests/Gravitas.Benchmarks/Diagnostics/DiagnosticsBenchmarks.cs`
- Create: `docs/wiki/DIAGNOSTICS.md`

**Tasks:**

- [x] Add a context-owned diagnostic sink that is disabled by default and
      records deterministic event structs when enabled.
- [x] Evaluate `SwiftCollections.Observable` for host-facing diagnostic
      projection only; keep the core diagnostic sink deterministic,
      context-owned, and allocation-aware.
- [x] Emit force delta, velocity delta, ground probe, ray/query, contact normal,
      contact point, and response impulse events through the sink.
- [x] Ensure diagnostics do not allocate when disabled.
- [x] Confirm Phase 9's collision-detection allocation baseline before wiring
      contact/response diagnostics, so diagnostics do not hide older
      collision-pipeline allocation debt.
- [x] Add tests proving event ordering is deterministic and scoped to one
      `GravitasWorldContext`.
- [x] Add a benchmark that compares disabled diagnostics against the same path
      with diagnostics enabled.
- [x] Document how a Unity or server host can consume diagnostics without
      linking engine types into Gravitas.

**Status:** Implemented.

**Implementation notes:**

- Added `GravitasWorldContext.Diagnostics`, a context-owned
  `GravitasDiagnosticSink` that stays disabled by default and resets with the
  context.
- Kept `SwiftCollections.Observable` out of the runtime sink. It remains a
  possible host/tooling projection layer, but the core capture path uses
  deterministic append-only buffers.
- Added diagnostic events for body force/torque deltas, response velocity
  deltas, ground probes, raycasts, swept-sphere queries, circle overlap queries,
  contacts, and response impulses.
- Added renderer-neutral draw commands for collider capture plus line, ray, and
  point overlays. Mesh collider capture emits one wire-triangle command per
  triangle.
- Added focused diagnostics tests for disabled allocation, context scoping,
  deterministic sequence ordering, collision event order, and debug draw command
  emission.
- Added the `diagnostics` benchmark selection to compare disabled and enabled
  event hooks plus disabled and enabled collider debug draw capture.
- Added `docs/wiki/DIAGNOSTICS.md` and linked diagnostics through the overview,
  runtime, collision, query, README, benchmark README, and contributor guide.

## Verification Gate For Every Phase

- [ ] Run focused tests for the changed subsystem.
- [ ] Run `dotnet build Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release --no-build`.
- [ ] Run `dotnet build Gravitas.slnx --configuration ReleaseLean` when
      settings, serialization, package references, or MemoryPack-adjacent code
      changes.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build`
      when the Lean build is touched.
- [ ] Run the relevant benchmark aliases for any hot-path, data-structure,
      query, collision, partition, or solver change.
- [ ] Update `docs/wiki/` and this plan status before marking a phase complete.
