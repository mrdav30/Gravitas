# Physics Hardening Backlog Action Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the former inline maintenance notes and current wiki prototype edges into a prioritized alpha-hardening backlog.

**Architecture:** Work from contracts and tests outward: layer/query semantics first, then collider shape state, narrow-phase detection, response, body/grounding behavior, broad-phase culling, and diagnostics. Each phase should leave source comments clean, docs/wiki current, and the affected hot paths covered by tests plus benchmarks where complexity or allocation risk changes.

**Tech Stack:** C# 11, `FixedMathSharp`, `SwiftCollections`, `GridForge`, xUnit v3, BenchmarkDotNet, Chronicler.

---

## Source Context Reviewed

- `src/Gravitas/Core/StiffBody.cs`
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
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Bounds/BoundingBox.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Bounds/BoundingFrustum.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs`
- `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/BoundingVolume/SwiftFixedBVH.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/BoundingVolume/Volume/FixedBoundVolume.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/Octree/SwiftFixedOctree.cs`
- `../SwiftCollections/src/SwiftCollections.FixedMathSharp/Query/SpatialHash/SwiftFixedSpatialHash.cs`
- `../SwiftCollections/src/SwiftCollections/Observable`
- `../SwiftCollections/src/SwiftCollections/Query`

## Cleanup Completed While Creating This Plan

- Removed the captured inline maintenance markers from `src/Gravitas`.
- Removed dead commented scaffolding around `LSCollider.IsInCollision`.
- Removed stale engine-specific debug draw comment blocks from body, detection, pair, and response code.
- Updated wiki docs that were stale after the allocation hardening refactor:
  - `docs/wiki/RUNTIME_ARCHITECTURE.md` no longer lists query hit buffers owned by query services.
  - `docs/wiki/COLLISION_PIPELINE.md` now describes direct spatial-cell/voxel scanning instead of `GridTracer.GetCoveredVoxels(...)`.
  - `docs/wiki/OVERVIEW.md` points to the completed allocation plan under `docs/feature-work/done`.

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
- `FixedBoundVolume` is min/max based. Current mesh BVH construction appears to
  pass center/size-style values in at least one place, so Phase 3 should verify
  and correct triangle bounds before relying on mesh query results.
- `SwiftCollections.Observable` may be useful for host-facing diagnostics,
  editor tooling, or presentation binding. Keep observable notifications out of
  authoritative simulation hot paths unless deterministic ordering, allocation
  behavior, and cost are covered by tests and benchmarks.

## Captured Inline Note Inventory

| Area | Captured concern | Priority | Destination |
| --- | --- | ---: | --- |
| `SingleLayer` | Type name and semantics blur layer index, bitmask, and host-layer metadata. | P0 | Phase 1 complete |
| `PhysicsSettings` | Collision matrix uses `bool[,]`; ground-check mask is legacy and not clearly configured. | P0 | Phase 1 complete |
| `docs/wiki/QUERY_SERVICES.md` | Query layer parameter behaves like include mask despite `ignoreLayers` naming. | P0 | Phase 1 complete |
| `docs/wiki/QUERY_SERVICES.md` | Horizontal raycasts are rejected by the height-slope path. | P0 | Phase 2 |
| `docs/wiki/QUERY_SERVICES.md` | Former circle-cast behavior was a proximity query, not a true swept shape query. | P0 | Phase 2 complete |
| `LSCollider` | Mesh rotation and bounds are not physically trustworthy enough. | P0 | Phase 3 |
| `LSCapsuleCollider` | Capsule derived points need deterministic invalidation/rebuild when size inputs change. | P0 | Phase 3 |
| `LSCapsuleCollider` | Default capsule dimensions can produce a zero cylinder-height inertia tensor diagonal. | P0 | Phase 3 |
| `PhysicsMesh` | Mesh input validation is missing. | P0 | Phase 3 |
| `PhysicsMesh` | Fixed query BVH bounds construction needs min/max verification. | P0 | Phase 3 |
| `LSMeshCollider` | Mesh collider limits, dynamic support, convexity, and ray overlap need explicit policy. | P0 | Phase 3 and Phase 4 |
| `docs/wiki/OVERVIEW.md` | Cylinder collider behavior is not implemented. | P1 | Phase 4 |
| `CollisionDetection` | Detection needs engine-agnostic tests and instrumentation, not Unity debug draw leftovers. | P1 | Phase 4 and Phase 8 |
| `CollisionResponse` | Response is prototype-level and needs physical solver hardening. | P1 | Phase 5 |
| `CollisionPair` | Time-spaced culling can miss behavior; culling should account for distance, velocity, and pair state. | P1 | Phase 7 |
| `LSCollider` | Teleports should invalidate culling assumptions. | P1 | Phase 7 |
| `PhysicsPartition` | Dynamic-object removal is linear. | P2 | Phase 7 |
| `StiffBody` | Initialization assumes grounded instead of deriving it from the world. | P1 | Phase 6 |
| `StiffBody` | Rotation visualization interpolation needs a clear speed/time contract. | P2 | Phase 6 |
| `StiffBody` | Force/ground debug visualization should become engine-agnostic diagnostics. | P2 | Phase 8 |
| `PhysicsMesh` | Edge cache may be removable if face normals and on-demand edge normals cover all callers. | P2 | Phase 3 |

## Recommendations

- Phase 1 resolved the `SingleLayer` hazard by splitting collider layer identity
  from query/ground-check bitmask membership. Keep new code on
  `PhysicsLayer`/`PhysicsLayerMask` rather than reintroducing ambiguous layer
  helpers.
- Do not begin a broad collision-response rewrite until narrow-phase shape-pair tests exist. Response bugs are difficult to diagnose if contact normals and depths are not already pinned.
- Split collider responsibilities deliberately. `LSCollider` currently owns identity, host binding, shape state, partition state, pair references, hierarchy filtering, query versions, and events. That is workable for a prototype but too dense for alpha hardening.
- Keep mesh collider dynamic support behind tests and benchmarks. Mesh work can become the whole project if it is not boxed into validation, limits, bounds, and query/collision behavior.
- Prefer downstream deterministic primitives before adding local equivalents. FixedMathSharp geometry and `SwiftCollections.FixedMathSharp` query structures should be the default starting point; if a custom Gravitas structure is better, prove it with tests, benchmarks, and a short design note.
- Use engine-agnostic diagnostics rather than adding editor hooks to runtime classes. Diagnostics should report deterministic values and let hosts decide how to draw them.
- Maintain `docs/wiki/` with each phase. The wiki is now useful enough that stale pages will mislead the next implementation pass.

## Phase 0: Baseline Test And Benchmark Harness

**Purpose:** Create enough harness coverage to safely refactor collider and collision internals.

**Files:**

- Create: `tests/Gravitas.Tests/Support/PhysicsScenarioBuilder.cs`
- Create: `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Create: `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- Create: `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Create: `tests/Gravitas.Benchmarks/CollisionHandling/CollisionResponseBenchmarks.cs`
- Modify: `tests/Gravitas.Benchmarks/Support/BenchmarkCatalog.cs` only if alias discovery needs new namespace handling.

**Tasks:**

- [x] Add a scenario builder that creates a `GravitasWorldContext`, grid coverage, `TestMatterAgent`, body, and collider combinations with fixed positions and fixed rotations.
- [x] Add detection tests for supported shape pairs using separated, edge-touching, overlapping, degenerate, and rotated cases.
- [x] Add response invariant tests for immovable-vs-dynamic, dynamic-vs-dynamic, trigger-vs-solid, restitution zero, and nonzero angular velocity cases.
- [x] Add short benchmarks for narrow-phase dispatch and response solver paths using 64 deterministic pairs.
- [x] Run `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionDetectionShapePairTests|FullyQualifiedName~CollisionResponseInvariantTests"`.
- [x] Run `dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collision-detection collision-response --filter "*" -j Short -i --exporters json`.

**Phase 0 completion notes:**

- Added `PhysicsScenarioBuilder`, primitive narrow-phase tests, response
  invariant tests, and `collision-detection`/`collision-response` benchmark
  aliases.
- Fixed three small issues exposed by the new baseline:
  - `StiffBody.Setup(...)` no longer evaluates stateful collider `Shape` before
    the collider has a transform/body.
  - capsule/capsule detection now handles degenerate capsule line segments
    deterministically.
  - trigger pairs now skip physical response in both pair dispatch and direct
    `CollisionResponse.CalculateImpulse(...)` calls.
- Short benchmark smoke on this machine:
  - `CollisionDetectionBenchmarks.CheckPreparedPrimitivePairs`: 64 pairs,
    mean about `53.44 us`, allocated about `3 KB`.
  - `CollisionResponseBenchmarks.CalculateImpulseForPreparedPairs`: 64 pairs,
    mean about `475.4 us`, allocated about `1.27 KB`.
- BenchmarkDotNet could not raise process priority in this sandbox and warned
  that the response benchmark iteration time is short. Treat these as smoke
  baselines, not canonical performance numbers.

## Phase 1: Layer And Settings Contract

**Purpose:** Remove ambiguity between layer index, layer mask, collision matrix, and ground-check filtering before those concepts spread into more tests.

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

- [x] Introduce explicit types for layer index and layer mask. Recommended names: `PhysicsLayer` for one index and `PhysicsLayerMask` for bitmask queries.
- [x] Replace query parameters named `ignoreLayers` with names that match observed behavior: `includedLayers` or `layerMask`.
- [x] Replace the legacy hard-coded `IgnoreForGroundCheck` default with a settings-owned mask that defaults to a documented value.
- [x] Decide whether the collision matrix remains `bool[,]` or moves to a SwiftCollections-backed bitset. If it changes, add benchmarks for layer lookups during pair filtering.
- [x] Add tests for single layer inclusion, multi-layer inclusion, include-all, include-none, collision matrix allow/deny, and ground-check layer filtering.
- [x] Update wiki examples to use the new names and semantics.
- [x] Run the focused Phase 1 layer/query/settings test slice before the Phase 2 circle-query rename.

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

**Purpose:** Make raycasts and circlecasts physically named and deterministic before collision and grounding work depends on them.

**Files:**

- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Rename/modify: `src/Gravitas/Raycasting/GravitasCircleQueryService.cs`
- Rename/modify: `src/Gravitas/Raycasting/RaycastSegmentWorker.cs`
- Modify: `tests/Gravitas.Tests/Raycasting/GravitasRaycastServiceTests.cs`
- Rename/modify: `tests/Gravitas.Tests/Raycasting/GravitasCircleQueryServiceTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Raycasting/QueryServiceBenchmarks.cs`
- Modify: `docs/wiki/QUERY_SERVICES.md`

**Tasks:**

- [x] Replace the raycast height-slope rejection with deterministic 3D segment handling so horizontal, vertical, and diagonal rays all have defined behavior.
- [x] Evaluate `FixedRay` and FixedMathSharp bounds/plane intersection APIs before keeping or expanding custom ray intersection workers.
- [x] Add raycast tests for horizontal rays, vertical rays, diagonal rays, starting-inside-collider rays, no-hit rays, multi-hit ordering, and cross-context isolation.
- [x] Rename or split the current circlecast behavior. Recommended contract: keep the current proximity scan as an overlap query, then implement true swept sphere or swept circle only after the dimensional model is explicit.
- [x] Add tests proving circle/proximity hit distance, normal, point, and ordering.
- [x] Keep caller-owned hit buffers and the allocation-free sorter in all all-hit paths.
- [x] Re-run the `query-service` allocation benchmark and confirm the all-hit paths remain at 0 B/op in the short smoke run.

**Phase 2 completion notes:**

- Replaced the raycast height-slope path with a 3D `RaycastSegmentWorker`.
  `Raycast` normalizes direction before applying max distance, and `RaycastAll`
  now accepts horizontal, vertical, diagonal, and starting-inside segments.
- Reviewed FixedMathSharp `FixedRay` and bounds intersections. They remain a
  good future primitive for first-hit infinite-ray checks, but Gravitas keeps a
  custom segment worker here because this service needs bounded segment scans,
  all intersection points, caller-owned buffers, and deterministic hit sorting.
- Renamed the context service from the old circle-cast property to `CircleQueries` and the
  service type to `GravitasCircleQueryService`. Current circle behavior is now
  exposed as `OverlapCircle`, `OverlapCircleAll`, and
  `OverlapCircleInDirection`; true swept circle/sphere casts remain deferred
  until the dimensional contract is explicit.
- Circle overlap hit data now uses the closest collider surface point, collider
  surface normal, and surface distance for ordering.
- Query docs, host examples, README/AGENTS references, and benchmark docs were
  updated with the new terminology.
- Short `query-service` benchmark smoke reported no managed allocation in the
  `Allocated` column for `RaycastAll`, `OverlapCircleAll`,
  `OverlapCircleInDirection`, and overlapping-context raycasts. BenchmarkDotNet
  still warned that this sandbox cannot raise process priority, so these are
  smoke numbers rather than canonical timing evidence.
- Also reran the `simulation-allocation` smoke because grounding now uses
  `OverlapCircleInDirection`; its scenarios also reported no managed allocation
  in the `Allocated` column.

## Phase 3: Collider Shape And Runtime State Refactor

**Purpose:** Reduce `LSCollider` responsibility density and make shape-derived state explicit, invalidatable, and testable.

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

- [x] Split shape-derived data from collider identity and partition/pair state. Recommended first step: extract a focused internal shape-state helper before renaming public collider types.
- [x] Add tests that changing offset, scale, radius, height, or rotation invalidates and rebuilds bounds, radius, area, and capsule segment data exactly once per simulate step.
- [x] Fix capsule height/radius rebuild behavior and add tests for short, tall, scaled, and rotated capsules.
- [x] Fix capsule inertia for degenerate/default dimensions or define a sphere-fallback policy for zero cylinder height.
- [x] Add `PhysicsMesh` validation for null arrays, triangle-count multiples of three, out-of-range triangle indices, duplicate triangle indices, and degenerate triangles.
- [x] Fix or confirm mesh `FixedBoundVolume` construction so triangle BVH entries and query bounds use min/max coordinates, not center/size values.
- [x] Reuse FixedMathSharp typed bounds helpers where behavior matches tests, especially for closest point, containment, and intersection checks.
- [x] Define mesh collider limits in settings or collider construction. Recommended first policy: fail fast when vertex or triangle counts exceed explicit deterministic limits.
- [x] Prove whether `_edges` and cached edge normals are still required. Remove them only after mesh/cuboid and mesh/mesh SAT tests prove equivalent detection behavior.
- [x] Fix mesh bounds under rotation or make mesh collider rotation limitations explicit in API and tests.
- [x] Run collider tests and the new collider shape benchmark in Release.

**Phase 3 completion notes:**

- Added `ColliderRuntimeShapeState`/`ColliderShapeSnapshot` so collider identity,
  pair ownership, and partition state no longer hide the rebuild boundary for
  bounds and shape-derived caches.
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
  current mesh SAT coverage does not yet prove it removable; Phase 4 shape-pair
  tests should decide whether it can be deleted.
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

**Purpose:** Complete and harden shape-pair detection before replacing response behavior.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSCylinderCollider.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/Support/ColliderType.cs`
- Modify: `src/Gravitas/Colliders/Support/CollisionType.cs`
- Modify: `src/Gravitas/Colliders/ColliderSettings.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Build a shape-pair matrix that lists every supported, unsupported, and intentionally deferred pair.
- [ ] Review `FixedPlane`, `FixedPlaneIntersectionType`, and FixedMathSharp typed bounds APIs before adding or rewriting SAT, plane classification, frustum, or mesh support helpers.
- [ ] Implement cylinder collision behavior or remove cylinder from active dispatch until its tests define support.
- [ ] Restore mesh ray overlap only after `PhysicsMesh` validation, bounds, and triangle acceleration tests exist.
- [ ] Decide non-convex mesh policy. Recommended alpha policy: preprocess into convex sub-meshes offline or at initialization, never during per-frame collision.
- [ ] Add tests for contact normal orientation, penetration depth sign, and point ordering for every supported pair.
- [ ] Add tests for pair dispatch stability so collider priority changes cannot silently flip contact data.
- [ ] Run collision detection benchmarks before and after each shape-pair algorithm change.

## Phase 5: Collision Response Solver Redesign

**Purpose:** Replace prototype response behavior with a physically explainable deterministic solver.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/CollisionResponse.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling/CollisionResponseBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Define units and invariants for mass, inverse mass, inertia tensor, angular velocity, restitution, friction, drag, damping, and penetration correction.
- [ ] Replace single-point response assumptions with an explicit contact model. Recommended first milestone: stable single-contact solver with room for contact manifolds.
- [ ] Add tests for conservation expectations, restitution thresholds, immovable body behavior, kinematic body behavior, angular impulse direction, trigger exclusion, and stable resting contact.
- [ ] Revisit `ContactPoint` depth clamping. Keep it only if tests prove it is a solver invariant rather than a hidden correction.
- [ ] Add deterministic replay tests that run the same body pair for many fixed frames and compare final state.
- [ ] Run response benchmarks and add allocation checks for active-pair processing after solver changes.

## Phase 6: Body, Grounding, And Visualization Semantics

**Purpose:** Make body state transitions deterministic and physically named instead of relying on prototype assumptions.

**Files:**

- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Runtime/GravitasClock.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Create: `tests/Gravitas.Tests/Core/StiffBodyGroundingTests.cs`
- Create: `tests/Gravitas.Tests/Core/StiffBodyIntegrationTests.cs`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`

**Tasks:**

- [ ] Replace grounded-on-initialize with an explicit initial grounding probe or a documented default of not grounded until the first simulation step.
- [ ] Rework ground checks to use the Phase 1 layer mask contract and the Phase 2 query contract.
- [ ] Add tests for grounded initialization, airborne initialization, moving platforms, skipped ground checks, slope normals, and layer-filtered ground.
- [ ] Define visual rotation interpolation as either frame accumulation or speed-limited interpolation. Add tests for both reset accumulation and steady visualize frames.
- [ ] Add integration tests for force, velocity, drag, friction, torque, angular damping, and rest-state transitions using fixed expected values.

## Phase 7: Broad-Phase Culling And Partition Performance

**Purpose:** Keep collision candidate management low-complexity without hiding missed contacts behind culling.

**Files:**

- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Modify: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Create: `tests/Gravitas.Tests/Partitions/PhysicsPartitionPerformanceShapeTests.cs`
- Create: `tests/Gravitas.Tests/CollisionHandling/CollisionPairCullingTests.cs`
- Create: `tests/Gravitas.Benchmarks/Core/PartitionCullingBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Add tests proving teleporting a collider invalidates pair culling and repartitions before the next collision distribution pass.
- [ ] Replace dynamic-list removal only if benchmarks show churn is meaningful. Recommended implementation if needed: swap-remove plus an ID-to-index map owned by `PhysicsPartition`.
- [ ] Benchmark current GridForge partitioning against `SwiftFixedSpatialHash<T>`, `SwiftFixedOctree<T>`, or `SwiftFixedBVH<T>` before replacing broad-phase structures.
- [ ] Add tests for partition activation/deactivation transitions after dynamic object add/remove churn.
- [ ] Redesign cull countdown using distance, relative velocity, frame count since collision, and partition movement state.
- [ ] Add tests for fast-moving objects, large objects, recently collided pairs, and culling-disabled colliders.
- [ ] Re-run `simulation-allocation`, `collision-partition`, and the new culling benchmark after every data-structure change.

## Phase 8: Engine-Agnostic Diagnostics

**Purpose:** Replace old debug draw intentions with deterministic diagnostic events that hosts can visualize however they want.

**Files:**

- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticEvent.cs`
- Create: `src/Gravitas/Diagnostics/GravitasDiagnosticSink.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionResponse.cs`
- Create: `tests/Gravitas.Tests/Diagnostics/GravitasDiagnosticSinkTests.cs`
- Create: `docs/wiki/DIAGNOSTICS.md`

**Tasks:**

- [ ] Add a context-owned diagnostic sink that is disabled by default and records deterministic event structs when enabled.
- [ ] Evaluate `SwiftCollections.Observable` for host-facing diagnostic projection only; keep the core diagnostic sink deterministic, context-owned, and allocation-aware.
- [ ] Emit force delta, velocity delta, ground probe, ray/query, contact normal, contact point, and response impulse events through the sink.
- [ ] Ensure diagnostics do not allocate when disabled.
- [ ] Add tests proving event ordering is deterministic and scoped to one `GravitasWorldContext`.
- [ ] Add a benchmark that compares disabled diagnostics against the same path with diagnostics enabled.
- [ ] Document how a Unity or server host can consume diagnostics without linking engine types into Gravitas.

## Verification Gate For Every Phase

- [ ] Run focused tests for the changed subsystem.
- [ ] Run `dotnet build Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release --no-build`.
- [ ] Run `dotnet build Gravitas.slnx --configuration ReleaseLean` when settings, serialization, package references, or MemoryPack-adjacent code changes.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build` when the Lean build is touched.
- [ ] Run the relevant benchmark aliases for any hot-path, data-structure, query, collision, partition, or solver change.
- [ ] Update `docs/wiki/` and this plan status before marking a phase complete.
