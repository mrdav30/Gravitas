# Query Services

Raycasts and circle-overlap queries are context-owned services. They use the same
GridForge-backed partitions as collision detection, resolve collider IDs through
the owning `GravitasPhysicsService`, and suppress duplicate hits when a collider
appears in multiple voxels.

## Raycasts

`GravitasRaycastService` exposes:

- `Raycast(origin, direction, maxDistance, out hit, layerMask)`
- `RaycastAll(start, end, layerMask, results)`

The raycast service owns:

- one `RaycastSegmentWorker`.
- a reusable intersection-point buffer.
- a duplicate collider checker.
- a duplicate voxel checker.
- a context-local query `Version`.

`RaycastAll` clears the caller-provided `SwiftList<LSRaycastHit>`, writes hits
into it, returns the hit count, and sorts the results by distance using an
allocation-free in-place sorter. Keep these result buffers owned by the caller or
context that issues the query.

The candidate path is:

1. prepare the worker for the 3D query segment.
2. snap the query bounds to the context `GridWorld` voxel size.
3. scan the covered spatial grid cells and active `VoxelGrid` instances.
4. suppress duplicate voxels and inspect each voxel's `PhysicsPartition`.
5. resolve dynamic and static collider IDs through the context physics service.
6. filter by layer mask.
7. skip colliders already checked in this query.
8. call the collider's `ColliderOverlapsRay(...)`.
9. build `LSRaycastHit` values from intersection points, normals, and distance.
10. sort `RaycastAll` results by distance.

Colliders also store `RaycastVersion`; this is a second duplicate guard scoped
to the service version.

Raycasts use deterministic 3D segment intersection points. Horizontal,
vertical, diagonal, and starting-inside segments have defined behavior and are
covered by focused tests. `FixedRay` was reviewed during this pass, but the
service keeps a custom segment worker because the query path needs all segment
intersection points, caller-owned buffers, and bounded segment distance rather
than the first forward hit on an infinite ray.

## Circle Overlap Queries

`GravitasCircleQueryService` exposes:

- `OverlapCircle(position, radius, out hit, layerMask)`
- `OverlapCircleInDirection(position, radius, direction, out hit, maxDistance, layerMask)`
- `OverlapCircleAll(position, radius, layerMask, results)`

The circle query service owns:

- a duplicate collider checker.
- a context-local query `Version`.

`OverlapCircleAll` clears the caller-provided `SwiftList<LSRaycastHit>`, writes
hits into it, returns the hit count, and uses the same allocation-free in-place
sorter as raycasts.

The candidate path is:

1. scan an X/Z square around the query position in world voxel-size increments.
2. resolve voxels through `context.World.TryGetVoxel(...)`.
3. inspect each voxel's `PhysicsPartition`.
4. resolve collider IDs through the context physics service.
5. filter by layer mask.
6. skip duplicate colliders for this query version.
7. perform a fast radius check against the collider's scaled radius.
8. build a proximity hit from the closest collider surface point.
9. return the closest hit or all hits sorted by surface distance.

`OverlapCircleInDirection` filters proximity hits by the direction from the
query origin to the hit point and by maximum hit-point distance. It is not a
swept circle or swept sphere query.

## Layer Mask Semantics

Queries accept `PhysicsLayerMask layerMask`. This is an include mask:

- `PhysicsLayerMask.FromLayer(layer)` includes one layer.
- `PhysicsLayerMask.FromLayers(...)` includes several layers.
- `PhysicsLayerMask.All` includes every layer.
- `PhysicsLayerMask.None` includes no layers.

Use `PhysicsLayer` for a collider's single collision/filter layer and
`PhysicsLayerMask` for query or ground-check filters. Keep those concepts
separate in new APIs so layer identity does not get confused with bitmask
membership.

## Reentrancy

Query services keep mutable buffers on the service instance. Do not run multiple
queries concurrently against the same context service. The current design
matches a single-threaded deterministic lockstep loop.

If future hosts need concurrent query work, the likely redesign is explicit
query job/state objects owned by the caller or rented from a context-local pool.

## Hit Data

`LSRaycastHit` is a readonly struct containing:

- `Collider`
- `Body`
- `Point`
- `Normal`
- `Distance`
- `Direction`

`Body` is `collider?.Body`, so static/bodyless hits can have a collider with a
null body.

## Query Hardening Targets

- replace horizontal-ray rejection with deterministic 3D segment handling.
- implement true swept circle or swept sphere queries once the 2D/3D dimensional
  contract is explicit.
- keep query benchmarks allocation-free as result ordering, filters, and shape
  support expand.
- add shape-specific query tests for every collider type.
- decide whether query services remain single-threaded or move to explicit
  query state objects.
