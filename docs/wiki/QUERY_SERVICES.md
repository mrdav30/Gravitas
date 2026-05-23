# Query Services

Raycasts and circlecasts are context-owned services. They use the same
GridForge-backed partitions as collision detection, resolve collider IDs through
the owning `GravitasPhysicsService`, and suppress duplicate hits when a collider
appears in multiple voxels.

## Raycasts

`GravitasRaycastService` exposes:

- `Raycast(origin, direction, maxDistance, out hit, layerMask)`
- `RaycastAll(start, end, layerMask)`

The raycast service owns:

- one `RaycastAxisWorker`.
- a reusable intersection-point buffer.
- a reusable hit buffer.
- a duplicate collider checker.
- a context-local query `Version`.

The candidate path is:

1. prepare the worker for the query line.
2. trace voxels with `GridTracer.TraceLine(context.World, start, end)`.
3. inspect each voxel's `PhysicsPartition`.
4. resolve dynamic and static collider IDs through the context physics service.
5. filter by layer mask.
6. skip colliders already checked in this query.
7. call the collider's `ColliderOverlapsRay(...)`.
8. build `LSRaycastHit` values from intersection points, normals, and distance.
9. sort `RaycastAll` results by distance.

Colliders also store `RaycastVersion`; this is a second duplicate guard scoped
to the service version.

Current limitation: the raycast implementation uses 2D path distance plus a
height slope to decide whether an intersection point is within collider height
bounds. Perfectly horizontal ray lines currently produce no hit because the
height slope is zero. Tests use a small deterministic height slope until this is
hardened.

## Circlecasts

`GravitasCirclecastService` exposes:

- `CircleCast(position, radius, out hit, layerMask)`
- `CircleCast(position, radius, direction, out hit, maxDistance, layerMask)`
- `CircleCastAll(position, radius, layerMask)`

The circlecast service owns:

- a reusable hit buffer.
- a duplicate collider checker.
- a context-local query `Version`.

The candidate path is:

1. scan an X/Z square around the query position in world voxel-size increments.
2. resolve voxels through `context.World.TryGetVoxel(...)`.
3. inspect each voxel's `PhysicsPartition`.
4. resolve collider IDs through the context physics service.
5. filter by layer mask.
6. skip duplicate colliders for this query version.
7. perform a fast radius check against the collider's scaled radius.
8. return the closest hit or all hits sorted by distance.

The directional overload is a post-filter on the closest non-directional hit. It
is not a full swept-volume query yet. Current circlecast hit distance is also
set to the query radius, so "closest" behavior should be treated as provisional
until the query returns a true distance.

## Layer Mask Semantics

The query parameter is currently named `ignoreLayers`, but the safe observed
usage is closer to a single-layer include check:

- `new SingleLayer(layerIndex)` includes colliders on that layer.
- multi-layer masks and "include all" semantics need API hardening before
  alpha.

For example, `new SingleLayer(0)` includes layer 0. This API should be renamed
or clarified before alpha so call sites do not encode the wrong expectation.

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
- make circlecast a precise shape query or rename it as a proximity query.
- clarify layer mask API naming.
- clarify `SingleLayer` versus bitmask semantics for queries and ground checks.
- remove avoidable steady-state allocations from enumerable result paths and
  sorting helpers.
- add shape-specific query tests for every collider type.
- decide whether query services remain single-threaded or move to explicit
  query state objects.
