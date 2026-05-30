# Query Services

Queries are split into explicit context-owned 2D and 3D services:
`GravitasWorldContext.Query2D` and `GravitasWorldContext.Query3D`. The 3D
service owns raycasts, swept-sphere queries, and X/Z circle overlap/proximity
queries. It uses the same GridForge-backed partitions as 3D collision
detection, resolves collider IDs through the owning `GravitasPhysicsService`,
and suppresses duplicate hits when a collider appears in multiple voxels.

Pure 2D queries live on `GravitasQuery2DService` and operate over `Vector2d`
shape data. They should not route through the 3D raycast path or the X/Z circle
query path by accident.

## 3D Raycasts

`GravitasQuery3DService` exposes:

- `Raycast(origin, direction, maxDistance, out hit, layerMask)`
- `RaycastAll(start, end, layerMask, results)`

The 3D query service owns:

- one `RaycastSegmentWorker`.
- a reusable intersection-point buffer.
- one `SweptSphereQueryWorker`.
- a duplicate collider checker.
- a duplicate voxel checker.
- a context-local `RaycastVersion`.

`RaycastAll` clears the caller-provided `SwiftList<Physics3DHit>`, writes hits
into it, returns the hit count, and sorts the results by distance using an
allocation-free in-place sorter. Keep these result buffers owned by the caller or
context that issues the query.

When diagnostics are enabled, raycast calls emit a `RayQuery` event. Swept
sphere calls use the same event kind with `ScalarA` set to the sweep radius and
`DataB` set to the captured hit count.

The candidate path is:

1. prepare the worker for the 3D query segment.
2. snap the query bounds to the context `GridWorld` voxel size.
3. scan the covered spatial grid cells and active `VoxelGrid` instances.
4. suppress duplicate voxels and inspect each voxel's `PhysicsPartition`.
5. resolve dynamic and static collider IDs through the context physics service.
6. filter by layer mask.
7. skip colliders already checked in this query.
8. call the collider's `ColliderOverlapsRay(...)`.
9. build `Physics3DHit` values from intersection points, normals, and distance.
10. sort `RaycastAll` results by distance.

Colliders also store `RaycastVersion`; this is a second duplicate guard scoped
to the service's raycast/sweep version.

3D raycasts use deterministic segment intersection points. Horizontal,
vertical, diagonal, and starting-inside segments have defined behavior and are
covered by focused tests. Mesh ray overlap is triangle-level and queries the
mesh triangle BVH before testing candidate triangles. `FixedRay` was reviewed
during this pass, but the service keeps a custom segment worker because the
query path needs all segment intersection points, caller-owned buffers, and
bounded segment distance rather than the first forward hit on an infinite ray.
Duplicate suppression is covered for colliders whose broad-phase bounds span
many voxels; the all-hit path should still report one hit per collider.

## Swept Sphere Queries

`GravitasQuery3DService` also exposes true 3D swept-sphere queries:

- `SweepSphere(origin, radius, direction, maxDistance, out hit, layerMask, excludedCollider)`
- `SweepSphereAll(start, end, radius, layerMask, results, excludedCollider)`

The contract is segment-based and deterministic:

- `radius <= 0`, zero direction, or zero segment length returns no hits.
- starting overlap returns a zero-distance hit.
- `excludedCollider` is skipped before duplicate stamping, which lets body-owned
  ground probes ignore their own collider.
- all-hit results are written into the caller-owned buffer, then sorted by
  impact distance and collider ID as an explicit tie-breaker.
- closest-hit queries use the same distance/collider-ID tie-break rule.
- hit distance is the swept center's time-of-impact distance along the segment.
- hit point is the target surface point closest to the swept center at impact.
- hit normal points away from the hit surface toward the swept sphere center
  when that can be resolved, with shape normals as the fallback.

Current swept-sphere support covers sphere, capsule, cuboid, and finite cylinder
targets. Mesh targets are intentionally excluded for now; mesh sweep needs a
triangle sweep policy and acceleration strategy beyond static ray/overlap
queries.

`StiffBody` continuous collision detection reuses this service as an opt-in
movement sweep. Body CCD passes the moving collider as `excludedCollider`, uses
the all-hit path so later valid static targets can be found after ignored
dynamic targets, and consumes the same deterministic distance/collider-ID
ordering as host-facing swept-sphere queries.

## Circle Overlap Queries

`GravitasQuery3DService` exposes:

- `OverlapCircle(position, radius, out hit, layerMask)`
- `OverlapCircleInDirection(position, radius, direction, out hit, maxDistance, layerMask)`
- `OverlapCircleAll(position, radius, layerMask, results)`

The 3D query service's circle path owns:

- a duplicate collider checker.
- a context-local `CircleVersion`.

`OverlapCircleAll` clears the caller-provided `SwiftList<Physics3DHit>`, writes
hits into it, returns the hit count, and uses the same allocation-free in-place
sorter as raycasts.

When diagnostics are enabled, circle overlap calls emit a `CircleQuery` event
with the query center, radius, optional direction extent, layer mask bits, and
hit count.

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

Duplicate suppression is also covered for large colliders that appear in many
voxel partitions. Circle queries remain X/Z overlap/proximity queries for the
current 3D grounding model; they are not the pure 2D query API. Use
swept-sphere queries for deterministic 3D swept movement.

## Pure 2D Queries

`GravitasQuery2DService` exposes:

- `OverlapCircleAll(center, radius, results)`
- `OverlapCircleAll(center, radius, layerMask, results)`
- `Raycast(start, end, out hit)`
- `Raycast(start, end, layerMask, out hit)`
- `RaycastAll(start, end, results)`
- `RaycastAll(start, end, layerMask, results)`

All-hit methods clear the caller-provided `SwiftList<Physics2DHit>`, write hits
into it, return the hit count, and sort by distance with collider ID as the
deterministic tie-breaker. Closest-hit `Raycast` overloads use the same
ordering and return the first hit through an `out Physics2DHit`.

Pure 2D query positions are `Vector2d` values in the X/Z plane. When hosts
convert from a `FixedTransform`, use `Vector3d.ToVector2d()` so world X maps to
2D X and world Z maps to 2D Y.

The overlap-circle candidate path is:

1. project the query circle's X/Z bounds into private GridForge storage on the
   pure 2D Y=0 partition plane.
2. scan covered GridForge spatial cells and voxels.
3. inspect `PhysicsPartition2D` payloads and copy static/dynamic collider IDs.
4. suppress duplicate collider hits with each collider's 2D query-version
   stamp when a broad collider spans several voxels.
5. reject inactive colliders, layer-mask misses, and separated 2D bounds.
6. ask each 2D shape for its closest point to the query center.
7. include the collider when that closest point lies within the query radius or
   the shape contains the query center.
8. sort hits by distance and collider ID.

The segment raycast path projects the segment's 2D bounds into the same
GridForge-backed candidate gatherer, then runs deterministic shape math against
circle, AABB, and convex polygon colliders. Zero-length segments return no
hits. Starting inside a collider returns a zero-distance hit. A collider that
spans multiple voxels is still reported once because candidate gathering
stamps duplicate collider visits before exact shape testing.

Current hit data is `Physics2DHit`: collider, optional body, point, normal, and
distance. AABB and polygon area-query APIs remain future 2D query hardening
work.

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

`Physics3DHit` is a readonly struct containing:

- `Collider`
- `Body`
- `Point`
- `Normal`
- `Distance`
- `Direction`

`Body` is `collider?.Body`, so static/bodyless hits can have a collider with a
null body.

## Query Hardening Targets

- decide whether swept mesh support belongs in the query service or in a
  dedicated continuous-collision path.
- keep query benchmarks allocation-free as result ordering, filters, and shape
  support expand.
- add shape-specific query tests for every collider type.
- add pure 2D AABB and polygon area-query APIs once the shape math and
  benchmark contract justify the public surface.
- revisit explicit query state objects only when a real host requires
  concurrent queries against one context.
