# Query Services

Queries are split into explicit context-owned 2D, 3D, and mixed services:
`GravitasWorldContext.Query2D`, `GravitasWorldContext.Query3D`, and
`GravitasWorldContext.QueryMixed`. The 3D service owns raycasts,
swept-sphere queries, convex-source sweeps, and X/Z circle
overlap/proximity queries. It uses the
same GridForge-backed partitions as 3D collision detection, resolves collider
IDs through the owning `GravitasPhysicsService`, and suppresses duplicate hits
when a collider appears in multiple voxels.

Pure 2D queries live on `GravitasQuery2DService` and operate over `Vector2d`
shape data. They should not route through the 3D raycast path or the X/Z circle
query path by accident.

Mixed 2D/3D queries live on `GravitasQueryMixedService` and are always
explicit. Pure `Query2D` and pure `Query3D` do not report cross-dimensional
hits.

## Query Surface Inventory And Fallback Policy

Reducer labels used below:

- `Exact`: the accepted hit is produced by shape math that matches the
  documented source and target geometry.
- `ConservativeFallback`: the accepted hit is produced by a fallback that is
  allowed to report earlier or extra hits, but must not create false negatives.
- `NotSupported`: no runtime public API exists for the source/target family.

Mixed query hits expose this through `PhysicsMixedHit.ReducerKind`. Pure 2D and
3D query paths currently expose only exact public hits, except where the table
names an internal CCD proxy.

### Public Query Surface

| Surface | Source shape | Target shapes | Reducer policy | Ordering key | Allocation behavior |
| --- | --- | --- | --- | --- | --- |
| `Query3D.Raycast`, `RaycastAll` | bounded 3D segment | sphere, capsule, cuboid, finite cylinder, mesh, compound | `Exact`; mesh targets query triangle BVH candidates, compound targets keep owner identity | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query3D.SweepSphere`, `SweepSphereAll` | 3D sphere | sphere, capsule, cuboid, finite cylinder, mesh, compound | `Exact` swept-sphere reducers in `SweptSphereQueryWorker`; mesh uses triangle face/edge/vertex TOI, compound reduces stable part order | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query3D.SweepCapsule`, `SweepCapsuleAll`, `SweepCuboid`, `SweepCuboidAll`, `SweepCylinder`, `SweepCylinderAll` | registered capsule, cuboid, or finite-cylinder collider at its current pose plus a displacement | sphere, capsule, cuboid, finite cylinder, convex mesh, concave mesh target triangles, compound | `Exact` support-mapped conservative advancement; source collider is skipped; concave mesh targets reduce triangle candidates to owner collider hits | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query3D.SweepConvexMesh`, `SweepConvexMeshAll` | convex `LSMeshCollider` at its current pose plus a displacement | sphere, capsule, cuboid, finite cylinder, convex mesh, concave mesh target triangles, compound | `Exact` support-mapped conservative advancement; concave source meshes throw; concave mesh targets reduce triangle candidates to owner collider hits | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query3D.SweepCompound`, `SweepCompoundAll` | authored `LSCompoundCollider` made from supported convex 3D parts | sphere, capsule, cuboid, finite cylinder, convex mesh, concave mesh target triangles, compound | `Exact` per-part convex source reduction with stable authored part order; unsupported or concave mesh source parts throw | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query3D.OverlapCircle`, `OverlapCircleInDirection`, `OverlapCircleAll` | X/Z circle proximity query | 3D colliders through closest-surface projection | `Exact` for the current X/Z proximity contract; this is not swept movement | distance, collider ID for all-hit | service-owned scratch, caller-owned all-hit buffer |
| `Query2D.OverlapCircle`, `OverlapCircleAll` | 2D circle | circle, AABB, convex polygon, compound | `Exact`; compound reports owner once through stable part reduction | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query2D.OverlapAabb`, `OverlapAabbAll`, `OverlapPolygon`, `OverlapPolygonAll` | 2D AABB or convex polygon area | circle, AABB, convex polygon, compound | `Exact` SAT/closest-point area overlap; compound reports owner once through stable part reduction | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query2D.Raycast`, `RaycastAll` | 2D segment | circle, AABB, convex polygon, compound | `Exact`; zero-length segments return no hit, starting-inside returns distance zero | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `Query2D.SweepCircle`, `SweepCircleAll` | 2D circle | circle, AABB, convex polygon, compound | `Exact` circle-source sweep reducers; compound reports owner once through earliest part | distance, collider ID | service-owned scratch, caller-owned all-hit buffer |
| `QueryMixed.SweepSphereAgainst2D`, `SweepSphereAgainst2DAll` | 3D sphere | 2D circle slab, AABB slab, convex polygon slab, compound slab | circle slab: `Exact`; AABB/polygon prism bounds: `ConservativeFallback`; compound preserves the winning part label | distance, 3D collider ID, 2D collider ID | service-owned scratch, caller-owned all-hit buffer |
| `QueryMixed.SweepCircleAgainst3D`, `SweepCircleAgainst3DAll` | 2D circle embedded in a finite Y slab | 3D sphere, capsule, cuboid, finite cylinder, mesh, compound | sphere/cuboid/world-Y capsule/world-Y cylinder: `Exact`; unsupported rotated capsule/cylinder, mesh, and compound: `ConservativeFallback` | distance, 3D collider ID, 2D collider ID | service-owned scratch, caller-owned all-hit buffer |
| Concave/raw mesh-source sweeps | concave `LSMeshCollider` or raw mesh as the moving query source | 2D, 3D, or mixed targets | `NotSupported`; use offline convex decomposition into supported `LSCompoundCollider` parts | none | no raw mesh-source query API |

### Internal CCD Query Surface

| Owner | Query path | Source proxy | Target set | Reducer policy |
| --- | --- | --- | --- | --- |
| `StiffBody` static/kinematic 3D CCD | `Query3D.SweepSphereAgainstStaticAll` | source collider proxy sphere, then shape-exact validation where supported | bodyless, immovable, kinematic 3D colliders | public swept-sphere hit is exact for target geometry; source-shape refinement runs for target spheres |
| `StiffBody` dynamic 3D CCD | dynamic candidate index plus relative sweep | source and target dynamic proxy spheres | movable dynamic 3D bodies | conservative proxy; exact dynamic shape reducers are owned by the CCD hardening plan |
| `StiffBody` mixed static 2D CCD | `QueryMixed.SweepSphereAgainstStatic2DAll` | 3D sphere source | bodyless, immovable, kinematic 2D slabs | same `ReducerKind` policy as public `SweepSphereAgainst2D` |
| `StiffBody` mixed dynamic 2D CCD | mixed dynamic candidate index plus relative sweep | 3D proxy sphere against 2D mixed proxy sphere | movable dynamic 2D bodies | `ConservativeFallback` |
| `StiffBody2D` static/kinematic 2D CCD | `Query2D.SweepCircleAgainstStaticAll` plus mover-shape refinement | source circle sweep, refined by mover shape when needed | bodyless, immovable, kinematic 2D colliders | exact for current pure 2D sweep contract |
| `StiffBody2D` dynamic 2D CCD | dynamic candidate index plus relative sweep | dynamic proxy circles | movable dynamic 2D bodies | conservative proxy until dynamic-vs-dynamic 2D CCD ordering/reducer work expands |
| `StiffBody2D` mixed static 3D CCD | `QueryMixed.SweepCircleAgainstStatic3DAll` | embedded 2D circle slab | bodyless, immovable, kinematic 3D colliders | same `ReducerKind` policy as public `SweepCircleAgainst3D` |
| `StiffBody2D` mixed dynamic 3D CCD | mixed dynamic candidate index plus relative sweep | embedded 2D proxy sphere against 3D proxy sphere | movable dynamic 3D bodies | `ConservativeFallback` |

### Mesh-Source Query Boundary

In this document, mesh-as-source means a public query where mesh geometry is the
moving sweep source. Convex mesh sources are supported through
`Query3D.SweepConvexMesh` and `SweepConvexMeshAll`. Capsule, cuboid, and
finite-cylinder collider sources have explicit source-sweep APIs, and authored
compounds made from supported convex parts are supported through
`Query3D.SweepCompound` and `SweepCompoundAll`.

Concave mesh sources are intentionally rejected. The unsupported case is exact
concave mesh-as-source sweeping or automatic runtime decomposition, because
that can hide `source triangles x target candidates` work, runtime convex
decomposition, or ambiguous concave-source ordering behind what looks like a
simple query call. Hosts should author concave movers as offline-decomposed
`LSCompoundCollider` assets with stable convex part order.

`SweptSphereQueryWorker.TrySweep(LSCollider collider, ...)` is not a
mesh-source query. That worker is prepared with a swept sphere source through
`Prepare(start, end, radius)`, then tests one target collider against that
prepared swept sphere. Passing an `LSMeshCollider` there means "sweep this
sphere against a mesh target", not "sweep this mesh as the source".

Convex and concave mesh colliders remain valid simulation colliders and valid
query targets. 3D raycasts and swept-sphere queries test mesh triangle
candidates and return the owning `LSMeshCollider` once. Convex mesh source
sweeps also support concave mesh targets by testing only the target triangles
inside the source swept bounds and reducing hits back to the target owner.

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
allocation-free in-place heap sorter. Closest-hit raycasts use the same
distance/collider-ID ordering, so equal-distance hits do not depend on partition
or sparse-set traversal order. Keep all-hit result buffers owned by the caller
or context that issues the query.

When diagnostics are enabled, raycast calls emit a `RayQuery` event. Swept
sphere calls use the same event kind with `ScalarA` set to the sweep radius and
`DataB` set to the captured hit count.

The candidate path is:

1. prepare the worker for the 3D query segment.
2. ask GridForge `GridTracer.TraceLine(...)` or `GetCoveredVoxels(...)` for topology-aware voxel candidates.
3. suppress duplicate voxels and inspect each voxel's `PhysicsPartition`.
4. resolve dynamic, kinematic, and static collider IDs through the context physics service.
5. filter by layer mask.
6. skip colliders already checked in this query.
7. call the collider's `ColliderOverlapsRay(...)`.
8. build `Physics3DHit` values from intersection points, normals, and distance.
9. sort `RaycastAll` results by distance.

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

Current swept-sphere support covers sphere, capsule, cuboid, finite cylinder,
mesh, and compound targets. Mesh targets query local-BVH triangle candidates,
then test triangle faces, edges, and vertices for deterministic time of impact.
Compound targets reduce over owned parts in stable declaration order while the
public hit remains the owning compound collider.

For registered collider sources whose current pose matters, `Query3D` also
exposes:

- `SweepCapsule(source, displacement, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCapsuleAll(source, displacement, layerMask, results, excludedCollider, includeTriggers)`
- `SweepCuboid(source, displacement, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCuboidAll(source, displacement, layerMask, results, excludedCollider, includeTriggers)`
- `SweepCylinder(source, displacement, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCylinderAll(source, displacement, layerMask, results, excludedCollider, includeTriggers)`
- `SweepConvexMesh(source, displacement, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepConvexMeshAll(source, displacement, layerMask, results, excludedCollider, includeTriggers)`
- `SweepCompound(source, displacement, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCompoundAll(source, displacement, layerMask, results, excludedCollider, includeTriggers)`

These source sweeps use support-mapped conservative advancement for convex 3D
sources. The source collider is skipped automatically, `excludedCollider` can
skip an additional collider, and all-hit overloads retain caller-owned result
buffers plus distance/collider-ID ordering. Concave mesh sources are rejected;
author concave-looking movers as stable `LSCompoundCollider` convex parts.

`StiffBody` continuous collision detection reuses this service as an opt-in
movement sweep. Public `SweepSphere` and `SweepSphereAll` remain all-target
queries: they can return movable dynamic, kinematic, immovable, and bodyless
colliders according to the normal layer, trigger, and exclusion filters. Body
CCD uses an internal static-style swept-sphere collector for its
static/kinematic leg, so only kinematic/static partition IDs are copied and
movable dynamics are handled by the separate relative-motion CCD path. The
internal collector keeps the same deterministic distance/collider-ID ordering
as host-facing swept-sphere queries for the targets it includes.

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

- `OverlapCircle(center, radius, out hit)`
- `OverlapCircle(center, radius, layerMask, out hit)`
- `OverlapCircleAll(center, radius, results)`
- `OverlapCircleAll(center, radius, layerMask, results)`
- `OverlapAabb(center, size, out hit)`
- `OverlapAabb(center, size, layerMask, out hit)`
- `OverlapAabbAll(center, size, results)`
- `OverlapAabbAll(center, size, layerMask, results)`
- `OverlapPolygon(vertices, out hit)`
- `OverlapPolygon(vertices, layerMask, out hit)`
- `OverlapPolygonAll(vertices, results)`
- `OverlapPolygonAll(vertices, layerMask, results)`
- `Raycast(start, end, out hit)`
- `Raycast(start, end, layerMask, out hit)`
- `RaycastAll(start, end, results)`
- `RaycastAll(start, end, layerMask, results)`
- `SweepCircle(start, end, radius, out hit)`
- `SweepCircle(start, end, radius, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCircleAll(start, end, radius, results)`
- `SweepCircleAll(start, end, radius, layerMask, results, excludedCollider, includeTriggers)`

All-hit methods clear the caller-provided `SwiftList<Physics2DHit>`, write hits
into it, return the hit count, and sort by distance with collider ID as the
deterministic tie-breaker. Closest-hit overlap, `Raycast`, and `SweepCircle`
overloads use the same ordering and return the first hit through an
`out Physics2DHit`.

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

`OverlapAabb` and `OverlapPolygon` use the same GridForge-backed candidate
gatherer with the query area's 2D bounds, then run exact fixed-point SAT and
closest-point checks against circle, AABB, convex polygon, and compound targets.
Polygon query vertices must be convex and non-collinear; edge-touching counts as
overlap. The service validates and computes area bounds once before candidate
testing so repeated candidate checks do not allocate or rebuild query shapes.

The segment raycast path projects the segment's 2D bounds into the same
GridForge-backed candidate gatherer, then runs deterministic shape math against
circle, AABB, and convex polygon colliders. Zero-length segments return no
hits. Starting inside a collider returns a zero-distance hit. A collider that
spans multiple voxels is still reported once because candidate gathering
stamps duplicate collider visits before exact shape testing.

`SweepCircle` uses the same candidate gatherer with the swept circle's expanded
bounds. It performs deterministic circle-vs-circle and circle-vs-convex-shape
sweeps, supports layer masks, optional trigger inclusion, and an excluded
collider for body CCD self/hierarchy filtering. It is the pure 2D equivalent of
the 3D swept-sphere query path; it is not a mixed 2D/3D bridge.

Public pure 2D sweep queries report movable dynamic, kinematic, immovable, and
bodyless colliders. `StiffBody2D` CCD uses an internal static-style swept-circle
collector for the static/kinematic leg, mirroring 3D: bodyless, immovable, and
kinematic targets are included through kinematic/static partition membership,
while movable dynamic targets are left to the relative dynamic CCD candidate
index.

Current hit data is `Physics2DHit`: collider, optional body, point, normal, and
distance.

## Mixed Queries

`GravitasQueryMixedService` exposes explicit cross-dimensional sweeps:

- `SweepSphereAgainst2D(start, end, radius, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepSphereAgainst2DAll(start, end, radius, layerMask, results, excludedCollider, includeTriggers)`
- `SweepCircleAgainst3D(start, end, radius, slabCenterY, halfThickness, layerMask, out hit, excludedCollider, includeTriggers)`
- `SweepCircleAgainst3DAll(start, end, radius, slabCenterY, halfThickness, layerMask, results, excludedCollider, includeTriggers)`

The mixed service keeps `Query2D` and `Query3D` pure by design. Use mixed
queries only when the host explicitly wants cross-dimensional query truth or
when mixed CCD is enabled through `PhysicsRuntimeMode.Mixed`.

Mixed query candidate gathering uses `PhysicsMixedPartition` payloads attached
to GridForge voxels. The gatherer refreshes the relevant mixed partition side,
scans deterministic voxel identities, suppresses duplicate collider IDs, filters
by layer and bounds, and orders hits by distance with 3D ID and 2D ID
tie-breakers. Single-hit mixed queries keep the best candidate directly with
the same ordering rule; all-hit overloads sort the caller-owned result buffer.

`SweepSphereAgainst2D` sweeps a 3D sphere center against embedded 2D mixed
slabs. 2D circles are treated as finite vertical cylinders; AABB and polygon
slabs use their finite mixed prism bounds for the current query policy. Hits
against 2D circle slabs report `PhysicsQueryReducerKind.Exact`; hits accepted
through AABB or polygon prism bounds report
`PhysicsQueryReducerKind.ConservativeFallback`.

`SweepCircleAgainst3D` sweeps a pure 2D circle embedded at the supplied slab Y
center and half-thickness against 3D targets. Sphere targets use an exact
finite-slab projection: vertical overlap determines the sphere's effective
planar reach, so tall slabs and slab-corner cases do not inflate the horizontal
sweep radius. Cuboid targets project the cuboid portion intersecting the slab's
Y interval into X/Z and sweep the source circle against that convex projection,
so rotated cuboids do not rely on the generic sphere proxy. World-Y capsule and
finite-cylinder targets use exact vertical-interval reducers: capsule cap reach
is reduced by slab interval distance, while finite cylinders require interval
overlap before the planar circle sweep is accepted.

Mesh, compound, rotated capsule, and rotated finite-cylinder targets currently
retain the conservative swept-sphere worker fallback. Mesh targets still use
triangle candidate acceleration and face/edge/vertex TOI checks within that
fallback; compound targets return one hit on the owning compound collider after
reducing over stable part order. The fallback now uses a circumsphere radius for
the source slab, so it can report earlier or extra hits but is not allowed to
miss a finite-slab corner case. Hits accepted through these fallback paths
report `PhysicsQueryReducerKind.ConservativeFallback`.

When diagnostics are enabled, mixed queries emit both `MixedQuery` and
`QuerySummary` events. `MixedQuery` reports the closest mixed hit and accepted
hit count. `QuerySummary` reports candidate-level exact reducer attempts,
accepted hits, fallback hits, and rejected conservative fallback candidates so
hosts can inspect query quality without reverse-engineering reducer labels.

`StiffBody` and `StiffBody2D` mixed CCD use these APIs only when the context is
in `PhysicsRuntimeMode.Mixed`. Pure `Both` mode still advances 2D and 3D
independently and does not run cross-dimensional CCD.

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

`Physics2DHit` is a readonly struct containing:

- `Collider`
- `Body`
- `Point`
- `Normal`
- `Distance`

`Body` is `collider.Body`, so bodyless/static 2D query hits can have a collider
with a null body.

`PhysicsMixedHit` is a readonly struct containing:

- `Collider3D`
- `Collider2D`
- `Body3D`
- `Body2D`
- `Point3D`
- `Point2D`
- `Normal3DTo2D`
- `NormalFor3DSource`
- `NormalFor2DSource`
- `ReducerKind`
- `Distance`
- `Direction3D`

`Normal3DTo2D` follows the mixed contact invariant: it points from the 3D side
toward the embedded 2D volume. CCD source helpers expose the normal orientation
needed by the moving source so velocity clamping does not have to reinterpret
the invariant at every call site. `ReducerKind` is
`PhysicsQueryReducerKind.Exact` when the hit was accepted by shape-specific
mixed query math and `PhysicsQueryReducerKind.ConservativeFallback` when the
hit came from a safe conservative proxy or bounds reducer.

## Query Hardening Targets

The completed
[`Query And Mixed Swept Shape Hardening`](../feature-work/done/2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
plan owns the current public query API shape, 2D query parity, convex source
sweeps, fallback labels, and query diagnostics. Remaining mixed finite-slab
exactness and convex mesh source scaling work is tracked in
[`Mixed Query Finite-Slab Reducer Completion`](../feature-work/2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md).

Longer-term query state objects remain evidence-gated: introduce caller-owned or
pooled query job/state objects only when a real host needs concurrent queries
against one context.
