# Query Reference

This page contains the detailed query API matrix, reducer policy, batch
contract, mesh-source boundary, and per-family implementation notes. For the
short guide, read [Query Services](QUERY_SERVICES.md).

## Reducer Labels

| Label                  | Meaning                                                                                            |
| ---------------------- | -------------------------------------------------------------------------------------------------- |
| `Exact`                | The accepted hit is produced by shape math that matches the documented source and target geometry. |
| `ConservativeFallback` | The accepted hit may be earlier or extra, but must not create false negatives.                     |
| `NotSupported`         | No public runtime API exists for that source/target family.                                        |

Mixed query hits expose this through `PhysicsMixedHit.ReducerKind`. 2D and 3D
public query paths expose exact public hits.

## Public Query Surface

### 3D

All 3D all-hit results use caller-owned buffers and distance/collider-ID
ordering, except cone volumes sort by axial distance before collider ID.

| Surface                                                         | Source                                                          | Targets                                                               |
| --------------------------------------------------------------- | --------------------------------------------------------------- | --------------------------------------------------------------------- |
| `Query3D.Raycast`, `RaycastAll`                                 | bounded 3D segment                                              | sphere, capsule, cuboid, finite cylinder, finite cone, mesh, compound |
| `Query3D.SweepSphere`, `SweepSphereAll`                         | 3D sphere                                                       | sphere, capsule, cuboid, finite cylinder, finite cone, mesh, compound |
| `SweepCapsule`, `SweepCuboid`, `SweepCylinder`, `SweepCone`     | registered convex 3D collider at current pose plus displacement | supported 3D targets, concave mesh target triangles, compound         |
| `SweepConvexMesh`                                               | convex `LSMeshCollider` at current pose plus displacement       | supported 3D targets and concave mesh target triangles                |
| `SweepCompound`                                                 | authored `LSCompoundCollider` from supported convex parts       | supported 3D targets                                                  |
| `OverlapCone`, `OverlapConeAll`                                 | apex-origin finite cone volume                                  | 3D colliders                                                          |
| `OverlapCircle`, `OverlapCircleInDirection`, `OverlapCircleAll` | X/Z circle proximity query                                      | 3D colliders through closest-surface projection                       |

Reducer notes:

- Capsule and finite-cylinder raycasts solve against the authored segment and
  return Q32.32 spatial distances with one final round-to-even narrowing. Hit
  points use the same exact authored-chord interpolation, so long segments do
  not amplify parameter rounding or lose small representable components.
- Finite-cone raycasts use the corresponding centered-cone physical-distance
  interval. Axial clipping, conic coefficients, discriminant evaluation, and
  root selection remain wide, while the closed interval owns side, apex, flat
  base, and rim contact without downstream feature deduplication. Cone bounds,
  support points, axial ordering, and closest-axis witnesses retain the exact
  squared length of an accepted near-unit axis rather than treating it as a
  mathematically exact unit vector.
- Swept-sphere capsule and mesh-edge reducers use the same full-domain
  finite-axis admission. Finite-cylinder sweeps currently use a conservative
  sharp-rim dilation rather than the rounded rim of the true Minkowski sum.
- mesh targets query triangle BVH candidates and report the owning collider.
- registered convex source sweeps use exact support-mapped conservative
  advancement and skip the source collider.
- high-vertex convex mesh sources use deterministic support-tree pruning.
- concave source meshes throw; author stable convex compound source parts.
- cone volumes use exact supported convex target checks and stable compound
  reduction.
- `OverlapCircle` is an X/Z proximity query, not swept movement.

### 2D

2D queries operate over `Vector2d` values in the X/Z plane. All-hit results sort
by distance and collider ID.

| Surface                               | Source                 | Targets                                         |
| ------------------------------------- | ---------------------- | ----------------------------------------------- |
| `OverlapCircle`, `OverlapCircleAll`   | 2D circle              | circle, capsule, AABB, convex polygon, compound |
| `OverlapAabb`, `OverlapAabbAll`       | 2D AABB area           | circle, capsule, AABB, convex polygon, compound |
| `OverlapPolygon`, `OverlapPolygonAll` | convex 2D polygon area | circle, capsule, AABB, convex polygon, compound |
| `Raycast`, `RaycastAll`               | 2D segment             | circle, capsule, AABB, convex polygon, compound |
| `SweepCircle`, `SweepCircleAll`       | 2D circle              | circle, capsule, AABB, convex polygon, compound |

Reducer notes:

- overlaps use exact SAT and closest-point checks.
- polygon query vertices must be convex and non-collinear.
- edge-touching counts as overlap.
- zero-length ray segments miss.
- starting inside a raycast target returns distance zero.
- Capsule raycasts and circle sweeps use the full-domain finite-segment capsule
  distance interval, with target radius and sweep expansion kept separate until
  the exact comparison. Hit points are reconstructed directly from the
  authored segment and returned distance.
- compounds report the owner once through stable part reduction.

### Mixed

Mixed query hits sort by distance, then 3D collider ID, then 2D collider ID.

| Surface                                           | Source                                                | Targets                                                                     |
| ------------------------------------------------- | ----------------------------------------------------- | --------------------------------------------------------------------------- |
| `SweepSphereAgainst2D`, `SweepSphereAgainst2DAll` | 3D sphere                                             | 2D circle slab, capsule slab, AABB slab, convex polygon slab, compound slab |
| `SweepCircleAgainst3D`, `SweepCircleAgainst3DAll` | 2D circle embedded in a finite Y slab                 | 3D sphere, capsule, cuboid, finite cylinder, finite cone, mesh, compound    |
| Concave/raw mesh-source sweeps                    | concave `LSMeshCollider` or raw mesh as moving source | no public runtime API                                                       |

Reducer notes:

- Capsule-slab boundary reducers use full-domain planar and spatial
  finite-segment capsule intervals, but the current horizontal-rim
  decomposition conservatively overexpands the true sphere-dilated boundary.
  Circle-slab axial and radial clipping also remains wide until the final
  authored-segment distance. The capsule reducer's current `Exact` label is
  tracked for correction with the rim model.
- Circle slabs are reduced through a conservative sharp-rim expanded cylinder;
  their current `Exact` label is a known model-labeling defect pending a rounded
  finite-cylinder reducer.
- AABB, convex-polygon, and stable compound reductions report `Exact`.
- supported 3D primitive slabs, finite cones at any rotation, mesh triangles,
  and compounds report `Exact`.
- conservative fallback labels are reserved for safe proxy candidates that must
  not create false negatives.
- concave/raw mesh-source sweeps are `NotSupported`; author stable convex
  compound source parts.

All public all-hit APIs use service-owned scratch and caller-owned hit buffers.

## Batch APIs

Every public query family also has typed batch access on its owning service.
Batch APIs keep dimensional semantics explicit instead of routing all queries
through one tagged request type.

| Service      | Batch families                                                                     |
| ------------ | ---------------------------------------------------------------------------------- |
| `Query3D`    | raycast, swept sphere, registered source sweeps, cone volume, X/Z circle proximity |
| `Query2D`    | raycast, overlap, swept circle                                                     |
| `QueryMixed` | sphere-against-2D and circle-against-3D                                            |

Closest-hit batches take a typed request span and one output hit span. The
output span must contain at least one slot per request. A miss writes the
default hit value for that request, and the return value is the number of
requests that hit.

All-hit batches take a typed request span, a caller-owned shared hit
`SwiftList<T>`, and a `Span<PhysicsQueryHitRange>` with one slot per request.
The shared hit list is cleared once at batch start. Each request writes one
range, including misses and zero-length movement requests.

Structurally invalid input, such as an undersized output span, invalid polygon
vertex range, invalid AABB size, or invalid sweep radius, throws before caller
output buffers are mutated.

Each query service exposes summary counters for the last batch call:

- `LastBatchRequestCount`
- `LastBatchHitCount`
- `LastBatchCandidateCount`
- `QueryMixed.LastBatchMeshTriangleCandidateCount`

These counters are frame-local tuning aids and benchmark signals, not replay
state.

## Mesh-Source Boundary

Mesh-as-source means a public query where mesh geometry is the moving sweep
source. Convex mesh sources are supported through `Query3D.SweepConvexMesh` and
`SweepConvexMeshAll`.

Capsule, cuboid, finite-cylinder, and finite-cone collider sources have explicit
source-sweep APIs. Authored compounds made from supported convex parts are
supported through `Query3D.SweepCompound` and `SweepCompoundAll`.

Concave mesh sources are intentionally outside the public runtime query surface.
That boundary avoids hiding `source triangles x target candidates` work, runtime
convex decomposition, and ambiguous concave-source ordering behind a simple
query call. Hosts should author concave-looking movers as offline decomposed
`LSCompoundCollider` assets with stable convex part order.

`SweptSphereQueryWorker.TrySweep(LSCollider collider, ...)` is not a mesh-source
query. That worker is prepared with a swept sphere source, then tests one target
collider against that swept sphere.

## 3D Raycasts

`RaycastAll` clears the caller-provided `SwiftList<Physics3DHit>`, writes hits,
returns the hit count, and sorts by distance then collider ID. Closest-hit
raycasts use the same ordering rule.

Candidate path:

1. prepare the segment worker.
2. ask GridForge for topology-aware voxel candidates.
3. suppress duplicate voxels and inspect each voxel's `PhysicsPartition`.
4. resolve collider IDs through the owning context.
5. filter by layer mask.
6. skip duplicate colliders.
7. run `ColliderOverlapsRay(...)`.
8. build `Physics3DHit`.
9. sort all-hit results.

Mesh ray overlap is triangle-level and queries the mesh triangle BVH before
testing candidate triangles.

## 3D Sweeps And Volumes

`SweepSphere` and `SweepSphereAll` use a segment-based true 3D swept sphere:

- non-positive radius, zero direction, or zero segment length returns no hits.
- starting overlap returns a zero-distance hit.
- `excludedCollider` is skipped before duplicate stamping.
- all-hit results sort by impact distance and collider ID.
- hit distance is the swept center's time-of-impact distance.
- hit point is the target surface point closest to the swept center at impact.

Registered source sweeps use support-mapped conservative advancement for convex
3D sources. The source collider is skipped automatically, and `excludedCollider`
can skip one additional collider.

`OverlapCone` and `OverlapConeAll` query an apex-origin finite cone volume.
Supported convex targets use deterministic support-mapped cone-volume
intersection. Compound targets scan parts in stable authored order and report
the owner once. Concave-mesh triangle edges use the shared exact finite-cone
point interval. Its high-resolution authored-chord interpolation keeps
spatially distinct witnesses on very long edges; the admitted interval remains
authoritative if the final lattice point rounds just outside a continuous
sub-raw boundary. Triangle face-interior intersections with no edge crossing
remain separately tracked; the current axis/triangle fallback only covers the
subset where the cone axis itself pierces the triangle.

The 3D `OverlapCircle` family is an X/Z proximity query for 3D colliders. It is
not a `Query2D` call, not a swept circle, and not a swept sphere.

## 2D Queries

2D positions are `Vector2d` values in the X/Z plane. Use `Vector3d.ToVector2d()`
when converting from a `FixedTransform`.

Candidate gathering:

1. project query bounds into the 2D Y=0 partition plane.
2. scan covered GridForge cells and voxels.
3. inspect `PhysicsPartition2D` payloads.
4. suppress duplicate collider hits with 2D query-version stamps.
5. reject inactive colliders, layer misses, and separated 2D bounds.
6. run exact 2D shape checks.
7. sort hits by distance and collider ID.

`OverlapAabb` and `OverlapPolygon` run exact fixed-point SAT and closest-point
checks against circle, capsule, AABB, convex polygon, and compound targets.
Polygon query vertices must be convex and non-collinear; edge-touching counts as
overlap.

`Raycast` returns the closest segment hit from `start` to `end`. Zero-length
segments miss. Starting inside a collider returns a zero-distance hit. Capsule
targets use one exact finite-segment interval rather than independent side and
endpoint tests, so authored center/axis/half-length geometry and the nearest
outer feature remain authoritative even at extreme scale.

`SweepCircle` is the 2D swept movement/query path used by 2D CCD. It is not a
mixed 2D/3D bridge.

## Mixed Queries

Mixed query candidate gathering uses `PhysicsMixedPartition` payloads attached
to GridForge voxels. The gatherer refreshes the relevant mixed partition side,
scans deterministic voxel identities, suppresses duplicate collider IDs, filters
by layer and bounds, and orders hits by distance with 3D ID and 2D ID
tie-breakers.

`SweepSphereAgainst2D` sweeps a 3D sphere center against embedded 2D slabs:

- circles use full-domain axial/radial clipping against a conservatively
  expanded finite vertical cylinder. The sharp expanded rim contains the true
  rounded sphere/cylinder Minkowski boundary; correcting the reducer and its
  current `Exact` label is tracked separately.
- capsule boundary segments use full-domain finite-segment capsule intervals;
  the current horizontal-rim decomposition is a conservative overexpansion
  pending an exact extruded-capsule sweep.
- AABB and convex polygon slabs use finite-prism reducers.
- compounds reduce supported parts in authored order and report the owner.

`SweepCircleAgainst3D` sweeps a 2D circle embedded at a supplied slab Y center
and half-thickness against 3D targets:

- spheres use finite-slab projection.
- cuboids project the slab-intersecting portion into X/Z.
- capsule edges use full-domain finite-segment capsule intervals, and cylinders
  use deterministic finite-slab reducers.
- finite cones use slab-clipped or support-mapped convex reducers depending on
  orientation.
- mesh targets clip candidate triangles to the slab Y interval, project into
  X/Z, and report the owning `LSMeshCollider`.
- compounds reduce supported parts in authored order and report the owner.

When diagnostics are enabled, mixed queries emit both `MixedQuery` and
`QuerySummary` events. `QuerySummary` reports exact reducer attempts, accepted
hits, fallback hits, and rejected conservative fallback candidates.

## Reentrancy And Allocation

Query services keep mutable scratch buffers on the service instance. Do not run
multiple queries concurrently against the same context service.

All-hit APIs use caller-owned buffers. Batch APIs reuse service-owned scratch
and caller-owned buffers. They do not allocate per-request arrays.

Polygon batches keep vertex ownership explicit: the request stores a
`VertexStart`/`VertexCount` range into the vertex span supplied to the batch
call, and the caller must keep that span stable for the duration of the call.

## Source Map

| Area                | Source                                                                                                             |
| ------------------- | ------------------------------------------------------------------------------------------------------------------ |
| 3D query service    | [`src/Gravitas/Queries/3D`](../../src/Gravitas/Queries/3D)                                                         |
| 2D query service    | [`src/Gravitas/Queries/2D`](../../src/Gravitas/Queries/2D)                                                         |
| Mixed query service | [`src/Gravitas/Queries/Mixed`](../../src/Gravitas/Queries/Mixed)                                                   |
| Query hit ranges    | [`src/Gravitas/Queries/Common/PhysicsQueryHitRange.cs`](../../src/Gravitas/Queries/Common/PhysicsQueryHitRange.cs) |
| CCD reference       | [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md)                                                |
