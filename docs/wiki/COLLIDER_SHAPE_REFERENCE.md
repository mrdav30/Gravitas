# Collider Shape Reference

Collider shape code owns deterministic geometry, bounds, authored shape data,
narrow-phase dispatch, and compound/mesh identity rules. This page is the
technical companion to [Collision Pipeline](COLLISION_PIPELINE.md).

## Quick Read

- Runtime colliders are not serialized asset definitions.
- Shape definitions are data-only authoring/import surfaces.
- Compound colliders own one public collider identity and stable private part
  order.
- Mesh colliders can be valid simulation/query targets; concave mesh source
  sweeps are outside the public runtime query surface.
- Mesh topology and acceleration structures are immutable after construction;
  pose and scale rebuild only scale-dependent runtime caches.
- Narrow phase uses stable pair priority and stable contact-normal orientation.
- SAT axes, mesh triangle candidates, and compound parts are processed in
  deterministic order.

## Runtime Shape State

`LSCollider` and `LSCollider2D` separate host-facing collider identity from
dense mutable state used by shape rebuilds, partition ownership, query duplicate
suppression, pair cleanup, and hierarchy filtering.

The 3D runtime-shape snapshot watches:

- world-space center.
- rotation.
- the standalone owner scale and compound-part scale as separate factors.
- local offset.
- unscaled local size.
- unscaled local radius.

Mutating `LocalOffset`, `Radius`, or `Size` marks the runtime shape dirty.
Changing host/body scale, position, or rotation is detected from the snapshot on
the next `Simulate()` call. If the snapshot has not changed, the collider skips
the rebuild and keeps existing partition state. Shape mutation wakes a sleeping
bound body before broad-phase refresh.

Every changed 3D snapshot is prepared and validated before its canonical
geometry, bounds, mesh transforms, compound parts, runtime version, mass
properties, or partition ownership are published. A failed live scale,
rotation, or geometry candidate therefore leaves the last committed runtime
shape intact; correcting the input allows the next simulation pass to rebuild
normally. Compound preparation walks every part in authored order and publishes
none of them until every candidate succeeds.

2D colliders use the same helper pattern where the payload is dimension-free.
Runtime-shape dirtying uses a 2D snapshot payload; query stamps, pair state, and
hierarchy state are shared concepts. Partition coordinates stay 2D-specific
because they store X/Z planar coverage.

## Hierarchy Versus Compound

Hierarchy binding and compound colliders solve different problems:

| Concept                | Meaning                                                                                                             |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `SetParent(...)`       | Links independently registered colliders so same-host parent/child or sibling collisions can be suppressed.         |
| `LSCompoundCollider`   | Owns several authored parts under one public 3D collider ID, body binding, broad-phase entry, and event surface.    |
| `LSCompoundCollider2D` | Owns several authored 2D parts under one public 2D collider ID, body binding, broad-phase entry, and event surface. |

Hierarchy state stores a dimension-tagged top-parent collider key. It does not
walk engine transform trees during simulation. When a parent collider
deactivates, child bindings are cleared before the parent collider ID returns to
the reusable ID pool.

Compound parts are not registered with the physics service, cannot be parented
independently, and are scanned in stable declaration order by the owning
compound collider. Public `CompoundColliderPart` and `CompoundColliderPart2D`
values are data-first authored descriptors, not independent runtime identities.

## Authoring Surfaces

| Authoring type              | Runtime owner                                              |
| --------------------------- | ---------------------------------------------------------- |
| `ColliderShapeDefinition`   | built-in `LSCollider` implementations and `LSCompoundCollider` parts     |
| `CompoundColliderPart`      | 3D compound authoring                                      |
| `ColliderShapeDefinition2D` | built-in `LSCollider2D` implementations and `LSCompoundCollider2D` parts |
| `CompoundColliderPart2D`    | 2D compound authoring                                      |

Shape definitions should be used by importers, tooling, and offline-authored
compound assets. Runtime collider shells own context binding, collider IDs,
partition coordinates, pair state, events, and query stamps.

`CompoundColliderPart` scale-safely normalizes its authored local rotation once
at construction, with a zero quaternion resolving to identity. The stored value
is therefore the single orientation consumed by bounds, mass properties,
queries, diagnostics, and replay hashing.

## 3D Shape Families

| Shape           | Runtime notes                                                                                                                                                                        |
| --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Sphere          | Center/radius support, simple mass and inertia.                                                                                                                                      |
| Capsule         | Canonical center, normalized rigid frame, local axis, full cylindrical axis length, and radius; rotation-aware projected frontal area and solid capsule mass properties with sphere and thin-rod limits. |
| Cuboid          | Face, edge, corner support; `Cuboid` covers axis-aligned and oriented cuboid dispatch.                                                                                               |
| Finite cylinder | Canonical center, normalized rigid frame, local axis, full axis length, and radius; flat-cap and side relations derive conceptual endpoints only inside exact operations.          |
| Finite cone     | Canonical center, normalized rigid frame, local base-to-apex axis, full height, and radius; analytic support/closest-surface geometry and COM stay center-relative.                 |
| Mesh            | Convex or concave target geometry with triangle candidates and cached mass properties.                                                                                               |
| Compound        | Stable part-order reduction under one public collider identity, with solid-volume mass distribution and deterministic equal-mass fallback when every part measure quantizes to zero. |

Cones use analytic support and closest-surface geometry instead of generating a
runtime triangle fan. Mesh colliders use cached closed-volume mass properties by
default when angular dynamics are enabled and keep an explicit uniform
thin-shell mass policy for open meshes.

For a 3D capsule with normalized motion direction `n`, world-space axis
`Rotation * Vector3d.Up`, radius `r`, and cylindrical height `h`, the projected
frontal area is `pi*r^2 + 2*r*h*sqrt(max(0, 1 - dot(n, axis)^2))`. A near-zero
direction returns the collider's total `Area`, matching the other 3D primitive
drag-area APIs. Solid capsule inertia assigns cylinder and cap masses in
proportion to volume. The paired hemispheres contribute
`mCaps*(2*r^2/5 + d^2 + 3*d*r/4)` transversely for `d = h/2`, including their
centroid displacement from the cap centers, and `2*mCaps*r^2/5` axially. A
capsule with no cylindrical span uses the solid-sphere limit `2*m*r^2/5`. If
fixed-point scaling leaves a positive span but quantizes the radius and both
volumes to zero, the tensor uses the thin-rod limit
`diag(m*h^2/12, 0, m*h^2/12)` before applying any requested parallel-axis shift.

A finite cylinder must retain a positive scaled full axis length when its
runtime shape is rebuilt. Gravitas does not require an odd raw-unit length to
have a representable half-height or materialized cap centers; exact centered-axis
relations retain the full length through classification and witness selection.
A positive authored height that scales to zero is still rejected because it
does not retain the cylinder's flat-cap contract. Capsules remain different by
design: a collapsed capsule center segment is the well-defined sphere limit.

3D compound colliders distribute uniform-density mass by each solid part's
volume rather than by the public `Area` value, whose drag/diagnostic meaning is
shape-specific. Closed-volume meshes contribute their validated scaled volume.
Meshes explicitly authored with `SurfaceApproximation` contribute scaled
triangle surface area and use physical uniform thin-shell COM/inertia
integration. Surface density and solid volume density are not dimensionally
interchangeable, so the policy stays an explicit authoring choice. Fixed-point
mass division preserves the requested total mass by assigning the rounding
residual to the last positive-weight authored part. If every part measure
quantizes to zero, part centers and masses use equal authored-order weights with
the same residual rule.

Part centers of mass and anisotropic center tensors are transformed into the
compound owner's local frame before parallel-axis shifts. Aggregate frontal area
remains the deterministic sum of part projections; it is conservative for
overlapping projections rather than an exact silhouette union. Aggregate
`ScaledRadius` encloses the compound's world bounds about the owner center.
There is no aggregate `ScaledSize` contract: a world AABB cannot represent an
owner-local size under arbitrary part rotations, and canonical primitive
geometry exposes radius, full axis length, or half-extents explicitly.

## 3D Shape-Pair Matrix

| A / B    | Sphere    | Capsule   | Cuboid    | Cylinder  | Cone      | Mesh      | Compound  |
| -------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- |
| Sphere   | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Capsule  | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Cuboid   | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Cylinder | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Cone     | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Mesh     | Supported | Supported | Supported | Supported | Supported | Supported | Supported |
| Compound | Supported | Supported | Supported | Supported | Supported | Supported | Supported |

`Cylinder/Mesh` and `Cone/Mesh` are normalized to `Mesh/Cylinder` and
`Mesh/Cone` by pair priority so contact data is written in the
mesh-to-curved-primitive direction.

## 2D Shape Families

| Shape            | Runtime notes                                                                                             |
| ---------------- | --------------------------------------------------------------------------------------------------------- |
| Circle           | Center/radius tests, support points, area, and scalar moment.                                             |
| Capsule          | Segment plus radius; analytic closest-point geometry rather than polygon approximation.                   |
| Axis-aligned box | X/Z-aligned bounds and 2D separating-axis checks.                                                         |
| Convex polygon   | Deterministic vertex-order SAT, triangle helper output, convexity validation, and collinearity rejection. |
| Compound         | Stable part-order reduction under one public 2D collider identity.                                        |

Boxes and polygons use 2D separating-axis tests over deterministic
center-relative vertex order. Absolute world vertices are best-effort
presentation values through `TryGetWorldVertex`; collision, query, mixed-prism,
and replay paths keep the collider origin separate so a conceptual vertex may
cross a `Fixed64` scalar face without deforming the polygon.
`LSPolygonCollider2D` rejects concave and collinear input up front. Concave 2D
runtime collision should be authored as stable convex compound parts.

## 2D Shape-Pair Matrix

| A / B          | Circle    | Capsule   | AABB      | Convex Polygon | Compound  |
| -------------- | --------- | --------- | --------- | -------------- | --------- |
| Circle         | Supported | Supported | Supported | Supported      | Supported |
| Capsule        | Supported | Supported | Supported | Supported      | Supported |
| AABB           | Supported | Supported | Supported | Supported      | Supported |
| Convex Polygon | Supported | Supported | Supported | Supported      | Supported |
| Compound       | Supported | Supported | Supported | Supported      | Supported |

Compound/primitive and compound/compound pairs scan owned parts in stable
declaration order and reduce back to the public owning collider identity.

## Narrow Phase Rules

The separating-axis invariant is the broad rule behind convex SAT paths: if a
candidate axis projects two convex shapes into non-overlapping intervals, the
shapes cannot be colliding. A collision is possible only when every required
axis overlaps.

Candidate axes must be generated and tested in stable order. Contact normals
must be oriented by the pair convention rather than by ad hoc shape-pair sign
fixes. If a response pair appears to need an impulse sign exception, audit
normal orientation and pair priority first.

SAT and mesh candidate paths use context-owned scratch state through
`GravitasWorldContext`. `CollisionSatScratch` owns reusable collision context,
object-info buffers, triangle candidate buffers, and SAT axis sets, including
the ordered cuboid/capsule axes, for one world context. Every reducer clears its
owned logical buffer before rebuilding it; concurrent worlds keep isolated
scratch, while repeated checks in the same world avoid per-check allocations and
pool churn.

## Mesh Policy

Mesh colliders are valid simulation colliders and query targets. 3D raycasts and
swept-sphere queries test mesh triangle candidates and return the owning
`LSMeshCollider` once. Convex mesh source sweeps also support concave mesh
targets by testing only target triangles inside the source swept bounds and
reducing hits back to the target owner.

`MeshColliderMode.Convex` is an enforced geometry contract, not an optimization
hint. Construction accepts either one connected, consistently wound, closed
convex manifold shell or one connected, consistently wound, open coplanar
triangulation that fills a single convex polygon. It rejects unused vertices,
duplicate faces, disconnected components, non-manifold edges, inconsistent
winding, disconnected vertex links, reflex closed edges, bent or folded open
surfaces, holes, and overlapping open triangles. Exact-position duplicates are
welded only in the temporary topology view so common hard-normal and UV seam
exports validate without changing authored vertices, triangle order, or query
tie-breaking.

`MeshColliderMode.Concave` supports arbitrary nonconvex open surfaces, closed
shells, and disconnected triangle surfaces. Both modes reject unreferenced
vertices. `PhysicsMesh.IsClosedSurface` and `LSMeshCollider.IsClosedSurface`
cache whether the complete exact-position-welded surface is one connected,
consistently wound, closed two-manifold. The property describes topology only;
`TryGetClosedVolumeMassProperties(...)` remains the richer volume and
representability contract. `MeshVolumeValidationResult.NonManifoldVertex`
distinguishes a pinched or disconnected vertex fan from an ordinary open edge.

Mesh closest-surface queries always return a point on an authored triangle.
They seed an exact upper-bound candidate, query the local triangle BVH inside a
conservative cube that contains every point capable of improving that bound,
and use a stable, allocation-free full triangle scan only when the bound cannot
be represented. Broad-phase bounds are never substituted for mesh geometry,
including mesh/sphere contacts and circle overlap queries. Full-domain
FixedMathSharp triangle and squared-distance predicates preserve the closest
authored candidate even when public Q32.32 distance values would saturate.

This validation is intentionally breaking for assets that previously selected
`Convex` despite violating its geometry contract. Author raw terrain, triangle
soups, bent open surfaces, and nonconvex shells as `Concave`; use deterministic
convex decomposition when a moving or support-mapped collider needs multiple
convex parts.

Runtime mesh points follow one explicit centered transform contract:
`origin + rotation * Scale(sourcePoint - authoredBoundsCenter, ownerScale,
partScale)`. Rotation must be a normalized, representable quaternion. Owner and
part scale factors remain separate until each final centered coordinate is
formed, must be strictly positive and diagonally applied, and must preserve
representable centered vertices, triangle areas, total area, and selected mass
properties. Invalid standalone meshes reject before collider registration.
Compound owners prepare every mesh part before publishing any part, so a failed
scale/rotation change cannot leave half-updated geometry.

Scaled mesh bounds use FixedMathSharp's full-domain nearest-even midpoint.
Same-sign extreme coordinates are therefore valid when the exact bounds size,
triangle geometry, and selected mass properties remain representable; a
saturated endpoint sum is not itself grounds for rejecting the mesh.

Collider broad-phase boxes are analytical conservative intersections with the
representable coordinate domain. Clipping applies only to final AABB endpoints;
it is never reused as narrow-phase geometry. Cuboids retain
`FixedOrientedBox(center, orientation, halfExtents)`, while capsules, cylinders,
and cones retain center, normalized rigid-frame rotation, normalized local
axis, full axis length, and radius. Their public `WorldAxis` values are derived
convenience projections for callers and diagnostics; exact support and contact
construction stays in the authoritative rigid frame.

Direct `PhysicsMesh` transform APIs retain this strict validation contract.
Compound-part authoring reaches the same mesh path only after
`CompoundColliderPart` has normalized its stored local orientation.

Source vertices, triangle indices, authored face normals, convex SAT edge
topology, and support topology are immutable and exposed only through read-only
views or internal query seams. Scale changes rebuild the scale-dependent local
triangle BVH, bounds, support ordering, and face normal/area caches in prepared
buffers before swapping them into committed state. World normals use the rigid
rotation of the inverse-transpose-scaled local normal and are normalized before
projection. Frontal area is the positive projected physical triangle area and
is invariant to direction magnitude.

Closed-volume mass properties transform analytic covariance under diagonal
scale. `SurfaceApproximation` performs stable two-pass thin-shell integration:
area-weighted COM relative to scaled bounds, then authored-order triangle
central tensors plus parallel-axis shifts. Checked fixed-point arithmetic
rejects volume collapse, saturation, and nonrepresentable COM/tensors rather
than publishing sentinel values.

True hierarchy shear is outside this contract. Hosts that compose rotated
nonuniform parent scales into shear must bake that affine deformation into
authored mesh vertices or provide a future explicit affine-mesh API; Gravitas
does not silently approximate shear as diagonal scale plus rotation.

Concave mesh-as-source sweeping and automatic runtime decomposition are not part
of the public runtime query surface. Author concave movers as offline decomposed
`LSCompoundCollider` assets with stable convex part order.

## Mixed Shape Policy

Mixed collision embeds 2D colliders into 3D as finite slabs/prisms.
`CollisionDetectionMixed` supports 3D primitive, compound, and mesh colliders
against embedded 2D circle, capsule, AABB, convex polygon, and compound slabs.

Compound mixed contacts scan owned parts in stable order and return one external
contact surface on either side. Mesh mixed contacts gather local-BVH triangle
candidates and test triangles against the embedded 2D slab volume.

Mixed shape-pair support belongs to the explicit mixed service. 2D and 3D
collision dispatch should not fall through to accidental mixed response.

## Source Map

| Area                   | Source                                                                                                   |
| ---------------------- | -------------------------------------------------------------------------------------------------------- |
| 3D colliders           | [`src/Gravitas/Colliders/3D`](../../src/Gravitas/Colliders/3D)                                           |
| 2D colliders           | [`src/Gravitas/Colliders/2D`](../../src/Gravitas/Colliders/2D)                                           |
| Shape definitions      | [`src/Gravitas/Colliders/Definitions`](../../src/Gravitas/Colliders/Definitions)                         |
| Compound parts         | [`src/Gravitas/Colliders/Compound`](../../src/Gravitas/Colliders/Compound)                               |
| Collider state helpers | [`src/Gravitas/Colliders/State`](../../src/Gravitas/Colliders/State)                                     |
| 3D detection           | [`src/Gravitas/CollisionHandling/Detection/3D`](../../src/Gravitas/CollisionHandling/Detection/3D)       |
| 2D detection           | [`src/Gravitas/CollisionHandling/Detection/2D`](../../src/Gravitas/CollisionHandling/Detection/2D)       |
| Mixed detection        | [`src/Gravitas/CollisionHandling/Detection/Mixed`](../../src/Gravitas/CollisionHandling/Detection/Mixed) |
| Mesh support           | [`src/Gravitas/Colliders/Mesh`](../../src/Gravitas/Colliders/Mesh)                                       |
