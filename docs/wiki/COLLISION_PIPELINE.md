# Collision Pipeline

Gravitas collision work is split into GridForge-backed broad phase,
context-local pair management, shape-pair narrow phase, deterministic contact
manifolds, manifold response, and late contact notification.

## 2D And 3D Boundary

The original collision handling path is the 3D runtime path. `StiffBody` and
`LSCollider` route through `GravitasPhysicsService`; `StiffBody2D` and
`LSCollider2D` route through `GravitasPhysics2DService`. The active path is
selected by `PhysicsSettings.RuntimeMode`, so pure 2D scenes do not advance 3D
pair distribution or visualization work.

Pure 2D uses X/Z host projection: world `Vector3d.x` maps to `Vector2d.x` and
world `Vector3d.z` maps to `Vector2d.y`. World `Vector3d.y` is height or future
mixed-dimension embedding metadata, not a pure 2D collision axis.

## Pure 2D Collision Path

`GravitasPhysics2DService` owns the alpha pure 2D path for `StiffBody2D` and
`LSCollider2D`. It keeps 2D collider IDs, 2D body registration, reusable pair
state, visualization publishing, and caller-buffered overlap/raycast query
output local to one
`GravitasWorldContext`.

The current 2D broad phase is GridForge-backed:

1. rebuild a 2D collider's `FixedBoundArea` when body motion, kinematic host
   motion, explicit bodyless collider refresh, or shape input edits change it.
2. project the collider's X/Z bounds into private GridForge storage on the Y=0
   plane.
3. scan covered GridForge spatial cells and voxels.
4. attach or reuse a `PhysicsPartition2D` payload on each covered voxel.
5. store collider IDs in static, dynamic, and awake-dynamic sparse sets.
6. distribute candidate pairs from active partitions in deterministic
   voxel/order and collider-ID order.
7. suppress duplicate pair work when broad colliders share several voxels by
   routing each pair through its deterministic first shared partition before
   the frame duplicate-pair set.
8. run same-agent, explicit hierarchy, layer, and bounds filtering before exact
   2D narrow-phase dispatch.

The Y=0 storage plane is not physical thickness and does not claim mixed
2D/3D interaction. It is a deterministic broad-phase identity that lets pure
2D and 3D use the same host-owned `GridWorld` model. `PhysicsRuntimeMode.Both`
keeps those paths side by side without cross-dimensional contacts; only
`PhysicsRuntimeMode.Mixed` enables the mixed lifecycle path. Mixed broad phase
now uses `PhysicsMixedPartition` payloads attached to GridForge voxels and emits
stable 3D/2D candidate keys after awake-dynamic, layer, same-agent, explicit
hierarchy, duplicate, and bounds filtering. The mixed embedding state on
`LSCollider2D` is a finite 3D `FixedBoundBox` built from pure 2D X/Z bounds plus a
positive Y half-thickness centered on the host transform's Y position.
`CollisionDetectionMixed` currently supports 3D sphere, cuboid, capsule, finite
cylinder, compound, and mesh contacts against embedded 2D circle, AABB, convex
polygon, and compound slabs. Compound mixed contacts scan owned parts in stable
order and return one external contact surface on either side. Mesh mixed
contacts gather local-BVH triangle candidates and test triangles against the
embedded 2D slab volume.
`CollisionPairMixed` owns stable 3D/2D pair identity, resting-pair retention,
wake propagation, mixed contact enter/stay/exit events, trigger-only mixed
trigger events, and pooled pair reuse. `CollisionResponseMixed` applies the
constrained mixed response: planar X/Z correction and impulses can move a
2D body, planar normal and friction impulse components can spin it around its
scalar yaw axis from the 2D COM-relative contact arm, and vertical Y correction
and impulses treat the 2D body as having infinite constrained mass. Mixed
diagnostics, explicit mixed queries, and mixed CCD hooks are implemented. 2D
swept-circle mixed CCD uses the shared swept-sphere worker, including 3D mesh
targets through local-BVH triangle candidate TOI checks and compound targets
through stable part-order reduction.

`CollisionDetection2D` currently supports:

- circle/circle.
- circle/axis-aligned box.
- axis-aligned box/axis-aligned box.
- circle/convex polygon.
- axis-aligned box/convex polygon.
- convex polygon/convex polygon.
- compound/primitive or compound/compound, resolved by scanning owned parts in
  stable declaration order and returning the owner collider identity.

Circles use center/radius tests and support points. Boxes and polygons use 2D
separating-axis tests over deterministic vertex order. `LSPolygonCollider2D`
rejects concave and collinear input up front; concave 2D decomposition is not
claimed yet. `CollisionPair2D` resolves collider priority up front and
`CollisionDetection2D` dispatches through `CollisionType2D`, so adding a new
2D shape pair should extend the settings/type table instead of growing public
type-check conditionals.

`CollisionPair2D` owns pure 2D pair lifecycle: stable collider priority,
wake propagation from awake movable bodies, trigger enter/exit events, and
contact enter/stay/exit events. Solid response is delegated to
`CollisionResponse2D`, which applies deterministic one-pass positional
correction to penetration slop, normal impulse when bodies are closing, and
tangent Coulomb friction impulse after the normal solve. It reads
`StiffBody2D.EffectiveInverseMass`,
`StiffBody2D.EffectiveInverseMomentOfInertia`, and
`StiffBody2D.WorldCenterOfMass` so immovable, kinematic, inactive,
non-positive-mass, and angular-disabled bodies remain infinite mass/inertia to
the solver while raw mass and scalar moment values stay inspectable. If a solid
pair has no awake movable participant, the existing pair is kept alive as
resting state without applying response or waking a sleeping body. This is still
a single-contact alpha solver; richer contact manifolds and warm-started pure
2D response remain future hardening work.

## Broad Phase: Voxel Partitions

When a collider initializes, moves, rotates, changes scale, or changes local
shape inputs, `LSCollider` rebuilds its runtime shape data and asks
`GravitasCollisionService` to repartition it. Shape inputs are tracked by an
internal snapshot so several local edits made before a simulation call collapse
into one bounds/shape rebuild.

`GravitasCollisionService.PartitionObject(...)`:

1. validates that the collider belongs to the service context.
2. asks GridForge `GridTracer.GetCoveredVoxels(...)` for topology-aware voxel coverage.
3. uses each covered grid's topology metrics as conservative voxel-position padding.
4. suppresses duplicate voxel visits with context-local sets.
5. checks that the voxel position falls within the collider bounds.
6. rents or reuses a `PhysicsPartition` on the voxel.
7. stores the collider's `WorldVoxelIndex`.
8. adds the collider ID to the partition dynamic or static membership set.

When a collider leaves a voxel, Gravitas removes the collider ID from the
partition but keeps the empty `PhysicsPartition` attached to that voxel. Empty
partitions are inactive and query-invisible, while future movement back into
the same voxel can reuse the existing partition without going through
GridForge's partition add path again. Empty retained partitions are not kept
forever: `PhysicsSettings.RetainedPartitionTimeToKillFrames` controls the
deterministic frame window, and
`PhysicsSettings.RetainedPartitionRetirementSweepBudget` bounds how many
retained partitions the collision service checks per distribution step. Expired
empty partitions are removed from their voxels and returned to the
context-local partition pool. `GravitasWorldContext.Reset()` is a stronger
session boundary: it detaches retained Gravitas partition payloads from
GridForge voxels, clears retained tracking, and clears partition pools before
collider IDs are reused.

`PhysicsPartition` is a GridForge voxel partition payload. It stores
context-local collider IDs, not collider references. Dynamic and static
membership uses `SwiftSparseSet`: lookups and removals are keyed by collider ID,
while dense-key storage keeps partition iteration compact. Do not treat dense-key
order as a semantic ordering rule; deterministic pair ordering must be explicit
at the pair or service layer. The owner service is required before the partition
is added to a voxel. The partition returns to that owner service pool when
GridForge removes the voxel partition itself, either through retained-partition
retirement or world/grid cleanup.

Current mobility membership is explicit: bodyless colliders and colliders whose
body has `Immovable == true` are added to `ContainedStaticObjects`; bodies with
`IsKinematic == true` are added to `ContainedKinematicObjects`; movable
non-kinematic bodies are added to `ContainedDynamicObjects`. Dynamic partitions
also keep `ContainedAwakeDynamicObjects`, a second sparse set for dynamic
collider IDs whose bodies are currently awake for collision work. Only dynamic
membership activates solver partition work. Sleeping bodies stay in normal
dynamic membership so queries, wake propagation, pair cleanup, and contact
lifecycle can still find them.

`PhysicsPartition2D` mirrors the same ID-first lessons for pure 2D. Bodyless
2D colliders and immovable 2D bodies are static members, kinematic 2D bodies
are kinematic members, and movable 2D bodies are dynamic members. Only awake
dynamic IDs activate pair distribution. Sleeping 2D bodies remain query-visible
in dynamic membership, but partitions with no awake dynamic IDs skip solver
work. Empty 2D partitions are retained, retired by the same deterministic TTK
settings, and returned to the 2D collision service's partition pool through
GridForge voxel removal.

## Collider Runtime Shape State

`LSCollider` and `LSCollider2D` separate collider identity and host binding from
the dense mutable state used by shape rebuilds, partition ownership, query
duplicate suppression, pair cleanup, and hierarchy filtering. The 3D
runtime-shape snapshot watches:

- world-space center.
- rotation.
- local scale.
- local offset.
- unscaled local size.
- unscaled local radius.

Mutating `LocalOffset`, `Radius`, or `Size` marks the runtime shape dirty.
Changing host/body scale, position, or rotation is detected from the snapshot on
the next `Simulate()` call. If the snapshot has not changed, the collider skips
the rebuild and keeps its existing partition state.
Shape mutation wakes a sleeping bound body before the broad phase refreshes, so
changed bounds cannot remain hidden behind a sleeping partition entry.

Pure 2D colliders use the same state-helper pattern where the payload is
dimension-free. `ColliderQueryState` is shared directly. Runtime-shape dirtying
reuses the same version/commit helper with a 2D snapshot payload, while pair
state uses the same generic helper over 2D collision pairs. Hierarchy state is
shared across 2D and 3D through dimension-tagged `ColliderHierarchyKey` values,
so cross-dimensional parent/child and sibling filters do not alias plain
collider IDs. Only partition coordinates remain 2D-specific because they store
X/Z planar coverage. A standalone 2D collider whose center, rotation, local
offset, or shape version has not changed skips `FixedBoundArea` rebuilds and
partition refreshes. A 2D compound part also includes inherited local scale in
its private runtime-shape snapshot so authored part scale changes rebuild the
owning aggregate without adding host-scale lookup cost to ordinary standalone
2D colliders.

Partition state tracks grid coordinates, previous broad-phase coverage bounds,
last mobility kind, partition-change flags, and broad-phase versioning together.
Mobility changes such as dynamic -> kinematic -> immovable refresh partition
state even when bounds do not change, and clears remove collider IDs from the
bucket they were previously inserted into. Query state tracks the raycast and
circle-query versions used by context-owned query services to suppress duplicate
collider hits. Pair state owns the one-sided collision-pair dictionary and the
opposite-side holder set; both are allocated lazily so colliders that never form
pairs do not pay for pair containers up front. Broad-phase versioning advances
from committed runtime-shape changes, so collision pairs do not need separate
collider position/rotation dirty flags. If the runtime snapshot changes,
bounds/partition state refreshes and pairs observe the broad-phase version
change.

Hierarchy state is explicit. Hosts call `child.SetParent(parent)` after collider
initialization when two colliders belong to the same engine object or aggregate
body and should not collide with each other. Gravitas stores the top parent as a
dimension-tagged collider key for filtering; it does not walk host transform
trees at simulation time. When a parent collider deactivates, its child bindings
are cleared before the parent collider ID returns to the reusable ID pool,
preventing stale hierarchy keys from suppressing collisions against future
unrelated colliders. Mixed 2D/3D hierarchy filtering uses the same state; body
policy inheritance remains dimension-local until mixed CCD or future island work
defines a stronger cross-dimensional body contract.

`LSCompoundCollider` is different from hierarchy binding. A parent/child
relationship links independently registered colliders that may represent
separate host objects. A compound collider owns internal geometry parts under
one collider ID, one body binding, one broad-phase entry, and one contact/event
surface. Compound parts are not registered with `GravitasPhysicsService`, cannot
be parented independently, and are scanned in stable part order by the owning
compound collider. Public `CompoundColliderPart` values are data-first authored
descriptors: each part stores a `ColliderShapeDefinition`, local offset, local
rotation, and local scale. `LSCompoundCollider` may materialize private runtime
part colliders internally to reuse the existing narrow phase, query, diagnostics,
and inertia code, but those internal colliders are not the authored asset format
and are not independent runtime identities.

`LSCompoundCollider2D` applies the same authored-data rule to pure 2D. Public
`CompoundColliderPart2D` values store `ColliderShapeDefinition2D`, local offset,
local rotation, and local scale. The owner materializes private `LSCollider2D`
part colliders in stable declaration order, keeps one 2D collider ID, one body
binding, one broad-phase entry, and one event/query/diagnostic identity, and
rejects lifecycle operations on the private parts.

Capsules rebuild their hemisphere centers, cylinder height, area, and segment
endpoints together. Short capsules collapse to a sphere-like segment and use a
sphere inertia fallback instead of producing a zero diagonal for the capsule's
main axis.

Cylinders rebuild their finite flat-capped axis segment, cap centers, height,
area, and inertia inputs together. Cylinder support intentionally treats the
shape as a real finite cylinder rather than a capsule with ignored hemispheres,
so cap separation and side separation are tested independently.

Cuboids keep face, edge, normal, and centroid arrays on the collider. Rotated
closest-surface queries walk the cached face index data directly rather than
materializing temporary face vertex arrays, because those queries are used by
SAT contact seeding.

`FixedTransform.LossyScale` uses basis-vector scale extraction rather than raw
matrix diagonals. This matters for rotated colliders because diagonal extraction
can report near-zero scale for 90-degree rotations and collapse derived shape
state.

Mesh colliders validate vertices and triangle indices at construction time.
`MeshColliderMode` declares whether the mesh is intended as `Convex` or
`Concave`. Convex meshes may use whole-shape convex assumptions where valid.
Concave meshes are explicit triangle collision data and are legal for
bodyless, immovable, kinematic, and dynamic bodies.

`PhysicsMesh` now owns source vertices, triangle normals, triangle areas, local
bounds, and the triangle BVH in local mesh space. Rigid movement updates the
mesh transform, inverse transform, and conservative world bounds without
rebuilding the local BVH or allocating new bounds after warmup. Mesh queries and
narrow-phase callers transform their world-space query bounds or points into
local space, query the local BVH, then transform final contact points and
normals back to world space. The full world-vertex array is retained only as an
on-demand compatibility view.

Alpha mesh policy is explicit rather than Unity-compatible by default: concave
meshes collide as triangle sets instead of being treated as one convex hull.
The concave narrow phase gathers local-BVH triangle candidates, runs
triangle-vs-shape or triangle-vs-triangle checks, and reduces contacts through
the pair-owned `ContactManifold`. Dynamic concave meshes keep topology and the
local BVH stable while rigid movement updates transform-derived state only.
For mesh-mesh pairs involving a concave mesh, alpha keeps the raw local-BVH
triangle-gather path rather than a direct BVH-vs-BVH traversal: Phase 4B
benchmarks found paired traversal slower with the current conservative bounds
transforms, while a narrower triangle SAT axis optimization improved the simple
concave mesh-mesh row without changing candidate truth or steady-state
allocation behavior.

Mesh policy work should keep these boundaries explicit:

- Concave triangle meshes are supported for static, kinematic, immovable, and
  dynamic bodies, but they should be chosen deliberately because candidate
  count scales with local triangle density. Simple authored physics meshes are
  the intended raw-triangle use case; dense rendered meshes should be
  simplified, decomposed, or represented as authored convex collision assets.
- Dynamic mesh bodies that can rotate require closed-volume inertia by default.
  `MeshInertiaPolicy.RequireClosedVolume` validates one connected, consistently
  wound triangle shell where every undirected edge has exactly two incident
  triangles. Boundary, duplicate-triangle, non-manifold, inconsistent-winding,
  disconnected-shell, and zero-volume meshes are rejected with explicit
  `MeshVolumeValidationResult` values. Reversed whole-mesh winding is accepted
  deterministically.
- Open or surface-only dynamic meshes must opt in with
  `MeshInertiaPolicy.SurfaceApproximation` when angular dynamics are enabled.
  Bodyless, static, immovable, kinematic, and explicitly angular-force-disabled
  mesh bodies do not consume mesh inertia and remain legal collision surfaces.
- Closed-volume mesh inertia is integrated with fixed-point signed tetrahedra
  and cached on the immutable mesh topology. `MeshMassProperties.CenterOfMass`
  is the homogeneous COM, and `MeshMassProperties.UnitMassInertiaTensor`
  preserves products of inertia about the cached reference point. Mesh inertia
  shifts between the reference point, COM, and requested body-local point with
  the full parallel-axis tensor. Runtime principal-axis diagonalization is not
  part of the current solver; if needed, it should land in FixedMathSharp or an
  offline/tooling path with deterministic tie rules and benchmark evidence.
- Convex mesh paths remain free to use whole-shape convex tests where valid.
- Compound colliders present one collider identity to hosts and one body to the
  solver, while internally ordering primitive or convex-mesh parts by stable
  part index. They aggregate part bounds, approximate mass/inertia from the
  parts, emit one event surface, and draw part geometry through the owning
  collider ID. Public authored parts use `ColliderShapeDefinition` data rather
  than pre-instantiated child `LSCollider` objects. Concave mesh parts are not a
  compound authoring surface; concave behavior belongs to `LSMeshCollider`.
  Authored/offline decomposed collision assets should use `LSCompoundCollider`
  for alpha unless a future asset pipeline proves that mesh-owned pieces need
  different public semantics.
- Host/offline convex decomposition should feed explicit primitive or convex
  mesh `ColliderShapeDefinition` parts into the owning compound collider without
  changing the owning collider identity. Runtime automatic decomposition remains
  out of scope.
  Automatic convex decomposition is not claimed unless the chosen algorithm is
  deterministic, bounded, tested on pathological input, and benchmarked. Ear
  clipping is a 2D polygon triangulation/partitioning tool, not a complete 3D
  convex decomposition strategy.
- Mesh simplification and collision LOD should be host/offline data for alpha.
  Runtime simplification must not alter authoritative collision geometry during
  a simulation frame.
- The old Unity Mesh Simplifier package is useful as reference material for
  quadric-error simplification, smart vertex linking, and preservation options,
  but should not be copied into the runtime without a fixed-point deterministic
  porting plan.
- Rigid dynamic meshes should keep local topology and BVH stable while updating
  only transform-derived state. Deformable or breakable topology changes require
  a separate invalidation/rebuild contract before support is claimed.
- `PhysicsMesh.CalculateInertiaTensor(...)` defaults to closed-volume mass
  properties. Callers that knowingly want the legacy surface-area approximation
  must pass `MeshInertiaPolicy.SurfaceApproximation`.
- `PhysicsMesh` does not inspect body mobility, kinematic state, or angular
  force policy. `StiffBody` decides whether angular inertia is needed before it
  asks the collider/mesh for geometry-derived mass properties.

## Continuous Collision Detection

CCD is body-owned and opt-in. `PhysicsSettings.DefaultContinuousCollisionMode`
defaults to `Discrete`; `StiffBody.ContinuousCollisionMode` and
`StiffBody2D.ContinuousCollisionMode` default to `Inherit`, so existing bodies
keep the discrete integration path unless the host sets a context default,
enables a body explicitly, or assigns a top-level parent body with an explicit
CCD mode. `ColliderHierarchyState` caches the top parent when parent
relationships are bound, so `Inherit` can check the parent policy in constant
time before falling back to the context default.

The alpha CCD path runs during body position integration, after velocity and
acceleration have produced an intended frame displacement and before the
authoritative position is committed. It uses a conservative swept proxy derived
from the moving collider:

- 3D sphere uses its exact scaled radius.
- 3D capsule, cuboid, finite cylinder, mesh, and compound movers use the
  world-bounds sphere radius (`Bounds.Scope.Magnitude`). This is intentionally
  conservative for elongated or sparse shapes: it can stop early, but it avoids
  the false-negative tunneling risk of using the smallest bounds axis while the
  shape's wider portion passes through a target away from the center path.
- 2D circle uses its scaled radius.
- 2D AABB and convex polygon use a conservative bounds radius.
- 2D compound uses a conservative aggregate radius over its private parts.

`Continuous` always sweeps when the proxy radius and displacement are non-zero.
`Auto` sweeps only when the intended displacement is larger than the proxy
radius. When a hit is accepted, the body clamps to the earliest swept center
time of impact and removes only the closing component of linear velocity,
preserving tangential velocity for later discrete response work.

Static and kinematic CCD targets are non-trigger bodyless colliders, immovable
bodies, and kinematic bodies whose layers are allowed by the context collision
matrix and whose hierarchy is not excluded. Static or kinematic mesh and
compound targets are covered by the query workers, so 3D swept-sphere CCD keeps
triangle and stable part-order target behavior. Mesh sweep normals are oriented
against the sweep direction when authored triangle winding would otherwise point
with the moving source, so closing velocity removal is two-sided and
deterministic. Pure 2D and 3D CCD use internal static-style query collectors for
this leg: public sweep queries still report movable dynamic, kinematic,
immovable, and bodyless targets, while CCD's static leg copies only
kinematic/static partition IDs and skips movable dynamics because the
relative-motion path below owns those candidates.

Dynamic-vs-dynamic CCD uses a frame-start relative-motion model. During
`LateSimulate`, the physics services cache each movable body's start position
and predicted displacement under a context-local late-simulation token before
any individual body commits movement. A moving source compares the
static/kinematic query hit with dynamic relative TOI candidates and chooses the
earliest distance, then higher closing speed, then stable collider ID order.
This prevents opposing fast bodies from depending on dynamic body iteration
order.

The dynamic path is intentionally conservative:

- 3D dynamic targets are represented by continuous sphere proxies. Sphere
  targets use their scaled radius; other 3D target shapes use the same
  conservative bounds radius as moving sources.
- pure 2D dynamic targets are represented by continuous circle proxies.
- mixed 2D slabs use the larger of planar radius and mixed half-thickness as
  their 3D proxy radius.
- dynamic mesh and compound bodies are supported as moving proxy bodies rather
  than exact swept mesh or exact swept compound sources.

Exact moving mesh/compound CCD would require a deeper shape-specific solver and
benchmark evidence before it should replace the conservative proxy path.

## Active Partitions

A partition becomes active when its dynamic membership transitions from empty to
non-empty. Active partitions are stored in
`GravitasCollisionService._activePartitions`.

During `context.Simulate()`, `GravitasPhysicsService.Simulate()` first lets
registered dynamic-body colliders refresh bounds and partition membership. This
pre-distribution pass catches host command teleports or direct body moves made
between frames. It then calls
`GravitasCollisionService.CheckAndDistributeCollisions()`. The collision service
increments its `Version`, copies active partitions into a reusable buffer, sorts
them by `WorldVoxelIndex`, and asks each active partition to distribute
candidate pairs.

`PhysicsPartition.Distribute()` checks:

- every awake dynamic against the other dynamic IDs in that partition.
- every awake dynamic against the static IDs in that partition.

The dynamic, awake-dynamic, and static sparse-set keys are copied into
context-owned buffers and sorted by collider ID before pair generation. This
keeps pair/contact ordering stable even when movement churn changes sparse-set
dense storage order. `SwiftSortedList` is not used for these scratch buffers:
its `AddRange` path still copies source items into a temporary array and then
merges sorted data, while the current reusable `SwiftList` buffers bulk-copy
and sort without adding another persistent membership structure.

If a partition contains no awake dynamic IDs, distribution returns before pair
generation. Static/static pairs are not distributed. This is the current alpha
sleep optimization: it is partition-local and flat, not a recursive island or
tree-propagation system.

## Pair Creation And Filtering

For each candidate ID pair, the partition asks
`owner.Context.Physics.GetCollisionPair(id1, id2)`.

The physics service resolves both IDs through the owning context and filters
with `RequireCollisionPair(...)`. A pair is required only when:

- both colliders are active.
- both colliders have real shapes.
- at least one collider has a body.
- the context collision matrix allows the two layers to collide.
- the colliders are not explicitly bound as parent-child or siblings in the
  host hierarchy.

If the pair already exists, it is reused. Otherwise, the service rents or
creates a `CollisionPair`, stores it on the lower-ordered collider, and stores a
holder entry on the other collider. This gives one owning side for cleanup while
still allowing either collider to remove its relationship during deactivation.

Inside the pair, `ColliderA` and `ColliderB` are ordered for narrow-phase
dispatch and contact data, not for ownership. Shape priority wins first because
some detection paths expect the higher-priority shape in `ColliderA`. When
shape priority ties and both colliders have bodies, the higher linear speed wins
so same-shape dynamic pairs produce contact normals from the more active body
toward the other body. Equal priority and equal speed keep the original
candidate order as the stable tie-breaker.

## Duplicate Suppression

The same two colliders can share several voxels. `CollisionPair.PartitionVersion`
prevents duplicate narrow-phase work in the same distribution pass.

Each `CheckAndDistributeCollisions()` call increments the collision service
`Version`. A partition processes a pair only if the pair's partition version is
not equal to the current service version, then stamps the pair with that version.

## Pair Update And Culling

`CollisionPair.UpdateCollision()` performs pair-local checks:

1. reject inactive pairs or inactive colliders.
2. queue the pair for active-pair maintenance if this is the first update after
   activation.
3. invalidate culling if either collider changed partition state or
   broad-phase version since the last pair check.
4. either run collision work immediately or decrement the cull counter.

Fast rejection happens before narrow phase:

- squared distance between collider centers is checked against combined bounds
  scope.
- collider AABB bounds must inclusively overlap so zero-depth touching contacts
  can reach narrow phase.

If a pair was colliding on the previous check, it can reuse that state only
while both colliders keep the same broad-phase version. Shape or bounds changes
must re-run narrow phase even when object transforms did not move.

If the pair is not colliding, `CalculateCullScore()` combines distance,
relative velocity, and time-since-last-collision into a frame countdown.
Distance and age increase the delay; relative velocity reduces the delay so
fast-moving pairs are checked more conservatively. Disabled or zero culling
thresholds disable that score contribution rather than dividing by zero. Large or
explicitly protected colliders can prevent culling.

## Narrow Phase

`CollisionDetection.DoCollisionCheck(pair)` dispatches by
`ColliderSettings.GetCollisionType(...)`.

Current shape support:

| Pair | Detection path |
| --- | --- |
| Sphere/Sphere | center distance against combined radius. |
| Capsule/Sphere | closest capsule surface point to sphere center. |
| Capsule/Capsule | closest points between capsule line segments plus radii, with a deterministic fallback for degenerate capsule segments. |
| Cuboid/Sphere | closest cuboid surface point to sphere center. |
| AABox/Capsule | closest capsule line point to box center, then box surface point. |
| OBBox/Capsule | separating axes from cuboid/capsule support. |
| Cuboid/Cuboid | axis-aligned face/edge/stack manifolds for AABoxes, SAT primary contacts for oriented boxes. |
| Cylinder/Sphere | finite cylinder closest surface against sphere radius. |
| Cylinder/Capsule | finite-cylinder projection axes against capsule segment/radius projection. |
| Cylinder/Cylinder | finite-cylinder projection axes, preserving flat cap separation. |
| Cuboid/Cylinder | cuboid vertex projection against finite-cylinder projection. |
| Mesh/Sphere | convex mesh uses closest surface point; concave mesh gathers triangle candidates against the sphere bounds. |
| Mesh/Capsule | convex mesh uses closest surface from the capsule line seed; concave mesh uses segment-vs-triangle closest points. |
| Mesh/Cuboid | convex mesh uses nearby-triangle SAT; concave mesh runs per-triangle SAT against the cuboid. |
| Mesh/Cylinder | triangle-BVH candidate scan against finite cylinder volume; concave mode writes triangle contacts. |
| Mesh/Mesh | convex mesh uses nearby-triangle SAT; concave-involved pairs run triangle-vs-triangle candidate checks. |
| Compound/* | stable part-order scan, existing part-vs-shape narrow phase, and pair-owned manifold reduction. |

Current shape-pair matrix:

| A / B | Sphere | Capsule | Cuboid | Cylinder | Mesh | Compound |
| --- | --- | --- | --- | --- | --- | --- |
| Sphere | Supported | Supported | Supported | Supported | Supported | Supported |
| Capsule | Supported | Supported | Supported | Supported | Supported | Supported |
| Cuboid | Supported | Supported | Supported | Supported | Supported | Supported |
| Cylinder | Supported | Supported | Supported | Supported | Supported | Supported |
| Mesh | Supported | Supported | Supported | Supported | Supported | Supported |
| Compound | Supported | Supported | Supported | Supported | Supported | Supported |

`Cuboid` covers both `AABox` and `OBBox` dispatch. `Cylinder/Mesh` is
normalized to `Mesh/Cylinder` by pair priority so contact data is written in the
mesh-to-cylinder direction.

SAT and mesh candidate paths use context-owned scratch state through
`GravitasWorldContext`. `CollisionSatScratch` owns the reusable
`CollisionContext`, cuboid object-info buffers, mesh object-info buffers,
mesh/cylinder triangle buffer, concave triangle candidate buffers, and SAT axis
sets for one world context. This is
intentionally not static: concurrent worlds keep isolated scratch, while repeated
checks in the same world avoid per-check object-info construction and pool
rent/release churn. Short `collision-detection` benchmark smoke currently
reports on aggregate primitive checks, single-contact primitive manifold
generation, axis-aligned cuboid face-manifold generation, cuboid/cuboid SAT,
mesh/cylinder, mesh/cuboid, mesh/mesh, compound/primitive, and concave mesh paths
after warmup.

## Contact Data

The narrow phase writes a `ContactManifold` owned by the `CollisionPair`.
Manifold contacts are value types (`ManifoldContact`) with:

- stable contact identity derived from the unordered pair of world-space contact
  points.
- point on collider A.
- point on collider B.
- penetration depth.
- normal oriented from collider A toward collider B.
- optional immovable collision direction.

Manifolds currently store up to four contacts. When more candidates are offered,
the manifold keeps the deepest four and breaks depth ties by lower stable contact
identity. Exposed contact order is stable ascending contact identity.
`PrimaryContact` remains a convenience for diagnostics, tests, and callers that
need one representative contact; the response solver iterates the full
manifold. Duplicate contact identities update only when the new candidate is
deeper.

Contact data stores the narrow phase's detected depth without adding a solver
margin. Touching contacts can therefore have zero depth, and any stabilization
margin belongs to the response solver rather than hidden inside contact data.
`ContactManifold.HasContact` distinguishes unset contact data from legitimate
zero-valued fields, such as touching contacts with zero depth or contact points
at the origin. The response solver ignores pairs whose manifold has not been
written by narrow phase.

Axis-aligned cuboid/cuboid detection now generates up to four contacts for
face-overlap and stacked/touching faces. Edge contact reduction naturally drops
duplicate corners and can produce two contacts; corner contact can produce one.
Sphere, capsule, cylinder, and oriented cuboid SAT paths currently write a
single manifold contact. Axis-aligned cuboids and concave mesh paths can write
multiple contacts, capped by the manifold's deterministic four-contact
reduction.

## Response

`CollisionPair.ProcessCollision()` calls `CollisionResponse.CalculateImpulse(...)`
when detection reports a collision and the pair should perform physics response.
Pairs with either collider marked as a trigger skip physical response; they can
still flow through contact notification.

When an awake dynamic body collides with a sleeping dynamic body, the pair wakes
the sleeping body before response. If every dynamic body in the partition is
sleeping, pair generation is skipped until a deterministic wake reason changes
one of those bodies or its shape state.

Current non-trigger response behavior is a deterministic fixed-capacity
manifold solver:

1. build up to four explicit solver contacts from the pair manifold, collider
   bodies, contact points, relative contact arms, detected depth, and normals
   oriented from collider A to collider B.
2. treat `Immovable` and `IsKinematic` bodies as infinite mass for response.
3. apply immediate positional correction only for depth above
   `CollisionResponse.PenetrationSlop`; the correction is distributed by
   inverse mass, scaled by `PenetrationCorrectionPercent`, and divided across
   the active manifold contacts so a four-contact face does not correct four
   times as far as the detected penetration.
4. compute normal contact velocity from linear velocity plus angular velocity at
   each relative contact arm.
5. compute normal impulse scalars for all contacts before applying them. This
   keeps symmetric face manifolds from injecting spin through whichever corner
   happens to be visited first.
6. skip normal impulse when the bodies are already separating along the contact
   normal.
7. apply direct normal velocity deltas to movable bodies, plus angular velocity
   deltas when angular forces are enabled.
8. compute tangential contact velocity after normal impulses, then apply a
   Coulomb friction impulse along the tangent. The tangent impulse is clamped to
   `normalImpulse * frictionCoefficient`, where the pair coefficient is the
   geometric mean of the two body coefficients.
9. store the solved normal and tangent impulse scalars in a fixed-size
   pair-local warm-start cache keyed by stable manifold contact identity.

When diagnostics are enabled, the pair emits contact and response events in the
same deterministic order as collision processing: `Contact`, one
`ResponseImpulse` event for each applied normal impulse, then body velocity-delta
events produced by normal and friction response. The diagnostics stream is
observational only; it does not change pair ordering, contact data, or response
behavior.

Response units and invariants:

- mass is body mass in the same unit model used by `StiffBody`.
- `StiffBody.InverseMass` is the raw reciprocal of body mass. Collision
  response reads `StiffBody.EffectiveInverseMass`, which maps immovable,
  kinematic, inactive, and non-positive-mass bodies to zero solver mass while
  leaving raw mass values inspectable.
- `StiffBody.CanTranslate`, `StiffBody.CanRotate`, and
  `StiffBody.EffectiveInverseInertiaTensor` are the 3D response mobility
  contract used by both ordinary 3D response and mixed 2D/3D response.
- `StiffBody2D.CanTranslate`, `StiffBody2D.CanRotate`,
  `StiffBody2D.EffectiveInverseMass`, and
  `StiffBody2D.EffectiveInverseMomentOfInertia` are the pure 2D body-side
  mobility contract. Pure 2D body integration consumes scalar moment for
  host-applied torque and angular impulses. Pure 2D contact response consumes
  the same scalar moment surface for COM-relative normal and friction angular
  velocity deltas. Mixed response consumes the same effective mass and scalar
  moment surface for the embedded 2D participant, but only from planar X/Z
  impulse components; vertical Y impulse remains constrained out of the 2D
  body model.
- 3D response torque arms are measured from `StiffBody.WorldCenterOfMass`.
  Collider centers remain collision-geometry references for narrow phase,
  culling, and normal fallback; they are not the implicit body COM.
- linear velocity is world units per second.
- angular velocity is radians per second around each local/world axis.
- pure 2D scalar rotation is radians around the yaw axis, and
  `StiffBody2D.WorldCenterOfMass` rotates the body-local COM offset in the X/Z
  simulation plane.
- inertia tensors are fixed-point `Fixed3x3` values supplied by the collider
  shape for the requested `StiffBody.LocalCenterOfMassOffset` and transformed by
  `StiffBody`. Primitive and aligned tensors keep a diagonal inversion fast
  path; mesh and compound tensors can preserve products of inertia and use full
  deterministic inversion. `StiffBody` keeps local and world-space tensor state
  separate so orientation refreshes are idempotent. Mesh colliders use cached
  closed-volume mass properties by default when angular dynamics are enabled and
  keep explicit surface approximation opt-in for open meshes.
- restitution is clamped to `[0, 1]` and combined by the lower coefficient so a
  low-bounce participant can dampen the pair.
- closing speeds at or below `RestitutionVelocityThreshold` use zero
  restitution to avoid resting-contact bounce.
- `StiffBody.FrictionCoefficient` is a non-negative Coulomb coefficient. Values
  above one are allowed for intentional high-friction materials.
- friction impulses oppose tangential contact motion and are clamped by the
  normal impulse. 3D pair-local warm-start storage records normal and tangent
  impulses by contact identity; applying cached impulses as a true warm-started
  iterative solve remains a later solver hardening step. Pure 2D response does
  not yet warm-start tangent impulses.
- penetration depth is a world distance from narrow phase; response slop is a
  solver invariant, not contact data.
- drag and angular damping remain integration/body behavior; contact friction is
  handled by the response solver.

This is still the first alpha milestone, not a full response engine. Static
friction for resting stacks, dynamic-vs-dynamic CCD, full iterative warm-start
application, explicit island solving, and richer mixed-dimension solver behavior
remain future work.

## Body Sleep And Wake

`StiffBody` owns deterministic sleep state. A dynamic non-kinematic body can
sleep after its linear and angular speeds remain at or below explicit thresholds
for `SleepFrameThreshold` fixed frames. Sleeping clears accumulated force,
impulse, velocity, torque, acceleration, and pending position-correction state,
but does not remove the collider from GridForge partitions.

Current deterministic wake stimuli are:

- explicit host wake through `Wake()`.
- non-zero force.
- non-zero linear impulse.
- non-zero angular impulse or torque.
- collision with an awake body.
- kinematic host motion.
- host transform teleport.
- collider shape mutation.

Waking refreshes the collider's awake membership across its current partitions.
For now, this is a flat voxel-partition optimization. A future explicit island
builder should use the same wake rules, then expand them across connected
contacts when island-wide sleep is introduced.

## Contact Notifications

Collision pairs are queued into the physics service active-pair queue the first
time they update. During `context.LateSimulate()`,
`GravitasPhysicsService.ProcessActiveCollisionPairs()`:

- deactivates pairs that have not collided for the inactive-frame threshold.
- emits ongoing contact notifications when the pair is still active and not
  culled.
- keeps active pairs queued for future maintenance.

Sleeping contact pairs are preserved while their manifold is still known to be
colliding. This prevents a resting sleeping contact from aging out and emitting
a false contact exit simply because its partition skipped pair generation.

`LSCollider.NotifyContact(...)` emits:

- `OnTriggerEnter` and `OnTriggerExit` for trigger colliders.
- `OnContactEnter`, `OnContact`, and `OnContactExit` for body contacts.

## Deactivation Cleanup

`LSCollider.Deactivate()`:

1. clears partition membership through the owning collision service.
2. removes owned collision-pair references.
3. removes holder references from the opposite colliders.
4. clears explicit parent binding.
5. deactivates and pools pairs when pooling is enabled.
6. returns the collider ID to the context-local physics service.
7. marks the collider inactive.

Partition cleanup must flow through the owning service. Do not manually return
the same partition through a second path; that risks double-release and stale
activation state.

## Determinism And Performance Notes

- Collider IDs are context-local; never resolve an ID through another context.
- Pair ordering, partition traversal, culling, and notification timing are
  deterministic behavior and should be tested when changed.
- Avoid LINQ and iterator allocations in collision hot paths.
- Use `SwiftCollections` collections and pools where ownership is clear.
- Add focused tests for every shape-pair change, including separated,
  edge-touching, overlapping, degenerate, and rotated cases.
- Add benchmarks when changing partitioning, pair distribution, narrow phase, or
  response loops.
