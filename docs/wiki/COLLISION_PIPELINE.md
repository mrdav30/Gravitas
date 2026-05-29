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

1. rebuild a 2D collider's `BoundingArea` when body motion, kinematic host
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
8. run same-agent, layer, and bounds filtering before exact 2D narrow-phase
   dispatch.

The Y=0 storage plane is not physical thickness and does not claim mixed
2D/3D interaction. It is a deterministic broad-phase identity that lets pure
2D and 3D use the same host-owned `GridWorld` model until Phase 10 defines a
real embedding and impulse-exchange policy.

`CollisionDetection2D` currently supports:

- circle/circle.
- circle/axis-aligned box.
- axis-aligned box/axis-aligned box.
- circle/convex polygon.
- axis-aligned box/convex polygon.
- convex polygon/convex polygon.

Circles use center/radius tests and support points. Boxes and polygons use 2D
separating-axis tests over deterministic vertex order. `LSPolygonCollider2D`
rejects concave and collinear input up front; concave 2D decomposition is not
claimed yet.

`CollisionPair2D` applies simple deterministic one-pass response for the alpha
slice: positional correction to penetration slop, normal impulse when bodies are
closing, static response against bodyless colliders, wake propagation from awake
movable bodies, trigger enter/exit events, and contact enter/stay/exit events.
If a solid pair has no awake movable participant, the existing pair is kept
alive as resting state without applying response or waking a sleeping body. It
does not yet claim a full 2D friction solver, angular impulses, richer contact
manifolds, or mixed 2D/3D impulse exchange.

## Broad Phase: Voxel Partitions

When a collider initializes, moves, rotates, changes scale, or changes local
shape inputs, `LSCollider` rebuilds its runtime shape data and asks
`GravitasCollisionService` to repartition it. Shape inputs are tracked by an
internal snapshot so several local edits made before a simulation call collapse
into one bounds/shape rebuild.

`GravitasCollisionService.PartitionObject(...)`:

1. validates that the collider belongs to the service context.
2. snaps collider bounds to the context `GridWorld` voxel size.
3. scans the covered spatial grid cells and active `VoxelGrid` instances.
4. suppresses duplicate grid and voxel visits with context-local sets.
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
context-local partition pool. `GravitasWorldContext.Reset()` clears retained
partition membership before collider IDs are reused.

`PhysicsPartition` is a GridForge voxel partition payload. It stores
context-local collider IDs, not collider references. Dynamic and static
membership uses `SwiftSparseSet`: lookups and removals are keyed by collider ID,
while dense-key storage keeps partition iteration compact. Do not treat dense-key
order as a semantic ordering rule; deterministic pair ordering must be explicit
at the pair or service layer. The owner service is required before the partition
is added to a voxel. The partition returns to that owner service pool when
GridForge removes the voxel partition itself, either through retained-partition
retirement or world/grid cleanup.

Current static-membership behavior is specific: colliders whose body exists and
has `Immovable == true` are added to `ContainedStaticObjects`. Other registered
colliders are added to `ContainedDynamicObjects`, including bodyless colliders.
Dynamic partitions also keep `ContainedAwakeDynamicObjects`, a second sparse set
for dynamic collider IDs whose bodies are currently awake for collision work.
Sleeping bodies stay in normal dynamic membership so queries, wake propagation,
pair cleanup, and contact lifecycle can still find them.

`PhysicsPartition2D` mirrors the same ID-first lessons for pure 2D. Bodyless
2D colliders and immovable 2D bodies are static members. Movable 2D bodies are
dynamic members, and only awake dynamic IDs activate pair distribution. Sleeping
2D bodies remain query-visible in dynamic membership, but partitions with no
awake dynamic IDs skip solver work. Empty 2D partitions are retained, retired by
the same deterministic TTK settings, and returned to the 2D collision service's
partition pool through GridForge voxel removal.

## Collider Runtime Shape State

`LSCollider` separates collider identity and host binding from the dense mutable
state used by shape rebuilds, partition ownership, query duplicate suppression,
pair cleanup, and hierarchy filtering. The runtime-shape snapshot watches:

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

Partition state tracks grid coordinates, previous snapped grid bounds,
partition-change flags, and broad-phase versioning together. Query state tracks
the raycast and circle-query versions used by context-owned query services to
suppress duplicate collider hits. Pair state owns the one-sided collision-pair
dictionary and the opposite-side holder set; both are allocated lazily so
colliders that never form pairs do not pay for pair containers up front.
Broad-phase versioning advances from committed runtime-shape changes, so
collision pairs do not need separate collider position/rotation dirty flags.
If the runtime snapshot changes, bounds/partition state refreshes and pairs
observe the broad-phase version change.

Hierarchy state is explicit. Hosts call `child.SetParent(parent)` after collider
initialization when two colliders belong to the same engine object or aggregate
body and should not collide with each other. Gravitas stores the top parent
collider ID for filtering; it does not walk host transform trees at simulation
time. When a parent collider deactivates, its child bindings are cleared before
the parent collider ID returns to the reusable ID pool, preventing stale
hierarchy IDs from suppressing collisions against future unrelated colliders.

`LSCompoundCollider` is different from hierarchy binding. A parent/child
relationship links independently registered colliders that may represent
separate host objects. A compound collider owns internal geometry parts under
one collider ID, one body binding, one broad-phase entry, and one contact/event
surface. Compound parts are not registered with `GravitasPhysicsService`, cannot
be parented independently, and are scanned in stable part order by the owning
compound collider.

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

Mesh policy work should keep these boundaries explicit:

- Concave triangle meshes are supported for static, kinematic, immovable, and
  dynamic bodies, but they should be chosen deliberately because candidate
  count scales with local triangle density.
- Convex mesh paths remain free to use whole-shape convex tests where valid.
- Compound colliders present one collider identity to hosts and one body to the
  solver, while internally ordering primitive or convex-mesh parts by stable
  part index. They aggregate part bounds, approximate mass/inertia from the
  parts, emit one event surface, and draw part geometry through the owning
  collider ID. Concave mesh parts are rejected; concave behavior belongs to
  `LSMeshCollider`.
- Host/offline convex decomposition should feed explicit convex mesh data or a
  future mesh-piece data path without changing the owning collider identity.
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
- `PhysicsMesh.CalculateInertiaTensor(...)` is currently an approximation. Any
  replacement should define whether the mesh is a thin shell, closed volume, or
  decomposed set of solids, then prove expected fixed-point values on simple
  reference meshes.

## Continuous Collision Detection

CCD is body-owned and opt-in. `PhysicsSettings.DefaultContinuousCollisionMode`
defaults to `Discrete`; `StiffBody.ContinuousCollisionMode` defaults to
`Inherit`, so existing bodies keep the discrete integration path unless the host
sets a context default, enables a body explicitly, or assigns a top-level parent
body with an explicit CCD mode. `ColliderHierarchyState` caches the top parent
when parent relationships are bound, so `Inherit` can check the parent policy in
constant time before falling back to the context default.

The alpha CCD path runs during `StiffBody` position integration, after velocity
and acceleration have produced an intended frame displacement and before the
authoritative position is committed. It uses a conservative swept-sphere proxy
derived from the moving collider:

- sphere, capsule, and cylinder use their scaled radius.
- cuboid uses the smallest world-space bounds half extent.

`Continuous` always sweeps when the proxy radius and displacement are non-zero.
`Auto` sweeps only when the intended displacement is larger than the proxy
radius. When a hit is accepted, the body clamps to the earliest swept center
time of impact and removes only the closing component of linear velocity,
preserving tangential velocity for later discrete response work.

Accepted CCD targets are non-trigger bodyless colliders, immovable bodies, and
kinematic bodies whose layers are allowed by the context collision matrix and
whose hierarchy is not excluded. Ordinary dynamic-vs-dynamic CCD is
intentionally deferred; it needs relative-velocity TOI ordering, pair
tie-breakers, and replay tests before it becomes part of the alpha contract.
Mesh targets are also excluded from swept-sphere CCD until Phase 7 defines the
mesh alpha policy.

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
- inverse mass is zero for immovable and kinematic bodies.
- linear velocity is world units per second.
- angular velocity is radians per second around each local/world axis.
- inertia tensors are diagonal fixed-point approximations supplied by the
  collider shape and transformed by `StiffBody`.
- restitution is clamped to `[0, 1]` and combined by the lower coefficient so a
  low-bounce participant can dampen the pair.
- closing speeds at or below `RestitutionVelocityThreshold` use zero
  restitution to avoid resting-contact bounce.
- `StiffBody.FrictionCoefficient` is a non-negative Coulomb coefficient. Values
  above one are allowed for intentional high-friction materials.
- friction impulses oppose tangential contact motion and are clamped by the
  normal impulse. Pair-local warm-start storage now records normal and tangent
  impulses by contact identity; applying cached impulses as a true warm-started
  iterative solve remains a later solver hardening step.
- penetration depth is a world distance from narrow phase; response slop is a
  solver invariant, not contact data.
- drag and angular damping remain integration/body behavior; contact friction is
  handled by the response solver.

This is still the first alpha milestone, not a full response engine. Static
friction for resting stacks, dynamic-vs-dynamic CCD, full iterative warm-start
application, explicit island solving, and 2D/3D mixed-dimension exchange rules
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
