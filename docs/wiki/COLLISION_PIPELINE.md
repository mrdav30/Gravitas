# Collision Pipeline

Gravitas collision work is split into GridForge-backed broad phase,
context-local pair management, shape-pair narrow phase, single-contact
response, and late contact notification.

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

`PhysicsPartition` is a GridForge voxel partition payload. It stores
context-local collider IDs, not collider references. Dynamic and static
membership uses `SwiftSparseMap<byte>` as a sparse set: lookups and removals are
keyed by collider ID, while dense-key spans keep partition iteration compact. Do
not treat dense-key order as a semantic ordering rule; deterministic pair
ordering must be explicit at the pair or service layer. The owner service is
required before the partition is added to a voxel, and the partition returns to
that owner service pool from `OnRemoveFromVoxel(...)`.

Current static-membership behavior is specific: colliders whose body exists and
has `Immovable == true` are added to `ContainedStaticObjects`. Other registered
colliders are added to `ContainedDynamicObjects`, including bodyless colliders.

## Collider Runtime Shape State

`LSCollider` separates collider identity and pair/partition ownership from the
derived runtime shape snapshot used by bounds, area, and shape-specific caches.
The current snapshot watches:

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

Capsules rebuild their hemisphere centers, cylinder height, area, and segment
endpoints together. Short capsules collapse to a sphere-like segment and use a
sphere inertia fallback instead of producing a zero diagonal for the capsule's
main axis.

Cylinders rebuild their finite flat-capped axis segment, cap centers, height,
area, and inertia inputs together. Cylinder support intentionally treats the
shape as a real finite cylinder rather than a capsule with ignored hemispheres,
so cap separation and side separation are tested independently.

`FixedTransform.LossyScale` uses basis-vector scale extraction rather than raw
matrix diagonals. This matters for rotated colliders because diagonal extraction
can report near-zero scale for 90-degree rotations and collapse derived shape
state.

Mesh colliders validate vertices and triangle indices at construction time.
Triangle BVH entries and mesh query bounds use min/max `FixedBoundVolume`
coordinates. Mesh collider construction transforms vertices from local mesh
space only after the collider is bound to runtime state, so rotated meshes
refresh bounds from transformed vertices instead of copying stale constructor
bounds.

Alpha mesh policy is conservative: non-convex meshes should be decomposed into
convex sub-meshes offline or during initialization, not during per-frame
collision. Mesh ray overlap and initial mesh/cylinder contact are
triangle-backed and covered by focused tests, but dynamic mesh behavior,
arbitrary mesh contact manifolds, and swept mesh queries remain hardening
targets.

## Active Partitions

A partition becomes active when its dynamic membership transitions from empty to
non-empty. Active partitions are stored in
`GravitasCollisionService._activePartitions`.

During `context.Simulate()`, `GravitasPhysicsService.Simulate()` first lets
registered dynamic-body colliders refresh bounds and partition membership. This
pre-distribution pass catches host command teleports or direct body moves made
between frames. It then calls
`GravitasCollisionService.CheckAndDistributeCollisions()`. The collision service
increments its `Version` and asks each active partition to distribute candidate
pairs.

`PhysicsPartition.Distribute()` checks:

- every dynamic/dynamic pair in that partition.
- every dynamic/static pair in that partition.

Static/static pairs are not distributed.

## Pair Creation And Filtering

For each candidate ID pair, the partition asks
`owner.Context.Physics.GetCollisionPair(id1, id2)`.

The physics service resolves both IDs through the owning context and filters
with `RequireCollisionPair(...)`. A pair is required only when:

- both colliders are active.
- both colliders have real shapes.
- at least one collider has a body.
- the context collision matrix allows the two layers to collide.
- the colliders are not siblings in the host hierarchy.

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
3. invalidate culling if either collider changed position, rotation, partition
   state, or broad-phase version since the last pair check.
4. either run collision work immediately or decrement the cull counter.

Fast rejection happens before narrow phase:

- squared distance between collider centers is checked against combined bounds
  scope.
- collider AABB bounds must intersect.

If a pair was colliding on the previous check, it can reuse that state only
while both colliders keep the same position, rotation, and broad-phase version.
Shape or bounds changes must re-run narrow phase even when object transforms did
not move.

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
| Cuboid/Cuboid | AABB overlap for axis-aligned boxes, SAT for oriented boxes. |
| Cylinder/Sphere | finite cylinder closest surface against sphere radius. |
| Cylinder/Capsule | finite-cylinder projection axes against capsule segment/radius projection. |
| Cylinder/Cylinder | finite-cylinder projection axes, preserving flat cap separation. |
| Cuboid/Cylinder | cuboid vertex projection against finite-cylinder projection. |
| Mesh/Sphere | closest mesh surface point to sphere center. |
| Mesh/Capsule | closest capsule line point to mesh surface. |
| Mesh/Cuboid | mesh/cuboid SAT using nearby mesh triangles. |
| Mesh/Cylinder | triangle-BVH candidate scan against finite cylinder volume. |
| Mesh/Mesh | mesh/mesh SAT using nearby mesh triangles. |

Current shape-pair matrix:

| A / B | Sphere | Capsule | Cuboid | Cylinder | Mesh |
| --- | --- | --- | --- | --- | --- |
| Sphere | Supported | Supported | Supported | Supported | Supported |
| Capsule | Supported | Supported | Supported | Supported | Supported |
| Cuboid | Supported | Supported | Supported | Supported | Supported |
| Cylinder | Supported | Supported | Supported | Supported | Supported |
| Mesh | Supported | Supported | Supported | Supported | Supported |

`Cuboid` covers both `AABox` and `OBBox` dispatch. `Cylinder/Mesh` is
normalized to `Mesh/Cylinder` by pair priority so contact data is written in the
mesh-to-cylinder direction.

## Contact Data

The narrow phase writes a `ContactPoint`:

- validity flag indicating narrow-phase contact data is present.
- point on collider A.
- point on collider B.
- penetration depth.
- normal.
- optional immovable collision direction.

`ContactPoint.SetContactPoint(...)` stores the narrow phase's detected depth
without adding a solver margin. Touching contacts can therefore have zero depth,
and any stabilization margin belongs to the response solver rather than hidden
inside contact data.

`ContactPoint.HasContact` distinguishes unset contact data from legitimate
zero-valued fields, such as touching contacts with zero depth or contact points
at the origin. The response solver ignores pairs whose contact data has not been
written by narrow phase.

## Response

`CollisionPair.ProcessCollision()` calls `CollisionResponse.CalculateImpulse(...)`
when detection reports a collision and the pair should perform physics response.
Pairs with either collider marked as a trigger skip physical response; they can
still flow through contact notification.

Current non-trigger response behavior is a deterministic single-contact solver:

1. build an explicit contact from collider A, collider B, the two contact
   points, relative contact arms, detected depth, and a normal oriented from A
   to B.
2. treat `Immovable` and `IsKinematic` bodies as infinite mass for response.
3. apply immediate positional correction only for depth above
   `CollisionResponse.PenetrationSlop`; the correction is distributed by
   inverse mass and scaled by `PenetrationCorrectionPercent`.
4. compute contact velocity from linear velocity plus angular velocity at each
   relative contact arm.
5. skip impulse when the bodies are already separating along the contact normal.
6. compute a normal impulse using inverse mass, inverse inertia, contact arms,
   and the combined restitution.
7. apply direct velocity deltas to movable bodies, plus angular velocity deltas
   when angular forces are enabled.

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
- penetration depth is a world distance from narrow phase; response slop is a
  solver invariant, not contact data.
- drag, friction, and angular damping remain integration/body behavior for now;
  tangential friction impulses are deferred until contact manifolds and stable
  stacking are designed.

This is still the first alpha milestone, not a full response engine. Contact
manifolds, friction impulses, continuous collision detection, warm starting,
island solving, and 2D/3D mixed-dimension exchange rules remain future work.

## Contact Notifications

Collision pairs are queued into the physics service active-pair queue the first
time they update. During `context.LateSimulate()`,
`GravitasPhysicsService.ProcessActiveCollisionPairs()`:

- deactivates pairs that have not collided for the inactive-frame threshold.
- emits ongoing contact notifications when the pair is still active and not
  culled.
- keeps active pairs queued for future maintenance.

`LSCollider.NotifyContact(...)` emits:

- `OnTriggerEnter` and `OnTriggerExit` for trigger colliders.
- `OnContactEnter`, `OnContact`, and `OnContactExit` for body contacts.

## Deactivation Cleanup

`LSCollider.Deactivate()`:

1. clears partition membership through the owning collision service.
2. removes owned collision-pair references.
3. removes holder references from the opposite colliders.
4. deactivates and pools pairs when pooling is enabled.
5. returns the collider ID to the context-local physics service.
6. marks the collider inactive.

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
