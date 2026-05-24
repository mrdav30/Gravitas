# Collision Pipeline

Gravitas collision work is split into GridForge-backed broad phase,
context-local pair management, shape-pair narrow phase, prototype response, and
late contact notification.

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
8. adds the collider ID to the partition dynamic or static list.

`PhysicsPartition` is a GridForge voxel partition payload. It stores
context-local collider IDs, not collider references. The owner service is
required before the partition is added to a voxel, and the partition returns to
that owner service pool from `OnRemoveFromVoxel(...)`.

Current static-list behavior is specific: colliders whose body exists and has
`Immovable == true` are added to `ContainedStaticObjects`. Other registered
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

Mesh colliders validate vertices and triangle indices at construction time.
Triangle BVH entries and mesh query bounds use min/max `FixedBoundVolume`
coordinates. Mesh collider construction transforms vertices from local mesh
space only after the collider is bound to runtime state, so rotated meshes
refresh bounds from transformed vertices instead of copying stale constructor
bounds.

## Active Partitions

A partition becomes active when its dynamic list transitions from empty to
non-empty. Active partitions are stored in
`GravitasCollisionService._activePartitions`.

During `context.Simulate()`, `GravitasPhysicsService.Simulate()` calls
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
3. either run collision work immediately or decrement the cull counter.
4. if a collider moved into a new partition while culled, reset culling and
   re-run collision work.

Fast rejection happens before narrow phase:

- squared distance between collider centers is checked against combined bounds
  scope.
- collider AABB bounds must intersect.

If the pair is not colliding, `CalculateCullScore()` combines distance,
relative velocity, and time-since-last-collision into a frame countdown. Large
or explicitly protected colliders can prevent culling.

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
| Mesh/Sphere | closest mesh surface point to sphere center. |
| Mesh/Capsule | closest capsule line point to mesh surface. |
| Mesh/Cuboid | mesh/cuboid SAT using nearby mesh triangles. |
| Mesh/Mesh | mesh/mesh SAT using nearby mesh triangles. |

Cylinder collider methods are currently not implemented. Mesh raycast overlap is
also disabled, even though mesh collision checks exist.

## Contact Data

The narrow phase writes a `ContactPoint`:

- point on collider A.
- point on collider B.
- penetration depth.
- normal.
- optional immovable collision direction.

`ContactPoint.SetContactPoint(...)` clamps depth to at least a small
penetration margin. That margin helps avoid tiny corrections that fail to
separate bodies, but it is also one of the response details that needs alpha
hardening.

## Response

`CollisionPair.ProcessCollision()` calls `CollisionResponse.CalculateImpulse(...)`
when detection reports a collision and the pair should perform physics response.
Pairs with either collider marked as a trigger skip physical response; they can
still flow through contact notification.

Current non-trigger response behavior:

1. apply position correction based on penetration depth, normal direction,
   inverse masses and immovable flags.
2. compute contact velocity from linear velocity plus angular velocity at the
   contact inputs.
3. project contact velocity onto the contact normal.
4. compute an impulse scalar using restitution, inverse mass, and angular
   inertia effect.
5. apply linear impulse to movable bodies.
6. apply angular impulse when angular forces are allowed.

This is intentionally documented as prototype response. It is a strong candidate
for future redesign around contact manifolds, stable stacking, continuous
collision detection, friction impulses, restitution thresholds, angular units,
and physically explainable 2D/3D interaction rules.

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
