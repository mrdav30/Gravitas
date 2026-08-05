# Collision Broad Phase

The broad phase maps colliders into GridForge-backed voxel partitions and emits
stable candidate pairs for narrow phase. It is responsible for candidate
coverage, mobility buckets, duplicate suppression, retained partition lifetime,
and deterministic pair routing.

## Quick Read

- 3D uses `PhysicsPartition`.
- 2D uses `PhysicsPartition2D` on the internal Y=0 storage plane.
- Mixed uses `PhysicsMixedPartition` with dimension-tagged 3D/2D candidate keys.
- Partitions store context-local collider IDs, not collider references.
- Static, kinematic, dynamic, and awake-dynamic membership are separate.
- Sleeping colliders remain query-visible but do not activate solver work by
  themselves.
- Empty partitions are retained for a deterministic frame window, then retired
  through a bounded sweep budget.

## Partition Owners

| Domain | Partition type          | Owner service                   |
| ------ | ----------------------- | ------------------------------- |
| 3D     | `PhysicsPartition`      | `GravitasCollisionService`      |
| 2D     | `PhysicsPartition2D`    | `GravitasCollision2DService`    |
| Mixed  | `PhysicsMixedPartition` | `GravitasMixedCollisionService` |

Partition ownership must flow through the owning service. Do not manually return
the same partition through a second path; that risks double-release and stale
activation state.

## 3D Partitioning

When a collider initializes, moves, rotates, changes scale, or changes local
shape inputs, `LSCollider` rebuilds runtime shape data and asks
`GravitasCollisionService` to repartition it. Shape inputs are tracked by an
internal snapshot so several local edits before a simulation call collapse into
one bounds/shape rebuild.

Prepared canonical geometry, analytical conservative bounds, and mass properties
commit atomically. Bounds clipped to the representable coordinate domain exist
only for partition coverage; narrow phase always consumes the canonical shape.

`GravitasCollisionService.PartitionObject(...)`:

1. validates that the collider belongs to the service context.
2. asks GridForge `GridTracer.GetCoveredVoxelsInto(...)` for topology-aware
   voxel coverage.
3. uses GridForge traversal state and topology metrics for conservative
   voxel-position padding.
4. suppresses duplicate voxel visits with GridForge traversal helpers and
   context-local sets.
5. checks that the voxel position falls within collider bounds.
6. rents or reuses a `PhysicsPartition` on the voxel.
7. stores the collider's `WorldVoxelIndex`.
8. adds the collider ID to the partition mobility bucket.

When a collider leaves a voxel, Gravitas removes the collider ID from the
partition but keeps the empty `PhysicsPartition` attached to that voxel. Empty
partitions are inactive and query-invisible. `PhysicsSettings`
`RetainedPartitionTimeToKillFrames` controls the deterministic retention window,
and `RetainedPartitionRetirementSweepBudget` bounds how many retained partitions
the collision service checks per distribution step.

`GravitasWorldContext.Reset()` is a stronger session boundary: it detaches
retained Gravitas partition payloads from GridForge voxels, clears retained
tracking, and clears partition pools. Within a session, released collider IDs
are not cached for reuse; reset starts a fresh context-local allocation
sequence.

## 2D Partitioning

`PhysicsPartition2D` mirrors the same ID-first model for 2D. `LSCollider2D`
rebuilds its `FixedBoundArea`, then `GravitasCollision2DService` maps X/Z bounds
into GridForge voxels on the internal Y=0 storage plane.

That Y=0 plane is broad-phase identity only. It is not physical thickness and
does not imply mixed collision. Mixed collision uses separate embedded 2D slab
state owned by `LSCollider2D` and `GravitasMixedCollisionService`.

2D partitions retain and retire empty payloads through the same deterministic
TTK settings and return them to the 2D collision service's pool.

## Mixed Partitioning

Mixed mode uses `PhysicsMixedPartition` payloads attached to GridForge voxels.
The service refreshes mixed 3D and 2D membership after 3D and 2D body
integration. Embedded 2D membership uses `LSCollider2D.MixedBounds3D`: 2D X/Z
bounds plus a positive Y half-thickness centered on the host transform's Y
position.

Mixed candidate links are dimension-tagged. Plain 2D and 3D collider IDs are not
comparable because the services intentionally own separate ID spaces.

## Mobility Buckets

Membership is explicit:

| Collider/body state                      | Partition bucket           |
| ---------------------------------------- | -------------------------- |
| Bodyless collider                        | static                     |
| `MotionType == BodyMotionType.Static`    | static                     |
| `MotionType == BodyMotionType.Kinematic` | kinematic                  |
| `MotionType == BodyMotionType.Dynamic`   | dynamic                    |
| Awake dynamic body with solver mobility  | dynamic plus awake-dynamic |

Freeze axes never change the partition role. Partial freezes and fully locked
dynamic bodies remain in dynamic membership; the solver constrains their degrees
of freedom through effective mass and inertia. A fully locked dynamic body does
not seed awake pair distribution because it has no solver mobility, but it
remains available to contacts, queries, wake propagation, and an awake
counterpart's candidate traversal.

Only awake dynamic membership activates pair distribution for solver work.
Sleeping bodies remain in normal dynamic membership so queries, wake
propagation, pair cleanup, and contact lifecycle retain access to them.

## Active Partitions And Pair Distribution

An active partition is one with awake dynamic members. Active partitions emit
candidate pairs against local static, kinematic, dynamic, and awake-dynamic
members according to the owning service's rules.

Candidate generation must be deterministic:

- voxel and partition traversal order is explicit.
- collider ID ordering is explicit.
- pair keys are stable.
- duplicate-pair suppression is applied before narrow phase.
- retained sleeping pairs can keep contact state without forcing fresh narrow
  phase every frame.

If every dynamic body in a partition is sleeping, pair generation is skipped
until a deterministic wake reason changes a body or collider shape state.

## Pair Filters

Candidate pairs are rejected before exact shape work when any required filter
fails:

| Filter                        | Notes                                                                                                                                                     |
| ----------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Context                       | Colliders must belong to the same context.                                                                                                                |
| Active state                  | Inactive colliders do not produce pairs.                                                                                                                  |
| Same agent                    | Host-owned same-agent collisions are suppressed.                                                                                                          |
| Explicit hierarchy            | Parent/child and sibling filters use dimension-tagged hierarchy keys.                                                                                     |
| Collision matrix              | Context-wide physical layer matrix must allow the pair.                                                                                                   |
| Collider-local ignored layers | Either collider can reject the other collider's layer.                                                                                                    |
| Trigger policy                | Bodyless trigger volumes skip physical response and emit trigger events only when exactly one collider is a trigger and the other collider is body-owned. |
| Bounds                        | Broad bounds must overlap before narrow phase.                                                                                                            |
| Awake/resting policy          | Sleeping/sleeping fresh links exit; retained resting links can keep island state.                                                                         |

Collider-local ignored layer masks affect collision pairs, trigger pairs,
internal CCD target eligibility, and grounding/support acceptance. Public query
services use caller-owned include masks instead.

## Duplicate Suppression

One broad collider can cover many voxels. Gravitas suppresses duplicate work in
two layers:

1. Voxel traversal suppresses duplicate voxel visits while collecting coverage
   or query candidates.
2. Pair routing sends a broad collider pair through its deterministic first
   shared partition before the frame duplicate-pair set.

This keeps multi-voxel colliders from generating repeated narrow-phase work or
repeated contact notifications.

## Pair Culling

Pair culling may delay checks only for non-colliding retained pairs according to
stable distance, velocity, age, and size scores. It must not skip active
contacts or nearby candidate pairs because a host considers an object less
important.

Authoritative collision detection is not asynchronous. Hosts may run independent
contexts on separate threads, but one context's collision and response phases
must keep stable observable order.

## Large Scene Policy

Large object counts are handled through deterministic broad-phase and solver
ownership:

- GridForge-backed voxel partitioning.
- retained `PhysicsPartition`, `PhysicsPartition2D`, and `PhysicsMixedPartition`
  payloads.
- mobility buckets and awake-dynamic sets.
- duplicate pair suppression.
- hierarchy, layer, and local physical filtering.
- retained collision pairs and warm-start caches.
- deterministic sleep state.
- CCD frame caches and bounded handoff queues.

Collision LOD is authored data, not camera-distance runtime mutation. A host or
offline tool may choose simpler fixed collision shapes for a simulation, but
Gravitas does not change authoritative collision geometry during a run based on
renderer distance or presentation priority.

## Deactivation Cleanup

`LSCollider.Deactivate()`:

1. clears partition membership through the owning collision service.
2. removes owned collision-pair references.
3. removes holder references from opposite colliders.
4. clears explicit parent binding.
5. deactivates and pools pairs when pooling is enabled.
6. returns the collider ID to the context-local physics service.
7. marks the collider inactive.

2D and mixed cleanup follow the same ownership principle through their owning
services.

## Source Map

| Area                     | Source                                                                                                                                                 |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 3D partition service     | [`src/Gravitas/Core/3D/GravitasCollisionService.cs`](../../src/Gravitas/Core/3D/GravitasCollisionService.cs)                                           |
| 2D partition service     | [`src/Gravitas/Core/2D/GravitasCollision2DService.cs`](../../src/Gravitas/Core/2D/GravitasCollision2DService.cs)                                       |
| Mixed partition service  | [`src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Partitioning.cs`](../../src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Partitioning.cs) |
| Partition payloads       | [`src/Gravitas/Partitions`](../../src/Gravitas/Partitions)                                                                                             |
| Collider partition state | [`src/Gravitas/Colliders/State`](../../src/Gravitas/Colliders/State)                                                                                   |
| Local filtering          | [`src/Gravitas/CollisionHandling/ColliderCollisionFilter.cs`](../../src/Gravitas/CollisionHandling/ColliderCollisionFilter.cs)                         |
