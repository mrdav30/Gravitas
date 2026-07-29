# Collision Pipeline

Collision in Gravitas is split into deterministic stages: broad phase, pair
filtering, narrow phase, contact manifold generation, response, notifications,
and cleanup. The same structure exists for 3D, 2D, and explicit mixed 2D/3D
contacts, with each domain owning its own state and ordering rules.

This page is the readable entry point. Use the reference pages when you need the
full implementation contract:

- [Collision Broad Phase](COLLISION_BROAD_PHASE.md)
- [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md)
- [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md)
- [Collision Response](COLLISION_RESPONSE.md)

## Quick Read

- 3D colliders route through `GravitasPhysicsService` and
  `GravitasCollisionService`.
- 2D colliders route through `GravitasPhysics2DService` and
  `GravitasCollision2DService`.
- Mixed contacts exist only in `PhysicsRuntimeMode.Mixed` and route through
  `GravitasMixedCollisionService`.
- GridForge-backed voxel partitions produce deterministic candidate pairs.
- Candidate pairs pass hierarchy, same-agent, layer, local physical ignore,
  trigger, awake, and bounds filters before narrow phase.
- Narrow phase writes deterministic contact manifolds.
- Response solves contacts and joints in stable islands during `LateSimulate()`.
- Contact notifications and pair cleanup happen after response.

```mermaid
flowchart LR
    Refresh["Refresh collider bounds"]
    Partition["GridForge partitions"]
    Candidates["Candidate pairs"]
    Filters["Deterministic filters"]
    Narrow["Shape narrow phase"]
    Contacts["Contact manifolds"]
    Response["Response islands"]
    Notify["Notifications + cleanup"]

    Refresh --> Partition --> Candidates --> Filters --> Narrow --> Contacts --> Response --> Notify
```

## Runtime Paths

| Path        | Body/collider types           | Broad phase                                            | Pair/response owner             |
| ----------- | ----------------------------- | ------------------------------------------------------ | ------------------------------- |
| 3D          | `SolidBody`, `LSCollider`     | `PhysicsPartition` on GridForge voxels                 | `GravitasPhysicsService`        |
| 2D          | `SolidBody2D`, `LSCollider2D` | `PhysicsPartition2D` on the internal Y=0 storage plane | `GravitasPhysics2DService`      |
| Mixed 2D/3D | existing 3D and 2D types      | `PhysicsMixedPartition` using embedded 2D slabs        | `GravitasMixedCollisionService` |

2D uses X/Z host projection: world `Vector3d.x` maps to `Vector2d.x` and world
`Vector3d.z` maps to `Vector2d.y`. World `Vector3d.y` is height or mixed
embedding metadata, not a 2D collision axis.

`PhysicsRuntimeMode.Both` runs 2D and 3D side by side without cross-dimensional
contacts. `PhysicsRuntimeMode.Mixed` adds the dedicated mixed lifecycle.

## End-To-End Phase Order

Collision work is part of `context.LateSimulate()`:

1. Bodies integrate motion and CCD frame state.
2. Dynamic collider bounds and partitions refresh.
3. Active partitions emit candidate pairs in stable order.
4. Pair filters reject invalid or non-physical interactions.
5. Narrow phase evaluates exact shape-pair collision.
6. Contact manifolds are written to pair-owned storage.
7. Contact and joint rows are solved in deterministic islands.
8. Grounding/support and sleep state update from post-response state.
9. Contact events, pair maintenance, retained partition cleanup, and
   deactivation cleanup run.

Mixed contacts run after both dimension-local services have integrated and
refreshed their own collider partitions, so mixed response observes
post-integration 2D and 3D positions.

## Pair Filtering

Broad-phase candidate pairs are filtered before exact shape work:

| Filter                        | Purpose                                                  |
| ----------------------------- | -------------------------------------------------------- |
| Context ownership             | Reject colliders from another context.                   |
| Same-agent/hierarchy          | Suppress host-owned sibling/parent-child collisions.     |
| Runtime mode                  | Keep 2D, 3D, `Both`, and `Mixed` behavior explicit.      |
| Mobility/awake state          | Avoid response work for fully sleeping local partitions. |
| Layer matrix                  | Apply context-wide physical collision policy.            |
| Collider-local ignored layers | Apply per-collider physical ignore masks.                |
| Bounds                        | Reject separated broad colliders before narrow phase.    |
| Duplicate partition routing   | Ensure a pair shared by several voxels runs once.        |

Public queries do not use collider-local ignored physical layer masks. Query
include masks are caller-owned; see [Query Services](QUERY_SERVICES.md).

## Narrow Phase

Narrow phase owns shape truth. Supported shape coverage includes:

| Domain | Shape families                                                                     |
| ------ | ---------------------------------------------------------------------------------- |
| 3D     | sphere, capsule, cuboid, finite cylinder, finite cone, mesh, compound              |
| 2D     | circle, capsule, AABB, convex polygon, compound                                    |
| Mixed  | supported 3D shapes against embedded 2D circle/capsule/AABB/polygon/compound slabs |

Convex SAT paths use stable axis generation and pair-oriented normals. Cuboid
versus capsule checks first solve exact segment-to-oriented-box distance for
rounded features, then use ordered SAT only when the capsule core reaches the
box; inclusive projections and directional exit depths preserve touching and
containment semantics. Convex mesh versus capsule checks use the closest
capsule-segment point to the mesh center for their exterior representative
manifold and fall back to deterministic BVH traversal with stable contact-ID
reduction when contact exists away from it. Closed-convex containment instead
orients face planes from the scaled world-space center of mass and reduces the
whole capsule over face and edge-cross axes to a matched support-feature exit
manifold. Other mesh paths use the same deterministic BVH candidate ownership;
candidate order follows the stable built tree rather than authored triangle
indices. Compound paths scan parts in stable declaration order and return the
owner collider as the public identity.

Finite axes, oriented cuboids, and planar convex shapes retain center-relative
canonical geometry through narrow phase. Contact witnesses use
`ContactAnchor` or `ContactAnchor2D`: a representable origin, normalized frame
rotation, and representable local point. Solvers and replay hashes consume that
canonical frame directly, so a valid contact is not dropped or deformed merely
because its rotated offset or absolute world point crosses a `Fixed64` scalar
face. `Origin`, `Rotation`, `LocalPoint`, and `LocalDisplacement` expose the
canonical components. The two local terms remain separate until exact
evaluation so a representable world witness is not lost to an overflowing
local intermediate.
`Offset` and legacy `PointA`/`PointB` views materialize derived coordinates and
throw when the requested view is not representable; domain-edge callers should
use `TryGetOffset` and the matching `TryGetPoint*` method.

Response keeps materialized 2D/3D lever vectors as the ordinary fast path and
reconstructs a Gravitas-owned `ExactLever3D` from contact anchors only when the
complete compact expression cannot be proven representable. Point velocity,
effective mass, warm-start completion, friction, and final body deltas then
remain exact through one final checked narrowing. Compound mass properties use
the Gravitas-owned `ExactMassPoint3D`, `ExactMassPoint2D`, and
`ExactMassWeight` types for the equivalent weighted-center and parallel-axis
contract; no saturated child center or weight is admitted as physical data.
These internal physics semantics consume FixedMathSharp's policy-neutral wide
arithmetic through the intentional friend-assembly boundary and never enter
Gravitas public signatures.

Embedded 2D mixed volumes also select planar boundary anchors semantically.
Built-in circles, capsules, boxes, polygons, and compounds therefore do not
require a public closest point or representable query-to-boundary distance to
produce a contact witness. Exact compound candidate ranking preserves authored
part order on ties.

For shape state, pair matrices, SAT invariants, mesh policy, and compound
ownership details, read [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md).

## Continuous Collision Detection

CCD is opt-in per body or through context defaults. The runtime supports:

- static/kinematic blockers for fast dynamic bodies.
- dynamic-vs-dynamic candidate indexing and relative movement checks.
- host-driven kinematic active sources.
- rotational CCD where supported.
- service-level handoff queues for chained contacts.
- mixed handoffs when `PhysicsRuntimeMode.Mixed` is active.

Sphere/cuboid time of impact uses the exact spherical dilation of the oriented
cuboid. The local-space reducer distinguishes planar faces from rounded edges
and corners, then reconstructs contact from the original authored world chord
without normalized-direction loss.

The CCD reference explains exact reducer paths, conservative proxy boundaries,
TOI ordering, and service counters:
[Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md).

## Response And Notifications

Non-trigger contacts are solved through deterministic manifold response.
Bodyless trigger volumes skip physical response and emit trigger notifications
only when exactly one collider in the pair is a trigger and the other collider
is body-owned. Both colliders in a valid trigger pair receive enter, stay, and
exit callbacks.

2D, 3D, and mixed pair callbacks use stable pair order: A/B for same-dimension
pairs and 3D/2D for mixed pairs. Exit admission is consumed before user
delegates run, so callback failure or reentrant teardown does not retry an
already admitted exit against the same pair lifetime. Both admitted sides are
attempted even if the first throws: one exception is re-thrown with its
original stack, while multiple failures are reported as an `AggregateException`
in pair order. Deferred exits retain the pair's notification guard until
cleanup finishes, preventing direct deactivation from reentering the same
separation.

3D and 2D response both:

- combine contact rows with enabled joint rows.
- build deterministic body islands.
- wake connected sleeping bodies when an island has an awake participant.
- apply warm-start impulses from pair-local caches.
- solve bounded iterations from `PhysicsSettings.DiscreteSolverIterations`.
- update sleep state after response.

Mixed response is constrained: 2D participants receive planar X/Z correction,
planar velocity deltas, and scalar yaw angular deltas. Vertical Y impulse is
constrained out of the 2D body model.

For contact manifolds, material response, warm-start caches, sleep/wake, and
event timing, read [Collision Response](COLLISION_RESPONSE.md).

## Common Extension Points

| Goal                                               | Start with                                                                                             |
| -------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| Add or improve a shape pair                        | [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md), then shape-pair tests.                        |
| Change broad-phase partition behavior              | [Collision Broad Phase](COLLISION_BROAD_PHASE.md), then partition/candidate benchmarks.                |
| Change tunneling behavior                          | [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md), then CCD replay and stress tests. |
| Change friction/restitution/contact solve behavior | [Collision Response](COLLISION_RESPONSE.md), then response invariant tests and benchmarks.             |
| Add host-facing query behavior                     | [Query Services](QUERY_SERVICES.md) and [Query Reference](QUERY_REFERENCE.md).                         |

## Rules That Matter

- Preserve deterministic pair, partition, contact, and response ordering.
- Use `Fixed64`, `Vector2d`, `Vector3d`, and fixed-point geometry in runtime
  collision code.
- Keep 2D, 3D, and mixed collision paths explicit.
- Do not compare plain collider IDs across dimensions.
- Avoid LINQ and iterator allocations in collision hot paths.
- Pool only when lifetime and ownership are obvious and testable.
- Add focused tests for separated, touching, overlapping, degenerate, rotated,
  and high-speed cases when changing collision behavior.
- Add benchmarks for partitioning, pair distribution, narrow phase, CCD, or
  response loop changes.

## Source Map

| Area                      | Source                                                                                                                                                                                                                                                                                                                                                                 |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 3D collision service      | [`src/Gravitas/Core/3D/GravitasCollisionService.cs`](../../src/Gravitas/Core/3D/GravitasCollisionService.cs)                                                                                                                                                                                                                                                           |
| 3D physics/pairs/response | [`src/Gravitas/Core/3D/GravitasPhysicsService.cs`](../../src/Gravitas/Core/3D/GravitasPhysicsService.cs), [`src/Gravitas/Core/3D/GravitasPhysicsService.Pairs.cs`](../../src/Gravitas/Core/3D/GravitasPhysicsService.Pairs.cs), [`src/Gravitas/Core/3D/GravitasPhysicsService.Response.cs`](../../src/Gravitas/Core/3D/GravitasPhysicsService.Response.cs)             |
| 2D collision service      | [`src/Gravitas/Core/2D/GravitasCollision2DService.cs`](../../src/Gravitas/Core/2D/GravitasCollision2DService.cs)                                                                                                                                                                                                                                                       |
| 2D physics/pairs/response | [`src/Gravitas/Core/2D/GravitasPhysics2DService.cs`](../../src/Gravitas/Core/2D/GravitasPhysics2DService.cs), [`src/Gravitas/Core/2D/GravitasPhysics2DService.Pairs.cs`](../../src/Gravitas/Core/2D/GravitasPhysics2DService.Pairs.cs), [`src/Gravitas/Core/2D/GravitasPhysics2DService.Response.cs`](../../src/Gravitas/Core/2D/GravitasPhysics2DService.Response.cs) |
| Mixed collision           | [`src/Gravitas/Core/Mixed`](../../src/Gravitas/Core/Mixed)                                                                                                                                                                                                                                                                                                             |
| Narrow phase              | [`src/Gravitas/CollisionHandling/Detection`](../../src/Gravitas/CollisionHandling/Detection)                                                                                                                                                                                                                                                                           |
| Contact data              | [`src/Gravitas/CollisionHandling/Contacts`](../../src/Gravitas/CollisionHandling/Contacts)                                                                                                                                                                                                                                                                             |
| Response                  | [`src/Gravitas/CollisionHandling/Response`](../../src/Gravitas/CollisionHandling/Response)                                                                                                                                                                                                                                                                             |
| Collision tests           | [`tests/Gravitas.Tests/CollisionHandling`](../../tests/Gravitas.Tests/CollisionHandling), [`tests/Gravitas.Tests/Physics2D`](../../tests/Gravitas.Tests/Physics2D), [`tests/Gravitas.Tests/MixedDimensions`](../../tests/Gravitas.Tests/MixedDimensions)                                                                                                               |
