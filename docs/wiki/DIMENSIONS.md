# 2D, 3D, And Runtime Modes

Gravitas is moving from a 3D-only prototype toward first-class 2D,
first-class 3D, and eventually mixed 2D/3D interaction. The current contract is
type-driven: `StiffBody` and `LSCollider` are the 3D path, while `StiffBody2D`
and `LSCollider2D` are the pure 2D path.

There is no `PhysicsDimension` enum in the runtime. Concrete body and collider
types define the simulation domain. Mixed behavior will be a policy between
those concrete types, not a third dimension value.

## Runtime Mode

`PhysicsSettings.RuntimeMode` selects which dimensional service a
`GravitasWorldContext` advances:

- `PhysicsRuntimeMode.ThreeD` advances `GravitasPhysicsService` and skips
  `GravitasPhysics2DService`.
- `PhysicsRuntimeMode.TwoD` advances `GravitasPhysics2DService` and skips
  `GravitasPhysicsService`.

The context clock, coroutines, diagnostics, and lifecycle hooks remain shared.
This lets pure 2D simulations use the same host loop without paying 3D
simulation or visualization cost. `Mixed` is intentionally absent until Phase 10
defines embedding, contact manifolds, and impulse exchange.

## Pure 2D Coordinate Contract

Pure 2D uses the LSF stack's X/Z planar convention:

- authoritative 2D position is `Vector2d`.
- `Vector2d.x` maps from world `Vector3d.x`.
- `Vector2d.y` maps from world `Vector3d.z`.
- world `Vector3d.y` remains vertical height or future embedding metadata.
- `Vector3d.ToVector2d()` is the correct conversion for this convention.
- scalar 2D rotation maps to yaw around the world Y axis when syncing from a
  host `FixedTransform`.

Pure 2D body motion has no `HeightPos`, ground probe, step offset, or grounded
platform state. Those belong to the current 3D y-up body model.

## 2D Bodies And Colliders

`StiffBody2D` is created with an `IMatterAgent` and an `LSCollider2D`, matching
the host-facing shape of the 3D body API. The agent supplies the context and
host transform bridge. Kinematic 2D bodies read the agent transform during
`LateSimulate` and project its X/Z position into authoritative `Vector2d`
state.

`LSCollider2D.InitializeWithNoBody(IMatterAgent)` binds bodyless static or
trigger colliders to the same host contract. Bodyless 2D colliders register
with `GravitasPhysics2DService`, participate in queries, triggers, layer
filtering, cleanup, and static collision response.

The current pure 2D shape set is:

- `LSCircleCollider2D`
- `LSAABBoxCollider2D`
- `LSPolygonCollider2D`

`LSPolygonCollider2D` validates convexity and rejects concave or collinear
input instead of silently accepting ambiguous collision truth. A rotated box
should be represented as a convex polygon for now; `LSAABBoxCollider2D` remains
axis-aligned by design.

## Broad Phase Status

The current pure 2D broad phase is GridForge-backed. `LSCollider2D` rebuilds
its `FixedMathSharp.BoundingArea` when body motion, host transform refresh, or
shape inputs change, then `GravitasCollision2DService` maps the X/Z bounds into
`PhysicsPartition2D` payloads attached to GridForge voxels. Partitions store
collider IDs, split static and dynamic membership, keep a separate awake
dynamic set, and distribute candidates in sorted deterministic order.

Pure 2D partition storage uses the internal Y=0 GridForge plane. This is only a
stable voxel identity for broad-phase lookup; it is not a public 3D slab,
thickness, or mixed-dimension contact rule.

`Physics2DBounds` has been removed. Do not add another public bounds/config
bridge for pure 2D.

## Queries

Pure 2D queries live on `GravitasPhysics2DService`:

```csharp
context.Physics2D.OverlapCircleAll(center, radius, results);
context.Physics2D.OverlapCircleAll(center, radius, layerMask, results);
```

Both overloads write into the caller-owned `SwiftList<Physics2DHit>`, run
GridForge-backed partition candidate gathering with duplicate suppression, run
layer-mask and exact 2D shape checks, and sort by deterministic hit ordering.

The existing `GravitasCircleQueryService` is a 3D X/Z ground-plane proximity
query. It is not the pure 2D query API.

## Mixed 2D/3D Boundary

Mixed 2D/3D is intentionally Phase 10. It must define:

- how a 2D plane is embedded in 3D space.
- whether 2D shapes have finite physical thickness.
- how 3D contact points project onto 2D manifolds.
- how impulses and positional correction exchange between 2D and 3D bodies.
- which shape pairs are supported, experimental, or rejected.

Until that contract exists, pure 2D and 3D collision dispatch should not fall
through to accidental mixed behavior.
