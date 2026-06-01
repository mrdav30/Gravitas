# 2D, 3D, And Runtime Modes

Gravitas is moving from a 3D-only prototype toward first-class 2D,
first-class 3D, and mixed 2D/3D interaction. The current contract is
type-driven: `StiffBody` and `LSCollider` are the 3D path, while `StiffBody2D`
and `LSCollider2D` are the pure 2D path.

There is no `PhysicsDimension` enum in the runtime. Concrete body and collider
types define the simulation domain. Mixed behavior will be a policy between
those concrete types, not a third dimension value.

## Runtime Mode

`PhysicsSettings.RuntimeMode` selects which dimensional service a
`GravitasWorldContext` advances:

- `PhysicsRuntimeMode.ThreeD` advances `GravitasPhysicsService` and skips
  `GravitasPhysics2DService` simulation and visualization.
- `PhysicsRuntimeMode.TwoD` advances `GravitasPhysics2DService` and skips
  `GravitasPhysicsService` simulation and visualization.
- `PhysicsRuntimeMode.Both` advances the pure 2D and pure 3D services side by
  side without cross-dimensional contacts.
- `PhysicsRuntimeMode.Mixed` advances both pure services plus the dedicated
  mixed lifecycle and broad-phase path. Mixed narrow phase supports 3D spheres,
  cuboids, capsules, finite cylinders, compound colliders, and mesh colliders
  against embedded 2D circle, AABB, and convex polygon slabs. Phase 10 response,
  diagnostic, mixed query, and CCD work fills in the rest of the interaction
  model.

The context clock, coroutines, diagnostics, and lifecycle hooks remain shared.
This lets pure 2D simulations use the same host loop without paying 3D
simulation or visualization cost. Runtime modes are validated exactly; `None`
and arbitrary bit combinations are rejected as settings values.

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

Dynamic 2D bodies publish their authoritative planar position and yaw rotation
back to the host `FixedTransform` during `Visualize()` when the context is in
`PhysicsRuntimeMode.TwoD`. The host transform's vertical `Vector3d.y` value is
preserved because it is not part of pure 2D physics.

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

## Mixed 2D Embedding State

`LSCollider2D` now carries the minimal deterministic 3D embedding data needed
by the mixed runtime path:

- `MixedHalfThicknessOverride` optionally overrides the context default.
- `PhysicsSettings.Mixed2DHalfThickness` supplies the default positive
  half-thickness.
- `MixedSlabCenterY` is cached from the host `FixedTransform.Position.y`.
- `MixedBounds3D` is a deterministic `BoundingBox` built from the pure X/Z
  2D bounds plus the cached Y slab.

The mixed 3D bounds do not change pure 2D collision truth or pure 2D partition
storage. They are the mixed broad-phase and mixed narrow-phase embedding volume
only.

## Queries

Pure 2D queries live on `GravitasWorldContext.Query2D`:

```csharp
context.Query2D.OverlapCircleAll(center, radius, results);
context.Query2D.OverlapCircleAll(center, radius, layerMask, results);
context.Query2D.Raycast(start, end, out Physics2DHit hit);
context.Query2D.RaycastAll(start, end, layerMask, results);
context.Query2D.SweepCircle(start, end, radius, out Physics2DHit hit);
context.Query2D.SweepCircleAll(start, end, radius, layerMask, results);
```

All-hit overloads write into caller-owned `SwiftList<Physics2DHit>` buffers,
run GridForge-backed partition candidate gathering with duplicate suppression,
run layer-mask and exact 2D shape checks, and sort by deterministic hit
ordering. `Raycast` returns the closest segment hit from `start` to `end` using
the same distance and collider-ID ordering as `RaycastAll`. `SweepCircle` is the
pure 2D swept movement/query path used by 2D CCD.

The existing `GravitasQuery3DService` is a 3D X/Z ground-plane proximity
query. It is not the pure 2D query API.

## Mixed 2D/3D Direction

Phase 10 is the first mixed runtime implementation. The target model is explicit
rather than Unity-style separate engines:

- `PhysicsRuntimeMode.Both` advances both pure 2D and 3D services without
  cross-dimensional contacts.
- `PhysicsRuntimeMode.Mixed` advances both pure 2D and 3D services plus a
  dedicated mixed collision lifecycle path. The mixed broad phase uses
  `PhysicsMixedPartition` and stable 3D/2D candidate keys. Mixed narrow phase
  currently supports 3D spheres, cuboids, capsules, finite cylinders, compound
  colliders, and mesh colliders against embedded 2D circle, AABB, and convex
  polygon slabs; response is filled in by the remaining Phase 10 work.
- mixed contacts embed 2D colliders into 3D as finite X/Z prisms centered on
  the host transform's Y position.
- 2D bodies remain plane-constrained: planar impulse can move them in X/Z,
  vertical impulse treats them as having infinite constrained mass.
- mixed broad phase uses GridForge-backed spatial identity, separate 2D and 3D
  collider ID spaces, awake-dynamic gating, layer filtering, same-agent and
  explicit hierarchy exclusion, and retained empty-partition cleanup.
- cross-dimensional hierarchy uses dimension-tagged collider keys in the shared
  hierarchy state. Plain collider IDs must not be compared across 2D and 3D
  services because those ID spaces are intentionally separate.
- unsupported mixed shape pairs must be explicit, tested, and documented.

Until mixed pair ownership and response land, pure 2D and 3D collision dispatch
should not fall through to accidental mixed response behavior.
