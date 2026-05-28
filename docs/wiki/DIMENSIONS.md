# Dimensions

Gravitas is moving from a 3D-only prototype toward first-class 2D, first-class
3D, and eventually mixed 2D/3D interaction. This page defines the current
contract so new code does not accidentally implement 2D as flattened 3D or make
pure 2D paths pay avoidable 3D costs.

## Current Status

The current runtime still only has production 3D primitive colliders and 3D
collision dispatch. Phase 9A/9B establishes the dimension contract and source
seams:

- `PhysicsDimension` identifies body and collider dimensionality.
- `StiffBody.Dimension` defaults to `ThreeD`, validates supported values, and
  cannot change after initialization.
- `LSCollider.Dimension` defaults to `ThreeD`; future 2D collider families
  should override it instead of pretending to be 3D colliders.
- body initialization rejects a body/collider dimension mismatch.
- `Physics2DBounds` maps pure 2D X/Y bounds into a deterministic
  `FixedBoundVolume` storage slab for current broad-phase infrastructure.

Pure 2D shapes, narrow phase, response, and query services are Phase 9C/9D
work. Mixed 2D/3D collision and impulse exchange are Phase 10 work.

## Dimension Types

`PhysicsDimension.TwoD` means a body or collider belongs to a pure 2D simulation
domain. It uses `Vector2d` X/Y coordinates, scalar rotation around the 2D normal,
2D mass/inertia rules, and 2D contact manifolds. It is not the same thing as a
3D body constrained to the X/Z ground plane.

`PhysicsDimension.ThreeD` means a body or collider belongs to the existing 3D
simulation domain. It uses `Vector3d`, `FixedQuaternion`, 3D bounds, 3D contact
manifolds, and the current y-up grounding model.

There is intentionally no `Mixed` body or collider dimension. Mixed behavior is
a policy between concrete 2D and 3D objects, not a third shape category.

## Pure 2D Coordinate Contract

Pure 2D code should use these rules:

- Authoritative 2D position is `Vector2d`.
- The axes are X/Y in the 2D plane.
- Rotation is scalar around the plane normal until a richer representation is
  proven necessary.
- Gravity is a `Vector2d` acceleration in pure 2D, not a 3D y-up grounding
  substitute.
- There is no `HeightPos`, ground probe, step offset, or grounded platform state
  in pure 2D body motion.
- Broad-phase storage may project a 2D area into a 3D slab, but that projection
  is storage metadata only. It is not a physical thickness contract.

The first Phase 9 shape set is circle, axis-aligned box, and convex polygon.
Concave 2D polygons should be rejected or preprocessed explicitly until a
deterministic decomposition/triangulation path has tests and benchmarks.

## Body And Collider Responsibilities

Body dimensionality and collider dimensionality must match before runtime
simulation begins. This avoids hidden shape conversions, such as using a 3D
sphere collider as a temporary 2D circle. Future 2D circle, box, and polygon
colliders should own 2D shape caches directly.

Shared body/collider responsibilities are allowed where they are truly shared:

- context binding.
- context-local IDs.
- lifecycle registration.
- layer/filter metadata.
- trigger/contact event surface.
- deterministic sleep/wake metadata.
- serialization hooks.

Dimension-specific responsibilities should stay dimension-specific:

- body motion state and integration.
- angular representation and inertia.
- bounds and broad-phase projection.
- narrow-phase pair dispatch.
- contact manifold generation.
- solver math and impulse application.
- query worker state.

This split keeps Gravitas from recreating Unity's separate Box2D/PhysX engines
while still avoiding one giant mode-flagged hot path.

## Broad Phase Bounds

The current broad phase is GridForge-backed and works with 3D voxel/world
coordinates. Pure 2D broad-phase work should start from `Physics2DBounds`:

```csharp
Physics2DBounds bounds = Physics2DBounds.FromMinMax(
    min,
    max,
    planeZ,
    halfThickness);

FixedBoundVolume storageVolume = bounds.ToFixedBoundVolume();
```

`Physics2DBounds.Area` stores the real pure 2D X/Y extents through
`BoundingArea`. `PlaneZ` and `HalfThickness` describe the deterministic slab
used to interact with current fixed broad-phase structures. The slab should not
be used as a physical contact depth. Phase 10 must define any actual finite
thickness used for mixed 2D/3D contact.

If a dedicated 2D broad-phase structure outperforms slab projection, it should
be added behind benchmarks and replay tests rather than hidden behind the same
3D partition loops.

## Queries

Pure 2D queries should be explicit 2D query APIs or workers:

- point/area overlap.
- segment or ray tests in X/Y.
- circle and AABB overlap.
- convex polygon overlap.

Do not route 2D query work through 3D raycast or X/Z circle workers unless a
benchmark proves the shared path is cheaper and the naming remains honest.

The existing `GravitasCircleQueryService` is an X/Z 3D-ground-plane proximity
query. It is not the Phase 9 pure 2D query contract.

## Serialization And Replay

Dimension is authoritative simulation state. It should be recorded with body
state and validated before a loaded body rejoins the simulation. A replay or
load path must not silently reinterpret a 3D body as 2D or a 2D body as 3D.

Replay tests for Phase 9D should compare:

- body position and velocity.
- collider bounds and broad-phase membership.
- contact manifold identity and ordering.
- response output.
- query result ordering.

## Mixed 2D/3D Boundary

Mixed 2D/3D is intentionally Phase 10. It must define:

- how a 2D plane is embedded in 3D space.
- whether 2D shapes have finite physical thickness.
- how 3D contact points project onto 2D manifolds.
- how impulses and positional correction exchange between 2D and 3D bodies.
- which shape pairs are supported, experimental, or rejected.

Until that contract exists, pure 2D and 3D collision dispatch should not fall
through to accidental mixed behavior.
