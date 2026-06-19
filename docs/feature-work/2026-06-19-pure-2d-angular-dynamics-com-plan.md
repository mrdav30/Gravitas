# Pure 2D Angular Dynamics And Center-Of-Mass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give pure 2D bodies the same physically meaningful mass-property boundary as 3D bodies: explicit center of mass, scalar angular inertia, torque, angular impulses, and off-center contact response.

**Architecture:** `StiffBody2D` owns runtime body state and effective solver mobility policy; `LSCollider2D` and primitive/compound subclasses derive geometry mass properties. Pure 2D response consumes body-owned effective mass/moment helpers and keeps X/Z planar physics explicit. Mixed 2D/3D angular effects are handled only after the pure 2D model is stable.

**Tech Stack:** C# 11, FixedMathSharp `Fixed64`/`Vector2d`, Chronicler `IRecordable`, SwiftCollections hot-path buffers, xUnit v3, FluentAssertions, BenchmarkDotNet.

---

**Date:** 2026-06-19
**Status:** Workstreams 1-4 implemented / Workstream 5 ready
**Owner:** Gravitas runtime/collision hardening

## Purpose

Pure 2D currently supports deterministic planar translation, scalar yaw
publishing, broad phase, narrow phase, queries, CCD, mixed embedding, and simple
linear collision response. It does not yet model center-of-mass offsets, scalar
moment of inertia, torque, angular impulses, or angular collision response.

That limitation is acceptable only as an alpha gap. A first-class deterministic
2D physics engine needs off-center impulses to spin bodies, compound and polygon
centroids to affect response, and serialization/replay coverage for angular
state. This plan captures the dedicated 2D work so it does not get folded into
the 3D mass/inertia workstream as a weak copy of the 3D tensor model.

## Current Baseline

- `StiffBody2D` owns X/Z-projected position, scalar yaw rotation, linear
  velocity, force integration, sleep/wake state, pure 2D CCD, mixed CCD source
  handling, visualization publishing, and Chronicler state.
- `StiffBody2D.Mass` and `InverseMass` are scalar, and Workstream 1 added
  scalar moment/inverse moment state plus effective inverse mass/moment helpers.
- `StiffBody2D.CanTranslate` and `CanRotate` now separate linear and angular
  solver mobility policy.
- `LSCollider2D` owns shape state, bounds, local offset, mixed embedding, query
  state, partition state, pair state, and hierarchy state. It does not expose
  center-of-mass or moment-of-inertia APIs.
- `LSCircleCollider2D`, `LSAABBoxCollider2D`, `LSPolygonCollider2D`, and
  `LSCompoundCollider2D` have enough deterministic geometry to derive area,
  centroid, and scalar inertia.
- `CollisionPair2D` resolves collision directly in the pair using inverse mass
  only. It applies penetration correction and a normal impulse, but no contact
  arm, angular denominator, torque, or friction impulse.
- `CollisionResponse2D` is only a constants holder. It can become the pure 2D
  response solver surface without preserving the pair-local implementation.
- `CollisionResponseMixed` projects impulses into the 2D plane, but currently
  applies only 2D linear velocity deltas.

## Design Decisions

- Pure 2D COM is a body-local `Vector2d` in the same X/Z plane as
  `StiffBody2D.Position`.
- `StiffBody2D.WorldCenterOfMass` is
  `Position + Vector2d.Rotate(LocalCenterOfMassOffset, Rotation)`.
- Pure 2D inertia is a scalar moment around the body yaw axis, not a tensor.
- Shape APIs derive moment about an explicit requested body-local reference
  point so callers cannot confuse collider center, body origin, and COM.
- `CanMove` was replaced by explicit `CanTranslate` in runtime code. Do not
  restore a duplicate public mobility alias; the project is pre-alpha and API
  clarity wins.
- `CanRotate`, `EffectiveInverseMass`, and `EffectiveInverseMomentOfInertia`
  belong on `StiffBody2D`, mirroring the 3D effective mass policy while keeping
  the scalar 2D model simple.
- Positive angular velocity uses the same orientation as `Vector2d.Rotate`.
  Point velocity from angular motion is:

```csharp
private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
    new Vector2d(-relativePoint.Y, relativePoint.X) * angularVelocity;
```

- Normal impulse denominator for 2D contact response is:

```csharp
Fixed64 angularA = Vector2d.CrossProduct(relativeA, normal);
Fixed64 angularB = Vector2d.CrossProduct(relativeB, normal);
Fixed64 denominator =
    inverseMassA
    + inverseMassB
    + angularA * angularA * inverseMomentA
    + angularB * angularB * inverseMomentB;
```

- Mixed 2D/3D angular response is a separate phase of this plan. The pure 2D
  solver must pass before mixed applies planar impulses around the 2D COM.

## File Map

- Modify `src/Gravitas/Core/StiffBody2D.cs`
  - Add COM, scalar moment, angular velocity, torque, angular sleep, effective
    mass/moment helpers, torque/impulse APIs, serialization fields, and replay
    restore behavior.
- Modify `src/Gravitas/Colliders/Primitives2D/LSCollider2D.cs`
  - Add the base 2D mass-property API and deterministic parallel-axis helper.
- Modify `src/Gravitas/Colliders/Primitives2D/LSCircleCollider2D.cs`
  - Add circle area, local COM, and scalar moment formulas.
- Modify `src/Gravitas/Colliders/Primitives2D/LSAABBoxCollider2D.cs`
  - Add rectangle area, local COM, and scalar moment formulas.
- Modify `src/Gravitas/Colliders/Primitives2D/LSPolygonCollider2D.cs`
  - Add deterministic convex polygon area, centroid, and polar moment formulas
    over scaled local vertices.
- Modify `src/Gravitas/Colliders/Primitives2D/LSCompoundCollider2D.cs`
  - Aggregate part area, COM, and scalar moment in stable part order.
- Modify `src/Gravitas/CollisionHandling/Pairs/CollisionPair2D.cs`
  - Move response math out of the pair and call `CollisionResponse2D.Resolve`.
- Modify `src/Gravitas/CollisionHandling/Response/CollisionResponse2D.cs`
  - Become the pure 2D normal/friction impulse solver.
- Modify `src/Gravitas/CollisionHandling/Response/CollisionResponseMixed.cs`
  - Add 2D angular effects only in the mixed workstream.
- Create `tests/Gravitas.Tests/Core/StiffBody2DMassPropertiesTests.cs`
- Create `tests/Gravitas.Tests/Colliders/Collider2DMassPropertyTests.cs`
- Create `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DAngularTests.cs`
- Modify `tests/Gravitas.Tests/Serialization/StiffBody2DSerializationTests.cs`
- Modify `docs/wiki/DIMENSIONS.md`
- Modify `docs/wiki/COLLISION_PIPELINE.md`
- Modify `docs/wiki/SERIALIZATION.md`
- Modify or extend `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs`
  when response hot-path cost changes measurably.

## Workstream 1: Body Mobility, COM, And Scalar Inertia

**Goal:** Make `StiffBody2D` own explicit body mass properties and effective
2D solver policy before collision response consumes them.

Tasks:

- [x] Add failing tests in
  `tests/Gravitas.Tests/Core/StiffBody2DMassPropertiesTests.cs` for movable,
  kinematic, immovable, zero-mass, and angular-disabled bodies.
- [x] Add failing tests for `LocalCenterOfMassOffset`, `WorldCenterOfMass`, and
  `ResetCenterOfMassFromCollider`.
- [x] Replace `CanMove` in `StiffBody2D` with explicit `CanTranslate` and
  `CanRotate`. Update all pure 2D and mixed callers in the same change.
- [x] Convert `Mass` from an auto-property into a setter that refreshes scalar
  moment and inverse moment when shape mass properties are available.
- [x] Add explicit COM and scalar moment state to `StiffBody2D`.
- [x] Add `ResetCenterOfMassFromCollider()` and
  `RefreshMassPropertiesFromColliderShape()` to derive default COM and moment
  from the bound collider.
- [x] Run focused body tests.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~StiffBody2DMassPropertiesTests
```

Expected after implementation: all new body mass-property tests pass.

**Progress 2026-06-19:** Workstream 1 added body-owned
`LocalCenterOfMassOffset`, `WorldCenterOfMass`, scalar moment/inverse moment,
`PreventAngularForces`, `CanTranslate`, `CanRotate`, `EffectiveInverseMass`,
and `EffectiveInverseMomentOfInertia`. Pure 2D and mixed response now consume
the 2D effective inverse mass surface instead of the removed `CanMove` alias.
`Mass`, collider shape changes, and COM overrides refresh scalar mass
properties, and Chronicler records the new body-owned COM/angular-policy state.
Focused coverage lives in
`tests/Gravitas.Tests/Core/StiffBody2DMassPropertiesTests.cs` and
`tests/Gravitas.Tests/Serialization/StiffBody2DSerializationTests.cs`.
The shared `LSCollider2D` mass-property surface and current shape formula
implementations were pulled forward to avoid placeholder body inertia.

## Workstream 2: 2D Collider Mass-Property API

**Goal:** Let every pure 2D collider compute deterministic local COM, area, and
scalar moment about an explicit body-local reference point.

Tasks:

- [x] Add failing tests in
  `tests/Gravitas.Tests/Colliders/Collider2DMassPropertyTests.cs` for circle,
  AABB, convex polygon, and compound COM/moment.
- [x] Add the base mass-property surface to `LSCollider2D`.
- [x] Implement circle mass properties in `LSCircleCollider2D`.
- [x] Implement AABB mass properties in `LSAABBoxCollider2D`.
- [x] Implement convex polygon area, centroid, and moment using scaled local
  vertices plus `ScaledLocalOffset`. Preserve declaration order and reject
  invalid polygons through the existing validation path.
- [x] Implement `LSCompoundCollider2D` aggregation in stable part order. Assign
  each part a mass proportional to `partArea / totalArea`, aggregate COM by
  area-weighted local COM, apply the owning collider's local offset, and
  aggregate moment by asking each part for moment about the compound COM.
- [x] Run focused collider mass-property tests.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~Collider2DMassPropertyTests
```

Expected after implementation: all circle, AABB, polygon, and compound
mass-property tests pass.

**Progress 2026-06-19:** Workstream 2 added focused collider coverage for
circle, AABB, convex polygon, and compound 2D mass properties. The current
`LSCollider2D` mass-property API now reports local COM, deterministic area, and
scalar moment about an explicit local reference point for all current pure 2D
shape types. Compound mass properties aggregate owned private parts in stable
part order, assign area-proportional part mass, honor the owning collider's
local offset, and apply authored part local scale and local rotation before
COM/moment aggregation. The same owner-center rule now drives 2D compound part
bounds so collision geometry and mass-property geometry use one coordinate
model.

## Workstream 3: Angular Integration, Sleep, And Serialization

**Goal:** Let hosts apply deterministic torque/angular impulses and replay the
same angular state after Chronicler populate.

Tasks:

- [x] Add failing tests for `AddTorque`, `AddAngularImpulse`, `LateSimulate`
  rotation, angular sleep threshold, and shape mutation refreshing moment.
- [x] Add angular state to `StiffBody2D`.
- [x] Add torque and angular impulse APIs.
- [x] Integrate angular velocity in `LateSimulate` before publishing rotation.
- [x] Update `Sleep()` and `UpdateSleepState()` so a body sleeps only when both
  linear and angular speed are at or below their thresholds.
- [x] Extend `RecordData` and `ApplyLoadedState` for angular state, COM state,
  and scalar moment inputs required for deterministic continuation.
- [x] Update `tests/Gravitas.Tests/Serialization/StiffBody2DSerializationTests.cs`
  so populate restores COM, angular velocity, queued torque acceleration, sleep
  thresholds, and replay continuation.
- [x] Run focused body and serialization tests.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~StiffBody2DAngularDynamicsTests|FullyQualifiedName~StiffBody2DSerializationTests"
```

Expected after implementation: angular integration and serialization tests pass
with identical replay state across uninterrupted and restored worlds.

**Progress 2026-06-19:** Workstream 3 added scalar angular velocity,
angular acceleration, angular speed, angular sleep threshold, `AddTorque`, and
`AddAngularImpulse` to `StiffBody2D`. `LateSimulate` now integrates queued
torque through `EffectiveInverseMomentOfInertia`, advances scalar yaw rotation,
and keeps sleep gated on both linear and angular speed. Sleep clears angular
runtime motion state. Chronicler now records 2D angular velocity, applied and
queued angular acceleration, angular speed, and angular sleep threshold so a
snapshot with queued torque can replay the next fixed step identically after
populate.

## Workstream 4: Pure 2D Angular Contact Response

**Goal:** Use COM-relative contact arms, scalar inverse moment, and contact
point velocity in pure 2D response.

Tasks:

- [x] Add failing tests in
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DAngularTests.cs`
  for off-center contacts, centered contacts, angular-disabled bodies,
  kinematic/immovable bodies, and friction.
- [x] Move response math from `CollisionPair2D.Resolve` into
  `CollisionResponse2D.Resolve(CollisionPair2D pair, Contact2D contact)`.
  `CollisionPair2D.MarkColliding` should call the response surface and retain
  pair notification/wake ownership.
- [x] Build solver body values from `StiffBody2D.EffectiveInverseMass`,
  `StiffBody2D.EffectiveInverseMomentOfInertia`, and
  `StiffBody2D.WorldCenterOfMass`.
- [x] Compute contact point velocity from linear and angular velocity.
- [x] Apply normal impulse to linear and angular velocity. The impulse applied
  to body A is `-impulse`, and the impulse applied to body B is `impulse`.
- [x] Add tangent friction impulse after normal impulse using the same
  coefficient rule as 3D response: geometric mean for two bodies, single-body
  coefficient when one side is bodyless/static, and clamp by
  `normalImpulse * frictionCoefficient`.
- [x] Keep positional correction translation-only for this workstream. Angular
  position correction requires a deeper manifold/solver iteration model and
  should not be smuggled into the first angular velocity response.
- [x] Run focused 2D response tests.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~CollisionResponse2DAngularTests
```

Expected after implementation: off-center contacts change angular velocity,
centered contacts do not inject spin, and disabled/kinematic/immovable angular
policy is honored.

**Progress 2026-06-19:** Workstream 4 moved pure 2D response ownership out of
`CollisionPair2D` and into `CollisionResponse2D.Resolve(...)`, leaving the pair
responsible for lifecycle, wake propagation, and notifications. Pure 2D contact
response now computes COM-relative contact arms, contact-point velocity from
linear plus scalar angular velocity, angular impulse denominators, normal
linear/angular velocity deltas, and tangent Coulomb friction impulses. Positional
correction remains translation-only by design. Focused coverage lives in
`tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DAngularTests.cs`.

## Workstream 5: Mixed 2D/3D Angular Semantics

**Goal:** Let mixed planar impulses spin 2D bodies only when that behavior is
physically explainable under the mixed embedding model.

Tasks:

- [ ] Add mixed response tests proving vertical-only 3D impulses do not spin or
  translate a 2D body, while planar impulses at an offset from 2D COM can change
  scalar 2D angular velocity.
- [ ] Update `CollisionResponseMixed` so the 2D participant uses
  `EffectiveInverseMass` and `EffectiveInverseMomentOfInertia`.
- [ ] Compute the 2D relative contact arm from the planar contact point to
  `StiffBody2D.WorldCenterOfMass`.
- [ ] Apply scalar 2D angular impulse only from the planar impulse component.
  The vertical Y impulse remains constrained out of the pure 2D body model.
- [ ] Preserve existing mixed rule: planar X/Z impulse can move the 2D body,
  vertical Y impulse treats the 2D participant as infinite constrained mass.
- [ ] Run focused mixed response tests plus existing mixed CCD/query tests.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Mixed|FullyQualifiedName~CollisionResponse2DAngularTests"
```

Expected after implementation: mixed angular behavior is explicit, planar-only,
and does not change pure 2D or pure 3D pair identity.

## Workstream 6: Docs, Benchmarks, And Release Validation

**Goal:** Make the new 2D body model understandable and prove the hot path did
not regress unexpectedly.

Tasks:

- [ ] Update `docs/wiki/DIMENSIONS.md` to state that pure 2D bodies own scalar
  angular dynamics around the yaw axis, with X/Z planar COM.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` to describe pure 2D normal and
  friction impulses, COM-relative contact arms, and remaining solver limits.
- [ ] Update `docs/wiki/SERIALIZATION.md` to list 2D COM, angular velocity,
  queued angular acceleration, moment policy, and sleep threshold state.
- [ ] Add or extend a 2D benchmark in
  `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs` if response
  benchmarks show a measurable cost from angular denominators or friction.
- [ ] Run the focused Release test project.

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release
```

- [ ] Run full standard and Lean validation before marking this plan complete.

```bash
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

Expected after implementation: Release and ReleaseLean builds/tests pass, and
docs no longer describe pure 2D response as linear-only.

## Exit Criteria

- Pure 2D has explicit body-owned COM, scalar inertia, and effective
  translate/rotate solver policy.
- Every shipped pure 2D collider can derive deterministic area, COM, and scalar
  moment about an explicit body-local reference point.
- Torque, angular impulse, angular velocity, angular sleep, and angular
  serialization are covered by tests.
- Pure 2D collision response uses COM-relative contact arms and angular impulse
  denominators.
- Mixed 2D/3D response either explicitly consumes the new 2D angular model or
  has a documented, tested reason for leaving mixed angular response disabled.
- Feature-work status, wiki docs, focused tests, full Release validation, and
  full ReleaseLean validation are updated before this plan moves to
  `docs/feature-work/done`.
