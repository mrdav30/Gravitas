# Pure 2D Capsule And Convenience Shapes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-class deterministic pure 2D capsule collider and lightweight triangle authoring helpers that materialize as existing convex polygons.

**Architecture:** Treat the 2D capsule as a real primitive because it is common in character-style physics and should not depend on polygon approximation. Treat 2D triangles as convenience input for `LSPolygonCollider2D`/`ColliderShapeDefinition2D.ConvexPolygon` so the runtime shape model stays simple.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet where dense primitive-pair cost needs evidence, FixedMathSharp `Vector2d`/`Fixed64`, SwiftCollections buffers, Gravitas 2D collision/query/CCD/mixed services, Chronicler explicit recording.

---

**Date:** 2026-06-26  
**Status:** Done  
**Completed:** 2026-06-27  
**Owner:** Gravitas pure 2D collider hardening

## Purpose

Pure 2D currently supports circles, axis-aligned boxes, convex polygons, and
compounds. That is enough for many scenes, but capsules are a first-class shape
in platformer and character physics because they move smoothly over edges,
avoid snagging on small corners, and pair naturally with 2D grounding/support.

Approximating capsules with convex polygons is possible, but it creates more
vertices, poorer contact normals, less predictable mass properties, and extra
query/CCD work. A specialized deterministic capsule primitive should be cheaper
and clearer for users.

Triangles do not need a new runtime shape. A triangle is already a convex
polygon with three vertices, so Gravitas should expose triangle convenience
helpers while keeping the runtime `ColliderType2D.ConvexPolygon` model.

## Current Baseline

- `ColliderType2D` contains `Circle`, `AABox`, `ConvexPolygon`, and `Compound`.
- `ColliderShapeDefinition2DKind` contains `Circle`, `AABBox`, and
  `ConvexPolygon`.
- `LSPolygonCollider2D` validates convex polygons and handles triangle input
  today if callers provide three valid vertices.
- `CollisionDetection2D`, `QueryDetection2D`, and pure 2D CCD already have
  shape-family dispatch over circle, AABB, convex polygon, and compound.
- Mixed 2D/3D collision and mixed swept queries support embedded 2D circle,
  AABB, convex polygon, and compound slabs.

## Non-Goals

- Do not add a separate `Triangle` runtime shape, collider type, pair family, or
  query reducer.
- Do not add a zero-width solid segment collider. Segment behavior should remain
  query/sensor-style unless a future measured need appears.
- Do not approximate the capsule internally with a polygon fan.
- Do not skip mixed 2D/3D support for the capsule. Any first-class 2D collider
  should participate in mixed mode unless explicitly rejected with tests and
  docs.

## Guiding Rules

- Keep capsule geometry analytic and deterministic.
- Preserve stable contact ordering and pair-owned warm-start behavior.
- Keep compound behavior owner-collapsed and stable by authored part order.
- Make the shape-derived center of mass and moment of inertia physically
  explainable.
- Avoid introducing duplicate public APIs for triangles.

## Proposed API Shape

- Add `ColliderType2D.Capsule`.
- Add `ColliderShapeDefinition2DKind.Capsule`.
- Add `LSCapsuleCollider2D`.
- Add `ColliderShapeDefinition2D.Capsule(Fixed64 radius, Fixed64 height)`.
- Add triangle convenience helpers:
  - `ColliderShapeDefinition2D.Triangle(Vector2d a, Vector2d b, Vector2d c)`
  - optional `LSPolygonCollider2D.CreateTriangle(...)` if it fits local style.
- Capsule local convention:
  - local shape axis follows local 2D Y.
  - `radius` is the semicircle radius.
  - `height` is full end-to-end height including caps.
  - `height >= radius * 2`.
  - body/collider rotation rotates the local capsule axis in the X/Z plane.

## Workstream 1: Shape API, State, And Authoring Definitions

**Problem**

The new primitive must fit the existing collider and shape-definition surfaces
without creating a second way to represent triangles.

**Tasks**

- [x] Add tests for `ColliderShapeDefinition2D.Capsule(...)`:
  - rejects non-positive radius.
  - rejects height smaller than diameter.
  - creates an `LSCapsuleCollider2D`.
  - preserves radius and height in shape equality/hash behavior.
- [x] Add tests for triangle convenience helpers:
  - valid triangle creates `ColliderShapeDefinition2DKind.ConvexPolygon`.
  - materialized runtime collider is `LSPolygonCollider2D`.
  - invalid or collinear triangle input is rejected by the same polygon
    validation rules as other convex polygons.
- [x] Add `Capsule` to `ColliderType2D`.
- [x] Add `Capsule` to `ColliderShapeDefinition2DKind`.
- [x] Implement `LSCapsuleCollider2D` under `src/Gravitas/Colliders/2D`.
- [x] Add `Radius`, `Height`, `ScaledRadius`, `SegmentStart`, `SegmentEnd`,
  and capsule bounds rebuild state.
- [x] Ensure `LocalOffset`, local scale, compound-local transform, and body
  rotation rebuild the capsule segment deterministically.
- [x] Update `ColliderSettings2D` priority ordering. The capsule should sort
  near circle/convex shapes without destabilizing existing pair priority.

**Done Criteria**

- Capsule definitions materialize as first-class runtime colliders.
- Triangle helpers produce convex polygon definitions rather than a new runtime
  type.
- Bounds, center, local offset, scale, and rotation are deterministic.

## Workstream 2: Mass Properties And Serialization

**Problem**

Capsules need shape-derived area, center of mass, and scalar moment of inertia
so `SolidBody2D` response remains physically explainable.

**Tasks**

- [x] Add tests for capsule area and mass properties:
  - area is rectangle segment area plus circular cap area.
  - center of mass is local offset when the capsule is symmetric.
  - moment of inertia scales with mass, radius, and height.
  - equal height and diameter behaves consistently with a circle-like capsule.
- [x] Implement `CalculateArea()`, `CalculateLocalCenterOfMassOffset()`, and
  `CalculateMomentOfInertia(...)` for `LSCapsuleCollider2D`.
- [x] Add explicit comments or docs for any fixed-point constants used in the
  inertia formula.
- [x] Update `ColliderShapeSnapshot2D` if capsule shape state needs snapshot
  coverage.
- [x] Update `LSCapsuleCollider2D.RecordData(...)` and shape definition
  recording paths.
- [x] Add JSON/MemoryPack-compatible save/populate tests for capsule colliders.

**Done Criteria**

- Capsule mass properties are deterministic and covered by formula tests.
- Serialization preserves capsule shape state and runtime continuation.
- Shape-derived COM integrates with the existing `SolidBody2D` mass-property
  pipeline.

## Workstream 3: 2D Collision Detection And Contact Manifolds

**Problem**

The capsule must collide with every existing pure 2D collider family and produce
stable contacts for warm-started response.

**Tasks**

- [x] Add collision tests for capsule against:
  - circle.
  - AABB.
  - convex polygon.
  - capsule.
  - compound targets containing supported parts.
- [x] Add edge-case tests:
  - end-cap contact.
  - side contact.
  - parallel capsule/capsule contact.
  - rotated capsule contact.
  - tangent/no-contact boundary.
  - deeply overlapping centers with deterministic fallback normal.
- [x] Add capsule dispatch to `CollisionDetection2D`.
- [x] Implement closest-segment and inflated-convex reducers needed for capsule
  contacts.
- [x] Ensure `ContactManifold2D` contact IDs remain stable when a capsule rests
  on a flat surface.
- [x] Add response tests proving capsule contacts warm start and apply friction
  like existing two-contact manifolds.

**Done Criteria**

- Capsule participates in pure 2D discrete contacts with stable normals,
  depths, and contact identities.
- Existing circle/AABB/polygon/compound behavior is unchanged.
- Warm-started resting capsule contacts are stable.

## Workstream 4: Query, CCD, And Grounding Support

**Problem**

First-class collider support requires the query and continuous-collision
surfaces to recognize capsules. It also needs to pair cleanly with the planned
2D grounding/support work.

**Tasks**

- [x] Add query tests for capsule targets:
  - overlap circle.
  - overlap AABB.
  - overlap convex polygon.
  - raycast.
  - swept circle.
  - all-hit deterministic ordering.
- [x] Add query tests for capsule sources where pure 2D CCD performs exact
  mover-shape validation.
- [x] Add capsule support to `QueryDetection2D`.
- [x] Add capsule mover/target support to pure 2D CCD exact validation.
- [x] Add allocation tests for repeated capsule raycast/sweep/overlap queries
  after warmup.
- [x] Add grounding/support tests once the 2D grounding plan is implemented:
  - capsule body grounds from side/foot contacts using correct planar normal.
  - capsule ground probe radius can derive from capsule radius.

**Done Criteria**

- Capsule queries and CCD validation are exact for supported 2D paths.
- Grounding/support can treat capsules as first-class character shapes.
- Query paths remain allocation-free after warmup.

## Workstream 5: Mixed 2D/3D Capsule Slab Semantics

**Problem**

Mixed mode embeds 2D colliders as finite slabs/prisms. Adding a 2D capsule
without mixed support would create a dimensional parity hole.

**Tasks**

- [x] Add mixed collision tests for 3D primitives against embedded 2D capsule
  slabs:
  - sphere.
  - cuboid.
  - capsule.
  - cylinder.
  - convex mesh where existing mixed reducers support the target/source family.
- [x] Add `SweepSphereAgainst2D` tests for capsule slab targets.
- [x] Add mixed CCD tests for 3D sphere/capsule sources against 2D capsule
  targets where existing source policy supports exact reducers.
- [x] Extend mixed bounds generation for `LSCapsuleCollider2D`.
- [x] Extend `CollisionDetectionMixed` to handle capsule slab contact points and
  normals.
- [x] Extend `GravitasQueryMixedService` sphere-against-2D reducers for capsule
  slabs.
- [x] Ensure compound 2D colliders containing capsules preserve owner identity
  and stable part ordering.

**Done Criteria**

- Mixed mode does not reject or silently approximate first-class 2D capsules.
- Mixed query/CCD diagnostics label capsule reducers consistently.
- Compound capsule parts behave like other supported 2D parts.

## Workstream 6: Docs, Benchmarks, And Release Validation

**Problem**

Adding a primitive expands the public shape matrix. Docs, benchmarks, and tests
need to make the support surface obvious.

**Tasks**

- [x] Update `docs/wiki/DIMENSIONS.md` with the new 2D shape family and
  triangle convenience behavior.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` with capsule contact support.
- [x] Update `docs/wiki/QUERY_SERVICES.md` with capsule query/CCD coverage.
- [x] Update `docs/wiki/SERIALIZATION.md` with capsule shape state.
- [x] Add benchmark rows only where useful:
  - dense capsule/capsule response.
  - capsule query sweep.
  - mixed capsule slab query if reducer cost is non-trivial.
- [x] Run:
  - `dotnet build Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet build Gravitas.slnx --configuration ReleaseLean`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- Public docs describe capsule support and triangle convenience helpers.
- Release and Lean validations pass.
- Any meaningful capsule hot-path cost has benchmark coverage.

## Final Done Criteria

- `LSCapsuleCollider2D` is a first-class pure 2D primitive across collision,
  response, query, CCD, mixed mode, serialization, and docs.
- Triangle helpers exist only as authoring convenience for convex polygons.
- No zero-width solid segment collider is added.
- Existing 2D shape behavior remains deterministic and allocation-conscious.
