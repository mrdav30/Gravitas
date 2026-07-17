# Cone Collider And Query Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-class deterministic 3D cone primitive and cone-volume
query support without relying on high-triangle mesh approximations.

**Architecture:** Model the cone as an analytic convex primitive with
deterministic support, bounds, mass properties, collision contacts, CCD/query
reducers, serialization, diagnostics, and docs. Build cone-volume queries as
query primitives over fixed-point cone geometry rather than as temporary mesh
colliders.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet for reducer/query cost,
FixedMathSharp `Fixed64`/`Vector3d`/`FixedQuaternion`/`Fixed3x3`,
SwiftCollections buffers, Gravitas 3D collision/query/CCD/mixed services,
Chronicler explicit recording.

---

**Date:** 2026-06-26  
**Status:** Done  
**Completed:** 2026-06-28  
**Owner:** Gravitas 3D collider and query hardening

## Completion Notes

- `LSConeCollider` is an analytic 3D primitive with deterministic bounds, COM,
  inertia, serialization, diagnostics, collision, CCD, and query integration.
- Cone-volume queries use apex, direction, length, and end-radius inputs, write
  all-hit results into caller-owned buffers, and avoid temporary mesh
  generation.
- Mixed `SweepCircleAgainst3D` treats vertical finite-cone slabs as exact. When
  the 3D cone is rotated relative to the horizontal mixed slab, the reducer uses
  a safe whole-cone projection and reports
  `PhysicsQueryReducerKind.ConservativeFallback` rather than claiming exact slab
  clipping.

## Purpose

A cone is a useful gameplay and simulation primitive: field-of-view volumes,
area abilities, directional effects, sensor zones, and physical cone-shaped
objects can all be represented more cleanly by an analytic cone than by a
high-triangle mesh. A mesh can approximate a cone, but a first-class primitive
can provide better bounds, query pruning, mass properties, contact normals, and
end-user ergonomics.

This plan treats cone support as a serious physics feature. It should not be a
mesh shortcut with a friendlier constructor. If Gravitas exposes
`LSConeCollider` or cone queries, the shape needs deterministic fixed-point
geometry, stable ordering, clear reducer policy, tests, and benchmark evidence.

## Current Baseline

- 3D runtime collider families are sphere, cuboid, capsule, finite cylinder,
  mesh, and compound.
- 3D convex source sweeps already use support mappings for sphere, capsule,
  cuboid, cylinder, convex mesh, and triangle targets in
  `ConvexSweepQueryWorker`.
- Discrete 3D collision still relies heavily on shape-specific narrow-phase and
  contact generation.
- Mixed finite-slab reducers include shape-specific support for existing 3D
  primitive target families.
- Query services expose raycast, swept-sphere, primitive source sweeps, convex
  mesh source sweeps, compound source sweeps, and X/Z overlap/proximity queries.
- No public cone collider or cone-volume query API exists.

## Non-Goals

- Do not approximate cone physics by generating a runtime triangle fan mesh.
- Do not add cone support only for queries while leaving collider/CCD behavior
  misleadingly partial.
- Do not support concave cone-like sources. Hosts should continue to use
  compound convex parts for concave-looking movers.
- Do not use floating-point trigonometry in deterministic runtime logic.
- Do not make cone-volume queries depend on engine camera, renderer, or
  gameplay-team concepts.

## Guiding Rules

- Keep cone geometry analytic in runtime paths.
- Prefer radius/height/direction representations over angle/trigonometric APIs
  where that improves deterministic fixed-point behavior.
- If exact contact generation requires a reusable convex-support contact
  primitive, extract it deliberately instead of adding fragile cone-only cases.
- Distinguish physical `LSConeCollider` behavior from non-physical cone-volume
  queries.
- Benchmark dense cone query and contact paths before accepting a support-map
  design as release-quality.

## Proposed API Shape

- Add `ColliderType.Cone`.
- Add `ColliderShapeDefinitionKind.Cone`.
- Add `LSConeCollider`.
- Add `ColliderShapeDefinition.Cone(Fixed64 radius, Fixed64 height)`.
- Local cone convention:
  - local axis follows local Y.
  - base cap center is at local `Y = -height / 2`.
  - apex is at local `Y = height / 2`.
  - radius is the base radius.
  - runtime rotation orients the cone axis.
  - shape-derived center of mass is offset along the cone axis and should flow
    through the existing COM/inertia model.
- Add a query-only cone volume type or overload family using deterministic
  fixed-point inputs:
  - apex or center/axis form must be selected during Workstream 1.
  - the preferred query input is likely origin, normalized direction, length,
    and end radius rather than half-angle.

## Workstream 1: Geometry Contract And API Decision

**Problem**

Cone geometry has several reasonable coordinate conventions. The runtime must
choose one and document it before tests and reducers are written.

**Tasks**

- [x] Add design tests that lock the local geometry convention:
  - base center.
  - apex.
  - center/bounds behavior.
  - transformed axis.
  - shape-derived COM offset direction.
- [x] Decide and document whether `LSConeCollider` local origin represents the
      bounding-volume center or another point. Prefer bounding-volume center for
      consistency with cylinder/capsule bounds, while storing COM offset
      separately.
- [x] Decide cone query input form:
  - `origin + direction + length + endRadius`.
  - or `center + rotation + radius + height`. Prefer the first for gameplay
    queries such as directional effects.
- [x] Add `ColliderType.Cone`.
- [x] Add `ColliderShapeDefinitionKind.Cone`.
- [x] Add shape-definition tests for radius/height validation and runtime
      collider materialization.
- [x] Add `LSConeCollider` skeleton with radius, height, base center, apex,
      axis, bounds, and support-point methods.
- [x] Add debug draw command/view coverage if cone visualization needs a new
      diagnostic primitive. Wireframe triangle-fan debug drawing is acceptable
      for visualization only.

**Done Criteria**

- Cone local geometry is unambiguous and tested.
- Query cone input semantics are selected before reducers are implemented.
- Runtime cone data does not depend on generated mesh triangles.

## Workstream 2: Mass, Inertia, Bounds, And Serialization

**Problem**

A physical cone has asymmetric center of mass and inertia. Gravitas should use
that as a strength rather than hiding it behind a mesh approximation.

**Tasks**

- [x] Derive and document deterministic fixed-point formulas for:
  - cone volume.
  - center-of-mass offset from the local origin.
  - principal inertia about the cone's local axis.
  - perpendicular principal inertia about the cone COM.
- [x] Add formula tests using fixed-point expected values and scale
      relationships.
- [x] Implement cone bounds rebuild for rotated cones without underestimating.
- [x] Implement shape-derived mass properties in `LSConeCollider`.
- [x] Ensure `SolidBody` receives the cone COM offset and full inertia tensor
      correctly.
- [x] Update `ColliderShapeSnapshot` if cone runtime state needs snapshot
      coverage.
- [x] Add Chronicler recording for cone shape state.
- [x] Add serialization replay tests for cone colliders.

**Done Criteria**

- Cone mass properties are physically explainable and deterministic.
- Bounds are conservative and tight enough for partition/query performance.
- Save/load preserves cone shape and body mass-property continuation.

## Workstream 3: Cone Volume Query API

**Problem**

Gameplay often needs a cone query without creating a collider, such as a cone of
frost or directional sensor. This query should be deterministic, explicit, and
allocation-conscious.

**Tasks**

- [x] Add public API tests for closest-hit and all-hit cone-volume queries:
  - sphere target.
  - cuboid target.
  - capsule target.
  - cylinder target.
  - convex mesh target.
  - compound target.
  - trigger filtering.
  - layer include-mask filtering.
  - deterministic hit ordering by distance and collider identity.
- [x] Add query argument validation:
  - direction must be non-zero.
  - length must be positive.
  - end radius must be positive or non-negative according to the selected query
    contract.
- [x] Implement broad candidate bounds for finite cone volume.
- [x] Implement exact or conservative-without-false-negative reducers per target
      family. Any conservative accepted hit must be explicitly labeled or
      documented if the public hit type is extended.
- [x] Add caller-owned all-hit buffer overloads.
- [x] Add allocation tests proving repeated cone queries allocate `0` bytes
      after warmup.
- [x] Add benchmark rows for dense cone-volume queries.

**Done Criteria**

- Users can perform deterministic cone-volume queries without authoring mesh
  colliders.
- Query ordering and filtering match existing `Query3D` expectations.
- Dense cone queries have measured cost.

## Workstream 4: Discrete Collision And Contact Quality

**Problem**

Physical cone colliders need stable contacts against existing 3D shapes. This is
the highest-risk part of the plan because cones combine a curved side, base cap,
apex, and asymmetric mass properties.

**Tasks**

- [x] Add discrete collision tests for cone against:
  - sphere.
  - cuboid.
  - capsule.
  - cylinder.
  - cone.
  - convex mesh.
  - compound.
- [x] Add contact-quality tests for:
  - base-cap resting.
  - side-surface contact.
  - apex contact.
  - shallow grazing contact.
  - rotated cone contact.
  - deep overlap fallback normal.
  - stable resting pair warm-start.
- [x] Evaluate two implementation routes with focused prototypes and tests:
  - analytic pair-specific reducers for cone against each existing primitive.
  - reusable deterministic convex-support contact generation for convex
    primitives, with cone as the first new consumer.
- [x] Choose the route that gives better deterministic contact quality and
      maintainable shape expansion.
- [x] Implement the chosen narrow-phase path without adding runtime mesh
      approximation.
- [x] Ensure contact generation produces stable manifold ordering for response
      and warm start.
- [x] Add regression tests proving existing non-cone shape pairs are unchanged.

**Done Criteria**

- Cone discrete contacts are stable enough for resting response.
- The implementation path is justified by tests and, where needed, benchmark
  signal.
- Adding a cone does not degrade existing primitive contact behavior.

## Workstream 5: Swept Queries, CCD, And Mixed Reducers

**Problem**

Cone colliders should participate in the same movement and query quality bar as
other convex primitives.

**Tasks**

- [x] Add cone support to `ConvexSweepQueryWorker` support mappings.
- [x] Add swept source tests:
  - cone source against sphere.
  - cone source against cuboid.
  - cone source against cylinder/capsule.
  - cone source against convex mesh.
  - cone source against compound.
- [x] Add cone target tests for swept sphere and supported primitive source
      sweeps.
- [x] Add CCD tests for dynamic cone movers and cone targets.
- [x] Add rotational CCD tests for fast-spinning cone edge/apex cases where
      existing rotational bounds need cone participation.
- [x] Extend mixed finite-slab reducers for cone targets in
      `SweepCircleAgainst3D` and mixed collision where the cone is a 3D
      participant.
- [x] Add mixed CCD tests for 3D cone bodies interacting with embedded 2D slabs.
- [x] Add benchmark rows if cone support mapping or mixed reducers show dense
      candidate cost.

**Done Criteria**

- Cone source and target sweeps behave consistently with other convex
  primitives.
- CCD does not silently fall back to sphere-only behavior for cones.
- Mixed mode treats cone targets as first-class 3D shapes.

## Workstream 6: Docs, Diagnostics, Benchmarks, And Release Validation

**Problem**

Cone support spans public shape APIs, query APIs, collision behavior, mixed
mode, and diagnostics. The docs need to make that surface easy to understand.

**Tasks**

- [x] Update `docs/wiki/DIMENSIONS.md` with cone shape support.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` with cone contact and CCD policy.
- [x] Update `docs/wiki/QUERY_SERVICES.md` with cone-volume query and cone sweep
      coverage.
- [x] Update `docs/wiki/SERIALIZATION.md` with cone shape state.
- [x] Update `docs/wiki/DIAGNOSTICS.md` and diagnostic adapters if cone debug
      draw support is added.
- [x] Add benchmark selections for cone volume queries and cone collision
      scaling where measured value exists.
- [x] Run:
  - `dotnet build Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet build Gravitas.slnx --configuration ReleaseLean`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- Docs describe cone collider and cone query behavior without implying mesh
  approximation.
- Diagnostics can visualize cones or cone query volumes when enabled.
- Release and Lean validations pass.

## Final Done Criteria

- `LSConeCollider` is a first-class 3D primitive across mass properties, bounds,
  serialization, collision, query, CCD, mixed mode, and docs.
- Cone-volume queries support gameplay-style directional effects without
  requiring temporary collider creation.
- Runtime cone behavior is analytic, deterministic, and benchmark-informed.
- Existing mesh and compound policies remain intact.
