# Pure 2D Constraint And Ragdoll Foundation Plan

**Date:** 2026-07-02  
**Status:** Done  
**Owner:** Gravitas pure 2D constraint and articulated-body hardening

---

> **For agentic workers:** Treat this as a living context guide. Update progress
> as workstreams complete, and move genuinely deferred discoveries into their
> own plan or the evergreen trackers instead of leaving vague wiki caveats
> behind.

**Goal:** Add first-class deterministic pure 2D constraints, joints, and
ragdoll-style articulated bodies using native planar semantics instead of
projecting the 3D joint model onto one ignored axis.

**Architecture:** Introduce a context-owned `GravitasConstraint2DService`,
2D-native joint definitions, scalar/planar constraint rows, and integration into
the pure 2D contact island solver. Reuse proven 3D service patterns where they
fit, but keep `SolidBody2D`, scalar inertia, planar COM, 2D colliders, and
planar sleep/CCD semantics authoritative.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp
`Fixed64`/`Vector2d`, SwiftCollections buffers, Gravitas pure 2D physics,
collision, response, grounding/support, Chronicler record data, replay hashing,
and engine-agnostic diagnostics.

## Purpose

Pure 2D is now a first-class runtime path in Gravitas: it has `SolidBody2D`, 2D
colliders, scalar inertia, 2D manifolds, warm-started response, CCD,
grounding/support, mixed embedding, serialization, replay hashing, diagnostics,
and benchmarks. Articulated-body support is the major remaining 2D parity gap.

This plan adds native 2D constraints instead of treating them as a reduced 3D
problem. A 2D joint should speak in planar anchors, scalar angles, scalar
angular velocity, scalar inertia, and X/Z plane motion. A 2D ragdoll should be
authored over ordinary `SolidBody2D` links and `LSCollider2D` colliders, solving
inside the same deterministic 2D island graph as contacts.

## Current Baseline

- `SolidBody2D` owns planar position, scalar yaw, scalar angular velocity,
  scalar inertia, sleep/wake state, CCD displacement, and support state.
- `GravitasPhysics2DService` owns 2D body/collider registration, 2D collision
  pair processing, response/event processing, partition refresh, and grounding.
- Pure 2D contact response has manifold and warm-start support.
- At plan start, 3D constraints existed under `src/Gravitas/Constraints/3D`, but
  there was no `Constraints/2D` subsystem.
- Existing collider hierarchy keys can identify 2D colliders, but physical
  articulation should live in dedicated constraint types.

## Non-Goals

- Do not project `Joint3D` or `RagdollRuntime3D` into 2D by ignoring an axis.
- Do not implement engine animation, IK, pose blending, or renderer-facing bone
  systems in Gravitas.
- Do not make 2D joints depend on mixed 2D/3D embedding thickness.
- Do not use floating-point math or nondeterministic collection ordering.
- Do not force 2D and 3D to share abstractions when scalar/planar semantics need
  clearer dedicated types.

## Relationship To 3D Stress Plan

This plan should run after
`2026-07-02-3d-constraint-solver-stress-and-tuning-hardening-plan.md` unless a
pure 2D release blocker appears first. The 3D stress pass should inform:

- diagnostic counter naming.
- solver row ordering.
- warm-start invalidation policy.
- tuning API shape, if any.
- benchmark fixture design.

2D should reuse those lessons, not copy 3D formulas.

## Completion Evidence

Completed on 2026-07-03.

Implementation results:

- Added `GravitasConstraint2DService`, `Joint2D`, 2D joint definitions, planar
  local frames, scalar limits, scalar motors, and validated deterministic
  context-local joint registration.
- Added native 2D distance, pin/revolute, weld/fixed, and prismatic/slider
  constraint rows with warm-start caching, fixed-point scalar/planar effective
  mass, bounded motor/limit impulses, and `JointSolveMetrics2D`.
- Integrated enabled 2D joints into the pure 2D contact island solver so contact
  rows and joint rows solve in one deterministic island order.
- Added allocation-free linked-collider suppression for 2D joint/ragdoll self
  collisions in physical pair creation and 2D CCD source filtering.
- Added `RagdollDefinition2D`, `RagdollLinkDefinition2D`,
  `RagdollJointDefinition2D`, and `RagdollRuntime2D` over ordinary `SolidBody2D`
  links and `LSCollider2D` colliders.
- Added Chronicler `RecordData(...)`, replay hash contribution, diagnostic
  events, debug draw capture, tests, and benchmark smoke coverage for the pure
  2D articulated-body path.
- Addressed independent review findings before closure: ragdoll registration is
  validation-first and failure-atomic, direct/ragdoll/load paths share the same
  2D joint payload validation, zero-error angular rows and coincident positive
  distance targets now emit deterministic solver rows, and enabled-joint counts
  prevent inactive ragdolls from forcing the joint-island path in 2D and 3D.

Evidence captured during implementation:

- Focused constraint tests cover registration, invalid payloads, cross-context
  rejection, solver correction, freeze-axis behavior, deterministic order,
  linked wake state, ragdoll activation/filtering, serialization, 2D CCD linked
  filtering, diagnostics, and disabled diagnostics/filtering allocation
  guardrails.
- `tests/Gravitas.Benchmarks/Core/Constraint2DBenchmarks.cs` mirrors the 3D
  stress families for long chains, resting ragdolls, contact-heavy articulated
  stacks, motor-driven chains, linked filtering, inactive ragdolls, and
  activation toggles.
- Validation passed:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Constraint2D"`:
    25 passed.
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Constraint3D"`:
    29 passed.
  - `dotnet test Gravitas.slnx --configuration Release`: 951 passed.
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`: 933 passed.
  - `dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll constraint-2d --filter "*SimulateInactiveRagdoll*" --job Dry --warmupCount 1 --iterationCount 1`:
    benchmark smoke passed with no managed allocation reported.

Deferred work: none. Mixed-dimension articulated constraints remain outside the
current model by design; they are not a hidden gap in this pure 2D plan.

## Workstream 1: API Shape, Service Ownership, And Identity

**Status:** Done

**Problem**

Pure 2D constraints need context ownership, stable IDs, explicit registration,
and public names that make 2D semantics obvious.

**Tasks**

- [x] Add `src/Gravitas/Constraints/2D/GravitasConstraint2DService.cs` as the
      context-owned service.
- [x] Expose the service from `GravitasWorldContext` as `Constraints2D`.
- [x] Add `src/Gravitas/Constraints/2D/Joint2D.cs` for runtime joint state.
- [x] Add `src/Gravitas/Constraints/2D/JointDefinition2D.cs` for registration
      input.
- [x] Add `src/Gravitas/Constraints/2D/JointFrame2D.cs` using a local `Vector2d`
      anchor and scalar local angle.
- [x] Add `src/Gravitas/Constraints/2D/JointType2D.cs` with 2D-native joint
      kinds:
  - distance.
  - pin/revolute.
  - weld/fixed.
  - prismatic/slider.
  - rope or maximum-length constraint if tests prove it belongs in the first
    release surface.
- [x] Add `src/Gravitas/Constraints/2D/JointLimit2D.cs` for angular and slider
      limits.
- [x] Add `src/Gravitas/Constraints/2D/JointMotor2D.cs` for angular and linear
      motor payloads where supported by the joint type.
- [x] Reuse `JointCollisionPolicy` only if the shared namespace and semantics
      are already dimension-neutral; otherwise extract a shared policy file
      without changing the public name.
- [x] Add tests for:
  - empty service ownership.
  - deterministic monotonic joint IDs.
  - duplicate body pairs with distinct joint IDs.
  - invalid cross-context bodies.
  - invalid same-body joints.
  - invalid limits and motor values.

**Done Criteria**

- `GravitasWorldContext` owns a pure 2D constraint service.
- 2D joint registration is deterministic and validated before solver work.
- Public 2D types do not imply 3D frames or hidden axis projection.

## Workstream 2: 2D Constraint Rows And Joint Solver

**Status:** Done

**Problem**

The 2D solver needs planar linear rows and scalar angular rows that respect
`SolidBody2D` mass, planar COM, scalar inertia, and freeze axes.

**Tasks**

- [x] Add `src/Gravitas/Constraints/2D/JointConstraintRow2D.cs` for internal row
      state:
  - planar linear axis.
  - scalar angular contribution for body A.
  - scalar angular contribution for body B.
  - effective mass.
  - bias velocity.
  - accumulated impulse.
  - lower and upper impulse bounds.
- [x] Add `src/Gravitas/Constraints/2D/JointSolver2D.cs` for row preparation,
      warm-start application, iterative solve, and impulse storage.
- [x] Add tests for row math:
  - anchor positions from local 2D frames.
  - distance error for distance joints.
  - coincident anchors for pin/revolute joints.
  - scalar angular error for weld/fixed joints.
  - slider axis projection for prismatic joints.
  - scalar effective mass respects frozen axes and angular freeze.
- [x] Implement distance joint rows.
- [x] Implement pin/revolute anchor rows.
- [x] Implement weld/fixed linear and angular rows.
- [x] Implement prismatic/slider axis rows and limits.
- [x] Implement angular limit rows.
- [x] Implement motor rows with bounded impulses.
- [x] Add deterministic warm-start tests across repeated frames.

**Done Criteria**

- 2D joints solve through impulses, not pose snapping.
- Scalar inertia and body freeze constraints are respected.
- Warm-start data is invalidated when joint type, limits, motors, linked bodies,
  or mobility changes.

## Workstream 3: 2D Island Integration With Contacts

**Status:** Done

**Problem**

2D joints and 2D contacts must solve as one deterministic island. A linked 2D
body resting on a platform should not have separate joint and contact solvers
fighting each other.

**Tasks**

- [x] Extend the pure 2D island graph to union dynamic bodies connected by
      enabled `Joint2D` constraints.
- [x] Sort 2D island constraints by documented stable keys:
  - island root dynamic ID.
  - constraint kind.
  - contact pair ID or joint ID.
  - row index.
- [x] Prepare 2D contact rows and 2D joint rows before solver iteration.
- [x] Warm-start both contact and joint rows before the first iteration.
- [x] Iterate contacts and joints in the same bounded loop.
- [x] Persist joint warm-start data after the solve.
- [x] Propagate sleep and wake state through enabled 2D joints.
- [x] Add tests for:
  - two linked bodies form one island.
  - a linked body contacting a static platform solves contacts and joints
    together.
  - registration order does not change final solve order.
  - pushing one link wakes the connected island.
  - disabled joints do not union islands.

**Done Criteria**

- 2D contacts and joints solve in one deterministic island.
- Sleep/wake behavior matches the linked 2D graph.
- Solver order remains stable under movement and registration churn.

## Workstream 4: 2D Linked Collision Filtering And Ragdoll Authoring

**Status:** Done

**Problem**

2D articulated bodies need linked-collider filtering and a higher-level
authoring surface without turning collider hierarchy into the physical joint
system.

**Tasks**

- [x] Add linked-collider filtering tests:
  - directly linked 2D colliders suppress collision by default.
  - linked colliders collide when policy is `Collide`.
  - non-adjacent ragdoll links follow the ragdoll self-collision policy.
  - external colliders still collide with ragdoll links.
- [x] Add allocation-free `Constraints2D.ShouldExcludeLinkedCollision(...)`
      helpers for 2D broad phase, 2D queries where relevant, and 2D CCD source
      filtering.
- [x] Add `src/Gravitas/Constraints/2D/RagdollDefinition2D.cs`.
- [x] Add `src/Gravitas/Constraints/2D/RagdollLinkDefinition2D.cs`.
- [x] Add `src/Gravitas/Constraints/2D/RagdollJointDefinition2D.cs`.
- [x] Add `src/Gravitas/Constraints/2D/RagdollRuntime2D.cs`.
- [x] Add `RagdollSelfCollisionPolicy` reuse or extraction if the existing 3D
      policy is dimension-neutral.
- [x] Validate 2D ragdoll definitions:
  - all link IDs are unique.
  - all joint link references resolve.
  - all links belong to one context.
  - every link body has an active 2D collider.
  - authored joints are compatible with their linked bodies.
- [x] Provide activation/deactivation methods that switch linked `SolidBody2D`
      instances between dynamic and kinematic/host-driven modes
      deterministically.

**Done Criteria**

- 2D ragdolls are authored over ordinary `SolidBody2D` links.
- Linked filtering is explicit, deterministic, and allocation-free after warmup.
- Collider hierarchy remains identity/default grouping infrastructure rather
  than the joint solver.

## Workstream 5: CCD, Serialization, Replay Hashing, And Diagnostics

**Status:** Done

**Problem**

2D constraints affect authoritative simulation state. CCD, save/load,
diagnostics, and replay hashing must be explicit before the feature is
release-quality.

**Tasks**

- [x] Add CCD tests:
  - fast linked 2D bodies keep existing body CCD behavior.
  - adjacent linked bodies do not self-hit through CCD.
  - external static blockers still clip active links.
  - joint wake propagation works after CCD handoff.
- [x] Add Chronicler `RecordData(...)` for `Joint2D` mutable state.
- [x] Add Chronicler `RecordData(...)` for `RagdollRuntime2D` activation state.
- [x] Add replay hash contribution for:
  - service joint counts and IDs.
  - enabled state.
  - joint type.
  - local frames.
  - limits.
  - motor payloads.
  - collision policy.
  - ragdoll activation and link/joint counts.
- [x] Add diagnostic events or views for:
  - 2D joint registered/removed.
  - 2D joint impulse.
  - 2D joint limit reached.
  - 2D ragdoll activated/deactivated.
  - 2D joint row count, clamped-row count, and solver error metrics.
- [x] Add debug draw capture for 2D joint anchors, axes, limits, and ragdoll
      links using existing engine-agnostic draw commands where possible.
- [x] Add tests proving disabled diagnostics allocate `0` bytes after warmup.

**Done Criteria**

- 2D constraint state can serialize into existing host-created shells.
- Replay hashes include authoritative 2D constraint state.
- Diagnostics expose enough deterministic data to debug 2D articulation.

## Workstream 6: Benchmarks, Docs, And Release Validation

**Status:** Done

**Problem**

Pure 2D constraints should ship with performance evidence and docs that make the
API feel first-class, not a shadow of 3D.

**Tasks**

- [x] Add `tests/Gravitas.Benchmarks/Core/Constraint2DBenchmarks.cs` covering:
  - long 2D chain solve.
  - 2D ragdoll activation.
  - contact-heavy articulated 2D bodies.
  - motor-driven 2D chain.
  - linked self-collision filtering.
- [x] Add allocation guardrails for steady-state 2D joint solve and linked
      filtering.
- [x] Update `docs/wiki/OVERVIEW.md` with the 2D constraint subsystem.
- [x] Update `docs/wiki/RUNTIME_ARCHITECTURE.md` with
      `GravitasConstraint2DService` ownership.
- [x] Update `docs/wiki/DIMENSIONS.md` to describe native 2D constraints and the
      difference from 3D joints.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` with 2D joint/contact island
      ordering.
- [x] Update `docs/wiki/HOST_INTEGRATION.md` with 2D joint and ragdoll examples.
- [x] Update `docs/wiki/SERIALIZATION.md` and `docs/wiki/DIAGNOSTICS.md`.
- [x] Run focused tests:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Constraint2D|FullyQualifiedName~Ragdoll2D"`
- [x] Run release validation:
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [x] Run focused 2D constraint benchmark smoke.
- [x] Move this plan to `docs/feature-work/done` only after tests, benchmarks,
      docs, and deferred-work extraction are complete.

**Done Criteria**

- 2D constraints and ragdoll-style articulated bodies are documented as
  first-class pure 2D physics.
- Benchmarks cover runtime cost and allocation behavior.
- Release and Lean validations pass.

## Final Done Criteria

- Gravitas exposes a context-owned pure 2D constraint service.
- 2D joints are native planar/scalar constraints over `SolidBody2D`.
- 2D contacts and joints solve in one deterministic island.
- 2D ragdoll authoring uses explicit links, joints, and self-collision policy.
- CCD, serialization, replay hashing, diagnostics, and benchmarks cover the 2D
  articulated-body path.
- No 2D API pretends to be 3D with one axis ignored.
