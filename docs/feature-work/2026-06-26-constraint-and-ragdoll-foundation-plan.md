# Constraint And Ragdoll Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add first-class deterministic articulated-body physics so Gravitas can support ragdolls, linked rigid bodies, joint limits, and animation-driven physical handoff without depending on an engine animation or rigidbody system.

**Architecture:** Reuse existing collider hierarchy keys for articulated group identity and default self-filtering, but add a dedicated constraint/joint model for physical links. Integrate joints into deterministic 3D solver islands alongside contacts, preserve warm-start state, expose authoring definitions for ragdoll links, and keep future animation libraries as hosts that feed deterministic targets into Gravitas rather than owning physical truth.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet for scaling signals, FixedMathSharp `Fixed64`/`Vector3d`/`FixedQuaternion`/`Fixed3x3`, SwiftCollections buffers and pools, Gravitas 3D body/collider/collision services, Chronicler explicit state recording.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas constraint/ragdoll hardening

## Purpose

Gravitas has deterministic rigid bodies, colliders, 3D contact manifolds,
continuous collision detection, mixed-dimension response, hierarchy-based
collider grouping, and warm-started solver islands. It does not yet have a
first-class physical articulation model.

Ragdolls should not be a thin Unity-style toggle that enables engine
`Rigidbody` and `Collider` components. For Gravitas, ragdoll support means:

- multiple deterministic bodies linked by deterministic constraints.
- explicit joint frames, anchors, angular limits, and optional motor targets.
- stable solver ordering and warm-started constraint impulses.
- collision filtering between linked body parts without hiding non-adjacent
  self-collision options.
- deterministic activation from animation-owned or kinematic poses into dynamic
  physical simulation.
- serialization, diagnostics, and benchmarks that prove the model is reliable.

Future deterministic animation libraries can consume this foundation by
creating ragdoll definitions, setting kinematic or motor targets, and reading
support/query/contact state. They should not own the physics constraints or rely
on engine-specific APIs.

## Current Baseline

- `src/Gravitas/Colliders/Hierarchy/ColliderHierarchyState.cs` stores
  parent/child/top-parent collider state, tracks dimension-tagged
  `ColliderHierarchyKey` values, suppresses parent-child and sibling
  collisions, and clears child parent references on deactivation.
- `LSCollider` and `LSCollider2D` already expose hierarchy-backed filtering
  through `ExcludesCollisionWith(...)`.
- Pure 3D, pure 2D, and mixed broad phases already respect hierarchy filtering
  during candidate generation and query/CCD filtering.
- 3D contacts are solved inside deterministic discrete islands under
  `GravitasPhysicsService`.
- Warm-start state is pair-owned for contact response.
- `PhysicsSettings.DiscreteSolverIterations` controls bounded projected-impulse
  iterations for multi-contact discrete islands.
- There is no public joint, constraint, articulation, ragdoll, or motor model.
- There is no constraint serialization or diagnostic event surface.

## Relationship To Collider Hierarchy

Reuse the existing collider hierarchy system for:

- stable group identity through `ColliderHierarchyKey`.
- parent/top-parent tracking for link ownership.
- default suppression of parent-child collision.
- deterministic cleanup when parent colliders deactivate or IDs are reused.
- cross-dimension identity if future 2D articulation work needs it.

Do not force collider hierarchy to own:

- physical joint constraints.
- angular limits.
- motor targets.
- constraint impulses.
- solver island edges.
- ragdoll definitions.
- per-link mass distribution.

The ragdoll implementation should either extend hierarchy filtering with an
explicit self-collision policy or layer an articulation-owned filter on top of
`ColliderHierarchyKey`. Current sibling suppression is useful for compound
collider behavior, but ragdolls need finer control: adjacent limbs usually do
not collide, while non-adjacent limbs may need to collide depending on the
authored model.

## Non-Goals

- Do not implement FootIK, HandIK, look-at solving, animation curve sampling, or
  pose blending in Gravitas.
- Do not depend on Unity, Godot, Unreal, or engine rigidbody/animator APIs.
- Do not add cloth, soft bodies, muscle simulation, or full humanoid animation
  controllers.
- Do not make ragdolls a separate physics world from ordinary `SolidBody`
  simulation.
- Do not hide determinism-sensitive behavior behind reflection or engine
  callbacks.
- Do not keep weak compatibility aliases if the `SolidBody` naming cleanup has
  already renamed `SolidBody`.
- Do not broaden this plan into pure 2D joint families unless implementation
  proves the shared 3D infrastructure naturally supports them. Pure 2D joints
  can be planned separately if a real 2D gameplay need appears.

## Ordering Assumptions

This plan should run after:

1. `SolidBody` naming cleanup, so new public APIs use the final body name.
2. `Restitution Gravity And Grounded State Hardening`, so solver/body settings
   are context-owned.
3. `Collider Local Collision Filtering`, so linked-collider collision policy has
   the final physical-filtering model.
4. `Body Axis Freeze Constraints`, so joints and ragdolls build on the final
   mobility/constraint semantics instead of deprecated `Immovable` and angular
   prevention state.

If this plan starts before the naming cleanup, apply every `SolidBody` reference
below to the current `SolidBody` type and rename during the cleanup workstream.

## Guiding Rules

- Constraints are authoritative physics state.
- All ordering must be stable by context-owned IDs, not hash iteration order.
- Constraint rows must use fixed-point math only.
- Solver islands should include contact edges and joint edges in one
  deterministic graph.
- Warm-start impulses must be pair/joint-owned and invalidated explicitly when
  topology changes.
- Disabled or absent constraint systems must add no per-frame allocations.
- Runtime ragdoll activation must be deterministic and replayable.
- Public APIs should make authored frames, limits, motors, and self-collision
  policy explicit.

## Proposed API Shape

The exact names should be finalized during Workstream 1. The intended shape is:

```csharp
public sealed class GravitasConstraint3DService
{
    public Joint3D RegisterJoint(in JointDefinition3D definition);
    public bool TryGetJoint(int jointId, out Joint3D joint);
    public bool RemoveJoint(int jointId);
}

public readonly struct JointDefinition3D
{
    public SolidBody BodyA { get; }
    public SolidBody BodyB { get; }
    public FixedTransform LocalFrameA { get; }
    public FixedTransform LocalFrameB { get; }
    public JointType3D Type { get; }
    public JointLimit3D Limits { get; }
    public JointMotor3D Motor { get; }
    public JointCollisionPolicy CollisionPolicy { get; }
}

public enum JointType3D
{
    BallSocket,
    Hinge,
    ConeTwist,
    Fixed
}
```

Suggested ragdoll authoring surface:

```csharp
public sealed class RagdollDefinition3D
{
    public RagdollLinkDefinition3D[] Links { get; }
    public RagdollJointDefinition3D[] Joints { get; }
    public RagdollSelfCollisionPolicy SelfCollisionPolicy { get; }
}
```

Implementation should prefer arrays or caller-owned spans for authoring data
over mutable lists in hot paths. Runtime services can copy validated definitions
into context-owned SwiftCollections storage.

## Workstream 1: Constraint Identity, Service Ownership, And Settings

**Problem**

Constraints need stable IDs, explicit context ownership, deterministic
registration order, and settings before any joint math lands.

**Tasks**

- [ ] Add tests for an empty constraint service:
  - a new `GravitasWorldContext` owns one `GravitasConstraint3DService`.
  - registering a joint assigns a deterministic monotonic ID.
  - removing a joint releases solver state and prevents future lookup.
  - duplicate body pair registration is allowed only when joint IDs differ.
- [ ] Add `src/Gravitas/Constraints/3D/GravitasConstraint3DService.cs`.
- [ ] Expose the service from `GravitasWorldContext` with a clear property such
  as `Constraints3D`.
- [ ] Add `src/Gravitas/Constraints/3D/Joint3D.cs` as runtime joint state.
- [ ] Add `src/Gravitas/Constraints/3D/JointDefinition3D.cs` as the public
  validated registration input.
- [ ] Add `src/Gravitas/Constraints/3D/JointType3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/JointLimit3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/JointMotor3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/JointCollisionPolicy.cs`.
- [ ] Add `PhysicsSettings.ConstraintSolverIterations` only if solver iteration
  count should differ from contact response. If the same solver loop handles
  contacts and joints, rename or document the existing discrete solver setting
  so it clearly applies to all discrete constraints.
- [ ] Validate joint definitions:
  - body A and body B are non-null.
  - body A and body B are different bodies.
  - both bodies belong to the same `GravitasWorldContext`.
  - local frames are finite deterministic transforms.
  - limits are non-negative and ordered.
  - motor strengths are non-negative.
- [ ] Run:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Constraint`

**Done Criteria**

- Constraint registration is deterministic and context-owned.
- Runtime joint IDs are stable within a context.
- Invalid definitions fail before reaching solver code.

## Workstream 2: Collider Hierarchy And Articulation Filtering

**Problem**

Ragdolls need linked collider filtering, but the existing hierarchy sibling
suppression is too coarse for all articulated bodies. The plan should reuse
hierarchy identity while allowing authored self-collision policy.

**Tasks**

- [ ] Add tests proving existing hierarchy behavior remains intact:
  - parent-child collisions are suppressed.
  - current compound-style sibling collisions are suppressed.
  - deactivating a parent clears child hierarchy references.
- [ ] Add articulation filtering tests:
  - adjacent joint-linked colliders do not collide by default.
  - non-adjacent links can collide when the ragdoll policy allows it.
  - non-adjacent links can be suppressed when the ragdoll policy forbids it.
  - external colliders still collide with ragdoll links.
- [ ] Introduce an explicit articulation self-collision policy rather than
  changing `ColliderHierarchyState.ExcludesCollisionWith(...)` into a ragdoll
  solver.
- [ ] Reuse `ColliderHierarchyKey` as the stable identity stored by the
  articulation filter.
- [ ] Add an allocation-free service helper for physical pair filters:
  `ShouldExcludeLinkedCollision(colliderA, colliderB)`.
- [ ] Route 3D broad-phase pair creation through the new helper after layer and
  local collision filtering.
- [ ] Route 3D query and CCD source-skip filtering through the same policy where
  linked self-collision would otherwise produce false hits.
- [ ] Keep pure 2D and mixed hierarchy behavior unchanged unless tests reveal a
  regression from shared helpers.

**Done Criteria**

- Collider hierarchy remains the ownership/default-filtering foundation.
- Ragdolls can express adjacent-only or full self-collision policy.
- Linked filtering is deterministic and allocation-free after warmup.

## Workstream 3: Constraint Rows And Warm-Started Solver Math

**Problem**

Ragdoll joints require physically meaningful constraint rows, not positional
teleporting. The solver should produce deterministic impulses for linear anchor
error and angular limit error.

**Tasks**

- [ ] Add isolated math tests for constraint row construction:
  - world anchor positions are derived from body transforms and local frames.
  - linear error is zero when anchors coincide.
  - angular error is zero when joint frames are aligned.
  - hinge axes and cone/twist axes are normalized deterministically.
- [ ] Add `src/Gravitas/Constraints/3D/JointConstraintRow3D.cs` for internal
  row state:
  - Jacobian linear axis.
  - angular axis for body A.
  - angular axis for body B.
  - accumulated impulse.
  - effective mass.
  - bias velocity.
  - lower and upper impulse bounds.
- [ ] Add `src/Gravitas/Constraints/3D/JointSolver3D.cs` for row preparation,
  warm-start application, iteration solve, and impulse storage.
- [ ] Implement fixed-point effective mass using existing body mass/inertia APIs.
- [ ] Include body-axis freeze constraints from the previous plan in effective
  mass calculations.
- [ ] Implement linear anchor rows for ball-socket behavior.
- [ ] Implement angular rows for fixed orientation behavior.
- [ ] Implement hinge limit rows.
- [ ] Implement cone/twist limit rows for shoulders, hips, neck, and spine-like
  ragdoll joints.
- [ ] Apply warm-start impulses only when the same joint row identity remains
  valid across frames.
- [ ] Clamp impulses deterministically using fixed lower/upper bounds.
- [ ] Add tests proving repeated runs produce identical body poses and
  accumulated impulses.

**Done Criteria**

- Constraint rows are deterministic and warm-started.
- Anchors and angular limits are solved through impulses, not pose snapping.
- Frozen body axes and inertia tensors are respected.

## Workstream 4: Discrete Island Integration With Contacts

**Problem**

Joints and contacts must solve together. A ragdoll resting on the ground is one
constraint graph, not separate contact and joint passes fighting each other.

**Tasks**

- [ ] Add island tests:
  - two bodies connected by one joint form one island.
  - a jointed body touching a static floor forms an island with joint and
    contact constraints.
  - joint registration order does not change final sorted island solve order.
  - sleeping one linked body wakes the connected island when another link is
    pushed.
- [ ] Extend 3D island graph construction to union dynamic bodies connected by
  enabled joints.
- [ ] Sort island constraints by:
  - island root dynamic ID.
  - constraint type, with contacts and joints in a documented deterministic
    order.
  - pair ID or joint ID.
  - row index.
- [ ] Prepare contact rows and joint rows before iteration.
- [ ] Warm-start both contact and joint rows before the first iteration.
- [ ] Iterate contacts and joints in the same bounded solver loop.
- [ ] Persist joint warm-start data after the solve.
- [ ] Update sleep logic so linked bodies sleep and wake as one island when
  joints are enabled.
- [ ] Add diagnostics counters for island joint count and row count.

**Done Criteria**

- Joint constraints and contact constraints solve in one deterministic island.
- Sleep/wake behavior propagates through articulation links.
- Solver order is stable under registration and movement churn.

## Workstream 5: Ragdoll Authoring Definitions And Runtime Activation

**Problem**

Hosts need a high-level way to author ragdolls without hand-registering every
joint row. The API should validate mass, collider, body, hierarchy, and joint
data before activation.

**Tasks**

- [ ] Add `src/Gravitas/Constraints/3D/RagdollDefinition3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/RagdollLinkDefinition3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/RagdollJointDefinition3D.cs`.
- [ ] Add `src/Gravitas/Constraints/3D/RagdollRuntime3D.cs`.
- [ ] Add tests for definition validation:
  - every joint references valid link IDs.
  - every link has a body and collider.
  - link bodies belong to one context.
  - mass values are positive and inertia is available.
  - default self-collision policy suppresses adjacent links.
  - invalid cycles are rejected only when they would create impossible hierarchy
    ownership; tree-like humanoid chains are accepted.
- [ ] Add a registration method such as
  `Constraints3D.RegisterRagdoll(in RagdollDefinition3D definition)`.
- [ ] Bind ragdoll links to collider hierarchy keys for group identity.
- [ ] Register one or more joints per ragdoll joint definition.
- [ ] Provide `ActivateDynamic()` to switch linked bodies into dynamic ragdoll
  simulation deterministically.
- [ ] Provide `DeactivateToKinematic()` to return linked bodies to
  host/animation-driven kinematic control without losing definitions.
- [ ] Ensure activation order is stable by link ID, not array reference identity.

**Done Criteria**

- Hosts can register a ragdoll from validated link and joint definitions.
- Activation/deactivation is deterministic.
- Ragdoll runtime state reuses collider hierarchy identity without abusing it as
  the joint solver.

## Workstream 6: Kinematic Targets, Motors, And Animation Handoff Boundary

**Problem**

The future deterministic animation library needs a clean way to drive ragdoll
links or blend toward animated poses. Gravitas should expose deterministic
kinematic/motor inputs, not animation curves, IK solvers, or engine animator
hooks.

**Tasks**

- [ ] Add tests for deterministic motor targets:
  - setting the same target pose produces the same impulse sequence.
  - motor strength zero produces no motor impulse.
  - motor strength above zero pulls toward the target within configured limits.
  - motor impulses respect joint limits and body freeze constraints.
- [ ] Add target fields to `JointMotor3D`:
  - target local orientation or frame.
  - angular drive strength.
  - angular drive damping.
  - maximum motor impulse.
- [ ] Provide explicit runtime methods:
  - `SetJointMotorTarget(int jointId, FixedQuaternion targetLocalRotation)`.
  - `ClearJointMotorTarget(int jointId)`.
  - `SetRagdollPoseTargets(...)` using caller-owned arrays or spans.
- [ ] Keep motor target application in deterministic simulation phases only.
- [ ] Document that FootIK, HandIK, look-at, and animation event curves belong
  in the future animation library.
- [ ] Add a small host-boundary sample in docs showing:
  - animation library computes target link frames.
  - host passes fixed target frames into Gravitas before `Simulate`.
  - Gravitas solves motors and constraints during `Simulate`/`LateSimulate`.

**Done Criteria**

- Gravitas can receive deterministic animation-driven physical targets.
- The API boundary is clean enough for a future animation library.
- No engine animation or IK code enters Gravitas.

## Workstream 7: CCD, Serialization, Diagnostics, And Benchmarks

**Problem**

Articulated bodies touch nearly every runtime concern. CCD, save/load,
diagnostics, and benchmark signal must be explicit before this feature can be
called release-ready.

**Tasks**

- [ ] Add CCD tests:
  - a fast ragdoll link uses existing body CCD when moving independently.
  - a connected link handoff wakes the linked island.
  - linked self-filtering prevents adjacent-link CCD self-hits.
  - external CCD blockers still clip active links.
- [ ] Add serialization tests:
  - joint definitions round-trip.
  - enabled/disabled joint state round-trips.
  - warm-start impulses round-trip only if needed for deterministic
    continuation.
  - ragdoll activation state round-trips.
- [ ] Add Chronicler `RecordData(...)` implementations for runtime joint and
  ragdoll state.
- [ ] Add diagnostic events:
  - joint registered/removed.
  - joint impulse.
  - joint limit reached.
  - motor target error.
  - ragdoll activated/deactivated.
- [ ] Add debug draw commands for joint frames, anchor error, hinge axes, and
  cone/twist limits.
- [ ] Add benchmarks:
  - `constraint-chain-scaling`
  - `humanoid-ragdoll-resting-stack`
  - `ragdoll-activation`
  - `ragdoll-self-collision-filtering`
  - `ragdoll-ccd-fast-link`
- [ ] Prove no allocations after warmup for:
  - steady-state registered ragdoll solve.
  - disabled diagnostics path.
  - linked self-filtering checks.
- [ ] Run:
  `dotnet build Gravitas.slnx --configuration Release`
- [ ] Run:
  `dotnet test Gravitas.slnx --configuration Release`
- [ ] Run:
  `dotnet build Gravitas.slnx --configuration ReleaseLean`
- [ ] Run:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- Articulated bodies participate in CCD safely.
- Constraint and ragdoll state serializes explicitly.
- Diagnostics expose enough state to debug solver behavior.
- Benchmarks cover runtime scaling and allocation behavior.

## Workstream 8: Docs, Samples, And Final Review

**Problem**

Ragdoll support introduces a major new physical model. Documentation must make
the boundary between Gravitas physics and future animation-library behavior
obvious.

**Tasks**

- [ ] Update `docs/wiki/RUNTIME_ARCHITECTURE.md` with constraint service
  ownership and island integration.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` with joint/contact island solve
  order and self-collision filtering.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md` with ragdoll registration,
  activation, deactivation, and motor target examples.
- [ ] Update `docs/wiki/SERIALIZATION.md` with joint and ragdoll runtime state.
- [ ] Update `docs/wiki/DIAGNOSTICS.md` with joint and ragdoll diagnostic
  events.
- [ ] Update `docs/wiki/OVERVIEW.md` with the new constraint/ragdoll subsystem.
- [ ] Update `AGENTS.md` if constraint/ragdoll work changes contributor
  guidance, source layout, or release validation expectations.
- [ ] Run a final source-structure review and split any file approaching the
  1000-line warning threshold.
- [ ] Search for stale wording:
  `rg -n "future ragdoll|no joint|no constraint|animation owns physics|Unity Rigidbody|Animator" docs src/Gravitas`
- [ ] Mark this plan done and move it to `docs/feature-work/done` only after
  deferred work is either completed or captured in a focused follow-up plan.

**Done Criteria**

- Docs explain constraints and ragdolls as Gravitas physics, not animation
  features.
- Future animation-library boundaries are clear.
- No stale wiki caveats hide missing physical behavior.

## Final Done Criteria

- Gravitas has a context-owned deterministic 3D constraint service.
- Ragdoll definitions register linked bodies, colliders, joints, and
  self-collision policy explicitly.
- Joint constraints solve in deterministic islands with contacts.
- Warm-start, sleep/wake, CCD, serialization, diagnostics, and benchmarks cover
  the articulated-body path.
- Existing collider hierarchy state is reused for identity and default
  filtering, but physical articulation lives in dedicated constraint/ragdoll
  types.
- FootIK, HandIK, look-at, animation curves, and pose blending remain outside
  Gravitas and are left for the future deterministic animation library.
