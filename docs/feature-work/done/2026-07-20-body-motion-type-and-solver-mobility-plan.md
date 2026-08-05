# Body Motion Type And Solver Mobility Hardening Plan

**Date:** 2026-07-20  
**Status:** Complete  
**Owner:** Gravitas body, solver, and lifecycle hardening

## Goal

Separate a body's runtime role from its constrained degrees of freedom so a
dynamic body may lock every translation axis while retaining angular motion. The
redesign must remain deterministic, engine agnostic, allocation-conscious, and
symmetric across 2D, 3D, and mixed simulation paths.

## Confirmed Root Cause

`BodyFreezeAxes2D.Position` and `BodyFreezeAxes3D.Position` are constraint
masks, but the runtime currently treats them as a body-role decision. Full
position freeze therefore leaks into collider static classification, partition
membership, awake admission, islands, joints, contact response, sleep,
visualization, and CCD. `CanRotate` also depends on `CanTranslate`, while the 3D
angular-inertia helpers independently reject a position-frozen body.

Changing only `CanRotate` would expose a half-enabled state: the body could
claim angular mobility while remaining in static partitions and outside the
solver lifecycles that must advance that rotation. The fix is an explicit body
role plus independent solver degrees of freedom.

## Locked Contract

### Explicit Body Role

Both dimensions use one engine-agnostic enum with stable values:

```csharp
public enum BodyMotionType
{
    Dynamic = 0,
    Kinematic = 1,
    Static = 2
}
```

- `Dynamic` bodies are solver controlled. `FreezeAxes` constrains their
  translational and angular degrees of freedom independently.
- `Kinematic` bodies are host controlled, remain simulated for deterministic
  host motion and CCD, and have infinite solver mass.
- `Static` bodies are immobile, have infinite solver mass, use static partition
  membership, and are excluded from simulated-body collections.
- `BodyFreezeAxes*.All` locks every solver degree of freedom but does not change
  `MotionType`. Hosts should select `Static` for permanently immobile bodies.

### Public API

- `Initialize(...)` accepts `BodyMotionType motionType = BodyMotionType.Dynamic`
  instead of `bool isDynamic`.
- Bodies expose read-only `MotionType`, `IsDynamic`, `IsKinematic`, and
  `IsStatic` inspection properties.
- `SetMotionType(BodyMotionType motionType)` owns runtime role transitions. A
  mutable property is deliberately avoided because a transition changes service
  membership, partition membership, CCD state, and sleep state.
- Undefined enum values and undefined freeze bits fail explicitly.
- Keep `IsPositionFullyFrozen` and `IsRotationFullyFrozen` as the symmetric
  read-only names in both dimensions. Remove `AngularMotionFrozen` and
  `AngularForcesHalted` rather than retaining duplicate or misleading aliases.

### Mobility Semantics

- `CanTranslate` continues to mean that at least one solver-responsive
  translational degree of freedom exists.
- `CanRotate` is independent of `CanTranslate` and requires a dynamic, active
  body with valid angular inertia and at least one unfrozen rotational degree of
  freedom.
- Internal `HasSolverMobility` is `CanTranslate || CanRotate` and is the body
  admission rule for awake state, islands, joints, integration, sleep,
  visualization, and rotational CCD.
- Kinematic host motion is not silently projected through dynamic-body freeze
  constraints.
- Collider `IsStatic` means bodyless or `MotionType.Static`; it must not infer
  static role from `DynamicId` or frozen axes.

### Runtime Transitions

- Motion-type changes are atomic and permitted only outside the whole `Simulate`
  to `LateSimulate` fixed-step transaction. A change after `Simulate` but before
  its matching `LateSimulate`, or reentrantly from any context, service, body,
  collision, constraint, or lifecycle callback entry path, fails clearly rather
  than being silently deferred.
- Dynamic-to-kinematic and kinematic-to-dynamic keep body/collider identity but
  clear incompatible accumulated force, velocity, sleep, pending handoff, and
  dirty CCD state before repartitioning.
- Transitions to or from static update the simulated-body collection,
  `DynamicId`, candidate lifetimes, pending CCD work, and pure/mixed partition
  membership without destroying colliders, joints, pairs, or host identity.
- Object, host, collider ID, pair, and joint identity are preserved. `DynamicId`
  is ephemeral simulated-body membership and may change after leaving and
  re-entering a moving role.
- Connected contact warm-start accumulators and joint solver caches are cleared
  before the new role can solve. Their registered objects are retained.
- Every accepted transition invalidates service-level prepared CCD indices,
  candidate lifetimes, dirty sets, processed sets, and queued handoffs in
  addition to body-local trajectory state.
- Transitioning from dynamic to kinematic or static publishes the authoritative
  body pose to the host transform first. It never adopts an interpolated host
  pose as new simulation truth or creates a false first-frame kinematic sweep.
- Every transition refreshes mass/inertia state after the new role is installed,
  so a kinematic/static body returning to dynamic can rotate immediately.
- Static bodies remain excluded from per-frame and solver motion but may be
  repositioned explicitly between fixed steps. Pose mutation refreshes pure and
  mixed partitions before the next query or collision step.
- Ragdoll role changes prevalidate the complete link set and apply as one batch;
  failure cannot leave a partially transitioned articulation.
- Transition logic must not depend on unstable collection iteration order and
  must remain allocation-free after warmup.

### Serialization And Replay

- Record `MotionType` explicitly with `Dynamic` as the default value.
- Remove serialized `IsKinematic` and the separate 2D `_isDynamic` truth.
- Read and validate motion type and freeze bits before applying loaded state.
  Population of an already-registered shell uses the same privileged atomic
  transition path and ordering as a public transition, without bypassing service
  membership, cache, partition, or inertia invariants.
- Bump the `body.2d` and `body.3d` replay section contracts from version 3 to
  version 4.
- Gravitas has not released yet, so no legacy role inference or compatibility
  switch is carried into the first alpha. `MotionType` is the only role truth.

## Non-Goals

- Do not add a scene graph, engine adapter, compatibility flag, or hidden body
  mode inference.
- Do not broaden `CanTranslate` to mean any kind of mobility.
- Do not add a second public mobility abstraction until a concrete host need
  exists.
- Do not rebuild joints or contact pairs merely because a body's role changes.
- Do not introduce allocations or general-purpose BCL collections in runtime hot
  paths.

## Phase 1: Contract Regressions And API Adoption

- [x] Add failing 2D and 3D regressions proving a dynamic position-frozen body
      can rotate, respond angularly, remain awake when rotating, and publish
      visualization state.
- [x] Add failing static, kinematic, and invalid-enum admission regressions.
- [x] Establish the first same-scenario warmed motion-role transition baseline;
      no pre-change equivalent existed for an explicit role transition.
- [x] Add `BodyMotionType` and migrate initialization/builders from the boolean
      role contract.
- [x] Replace mutable `IsKinematic` inputs with read-only role inspection and
      `SetMotionType(...)`.
- [x] Remove misleading duplicate freeze/mobility properties and update focused
      public documentation.

## Phase 2: Independent Solver Degrees Of Freedom

- [x] Make 2D/3D `CanRotate` independent of translation and preserve constrained
      angular inertia for position-frozen dynamic bodies.
- [x] Add `HasSolverMobility` and migrate body integration, awake admission,
      sleep, visualization, islands, and joints from translation-only gates.
- [x] Update pure 2D, pure 3D, and mixed response admission and effective-mass
      paths so angular-only participants receive torque without linear motion.
- [x] Cover off-center contacts, friction, resting contacts, joint response,
      fully rotation-frozen bodies, and fully locked dynamic bodies.

## Phase 3: Role-Aware Runtime Ownership

- [x] Make collider static classification depend only on body role or bodyless
      ownership.
- [x] Add focused simulated-body membership helpers so static transitions do not
      dessimilate colliders, joints, or contact pairs.
- [x] Repartition pure 2D, pure 3D, and mixed collider membership after an
      accepted role transition.
- [x] Clear incompatible force, velocity, sleep, contact warm-start, joint
      solver, candidate lifetime, prepared-index, dirty/processed CCD, and
      handoff state atomically.
- [x] Publish the authoritative pose before changing to kinematic/static and
      refresh mass/inertia before returning to dynamic.
- [x] Reject role changes during the entire context fixed-step transaction and
      from direct service/body entry paths.
- [x] Migrate ragdoll activation/deactivation/load to a prevalidated atomic
      batch transition.
- [x] Cover every transition direction, identity preservation, partition role
      changes, pair/joint preservation, repeated transitions, deterministic
      replay, and warmed zero-allocation behavior.
- [x] Cover post-`Simulate`/pre-`LateSimulate` rejection, callback and direct
      service/body reentrancy, warmed contact/joint invalidation, interpolation
      lag at pose handoff, first-step angular response after returning to
      dynamic, and prepared-index invalidation.
- [x] Cover static explicit repositioning and prove queries/contacts observe the
      refreshed pure and mixed partitions without per-frame simulation.
- [x] Cover a fully locked dynamic body remaining in dynamic partition
      membership, not seeding an awake island, still colliding with an awake
      counterpart, receiving no impulse, and emitting no duplicate lifecycle
      callbacks.

## Phase 4: CCD And Dimension Parity

- [x] Keep translational CCD admission tied to translational mobility.
- [x] Admit position-frozen dynamic bodies to rotational CCD when angular motion
      can change their collider bounds.
- [x] Update 2D, 3D, and mixed target/source policies that currently use full
      position freeze as static equivalence.
- [x] Cover angular-only dynamic sources and targets, static and kinematic role
      parity, candidate refresh, same-frame requeue cleanup, and deterministic
      ordering.

## Phase 5: Serialization, Replay, Documentation, And Benchmarks

- [x] Serialize `MotionType` and freeze axes as the only authoritative role and
      constraint state, with strict load validation.
- [x] Cover all six cross-role population directions for JSON and MemoryPack,
      including already-registered 2D and 3D shells.
- [x] Bump replay section versions and update replay hashing/tests.
- [x] Update host integration, dimensions, response, CCD, runtime architecture,
      serialization, and overview documentation.
- [x] Migrate benchmarks from full position freeze-as-static to explicit
      `BodyMotionType.Static` where that is the intended scenario.
- [x] Establish warmed motion-type transition baselines at about 15.146
      microseconds for 3D and 6.751 microseconds for 2D with zero managed
      allocation. No pre-change same-scenario comparison exists because the
      explicit transition operation is new.

## Phase 6: Closure

- [x] Move the issue to `Resolved Issues` with RCA, behavior, benchmark, and
      verification evidence.
- [x] Run focused regression suites during each red-green cycle.
- [x] Run the full `Release` gate and attempt `ReleaseLean`, recording its
      lower-stack local-link blocker without adding a downstream workaround.
- [x] Re-achieve 100% line, branch, and method coverage for hand-authored source
      without hollow tests or zombie branches.
- [x] Run replay determinism and relevant benchmark gates.
- [x] Run `git diff --check` and inspect all unstaged changes, preserving the
      local-link project-file scaffolding.
- [x] Obtain an independent final code review and resolve every critical or
      important finding before handoff.
- [x] If locally linked lower-stack packaging prevents `ReleaseLean`, record the
      exact external blocker rather than weakening or claiming that gate.

## Verification Record

Populate this section as the work progresses. Do not mark the plan done until
fresh final evidence is recorded.

- Baseline focused mobility tests: 31 passed on 2026-07-20.
- Baseline authoritative coverage: 3,123 tests; 33,539/33,539 lines,
  12,137/12,137 branches, and 4,158/4,158 methods.
- Release verification: 3,237/3,237 tests passed on `net8.0`; the library and
  benchmark projects build with zero warnings and zero errors.
- ReleaseLean verification: attempted for both library targets. The retained
  local GridForge project link exposes `MemoryPack.Core` public interfaces to
  the lean consumer at `PhysicsMesh.cs:660`, `PhysicsMesh.cs:666`, and
  `CollisionNotificationExceptions.cs:30`, producing 12 `CS0012` diagnostics
  across `net8.0` and `netstandard2.1`. This is the previously recorded
  lower-stack local-link packaging blocker and was not masked in Gravitas.
- Coverage verification: 3,237 tests; 33,894/33,894 lines, 12,211/12,211
  branches, and 4,206/4,206 methods. The extracted method-gap report is empty.
- Replay verification: 80/80 focused replay and determinism tests passed.
- Benchmark verification: warmed motion-role transitions measure about 15.146
  microseconds for 3D and 6.751 microseconds for 2D, both with zero managed
  allocation. The benchmark project builds with zero warnings and errors.
- Independent review: completed after every important finding was fixed with a
  substantive regression; the final pass reported no remaining critical or
  important findings.

## Completion Summary

Gravitas now owns explicit `Dynamic`, `Kinematic`, and `Static` body roles
without overloading freeze axes as runtime identity. Dynamic translation and
rotation mobility are independent across 2D, 3D, mixed response, constraints,
sleep, visualization, partitioning, and CCD. Role transitions, ragdoll batches,
serialization population, and explicit static pose changes prevalidate before
mutation, preserve runtime object identity, clear incompatible solver state, and
refresh pure and mixed partitions atomically. Static 3D colliders can leave and
re-enter a grid and remain immediately query-visible.

The first-alpha contract is documented directly without a migration layer or
legacy API aliases. Substantive 2D/3D/mixed, serialization, replay, lifecycle,
partition, invalid-transform, and zero-allocation regressions retain 100% line,
branch, and method coverage.
