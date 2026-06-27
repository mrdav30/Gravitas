# Body Axis Freeze Constraints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace coarse immovable/angular-force toggles with explicit axis freeze constraints that solver, CCD, sleep, partitioning, serialization, and mixed response all understand.

**Architecture:** Model body constraints as dimension-correct freeze flags instead of hidden boolean shortcuts. The solver should use constrained inverse mass and inertia, body convenience properties should derive from those constraints, and old mutable booleans should disappear if they would create duplicate APIs.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp `Fixed64`/`Vector2d`/`Vector3d`/`Fixed3x3`, Gravitas body motion, collision response, CCD, mixed response, Chronicler explicit state recording.

---

**Date:** 2026-06-26  
**Status:** Done  
**Owner:** Gravitas body/solver hardening

Completed on 2026-06-27. The implementation landed explicit 3D and pure 2D
freeze-axis APIs, removed mutable coarse mobility setters, projected motion and
CCD through frozen degrees of freedom, updated pure 3D/pure 2D/mixed response
to use constrained inverse mass/inertia, serialized freeze state explicitly,
removed the old read-only `Immovable` and `PreventAngularForces` aliases, and
refreshed tests, benchmarks, and docs.

## Purpose

`Immovable` and `PreventAngularForces` were useful early body controls, but they
are now too coarse for a first-class physics engine. Hosts often need to freeze
specific translation or rotation axes while leaving the rest of the body
dynamic. Examples include:

- a 3D body that can slide on X/Z but not move on Y.
- a 3D body that can translate but not rotate around one or more axes.
- a pure 2D body that can move along one planar axis but not the other.
- a pure 2D body with yaw rotation frozen while planar translation remains
  dynamic.

The existing booleans also make API semantics fuzzy:

- `Immovable` currently means infinite response mass and static partition
  membership, not just "position cannot change."
- `PreventAngularForces` currently disables angular response, but does not
  express which angular degrees of freedom are locked.

This plan replaces those booleans with explicit freeze constraints and keeps any
convenience state derived from the constraints rather than independently
mutable.

## Current Baseline

- `src/Gravitas/Core/3D/SolidBody.cs` owns `Immovable`,
  `PreventAngularForces`, `CanTranslate`, `CanRotate`, effective inverse mass,
  effective inverse inertia, sleep, and awake-state checks.
- `src/Gravitas/Core/2D/SolidBody2D.cs` owns matching pure 2D `Immovable`,
  `PreventAngularForces`, `CanTranslate`, `CanRotate`, effective inverse mass,
  scalar inverse moment of inertia, and sleep checks.
- 3D response treats immovable or kinematic bodies as infinite mass.
- Pure 2D response treats immovable or kinematic bodies as infinite mass and
  ignores angular response when `PreventAngularForces` is true.
- Mixed response checks both 3D and pure 2D body mobility.
- Partition services classify bodyless and immovable colliders as static,
  kinematic bodies as kinematic, and movable dynamic bodies as dynamic.
- CCD source eligibility skips immovable and kinematic bodies.
- Serialization records `Immovable` and `PreventAngularForces`.

## Non-Goals

- Do not add joints, motors, ragdoll constraints, or articulation graphs in this
  plan.
- Do not create compatibility aliases that preserve the old mutable boolean
  behavior.
- Do not make a body with one frozen translation axis automatically static in
  the broad phase; only fully frozen translation should behave like immovable
  partition membership.
- Do not hide dimension differences behind a shared abstraction that makes 2D
  axes ambiguous.
- Do not change collider shapes or query behavior except where mobility checks
  must respect freeze constraints.

## Guiding Rules

- Axis constraints are authoritative simulation state.
- Convenience properties must derive from constraints, not store duplicate
  mutable truth.
- Effective inverse mass and inertia should reflect the requested degrees of
  freedom at the solver boundary.
- Partition mobility should remain conservative and deterministic.
- Serialization must preserve freeze state explicitly.
- Mixed response must not accidentally give 2D bodies forbidden motion through
  3D impulses.

## Proposed API Shape

The exact names should be finalized in Workstream 1, but the recommended shape
is dimension-specific for clarity:

```csharp
[Flags]
public enum BodyFreezeAxes3D
{
    None = 0,
    PositionX = 1 << 0,
    PositionY = 1 << 1,
    PositionZ = 1 << 2,
    RotationX = 1 << 3,
    RotationY = 1 << 4,
    RotationZ = 1 << 5,
    Position = PositionX | PositionY | PositionZ,
    Rotation = RotationX | RotationY | RotationZ,
    All = Position | Rotation
}

[Flags]
public enum BodyFreezeAxes2D
{
    None = 0,
    PositionX = 1 << 0,
    PositionY = 1 << 1,
    Rotation = 1 << 2,
    Position = PositionX | PositionY,
    All = Position | Rotation
}
```

Suggested body properties:

```csharp
public BodyFreezeAxes3D FreezeAxes { get; set; }
public bool IsPositionFullyFrozen => (FreezeAxes & BodyFreezeAxes3D.Position) == BodyFreezeAxes3D.Position;
public bool AngularMotionFrozen => (FreezeAxes & BodyFreezeAxes3D.Rotation) == BodyFreezeAxes3D.Rotation;
public bool CanTranslate => Active && !IsKinematic && InverseMass > Fixed64.Zero && !FreezeAxes.HasFlag(BodyFreezeAxes3D.Position);
public bool CanRotate => CanTranslate && !FreezeAxes.HasFlag(BodyFreezeAxes3D.Rotation) && _inverseInertiaTensor != Fixed3x3.Zero;
```

For pure 2D, `PositionY` means the second coordinate in the 2D simulation plane,
which maps to world Z when published through the host transform.

During implementation, prefer explicit bit checks over `Enum.HasFlag(...)` in
hot paths because `HasFlag` can box on older targets. The snippets above show
API intent, not hot-path implementation style.

## Workstream 1: API Semantics And Migration

**Problem**

The body API needs one source of truth for mobility constraints before solver
math changes. Old mutable booleans should be removed or converted to derived
read-only properties so hosts do not have two ways to express the same state.

**Tasks**

- [x] Add `BodyFreezeAxes3D` and `BodyFreezeAxes2D` in focused files under
  `src/Gravitas/Core/3D` and `src/Gravitas/Core/2D`.
- [x] Add XML docs explaining each axis and the pure 2D X/Y to world X/Z
  mapping.
- [x] Add unit tests proving default bodies have `FreezeAxes == None`.
- [x] Replace mutable `Immovable` assignment tests with `FreezeAxes = Position`
  or `FreezeAxes = All`, depending on the scenario.
- [x] Replace mutable `PreventAngularForces` assignment tests with rotation
  freeze flags.
- [x] Remove `Immovable` as a public body property and expose
  `IsPositionFullyFrozen` as the clear derived state.
- [x] Remove `PreventAngularForces` as a public body property and expose
  `AngularMotionFrozen` as the clear derived state.
- [x] Remove old setters and backing fields after all call sites move to
  freeze axes.
- [x] Run:
  `rg -n "Immovable\\s*=|PreventAngularForces\\s*=" src/Gravitas tests/Gravitas.Tests`
  and ensure there are no remaining mutable assignments.

**Done Criteria**

- Body freeze state has one mutable API.
- Old booleans no longer store independent truth.
- Tests and helper builders use freeze axes directly.

## Workstream 2: 3D Motion, Mass, And Solver Constraints

**Problem**

3D response cannot treat axis freeze as a visual-only clamp. The solver must see
constrained inverse mass and inverse inertia so impulses are physically
explainable.

**Tasks**

- [x] Add tests for 3D linear freeze behavior:
  - full position freeze behaves like current immovable response.
  - `PositionY` freeze blocks vertical movement while allowing X/Z response.
  - `PositionX` and `PositionZ` block only their matching axes.
  - force application does not accumulate velocity on frozen axes.
- [x] Add tests for 3D angular freeze behavior:
  - full rotation freeze behaves like current angular-force prevention.
  - freezing one rotation axis prevents angular velocity around that axis.
  - unfrozen rotation axes still respond to off-center contacts.
- [x] Update `SolidBody.Motion.cs` to project velocity, acceleration, applied
  corrections, and force-derived deltas through freeze constraints.
- [x] Add helper methods on `SolidBody` for constrained linear inverse mass
  along a direction.
- [x] Add helper methods on `SolidBody` for constrained angular inverse inertia
  along a torque axis.
- [x] Update 3D response impulse denominator logic to use constrained inverse
  mass and inertia instead of only scalar `InverseMass` and full tensor checks.
- [x] Update position correction so frozen axes do not receive solver movement.
- [x] Update sleep checks so fully frozen translation does not keep a body awake
  through impossible residual velocity.
- [x] Run focused 3D response and motion tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "SolidBody|CollisionResponse"`

**Done Criteria**

- 3D impulses cannot move or rotate frozen axes.
- Unfrozen axes still behave dynamically.
- Full position freeze preserves current immovable physical behavior.

## Workstream 3: Pure 2D Motion, Mass, And Solver Constraints

**Problem**

Pure 2D has a scalar moment of inertia and two planar translation axes. It needs
the same constraint quality as 3D without pretending to own world-Y height.

**Tasks**

- [x] Add pure 2D tests for linear freeze behavior:
  - full position freeze behaves like current immovable response.
  - `PositionX` blocks X motion while allowing planar Y motion.
  - `PositionY` blocks planar Y motion while allowing X motion.
  - forces and position correction do not accumulate on frozen axes.
- [x] Add pure 2D tests for rotation freeze behavior:
  - `Rotation` blocks yaw angular velocity.
  - unfrozen yaw still responds to off-center contacts.
- [x] Update `SolidBody2D` integration to project velocities,
  accelerations, and corrections through 2D freeze constraints.
- [x] Update pure 2D effective mass helpers so impulse denominators respect
  frozen translation axes and yaw freeze.
- [x] Update `CollisionResponse2D` to use constrained 2D inverse mass and
  constrained scalar angular mass.
- [x] Update warm-start and resting friction to skip forbidden degrees of
  freedom without changing contact ordering.
- [x] Update sleep checks for fully frozen translation and rotation.
- [x] Run focused 2D response tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "2D&CollisionResponse"`

**Done Criteria**

- Pure 2D response honors per-axis freeze constraints.
- Full position freeze preserves current immovable behavior.
- The 2D API stays planar and does not leak 3D height semantics.

## Workstream 4: Mixed Response, Queries, And Partition Mobility

**Problem**

Mixed 2D/3D response has dimension-specific movement rules. Freeze constraints
must apply without letting vertical 3D impulses move a pure 2D body or letting a
partially frozen 3D body be misclassified as static.

**Tasks**

- [x] Add mixed response tests where a 3D body has one frozen translation axis
  and collides with a 2D slab.
- [x] Add mixed response tests where a pure 2D body has one frozen planar axis.
- [x] Add mixed response tests where full 2D position freeze behaves like the
  current immovable target behavior.
- [x] Update `CollisionResponseMixed` and mixed service response helpers to use
  constrained inverse mass for each participant.
- [x] Keep the existing mixed rule that vertical Y impulse affects only the 3D
  participant.
- [x] Update 3D, pure 2D, and mixed partition mobility classification:
  - bodyless colliders are static.
  - fully position-frozen dynamic bodies are static-equivalent for partition
    membership.
  - kinematic bodies remain kinematic.
  - partially frozen dynamic bodies remain dynamic.
- [x] Update query helper names only where "static" currently means bodyless or
  fully immovable and the wording becomes misleading.
- [x] Run mixed tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Mixed`

**Done Criteria**

- Mixed response respects both dimensional freeze models.
- Partition membership remains deterministic and conservative.
- Partially frozen bodies are still considered dynamic for broad-phase refresh.

## Workstream 5: CCD, Grounding, And Lifecycle Interactions

**Problem**

CCD and grounding use predicted velocities, displacements, and mobility checks.
Freeze constraints must be applied before candidates are collected and before
TOI response can hand off impossible motion.

**Tasks**

- [x] Add 3D CCD tests:
  - fully position-frozen bodies are not active CCD sources.
  - partially frozen bodies cast only along allowed displacement.
  - TOI response cannot introduce velocity on frozen axes.
- [x] Add pure 2D CCD tests with the same three assertions for planar axes.
- [x] Add mixed CCD tests for constrained 3D and constrained 2D participants.
- [x] Update 3D CCD source eligibility to use full position freeze instead of
  the old `Immovable` backing field.
- [x] Update pure 2D CCD source eligibility the same way.
- [x] Project predicted velocity and displacement through freeze constraints
  before candidate collection.
- [x] Update 3D grounding so frozen vertical movement does not create misleading
  gravity or support transitions.
- [x] Update the pure 2D grounding plan before implementation so support logic
  accounts for planar freeze constraints.

**Done Criteria**

- CCD candidates and TOI response cannot use frozen degrees of freedom.
- Grounding remains deterministic with constrained bodies.
- Active-source handoff preserves freeze rules.

## Workstream 6: Serialization, Docs, Benchmarks, And Release Validation

**Problem**

Freeze constraints replace public body state and affect hot-path solver math.
The migration needs explicit serialization, docs, and targeted performance
checks.

**Tasks**

- [x] Update `SolidBody.Serialization.cs` to record 3D freeze axes.
- [x] Update `SolidBody2D.Serialization.cs` to record 2D freeze axes.
- [x] Add save/populate tests proving freeze axes round-trip for both body
  types.
- [x] Remove old serialized `Immovable` and `PreventAngularForces` fields unless
  the implementation chooses a one-time internal migration for existing local
  test artifacts. Backward compatibility is not required.
- [x] Update `docs/wiki/HOST_INTEGRATION.md` with examples for common freeze
  configurations.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` with solver semantics for frozen
  degrees of freedom.
- [x] Update `docs/wiki/DIMENSIONS.md` with pure 2D axis naming.
- [x] Update `docs/wiki/SERIALIZATION.md` with freeze-axis state.
- [x] Add benchmark rows only for response or CCD paths if profiler or unit
  allocation tests show measurable overhead from constrained mass helpers.
- [x] Run:
  `dotnet build Gravitas.slnx --configuration Release`
- [x] Run:
  `dotnet test Gravitas.slnx --configuration Release`
- [x] Run:
  `dotnet build Gravitas.slnx --configuration ReleaseLean`
- [x] Run:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [x] Search for stale mutable APIs:
  `rg -n "Immovable\\s*=|PreventAngularForces\\s*=" src/Gravitas docs/wiki tests/Gravitas.Tests`

**Done Criteria**

- Freeze constraints are serialized explicitly.
- Docs no longer describe `Immovable` or `PreventAngularForces` as mutable
  body controls.
- Standard and Lean validation pass.

## Final Done Criteria

- 3D and pure 2D bodies expose explicit axis freeze constraints.
- Coarse mutable `Immovable` and `PreventAngularForces` state is removed, and
  the old read-only aliases are not retained.
- Discrete response, CCD, mixed response, sleep, grounding, and partition
  mobility all respect frozen degrees of freedom.
- Serialization and docs describe one clear body-constraint model.
