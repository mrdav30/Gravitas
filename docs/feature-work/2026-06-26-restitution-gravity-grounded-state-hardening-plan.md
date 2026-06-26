# Restitution Gravity And Grounded State Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace hardcoded bounce cutoffs and coarse gravity/grounded-state body hooks with explicit deterministic settings that apply consistently across 3D, pure 2D, mixed response, and CCD.

**Architecture:** Move the restitution velocity threshold into `PhysicsSettings`, route discrete and continuous response through context-owned configuration, add body-level gravity scaling instead of duplicate ignore flags, and expose previous-frame grounded state where grounding is authoritative. Keep these changes small and mechanically verifiable before broader 2D grounding work lands.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp `Fixed64`/`Vector2d`/`Vector3d`, Gravitas collision response services, Chronicler explicit state recording, MemoryPack-compatible settings serialization.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas body/response hardening

## Purpose

Gravitas currently has a physically sensible restitution cutoff, but it is
hardcoded as separate static values in the 3D, pure 2D, and mixed response
helpers. That makes it hard for hosts to tune bounce behavior consistently and
creates duplicated policy in hot-path code.

The body notes also point at two related API gaps:

- hosts need a deterministic way to disable or scale gravity per body without
  duplicating an `IgnoreGravity` boolean next to gravity values.
- grounding logic should expose whether a body was grounded on the previous
  authoritative step so hosts and future 2D grounding work can distinguish
  landing, staying grounded, and leaving support.

This plan keeps the scope tight: response threshold configuration, gravity
scale, and grounded transition state. It does not redesign body constraints;
axis freeze work is captured separately.

## Current Baseline

- `src/Gravitas/CollisionHandling/Response/3D/CollisionResponse.cs` defines
  `RestitutionVelocityThreshold = 0.25`.
- `src/Gravitas/CollisionHandling/Response/2D/CollisionResponse2D.cs` defines
  the same hardcoded threshold.
- `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
  defines the same hardcoded threshold.
- 3D dynamic CCD restitution in
  `src/Gravitas/Core/3D/StiffBody.ContinuousCollision.Dynamic.cs` reads the 3D
  static threshold.
- Pure 2D dynamic CCD restitution in
  `src/Gravitas/Core/2D/StiffBody2D.ContinuousCollision.Dynamic.cs` reads the
  2D static threshold.
- `PhysicsSettings` owns other solver-level tunables such as
  `DiscreteSolverIterations`, `ContinuousCollisionMaxToiIterations`, and
  `Mixed2DHalfThickness`.
- `PhysicsSettingsSaver` records configurable settings for deterministic
  save/apply flows.
- `StiffBody` uses context environment gravity during integration and CCD
  prediction.
- `StiffBody2D` owns a per-body planar `Gravity` vector.
- `StiffBody` exposes `IsGrounded`, but not `WasGrounded`.
- `StiffBody2D` grounding is planned in
  `docs/feature-work/2026-06-26-pure-2d-grounding-and-support-plan.md`.

## Non-Goals

- Do not add material-level restitution curves in this pass.
- Do not add duplicate `IgnoreGravity` and `GravityScale` APIs. Prefer one
  expressive control.
- Do not change the collision material model beyond routing the existing bounce
  cutoff through settings.
- Do not add 2D grounding implementation here. This plan only records the
  `WasGrounded` requirement that the 2D grounding plan should include.
- Do not introduce engine-specific character-controller or navigation checks.

## Guiding Rules

- Settings must be context-owned and deterministic.
- Threshold defaults should preserve current behavior until a host opts into a
  different value.
- Negative velocity thresholds are invalid.
- `GravityScale == Fixed64.Zero` is the per-body ignore-gravity behavior.
- Grounded transition state must update at deterministic simulation boundaries,
  not during visualization.
- Save/load should preserve authoritative body and settings state required for
  deterministic continuation.

## Proposed API Shape

The exact names should be finalized during Workstream 1, but the intended shape
is:

- `PhysicsSettings.DefaultRestitutionVelocityThreshold`
- `PhysicsSettings.RestitutionVelocityThreshold`
- `StiffBody.GravityScale`
- `StiffBody2D.GravityScale`
- `StiffBody.WasGrounded`
- `StiffBody2D.WasGrounded` as part of the pure 2D grounding plan

`GravityScale` should default to `Fixed64.One`. A zero scale disables gravity
for that body. Values greater than one intentionally allow stronger gravity.
Reject negative values unless a concrete reverse-gravity use case is designed
with tests.

Remove the duplicated hardcoded response thresholds once all call sites receive
the context setting. If a private helper constant remains during migration, it
should live in `PhysicsSettings`, not in dimension-specific response types.

## Workstream 1: Settings-Driven Restitution Threshold

**Problem**

The current bounce threshold is a duplicated static response helper value. It
should be a first-class context setting so 3D, pure 2D, mixed, and CCD response
share one deterministic policy.

**Tasks**

- [ ] Add a failing settings test in
  `tests/Gravitas.Tests/Settings/PhysicsSettingsTests.cs` or the nearest
  existing settings test file:
  - default threshold equals `(Fixed64)0.25`.
  - setting a positive threshold stores the value.
  - setting zero is allowed and means every positive closing speed can bounce.
  - setting a negative threshold throws.
- [ ] Add `public static readonly Fixed64 DefaultRestitutionVelocityThreshold`
  to `src/Gravitas/Settings/PhysicsSettings.cs`.
- [ ] Add a private backing field and public property:
  `public Fixed64 RestitutionVelocityThreshold`.
- [ ] Validate with `SwiftThrowHelper.ThrowIfArgument(value < Fixed64.Zero, ...)`.
- [ ] Add `Fixed64? RestitutionVelocityThreshold` to
  `src/Gravitas/Settings/PhysicsSettingsSaver.cs`.
- [ ] Update `CreateSettings()` so saved values apply after construction.
- [ ] Add JSON and MemoryPack serialization coverage for the new settings
  field using the existing settings serialization tests.
- [ ] Run focused settings tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter PhysicsSettings`

**Done Criteria**

- Restitution threshold is context-owned.
- Settings save/apply flows preserve configured thresholds.
- Invalid negative thresholds fail before they can affect simulation.

## Workstream 2: Discrete 3D, 2D, And Mixed Response Integration

**Problem**

Discrete response should not read dimension-specific static threshold values.
Every response path should use the same context setting.

**Tasks**

- [ ] Add focused 3D response tests in
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`:
  - closing speed below the configured threshold produces no restitution bounce.
  - closing speed above the configured threshold applies restitution.
  - changing `context.Settings.RestitutionVelocityThreshold` changes the result.
- [ ] Add pure 2D manifold response tests in
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DManifoldTests.cs`
  or the nearest response file with the same three assertions.
- [ ] Add mixed response tests in
  `tests/Gravitas.Tests/MixedDimensions/MixedResponseTests.cs` with the same
  three assertions.
- [ ] Thread `PhysicsSettings.RestitutionVelocityThreshold` into the 3D response
  calculation through the existing pair, island, or context call path.
- [ ] Thread the same setting into pure 2D response without adding per-contact
  allocations.
- [ ] Thread the same setting into mixed response.
- [ ] Remove `CollisionResponse.RestitutionVelocityThreshold`,
  `CollisionResponse2D.RestitutionVelocityThreshold`, and
  `CollisionResponseMixed.RestitutionVelocityThreshold` once no call sites need
  them.
- [ ] Run focused response tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter CollisionResponse`

**Done Criteria**

- All discrete response dimensions use the context setting.
- The old static policy duplication is gone.
- Test coverage proves low-speed contacts do not bounce and higher-speed
  contacts still can.

## Workstream 3: Continuous-Collision Restitution Integration

**Problem**

CCD restitution currently reads the same static response thresholds as discrete
response. That would leave a split policy after Workstream 2 unless CCD is
updated at the same time.

**Tasks**

- [ ] Add 3D CCD tests in
  `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
  proving the configured threshold controls bounce after TOI resolution.
- [ ] Add pure 2D CCD tests in
  `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs` proving the
  configured threshold controls bounce after TOI resolution.
- [ ] Add mixed CCD tests only if the mixed dynamic path applies restitution in
  the current implementation; otherwise document the non-use in the test name
  that verifies no stale static threshold remains.
- [ ] Replace static threshold reads in
  `src/Gravitas/Core/3D/StiffBody.ContinuousCollision.Dynamic.cs` with the
  context setting.
- [ ] Replace static threshold reads in
  `src/Gravitas/Core/2D/StiffBody2D.ContinuousCollision.Dynamic.cs` with the
  context setting.
- [ ] Search with
  `rg -n "RestitutionVelocityThreshold" src/Gravitas tests/Gravitas.Tests`
  and ensure remaining references are settings, tests, or docs.
- [ ] Run focused CCD tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter ContinuousCollision`

**Done Criteria**

- CCD and discrete response share one restitution cutoff.
- No static dimension-specific threshold remains.
- CCD tests cover low-speed non-bounce behavior.

## Workstream 4: Body Gravity Scale

**Problem**

Hosts need per-body gravity control. A boolean `IgnoreGravity` is tempting, but
it duplicates state once partial gravity is needed. A deterministic
`GravityScale` is the stronger API because zero covers ignore-gravity and
positive values cover common gameplay tuning.

**Tasks**

- [ ] Add 3D integration tests in
  `tests/Gravitas.Tests/Core/StiffBodyIntegrationTests.cs`:
  - default `GravityScale` preserves current gravity behavior.
  - `GravityScale = Fixed64.Zero` prevents environment gravity from changing
    velocity.
  - `GravityScale = Fixed64.Half` applies half gravity.
  - negative values throw.
- [ ] Add pure 2D integration tests in
  `tests/Gravitas.Tests/Physics2D/Physics2DSimulationTests.cs` or
  `tests/Gravitas.Tests/Core/StiffBody2DAngularDynamicsTests.cs`:
  - default scale preserves current per-body `Gravity`.
  - zero scale prevents planar gravity from changing velocity.
  - half scale applies half planar gravity.
  - negative values throw.
- [ ] Add `GravityScale` to `src/Gravitas/Core/3D/StiffBody.cs` with
  `Fixed64.One` default and negative-value validation.
- [ ] Apply `GravityScale` in 3D integration in
  `src/Gravitas/Core/3D/StiffBody.Motion.cs`.
- [ ] Apply `GravityScale` in 3D CCD prediction in
  `src/Gravitas/Core/3D/StiffBody.ContinuousCollision.cs`.
- [ ] Add `GravityScale` to `src/Gravitas/Core/2D/StiffBody2D.cs` with the same
  validation.
- [ ] Apply `GravityScale` in pure 2D integration and CCD prediction.
- [ ] Record `GravityScale` in `StiffBody.Serialization.cs` and
  `StiffBody2D.Serialization.cs`.
- [ ] Add serialization tests proving save/populate preserves the scale for both
  body types.
- [ ] Update docs in `docs/wiki/HOST_INTEGRATION.md`,
  `docs/wiki/RUNTIME_ARCHITECTURE.md`, and `docs/wiki/SERIALIZATION.md`.

**Done Criteria**

- Gravity can be disabled or scaled per body without adding an extra boolean
  API.
- 3D and pure 2D body behavior match their existing gravity ownership models.
- Save/load preserves gravity scaling.

## Workstream 5: Grounded Transition State

**Problem**

Hosts and solver hardening need to distinguish previous-frame grounded state
from current grounded state. That is useful for landing events, leave-ground
behavior, and the planned pure 2D grounding model.

**Tasks**

- [ ] Add 3D grounding tests in
  `tests/Gravitas.Tests/Core/StiffBodyGroundingTests.cs` or the nearest
  grounding test file:
  - `WasGrounded` is false before the first successful ground check.
  - `WasGrounded` is true on the frame after a grounded check succeeds.
  - `WasGrounded` remains true for the authoritative step where the body loses
    support and `IsGrounded` becomes false.
  - manual grounding updates `WasGrounded` deterministically.
- [ ] Add `public bool WasGrounded { get; private set; }` to the 3D grounding
  partial.
- [ ] Update `WasGrounded` exactly once per authoritative grounding refresh
  before changing `IsGrounded`.
- [ ] Ensure disabled grounding clears `IsGrounded` while preserving the
  previous value long enough for the current step's transition to be observable.
- [ ] Decide whether `WasGrounded` must be serialized for deterministic
  continuation. If landing/leave-ground events can be replayed differently
  after load without it, record the field in `StiffBody.Serialization.cs`.
- [ ] Verify the pure 2D grounding plan still carries `WasGrounded` state and
  transition tests before implementation starts.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md` to describe the frame boundary for
  `IsGrounded` and `WasGrounded`.

**Done Criteria**

- 3D grounded transitions are directly observable.
- The pure 2D grounding plan carries the same transition-state requirement.
- Serialization behavior is explicit.

## Workstream 6: Docs, Benchmarks, And Release Validation

**Problem**

These changes touch public settings, body state, response semantics, and
serialization. Docs and validation need to cover both standard and Lean builds.

**Tasks**

- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` so restitution threshold is
  described as `PhysicsSettings.RestitutionVelocityThreshold`.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md` with gravity-scale examples for 3D
  and pure 2D hosts.
- [ ] Update `docs/wiki/SERIALIZATION.md` with new settings/body fields.
- [ ] Update `docs/wiki/DIMENSIONS.md` only if the 2D grounding plan is amended
  in this pass.
- [ ] Add benchmark rows only if response threshold routing or gravity scaling
  changes a hot path in a measurable way. Otherwise record no benchmark delta in
  the workstream summary.
- [ ] Run:
  `dotnet build Gravitas.slnx --configuration Release`
- [ ] Run:
  `dotnet test Gravitas.slnx --configuration Release`
- [ ] Run:
  `dotnet build Gravitas.slnx --configuration ReleaseLean`
- [ ] Run:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [ ] Search for stale policy:
  `rg -n "RestitutionVelocityThreshold|IgnoreGravity|WasGrounded|GravityScale" src/Gravitas docs/wiki tests/Gravitas.Tests`

**Done Criteria**

- Public docs describe the settings-driven restitution policy and body gravity
  control.
- Release and Lean builds pass.
- No stale hardcoded restitution threshold remains.

## Final Done Criteria

- Restitution cutoff is one context setting used by 3D, pure 2D, mixed, and CCD
  response paths.
- `GravityScale` provides the body-level ignore/scale behavior without a
  duplicate `IgnoreGravity` API.
- Grounded transition state is explicit for 3D and carried into the pure 2D
  grounding plan.
- Settings, body serialization, docs, tests, and release validation are aligned.
