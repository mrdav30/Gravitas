# Physics Material Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace ad hoc body-owned friction and restitution coefficients with a first-class deterministic physics material model for 3D, pure 2D, mixed response, CCD, serialization, and authoring definitions.

**Architecture:** Introduce explicit material values with static friction, dynamic friction, restitution, and combine policies. Make collider surfaces the primary material owner, provide body-level defaults only where useful for ergonomic setup, and route every discrete, CCD, and mixed response path through one material resolution policy.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp `Fixed64`, Gravitas collider/body/response services, Chronicler explicit state recording, MemoryPack-compatible settings and value types.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas material/response hardening

## Purpose

Gravitas currently stores `FrictionCoefficient` and `RestitutionCoefficient`
directly on bodies. That works for early rigid body response, but it makes
surface authoring fuzzy as the engine grows:

- different colliders on the same body cannot naturally represent different
  surfaces.
- compound collider parts cannot express authored surface differences.
- static and dynamic friction share one coefficient.
- restitution and friction combine policy is implicit in response helpers.
- future ragdolls, constraints, and engine-specific adapters need a clearer
  material boundary.

This plan creates a deterministic physics material model that is explicit,
serializable, and shared across 3D, pure 2D, mixed, and CCD response.

## Current Baseline

- `SolidBody.RestitutionCoefficient` defaults to `0.5`.
- `SolidBody.FrictionCoefficient` defaults to `1`.
- `SolidBody2D.RestitutionCoefficient` defaults to `Fixed64.Half`.
- `SolidBody2D.FrictionCoefficient` defaults to `Fixed64.One`.
- `CollisionResponse`, `CollisionResponse2D`, and `CollisionResponseMixed`
  resolve restitution as the minimum of body coefficients.
- Friction currently combines with `sqrt(bodyA * bodyB)` when both bodies are
  present.
- Body serialization records the two coefficients.
- `ColliderShapeDefinition`, compound parts, and colliders do not own a public
  material value.
- The restitution threshold is context-owned through
  `PhysicsSettings.RestitutionVelocityThreshold`; see
  `done/2026-06-26-restitution-gravity-grounded-state-hardening-plan.md`.

## Non-Goals

- Do not add audio, visual, particle, decal, or gameplay surface tags in this
  plan.
- Do not add anisotropic friction unless a measured use case appears.
- Do not introduce engine asset references into Gravitas materials.
- Do not preserve duplicate mutable body coefficient APIs when the material API
  replaces them.
- Do not make materials process-global registries. Material values should be
  deterministic data, not hidden ambient state.

## Guiding Rules

- Material resolution must be deterministic and allocation-free.
- Collider surface material should be the primary source of contact material.
- Body-level defaults may exist for ergonomic setup, but response should not
  need to guess which body coefficient applies to which collider.
- Static and dynamic friction should be separate values.
- Combine policy should be explicit and documented.
- Defaults should preserve current behavior as closely as the stronger model
  allows.
- Materials must serialize through Chronicler and standard package serializers.

## Proposed API Shape

The exact names should be finalized during Workstream 1. The intended shape is:

```csharp
public readonly struct PhysicsMaterial : IEquatable<PhysicsMaterial>
{
    public Fixed64 StaticFriction { get; }
    public Fixed64 DynamicFriction { get; }
    public Fixed64 Restitution { get; }
    public PhysicsMaterialCombine FrictionCombine { get; }
    public PhysicsMaterialCombine RestitutionCombine { get; }
}

public enum PhysicsMaterialCombine
{
    Minimum,
    Maximum,
    Average,
    Multiply,
    GeometricMean
}
```

Suggested defaults:

- `PhysicsMaterial.Default`:
  - static friction `Fixed64.One`
  - dynamic friction `Fixed64.One`
  - restitution `Fixed64.Half`
  - friction combine `GeometricMean`
  - restitution combine `Minimum`
- `PhysicsMaterial.Frictionless`
- `PhysicsMaterial.Bouncy`

The model should prefer value semantics. If a future engine adapter needs
asset-style material catalogs, that adapter can map stable asset IDs to
`PhysicsMaterial` values before simulation.

## Workstream 1: Material Value Type And Combine Policy

**Problem**

The engine needs one explicit material value and one combine policy before
response paths can migrate.

**Tasks**

- [ ] Add `src/Gravitas/Materials/PhysicsMaterial.cs`.
- [ ] Add `src/Gravitas/Materials/PhysicsMaterialCombine.cs`.
- [ ] Validate values:
  - static friction cannot be negative.
  - dynamic friction cannot be negative.
  - restitution must be clamped or rejected outside `[0, 1]`; prefer rejecting
    invalid values so authoring mistakes are visible.
  - dynamic friction cannot exceed static friction unless a specific use case is
    documented and tested.
- [ ] Add deterministic combine helpers:
  - `CombineFriction(...)`
  - `CombineRestitution(...)`
  - `CombineScalar(...)`
- [ ] Preserve current default behavior with `GeometricMean` friction and
  `Minimum` restitution.
- [ ] Add tests in `tests/Gravitas.Tests/Materials/PhysicsMaterialTests.cs`:
  - defaults match current coefficients.
  - invalid values throw.
  - each combine policy returns expected fixed-point results.
  - equal material values compare equal.
- [ ] Add XML docs for units, ranges, and combine semantics.

**Done Criteria**

- `PhysicsMaterial` is deterministic value data.
- Combine policy is explicit and tested.
- Defaults preserve current response behavior.

## Workstream 2: Collider And Shape Material Ownership

**Problem**

Materials describe surfaces. Colliders and authored shape definitions should
own surface material before body response migrates.

**Tasks**

- [ ] Add `PhysicsMaterial Material` to `LSCollider`.
- [ ] Add `PhysicsMaterial Material` to `LSCollider2D`.
- [ ] Add material to `ColliderShapeDefinition` and
  `ColliderShapeDefinition2D`.
- [ ] Add material to `CompoundColliderPart` and `CompoundColliderPart2D`.
- [ ] Ensure `LSCompoundCollider` and `LSCompoundCollider2D` materialize part
  colliders with the part material.
- [ ] Define owner material fallback for compound parts that omit material:
  use the compound owner material at materialization time.
- [ ] Add tests proving:
  - standalone colliders use their assigned material.
  - shape definitions materialize colliders with the same material.
  - compound parts keep distinct materials.
  - compound owner material is used only when a part does not supply its own
    material.
- [ ] Update serialization for `LSCollider` and `LSCollider2D` material state.

**Done Criteria**

- Surface material lives on colliders and authored shape data.
- Compound parts can carry distinct material.
- Material state survives save/populate.

## Workstream 3: Body API Migration

**Problem**

Body-owned coefficients should not remain as a second mutable material model.
The migration needs a clean ergonomic replacement.

**Tasks**

- [ ] Add a body-level default material property only if it materially improves
  setup ergonomics, such as `DefaultMaterial`.
- [ ] If body defaults are added, define when they copy to colliders:
  - during body/collider setup.
  - through an explicit method such as `ApplyMaterialToColliders(...)`.
  - never implicitly during response.
- [ ] Remove mutable `RestitutionCoefficient` and `FrictionCoefficient` from
  bodies after response paths migrate.
- [ ] Replace tests that set body coefficients with collider or material
  assignment.
- [ ] Update scenario builders so material setup is concise and explicit.
- [ ] Update body serialization tests so material state is recorded on the
  collider surface that owns it.
- [ ] Search for stale body coefficient assignments:
  `rg -n "RestitutionCoefficient|FrictionCoefficient" src/Gravitas tests/Gravitas.Tests docs/wiki`

**Done Criteria**

- There is one public material model.
- Bodies no longer own independent friction/restitution truth.
- Tests and helpers make material ownership clear.

## Workstream 4: Discrete Response And Resting Friction Integration

**Problem**

3D, pure 2D, and mixed response must resolve material from contacts and use
static/dynamic friction coherently.

**Tasks**

- [ ] Add 3D response tests:
  - restitution comes from collider material combine policy.
  - low-speed restitution threshold still suppresses bounce.
  - static friction holds resting tangential motion up to the static limit.
  - dynamic friction applies once tangential impulse exceeds static limit.
- [ ] Add pure 2D response tests with the same material cases.
- [ ] Add mixed response tests proving 3D and 2D collider materials combine
  correctly.
- [ ] Update `SolverContact`, `SolverContact2D`, and mixed response helpers to
  carry or resolve material values from colliders.
- [ ] Route `CollisionResponse` through material combine helpers.
- [ ] Route `CollisionResponse2D` through the same material policy.
- [ ] Route `CollisionResponseMixed` through the same material policy.
- [ ] Ensure warm-started friction clamps against the correct static or dynamic
  coefficient.
- [ ] Keep all material resolution allocation-free.

**Done Criteria**

- All discrete response paths use collider materials.
- Static and dynamic friction have clear behavior.
- Restitution threshold remains a settings-level policy, not hidden material
  behavior.

## Workstream 5: CCD, Queries, Serialization, And Docs

**Problem**

CCD restitution and serialized collider state must use the new material model.
Queries should report colliders as before without absorbing material semantics
unless the host reads the collider material from hits.

**Tasks**

- [ ] Update 3D CCD restitution to resolve source/target collider material.
- [ ] Update pure 2D CCD restitution the same way.
- [ ] Update mixed CCD restitution where applicable.
- [ ] Verify `Physics3DHit`, `Physics2DHit`, and `PhysicsMixedHit` expose enough
  collider identity for hosts to read material without duplicating material into
  every hit payload.
- [ ] Add serialization tests for:
  - 3D collider material.
  - 2D collider material.
  - compound part material.
  - shape definition material.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md`.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md`.
- [ ] Update `docs/wiki/SERIALIZATION.md`.
- [ ] Update `docs/wiki/DIMENSIONS.md` for pure 2D and mixed material behavior.

**Done Criteria**

- CCD and discrete response share material resolution.
- Material state serializes explicitly.
- Query hits continue to identify colliders without bloating hit structs.

## Workstream 6: Benchmarks And Release Validation

**Problem**

Material resolution is a contact hot path. The final model must preserve
deterministic behavior and avoid avoidable overhead.

**Tasks**

- [ ] Add or update response benchmarks:
  - default material 3D contacts.
  - distinct material 3D contacts.
  - pure 2D manifold material contacts.
  - mixed material contacts.
  - compound part material contacts.
- [ ] Add allocation guardrails proving steady-state material response allocates
  `0` bytes after warmup.
- [ ] Run focused material tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Material`
- [ ] Run response tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter CollisionResponse`
- [ ] Run full validation:
  `dotnet test Gravitas.slnx --configuration Release`
- [ ] Run Lean validation:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- Material response has tests, docs, and benchmark signal.
- No steady-state material hot path allocations remain.
- Release and Lean builds pass.

## Final Done Criteria

- Gravitas exposes a first-class deterministic `PhysicsMaterial` value.
- 3D, pure 2D, mixed, CCD, compound, and shape-definition paths use one material
  policy.
- Body-owned friction/restitution coefficients are removed or reduced to
  explicit setup helpers that do not participate directly in response.
- Static friction, dynamic friction, restitution, and combine behavior are
  documented, serialized, and tested.
