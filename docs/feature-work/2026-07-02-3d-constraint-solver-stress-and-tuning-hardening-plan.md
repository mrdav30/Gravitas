# 3D Constraint Solver Stress And Tuning Hardening Plan

**Date:** 2026-07-02  
**Status:** Planned  
**Owner:** Gravitas 3D constraint, ragdoll, and solver-quality hardening

---

> **For agentic workers:** Treat this as a living context guide. Update progress
> as workstreams complete, and move genuinely deferred discoveries into their
> own plan or the evergreen trackers instead of leaving vague wiki caveats
> behind.

**Goal:** Prove the current 3D constraint/ragdoll foundation under first-class
stress cases, then add explicit tuning only where measured stability evidence
shows the existing internal bias model is not enough.

**Architecture:** Keep the existing context-owned `GravitasConstraint3DService`
and contact-integrated island model. Add stress fixtures, diagnostics, and
benchmarks first; only promote new joint tuning APIs after tests demonstrate a
real stability or ergonomics gap.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp
`Fixed64`/`Vector3d`/`FixedQuaternion`/`Fixed3x3`, SwiftCollections buffers,
Gravitas 3D constraints, 3D collision/contact islands, Chronicler replay hash
contributors, and engine-agnostic diagnostics.

## Purpose

`Constraint And Ragdoll Foundation` established the right architecture:
constraints are context-owned, joints solve in the same 3D island graph as
contacts, ragdolls are authored data over ordinary `SolidBody` links, and
animation stays outside Gravitas. That boundary should stay.

The open question is not architecture; it is evidence. Real articulation
problems usually appear in long chains, humanoid-like ragdolls, motor-driven
chains, and contact-heavy stacks. This plan adds those stress cases and uses
them to decide whether Gravitas needs explicit public joint stabilization
tuning, richer diagnostics, or only better test/benchmark coverage around the
current solver.

## Current Baseline

- `src/Gravitas/Constraints/3D/GravitasConstraint3DService.cs` owns 3D joint
  IDs, ragdoll runtimes, linked-collider filtering, motor handoff, and replay
  hash contribution.
- `src/Gravitas/Constraints/3D/JointSolver3D.cs` prepares and solves joint rows
  with internal bias behavior.
- `src/Gravitas/Constraints/3D/JointMotor3D.cs` exposes angular drive strength,
  damping, and maximum motor impulse for motorized joints.
- 3D discrete islands include contact pairs and enabled `Joint3D` constraints.
- Constraint tests cover registration, filtering, solver integration,
  serialization, diagnostics, and basic ragdoll activation.
- `tests/Gravitas.Benchmarks/Core/Constraint3DBenchmarks.cs` covers a chain
  solve and ragdoll activation, but not richer humanoid/contact/motor stress.

## Non-Goals

- Do not replace the current 3D constraint architecture.
- Do not add public stiffness/compliance/damping APIs before stress evidence
  proves the need and shape.
- Do not move FootIK, HandIK, animation curve sampling, pose blending, or engine
  animator concepts into Gravitas.
- Do not split this into pure 2D constraints. Pure 2D gets its own release-scope
  plan with native planar semantics.
- Do not use floating-point math, background threads, or nondeterministic
  ordering in stress fixtures or solver changes.

## Workstream 1: Stress Fixture Inventory And Baselines

**Status:** Planned

**Problem**

The current constraint foundation has focused tests, but the stress scenarios
that usually reveal articulation weakness are still thin.

**Tasks**

- [ ] Inventory the current 3D constraint and island paths:
  - `src/Gravitas/Constraints/3D/GravitasConstraint3DService.cs`
  - `src/Gravitas/Constraints/3D/Joint3D.cs`
  - `src/Gravitas/Constraints/3D/JointSolver3D.cs`
  - `src/Gravitas/Constraints/3D/JointConstraintRow3D.cs`
  - `src/Gravitas/Core/3D/GravitasPhysicsService.cs`
  - `src/Gravitas/CollisionHandling/Response/3D`
- [ ] Add reusable test helpers for articulated 3D scenes:
  - a long chain with ball-socket joints.
  - a hinge chain with alternating axes.
  - a humanoid-ish torso/limb ragdoll graph.
  - a motor-driven chain with target rotations.
  - a contact-heavy ragdoll resting on stacked/static geometry.
- [ ] Add baseline correctness tests that run each fixture for repeated frames
      and assert:
  - no invalid fixed-point values.
  - stable deterministic replay hash across repeated runs.
  - bounded anchor error after settling.
  - sleeping/waking propagates through linked bodies.
  - no allocations after warmup for steady-state solve.
- [ ] Add benchmark rows in
      `tests/Gravitas.Benchmarks/Core/Constraint3DBenchmarks.cs`:
  - `SimulateLongConstraintChain`.
  - `SimulateHumanoidRagdollResting`.
  - `SimulateContactHeavyRagdollStack`.
  - `SimulateMotorDrivenConstraintChain`.
  - allocation/counter rows where the harness supports them cleanly.
- [ ] Record baseline benchmark and allocation numbers in this plan before
      solver changes.

**Done Criteria**

- Constraint stress fixtures exist before tuning changes.
- Baseline tests describe the current solver quality instead of assuming it.
- Benchmark rows cover chain length, contacts, motors, and activation pressure.

## Workstream 2: Solver Diagnostics And Error Visibility

**Status:** Planned

**Problem**

If a ragdoll jitters or a chain stretches, developers need deterministic error
signals. Raw body positions are not enough to diagnose whether the issue is
anchor drift, angular limit error, motor overshoot, or contact interaction.

**Tasks**

- [ ] Add internal solver counters for each active joint solve:
  - prepared row count.
  - linear anchor error magnitude.
  - angular limit error magnitude.
  - accumulated impulse magnitude.
  - motor impulse magnitude.
  - clamped row count.
- [ ] Route the counters through disabled-by-default diagnostics without
      allocations when diagnostics are off.
- [ ] Extend `GravitasDiagnosticEvent` or diagnostic views only if the existing
      joint diagnostic payload cannot represent the new counters cleanly.
- [ ] Add tests proving diagnostics remain deterministic and disabled
      diagnostics allocate `0` bytes after warmup.
- [ ] Add tests that compare diagnostic error trends for:
  - a stable resting ragdoll.
  - an overdriven motor chain.
  - a long chain with insufficient solver iterations.

**Done Criteria**

- Stress failures can be attributed to a specific joint/error category.
- Diagnostics do not perturb authoritative simulation order or allocation
  behavior.
- Solver counters are stable enough to use as benchmark/run evidence.

## Workstream 3: Evidence-Gated Stabilization API Decision

**Status:** Planned

**Problem**

The current solver has internal bias behavior. A first-class public API should
not expose vague tuning knobs unless measured stress cases prove the knobs solve
real problems and can be named clearly.

**Tasks**

- [ ] Run Workstream 1 stress fixtures using current defaults and record:
  - max anchor error.
  - max angular error.
  - settle frame count.
  - average solve time.
  - allocation count.
- [ ] Decide whether current defaults are sufficient for release:
  - if yes, document the no-change decision in this plan and keep the public API
    smaller.
  - if no, introduce an explicit value type such as `JointStabilization3D` or
    `JointSolverTuning3D`.
- [ ] If tuning is needed, keep the API fixed-point and physically explainable:
  - linear bias factor.
  - angular bias factor.
  - compliance or softness.
  - damping ratio or damping factor.
  - maximum correction velocity or maximum stabilization impulse.
- [ ] Attach tuning at the correct scope based on evidence:
  - per-joint for authored ragdoll differences.
  - context settings for global solver defaults.
  - both only if tests prove both scopes are necessary.
- [ ] Add validation tests for all public tuning values:
  - negative values fail.
  - defaults reproduce current behavior.
  - extreme but valid values remain bounded and deterministic.
- [ ] Add serialization and replay hash coverage if tuning becomes runtime
      state.

**Done Criteria**

- Public tuning either exists because evidence demands it, or the no-change
  decision is documented with stress data.
- Any added API has explicit units/invariants and deterministic defaults.
- Existing ragdoll definitions remain coherent without hidden magic constants.

## Workstream 4: Long-Chain, Motor, And Contact-Heavy Hardening

**Status:** Planned

**Problem**

Stress evidence may reveal concrete solver issues that do not require new API:
row ordering, warm-start invalidation, motor clamping, sleep propagation, or
contact/joint solve sequencing.

**Tasks**

- [ ] Fix any measured row-ordering instability by sorting joints and rows using
      documented context-owned IDs.
- [ ] Fix any warm-start leakage when joint type, limits, motor payload, or
      linked body mobility changes.
- [ ] Fix any motor impulse overshoot by clamping through fixed-point row bounds
      rather than post-solve pose correction.
- [ ] Fix any contact-heavy ragdoll jitter by adjusting row preparation order
      only when tests prove the ordering effect.
- [ ] Fix any sleep/wake issue so connected dynamic bodies sleep and wake as one
      articulation island when joints are enabled.
- [ ] Re-run the Workstream 1 stress tests and benchmark rows after each solver
      change.
- [ ] Record before/after evidence in this plan.

**Done Criteria**

- Hardening changes are tied to measured stress failures.
- Solver behavior remains deterministic across repeated runs.
- No fix improves one fixture by regressing an existing contact or joint test.

## Workstream 5: Docs, Benchmarks, And Release Validation

**Status:** Planned

**Problem**

Ragdoll and joint users need to understand the actual quality envelope: which
stress cases are covered, which tuning exists, and where animation libraries
should feed motor targets.

**Tasks**

- [ ] Update `docs/wiki/RUNTIME_ARCHITECTURE.md` with any solver diagnostic or
      tuning ownership changes.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` with final joint/contact island
      solve ordering if it changes.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md` with tuning or motor guidance if
      public APIs change.
- [ ] Update `docs/wiki/DIAGNOSTICS.md` with new joint/ragdoll counters or
      views.
- [ ] Update this plan with final stress benchmark numbers and allocation
      evidence.
- [ ] Run focused tests:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~Constraint|FullyQualifiedName~Ragdoll"`
- [ ] Run release validation:
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [ ] Run focused constraint benchmark smoke.
- [ ] Move this plan to `docs/feature-work/done` only after tests, benchmarks,
      docs, and any tuning decision agree.

**Done Criteria**

- 3D constraints and ragdolls have credible stress coverage.
- Public tuning either exists with evidence or remains intentionally absent with
  evidence.
- Docs describe the final boundary without implying animation belongs in
  Gravitas.

## Final Done Criteria

- Long-chain, humanoid-ish, contact-heavy, and motor-driven 3D articulation
  fixtures are covered by tests and benchmark smoke.
- Constraint solve paths remain allocation-free after warmup.
- Diagnostics expose enough deterministic signal to debug joint instability.
- Any public tuning API is justified, fixed-point, validated, serialized when
  necessary, and replay-hashed when authoritative.
- The 3D stress results are captured as input to the pure 2D constraint plan.
