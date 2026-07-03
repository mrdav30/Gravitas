# Rotated Cone Projection And Query Bounds Hardening Plan

**Date:** 2026-07-02  
**Status:** Planned  
**Owner:** Gravitas cone geometry, query, and mixed reducer hardening

---

> **For agentic workers:** Treat this as a living context guide. Update progress
> as workstreams complete, and move genuinely deferred discoveries into their
> own plan or the evergreen trackers instead of leaving vague wiki caveats
> behind.

**Goal:** Promote rotated finite-cone mixed slab queries from safe conservative
fallbacks to benchmark-backed exact reducers where deterministic cone geometry
supports it, and tighten cone query/collider bounds when evidence shows a
candidate-count or precision win.

**Architecture:** Keep cone geometry analytic, fixed-point, allocation-free, and
shared across the physical cone collider, 3D cone-volume queries, and mixed
finite-slab reducers. Preserve `PhysicsQueryReducerKind.ConservativeFallback`
only for paths that remain deliberately unsupported after measured evidence.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp
`Fixed64`/`Vector2d`/`Vector3d`/`FixedQuaternion`/bounds primitives,
SwiftCollections scratch buffers, GridForge-backed partitions, Gravitas 3D and
mixed query services.

## Purpose

`LSConeCollider` is already a first-class analytic 3D primitive. It has
shape-derived mass properties, support mapping, ray/sweep/query integration,
mixed mode participation, diagnostics, serialization, and benchmark smoke.

One remaining asymmetry is intentionally called out in the current mixed query
surface: `QueryMixed.SweepCircleAgainst3D` treats vertical finite-cone targets
as exact finite-slab reductions, but rotated finite-cone targets use whole-cone
projection and report `PhysicsQueryReducerKind.ConservativeFallback`. That is
safe and honest, but it can report earlier or extra hits when a tilted cone's
whole X/Z projection overlaps the swept 2D slab even though the cone volume
clipped to that slab does not.

This plan owns the investigation and, if supported by tests and benchmarks, the
promotion of rotated cone slab reducers to exact. It also captures adjacent cone
bound work because tighter analytic cone AABBs can benefit physical cone
partitioning and 3D cone-volume query candidate gathering without changing
public semantics.

## Current Baseline

- `LSConeCollider` stores world base center, apex, axis, height, radius, volume,
  center of mass offset, and inertia tensor in
  `src/Gravitas/Colliders/3D/LSConeCollider.cs`.
- `ConvexColliderSupport` supports finite cones for GJK-style intersection,
  support-mapped source sweeps, and cone-volume overlap tests.
- `FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(...)` is exact only when
  the cone axis is world-Y vertical.
- `FiniteSlabProjectionSweep.TrySweepCircleAgainstConeWholeProjection(...)`
  provides the rotated-cone conservative fallback.
- `GravitasQueryMixedService.ClassifySweepCircleAgainst3DReducer(...)` reports
  rotated cones and compounds containing rotated cones as
  `ConservativeFallback`.
- `Query3D.OverlapCone*` uses analytic cone-volume checks, but its broad-phase
  query bounds are the simple apex/end-plus-radius AABB.
- `LSConeCollider` currently relies on the base collider's generic rotated
  bounding-box path, which is safe but overestimates the actual finite cone
  volume near the apex.

## Non-Goals

- Do not implement full EPA or replace existing analytic/SAT/manifold paths as
  part of this plan.
- Do not approximate cone geometry with generated runtime mesh triangles.
- Do not use floating-point trigonometry, sampling, or platform-dependent
  ordering in runtime reducers.
- Do not remove `ConservativeFallback` as a public reducer kind. It remains the
  correct label for genuinely conservative paths.
- Do not broaden this plan into general cone contact manifold redesign unless a
  direct defect is discovered while working the cone projection/bounds paths.

## Workstream 1: Baseline, Stress Cases, And Evidence Shape

**Status:** Planned

**Problem**

Before changing reducer math, we need tests and benchmark rows that prove the
current rotated-cone fallback is either acceptable or worth replacing.

**Tasks**

- [ ] Inventory current cone-sensitive paths:
  - `src/Gravitas/Colliders/3D/LSConeCollider.cs`
  - `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
  - `src/Gravitas/Queries/Mixed/FiniteSlabProjectionSweep.cs`
  - `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.CircleAgainst3DReducers.cs`
  - `src/Gravitas/Queries/3D/GravitasQuery3DService.Cone.cs`
- [ ] Add mixed query tests that demonstrate the current rotated-cone
      conservative fallback policy:
  - a rotated cone that hits and reports `ConservativeFallback`.
  - a rotated cone where whole-cone projection can report an earlier distance
    than the finite slab slice should.
  - a rotated cone where a whole-cone projection hit should be rejected once the
    finite slab slice is exact.
  - a compound containing a rotated cone part so owner reduction and reducer
    kind classification are covered.
- [ ] Add benchmark rows in
      `tests/Gravitas.Benchmarks/Queries/MixedQueryBenchmarks.cs`:
  - dense vertical cone targets.
  - dense rotated long/narrow cone targets.
  - dense rotated short/fat cone targets.
  - false-positive-heavy rotated cone targets.
  - candidate count rows for each scenario.
- [ ] Add benchmark rows in
      `tests/Gravitas.Benchmarks/Queries/QueryServiceBenchmarks.cs`:
  - `OverlapConeAll` long/narrow query volume across dense targets.
  - `OverlapConeAll` short/fat query volume across dense targets.
  - candidate count rows if the benchmark harness exposes them cleanly.
- [ ] Record local baseline numbers in this plan before implementation changes.

**Done Criteria**

- The exact behavior gap is demonstrated by tests or closed with evidence.
- Baseline mixed/query cone rows exist before reducer or bounds changes.
- The plan contains enough measured context to judge later changes.

## Workstream 2: Shared Analytic Cone Geometry

**Status:** Planned

**Problem**

Cone math is currently distributed across collider state, support mapping, mixed
reducers, and 3D query helpers. Promoting rotated slab support should not create
another private cone formula that diverges from the physical collider.

**Tasks**

- [ ] Create a focused internal helper, likely
      `src/Gravitas/Colliders/3D/ConeGeometry.cs`, for reusable deterministic
      cone calculations that are not tied to collider lifecycle.
- [ ] Include helpers for:
  - apex/base/axis frame normalization.
  - radius at axial fraction or local height.
  - exact finite cone AABB from apex, base center, axis, height, and base
    radius.
  - support point for a finite cone in a world direction.
  - finite cone Y-slab interval checks.
- [ ] Route `LSConeCollider` and query code through the helper where it reduces
      duplication without hiding collider-owned runtime state.
- [ ] Add unit tests for:
  - vertical cone AABB equals the expected base disk plus apex extent.
  - rotated cone AABB contains apex and deterministic base disk extreme points.
  - support point tie-breaking remains stable for zero or axis-aligned
    directions.
  - radius interpolation remains exact for base, midpoint, and apex.

**Done Criteria**

- Cone geometry formulas live in one deterministic helper where reuse is real.
- The helper introduces no managed allocations in tests after warmup.
- Existing cone behavior remains unchanged before reducer promotion begins.

## Workstream 3: Exact Rotated Cone Finite-Slab Projection

**Status:** Planned

**Problem**

The X/Z projection of a finite cone clipped to a world-Y slab is convex, but the
support extrema for a rotated cone are more complex than the vertical case. The
runtime reducer needs a bounded, deterministic support function, not sampling.

**Tasks**

- [ ] Derive the finite-slab support candidates for a rotated cone in a fixed
      planar direction:
  - apex when inside the slab.
  - base disk clipped by the slab.
  - side surface extrema inside the slab.
  - slab boundary intersections with the cone side.
  - deterministic tie-breaks for equal support projections.
- [ ] Prototype the support candidate set behind tests in
      `FiniteSlabProjectionSweep` or a focused helper such as
      `ConeSlabProjectionGeometry`.
- [ ] Add tests comparing vertical cone exact results against the current
      vertical fast path to preserve existing behavior.
- [ ] Add tests for tilted cones:
  - reject proxy-only hits.
  - preserve true hits.
  - report stable earliest distance.
  - handle narrow/long and short/fat cones.
  - handle zero-thickness and thick mixed slabs.
- [ ] Replace rotated-cone whole-projection sweep with the exact projection when
      tests prove the support function is complete.
- [ ] Update `ClassifySweepCircleAgainst3DReducer(...)` so rotated cones and
      compounds containing rotated cones report `Exact` only after the exact
      reducer is active.
- [ ] Keep a conservative fallback only if the exact derivation exposes a
      deterministic unsupported shape state; document that state in this plan
      and in `docs/wiki/QUERY_SERVICES.md`.

**Done Criteria**

- `QueryMixed.SweepCircleAgainst3D` treats supported rotated finite cones as
  exact finite-slab reducers.
- Proxy-only rotated cone hits are rejected without false negatives.
- The reducer remains bounded, allocation-free, and deterministically ordered.

## Workstream 4: Cone Bounds And Candidate Reduction

**Status:** Planned

**Problem**

Even if mixed reducer precision is fixed, broad-phase candidate cost can stay
higher than necessary if cone bounds are still cylinder-box shaped. Tight
analytic cone bounds can reduce partition churn and query candidate counts.

**Tasks**

- [ ] Override or refine cone collider bounds in `LSConeCollider` using the
      shared exact cone AABB helper instead of the generic full-box rotated
      bounds, if tests confirm it is always conservative.
- [ ] Update tests proving the cone AABB contains:
  - apex.
  - base center.
  - base disk extremes for X, Y, and Z axes.
  - representative side surface points.
- [ ] Tighten `Query3D.OverlapCone*` broad-phase bounds in
      `GravitasQuery3DService.Cone.cs` using the same finite cone AABB formula.
- [ ] Compare candidate counts and timings for the Workstream 1 cone benchmark
      rows before and after bound tightening.
- [ ] Confirm partition and query behavior remain deterministic when multiple
      cone candidates share equal distances or equal collider IDs.

**Done Criteria**

- Physical cone collider bounds are no larger than needed while remaining
  conservative.
- Cone-volume query bounds use analytic finite-cone extents.
- Benchmarks show the candidate-count effect or document a measured no-change
  decision.

## Workstream 5: Docs, Diagnostics, And Release Validation

**Status:** Planned

**Problem**

If rotated cone reducers become exact, docs must stop teaching the old fallback
policy. If evidence says to keep a fallback, docs must explain the precise
boundary without making the feature sound unfinished.

**Tasks**

- [ ] Update `docs/wiki/QUERY_SERVICES.md`:
  - mixed `SweepCircleAgainst3D` reducer policy.
  - cone-volume query bound behavior when relevant.
  - reducer kind semantics if any fallback remains.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` if physical cone bounds or mixed
      cone collision behavior changes.
- [ ] Update `docs/wiki/DIMENSIONS.md` if mixed rotated cone slab policy
      changes.
- [ ] Update this plan with benchmark before/after numbers and final reducer
      policy.
- [ ] Run focused tests:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~MixedQueryCcdTests|FullyQualifiedName~GravitasQuery3DServiceConeTests"`
  - relevant cone query/collision allocation tests.
- [ ] Run release validation:
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [ ] Run focused benchmark smoke for mixed cone and cone-volume query rows.
- [ ] Move this plan to `docs/feature-work/done` only after reducer policy,
      docs, tests, and benchmark evidence agree.

**Done Criteria**

- Wiki docs and public reducer labels describe the implemented behavior.
- Release and Lean tests pass.
- Benchmark smoke records CPU, candidate-count, and allocation results.

## Final Done Criteria

- Rotated finite-cone mixed slab sweeps are exact, or the remaining fallback is
  backed by a precise deterministic no-change decision.
- Physical cone bounds and cone-volume query bounds use shared analytic cone
  geometry where it improves candidate quality.
- Cone-specific hot paths remain allocation-free after warmup.
- Tests cover long/narrow, short/fat, vertical, rotated, thick-slab, and
  zero-thickness slab cases.
- Documentation no longer carries vague rotated-cone caveats.
