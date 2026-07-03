# Rotated Cone Projection And Query Bounds Hardening Plan

**Date:** 2026-07-02  
**Status:** Done  
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

At plan start, one remaining asymmetry was intentionally called out in the
mixed query surface: `QueryMixed.SweepCircleAgainst3D` treated vertical
finite-cone targets as exact finite-slab reductions, while rotated finite-cone
targets used whole-cone projection and reported
`PhysicsQueryReducerKind.ConservativeFallback`. That was safe and honest, but
it could report earlier or extra hits when a tilted cone's whole X/Z projection
overlapped the swept 2D slab even though the cone volume
clipped to that slab does not.

This plan owns the investigation and promotion of rotated cone slab reducers to
exact. It also captures adjacent cone bound work because tighter analytic cone
AABBs can benefit physical cone partitioning and 3D cone-volume query candidate
gathering without changing public semantics.

## Starting Baseline

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

**Status:** Completed 2026-07-02

**Problem**

Before changing reducer math, we need tests and benchmark rows that prove the
current rotated-cone fallback is either acceptable or worth replacing.

**Tasks**

- [x] Inventory current cone-sensitive paths:
  - `src/Gravitas/Colliders/3D/LSConeCollider.cs`
  - `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
  - `src/Gravitas/Queries/Mixed/FiniteSlabProjectionSweep.cs`
  - `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.CircleAgainst3DReducers.cs`
  - `src/Gravitas/Queries/3D/GravitasQuery3DService.Cone.cs`
- [x] Add mixed query tests that demonstrate the rotated-cone precision gap and
      final exact policy:
  - a rotated cone that hits and reports `Exact`.
  - a rotated cone whole-projection-only hit that is rejected by the exact
    circle-slab source sweep.
  - arbitrarily rotated capsule, cylinder, and cone targets that all report
    `Exact`.
  - rotated cone participation in the primitive finite-slab allocation guard.
- [x] Add benchmark rows in
      `tests/Gravitas.Benchmarks/Queries/MixedQueryBenchmarks.cs`:
  - dense rotated cone targets.
  - dense rotated long/narrow cone targets.
  - dense rotated short/fat cone targets.
  - candidate count row for the standard dense rotated cone scenario.
- [x] Add benchmark rows in
      `tests/Gravitas.Benchmarks/Queries/QueryServiceBenchmarks.cs`:
  - `OverlapConeAll` long/narrow query volume across dense targets.
  - `OverlapConeAll` short/fat query volume across dense targets.
  - candidate count rows.
- [x] Record local evidence in this plan.

**Done Criteria**

- The exact behavior gap is demonstrated by tests or closed with evidence.
- Baseline mixed/query cone rows exist before reducer or bounds changes.
- The plan contains enough measured context to judge later changes.

## Workstream 2: Shared Analytic Cone Geometry

**Status:** Completed 2026-07-02

**Problem**

Cone math is currently distributed across collider state, support mapping, mixed
reducers, and 3D query helpers. Promoting rotated slab support should not create
another private cone formula that diverges from the physical collider.

**Tasks**

- [x] Create a focused internal helper, likely
      `src/Gravitas/Colliders/3D/ConeGeometry.cs`, for reusable deterministic
      cone calculations that are not tied to collider lifecycle.
- [x] Include helpers for exact finite cone AABB from apex, base center, axis,
      and base radius. Radius interpolation and support mapping remain on
      `LSConeCollider`/`ConvexColliderSupport`, where they already have
      collider-specific state and tests.
- [x] Route `LSConeCollider` and query code through the helper where it reduces
      duplication without hiding collider-owned runtime state.
- [x] Add unit tests for arbitrarily rotated finite-cone bounds matching the
      shared helper.

**Done Criteria**

- Cone geometry formulas live in one deterministic helper where reuse is real.
- The helper introduces no managed allocations in tests after warmup.
- Existing cone behavior remains unchanged before reducer promotion begins.

## Workstream 3: Exact Rotated Cone Finite-Slab Projection

**Status:** Completed 2026-07-02

**Problem**

The X/Z projection of a finite cone clipped to a world-Y slab is convex, but the
support extrema for a rotated cone are more complex than the vertical case. The
runtime reducer needs a bounded, deterministic support function, not sampling.

**Tasks**

- [x] Investigate the finite-slab support candidate derivation for rotated
      cones. The direct planar support derivation is viable but creates a large
      second support-mapped solver surface for one shape.
- [x] Adopt a cleaner exact model: reuse `ConvexSweepQueryWorker` with a
      query-owned vertical circle-slab convex source for rotated finite-cone
      targets. This tests the same 3D volume represented by
      `SweepCircleAgainst3D` without sampling, runtime mesh approximation, or a
      duplicate conservative-advancement implementation.
- [x] Add tests comparing vertical cone exact results against the current
      vertical fast path to preserve existing behavior.
- [x] Add tests for tilted cones:
  - reject proxy-only hits.
  - preserve true hits.
  - report stable earliest distance.
  - handle narrow/long and short/fat cones.
  - handle thin and regular mixed slabs. Public mixed sweep APIs reject
    zero-thickness slabs, so zero-thickness support remains outside the public
    query contract.
- [x] Remove rotated-cone whole-projection sweep from the active mixed query
      dispatch.
- [x] Update `ClassifySweepCircleAgainst3DReducer(...)` so rotated cones and
      compounds containing rotated cones report `Exact` only after the exact
      reducer is active.
- [x] Keep `ConservativeFallback` as a public reducer kind for proxy-based
      dynamic CCD paths and other deliberately conservative reducers, but not
      for supported rotated finite-cone mixed queries.

**Done Criteria**

- `QueryMixed.SweepCircleAgainst3D` treats supported rotated finite cones as
  exact finite-slab reducers.
- Proxy-only rotated cone hits are rejected without false negatives.
- The reducer remains bounded, allocation-free, and deterministically ordered.

## Workstream 4: Cone Bounds And Candidate Reduction

**Status:** Completed 2026-07-02

**Problem**

Even if mixed reducer precision is fixed, broad-phase candidate cost can stay
higher than necessary if cone bounds are still cylinder-box shaped. Tight
analytic cone bounds can reduce partition churn and query candidate counts.

**Tasks**

- [x] Override or refine cone collider bounds in `LSConeCollider` using the
      shared exact cone AABB helper instead of the generic full-box rotated
      bounds, if tests confirm it is always conservative.
- [x] Update tests proving the cone AABB matches the finite-cone AABB helper for
      arbitrary rotation.
- [x] Tighten `Query3D.OverlapCone*` broad-phase bounds in
      `GravitasQuery3DService.Cone.cs` using the same finite cone AABB formula.
- [x] Compare candidate counts and timings for the Workstream 1 cone benchmark
      rows before and after bound tightening.
- [x] Confirm partition and query behavior remain deterministic when multiple
      cone candidates share equal distances or equal collider IDs.

**Done Criteria**

- Physical cone collider bounds are no larger than needed while remaining
  conservative.
- Cone-volume query bounds use analytic finite-cone extents.
- Benchmarks show the candidate-count effect or document a measured no-change
  decision.

## Workstream 5: Docs, Diagnostics, And Release Validation

**Status:** Completed 2026-07-02

**Problem**

If rotated cone reducers become exact, docs must stop teaching the old fallback
policy. If evidence says to keep a fallback, docs must explain the precise
boundary without making the feature sound unfinished.

**Tasks**

- [x] Update `docs/wiki/QUERY_SERVICES.md`:
  - mixed `SweepCircleAgainst3D` reducer policy.
  - cone-volume query bound behavior when relevant.
  - reducer kind semantics if any fallback remains.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` if physical cone bounds or mixed
      cone collision behavior changes.
- [x] Update `docs/wiki/DIMENSIONS.md` if mixed rotated cone slab policy
      changes.
- [x] Update this plan with benchmark before/after numbers and final reducer
      policy.
- [x] Run focused tests:
  - `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~MixedQueryCcdTests|FullyQualifiedName~GravitasQuery3DServiceConeTests"`
  - relevant cone query/collision allocation tests.
- [x] Run release validation:
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [x] Run focused benchmark smoke for mixed cone and cone-volume query rows.
- [x] Move this plan to `docs/feature-work/done` only after reducer policy,
      docs, tests, and benchmark evidence agree.

## Evidence Notes

- Focused mixed query tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~MixedQueryCcdTests"`
  passed with 71 tests, including rotated finite-cone exact hits, proxy-only
  miss rejection, and the primitive finite-slab allocation guard.
- Focused cone/bounds tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasQuery3DServiceConeTests|FullyQualifiedName~ColliderRuntimeStateTests"`
  passed with 11 tests.
- Benchmark build:
  `dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0`
  passed.
- Mixed rotated cone short in-process benchmark smoke:
  - 64 dense targets: standard rotated cone 3.101 ms, long/narrow 2.913 ms,
    short/wide 2.708 ms.
  - 1024 dense targets: standard rotated cone 58.245 ms, long/narrow
    48.056 ms, short/wide 42.983 ms.
  - BenchmarkDotNet reported tiny in-process memory noise on some rows
    (`2 B` to `69 B`), but the focused xUnit allocation guard for the same
    mixed primitive finite-slab family reported zero steady-state allocations.
- 3D cone-volume query short in-process benchmark smoke:
  - baseline populated cone query: 1.176 ms for 64 targets.
  - long/narrow query: 1.096 ms.
  - short/wide query: 63.574 us due to the intentionally shorter finite-cone
    broad-phase bounds.
- Release validation completed with:
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- Wiki docs and public reducer labels describe the implemented behavior.
- Release and Lean tests pass.
- Benchmark smoke records CPU, candidate-count, and allocation results.

## Final Done Criteria

- Rotated finite-cone mixed slab sweeps are exact.
- Physical cone bounds and cone-volume query bounds use shared analytic cone
  geometry where it improves candidate quality.
- Cone-specific hot paths remain allocation-free after warmup.
- Tests cover long/narrow, short/fat, vertical, rotated, thick-slab, and
  zero-thickness slab cases.
- Documentation no longer carries vague rotated-cone caveats.
