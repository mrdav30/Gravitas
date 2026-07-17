# Mixed Query Finite-Slab Reducer Completion Plan

> **For agentic workers:** Treat this as a living context guide. Update progress
> as workstreams complete, and move genuinely deferred discoveries into their
> own plan or the evergreen trackers instead of leaving vague wiki caveats
> behind.

**Goal:** Promote the remaining mixed swept-circle conservative target families
to exact finite-slab reducers where deterministic shape math can provide
meaningful query truth, and validate convex mesh swept-source scaling before
alpha.

**Architecture:** Keep `QueryMixed` explicit, allocation-free after warmup, and
deterministically ordered. Public mixed hits must continue to label reducer
quality through `PhysicsMixedHit.ReducerKind`, and CCD should reuse public query
reducers instead of carrying a second policy.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry,
SwiftCollections scratch buffers, GridForge-backed partitions, Gravitas query
and CCD services.

---

**Date:** 2026-06-22 **Status:** Done **Owner:** Gravitas query and mixed CCD
hardening

## Purpose

At the start of this plan, `QueryMixed.SweepCircleAgainst3D` was exact for 3D
spheres, cuboids, world-Y capsules, and world-Y finite cylinders. Mesh,
compound, rotated capsule, and rotated finite-cylinder targets still used a
labeled `ConservativeFallback` path. That fallback was safe from false
negatives, but it could report earlier or extra hits in dense mixed scenes.

For alpha, that policy should either become exact for the current runtime shape
families or remain explicitly justified by benchmark evidence and documented API
semantics. This plan owns that closure so the completed query hardening plan
does not carry hidden follow-up work.

## Relationship To Existing Plans

- [`2026-06-21-query-and-mixed-swept-shape-hardening-plan.md`](2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  established public query parity, convex source APIs, fallback labels, query
  diagnostics, and release validation.
- [`2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md`](2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  owns broader CCD exact time-of-impact promotion. CCD should consume reducers
  produced here instead of duplicating mixed query math.
- [`benchmark-signal-hardening-backlog.md`](../benchmark-signal-hardening-backlog.md)
  remains the place for measured benchmark regressions. This plan owns the
  first-class reducer work and the benchmark rows needed to judge it.

## Workstream 1: Rotated Curved Target Finite-Slab Reducers

**Status:** Done.

**Tasks**

- [x] Add red tests for rotated capsule and rotated finite-cylinder targets
      where the circumsphere fallback reports an early or extra hit.
- [x] Research and implement deterministic fixed-point reducers for a 2D circle
      slab swept against arbitrarily oriented capsules and finite cylinders.
- [x] Preserve world-Y capsule/cylinder fast paths and exact labels.
- [x] Keep `ConservativeFallback` only for cases that remain deliberately
      unsupported after measured evidence.
- [x] Add benchmark rows for dense rotated capsule/cylinder mixed sweeps.

**Implementation notes**

- Rotated capsule and finite-cylinder targets now use deterministic finite-slab
  projection support reducers instead of the mixed circumsphere fallback. The
  reducer sweeps the 2D circle in X/Z against the target volume clipped to the
  slab Y interval.
- World-Y capsule/cylinder paths still use the cheaper vertical-interval
  reducers.
- Mesh and compound target fallback policy was closed by Workstream 2.

## Workstream 2: Mesh And Compound Target Finite-Slab Reducers

**Status:** Done.

**Tasks**

- [x] Add tests that distinguish finite-slab truth from current fallback hits
      for mesh triangle targets and authored compound targets.
- [x] Implement exact finite-slab reducers against mesh triangle candidates,
      reducing hits back to the owning mesh collider with stable triangle
      ordering.
- [x] Implement compound target reduction over supported parts in authored part
      order, preserving one owner hit and deterministic tie-breaks.
- [x] Share reducer policy with mixed static CCD collectors.
- [x] Benchmark mesh and compound mixed sweeps at sparse, dense, and
      false-positive-heavy scales.

**Implementation notes**

- Mesh target reducers now query triangle BVH candidates, clip each world-space
  triangle to the query slab Y interval, project the clipped polygon into X/Z,
  and sweep the source circle against the resulting point, segment, or convex
  polygon.
- Mesh hits reduce to the owning `LSMeshCollider`; equal-distance triangle hits
  keep the lower authored triangle index without sorting the candidate buffer.
- Compound target reducers iterate private parts in authored order, preserve one
  owner hit on the `LSCompoundCollider`, and keep the earlier part on
  equal-distance ties.
- `SweepCircleAgainstStatic3DAll` uses the same reducer path as public
  `SweepCircleAgainst3DAll`, so mixed static CCD no longer receives mesh or
  compound circumsphere-proxy hits for this source family.
- Benchmark rows now cover sparse, dense, and false-positive-heavy mesh and
  compound target sweeps.

## Workstream 3: Convex Mesh Source Scaling Signal

**Status:** Done.

**Tasks**

- [x] Add a high-vertex convex mesh source benchmark beyond the current cube
      source row.
- [x] Measure whether per-support full-vertex scans are acceptable for alpha
      query workloads.
- [x] If measured cost is high, prototype deterministic support acceleration or
      cached directional support data without introducing floating-point or
      platform-order dependence.
- [x] Keep concave mesh sources unsupported; hosts should use authored convex
      decomposition into `LSCompoundCollider` parts.

**Implementation notes**

- `QueryServiceBenchmarks` now includes
  `SweepConvexMeshAllAcrossSphereTargets_HighVertexSource`, using a
  subdivision-16 closed convex cube source against the existing 64-sphere target
  line.
- Short-run BenchmarkDotNet signal showed the original full transform-per-
  support scan at `64.237 ms/op` for the high-vertex row versus `1.366 ms/op`
  for the cube source row.
- `PhysicsMesh` now builds an exact deterministic support tree for high-vertex
  convex meshes. Support lookup prunes local vertex bounds by fixed-point
  projection, preserves lower source-vertex index for ties, and transforms only
  the winning support vertex to world space.
- After support-tree acceleration, the final short-run signal measured
  `2.858 ms/op` for the high-vertex source row and `1.106 ms/op` for the cube
  source row, both with zero managed allocation. Treat these short-run numbers
  as local scaling evidence, not canonical release baselines.
- Concave mesh sources remain intentionally rejected. Hosts should author
  concave-looking movers as stable `LSCompoundCollider` convex decomposition
  parts.

## Workstream 4: Diagnostics, Docs, And Validation

**Status:** Done.

**Tasks**

- [x] Ensure `GravitasQuerySummaryDiagnosticView` remains accurate for exact
      attempt, accepted hit, fallback hit, and rejected fallback candidate
      counts.
- [x] Update `docs/wiki/QUERY_SERVICES.md`, `COLLISION_PIPELINE.md`, and
      `DIMENSIONS.md` with final exact/fallback policy.
- [x] Validate Release and ReleaseLean builds/tests.
- [x] Move this plan to `docs/feature-work/done` when all mixed query reducer
      work owned by this plan is closed or intentionally extracted.

**Implementation notes**

- Mixed query diagnostic emission now uses a shared helper so closest and
  all-hit mixed query paths emit `MixedQuery` and `QuerySummary` with identical
  counter semantics.
- Added diagnostics regression coverage for accepted and rejected conservative
  `SweepSphereAgainst2D` prism-bound candidates.
- Removed the per-mesh triangle-candidate sort from the mixed mesh target
  reducer. Equal-distance hits now preserve lower authored triangle index by
  tracking the best triangle index directly, avoiding an unnecessary
  `O(k log k)` step in the hot path.
- Wiki docs now state the final policy for this plan: `SweepCircleAgainst3D`
  target reducers are exact for supported 3D target families; the remaining
  public mixed query fallback is the opposite source direction.
- Exact AABB, polygon, and compound slab reducers for
  `QueryMixed.SweepSphereAgainst2D` were extracted into
  [`Mixed Sphere Against 2D Slab Reducer Completion`](2026-06-23-mixed-sphere-2d-slab-reducer-completion-plan.md).
- Mesh triangle-level finite-slab benchmark attribution was captured in
  [`benchmark-signal-hardening-backlog.md`](../benchmark-signal-hardening-backlog.md)
  instead of broadening this close-out pass.

## Done Criteria

- Mixed swept-circle queries no longer rely on conservative fallback for a
  supported target family without explicit benchmark-backed justification.
- Mesh and compound target reductions preserve owner identity, triangle/part
  ordering, and caller-owned result buffers.
- Rotated capsule/cylinder behavior is either exact or has a measured,
  documented no-change rationale.
- Convex mesh source query scaling has benchmark signal beyond trivial cube
  sources.
- Query diagnostics, docs, tests, and benchmarks all agree on exact versus
  conservative reducer policy.
