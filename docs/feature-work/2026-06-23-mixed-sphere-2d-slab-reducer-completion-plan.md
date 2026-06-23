# Mixed Sphere Against 2D Slab Reducer Completion Plan

> **For agentic workers:** Treat this as a living context guide. Update
> progress as workstreams complete, and move genuinely deferred discoveries into
> their own plan or the evergreen trackers instead of leaving vague wiki caveats
> behind.

**Goal:** Promote `QueryMixed.SweepSphereAgainst2D` AABB, convex polygon, and
compound slab hits from conservative prism bounds to exact finite-slab reducers
where deterministic fixed-point geometry can reject false positives without
introducing false negatives.

**Architecture:** Keep mixed broad-phase candidate gathering conservative, then
run shape-specific 3D-sphere-source reducers against embedded 2D slab geometry.
Hits must preserve 2D owner identity, stable part ordering, reducer labels, and
caller-owned all-hit buffers. Public mixed query reducers and mixed static CCD
collectors should share policy instead of carrying parallel fallback rules.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry,
SwiftCollections scratch buffers, GridForge-backed mixed partitions, Gravitas
mixed query and CCD services.

---

**Date:** 2026-06-23
**Status:** Pre-alpha release blocker
**Owner:** Gravitas query and mixed CCD hardening

## Purpose

`QueryMixed.SweepCircleAgainst3D` now has exact finite-slab reducers for the
supported 3D target families, including rotated curved primitives, mesh
targets, and compound targets. The remaining public mixed query fallback is the
opposite source direction: `QueryMixed.SweepSphereAgainst2D` is exact for 2D
circle slabs, but AABB and convex polygon slabs still accept conservative mixed
prism-bound hits.

That policy is safe from false negatives, but it can report earlier or extra
hits when a 3D sphere passes near the prism volume without actually intersecting
the embedded 2D shape's finite slab. For alpha, that remaining public fallback
needs its own tests, reducer design, benchmark signal, and docs instead of being
buried in the completed mixed swept-circle reducer plan.

## Relationship To Existing Plans

- [`2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md`](done/2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md)
  completed exact `SweepCircleAgainst3D` target reducers and convex mesh source
  scaling signal.
- [`2026-06-21-query-and-mixed-swept-shape-hardening-plan.md`](done/2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  established public mixed query labels, query diagnostics, and convex source
  API boundaries.
- [`2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md`](2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  owns broader continuous-collision exact TOI promotion. This plan should feed
  mixed static 3D-sphere-against-2D reducer policy into CCD, but it does not own
  dynamic mixed CCD island or relative-motion design.

## Guiding Rules

- Conservative broad candidates remain the no-false-negative safety net.
- Exact reducers must be deterministic under equal-distance, equal-feature, and
  compound-part tie cases.
- AABB and polygon slabs should report exact hits only when the moving 3D sphere
  intersects the finite vertical slab volume of the 2D shape.
- Compound 2D targets should preserve the owning `LSCompoundCollider2D` hit and
  stable authored part order.
- Unsupported or degenerate geometry must be explicit, tested, and labeled
  instead of silently falling through to ambiguous behavior.
- Runtime query paths must remain allocation-free after warmup.

## Workstream 1: Inventory, Red Tests, And Benchmark Attribution

**Tasks**

- [ ] Inventory every `SweepSphereAgainst2D` path that can still accept
  `ConservativeFallback`.
- [ ] Add red tests where AABB and convex polygon prism bounds report a hit that
  the exact finite 2D shape slab should reject.
- [ ] Add true-hit tests for direct face, edge, corner, starting-overlap, and
  vertical-grazing cases.
- [ ] Add compound tests for multiple parts at different distances and
  equal-distance authored part ordering.
- [ ] Add benchmark rows or diagnostics that attribute candidate count, exact
  attempts, accepted hits, fallback hits, and rejected conservative candidates.

## Workstream 2: AABB Slab Reducer

**Tasks**

- [ ] Implement a deterministic swept-sphere reducer against finite 2D AABB
  slabs.
- [ ] Preserve cheap vertical-interval and planar-distance rejection before
  expensive exact work.
- [ ] Keep result points and normals consistent with mixed contact/query
  orientation rules.
- [ ] Route mixed static CCD sphere-source collectors through the same reducer
  policy where applicable.
- [ ] Prove no managed allocation after warmup for accepted, rejected, and
  all-hit AABB slab query paths.

## Workstream 3: Convex Polygon And Compound Slab Reducers

**Tasks**

- [ ] Implement exact swept-sphere reduction against finite convex polygon
  slabs, including segment and vertex feature cases.
- [ ] Reuse FixedMathSharp geometry helpers where they fit; document any custom
  fixed-point math invariants.
- [ ] Reduce compound 2D targets over supported parts in authored order and
  report one owner hit.
- [ ] Add false-positive-heavy polygon and compound benchmark rows before
  optimizing reducer internals.
- [ ] Preserve `PhysicsMixedHit.ReducerKind` and `QuerySummary` counter
  semantics.

## Workstream 4: Docs, Diagnostics, And Release Validation

**Tasks**

- [ ] Update `docs/wiki/QUERY_SERVICES.md`, `COLLISION_PIPELINE.md`, and
  `DIMENSIONS.md` with the final exact/fallback policy.
- [ ] Ensure diagnostics distinguish exact attempts, accepted hits, fallback
  hits, and rejected conservative candidates for this source direction.
- [ ] Update benchmark backlog entries or close them with evidence.
- [ ] Validate Release and ReleaseLean builds/tests.
- [ ] Move this plan to `docs/feature-work/done` once the remaining
  `SweepSphereAgainst2D` public fallback is closed or explicitly rejected with
  evidence.

## Done Criteria

- `SweepSphereAgainst2D` no longer accepts conservative AABB, convex polygon, or
  compound slab hits without explicit benchmark-backed justification.
- Exact reducers preserve owner identity, stable ordering, normals, hit points,
  and caller-owned result buffers.
- Mixed static CCD and public mixed query policy agree for 3D sphere sources
  against 2D slab targets.
- Tests cover true hits, false-positive rejection, equal-distance ties,
  degenerate inputs, diagnostics, and allocation guardrails.
- Benchmarks expose dense and false-positive-heavy AABB, polygon, and compound
  slab workloads.
