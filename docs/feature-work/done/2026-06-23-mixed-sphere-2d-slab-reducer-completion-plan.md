# Mixed Sphere Against 2D Slab Reducer Completion Plan

> **For agentic workers:** Treat this as a living context guide. Update progress
> as workstreams complete, and move genuinely deferred discoveries into their
> own plan or the evergreen trackers instead of leaving vague wiki caveats
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

**Date:** 2026-06-23 **Status:** Done 2026-06-23 **Owner:** Gravitas query and
mixed CCD hardening

## Purpose

`QueryMixed.SweepCircleAgainst3D` now has exact finite-slab reducers for the
supported 3D target families, including rotated curved primitives, mesh targets,
and compound targets. At plan start, the opposite source direction still had one
public mixed query fallback: `QueryMixed.SweepSphereAgainst2D` was exact for 2D
circle slabs, but AABB and convex polygon slabs still accepted conservative
mixed prism-bound hits.

That old policy was safe from false negatives, but it could report earlier or
extra hits when a 3D sphere passed near the prism volume without actually
intersecting the embedded 2D shape's finite slab. For alpha, that public
fallback needed its own tests, reducer design, benchmark signal, and docs
instead of being buried in the completed mixed swept-circle reducer plan.

## Relationship To Existing Plans

- [`2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md`](2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md)
  completed exact `SweepCircleAgainst3D` target reducers and convex mesh source
  scaling signal.
- [`2026-06-21-query-and-mixed-swept-shape-hardening-plan.md`](2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
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

- [x] Inventory every `SweepSphereAgainst2D` path that can still accept
      `ConservativeFallback`.
- [x] Add red tests where AABB and convex polygon prism bounds report a hit that
      the exact finite 2D shape slab should reject.
- [x] Add true-hit tests for direct face, edge, corner, starting-overlap, and
      vertical-grazing cases.
- [x] Add compound tests for multiple parts at different distances and
      equal-distance authored part ordering.
- [x] Add benchmark rows or diagnostics that attribute candidate count, exact
      attempts, accepted hits, fallback hits, and rejected conservative
      candidates.

**Result:** Public closest/all-hit queries and mixed static 2D CCD share the
same `TrySweepSphereAgainst2DCandidate` policy. Red tests now cover AABB and
polygon proxy misses, accepted side/edge/cap cases, starting overlap, compound
owner identity, and equal-distance authored part order. Query diagnostics assert
exact attempt, accepted hit, fallback hit, and rejected conservative counters.
Benchmark rows expose dense and false-positive-heavy AABB, polygon, and compound
target workloads plus candidate counts.

## Workstream 2: AABB Slab Reducer

**Tasks**

- [x] Implement a deterministic swept-sphere reducer against finite 2D AABB
      slabs.
- [x] Preserve cheap vertical-interval and planar-distance rejection before
      expensive exact work.
- [x] Keep result points and normals consistent with mixed contact/query
      orientation rules.
- [x] Route mixed static CCD sphere-source collectors through the same reducer
      policy where applicable.
- [x] Prove no managed allocation after warmup for accepted, rejected, and
      all-hit AABB slab query paths.

**Result:** AABB slabs now use exact finite-prism shape casts: starting overlap,
cap faces, side faces, and prism edges are tested without accepting expanded
bounds-only hits. Result construction uses shared embedded-slab closest-point
geometry so starting-inside normals resolve to a real slab boundary.

## Workstream 3: Convex Polygon And Compound Slab Reducers

**Tasks**

- [x] Implement exact swept-sphere reduction against finite convex polygon
      slabs, including segment and vertex feature cases.
- [x] Reuse FixedMathSharp geometry helpers where they fit; document any custom
      fixed-point math invariants.
- [x] Reduce compound 2D targets over supported parts in authored order and
      report one owner hit.
- [x] Add false-positive-heavy polygon and compound benchmark rows before
      optimizing reducer internals.
- [x] Preserve `PhysicsMixedHit.ReducerKind` and `QuerySummary` counter
      semantics.

**Result:** Convex polygon slabs share the exact finite-prism reducer with AABB
slabs by walking stable collider vertices. Compound 2D sweeps reduce private
parts in authored order, keep owner identity, and use strict distance
replacement so equal-distance ties keep the earlier part. Supported AABB,
polygon, circle, and compound slab hits are now labeled `Exact`.

## Workstream 4: Docs, Diagnostics, And Release Validation

**Tasks**

- [x] Update `docs/wiki/QUERY_SERVICES.md`, `COLLISION_PIPELINE.md`, and
      `DIMENSIONS.md` with the final exact/fallback policy.
- [x] Ensure diagnostics distinguish exact attempts, accepted hits, fallback
      hits, and rejected conservative candidates for this source direction.
- [x] Update benchmark backlog entries or close them with evidence.
- [x] Validate Release and ReleaseLean builds/tests.
- [x] Move this plan to `docs/feature-work/done` once the remaining
      `SweepSphereAgainst2D` public fallback is closed or explicitly rejected
      with evidence.

**Result:** Wiki docs now describe exact public/static-CCD mixed reducers for 3D
sphere sources against supported 2D slab targets. No new deferred work was
found; proxy-based dynamic mixed CCD remains owned by the broader CCD plans.
Validation evidence is recorded in the final implementation summary.

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
