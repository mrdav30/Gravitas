# Query And Mixed Swept Shape Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Gravitas query and swept-shape behavior first-class across pure 2D, mixed 2D/3D, mesh-as-source sweeps, and finite-slab mixed CCD paths.

**Architecture:** Keep caller-owned result buffers, deterministic candidate ordering, and conservative broad candidates. Add shape-specific exact query reducers only when they remove meaningful false positives without introducing false negatives or unacceptable hot-path cost.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry primitives, SwiftCollections buffers, GridForge-backed partitions, Gravitas query and CCD services.

---

**Date:** 2026-06-21
**Status:** Backlog / alpha candidate
**Owner:** Gravitas query and swept-shape hardening

## Purpose

The query services are now context-owned and deterministic, with 3D raycast,
swept-sphere, X/Z area queries, pure 2D overlap/raycast/swept-circle, and mixed
2D/3D swept APIs. The remaining gaps are sharper: pure 2D does not yet expose
AABB or polygon area-query APIs, mesh-as-source swept query families are still
future hardening, and mixed swept-circle queries are exact for 3D sphere targets
but conservative for cuboid, capsule, finite cylinder, mesh, and compound
targets.

For a first-class physics engine, those limitations need explicit reducer
policy, tests, benchmarks, and docs. A public query API that works by
conservative fallback is acceptable only when the fallback is named, measured,
and known not to create false negatives.

## Relationship To Existing Plans

- [`2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md`](2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  owns exact CCD reducer promotion for continuous-collision internals.
- This plan owns public query API shape, pure 2D query parity, mixed finite-slab
  swept-circle exactness, and mesh-as-source query families. CCD should reuse
  any exact reducers produced here rather than maintaining a second policy.
- [`2026-06-21-discrete-response-and-contact-quality-hardening-plan.md`](2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
  owns contact response after query/narrow phase has produced hits.

## Current Baseline

- 3D query services expose raycast, swept-sphere, overlap-circle, and
  proximity-style X/Z queries with caller-owned hit buffers.
- Pure 2D query services expose overlap-circle, segment raycast, swept-circle,
  and static-style swept-circle collectors used by 2D CCD.
- Pure 2D AABB and polygon area-query APIs remain future work.
- Mixed `SweepSphereAgainst2D` treats 2D shapes as finite slabs/prisms.
- Mixed `SweepCircleAgainst3D` uses an exact finite-slab projection for 3D
  sphere targets.
- Mixed swept-circle against capsule, cuboid, finite cylinder, mesh, and
  compound targets still uses the conservative swept-sphere worker fallback.
- Mesh targets are supported for raycast/sphere-sweep target queries through
  triangle candidates, but mesh-as-source swept query families remain future
  hardening.

## Guiding Rules

- Query results must remain deterministic: distance first, then stable collider
  identity and stable private part/triangle ordering.
- All-hit APIs must write into caller-owned buffers and remain allocation-free
  after warmup.
- Broad candidates may be conservative, but accepted query hits should be
  shape-exact whenever the API claims shape truth.
- Unsupported exact reducers must fall back safely with no false negatives and
  explicit documentation.
- Mixed finite-slab semantics must preserve the 2D slab's Y center and
  half-thickness instead of inflating into a generic sphere proxy.
- Benchmarks must precede mesh/compound exact expansion.

## Workstream 1: Query Surface Inventory And Fallback Policy

**Tasks**

- [ ] Inventory every public query and internal CCD query path, including
  source shape, target shape, exact reducer, conservative fallback, ordering
  key, and allocation behavior.
- [ ] Add tests that distinguish exact shape truth from accepted conservative
  fallback for mixed swept-circle and mesh-as-source families.
- [ ] Update query docs with an explicit support matrix and fallback labels.
- [ ] Rank missing query families by end-user value, false-positive severity,
  and benchmark cost before implementing new reducers.

## Workstream 2: Pure 2D Area Query Parity

**Problem**

Pure 2D has overlap-circle and segment raycasts, but lacks AABB and convex
polygon area-query APIs. Hosts that already author 2D boxes or polygons should
not need to approximate every area query with a circle.

**Tasks**

- [ ] Add tests for `OverlapAabb2D` style queries against circle, AABB, convex
  polygon, and compound colliders.
- [ ] Add tests for convex polygon area queries, including separated,
  edge-touching, full-overlap, and compound-part cases.
- [ ] Reuse `QueryDetection2D` and existing collision SAT helpers where they
  preserve deterministic ordering and avoid allocations.
- [ ] Expose single-hit and all-hit APIs with caller-owned `SwiftList` buffers,
  matching existing 2D query naming and layer/trigger semantics.
- [ ] Update query docs and benchmarks for 2D area-query parity.

## Workstream 3: Mixed Finite-Slab Swept-Circle Solvers

**Problem**

`SweepCircleAgainst3D` is exact for 3D sphere targets, but capsule, cuboid,
finite cylinder, mesh, and compound targets still use a conservative
swept-sphere fallback. Tall or offset slabs can therefore report early
false-positive hits.

**Tasks**

- [ ] Add red tests for cuboid, capsule, and finite-cylinder targets where the
  conservative fallback reports a hit that finite-slab geometry should reject
  or report later.
- [ ] Implement exact finite-slab reducers for cuboid, capsule, and finite
  cylinder targets before considering mesh or compound expansion.
- [ ] Preserve current sphere exact behavior and result ordering.
- [ ] Route mixed 2D CCD through the same finite-slab reducers used by public
  `QueryMixed` APIs.
- [ ] Add benchmark rows for dense mixed swept-circle scenes, including false
  positives, accepted hits, and candidate counts.

## Workstream 4: Mesh-As-Source Swept Query Families

**Problem**

Mesh colliders can be queried as targets, but source-side mesh sweeps remain a
future hardening area. A mesh-as-source sweep can become expensive quickly if it
naively scans triangles or hides convex decomposition inside runtime queries.

**Tasks**

- [ ] Define which mesh source queries belong in runtime for alpha: convex mesh
  source, concave mesh source, authored convex decomposition, or explicit
  no-runtime-support policy.
- [ ] Add tests for the chosen mesh source policy, preserving owner collider
  identity and stable triangle or part ordering.
- [ ] Add mesh-as-source benchmark rows before implementing concave or compound
  expansion.
- [ ] Prefer offline convex decomposition or authored compound colliders when
  runtime exact mesh source sweeps would have unbounded triangle cost.
- [ ] Keep mesh target query behavior stable while source-family work is added.

## Workstream 5: Query Diagnostics, Docs, And Release Validation

**Tasks**

- [ ] Add optional diagnostic counters for query fallback hits, exact reducer
  attempts, accepted hits, and rejected conservative candidates where they help
  hosts debug query quality.
- [ ] Update `docs/wiki/QUERY_SERVICES.md`, `docs/wiki/COLLISION_PIPELINE.md`,
  and `docs/wiki/DIMENSIONS.md` with the final support matrix.
- [ ] Add or update benchmarks for every new public query family and exact mixed
  reducer.
- [ ] Validate `Release` and `ReleaseLean` after runtime query changes.

## Done Criteria

- Pure 2D exposes area-query parity for AABB and convex polygon use cases.
- Mixed swept-circle against primitive 3D targets no longer relies on accidental
  generic sphere proxy behavior.
- Mesh-as-source query policy is explicit, tested, documented, and benchmarked
  before any expensive runtime expansion.
- Public query docs distinguish exact support from conservative fallback.
- All new recurring query paths are allocation-free after warmup.
