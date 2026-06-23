# Query And Mixed Swept Shape Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Gravitas query and swept-shape behavior first-class across pure 2D, mixed 2D/3D, primitive/convex mesh/compound swept sources, and finite-slab mixed CCD paths.

**Architecture:** Keep caller-owned result buffers, deterministic candidate ordering, and conservative broad candidates. Add shape-specific exact query reducers only when they remove meaningful false positives without introducing false negatives or unacceptable hot-path cost.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp geometry primitives, SwiftCollections buffers, GridForge-backed partitions, Gravitas query and CCD services.

---

**Date:** 2026-06-21
**Status:** Done
**Owner:** Gravitas query and swept-shape hardening

## Purpose

The query services are now context-owned and deterministic, with 3D raycast,
swept-sphere, X/Z area queries, pure 2D circle/AABB/polygon
overlap/raycast/swept-circle APIs, and mixed 2D/3D swept APIs. The remaining
gaps are sharper: explicit convex swept-source APIs need a bounded runtime
policy for primitives, convex meshes, and authored compounds, and mixed
swept-circle queries now have exact primitive finite-slab reducers for sphere,
cuboid, world-Y capsule, and world-Y finite-cylinder targets while mesh,
compound, and unsupported rotated curved primitives remain explicit
conservative fallbacks.

For a first-class physics engine, those limitations need explicit reducer
policy, tests, benchmarks, and docs. A public query API that works by
conservative fallback is acceptable only when the fallback is named, measured,
and known not to create false negatives.

## Relationship To Existing Plans

- [`2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md`](../2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  owns exact CCD reducer promotion for continuous-collision internals.
- This plan owns public query API shape, pure 2D query parity, mixed finite-slab
  swept-circle exactness, and primitive/convex mesh/compound source query
  support. CCD should reuse any exact reducers produced here rather than
  maintaining a second policy.
- [`2026-06-21-discrete-response-and-contact-quality-hardening-plan.md`](2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
  owns contact response after query/narrow phase has produced hits.

## Current Baseline

- 3D query services expose raycast, swept-sphere, overlap-circle, and
  proximity-style X/Z queries with caller-owned hit buffers.
- Pure 2D query services expose circle, AABB, and convex polygon overlap
  queries, segment raycasts, swept-circle queries, and static-style
  swept-circle collectors used by 2D CCD.
- Mixed `SweepSphereAgainst2D` treats 2D shapes as finite slabs/prisms.
- Mixed `SweepCircleAgainst3D` uses exact finite-slab reducers for 3D sphere,
  cuboid, world-Y capsule, and world-Y finite-cylinder targets.
- Mixed swept-circle against mesh, compound, rotated capsule, and rotated
  finite-cylinder targets still uses an explicit conservative swept-sphere
  fallback labeled on `PhysicsMixedHit.ReducerKind`.
- Mesh targets are supported for raycast/sphere-sweep target queries through
  triangle candidates. Capsule, cuboid, finite-cylinder, convex mesh, and
  authored compound sources are supported by explicit 3D sweep APIs. Concave
  mesh sources and raw mesh sources are intentionally rejected; hosts should use
  offline convex decomposition into `LSCompoundCollider` parts when they need
  concave-looking movers.

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
- Benchmarks must cover primitive, convex mesh, and compound source expansion.

## Workstream 1: Query Surface Inventory And Fallback Policy

**Tasks**

- [x] Inventory every public query and internal CCD query path, including
  source shape, target shape, exact reducer, conservative fallback, ordering
  key, and allocation behavior.
- [x] Add tests that distinguish exact shape truth from accepted conservative
  fallback for mixed swept-circle and concave/raw mesh-source query families.
- [x] Update query docs with an explicit support matrix and fallback labels.
- [x] Rank missing query families by end-user value, false-positive severity,
  and benchmark cost before implementing new reducers.

**Progress 2026-06-22:** Workstream 1 established the explicit query surface
inventory in `docs/wiki/QUERY_SERVICES.md`, covering public query APIs and
internal CCD query paths with source shape, target shape, reducer policy,
ordering key, and allocation ownership. Mixed query hits now expose
`PhysicsMixedHit.ReducerKind`, with `Exact` used for current finite-slab
reducers and `ConservativeFallback` used for prism/proxy reducers that can
return early or extra hits without false negatives.

Focused tests now assert exact mixed sphere/circle behavior, conservative
fallback labeling for non-sphere/prism mixed paths, and the absence of public
unbounded concave/raw mesh-source sweep APIs. Missing query families were sorted
for follow-up: pure 2D AABB area queries, pure 2D convex polygon area queries,
mixed primitive finite-slab swept-circle reducers, and mixed mesh/compound
finite slab reducers. Workstream 2 closes the pure 2D AABB and convex polygon
area query entries.

## Workstream 2: Pure 2D Area Query Parity

**Problem**

Pure 2D had overlap-circle and segment raycasts, but lacked AABB and convex
polygon area-query APIs. Hosts that already author 2D boxes or polygons should
not need to approximate every area query with a circle.

**Tasks**

- [x] Add tests for `OverlapAabb2D` style queries against circle, AABB, convex
  polygon, and compound colliders.
- [x] Add tests for convex polygon area queries, including separated,
  edge-touching, full-overlap, and compound-part cases.
- [x] Reuse `QueryDetection2D` and existing collision SAT helpers where they
  preserve deterministic ordering and avoid allocations.
- [x] Expose single-hit and all-hit APIs with caller-owned `SwiftList` buffers,
  matching existing 2D query naming and layer/trigger semantics.
- [x] Update query docs and benchmarks for 2D area-query parity.

**Progress 2026-06-22:** Workstream 2 added exact pure 2D AABB and convex
polygon area queries through `GravitasQuery2DService.OverlapAabb`,
`OverlapAabbAll`, `OverlapPolygon`, and `OverlapPolygonAll`, and closed the
legacy closest-hit gap with `OverlapCircle`. The service
validates area inputs once, gathers GridForge-backed 2D bounds candidates, runs
allocation-free fixed-point SAT/closest-point checks in `QueryDetection2D`, and
sorts all-hit buffers by distance then collider ID. Compound targets reduce to
their owning `LSCompoundCollider2D` through stable best-part selection.

Focused coverage now includes AABB queries against circle, AABB, convex
polygon, and compound-style scenes, polygon queries for separated,
edge-touching, full-overlap, closest-hit, and compound owner behavior, and an
after-warmup allocation guard for both area-query families. Query docs and 2D
benchmark rows were updated for general and compound target scenes.

## Workstream 3: Mixed Finite-Slab Swept-Circle Solvers

**Problem**

`SweepCircleAgainst3D` is exact for 3D sphere targets, but capsule, cuboid,
finite cylinder, mesh, and compound targets still use a conservative
swept-sphere fallback. Tall or offset slabs can therefore report early
false-positive hits.

**Tasks**

- [x] Add red tests for cuboid, capsule, and finite-cylinder targets where the
  conservative fallback reports a hit that finite-slab geometry should reject
  or report later.
- [x] Implement exact finite-slab reducers for cuboid, capsule, and finite
  cylinder targets before considering mesh or compound expansion.
- [x] Preserve current sphere exact behavior and result ordering.
- [x] Route mixed 2D CCD through the same finite-slab reducers used by public
  `QueryMixed` APIs.
- [x] Add benchmark rows for dense mixed swept-circle scenes, including false
  positives, accepted hits, and candidate counts.

**Progress 2026-06-22:** Workstream 3 promoted mixed
`SweepCircleAgainst3D` primitive targets away from the accidental generic
swept-sphere proxy. Sphere behavior remains exact. Cuboid targets now clip the
target box against the source slab's Y interval, build a deterministic X/Z
convex projection, and sweep the source circle against that projection without
allocations. World-Y capsule and finite-cylinder targets use exact
vertical-interval reducers; unsupported rotated capsule/cylinder targets remain
explicit `ConservativeFallback` paths until a measured rotated curved-surface
solver is justified. Mesh and compound targets also remain labeled fallback, now
using a circumsphere source proxy so fallback paths cannot miss finite-slab
corner cases. Focused mixed query/CCD tests cover exact primitive reducer labels,
tall-slab report-later cases, proxy-only rejection, and fallback labeling.
`MixedQueryBenchmarks` now includes dense cuboid, capsule, and cylinder
swept-circle rows plus candidate-count rows.

## Workstream 4: Convex Swept Source Query Families

**Problem**

Mesh colliders can be queried as targets and can participate in simulation.
Capsule, cuboid, finite-cylinder, convex mesh, and authored compound sources
should also be queryable as first-class swept sources, but the API must not
pretend concave mesh sources are cheap or bounded. The high-risk unsupported
case is exact concave mesh-as-source sweeping or automatic runtime
decomposition, both of which can become expensive quickly if they naively scan
triangles or hide convex decomposition inside runtime queries.

**Tasks**

- [x] Define which convex source queries belong in runtime for alpha:
  primitive source, convex mesh source, authored compound source, exact concave
  mesh source expansion, or explicit no-runtime-support for concave/raw mesh
  sources.
- [x] Add tests for the chosen source boundary, preserving owner collider
  identity and stable primitive, triangle, or part ordering.
- [x] Add mesh-target and mesh/compound source benchmark rows before concave
  source expansion.
- [x] Prefer offline convex decomposition or authored compound colliders when
  exact concave mesh-source sweeps would have unbounded triangle cost.
- [x] Keep mesh target query behavior stable while source-family work is added.

**Progress 2026-06-22:** Workstream 4 added explicit
`Query3D.SweepCapsule`, `SweepCapsuleAll`, `SweepCuboid`, `SweepCuboidAll`,
`SweepCylinder`, `SweepCylinderAll`, `SweepConvexMesh`,
`SweepConvexMeshAll`, `SweepCompound`, and `SweepCompoundAll` APIs. Capsule,
cuboid, finite-cylinder, and convex mesh sources use support-mapped
conservative advancement against sphere, capsule, cuboid, finite-cylinder,
convex mesh, concave mesh target triangles, and compound targets. Authored
compound sources reduce supported convex parts in stable part order and report
the target owner once. Concave mesh sources are rejected with a clear error that
points hosts to offline convex decomposition into `LSCompoundCollider` parts;
raw mesh source queries and runtime decomposition remain intentionally
unsupported.
`SweptSphereQueryWorker.TrySweep(LSCollider collider, ...)` remains a
sphere-source reducer against a target collider, including mesh targets.
Focused tests cover primitive source hits, rotated capsule/cylinder source
geometry, convex mesh source hits, concave source rejection, compound source
reduction, mesh-target owner collapse, and the absence of generic/raw
mesh-source APIs. `QueryServiceBenchmarks` now includes mesh-target swept
sphere, capsule source, cuboid source, cylinder source, convex mesh source, and
compound source rows.

## Workstream 5: Query Diagnostics, Docs, And Release Validation

**Tasks**

- [x] Add optional diagnostic counters for query fallback hits, exact reducer
  attempts, accepted hits, and rejected conservative candidates where they help
  hosts debug query quality.
- [x] Update `docs/wiki/QUERY_SERVICES.md`, `docs/wiki/COLLISION_PIPELINE.md`,
  and `docs/wiki/DIMENSIONS.md` with the final support matrix.
- [x] Add or update benchmarks for every new public query family and exact mixed
  reducer.
- [x] Validate `Release` and `ReleaseLean` after runtime query changes.

**Progress 2026-06-22:** Workstream 5 added `QuerySummary` diagnostics through
`GravitasQuerySummaryDiagnosticView`, emitted candidate-level reducer counters
from mixed query paths, and kept disabled diagnostics on the existing early
return path. Final review hardening also centralized deterministic query
ordering: closest 3D raycasts now use the same distance/collider-ID tie-break as
all-hit raycasts and sweeps, 2D and mixed all-hit sorters now use allocation-free
heap sorting instead of duplicated insertion sorts, mixed single-hit queries now
keep the best candidate directly instead of filling and sorting all results, and
convex sweep reducer ties now use authored compound part or mesh triangle order
before collapsing hits back to owner identity.

Docs were updated across query, collision, dimensions, and diagnostics pages,
including the missing `Physics2DHit` hit-data entry and the final query
diagnostic surface. Remaining mixed swept-circle finite-slab reducer work for
mesh, compound, rotated capsule, and rotated finite-cylinder targets, plus
high-vertex convex mesh source benchmark signal, was extracted to
[`2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md`](2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md).

## Done Criteria

- Pure 2D exposes area-query parity for AABB and convex polygon use cases.
- Mixed swept-circle against promoted primitive 3D targets (sphere, cuboid,
  world-Y capsule, and world-Y finite cylinder) no longer relies on accidental
  generic sphere proxy behavior.
- Primitive, convex mesh, and compound source query behavior is explicit,
  tested, documented, and benchmarked; concave/raw mesh sources remain
  rejected.
- Public query docs distinguish exact support from conservative fallback.
- All new recurring query paths are allocation-free after warmup.

## Completion

Completed 2026-06-22 and moved to `docs/feature-work/done`.
