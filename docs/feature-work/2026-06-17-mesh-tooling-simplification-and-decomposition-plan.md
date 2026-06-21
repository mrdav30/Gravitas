# Mesh Tooling Simplification And Decomposition Plan

**Date:** 2026-06-17
**Expanded:** 2026-06-18
**Status:** Post-alpha / research-gated
**Owner:** Gravitas asset-tooling research

## Purpose

Gravitas runtime should consume deterministic collision geometry; it should not
perform expensive mesh simplification or convex decomposition during simulation.
This plan captures a future Gravitas-owned .NET tooling package for preparing
physics meshes, closed-volume inertia data, and authored convex collision
assets.

The package should live in the Gravitas solution as a separate project/package,
for example `Gravitas.MeshTools`, so games can bake assets offline while the
runtime package remains lean and deterministic. The R&D work should start by
adding the project scaffolding to the solution, not by keeping experiments as
loose scripts. If decomposition succeeds, the package is already in the right
shape to graduate. If it does not, the project still preserves fixtures,
benchmarks, diagnostics, and failed experiments for future research.

This plan is intentionally evidence-heavy. The goal is not to clone CGAL,
V-HACD, CoACD, or any other package. Those systems are useful reference
material, but Gravitas should adopt an algorithm only when tests and benchmarks
show it is the best deterministic path for LSF runtime collision assets.

## Runtime Boundary

Runtime remains the source of truth for simulation behavior:

- `LSMeshCollider` keeps the raw local-BVH triangle path for simple concave
  physics meshes where exact triangle collision is the right tradeoff.
- `LSCompoundCollider` is the current runtime target for authored convex
  collision assets: one collider ID, one body binding, one broad-phase identity,
  one event surface, and private deterministic part colliders.
- `ColliderShapeDefinition.ConvexMesh(...)` and `CompoundColliderPart` are the
  export target for offline convex pieces.
- Runtime automatic decomposition remains out of scope. No `Simulate` path
  should simplify, decompose, remesh, or otherwise mutate authoritative
  collision geometry.

Tooling may eventually export a baked asset DTO, but runtime import should still
materialize ordinary Gravitas public shapes. A dedicated mesh-owned piece path is
allowed only if benchmarks prove that public compound colliders cannot meet the
runtime cost or memory target.

## Scope

This is tooling R&D, not an alpha runtime dependency.

In scope:

- deterministic import normalization and explicit quantization.
- deterministic mesh validation and repair diagnostics.
- closed-volume checks that match runtime mesh inertia policy.
- optional collision-mesh simplification, not rendered-art simplification.
- convex hull generation and convexity tests over fixed-point or reproducibly
  quantized coordinates.
- authored approximate or exact convex decomposition output that feeds
  `LSCompoundCollider`.
- quality reports: piece count, hull vertex count, added volume, missing
  coverage, collision-probe preservation, validation failures, and benchmark
  stats.
- comparison baselines against raw `LSMeshCollider` triangle BVH and authored
  hand-built compound proxies.

Out of scope:

- runtime decomposition during `Simulate`.
- hidden decomposition inside `LSMeshCollider`.
- native C++ dependencies in the Gravitas tooling package.
- stochastic output as authoritative asset data.
- silent repair of broken meshes.
- treating rendered high-poly meshes as good physics meshes by default.

External tools may be used to generate comparison fixtures during research, if
their licenses allow checked-in fixture output. They should not become Gravitas
package dependencies.

## Design Principles

1. **Determinism first.** Importers may read floating-point formats, but the
   tooling boundary must quantize through explicit scale and rounding settings.
   Internal algorithms should operate on `Fixed64`, scaled integers, or
   deterministic rational-like predicates where needed.
2. **Evidence before algorithm commitment.** A known algorithm is a hypothesis,
   not a design decision. Promote it only after it beats simpler options on
   deterministic quality, runtime cost, and failure behavior.
3. **Exact when cheap, approximate when useful.** Raw triangle BVH remains exact
   and can be better for simple concave meshes. Decomposition is for dense or
   complex collision assets where convex pieces reduce runtime pair cost enough
   to justify preprocessing.
4. **No silent under-coverage.** Approximate convex pieces may include extra
   volume when the user accepts that tradeoff, but missing source volume or
   losing collision-critical openings must be reported.
5. **Stable asset output.** Piece order, hull vertex order, triangle winding,
   diagnostics, and report rows must be stable for identical input and settings.
6. **Authoring ergonomics matter.** Tool output should be easy to serialize,
   diff, review, and feed into runtime shape definitions without fake runtime
   colliders.
7. **Reuse lower-stack assets first.** Before adding mesh-tooling math,
   geometry, pooling, query, serialization, or grid-sampling infrastructure,
   review the matching FixedMathSharp, SwiftCollections, Chronicler, and
   GridForge APIs. Add new infrastructure only when the fit gap is understood,
   documented, and covered by tests or benchmarks.

## Research Context

CGAL documents two useful reference directions:

- Exact convex decomposition of bounded Nef polyhedra resolves reflex edges and
  can produce a worst-case optimal `O(r^2)` number of convex pieces, where `r`
  is the number of reflex edges.
- Approximate convex decomposition covers a closed input mesh with convex
  volumes that may include additional empty space. Its documented pipeline uses
  hierarchical splitting, voxel-assisted inside/outside classification, convex
  hulls, optional refitting, and bottom-up merging.

Those ideas are useful, but the CGAL Nef model is broader than Gravitas needs,
and its exact path can be the wrong runtime asset shape if it explodes piece
count.

V-HACD is useful as a historical reference for voxelized hierarchical
approximation, tuning knobs, and hull-count-driven workflows. It is C++, its
latest public release line is still native, and its own docs warn that modern
V-HACD does not try to find a minimum solution. It should not become a
dependency.

CoACD is useful because its collision-aware concavity and multi-step search
explicitly target physics collision quality rather than only visual hull fit.
However, the public tooling includes stochastic/default-seed behavior and native
implementation details. Gravitas can learn from the metrics and search framing,
but must own deterministic candidate generation and tie-breaking.

A user-provided algorithm sketch is also worth testing:

1. start with one source triangle.
2. grow a piece through adjacent source triangles.
3. build or update the convex hull of the candidate piece.
4. accept the triangle only if the hull does not extend outside the source
   closed mesh by more than the configured tolerance.
5. finish the piece when no adjacent triangle can be accepted, then repeat.

This source-triangle growth idea is attractive because it avoids creating new
source triangles and may produce fewer pieces than reflex-edge exact methods on
game-friendly meshes. It still needs proof around hull containment tests,
candidate ordering, narrow tunnels, slivers, self-intersections, disconnected
regions, and hull-update cost.

## Proposed Solution Shape

Phase 7 should add real solution projects early:

| Project | Purpose |
| --- | --- |
| `src/Gravitas.MeshTools/Gravitas.MeshTools.csproj` | Offline deterministic mesh validation, simplification, decomposition, metrics, and export APIs. |
| `tests/Gravitas.MeshTools.Tests/Gravitas.MeshTools.Tests.csproj` | Focused xUnit coverage for quantization, topology, validation diagnostics, hulls, simplification, decomposition, exports, and determinism. |
| `tests/Gravitas.MeshTools.Benchmarks/Gravitas.MeshTools.Benchmarks.csproj` | BenchmarkDotNet coverage for validation, hull generation, simplification, decomposition, metric scoring, and runtime-export comparison. |

The tooling project may reference `Gravitas` when it needs runtime value types,
`ColliderShapeDefinition`, `CompoundColliderPart`, `PhysicsMesh` validation
behavior, or export materialization tests. Runtime `Gravitas` must not reference
`Gravitas.MeshTools`.

The first version should be marked experimental and should not be packed as a
recommended public package until the promotion criteria below are met. Keeping
the project in the solution is still valuable because CI can compile the API,
tests can lock deterministic behavior, and benchmark artifacts can survive
between research passes.

## LSF Stack Reuse

Mesh tooling should be a good citizen of the existing stack, not a parallel
foundation. The first implementation pass should inventory these libraries
before writing custom equivalents:

| Library | Reuse first for |
| --- | --- |
| `FixedMathSharp` | `Fixed64`, vectors, quaternions, bounds, rays, planes, containment tests, and deterministic geometry primitives. |
| `SwiftCollections` | low-allocation lists, sets, queues, pools, deterministic candidate buffers, and FixedMathSharp query structures such as BVHs, octrees, or spatial hashes when their ordering can be made explicit. |
| `Chronicler` | explicit persisted asset/report data contracts when baked mesh-tooling output needs deterministic save/load behavior. |
| `GridForge` | deterministic voxel/topology concepts for fixed-grid quality probes, union-volume estimates, or sampling experiments when they fit better than a mesh-tooling-owned grid. |

If a lower-stack type is rejected, the plan or implementation notes should say
why. Good reasons include an API mismatch, a proven benchmark cost, missing
deterministic ordering control, or a tooling-only requirement that would pollute
runtime-focused libraries.

## Proposed Package Shape

Future implementation should start small and keep responsibilities separate:

| Area | Responsibility |
| --- | --- |
| `Gravitas.MeshTools` | Tooling package, separate from runtime simulation. |
| `MeshToolingMesh` | Immutable quantized vertex/triangle source, canonical IDs, bounds, and source metadata. |
| `MeshTopologyGraph` | Deterministic edges, triangle adjacency, shell/component grouping, and winding state. |
| `MeshValidationReport` | Structured diagnostics for asset problems; no exception-only user feedback. |
| `MeshQualityReport` | Deterministic metrics for coverage, error, hull count, hull vertices, and runtime proxy cost. |
| `ConvexHullBuilder` | Managed deterministic 3D hull generation for fixed/quantized input. |
| `MeshSimplifier` | Optional collision simplification experiments with closed-volume preservation. |
| `ConvexDecomposer` | Experimental strategy host for split/merge and source-triangle-growth approaches. |
| `CompoundCollisionAsset` | Stable serialized tool output that can materialize `CompoundColliderPart[]`. |

This table is a planning boundary, not a demand to create every type on day
one. The implementation should grow only as the earlier evidence phases need
each component.

## Core Data Contract

### Input

Tooling input should record:

- source vertex positions after explicit unit scale and quantization.
- source triangle indices in stable import order.
- source coordinate-system metadata.
- quantization settings: unit scale, fixed precision, rounding mode, and maximum
  accepted coordinate magnitude.
- authoring intent: validation only, closed-volume mesh, surface collision mesh,
  simplification candidate, or decomposition candidate.

If a format importer reads floats or doubles, that data must be converted at the
boundary. Internal reports should record enough metadata to reproduce the same
quantized mesh.

### Output

The first runtime export target should be:

```csharp
CompoundColliderPart.ConvexMesh(
    vertices,
    triangles,
    localOffset,
    localRotation,
    localScale,
    MeshInertiaPolicy.RequireClosedVolume)
```

Each convex piece should be a closed convex mesh unless the user explicitly
exports primitive parts or a future surface-only tooling mode. Piece transforms
should be stable and explicit. Source triangle provenance should be retained in
tooling metadata, but runtime does not need it.

The output report should include:

- input mesh hash and quantization settings.
- validation result.
- chosen strategy and settings.
- piece count.
- per-piece vertex and triangle counts.
- total convex hull volume, approximate union volume, and source volume.
- over-coverage and under-coverage metrics.
- collision-probe pass/fail counts.
- estimated runtime collider count and broad-phase bounds footprint.
- benchmark fixture IDs used for comparison.

## Quality Metrics

The tooling must measure quality before judging an algorithm:

### Geometric Metrics

- **Closed-volume validity:** every generated closed piece must pass the same
  topological invariants as runtime mesh inertia.
- **Convexity:** every generated piece must be convex under fixed/quantized
  predicates.
- **Over-coverage:** estimate or compute extra volume covered by convex pieces.
  Since convex pieces can overlap, report both summed hull volume and an
  approximate union-volume metric.
- **Under-coverage:** report any deterministic source samples or volume probes
  not contained by the output pieces.
- **Surface drift:** report maximum and percentile distance from source surface
  probes to generated hull surfaces.
- **Feature preservation:** track openings, channels, handles, slots, and
  concave pockets through explicit collision probes rather than relying on a
  single global volume score.

### Runtime Metrics

- generated compound vs generated compound collision time.
- raw dense concave mesh vs raw dense concave mesh collision time.
- raw dense concave mesh vs generated compound collision time.
- generated compound partition movement cost.
- query and mixed-query cost for generated pieces where relevant.
- allocation behavior after warmup.
- hull vertex count and broad-phase coverage pressure.

### Determinism Metrics

- identical output hashes across repeated runs.
- identical output hashes across Windows and Linux.
- identical diagnostic order.
- stable tie-breaking when candidate scores match.
- no dependence on hash iteration order or wall-clock time.

## Fixture Corpus

The fixture corpus should be built before algorithm work:

- simple convex primitives: cube, rectangular prism, tetrahedron, wedge.
- simple concave physics meshes: U-channel, inside corner, stair-step, open
  channel.
- dense versions of current benchmark fixtures.
- closed concave fixtures with known volume and known openings.
- disconnected closed components.
- thin walls and narrow slots.
- handles, rings, and tunnel-like meshes.
- sliver triangles and nearly coplanar faces.
- non-manifold edges, boundary holes, duplicate faces, inconsistent winding,
  self-intersection candidates, and zero-volume shells.
- high-aspect mechanical-ish parts where hull over-coverage is dangerous.

Each fixture should have an expected validation result and at least one reason
it exists. A fixture without a specific failure mode or quality signal is noise.

## Milestones

### Tooling Phase A: Package Boundary And Fixture Harness

- [ ] Add `src/Gravitas.MeshTools/Gravitas.MeshTools.csproj` to the solution.
- [ ] Add `tests/Gravitas.MeshTools.Tests/Gravitas.MeshTools.Tests.csproj` to
  the solution.
- [ ] Add `tests/Gravitas.MeshTools.Benchmarks/Gravitas.MeshTools.Benchmarks.csproj`
  to the solution.
- [ ] Inventory FixedMathSharp, SwiftCollections, Chronicler, and GridForge
  APIs that can be reused for the first validation and fixture tasks.
- [ ] Keep package metadata experimental so it cannot be mistaken for the
  recommended alpha asset workflow before the evidence gates are met.
- [ ] Add only the minimal public API root required by the first validation
  task; avoid marker types with no behavior.
- [ ] Add fixture helpers for deterministic source meshes and current runtime
  dense-mesh benchmark shapes.
- [ ] Add golden hash infrastructure for generated tool outputs.
- [ ] Add cross-platform deterministic-output tests to CI only after the first
  algorithm exists.

**Exit criteria:** tooling experiments compile as first-class solution projects
without adding dependencies from runtime `Gravitas` back to the tooling package.

### Tooling Phase B: Quantized Mesh And Validation Kernel

- [ ] Define immutable quantized mesh input with explicit scale and rounding
  settings.
- [ ] Build deterministic topology: canonical vertices, directed edges,
  undirected edge groups, triangle adjacency, connected shells, and winding
  classification.
- [ ] Validate edge manifoldness, boundary edges, duplicate faces, degenerate
  faces, inconsistent winding, disconnected shells, coordinate overflow, and
  unsupported triangle counts.
- [ ] Add self-intersection candidate diagnostics. Exact self-intersection can
  start conservative: report candidate triangle pairs from a BVH and fail the
  asset unless a follow-up exact test clears them.
- [ ] Return `MeshValidationReport` diagnostics in stable order.

**Exit criteria:** validation can explain why an asset is accepted, rejected, or
needs offline repair without throwing as the primary user-facing API.

### Tooling Phase C: Mass Properties And Runtime Export Contract

- [ ] Mirror or share runtime closed-volume mass-property math.
- [ ] Export baked source volume, center of mass, reference center, bounds, and
  inertia metadata.
- [ ] Verify runtime `PhysicsMesh.TryGetClosedVolumeMassProperties(...)` agrees
  with tooling output on closed fixtures.
- [ ] Define `CompoundCollisionAsset` as a stable DTO for authored convex
  pieces and transforms.
- [ ] Add import tests that materialize `LSCompoundCollider` from the DTO and
  preserve one runtime collider identity.

**Exit criteria:** tooling can validate and export known-good closed convex
pieces before decomposition exists.

### Tooling Phase D: Convex Hull And Convexity Primitives

- [ ] Implement a deterministic 3D convex hull builder for fixed/quantized
  points, or write a prototype plus tests that prove a simpler managed option
  can be made deterministic.
- [ ] Define hull vertex and face ordering rules.
- [ ] Validate outward winding and closed-volume mass properties on generated
  hulls.
- [ ] Add hull containment and point-in-convex-polyhedron tests.
- [ ] Benchmark hull construction and incremental hull updates on small,
  medium, and dense fixture subsets.

**Exit criteria:** hull generation is stable enough to be a dependency of
simplification/decomposition experiments. If it is not, decomposition pauses
here.

### Tooling Phase E: Quality And Collision Probe Suite

- [ ] Implement deterministic surface probes, volume probes, and contact probes.
- [ ] Add feature probes for channels, handles, slots, and inside corners.
- [ ] Implement approximate union-volume estimation for overlapping convex
  pieces using deterministic sampling or a fixed voxel grid.
- [ ] Add benchmark harnesses that compare raw triangle BVH, hand-authored
  compound proxies, and generated compounds on the same fixtures.
- [ ] Define report thresholds for:
  - maximum piece count.
  - maximum per-piece hull vertices.
  - maximum over-coverage.
  - zero or bounded under-coverage.
  - minimum runtime speedup on dense mesh-mesh fixtures, recorded before
    choosing a winning strategy.

**Exit criteria:** the project has a scoreboard before any decomposition
strategy can claim success.

### Tooling Phase F: Collision Mesh Simplification Experiments

- [ ] Prototype deterministic edge-collapse simplification with fixed-point or
  scaled-integer error metrics.
- [ ] Prototype feature-preserving simplification that pins boundary, sharp
  crease, and semantic marker vertices.
- [ ] Require closed-volume preservation when simplification is used for dynamic
  mesh inertia.
- [ ] Compare simplified raw mesh collision against unsimplified raw mesh and
  authored compound output.
- [ ] Reject simplification settings that create non-manifold output, invalid
  winding, or unbounded feature drift.

**Exit criteria:** simplification is either promoted as a separate optional tool
or explicitly deferred. It must not be a hidden pre-step for decomposition
unless it has its own quality proof.

### Tooling Phase G: Decomposition Strategy Experiments

Run at least two strategies against the same Phase E scoreboard.

#### Strategy 1: Deterministic Split/Merge Approximation

- Use deterministic split-plane candidates from bounding-box axes, reflex-edge
  neighborhoods, high-concavity probes, and stable feature axes.
- Split recursively while fixed concavity/coverage thresholds fail.
- Build hulls for each region.
- Optionally refit hulls if refitting reduces over-coverage without
  under-coverage.
- Merge bottom-up by stable priority queue: least quality loss, then lower
  piece count, then source triangle range, then lexicographic centroid.

This is closest to the CGAL/V-HACD family, but with deterministic candidate
sets and explicit Gravitas quality metrics.

#### Strategy 2: Source-Triangle Growth With Hull Containment

- Seed pieces by stable triangle order or highest local concavity score.
- Grow only through adjacency.
- For each candidate triangle, update the convex hull of the source vertices in
  the piece.
- Accept the triangle if the candidate hull does not exceed configured
  outside-volume, feature-probe, and under-coverage tolerances.
- Close the piece when no adjacent candidate passes.
- Repeat until all source triangles are assigned.
- Optionally merge adjacent accepted pieces when the merged hull still passes
  the same quality gates.

This strategy is worth trying because it may preserve source-triangle
provenance and reduce unnecessary piece count on simple game collision meshes.
It must be abandoned if hull containment checks become too slow or too
conservative on dense meshes.

#### Strategy 3: Exact Reflex-Edge Decomposition Reference

- Keep this as an optional reference/oracle for small fixtures, not the default
  asset path.
- Use it to understand lower bounds, reflex-edge behavior, and exact convex
  partitioning failure modes.
- Do not promote it if piece count or implementation complexity fights runtime
  performance goals.

**Exit criteria:** one strategy becomes the recommended alpha candidate only if
it produces deterministic output, passes quality probes, and meets the Phase E
runtime-speedup threshold against raw dense mesh runtime benchmarks. Otherwise,
Phase 7 exits with a validated "manual authored compound assets only for alpha"
recommendation.

### Tooling Phase H: Runtime Export And Authoring Experience

- [ ] Export stable `CompoundCollisionAsset` data with one ordered part list.
- [ ] Materialize runtime `CompoundColliderPart.ConvexMesh(...)` values without
  exposing internal runtime part colliders.
- [ ] Add report output that asset pipelines can display in editors or CI.
- [ ] Add "why this failed" diagnostics for common authoring mistakes.
- [ ] Add deterministic text or JSON output intended for source control review.
- [ ] Document that output is a physics asset, not rendered mesh data.

**Exit criteria:** a user can bake a mesh, review the result, materialize a
runtime `LSCompoundCollider`, and understand the quality/cost tradeoff.

### Tooling Phase I: Recommendation Gate

Before recommending generated decomposition for alpha:

- [ ] Run the fixture corpus through every promoted strategy.
- [ ] Run runtime benchmarks comparing generated output against raw triangle
  BVH and hand-authored compound proxies.
- [ ] Run deterministic-output tests repeatedly and cross-platform.
- [ ] Document recommended settings and failure modes.
- [ ] Explicitly state when users should keep raw concave `LSMeshCollider`
  instead of decomposition.

**Exit criteria:** docs can honestly say which asset path to use for simple
concave meshes, dense concave meshes, dynamic closed-volume meshes, and authored
compound collision assets.

## Promotion Criteria

An algorithm or tool phase is promotable only when all of these are true:

- output is byte-for-byte stable for identical quantized input and settings.
- all generated runtime pieces satisfy `PhysicsMesh` deterministic limits.
- generated convex mesh pieces pass closed-volume validation.
- under-coverage is zero for required-solid settings, or explicitly reported
  for approximate settings.
- feature probes catch unacceptable filled holes, tunnels, and channels.
- generated compound-vs-compound runtime collision meets the documented Phase E
  speedup threshold against raw dense concave mesh-vs-mesh on dense fixtures.
- generated output keeps zero steady-state runtime allocation after warmup.
- docs describe when not to use the tool.

If no strategy satisfies these criteria, the correct alpha answer is to ship
manual/authored compound collision assets and keep automatic decomposition as a
post-alpha research track.

## Open Design Questions

- Should tooling use `Fixed64` throughout, or use scaled integer predicates for
  hull orientation and containment while exporting `Fixed64` vertices?
- Should non-manifold input fail by default, or should a separate explicit
  repair/remesh package be planned later?
- How much over-coverage is acceptable for game collision assets before users
  should be told to author pieces manually?
- Should generated convex pieces preserve source triangle provenance in the
  serialized asset, or only in a sidecar report?
- Should simplification be allowed before decomposition, or should it remain an
  explicit independent tool until proven safe?

## Current Recommendation

For alpha, do not promise automatic convex decomposition yet. The runtime path
is already coherent:

- simple concave physics meshes can use raw `LSMeshCollider` triangle BVH.
- complex dense collision assets should be authored offline as compound convex
  pieces.
- dynamic mesh inertia should continue to require closed-volume truth by
  default.

Phase 7 should build the evidence platform and prototype strategies, then let
the numbers choose the first recommended automatic tooling path. My current bet
is that a deterministic split/merge approximation will be easier to make robust,
while source-triangle growth is the more interesting experiment for reducing
piece count on game-friendly closed meshes. Neither should be accepted without
the Phase E scoreboard.

## References

- CGAL Convex Decomposition of Polyhedra:
  <https://doc.cgal.org/latest/Convex_decomposition_3/index.html>
- CoACD project:
  <https://colin97.github.io/CoACD/>
- CoACD repository:
  <https://github.com/SarahWeiii/CoACD>
- V-HACD repository:
  <https://github.com/kmammou/v-hacd>
