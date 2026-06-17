# Mesh Tooling Simplification And Decomposition Plan

**Date:** 2026-06-17
**Status:** Draft
**Owner:** Gravitas asset-tooling research

## Purpose

Gravitas runtime should consume deterministic collision geometry; it should not
perform expensive mesh simplification or convex decomposition during simulation.
This plan captures a future Gravitas-owned .NET tooling package for preparing
physics meshes, closed-volume inertia data, and authored convex collision assets.

The package should live in the Gravitas solution as a separate project/package,
for example a future `Gravitas.Tools` or `Gravitas.MeshTools`, so games can bake
assets offline while the runtime package remains lean and deterministic.

## Scope

This is tooling R&D, not an alpha runtime dependency.

In scope:

- deterministic mesh validation and repair diagnostics.
- closed-volume checks that match runtime mesh inertia policy.
- mesh simplification for collision meshes, not rendered art.
- convex hull generation and convexity tests over fixed-point or reproducibly
  quantized coordinates.
- authored approximate or exact convex decomposition output that can feed
  `LSCompoundCollider` or a future mesh-owned convex-piece data path.
- quality reports: piece count, added volume, max concavity/error, validation
  failures, and benchmark-friendly stats.

Out of scope:

- runtime decomposition during `Simulate`.
- hidden decomposition inside `LSMeshCollider`.
- dependencies on native C++ decomposition libraries.
- non-deterministic floating-point outputs as authoritative runtime data.

## Research Notes

CGAL documents two useful reference directions:

- Exact convex decomposition of bounded Nef polyhedra resolves reflex edges and
  can produce `O(r^2)` convex pieces, where `r` is the number of reflex edges.
- Approximate convex decomposition covers a closed input mesh with convex
  volumes that may include additional empty space. Its pipeline uses hierarchical
  splitting, voxel-assisted inside/outside classification, convex hulls,
  refitting, and bottom-up merging.

Those are useful design references, not dependency choices. CGAL is C++ and its
exact Nef model is much broader than what Gravitas needs for deterministic game
collision assets.

V-HACD and CoACD are also reference material. V-HACD is not a desirable runtime
dependency. CoACD's collision-aware decomposition ideas are interesting, but
the Gravitas-owned path should still be a managed, deterministic toolchain with
explicit settings and reproducible output.

A user-provided algorithm sketch is also worth exploring: grow convex pieces
from adjacent source triangles, keep a triangle only when the convex hull of the
candidate piece does not extend outside the source closed mesh, then start a new
piece when no adjacent triangle can be accepted. This is attractive because it
tries to avoid creating new triangles and may produce fewer pieces than
reflex-edge exact methods. It needs proof around containment tests, hull update
cost, candidate ordering, disconnected regions, narrow tunnels, slivers,
self-intersections, and deterministic tie-breaking.

## Milestones

### Tooling Phase A: Mesh Validation Kernel

- [ ] Define an immutable tooling mesh representation with deterministic vertex,
  edge, triangle, adjacency, and shell ordering.
- [ ] Validate edge manifoldness, boundary edges, duplicate faces, degenerate
  faces, inconsistent winding, disconnected shells, self-intersection
  candidates, and scale/quantization limits.
- [ ] Return structured diagnostics instead of exceptions for user asset
  problems.
- [ ] Add pathological fixture meshes and golden diagnostics.

### Tooling Phase B: Solid Properties And Runtime Export

- [ ] Share or mirror the runtime closed-volume mass-property math.
- [ ] Export baked mass properties, bounds, source scale, and validation
  metadata.
- [ ] Add deterministic serialization for tool output so runtime import does not
  need to recompute expensive topology.

### Tooling Phase C: Collision Mesh Simplification

- [ ] Investigate deterministic simplification strategies suitable for collision
  meshes.
- [ ] Preserve hard boundaries, sharp features, and closed-volume validity when
  requested.
- [ ] Report triangle reduction, volume drift, bounds drift, and feature loss.
- [ ] Keep simplification optional; no runtime mesh should be simplified
  implicitly.

### Tooling Phase D: Convex Hull And Convexity Primitives

- [ ] Implement deterministic convex hull construction or choose a managed
  implementation that can be made deterministic under fixed/quantized input.
- [ ] Add convexity and hull-containment checks used by decomposition scoring.
- [ ] Benchmark hull updates on small, medium, and dense closed meshes.

### Tooling Phase E: Approximate Convex Decomposition

- [ ] Compare at least two managed strategies:
  - hierarchical split/merge inspired by CGAL approximate decomposition and
    V-HACD-style tooling.
  - adjacency-grown source-triangle pieces with convex hull containment checks.
- [ ] Define settings for max pieces, max concavity/volume error, max runtime
  hull vertices, voxel or spatial index resolution, and deterministic time or
  work budgets.
- [ ] Emit stable piece order and stable vertex/triangle order.
- [ ] Produce assets that can feed `LSCompoundCollider` or a future dedicated
  authored convex-piece collider path.

## Evidence Bar

Before this tooling becomes a recommended alpha asset workflow:

- all output must be deterministic for identical input and settings.
- failure modes must be explicit and reproducible.
- generated assets must benchmark against raw triangle BVH on simple and dense
  concave fixtures.
- generated pieces must preserve one runtime collider identity through the
  consuming Gravitas runtime path.
- docs must state that this is offline asset preparation and that physics meshes
  should be simpler than rendered meshes.

## References

- CGAL Convex Decomposition of Polyhedra:
  <https://doc.cgal.org/latest/Convex_decomposition_3/index.html>
- CoACD project:
  <https://colin97.github.io/CoACD/>
- CoACD repository:
  <https://github.com/SarahWeiii/CoACD>
- V-HACD repository:
  <https://github.com/kmammou/v-hacd>
