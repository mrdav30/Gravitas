# Static Surface Continuity And Heightfield Integration

**Date:** 2026-10-15  
**Status:** Deferred architectural proposal; no implementation or new API approved.  
**Order:** Resolve #132 and #133 before terrain-specific work. Reuse any generic
continuity foundation required by #133 instead of implementing it twice.  
**Goal:** Let hosts describe a continuous static environment using bounded,
spatially indexed pieces, while evaluating reuse of existing deterministic
height data for physics and navigation.

## Agreed Direction

- Physics owns geometry, contact boundaries and response; hosts decide whether
  a surface represents terrain, roads, floors or another environment feature.
- Distinguish a logical continuous surface from its storage/streaming pieces.
  Explicit membership expresses intended continuity; geometric validation must
  establish which borders actually join. Membership alone cannot erase edges.
- Members retain bounded geometry, collider identity and spatial membership.
  They do not become one rigid body or compound with a common mass, inertia,
  center of mass or enormous attachment/proxy radius.
- Evaluate GridForge-backed candidate gathering and a member-level BVH against
  the same workloads, reusing current per-mesh BVHs and topology/contact logic.
  Choose index ownership from evidence; avoid redundant authoritative indexes
  and work that scans every member per frame.
- Heightfields are a separate, optional representation. Establish correct mesh
  continuity before deciding whether a specialized backend improves our needs.

## Current Evidence And Ownership

[#132](issue-tracker.md#grv-issue-132---bodyless-3d-static-geometry-does-not-receive-discrete-blocking-response)
owns missing bodyless 3D discrete response.
[#133](issue-tracker.md#grv-issue-133---adjacent-mesh-collider-rims-alter-frictionless-seam-crossing-response)
owns the artificial-rim response difference across separate mesh colliders.
The [supported-geometry closeout](done/2026-10-12-supported-geometry-range-plan.md#phase-4-compatibility-performance-and-096-closeout)
retains the continuous/chunked comparison, repeated original/final traces and
bounded cost controls. It does not establish physically equivalent chunking.

Local Trailblazer source inspected for this proposal:

- `src/Trailblazer/Heightmaps/HeightmapSurface.cs`: immutable short-backed X/Z
  samples, reference-height/step decompression and bilinear ground-Y sampling.
- `HeightmapCompression.cs`: explicit quantization; `FromHeights` clamps to the
  chosen short range. A physics import must account for this authored loss.
- `TrailblazerHeightmapService.cs`: context-owned layer registration and stable
  height-layer selection, not rigid-body collision generation.
- `Navigation/Navigator/Navigator.HeightmapGrounding.cs`: opt-in ground contact
  using `Vector3d.Up`, with optional position projection without velocity change.
  The helper assumes the caller has already established solid-ground traversal.

A physics heightfield is actual collision geometry backed by height samples.
It needs finite-shape intersection, contact witnesses/normals, response, queries
and CCD. Point-height sampling alone does not provide those semantics.
Trailblazer's existing data/quantization is an optional reuse candidate, not an
existing Gravitas collider. Keep both core libraries independent.

`src/Trailblazer.Gravitas` already provides the optional integration boundary.
Its `GravitasNavigator3D` prepares requested movement, commits the pose accepted
by Gravitas, and translates physical support into navigation ground state.
Inspect/reuse that adapter before adding integration behavior. Additional
physics-backed ground sampling or alignment policies belong there when needed,
not in core Gravitas or core Trailblazer. Prefer host adapters/shared authored
input before moving types or introducing a shared package. GridForge owns
spatial organization, FixedMathSharp neutral math, Gravitas physics, and
Trailblazer navigation policy.

## Independent Usage Profiles

These are documentation-level host choices, not a shared runtime mode setting:

- **Height-driven navigation:** Trailblazer samples ground height for agents
  that do not require physical contacts with that surface. No Gravitas terrain
  collider or specialized physics heightfield is required for this use.
- **Physics-backed environment:** Gravitas mesh surfaces provide collision and
  response. Neither Trailblazer heightmaps nor a new heightfield backend are
  prerequisites. `Trailblazer.Gravitas` can connect navigation to accepted
  physical movement/support when a host needs both libraries.
- **Optional mixed population:** A host may use height-driven agents alongside
  physics-driven vehicles, props or projectiles. Define consistent ground data
  only where actors need agreement; do not apply competing height projection
  and physics pose resolution to the same degrees of freedom.

## Surface Agreement When Profiles Share Ground

The same sample grid can describe different interior geometry. Trailblazer uses
bilinear interpolation; a triangle-cell backend describes a piecewise planar
surface. For unit-cell heights `h00=h10=h01=0`, `h11=1`, the center height is
`1/4` bilinearly, but `0` or `1/2` with the two possible triangle diagonals.

Only when both representations must describe the same ground, choose a
canonical physical surface and define navigation agreement: exact shared
evaluation or an intentional, measured approximation. Independent usage does
not require cross-library sampling agreement or new integration machinery.
Decide cell diagonals, normal semantics, hole/layer selection, quantization,
coordinates and rounding. Preserve existing sampling behavior unless an
intentional compatibility change is justified. A single height per X/Z location
cannot represent overhangs; arbitrary meshes and layered environments remain
separate supported cases. Avoid promising heightfield/mesh equivalence without
matching actual surface definitions.

## Candidate Member Lookup Designs

Each `PhysicsMesh` already uses `SwiftFixedBVH<int>` for local triangle lookup.
A BVH over member mesh bounds is a valid candidate for the logical surface:
query/body bounds -> relevant mesh members -> existing local triangle BVHs.
Reuse SwiftCollections' fixed-point BVH operations before writing index mechanics.

Compare the existing grid-backed route with this two-level hierarchy for sparse,
dense and irregular layouts, large member counts, and streaming/edit churn.
Decide whether a member BVH augments or replaces the relevant static-candidate
route; retaining two indexes that do the same job is not a requirement. Include
lookup time, update/refit/rebuild cost, memory, candidate counts and tail latency.
Keep stable member/feature keys and deterministic final processing order rather
than relying on tree traversal order. Audit large-world internal-node bounds
without treating an index node as a rigid collider or computing a world-sized
body radius.

A BVH answers spatial questions; overlapping or neighboring bounds do not prove
that two edges form a continuous surface. Validated border adjacency and surface
membership still supply that meaning to contact generation/reduction. Store
that information alongside the selected index/owners without presupposing a
second topology tree or a new broad-phase framework.

## Design And Implementation Gates

1. **Prerequisites and reuse.** Close #132/#133, inspect their resulting ownership
   and baseline, and identify the remaining work. A necessary generic #133 fix
   belongs to that issue; this proposal must not defer it or duplicate it.
2. **Continuity contract.** Define stable surface/member/border identities and
   exact join validation. Cover real rims, holes, nonmanifold/ambiguous joins,
   different materials/filters and unrelated adjacent objects. Decide behavior
   for registration, removal, unloaded neighbors and geometry edits between
   fixed steps, including cache invalidation and replay/restore compatibility.
   Decide supported static ownership explicitly before accepting moving members.
3. **Bounded integration.** Retain each collider's geometry admission and local
   bounds while permitting a logical static surface to span the world. Validate
   range-safe border comparisons and deterministic local-neighbor lookup. Surface
   membership must not introduce an unbounded body frame or bypass the existing
   compound reach contract. Define how continuity affects retained constraints
   across pairs while preserving explicit contact provenance. Decide public
   per-collider query/event semantics before changing or deduplicating them.
4. **Physics conformance.** Compare a continuous control against equivalent
   partitions: coplanar and sloped joins, corners, real gaps/rims, winding,
   translated world limits and load/unload transitions. Exercise sphere/capsule
   equivalence and whole-shape behavior through full stepping, response,
   grounding, queries and CCD. Check pure 2D/mixed counterparts where applicable.
5. **Optional heightfield decision.** Height-driven navigation and mesh physics
   remain independently supported host choices. If a physics heightfield or
   shared-data workflow is needed, evaluate the existing `Trailblazer.Gravitas`
   adapter and a narrow authored-sample-to-mesh path before a specialized cell
   backend. Measure preparation, memory, rebuild/streaming and warmed contact
   costs with identical physical surfaces. No backend is selected in advance;
   no core-library coupling, shared mode switch or duplicate terrain math is
   required by the independent profiles.
6. **Acceptance and guidance.** Require deterministic replay, zero added warmed
   allocations and no ordinary-path regression beyond measured uncertainty.
   Maintain raw 100% reachable line/branch/method coverage in Release and Lean,
   including coordinated sibling changes. Validate with `UseLocalLsfStack=true`
   and native conformance lanes; released packages remain the release gate.
   Obtain independent ponytail/geometry review. Publish the actual continuity,
   streaming and sampling contract in evergreen guides only after implementation.

## Boundaries

No engine-specific terrain API, renderer/LOD system, terrain editor, automatic
global welding or new spatial-index framework is authorized by this proposal.
Do not silently smooth real features or introduce a height-only substitute for
rigid-body contacts. Record experiments and phase results here; keep one living
design/plan rather than separate phase reports. Update this proposal with the
reviewed design and implementation sequence after the prerequisites are resolved.

## Reference Context

Other engines are evidence about representations and known failure modes, not
the specification for Gravitas:

- [Unity TerrainCollider](https://docs.unity.com/en-us/engine/6000.3/manual/physics-section/physics-overview/collision-section/collider-shapes/terrain-colliders/introduction)
  derives physics collision geometry from height data.
- [Godot HeightMapShape3D](https://docs.godotengine.org/en/stable/classes/class_heightmapshape3d.html)
  is a collision shape with two triangles per grid cell.
- [Godot/Jolt edge handling](https://docs.godotengine.org/en/stable/tutorials/physics/using_jolt_physics.html#ghost-collisions)
  documents internal-edge mitigation and its boundary across separate bodies.
