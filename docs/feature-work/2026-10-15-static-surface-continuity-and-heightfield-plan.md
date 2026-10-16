# Static Surface Continuity And Heightfield Integration

**Date:** 2026-10-15  
**Status:** Phase 1 in progress, 2026-10-16, to resolve
#133/#139 together. Heightfield and alternative-index work remain deferred;
no new runtime API has been implemented.  
**Order:** #132 is resolved. Resolve #133 and its confirmed pure 2D counterpart,
#139, through the generic-continuity phases below before terrain-specific work.
Keep one implementation plan rather than duplicate the prerequisite design.  
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
resolved missing bodyless 3D discrete response using the existing solver and
zero-mobility endpoints. Bodyless and explicit-static support now agree.
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

## Generic Continuity Contract Proposed For #133/#139

The prerequisite is substantial: contact generation currently sees each member's
finite boundary independently. Removing sideways normals in response would
leave artificial face samples and torque, and would not fix mixed CCD. Advance
this generic portion of the plan rather than implement a temporary seam filter
and replace it during terrain work. The user approved resolving #133/#139
together through this plan; final API naming and arithmetic feasibility remain
Phase 1 decisions.

- **Explicit membership:** Prefer a context-owned static-surface handle with
  creation, atomic member replacement and disposal. Separate typed 3D mesh and
  2D surface membership. Mixed contacts need both directions checked: 2D movers
  against 3D surfaces and 3D movers against embedded 2D pieces. Host Y and slab
  half-thickness must preserve real steps rather than assume planar continuity
  proves identical embedded geometry. Use existing dimensional physics-service
  ownership rather than add a second world registry.
  One membership per collider initially. Final API names are not committed.
  Membership permits geometric matching within that set; it does not assert
  that all pieces touch. Disconnected members retain their genuine boundaries.
- **Eligible pieces:** Bodyless and explicit-static colliders share the contract.
  Independent kinematic/dynamic bodies remain independent. Start 3D admission
  with concave mesh surface geometry and 2D with AABB/convex polygon pieces;
  3D pieces form finite sheets; 2D pieces form a filled planar union whose
  interior shared edges cease being perimeter. Do not reinterpret closed convex
  volumes as thin sheets or invent a 2D mesh type. Audit compound leaves before
  deciding whether they can join: do not silently merge a compound's own internal
  boundaries as a side effect.
- **Exact joins:** Validate actual committed boundary spans, including unequal
  subdivision and partial edge coverage. Only a nonzero geometrically covered
  span can cease being a boundary. Point-only touches, gaps, overlapping fills
  and ambiguous/nonmanifold joins do not authorize erasing rims. Preserve real
  creases at folded joins rather than smoothing normals. No proximity welding,
  implicit snapping or tolerance based on rounded world vertices. Exactness is
  relative to committed scaled-local geometry and stored rigid transforms,
  before world-point materialization; visually equal rounded endpoints can
  still describe a real gap under that metric.
- **Local frames:** Retain each member's origin, rotation, admitted local data,
  partitions and existing triangle BVH. Support independently transformed pieces
  without a common-origin/rotation restriction. Do not build a giant combined
  `PhysicsMesh`, narrow absolute-world differences or round neighboring vertices
  into one chart before proving a join. A logical surface has no aggregate body
  reach, mass or inertia. Its per-query work must follow local candidates and
  admitted connectivity, not scan every member of a world-sized set.
- **Physical domain:** Join finite-shape-admitted regions across validated
  neighbors before selecting face samples and true-perimeter constraints.
  Connectivity alone does not certify a whole-shape exit. Coverage must respect
  the moving participant's filters, trigger policy and live neighbor state.
  An excluded or unloaded piece cannot hide an eligible member's rim.
- **Identity and materials:** Keep each collider, pair, query hit and geometric
  overlap event distinct. Deduplicate physical constraints where appropriate
  without manufacturing contact exits for an overlapping member. Every retained
  witness keeps its supplying member's material and feature provenance; material
  discontinuities alone are not holes. Define deterministic ownership at an
  exactly shared feature before publishing reduction logic.
- **Transactions and lifetime:** Change membership only between full fixed
  steps. Rejected replacement preserves the previous usable state. Bind members
  to collider lifetimes; deactivation removes membership, reactivation needs an
  explicit declaration, and context reset/disposal invalidates its handles.
  Geometry edits publish the affected join changes coherently: moving a member
  can open a real gap. Invalidate affected neighbors' contact, support, warm-start
  and CCD dependencies even when their own geometry did not change. Rebuild
  locally rather than rebake every member after an edit.
- **Replay and restore:** Membership is authoritative; adjacency is derived.
  Include membership in replay hashes using stable identities, not addresses or
  recycled collider IDs. Define explicit recording/reconstruction for host-created
  handles and member lifetimes, following existing Chronicler patterns. Test
  populate-existing and continuation; a disposable handle is not itself a saved
  authoring identity. Keep derived caches out of the authored representation.

## Generic Continuity Implementation Sequence

Every runtime phase must preserve raw reachable 100% line/branch/method coverage
in Release/Lean, relevant replay behavior, zero added warmed allocations and the
ordinary warm baseline, using `UseLocalLsfStack=true`. Refine introduced costs
before handoff and obtain independent ponytail review. Phase 4 consolidates the
evidence; it does not postpone these requirements. Keep experimental membership
internal until response, query/CCD and restore semantics are coherent.

### Investigation - Complete, 2026-10-16

On the unchanged committed stack, settle both explicit-static and bodyless
floors for 32 frames, then repeat the original 64-frame sphere/capsule crossing.
All four split controls first lose speed at frame 30: maximum X-speed loss
`0.024248116184026003`, final X `1.9498989908024669` and velocity
`0.975751883815974`. Continuous controls finish at X `2`, velocity `1`.
Each case repeats byte-identically. This updates the earlier bodyless probe,
which did not settle, to exercise #132's corrected support path equally.

The pure 2D counterpart also reproduces: circle/capsule against either adjacent
AABBs or convex polygons loses `0.02416824991814792` speed at frame 30 and
finishes at X `1.9501529859844595`. Continuous controls again finish at X `2`,
velocity `1`; all eight cases repeat identically. Record this as #139, with
shared design review and dimensional implementation rather than a new mesh API.
Mixed runtime probes now confirm the same defect in both directions. Planar
circle/capsule movers crossing a split vertical 3D sheet first lose speed at
frame 30 and stall or cross through the sheet. 3D sphere/capsule movers crossing
adjoining embedded 2D AABB slabs stop at X=-0.5 at frame 23. Continuous controls
finish at X=2 with unit X velocity. Bodyless and explicit-static floors agree.
All 16 cases repeat identically; an independent rerun matches the complete
`mixed-baseline/{results,root-rerun}.json` payload byte-for-byte against the same
eight frozen modules. Reported contact rows are fresh post-step narrow-phase
inspections, not retained solver-manifold captures. Mixed CCD remains a later
implementation gate.

Independent ponytail and dimensional reviews traced face sampling, true-rim
generation, event ownership, filters and neighbor invalidation. They reject a
response-only normal filter. Existing exact rigid projections, anchored points,
coplanar topology and finite-section machinery are the reuse starting point;
their existence does not prove multi-frame reduction needs no math extension.

Ignored `artifacts/grv-issue-133/original-settled-terrain.json` and
`parity/{original,binding,source-hashes}.json` retain fixtures and loaded-module
bindings. The eight-module baseline uses Gravitas `8A7E5C4D` and production inputs
matching committed `fa7da28`; it is the retained tested #138 runtime, not a claim
that binaries were rebuilt at this documentation commit. The frozen baseline and
six terrain/five ordinary warm controls are under `artifacts/grv-issue-133/perf`.
The six terrain baseline guards pass eight repeats each with zero warmed
allocations. No candidate timings, new runtime behavior or coverage result is
claimed here.

### Phase 1 - Prove Bounded Cross-Member Geometry And Ownership

- [x] Review the combined #133/#139 scope: the user approved resolving both
  through this plan, with heightfields and new indexes deferred.
- [x] Supply exact capsule math prerequisites: member-scale depth ordering and
  original-edge rational clipping with reconstructible events. Keep the joined
  domain and final witness-ownership proof as the separate gates below.
- [ ] Add maintained full-loop regressions for the confirmed 3D and 2D crossings
  and a vertical mixed surface with a planar response normal. Use existing
  scenario helpers; do not test probe/report plumbing for coverage.
- [ ] Demonstrate exact boundary coincidence, coplanarity, opposite-side fill,
  partial coverage and shared-corner handling across independent member frames.
  Extend existing `PhysicsMesh` topology and `MeshSurfaceConnectivity` access
  only where needed; retain per-member feature identities. Use actual committed
  collider centering/transforms in the runtime fixtures: an arithmetic proof
  over synthetic frames does not establish chunk-authoring equivalence after
  independently rounded poses or centering.
- [ ] Prove face-region reduction across those frames before materialization,
  with explicit width/rounding bounds and a correct material/sample owner.
  Reuse FixedMathSharp's wide rigid projections and anchored geometry. If a
  missing operation is policy-neutral math, add the narrow operation upstream;
  keep connectivity, physics selection and failure policy in Gravitas.
- [ ] Measure the two-member seam/interior path against the frozen control.
  Refine introduced costs in this phase. Stop and revise the design if exact
  multi-frame reduction requires a large parallel geometry solver, a global
  mesh rebake or a resolved ordinary-path warm regression.

Record a concise phase result here; keep #133/#139 open until their full runtime
and lifecycle gates pass.

**Progress, 2026-10-16:** Isolated experiments, independently rerun against frozen
FixedMathSharp `5C26BDA7`, pass 24 named join cases with 109 symmetry/reversal
evaluations and six three-member pair classifications. Existing anchored
numerators suffice for exact coplanar span predicates, including partial spans,
different frames and world-limit positions. Conservative 3D determinant/plane
bounds fit `Signed576`; 2D determinants fit `Signed320`. Pairwise matches alone
do not prove trusted topology: a valid subdivided join and ambiguous overlapping
coverage can produce the same pairwise boolean matrix. Folded cases only prove
that coplanar erasure is rejected; folded normal fans remain unimplemented.

The capsule experiment reconstructs 48 native events and passes 36 exact
cross-frame containment/outside checks, including irrational coordinates and
unequal frame denominators. It matches 32 physical point pairs; 17 raw depth
comparisons differ despite equivalent physical events. Existing comparisons are
correct for their single-frame contract; pooling them unchanged is invalid.
Reuse the moving-shape chart with retained member/frame provenance and add
scale-aware exact comparison in the existing math owner. A rounded-depth oracle
is diagnostic only. Partial boundary spans also need exact clipped parameters
or framed endpoints, not rounded replacement segments. Cone/slab metrics require
their own verification. `geometry-proof/` and `capsule-proof/` under the ignored
artifact root retain source/binary bindings and root reruns. Maintained runtime,
full topology/reduction, coverage and candidate cost gates remain open; Phase 1
is not complete and neither issue is resolved.

**Implementation progress, 2026-10-16:** FixedMathSharp now provides exact
coplanar capsule-depth comparison across member scales and closed rational
segment clips for its existing plane-ray events. Physical depth is
`t * sqrt(N·N) / (2D)`; raw `t` comparisons across different `D` are invalid.
The new comparison requires the same capsule chart and certified coplanar
primitive normals with the same squared metric. It compares rational/quadratic
values and retained algebraic roots without rounding depths or re-isolating
transformed polynomials. Positive divisor growth is included in the existing
root-separation proof; scaled proportional-polynomial identity avoids refining
exact ties to that worst-case bound. The ordinary entry retains its prior
comparison path.

Clips retain the original edge plus exact parameter bounds. They emit their
own rational endpoints and filter original section/stationary events, including
targeted reconstruction. Full intervals use the existing default path; point
intervals remain closed. For normalized member/capsule rotations and full raw
authored points, doubled transformed coordinates/differences are below 198
bits. Combining them with positive rational bounds below 319 bits gives endpoint
coordinate numerators below 518 bits, fitting `Signed576`; point-ray products
remain within the existing 4096-bit fields and 3072-bit discriminant storage.
Clipping does not change section/stationary polynomials or their radicands.
Focused cases include opposite rays, zero/nonzero cores, exact endpoint ties,
sub-raw distinct points/depths, rejected section/stationary replay and large
coprime plane normals with independently rotated frames.

Independent ponytail review removed a same-frame caller change that added no
behavior. These are internal math prerequisites, not an integrated continuity
feature: Gravitas still needs the prepared joined boundary domain, stable span
and member provenance, aggregate normal fans and face-sample reduction. Do not
add synthetic clipping calls to existing wholly exposed single-mesh edges.
Cone/slab scale proofs, actual committed-collider fixtures and the corrected
two-member runtime cost remain open before Phase 1 can close.

Validation of this math increment uses `UseLocalLsfStack=true`: both complete
solutions build without warnings in Release/ReleaseLean. FixedMathSharp passes
5,912/5,891 core cases and 49 recording cases per profile; Gravitas passes
6,683/6,615 cases. All four core coverage captures have 100% raw reachable
line/branch/method coverage. Six native replay fixtures agree with shared
expectations and directly across Windows x64 Release/Lean; other OS/architecture
lanes were not run locally. Frozen inputs, raw captures and replay comparison
are under `artifacts/grv-issue-133/phase1/{fms-warm-final,coverage,replay-warm-final}`.
The expanded arithmetic/event guards pass 128 warmed repeats at 0 B, including
all four quadratic/algebraic representation pairs and clipped event generation.
Independent geometry and final ponytail reviews found no remaining source
blocker; the existing complexity register records the arithmetic invariants.

The final balanced original/candidate/candidate/original arithmetic experiment
retains all 132 actual observations, four generated-child dependency/fixture
bindings and 22 successful case launches. Mean microseconds per fixture batch:

| Ordinary control | Frozen original | Final | Difference |
| --- | ---: | ---: | ---: |
| Same-frame depth comparison, 38 pairs | 34.846 | 33.904 | -2.7% |
| Unchanged containment control | 151.246 | 151.395 | +0.1% |
| Plane event production | 411.975 | 395.646 | -4.0% |
| Segment event production | 398.938 | 385.343 | -3.4% |

Both paired blocks are lower for the affected ordinary rows; containment has
opposite block signs. With two launches per variant, accept baseline preservation
at this measurement resolution rather than infer a causal or engine-wide gain.
New-entry budgets per 38 comparisons are 36.526 us with matching frames and
68.182 us across frames; clipped production is 58.372 us per batch. These are
math-kernel costs, not corrected seam
simulation costs. Every measured row allocates 0 B; the expanded 128-repeat
guard also passes at 0 B against the actual Release/Lean math graphs. The final
Release math is `7855DA98`, Lean `9341D618`;
`perf/selection/acceptance-final.json` summarizes the accepted gate and
`perf/selection/warm-final-*` retains
actual child bindings, source manifests, all observations and the paired
analysis. Earlier slow-tie and incomplete-binding runs are diagnostic only.

### Phase 2 - Integrate Static Membership And Contact Reduction

- [ ] Implement the reviewed membership operations through existing dimensional
  services and collider lifecycle owners. Cover invalid ownership/roles, atomic
  replacement, removal, disposal/reset and ID/lifetime reuse.
- [ ] Integrate the joined domain into 3D face-region and genuine-boundary
  generation, including capsule/sphere and cone paths; establish cuboid,
  cylinder, convex/concave mesh and compound-source behavior rather than assume
  rounded-shape success covers them. Retire superseded local-only branches when
  the shared domain can own both cases; preserve the independent-member fast path.
- [ ] Implement #139's dimensional boundary reduction without projecting 3D
  sheets into 2D volumes. Test circles, zero-core capsules, nonzero capsules and
  polygonal bodies against connected convex pieces and genuine concave corners.
- [ ] Apply the same ownership/eligibility contract to mixed finite slabs.
  Verify connected 2D pieces as embedded mixed targets too, with matching and
  differing host Y/thickness. Preserve per-member events and query identity
  independently of solver rows, with deterministic ordering and correct material
  provenance.
- [ ] Exercise real ends, holes, gaps, partial spans, point touches, folded and
  ambiguous joins, winding reversal and member-order reversal. Test free as well
  as frozen rotation and complete-shape clearance behind the first witness.

### Phase 3 - Continuation, Queries And Continuous Collision

- [ ] Extend existing dependency invalidation to affected neighbors for geometry,
  filtering, trigger state, removal and membership changes. Test sleeping contact
  and query-grounded bodies, including expose-a-rim transitions without a local
  pose change. Verify the current step uses one coherent committed topology.
- [ ] Audit 3D sweep/initial-overlap and mixed query/CCD certification, and 2D CCD
  against the same physical domain. Cover fast seam crossing, rotation, retained
  tokens and true-edge impact; keep per-collider query results distinct.
  A horizontal 3D floor cannot prove planar mixed sliding parity.
- [ ] Complete authoritative membership hashing, explicit recording/restore and
  host reconstruction, including unloaded/replaced members and context reuse.
  Extend shared native replay conformance with reviewed expectations; do not
  weaken existing unrelated fixtures to accommodate a seam correction.

### Phase 4 - Close #133/#139 And Publish The Supported Contract

- [ ] Validate Release/ReleaseLean, raw reachable 100% line/branch/method coverage,
  relevant replay lanes and warmed allocation guards using `UseLocalLsfStack=true`.
  Coordinate any upstream math change and preserve its same coverage guarantees.
- [ ] Run repeated, bound seam/interior and unaffected 3D/2D/mixed/query controls.
  Require zero added warmed allocations and no ordinary-path regression beyond
  measured uncertainty. Broken seam response is not a speedup denominator;
  document the corrected-work cost and refine introduced overhead before closeout.
- [ ] Obtain independent geometry and ponytail reviews of the complete changes,
  including deletion/reuse opportunities and every coordinated sibling change.
- [ ] Update evergreen host/collision/dimensional/serialization guidance with the
  implemented membership contract, exact joining requirements and finite edges.
  Close each issue only after its dimensional behavior is covered. Leave
  heightfields and optional index experiments deferred for a separate decision.

## Later Terrain And Heightfield Gates

1. **Reuse the completed foundation.** Close the generic continuity issues and
   reuse their membership, geometry, lifecycle and conformance contracts. Do not
   redesign border identity, query/event ownership or persistence for terrain.
   Select member-level BVH experiments only if existing spatial lookup leaves a
   measured workload gap, using the comparison requirements above.
2. **Optional heightfield decision.** Height-driven navigation and mesh physics
   remain independently supported host choices. If a physics heightfield or
   shared-data workflow is needed, evaluate the existing `Trailblazer.Gravitas`
   adapter and a narrow authored-sample-to-mesh path before a specialized cell
   backend. Measure preparation, memory, rebuild/streaming and warmed contact
   costs with identical physical surfaces. No backend is selected in advance;
   no core-library coupling, shared mode switch or duplicate terrain math is
   required by the independent profiles.
3. **Representation conformance.** Reuse Phase 4's correctness, determinism,
   coverage and cost gates for any selected representation. Add explicit sample
   quantization/diagonal agreement, streaming/edit churn and adapter validation
   only where that representation introduces them. Publish its supported
   contract after implementation; released packages remain the release gate.

## Boundaries

No engine-specific terrain API, renderer/LOD system, terrain editor, automatic
global welding or new spatial-index framework is authorized by this proposal.
Do not silently smooth real features or introduce a height-only substitute for
rigid-body contacts. Record experiments and phase results here; keep one living
design/plan rather than separate phase reports. Record reviewed contract changes
and implementation results here as the generic prerequisites are resolved.

## Reference Context

Other engines are evidence about representations and known failure modes, not
the specification for Gravitas:

- [Unity TerrainCollider](https://docs.unity.com/en-us/engine/6000.3/manual/physics-section/physics-overview/collision-section/collider-shapes/terrain-colliders/introduction)
  derives physics collision geometry from height data.
- [Godot HeightMapShape3D](https://docs.godotengine.org/en/stable/classes/class_heightmapshape3d.html)
  is a collision shape with two triangles per grid cell.
- [Godot/Jolt edge handling](https://docs.godotengine.org/en/stable/tutorials/physics/using_jolt_physics.html#ghost-collisions)
  documents internal-edge mitigation and its boundary across separate bodies.
