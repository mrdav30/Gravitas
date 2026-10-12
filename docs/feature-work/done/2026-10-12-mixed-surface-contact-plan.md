# Mixed Surface Contact Implementation Plan

> **For agentic workers:** Implement in this checkout with test-first changes
> and independent geometry and ponytail reviews. Leave all changes unstaged and
> uncommitted. Keep concise validation summaries at the end of each phase;
> do not create a separate design or phase-report document.

**Goal:** Close `GRV-Issue-098` by retaining the finite surface constraints
needed for deterministic mixed response, including circle, stadium, AABB and
polygon slabs and related initial mesh-query admission.

**Architecture:** Reuse `ContactManifold` with A identifying the 3D participant
and B the embedded 2D participant. Gravitas owns trusted topology, contact
regions, reduction and constrained response. FixedMathSharp owns exact finite
slab admission, feature geometry, directional rays and final materialization.

**Tech stack:** C# 11, `netstandard2.1`/`net8.0`, FixedMathSharp,
SwiftCollections, GridForge, Chronicler.Hashing, xUnit v3 and BenchmarkDotNet.

**Status:** Complete, 2026-10-12. Geometry, response, query and local-stack
validation gates pass. Native CI and released-package validation remain
integration/release gates, as recorded below.

## Evidence And Decision

The original two-triangle quad reproduces the covered diagonal for circle and
stadium slabs with both coherent windings and selective reversal. Unit square
AABB/polygon slabs centered at X/Z `(9/20,9/20)` reproduce the same defect:
their individual diagonal exit is smaller than the genuine `1/4` vertical exit.
The matching curved-slab full loop introduces lateral position corrections
despite frictionless planar travel. The related initial mixed circle sweep
reports a lateral normal from a triangle witness instead of the finite face.

A square annulus with outer X/Z bounds `[-2,2]` and hole bounds `[-1,1]`
supplies the contract counterexample. A centered stadium of core length `3`,
radius `1/2` and half-thickness `2` meets two disconnected material regions.
The current reported normal is approximately `(0.31623,0,-0.94868)` at depth
`0.65811`; the actual owner still overlaps after that reported exit. Literal
vertical translations preserve contact at `2` and separate just beyond `2`.
An individual feature exit therefore cannot establish whole-union escape.

Use grouped contacts, matching the established 3D surface model. A complete
global minimum-translation solver would need exposed intersections between
configuration-obstacle boundaries and exact joint-event ranking. It would still
supply response only one constraint. Do not introduce that separate algorithm
to retain a flawed reduction policy. Do not merely replace discarded seam
normals with a plane normal: the discarded triangle may have a genuine finite
face or perimeter candidate that must be generated from its geometry.

`#103` remains separate: its solid-target containment normal is derived by a
different query policy. Combine only if an implemented shared owner and its
own regressions actually close it. `#101`, `#102` and `#096` remain outside
this migration. The original 3D cylinder counterpart passes the direct
all-retained-normal control; that establishes this fixture only.

## Contracts And Reuse Boundaries

- Contact generation is distinct from a whole-owner minimum translation.
  Independent admitted surface regions and perimeter constraints remain active
  together. Each retained point has matched anchors, a physical normal,
  directional depth, material provenance and clamp state.
- Use existing `ContactManifold`, `ContactGroupKey`, canonical ordering,
  per-group four-point reduction and retained overflow. Do not add a second
  manifold, a fixed pair-wide cap, a temporary collider or a global mesh cache.
- Reuse `PhysicsMesh.GetManifoldSurfaceOwner`, prepared convex fills, canonical
  true boundaries and `MeshSurfaceBoundaryTopology`. Ambiguous topology retains
  individual domains; winding alone does not establish physical side.
- Exact shape admission precedes grouping. Adjacent triangles connect only
  through an edge/vertex that actually meets the finite slab. Holes, notches and
  disconnected overlap regions cannot become a convex fill or one point group.
- Circle/stadium slabs have flat Y caps. `CapsuleFiniteSectionFrame` describes
  a rounded 3D capsule and cannot stand in for this geometry. Reuse existing
  `TriangleCylinderContact`, `TriangleCircularGeometry`, prism relations and
  exact event/ratio owners before introducing a focused missing operation.
- Keep intersection/separation admission separate from permitted contact
  features. Removing a covered edge from separating-axis tests could admit a
  hole. Physical ranking must filter both analytic and curved-rim candidates
  before selecting their winner or pruning upper bound.
- Compare admission, connectivity, regional direction/depth and provenance
  before rounding. Rank final finite coverage samples with the established
  exact metrics and documented materialization bounds.
- Public single-hit/single-contact inspection is representative, not proof of
  a global escape. Internal response and CCD admission consume every relevant
  constraint before selecting a summary. Preserve public APIs where their
  corrected contract is clear; do not add an adapter that preserves a false
  minimum-exit promise.
- Mixed response keeps 2D Y fixed. Vertical rows affect only a movable 3D
  participant; planar rows use the existing constrained impulse kernels.
  Coupling, awake decisions and islands inspect all rows. Multiple points in a
  single pair use configured iterations; equivalent directional position
  correction must have one owner, rather than applying full depth per sample.
- Reuse accumulated exact/compact mixed impulse kernels for per-step solving.
  Do not add cross-frame mixed warm starts unless measured evidence requires
  them. Any retained state needs explicit mutation/load/pooling/replay ownership.
- Pair enter/stay/exit events remain once per pair. Hash all retained groups,
  rows, identities, anchors and materials with an intentional mixed schema bump.
  Diagnostics may retain a primary summary plus individual response events.

## Phase 1: Mixed Manifold Ownership And Response

**Owners:** `CollisionPairMixed`, `CollisionDetectionMixed`,
`GravitasMixedCollisionService.Pairs/Response`, `CollisionResponseMixed`,
mixed replay hashing and existing mixed response/CCD tests.

- [x] Add caller-owned mixed manifold construction, preserving a single
  primitive contact as one group. Reuse the existing typed mixed contact for
  its planar lever/material semantics where that avoids duplication.
- [x] Give mixed pairs retained manifold ownership. Reset all active rows on
  initialization/separation/pooling and publish atomically through callbacks.
  Keep bodyless/trigger/resting paths and material resolution intact.
- [x] Solve every retained row with per-step accumulation and configured
  iteration ownership. Reuse existing exact kernels; apply equivalent-direction
  positional correction once. Preserve the one-contact fast path.
- [x] Make island coupling and wake decisions inspect every constraint.
  Audit all `pair.Contact`, `TryCollide` and mixed CCD consumers; representative
  contacts may serve inspection but must not hide a closing sibling row.
- [x] Version and hash complete mixed manifold state in stable order; cover
  row identity, transient reset, events, awake membership and diagnostics.
- [x] Prove two independent planes stop both incoming components, movable 3D
  vertical response leaves 2D Y fixed, grouping does not multiply correction,
  and one-row behavior/allocation cost remains coherent.
- [x] Complete independent ponytail review and raw 100% Release/Lean coverage
  before treating ownership migration as done. Geometry regressions remain
  explicitly pending until phase 2.

**Phase summary:** Mixed pairs retain the existing grouped manifold through
discrete response, translational/rotational CCD, awake coupling, diagnostics
and replay hashing. Each row accumulates impulses within one step; equal-normal
position correction has one owner. Full-loop regressions cover independent
walls, vertical 3D response, restitution, compounds, triggers and pooling.
Exact planar levers preserve minimum signed yaw without a spatial round trip.
Oversized impulse banks allocate only on promotion and are reused; ordinary
rows remain scalar. Independent review approved the source; final Release/Lean
and allocation gates are recorded in phase 3.

**Impulse width and rounding contract:** A frozen row keeps its anchors, axes
and inertia throughout one step. Its exact normal denominator is constant, so
retaining and adding numerators avoids denominator growth. The current bound is
2088 numerator bits, 1932 denominator bits and at most 31 accumulator carry bits
for the configured Int32 iteration domain. Oversized tangents retain rounded
Q32.32 integer magnitudes rather than nested radical expressions. Maximum
friction bounds these magnitudes below 2150 bits; the combined tangent numerator
remains below 6014 bits, fitting the existing 96-word disk and 320-word radical
comparison banks. Arrays and row objects are allocated only on successful
oversized promotion; ordinary rows stay on scalar caches. These per-step banks
are cleared before reuse and are not authoritative replay state.

## Phase 2: Exact Finite Surface Geometry And Initial Admission

**Owners:** existing FixedMathSharp triangle/slab and convex-prism geometry;
Gravitas mixed complex detection, surface connectivity/boundary owners and
mixed mesh query reducers. Extend shared owners where production use proves it
is appropriate; do not copy wide arithmetic downstream.

**Geometry proof and implementation boundary:** Initial triangle/slab
admission remains exact and independent of contact generation. A face normal
can apply an impulse only at a physical slab support in that direction.
Maximizing a finite-domain ray is insufficient: when a mesh boundary clips its
source, the winner can have a different physical normal. Original-section
samples have the same defect. A frictionless tilted face then spuriously resists
circular yaw; these models are rejected rather than retained as extra rows.

Compare both exact unbounded plane-support depths to choose the physical
side. A finite triangle, shared edge or vertex meets the projected support
stratum exactly when its maximum equals that cached plane peak. Use this same
predicate for admission and connectivity. A unique support keeps one point;
a genuine support line keeps its two actual endpoints, since normal velocity
and wrench are affine along that line. Axial caps and horizontal stadium/prism
side rectangles are two-dimensional supports: retain their separate clipped
regions and reduce canonical intrinsic extrema plus true-boundary endpoints
with the existing four-point reducer. A straight canonical boundary span meets
the convex support footprint in one interval; its minimum span ordinal orders
multiple support regions independently of authored subdivision. Publish true
perimeters using original-section connectivity before reusing that graph for
support connectivity.

Physical support clipping alone is incomplete for deep finite penetration. A
tall cylinder can meet a tilted finite patch while neither its projected plane
support nor any original perimeter lies inside the slab. If an exactly admitted
owner has no ordinary support/perimeter constraint, generate finite feature
penetration witnesses with the existing segment geometry. Permit source `q`
outside the original slab, but require actual finite-feature ownership, actual
slab-boundary `p`, the slab's outward normal cone, and global mesh support:
with response displacement `d=q-p`, every filled owner vertex must satisfy
`dot(d,v-q)<=0`. This certifies a separating support plane for the entire owner
without convexifying initial admission or admitting a hole. Search both sides.
The fallback can choose an exterior exit instead of a shorter nonconvex hole
exit; it is conservative certified clearance, not an exact union minimum.

The fallback pool is nonempty: a farthest pair between the compact filled
owner and convex slab has an actual owner boundary vertex `q`. Maximizing
`|q-p|^2` gives `2 dot(q-p,v-q)+|v-q|^2<=0` for every owner vertex, so `q`
supports the entire owner in direction `q-p`. The opposite direction belongs
to the slab's outward normal cone at `p`. The existing point/cap/radial or
prism charts enumerate those witnesses, including far radial poles and
degenerate families. This proves existence without replacing finite hole
admission with the owner's convex hull.

All feature certification precedes rounding. Rank the resulting fallback pool
by final Q32.32 depth, preferring unclamped candidates and canonical feature
order for ties. Nearest-even materialization bounds the selected exact depth
within one raw unit of the best **unclamped enumerated candidate**. If every
candidate is clamped, canonical feature order is the stated policy and that
bound does not apply. Do not introduce a separate hybrid algebraic selector
for this final bounded ranking. Final support samples similarly use existing
exact metrics over their Q32.32 anchors.

Circle yaw about its geometric center is a structural null direction for
normal response. Preserve that identity in detected normal Jacobians instead
of deriving spurious torque from independently rounded normals and radial
anchors. A circle-center-to-body-COM lever still contributes legitimate offset
torque; friction keeps the actual surface lever.

A stadium is a central rectangle plus two endpoint disks. The face clipping
owner uses linear triangle/cap/wall constraints and at most one radial quadratic.
The obsolete initial-section/source-ray helpers and dual-disk face quartic are
removed. Genuine oblique perimeter rim events remain in their existing owner.
Unbounded support visits only outward caps and physical footprint poles,
reusing the existing cap-constrained stationary disk chart and rational
three-plane solver with the complete shape admission bank. Convex prism ties
retain their exact lexicographic endpoints; duplicate or collinear authored
vertices add no redundant intrinsic chart. For an oblique support line with a
fixed Y cap, solve that cap and its physical footprint wall against the remaining
triangle/shape planes. This clips the actual line, including a single touching
endpoint, without enumerating unrelated radial or prism charts. Free-Y and
two-dimensional supports retain general clipping; canonical boundary segments
retain the general finite solver.
**Geometry width contract:** Rigid points/edges remain below 198 bits,
conditioned normals below 268, prism rows below 334, radial chart coordinates
below 532 and conic coefficients below 1065. The single radial constraint
produces quadratic roots; reconstruction and depth/sign products remain below
3400 bits in the existing 64-word bank. Flat-section clipping uses the existing
ten-limb point/seventeen-limb root storage: interval products are below 398
bits, radial coefficients below 400 and discriminants below 801. Segment
feature witnesses reuse the forty-word contact bank; minors remain below 800,
radial/point products below 1100 and squared metrics below 2400 bits. Genuine
oblique rim quartics retain the existing circular-rim algebra and root owner.
Source anchors outside Q32.32 are rejected explicitly; representable rigid-frame
anchors near scalar limits remain supported. Final samples use nearest-even
Q32.32 materialization, with exact admission and ranking before that boundary.

- [x] Establish the finite face-section/ray extrema construction for flat-cap
  circle/stadium slabs and polygon prisms. Record width, root degree, exact
  connectivity, tie and rounding proofs here before generalizing an owner.
  Physical support clipping and deep finite-feature fallback are complementary.
- [x] Admit the actual slab/triangle intersection, connect only admitted shared
  features, then group the actual clipped physical support strata. Compare
  both plane-support depths before choosing their common physical side.
- [x] Generate true perimeter edge/corner constraints using canonical segments
  and incident filled fans. Do not promote internal vertices, convexify reflex
  corners or discard a real candidate hidden by a covered seam minimum.
- [x] Keep compound material overrides, part namespaces and public owner IDs
  through recursive manifold transfer. Use retained caller/scratch buffers and
  preserve all independent constraints.
- [x] Route initial mesh-query/CCD admission through the actual finite contact
  geometry. Preserve touching, real rims and motion-independent surface side;
  a nonclosing representative may not hide a closing constraint.
- [x] Turn the original RED cases into maintained contract regressions: all
  four embedded shapes, winding/order/diagonal/subdivision parity, real rims,
  notches/holes, ring strips, independent planes, tilted crossings, full-domain
  anchors, compounds and complete `Simulate` plus `LateSimulate` response.
- [x] Keep whole-shape touch/separation checks for contacts claiming a complete
  exit. For regional manifold depths, check the actual matched directional ray
  and each retained constraint rather than relabeling it a global translation.
- [x] Benchmark matched ordinary, perimeter, oblique and dense-subdivision
  fixtures before/after; prove warmed 0 B and bounded work per admitted region.
  Refine introduced cost in this phase. Do not defer a new regression to the
  benchmark backlog.
- [x] Complete geometry and ponytail reviews, both repos' raw 100% reachable
  line/branch/method coverage, and affected public geometry allocation gates.

**Phase summary:** Manifold ownership, original finite admission, real perimeter
contacts and all-row query/CCD consumers are integrated. Reviewed experiments
removed source-in-section clearance, dual-disk face quartics and canonical
section sampling that could apply false yaw torque. The replacement clips
actual physical supports and reuses the existing graph. Deep finite penetration
has a certified whole-owner fallback; nonconvex touch/separation tests cover
subdivision and winding. Structural circular normal Jacobians retain legitimate
offset torque and friction while eliminating rounding-induced yaw torque.
Compound rows now preserve their leaf namespaces through all publication paths.
The geometry checkpoint passes 324 focused math cases. Independent review
identified and verified the correction of a collinear prism corner-cone defect;
both winding directions and genuine acute corners have maintained controls.
Release and Lean Gravitas suites pass with raw 100% line/branch/method coverage;
the final upstream coverage and integration results are recorded in phase 3.

Matched fixed-runtime measurements retain 0 B/op in all 16 fixtures. Physical
support chart refinement reduces the tilted circle/stadium/box/polygon cases
from 3.21/10.61/3.72/3.76 ms to 1.65/4.98/2.01/2.04 ms on this host. Ordinary
interiors measure 0.25–0.45 ms and the 32-subdivision curved interiors measure
0.90–1.37 ms. Final maintained BenchmarkDotNet fixtures measure tilted
circle/stadium/box/polygon construction at 1.57/4.72/1.98/2.01 ms. Limiting
initial-query admission to the existing finite-start bounds reduces the dense
32-subdivision circle query from 2.21 ms to 0.916 ms; the full swept candidate
bank remains available for later impacts and diagnostics. All cases retain
0 B/op. These are local fixture costs, not game-capacity promises.

## Phase 3: Integration And Closeout

- [x] Run fresh Release/ReleaseLean multi-target builds, complete relevant
  stack test suites and raw coverage with `UseLocalLsfStack=true`. Set the same
  environment property for benchmark-generated child builds.
- [x] Update intentional mixed replay versions/expected fixtures with reviewed
  state changes. Compare shared captures across local Release/Lean; retain the
  native-platform and released-package CI gates without claiming local proof.
- [x] Exercise lifecycle, CCD, mass/load invalidation, query ordering,
  representative inspection, solver iterations and retained-buffer allocation.
  Add no tests-for-tests or coverage exclusions.
- [x] Publish the corrected lasting contact/inspection contract in the wiki;
  keep dated results here and in the owning tracker. Build DocFX with warnings
  as errors and check local links/main partial `<content>` tags.
- [x] Independent ponytail review covers the complete live diff, new files and
  upstream changes. Resolve all introduced issues within scope; record only
  confirmed independent defects or measured unrelated signals in the trackers.
- [x] Close `#098` only when geometry, response and related mesh-query gates
  pass. Archive this completed plan under `done/`, update links and provide
  concise recommended commit messages. Leave the changes unstaged/uncommitted.

**Phase summary:** Final source-stack Release and ReleaseLean builds pass for
both target frameworks with zero warnings/errors. Complete Gravitas suites
pass 5,540/5,475 tests; FixedMathSharp passes 5,653/5,632. Raw reachable line,
branch and method coverage is 100% in all four runs, without exclusions.
Gravitas sequence points are 46,973/46,971, branches 15,194 in both and fully
covered methods 4,898/4,897. FixedMathSharp sequence points are 59,733/59,826,
branches 16,922 in both and fully covered methods 4,522/4,518. Final Gravitas
coverage is under `artifacts/grv-issue-098/query-refined-{release,lean}/`;
upstream captures are `math-release-complete/` and `math-lean-final/` in the
same artifact root.

All six replay fixtures agree with shared expectations and directly between
Windows x64 Release/Lean captures under `query-refined-replay/`. Mixed pair
hash schema 7 intentionally records every retained row; only the eight mixed
lifecycle frame hashes change, with body/event/query/solver-cache sections
unchanged. Full-loop parity, finite holes and boundaries, compound materials,
minimum signed yaw, CCD preview restoration, mass/load invalidation and warmed
allocation guards pass. The 21 maintained geometry/initial-query cases and
four prepared-response controls retain 0 B/op; response controls measure about
140–147 us for 16 pairs and 544–557 us for 64. Short benchmark jobs establish
local smoke evidence rather than precise capacity limits. Captures are under
`final-benchmarks/`, `query-refined-benchmarks/` and
`response-final-benchmarks/`.

Independent geometry and ponytail reviews approved the complete live diff,
including new files and upstream changes; the introduced collinear corner
cone defect was corrected and reverified within this work. Both DocFX builds
pass with warnings as errors, generated API links/assets and modified wiki
links pass, and main partial owners have `<content>` tags. No additional
abstraction or global union-exit solver is required. #098 is resolved and this
plan is archived; all changes remain unstaged/uncommitted.

Two independent follow-ups are recorded: GRV-Issue-109 concerns authored
segment tangency lost through rounded normalization, and FixedMathSharp owns the
finite-slab projected circle/triangle sweep throughput signal. Direct math-only
controls reproduce 13.80/52.63/205.58 ms for 128/512/2,048 triangles at 0 B/op
in unchanged projected sweep owners. Neither is deferred introduced work.

**Integration boundary:** Commit FixedMathSharp first, then update Gravitas's
immutable develop CI dependency pin to that commit before pushing Gravitas.
No future commit SHA is fabricated in this uncommitted change. Native Linux/ARM
lanes have not run locally. Release FixedMathSharp before Gravitas, then
revalidate against released packages; local source-stack results do not replace
those CI or release gates.

## Investigation Evidence

2026-10-12 local-stack Release RED runs reproduce all ten covered-seam cases
across the four embedded shape families, both curved-slab full-loop cases and
both initial mixed circle query cases. The two real outer-rim controls pass.
The separate ring proof fails after validating literal vertical touching and
separation; the corresponding direct 3D cylinder control passes. Ignored
`artifacts/grv-issue-098/` retains the probe source, TRX and logs. These prove
pre-existing behavior, not completion of the implementation or fresh coverage.
