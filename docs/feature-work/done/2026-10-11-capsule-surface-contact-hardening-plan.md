# Capsule Surface Contact Hardening

**Status:** Complete; GRV-Issue-104 and GRV-Issue-105 resolved on 2026-10-11.

**Primary issue:** [GRV-Issue-104](../issue-tracker.md#grv-issue-104---crossing-capsule-cores-understate-concave-mesh-exit-depth).
**Coordinated issue:** [GRV-Issue-105](../issue-tracker.md#grv-issue-105---curved-contacts-share-a-sample-cap-across-disconnected-planar-overlap-regions).
Resolve both issues together: exact face-ray selection needs the same admitted
region ownership that prevents disconnected regions sharing a sample cap.

## Problem And Confirmed Evidence

Before this work, the shared triangle/capsule relation constructed a
closest-core witness and
reported `radius - distance(core, triangle)`. That is appropriate for its finite
closest-feature query. It does not establish a capsule-boundary ray through an
intersected finite mesh face when the core crosses the face. Gravitas also
reversed coincident fallback normals after contact construction, replacing only
the radial displacement; a complete directional contact must choose its side
before constructing both anchors and depth.

The unchanged-code local-stack Release regressions failed with both coherent
windings and with
two- and four-triangle versions of the same quad:

- A radius-1/2, core-length-1 vertical capsule centered at
  `(3/10, -1/4, 3/10)` reports depth `1/2`. Its admitted upward boundary anchor
  is at Y=`3/4`, so its downward contact requires depth `3/4`.
- A radius-1/2, core-length-8 capsule centered on the plane, directed near
  `(1,1,-1).Normalized`, has both endpoint projections outside the quad. Its
  complete plane section remains inside the quad. The face-region maximum is
  `2r / sqrt(1 - (axis dot normal)^2)`, approximately `1.224744871`;
  detection again reports `1/2`. The existing radial fallback also places its
  paired anchor inside the cylindrical body rather than on its boundary.

The ignored regression patch and logs are under `artifacts/grv-issue-104`.
`red-surface-contract.log` records all eight depth failures;
`red-capsule-anchor.log` separately exercises the angled boundary-anchor check.
Maintained runtime regressions now cover these depth and boundary-anchor defects.

## Contract And Scope

Use the established
[surface-contact contract](2026-10-08-surface-contact-manifold-plan.md#geometry-contract):
actual finite face regions and genuine perimeter constraints remain independent.
For each connected admitted face region, retain both exact signed
capsule-boundary ray maxima, choose the smaller maximum once for that region,
then reduce its finite sample pool. Exact symmetric ties use the existing
canonical plane convention. Individual triangles or samples must not choose
opposing hemispheres within the same region.

Each group admits at most four samples. A wholly contained, single plane
section with positive radius uses one contact when the shape is a sphere or
its capsule core is exactly parallel to the face normal, and the section meets
no genuine perimeter. The unique smooth support point attains both signed
regional maxima, so retaining the selected support preserves the complete
directional depth without adding lower-depth interior rim constraints.
Clipped, disconnected, nonunique and zero-radius cases retain the general
finite sample reduction and independent boundary constraints.

A face ray begins at a point belonging to both the actual filled mesh domain
and the capsule's plane section. A projected endpoint support point is a fast
certificate only when that base point satisfies both predicates. Neither an
infinite-plane support gap nor an endpoint projection alone establishes finite
contact admission.

These are local directional response depths. They do not promise that a
translation clears every future intersection with the entire concave patch.
Whole-capsule clearance remains a useful assertion for the vertical repro and
other cases with an independently established complete directional certificate.
General angled runtime cases require finite witness, boundary-ray and host
response assertions. The separate convex-hull minimum-translation relation
retains its own clearance contract.

Preserve the public FixedMathSharp closest-feature triangle/capsule relation's
classification and documented arbitrary coincident fallback direction. Correct
its positive-core coincident anchor/depth using the actual capsule-boundary ray
through the selected exact triangle witness; retaining a radial offset that
places `pointOnCapsule` inside the capsule would preserve the defect. This
single-witness query remains distinct from a runtime face-region maximum.
Document that intentional correction and update affected math regressions.
Add only the internal finite face-ray operations actually consumed by Gravitas.
Keep the existing finite radial edge/corner query where it establishes a valid
contact. A nonzero closest-core distance alone is not proof that it supplies a
face region's selected maximum.

Do not build a general nonconvex minimum-translation arrangement solver, treat
a nonconvex patch as a convex hull, or copy cone-specific height/nappe/rim
equations into a capsule adapter. Mixed slab contact, pure 2D contacts and
convex-volume contacts remain distinct geometry contracts; audit their consumers
and retain focused parity controls rather than routing them through this path.

## Existing Owners To Reuse

| Owner | Responsibility |
| --- | --- |
| FixedMathSharp `ContactQuadratic` | Exact quadratic-field values, independent-radical comparison and final nearest-even rounding |
| FixedMathSharp `FiniteAxisValueRoot` / `FiniteAxisValueRoots` | Existing degree-at-most-eight isolation, exact polynomial signs, root equality and final threshold comparisons for cross-feature rays |
| FixedMathSharp centered-capsule geometry | Finite side/cap classification, odd raw half-length terms and ray coefficients |
| FixedMathSharp projected-core matching | Exact finite face/edge support-family clipping from FMS-Issue-028 |
| Gravitas `PhysicsMesh` | Committed welded incidence, trusted coplanar domains, genuine boundaries, holes and convex rings |
| Gravitas surface-region reduction | Region ownership and deterministic contact sampling; extract shared operations only for implemented capsule and cone consumers |
| Gravitas `ContactGroupKey` / `ContactManifold` | Independent groups, retained capacity and at most four samples per group |

Capsule-specific equations belong in a focused FixedMathSharp internal owner.
Gravitas owns region orientation, incident-sector policy, contact grouping and
response. Reuse the existing cone boundary incident-fan operation if both
producers need it, rather than retaining duplicate topology walks. A shared
point carrier is justified only by an implemented consumer.

## Phase 1 — Exact Capsule Rays And Finite Events

- [x] Restore the two investigated regressions as maintained runtime tests.
- [x] Establish exact point-ray exits for the finite cylindrical side and two
  endpoint hemispheres, including zero-core, zero-radius, parallel rays,
  tangency, odd raw lengths and unrepresentable final coordinates.
- [x] Apply that shared point-ray owner to the public positive-core coincident
  triangle contact, retaining its fallback direction and exact selected witness.
  Verify raw-coordinate and rigid-anchor forms, including their failure and
  clamp contracts; preserve zero-core and ordinary finite radial controls.
- [x] Establish the complete finite extremum inventory: admitted support
  projections/families, admitted triangle vertices, triangle-edge interval
  endpoints and stationary rays, capsule section-boundary stationary events,
  and body/hemisphere transition events. Reject the wrong root/hemisphere before
  ranking; touching must not turn into a distant alternate exit.
- [x] Keep parameters, section membership and ray depths exact until final
  materialization. The existing rounded capsule interval API cannot establish
  exact event admission or extrema ranking.
- [x] Document coefficient width bounds and event completeness next to the
  implementation. Reuse quadratic/root owners; introduce no new general
  arbitrary-precision arithmetic, downstream limb facade or speculative API.
- [x] Prove independent math fixtures for the vertical capsule, long angled
  cylindrical section, finite edge-limited section, hemisphere transitions,
  support families and one-raw touch neighbors.

**Implementation summary:** The shared public triangle query constructs the
first actual capsule-boundary ray through its retained exact coincident witness,
including arbitrary fallback directions. Raw and rigid forms retain their
existing classification, clamp and failure contracts. Rigid anchors compose
source and target frames before final rounding and retain bounded radial
coordinates separately from the exact finite-core term. Maintained controls
cover zero cores, ordinary radial contacts, odd lengths, tangency, rotated
frames and full-range materialization.

The finite-face producer enumerates admitted support points/families, vertices,
clipped edge endpoints and stationary rays, section-boundary extrema and
body/cap transitions. Admission and outward-hemisphere checks precede ranking.
Some finite rays require two independent algebraic values: the exact quadratic
base point and its boundary depth. For example, a radius-1, core-length-2 Up
capsule and the triangle
`(-1/2,3/8,31/32), (-1,3/4,11/32), (-1/2,3/8,63/32)` on the plane with normal
`(3/5,4/5,0)` have positive maximum
`4/5 + sqrt(19978479/43033600 + (1641/215168)*sqrt(23))`, approximately
`1.507692665`. A single quadratic field cannot retain this value.

The implementation preserves the base field and reuses the existing value-root
owner for depth. A ray equation `A(t) + sqrt(R)*B(t) = 0` supplies a norm
`A(t)^2 - R*B(t)^2` of degree at most four; exact unsquared field signs reject
conjugate roots and the derivative establishes the outward crossing. Rational
coefficients retain their original quadratic rather than a squared quartic.
This also handles independent base/depth radicals: an edge along Z through
`(-3/4,-3/2,0)`, with plane normal `(5,12,0)/13`, has admitted base endpoints
`Z=+/-sqrt(3)/4` and positive depth `(135+sqrt(2001))/52`. No new root framework
or rounded intermediate witness is required. Independent geometry review and
complete FixedMathSharp Release/ReleaseLean raw coverage validate these owners;
the final totals are recorded in Phase 3.

## Phase 2 — Region Integration And Genuine Boundaries

- [x] Choose one signed ray maximum per admitted connected face region before
  constructing contacts. Remove the radial-only post-construction normal flip
  for the new capsule face path.
- [x] Preserve true edges and corners with exact finite intersection and
  incident-fan admission. Reflex corners and hole rims do not authorize a
  fabricated convex fill or an oblique covered-seam normal.
- [x] Preserve every admitted sphere/capsule region (#105) before its sample cap
  and use the same region ownership for hemisphere choice.
- [x] Retain one unique smooth support for a wholly contained positive-radius
  sphere or exactly normal-axis capsule section. Preserve general coverage
  when that certificate does not apply, and validate central impacts, settled
  support and sliding/rolling through the complete host loop.
- [x] Verify all retained anchors, winding, triangle subdivision/order,
  translated/rotated charts, reversed arguments, compound namespaces/materials,
  holes and independent surfaces. Include stationary touch and frictionless
  complete-host-loop response.
- [x] Audit initial capsule/mesh sweeps separately: their initial-contact and
  impact contracts must not silently inherit a discrete surface policy change.

**Implementation summary:** Connected capsule/plane sections own their signed
maxima and sample capacities independently. One hemisphere is selected per
region before constructing contacts, and every retained sample has its own
matched boundary ray and depth. Genuine perimeter features use exact incidence,
filled-sector and region-hemisphere admission. Maintained tests cover five-tab
regions, holes, true corners, parallel edge families, winding, subdivision,
reversed arguments, compounds and complete-host-loop response. Cone consumers
reuse the connectivity and coverage owners while retaining their own geometry.
Pure 2D, mixed slabs and convex volumes keep their existing separate contracts.
The initial-sweep audit recorded the pre-existing covered-seam defect as #108;
its resolution is outside this discrete-contact scope.

**Sampling refinement:** Capsule coverage ranks the finite pool after rounding
each exact base point once directly into the authored mesh frame. The rigid
basis preserves distances and area, while admitted points are convex
combinations of finite authored vertices and therefore remain representable.
Exact admission, connectivity, signed depth, hemisphere and provenance precede
this rounding. Each coordinate error is at most half a raw unit; one sample's
Euclidean error is at most `sqrt(3)/2` raw units and a sampled displacement's
error is at most `sqrt(3)` raw units. Exact metrics rank those final Q32.32
anchors, with exact authored-point order breaking equal metrics. Materialize
the selected capsule boundary independently as bounded radial coordinates
plus the exact finite-core term; never invert a rounded world point or narrow
an unrepresentable whole capsule-local Y coordinate. Existing cone sampling
retains its approved translated world lattice.

**Unique-support refinement — accepted:** A contained sphere or normal-axis
capsule has one strictly curved directional maximum at the projected core.
Retaining only this support removes lower-depth rim constraints and their
additional normal-impulse levers, while preserving the exact regional maximum
and hemisphere. The certificate needs a nonempty single admitted region and no
intersection with the complete genuine perimeter; it does not require every
triangle of a densely subdivided floor to intersect the shape. Clipped,
disconnected, oblique, nonunique and zero-radius sections retain general coverage.

The 112 focused runtime controls pass, including exact frictionless spin and
central impact without normal torque, finite-friction sliding/rolling,
96-frame gravity settling against an equivalent flat cuboid, save/restore,
owner reuse and densely subdivided floors. A three-raw angular-velocity change
exposed during the spinning-sphere control came from rotating the zero-core
geometry frame before radial materialization. Canonicalizing that frame to
identity for both spheres and zero-core capsules removed the artificial lever
without loosening the assertion. Rotation-parity controls compare every world
witness, split anchor, normal, depth and contact ID. Final whole-runtime coverage
and replay validation passed in Phase 3.

## Phase 3 — Cost, Coverage And Closeout

- [x] Benchmark ordinary interior and exposed-edge sphere/capsule controls,
  crossing/angled finite cases, and region scaling. Refine costs introduced by
  this work before closing the phase; do not defer them as new benchmark debt.
- [x] Preserve warmed zero allocation, bounded scratch and deterministic sample
  ordering. Retain exact cheap certificates where their admission proof holds.
- [x] Run complete FixedMathSharp and Gravitas Release/ReleaseLean coverage with
  raw reachable line, branch and method totals at 100%. Run affected replay,
  multi-target, documentation, allocation and sibling-consumer gates with
  `UseLocalLsfStack=true`.
- [x] Request independent geometry and ponytail review of the final unstaged
  changes in both repositories, including test meaning and net code growth.
- [x] Update evergreen collision/API guidance only for lasting contract changes.
  Put concise phase evidence here, close only proven issues, and archive this
  plan when complete. Release FixedMathSharp before validating Gravitas against
  the released package.

**Implementation and measurement summary:** Refinement removes discarded global
ranking and repeated materialization, retains exact base points for selected
samples, and bounds scratch by active coefficient widths. Primitive identity
bases and homogeneous power-of-two reduction remove artificial scale without
changing geometry. Existing quadratic/root interval bounds reduce final exact
threshold searches; exact range, conjugate, hemisphere and nearest-even checks
remain authoritative. A scale assumption exposed by the identity refinement was
repaired in the shared projection-depth owner with a bounded exact search,
including the existing full-range near-unit cylinder regression.

Native Windows x64, .NET 8 comparisons use three warmups and five measured
iterations with `UseLocalLsfStack=true`. These are short-run means, not gameplay
frame-tail budgets. All 14 final workloads report `0 B/op`.

| Ordinary contact | Unchanged legacy baseline (us) | First integrated correct path (us) | Final native (us) | Faster than first integrated |
| --- | ---: | ---: | ---: | ---: |
| Sphere interior | 13.81 | 425.7 | 36.74 | 11.6x |
| Sphere exposed edge | 11.04 | 628.6 | 219.59 | 2.9x |
| Capsule interior | 25.19 | 556.2 | 52.08 | 10.7x |
| Capsule exposed edge | 22.94 | 906.2 | 272.42 | 3.3x |

| Acceptance workload | Final native (us) |
| --- | ---: |
| Angled face sampling | 290.30 |
| Axial face sampling | 35.88 |
| Clipped face sampling | 760.70 |
| Subdivided angled face sampling | 290.42 |
| Connected comb, sphere, 1 / 4 / 8 regions | 319.7 / 1,039.5 / 1,979.9 |
| Connected comb, capsule, 1 / 4 / 8 regions | 378.4 / 1,312.6 / 2,454.4 |

The final native capture is under `artifacts/grv-issue-104/unique-final-cost`;
`baseline`, `integrated-cost` and `retained-point-cost` retain the comparisons.
The retained-point interior paths were still roughly 110–130 us before the
unique-support refinement; the final 36.74/52.08 us paths retain the complete
physical support directly. This is accepted same-phase refinement of the costs
introduced by the new contract, rather than deferred benchmark debt.

The final path does not claim performance parity with the cheaper legacy
13.81/25.19 us interior paths. Those paths omitted the complete directional
contact and region semantics established here. Exact genuine-edge constraints,
clipped algebraic rays and multiple finite regions still require additional
work; their costs are explicitly measured above. The accepted implementation
keeps those complete paths, bounded scratch and warmed zero allocation, with
ordinary unique support avoiding their unnecessary event/replay work.

**Rejected experiments:** Global-support/points-only and radial-ranking modes
are not retained; their extra paths and state did not justify the measured
benefit. The complete finite event/ray path remains authoritative. A scoped
`SkipLocalsInit` experiment was also reverted because its modest gain did not
justify compiler configuration and continuing initialization constraints.

**Verification and closeout evidence:** FixedMathSharp's final
`math-Release-coverage-9` and `math-ReleaseLean-coverage-9` captures both pass:

| Configuration | Passing tests | Raw covered lines | Raw covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 5,330 | 57,821 / 57,821 | 15,560 / 15,560 | 4,345 / 4,345 |
| ReleaseLean | 5,309 | 57,914 / 57,914 | 15,560 / 15,560 | 4,341 / 4,341 |

Gravitas's final `unique-final-Gravitas-Release-coverage` and
`unique-final-Gravitas-ReleaseLean-coverage` captures also pass:

| Configuration | Passing tests | Raw covered lines | Raw covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 5,257 | 46,283 / 46,283 | 14,526 / 14,526 | 4,807 / 4,807 |
| ReleaseLean | 5,192 | 46,281 / 46,281 | 14,526 / 14,526 | 4,806 / 4,806 |

The modified math complexity-register entries match both configurations and
record 100% line/branch coverage. Both Gravitas solution builds pass for
`netstandard2.1` and `net8.0` with zero warnings or errors. All six shared replay
fixtures match their expected hashes and each other across the native Windows
x64 Release/ReleaseLean captures under `unique-final-replay`. Sibling-consumer
tests pass in both configurations: FixedMathSharp.Chronicler 49, SwiftCollections
FixedMathSharp 43, and GridForge 1,171 tests per configuration. Both DocFX builds
pass with warnings as errors; local API/Markdown links and main partial content
tags pass. Independent geometry and ponytail review found no production blockers.

The final shared-owner parity capture under `final-parity-cost` completes all
11 cone and 14 cylinder/mixed circle-slab workloads, each at `0 B/op`. Cone means
range from 92.82 us (quad interior) to 2,484.52 us (thin tab), with every mean
below the earlier accepted phase-3 capture. Cylinder/circle-slab means range
from 15.85 us to 749.94 us; representative side-face controls are 19.91/20.11 us
and oblique-rim controls are 515.95/537.38 us. Earlier captures use different
iteration settings and the stack has evolved, so these controls establish no
observed regression rather than attributing every improvement to this change.

Both issues are resolved and this plan is archived. The separate initial-sweep
defect remains tracked as #108. Hosted Linux/ARM runs and released-package
validation were not executed locally. Release FixedMathSharp first, then run
the package-based Gravitas release gates against that version.
