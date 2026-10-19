# Mesh Authored-Frame Preservation

## Status And Scope

**Completed:** 2026-10-18. **GRV-Issue-141 is resolved**, clearing its prerequisite
for Phase 6 of the
[static-surface continuity plan](../2026-10-15-static-surface-continuity-and-heightfield-plan.md).
The defect is reproduced on committed Gravitas `0662abef6d342ca074b2865990eb96f1161e7983`.
Implementation, consumer migration, final simplification and independent reviews
are complete. On 2026-10-18 the user explicitly accepted the measured 4-11%
ordinary curved-contact cost in exchange for correctness. Both subsequent
external-review findings are corrected and covered by regressions. Final
Release/ReleaseLean correctness, exact coverage and build validation pass.

Preserve shared authored geometry through mesh centering, scale and rotation.
This is a mesh-coordinate correction, not a new continuity algorithm, terrain
type, spatial index or collision solver. Keep collider placement APIs and
centered mass/inertia meaning unless a demonstrated conflict requires an
explicit contract decision.

## Confirmed Problem

`LSMeshCollider` uses the source bounds center as its default local offset.
`LSCollider.CreateStandaloneSnapshot` materializes that transformed offset as
scalar `Center`. `PhysicsMesh.PrepareScaledGeometry` independently rounds each
scaled vertex-minus-center difference. Narrow-phase and query workers then
interpret the result as a scalar mesh origin plus a rotated local vertex.

This is not generally the same point as retaining the authored transformation
and applying final rounding once at the intended geometry boundary. Separate
pieces with different centers can therefore lose a common authored edge.
Exact continuity correctly rejects their different committed geometry.

Fresh local-stack Release reproduction:

| Geometry | Shared endpoint | Joined spans | Cuboid response |
| --- | --- | --- | --- |
| Original pieces, no rotation/unit scale | Exact match | 1 | Contact and host-step correction |
| Original pieces, 45-degree Z rotation | Different exact points; materialized difference `(-2,10,0)` raw units | 0 | 0 rows; overlapping body remains unmoved |
| Unequal-width pieces, no rotation/fractional scale | Different points; materialized difference `(1,0,0)` raw units | 0 | Artificial boundary retained |

Original source X ranges are `[-4,0.9]` and `[0.9,4]`, with Z=`[-4,4]`, Y=0.
The unequal-width control changes the second range to `[0.9,5]` and uses scale
`(2/3,1,3/5)`. The upright cuboid has size `(4,1,2)` and starts at the rotated
Up direction divided by 10. The probe calls both `Simulate` and `LateSimulate`.
These are small, ordinary geometry fixtures; scalar-domain extremes are not
needed to reproduce either loss.

Ignored `artifacts/grv-issue-141/reproduction.json` retains all eight controls,
exact anchor comparisons, prepared boundary spans, full-step outputs and loaded
module hashes. The probe source and fresh baseline build log are alongside it.

## Recommended Contract

The implemented exact composition is
`T + Rp(offset * ownerScale) + Rg((vertex - reference) * ownerScale * partScale)`.
Placement consumes owner scale only; compound geometry retains the existing
normalized combined rotation. Products share a Q² denominator before rigid
projection; the existing rigid basis remains orthogonal. Exact ingress, finite
admission and selection precede final nearest-even Q32.32 materialization in a
bounded translated physical anchor chart. This intentionally changes vertices
previously displaced by independent center-relative rounding. It does not widen
public anchors or silently admit larger geometry.

The admitted 4096-unit centered reach bounds scaled edge numerators below 109
bits, triangle normals below 219 bits and their rigid projections below 283
bits. Point ingress uses existing 320-bit numerators; relative vectors use 576
bits, and native SAT projections use 832 bits. Larger transient curved products
reuse existing `ContactQuadratic` limb banks, without a new persistent integer
family. The general finite circle/slab sweep bounds coefficients below 1090
bits, radicands below 2182 bits and wall tests below 2510 bits.

Final samples round nearest-even in a bounded physical anchor chart. An odd raw
chart origin can make independently rounded source and cap world witnesses
differ by one raw unit at a half-raw tie. Tests check each chart's exact rounding
and that physical bound; authored incidence, admission, depth and provenance do
not use that rounded sample as their input.

- The same authored vertex under the same admitted geometric transformation
  must identify the same exact point across independently centered pieces.
  Rounding the collider center must not silently translate its geometry.
- Settle the scale/materialization boundary explicitly in the first phase.
  Retaining only a rotated-center remainder is insufficient: independent
  center-relative scale rounding can already split a shared vertex without
  rotation. Any change to the current effective-vertex rounding contract needs
  explicit regression expectations and supported-range analysis.
- Retain affine scale exactly until the consuming geometry operation's final
  materialization. Independently rounding authored scaled vertices is rejected:
  it can bend a previously straight edge and make a planar patch nonplanar.
  Reuse the existing path where preparation proves it represents the same exact
  geometry; do not widen ordinary arithmetic unconditionally.
- Preserve public `Center`, `LocalOffset`, standalone `PhysicsMesh` placement
  and compound-part center-offset semantics. Keep physical mass centers and
  inertia about their documented reference points. A geometry frame need not
  be the rounded collider center or the mass-integration reference frame.
- Collision, support, local/world conversion, closest-point and ray/sweep
  queries, boundary certification and conservative bounds must consume one
  coherent committed geometry representation. No continuity-only correction.
- Retain the supported reach and transactional publication contracts, including
  bounded effective meshes with large authored coordinates or compensating
  placement. Removing centering must not introduce intermediate range failures.
- Preserve deterministic ordering, finite witnesses, materials and provenance.
  Reject actual gaps and retain real rims. No tolerance welding or snapping.

## Ownership And Reuse

Gravitas owns collider snapshots, placement policy, mesh geometry publication,
derived-state invalidation, mass/inertia interpretation and runtime integration.
FixedMathSharp owns any missing policy-neutral exact frame composition,
projection, rounding or materialization operation.

Start from existing `FixedPointAnchor` additive local components, anchored
geometry and wide rigid projections. Establish the narrowest representation
needed by implemented mesh consumers; do not create a general transform graph,
parallel geometry stack or forwarding facade. Existing scalar origin/rotation
entry points cannot simply be assumed to retain an additional exact origin.

Keep centered BVH and mass calculations where their coordinate mapping remains
valid. Reuse their owners and document the mapping; do not maintain two
conflicting versions of the surface. Avoid baking separately rounded world
vertices, which can destroy authored coplanarity and force pose-time BVH work.

Pure 2D polygons retain authored vertices and do not perform automatic mesh
recentering. Verify that counterpart without inventing a 2D mesh API. Mixed
mesh contacts consume the corrected 3D owner and require consumer validation.

## Implementation Checklist

### Phase 1 - Prove The Coordinate Contract In The Small Native Workload

- [x] Resolve the scale/materialization contract and document exact composition,
  width, rounding, overflow and failure invariants before choosing storage.
- [x] Add maintained regressions for the original rotated edge and unequal-width
  scaled edge. Independently assert exact shared points/planes and genuine gaps.
- [x] Implement only the math and runtime changes needed by the production mesh
  consumers in this slice. No source helper whose only caller is a test/probe.
- [x] Prove one complete two-member cuboid workload: certified join, finite
  contact anchors, genuine host response and consistent queries/bounds. Preserve
  existing scalar/centered fast paths only when they are mathematically equivalent.
- [x] Freeze and compare ordinary and corrected-work controls. Require zero
  added warm allocation and resolve any unaccepted ordinary warm regression.
  The measured correctness cost was explicitly accepted on 2026-10-18 and is
  recorded below; no introduced cost is deferred into a later phase.
- [x] Obtain independent geometry/ponytail review of representation, all actual
  callers, net runtime growth and obsolete machinery before expansion.

**Implementation summary:** The committed-stack controls reproduce both original
losses. The production map now retains separate exact placement and centered
scale products in the existing rigid basis. Native contacts, source-plane
certification and full host response share that map; no tolerance welding,
world-vertex bake or continuity-only correction is used. Independent integer
oracles check authored points, depths, finite witnesses and genuine sub-raw gaps.
The ordinary compact path remains when preparation proves identical geometry.
Independent geometry/ponytail review covered the representation and real callers.

The initial restricted prototype established feasibility only. Its old timing
captures remain under `artifacts/grv-issue-141/proof-*.json`; final production
measurements below supersede them. Adverse production measurements, including
unnecessary inverse-bound work, repeated slab preparation, widened rigid curved
work and direct-scale rebuilding, remain preserved alongside the final captures.
They are refinement evidence, not deferred acceptance gates.

### Phase 2 - Complete Mesh Consumers And Lifecycle

- [x] Route every mesh collision/query consumer through the agreed geometry,
  including native hulls, spheres/capsules, cones/cylinders, compounds and mixed
  slabs. Retire superseded rounded-frame assumptions in the same change.
- [x] Cover explicit local offsets, opposite winding, fractional nonuniform
  scale, independent common poses, compound rotation/scale and documented part
  center offsets. Preserve finite rims and disconnected geometry.
- [x] Verify support, nearest points, ray/sweep witnesses, local/world conversion
  and broad-phase/BVH bounds against the same exact source transformation.
- [x] Verify centered mass/inertia, detached inspection followed by attachment,
  geometry/pose mutation, sleeping dependencies, pooling, snapshot restore and
  failed-candidate rollback. Do not reinterpret centers to hide the discrepancy.
- [x] Cover supported effective geometry near scalar faces and large authored
  source coordinates; keep final representability distinct from intermediates.
- [x] Check pure 2D parity and both mixed directions through their actual owners.

**Completion summary:** Native hull/triangle contacts, all curved families,
compounds, ray/sphere/convex/mixed sweeps, posture and CCD certificates consume
the committed geometry map. Existing support and triangle indexes remain in
use; pose publication does not rebuild them. Positive diagonal scale preserves
source topology, eliminating duplicated prepared topology. Independent integer
oracles cover support, shared/gapped geometry, mass/COM/tensors, offsets and
compound placement, scalar faces and large authored coordinates. Lifecycle
regressions cover failed publication, attachment, sleep, pooling and restore
continuation across the existing transports. Pure 2D remains on its authored
polygon owner; both mixed query/contact directions pass their full-suite checks.

### Phase 3 - Validate And Close #141

- [x] Pass Release/ReleaseLean and exact reachable 100% line/branch/method
  coverage in every changed repository using `UseLocalLsfStack=true`.
- [x] Build both target frameworks; verify shared replay and restore continuation.
  Explain any intentional raw geometry change rather than updating expectations
  to accommodate an unexplained difference.
- [x] Compare frozen ordinary mesh/query/mixed/full-step baselines and representative
  scale/pose updates. Broken seam response is not the corrected-work denominator.
  Preserve all adverse/inconclusive measurements and loaded-module bindings.
- [x] Obtain independent final ponytail review, report net runtime/source growth
  and remove duplicate or unused paths. Keep the change scoped to actual consumers.
- [x] Update evergreen coordinate/authoring guidance and record concise results
  here.
- [x] Close #141 with the user's accepted measured cost recorded. Phase 6 of
  the existing continuity plan may resume; it is not implemented by this plan.
- [x] Leave changes unstaged/uncommitted and record the coordinated internal ABI
  dependency. After committing/pushing FixedMathSharp, pin that fetchable commit
  in Gravitas develop CI. Release FixedMathSharp before Gravitas package
  validation/release; no unpublished commit hash or package validation is claimed.

**Validation summary:** Final source passes the following local Windows x64
matrix, with `UseLocalLsfStack=true` carried through every operation. Raw counts
are exact, not rounded percentages; methods use OpenCover's full method summary.

| Library / configuration | Tests passed | Covered / total lines | Covered / total branches | Covered / total methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 6,532 | 64,427 / 64,427 | 20,066 / 20,066 | 4,993 / 4,993 |
| FixedMathSharp ReleaseLean | 6,511 | 64,520 / 64,520 | 20,066 / 20,066 | 4,989 / 4,989 |
| Gravitas Release | 7,272 | 50,066 / 50,066 | 18,252 / 18,252 | 5,304 / 5,304 |
| Gravitas ReleaseLean | 7,203 | 50,064 / 50,064 | 18,252 / 18,252 | 5,303 / 5,303 |

FixedMathSharp.Chronicler additionally passes 49 tests in each configuration.
Both core libraries build `netstandard2.1` and `net8.0` in Release and
ReleaseLean with zero warnings and errors.
The Gravitas suites include shared replay, restore continuation and warmed
allocation guards. The intentional surface-contact fixture representation change
is explained below; the remaining shared fixture expectations are unchanged.
Coverage policy and exclusions are unchanged. Raw Cobertura/OpenCover outputs
and logs are under ignored `artifacts/grv-issue-141/*coverage*-review-final/` and
`*-review-final.log`; earlier captures are retained. Changed text is valid UTF-8,
relative documentation targets exist, whitespace checks pass and both indexes
remain empty. Cross-platform CI
and released-package validation remain external follow-up checks.

## Root Cause And Review Findings

The responsible representation is visible in `43d62949` (2026-07-27,
`refactor!: make collider geometry canonical`): snapshot preparation materializes
the collider center, while mesh preparation independently materializes scaled
vertex-minus-center differences. Adding and rotating those rounded results does
not restore the authored affine map. Separate pieces have different centers and
therefore different rounding residuals. The executed committed-stack reproduction
above establishes the defect on `0662abef`; the history identifies the current
representation's age, not the earliest affected release.

The earlier tests also encoded that representation as their oracle.
`PreparedScale_ShouldPublishPatchOnlyWithItsMatchingGeometry` explicitly expected
fractional scale to kink an authored plane and split its surface ownership.
`WarmedScaleQuantization_ShouldReuseBuffersWithoutExposingPreviousCornerTails`
expected half scale to erase a real one-raw corner. Both entered in `8f1c2888`
(2026-10-08) and retained that interpretation through `0240d9e7`. They thoroughly
checked consistency with quantized geometry, without requiring authored incidence
to survive the transformation. Full coverage and repeatable replay cannot detect
a wrong geometric invariant when the expected result affirms it.

The correction consequently reaches existing consumers rather than just the join
certificate. Leaving support, closest-point queries, sweeps, CCD, bounds or mass
on the old chart would give one collider contradictory geometry. Gravitas retains
placement/physics policy; FixedMathSharp retains exact arithmetic and finite
geometry. The new math files supply implemented consumers, without a new spatial
index, solver, transform graph or persistent integer family. Ordinary rigid
arithmetic remains separate where its smaller proven widths preserve warm cost.
This explains the breadth, but does not exempt the added math from simplification
or performance review.

Fresh independent reviews found and removed an inverse-conversion convenience
method whose only callers were tests. They also identified duplicated rational
edge materialization and an affine nearest-face reducer that ranked already
rounded witnesses. A two-face regression independently distinguishes squared
distances `5/4 raw²` and `1 raw²` whose witnesses round to the same point; a closer
face must still supply the normal regardless of triangle order. These are
closeout corrections within this change, not deferred issues. Constructor-only
topology preparation now releases scratch capacity after copying immutable views.

New independent integer oracles check exact point equality and planarity before
rounding, owner versus part scale, compound placement, finite physical witnesses,
mass tensors, true gaps and scalar-domain faces. Full lifecycle checks exercise
attachment, failed publication, sleeping, pooling and restore continuation. Pure
2D has no per-polygon recentering mechanism, so this particular defect does not
apply; its existing rounded polygon-scale contract is not redefined here.

The surface-contact replay fixture intentionally changes its authoritative and
solver-cache hashes for three frames. Both modes include `pair.3d` manifolds:
anchor origin, rotation, local point/displacement and local-feature hash are
hashed, along with contact identity and warm-start identity. Moving a witness to
the canonical translated physical chart changes that internal representation
even when its physical point is unchanged. Comparison of the captured fixture
retains every body raw state, clock stamp, event and query result; only those six
hash strings change. Existing wall-normal and covered-boundary assertions remain
independent of the hashes. This is an intentional geometry/cache representation
change, not evidence of different body trajectories in that fixture.

## External Review Closeout

The user's additional review identified two valid migration defects. The CCD
exclusion certificate had substituted the ordinary support selector for its
exact vertex scan. The rigid selector rounds and normalizes an inverse
direction; it can therefore choose a lower vertex and certify a false gap.
The existing certificate owner now compares every matching geometry-frame point
exactly, retaining first-vertex ties. Its regression obtains a real Up-normal,
zero-distance public sweep hit for the 3:4 quaternion fixture, independently
checks the raised vertex, and rejects the exclusion. It asserts the unsafe
certificate result, not an unobserved full-step tunneling outcome.

The affine mixed sweep retained exact hit selection but materialized its witness
relative to the host translation. Compensating placement can put that translation
more than one scalar range from a small, admitted physical mesh. The existing
internal sweep now receives the caller's already available bounded mesh origin
for both interior and final-feature materialization. Exact admission, root
ranking and rigid delegation are unchanged. Direct math cases retain rejection
of an unrepresentable chart and prove success with the bounded chart. The public
eight-unit sweep independently derives its committed quaternion/scale plane,
entry distance and finite witness using integer arithmetic. With the old host
chart restored in a controlled run, that exact public sweep returns false; with
the correction it succeeds. The regression grid covers the actual committed
physical location after host-matrix decomposition.

These corrections add eleven net runtime lines and no runtime files or types.
An independent incremental ponytail review found no further correctness or
duplication finding. Pure 2D has no corresponding mesh-frame path to migrate.

Quiet ABBA controls compare the complete frozen pre-review and corrected stacks.
Every measured window allocates `0 B/op` and retains its expected result:

| Targeted operation | Pre-review median range us | Corrected median range us |
| --- | ---: | ---: |
| Four-vertex exact CCD certificate | 3.06-3.10 | 3.98-4.11 |
| Tangential yawing cuboid complete host step | 61.21-61.72 | 61.70-65.43 |
| Ordinary affine mixed initial query | 3,035.18-3,085.08 | 3,046.11-3,132.25 |
| Ordinary affine mixed impact query | 154.30-155.94 | 154.48-155.47 |

The restored exact certificate has approximately one microsecond of additional
kernel cost against the rejected approximate selector. Full-step samples and
affine query samples retain run variation; these ranges are not a zero-overhead
claim. The original pre-#141 certificate also scanned every vertex. Captures,
full dependency bindings and frozen payloads are retained as
`artifacts/grv-issue-141/review-perf-{1-baseline,2-candidate,3-candidate,4-baseline}.json`.

## Final Source And Performance Review

The final source inventory covers 190 maintained files across both repositories;
it does not mean 190 new components. Runtime changes are still substantial:

| Repository | Runtime files touched | New runtime files | Added / removed lines | Net runtime lines |
| --- | ---: | ---: | ---: | ---: |
| Gravitas | 49 | 1 | 1,626 / 957 | +669 |
| FixedMathSharp | 89 | 11 | 7,655 / 2,017 | +5,638 |

The remaining 52 files are tests/benchmarks and documentation. Gravitas adds one
geometry-frame partial and migrates existing owners. FixedMathSharp's new files
extend the existing finite triangle, axis, hull, projection and query owners to
consume that frame. The shared frame itself is internal. This is implemented
geometry support, not infrastructure reserved for later continuity phases.

Simplification removed duplicated prepared topology, pose-time topology rebuilds,
obsolete hull/sweep branches, duplicate rational endpoint materialization and a
test-only inverse helper. Construction scratch is released once immutable
topology is committed. Existing support/BVH indexes, wide integer families and
polynomial scratch owners remain in use. Compact exact geometry and proven
smaller rigid coefficient widths remain available. The source count is reported
separately from tests because independent integer oracles are meaningful evidence,
not a justification for runtime growth by themselves.

The full-host native controls use one dynamic cuboid over one or two declared
surface members, including the original rotated seam and unequal-width fractional
scale. Each operation executes `Simulate` and `LateSimulate`; checks independently
verify the plane, finite anchors and actual response. Four frozen baseline / current
launches in ABBA order give these representative median ranges in microseconds:

| Full host step | Committed stack | Current stack |
| --- | ---: | ---: |
| Single flat member | 94.76-95.34 | 75.05-75.73 |
| Joined flat members | 173.64-174.89 | 103.78-104.44 |
| Single oblique member | 93.67-93.87 | 53.83-57.05 |
| Joined oblique members | 88.31-88.84 | 54.88-56.43 |
| Joined fractional scale | 3,513.96-3,541.28 | 232.97-235.27 |
| Joined nonlattice seam | 3,529.08-3,540.66 | 233.50-236.63 |

All allocate `0 B/op`. Flat controls retain identical body raw state and physical
contact sets, with changed canonical contact ordering. The committed oblique and
fractional cases have incorrect or absent response; their numbers demonstrate
the practical cost of corrected work, not an equivalent-output speedup. The
native improvement also includes the declared filled-hull contact route and
removed repeated work; it must not all be attributed to frame arithmetic.

Ordinary controls were additionally compared in one process with isolated full
dependency payloads, alternating 256-operation blocks for 24 rounds after paired
warmup. A second process reverses load order. Physical witness sets/checksums are
checked outside timing, every case remains `0 B/op`, and loaded module hashes
identify both payloads. Median operation ranges and median paired deltas are:

| Unchanged physical workload | Baseline us | Current us | Paired delta across two runs |
| --- | ---: | ---: | ---: |
| Mesh pose and query | 22.00-22.05 | 22.40-22.87 | +1.1% to +4.1% |
| Mesh rotation | 22.17-22.33 | 22.85-22.92 | +2.7% to +3.3% |
| Mesh scale | 11.19-11.35 | 11.00-11.29 | -1.8% to -0.6% |
| Cone base face | 102.80 | 108.75-110.27 | +5.7% to +7.6% |
| Cylinder base face | 39.29-41.84 | 43.74-43.87 | +5.5% to +11.1% |
| Mixed cylinder base | 165.27-166.32 | 174.52-175.38 | +5.4% to +6.0% |
| Cone oblique | 1,304.35-1,309.07 | 1,399.90-1,410.56 | +7.6% |
| Cylinder oblique | 536.11-537.69 | 558.70-570.72 | +4.0% to +5.1% |
| Mixed oblique | 476.85-479.96 | 519.23-520.44 | +7.8% to +8.9% |

These are separate maintained benchmark operations, not all complete simulation
frames. Paired percentages are computed from paired samples, not by dividing
the separately reported medians. Measurements use Windows x64, i7-9700K,
.NET 8.0.29 Release and `UseLocalLsfStack=true`, without concurrent builds,
tests or profiling. Frozen controls contain the complete committed dependency
stack, rather than mixing old Gravitas with new math.

Direct scale cost was recovered, and focused profiles guided removal of repeated
preparation, magnitude clearing, coefficient copying and redundant polynomial
work. The remaining ordinary curved cost is resolved and adverse. On 2026-10-18
the user explicitly accepted that measured cost in exchange for correctness.
**This is accepted correctness cost, not a performance-neutral result.** It stays
in this plan rather than becoming a pre-existing signal or deferred optimization
gate. The native improvements do not establish ordinary-work equivalence.

Captures are retained under ignored `artifacts/grv-issue-141/`:
`native-complete-{1-baseline,2-candidate,3-candidate,4-baseline}.json`,
`interleaved-readonly-{1,2}.json`, corresponding frozen payloads/bindings and the
adverse refinement captures. The dense ordinary 128/2,048-triangle sweep controls
remain allocation-free without a consistent larger regression; their already
millisecond-scale committed cost is tracked separately as GRV-Benchmark-029.

## Decision Checkpoint

Approved: retain the existing host-facing center/offset contracts and correct
the shared internal mesh-frame representation, beginning with the bounded proof
above. This is broader than substituting two anchors in #140; validate the
representation before changing mathematics and mesh consumer signatures together.
Do not substitute a seam-only workaround to keep the diff superficially small.
