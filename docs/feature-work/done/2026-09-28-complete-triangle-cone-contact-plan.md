# Complete Triangle/Cone Contact

**Status:** Completed; correctness, coverage, resource, review and performance gates verified.  
**Started:** 2026-09-28.  
**Completed:** 2026-09-29.  
**Repositories:** FixedMathSharp and Gravitas.  
**Issue:** [GRV-Issue-086](../issue-tracker.md#grv-issue-086---meshcone-contact-uses-incomplete-triangle-sampling-and-scalar-frame-rejection).

## Goal And Scope

Replace the incomplete mesh/cone triangle contact path with one complete
FixedMathSharp relation. Select collision admission, normal, penetration depth
and both contact anchors from the same geometric feature. Preserve deterministic
lockstep behavior, canonical frames and allocation-free warmed execution.

The approved direction includes rejecting genuine positive gaps in the new
triangle query. Keep the repair focused and navigable: reuse existing arithmetic
and delete superseded downstream heuristics instead of retaining a second answer
path. This document is the single cross-repository coordination record; keep
execution results here rather than creating separate progress documents.

Included:

- FixedMathSharp finite cone/triangle geometry and its focused tests.
- Gravitas's shared convex/concave mesh-triangle consumer, including compound
  dispatch that already reaches that consumer.
- Matching benchmarks, allocation/resource gates, coverage and affected public
  documentation.

Not included:

- [GRV-Issue-087](../issue-tracker.md#grv-issue-087---cone-volume-queries-reject-an-intersecting-mesh-when-the-apex-cannot-enter-its-scalar-frame): cone-volume queries need a minimum-axial intersection witness, not a penetration contact.
- Replacing the closed-convex containment/GJK fallback, its tolerance policy,
  other cone/primitive or mixed cone/slab solvers, CCD, or collision response.
- A generic collider framework, speculative contact margin, new dependencies,
  persistent caches, a new benchmark project, or new CI infrastructure.

## Evidence And Root Cause

The issue tracker retains the exact fixtures and independent geometric proofs.
Three diagnostic cases establish the current problem:

1. A real intersection is skipped when the cone center cannot be represented
   in the triangle's local scalar frame.
2. An ordinary radius-1, height-2 cone misses a triangle crossing its side:
   neither the triangle point nearest the cone center nor one projected cone
   support detects that intersection.
3. An ordinary contact combines a nearest-cone-surface depth with a separately
   chosen triangle-face normal and inconsistent witnesses.

The source inspection is at Gravitas `3471085` / FixedMathSharp `3f7a606`.
Existing Release binaries reproduced these cases through the actual collider
initializer and dispatcher in PowerShell/.NET 10. These are diagnostic evidence,
not the required fresh .NET 8 regression or performance gates.

## Public Geometry Contract

Add `FixedTriangle.TryGetCenteredFiniteConeContact(...)` alongside the existing
rigid-frame contact APIs. It accepts the triangle origin/normalized rotation,
cone center/normalized rotation, full cone height, nonnegative radius, and an
`out FixedContactAnchors` result. The cone's local axis is +Y, with its base at
`-height/2` and apex at `+height/2`.

- Invalid rotations, nonpositive height and negative radius follow the existing
  argument-validation conventions. Zero radius is a supported axial segment;
  retain its actual minimum exit depth, which can be positive when the segment
  pierces the triangle. Do not equate zero volume with zero exit distance.
- Preserve the established `FixedTriangle.IsDegenerate` admission contract,
  including its documented exact-normal-magnitude threshold. Do not invent a
  second triangle-degeneracy policy or change neighboring relations here.
- For admitted triangles, classify authored geometry before final rounding:
  separation returns `false`, exact touching returns a zero-depth contact, and
  overlap returns a contact whose minimum exit depth may itself round to zero.
  A positive gap must not become contact merely because a scalar approximation
  rounds that gap to zero or places it within `Fixed64.Epsilon`.
- Normal, depth and witnesses belong to the same selected feature. The normal
  points from triangle toward cone. Anchors retain the authored triangle and
  cone frames; overflowing relative centers or absolute world points must not
  reject otherwise admitted geometry.
- Keep odd-raw half-height and other exact anchor terms until the existing final
  materialization boundary. Round final ordinary values nearest-even; preserve
  the existing conceptual-overflow `DepthIsClamped` contract.
- Use explicit deterministic feature order and retain the earlier candidate on
  exact depth ties. Do not promise permutation-invariant normals for genuinely
  nonunique ties; unique geometric winners must be winding/reindex invariant.

Keep `TryGetCenteredFiniteConeSupportContact(...)` unchanged. Its documented job
is to project one caller-selected support, not prove complete intersection.
It must not become an alias for the new relation.

### What A Positive Gap Means To Gravitas

For that triangle, a separated result contributes no manifold point and no
contact impulse. The mesh scan still tests other admitted triangles. Broad-phase
bounds may continue to admit the pair, and subsequent simulation steps evaluate
its new pose. Closed-convex containment still uses its existing separate fallback.

This is not a new global separation policy: the fallback's existing GJK tolerance
is outside this repair. A future speculative-contact margin would need an
explicit physics contract. Fast traversal between discrete poses belongs to CCD,
not to falsely reporting a gap as overlap.

## Geometry Design And Reuse

A finite cone is the convex hull of its apex and base disk. For a local support
direction `m`, full height `H`, radius `R`, and
`rho = sqrt(m.x*m.x + m.z*m.z)`, its support value is:

```text
max((H/2)*m.y, -(H/2)*m.y + R*rho)
```

This partitions directions into apex support, base-rim support, and their shared
lateral-generator boundary `H*m.y = R*rho`; the downward axial direction owns
the whole base disk. Triangle face, edge and vertex support regions must be
combined with these cone regions before any candidate can reject or win.

Use a focused cone/triangle owner with one exact winner. Establish a complete
enumeration of interior stationary candidates and region boundaries, including
zero-radial directions, tangent roots, collapsed charts and the radius-zero
case. Squared stationary equations require their original unsquared signs and
both shapes' feature-admission predicates. A root outside its valid region is
not a separation certificate. Do not assume the cylinder's skipped candidates
or pruning proofs apply to a cone.

Reuse these existing responsibilities rather than reproducing them:

| Responsibility | Existing owners to compose |
| --- | --- |
| Exact rigid frames and coordinate products | `WideRationalBasis3d`, `WideRigidProjection`, the existing finite-axis/polytope frame machinery |
| Signed radial support values, stable depth ranking and final normalization | `CylinderContactAlgebra`, `ConvexContactCandidate`, existing convex-contact materializers |
| Stationary base-rim edge algebra and exact roots | `CylinderEdgeContactPolynomial`, `FiniteAxisValueRoot`, `ConvexContactValueRoot` |
| Canonical endpoints, odd-raw dimensions and final anchor terms | `FixedPointAnchor` and `FixedPointAnchorTerm3d` |

Reuse is conditional on the actual mathematical contract. In particular, the
existing radial stationary expression can serve base-rim candidates, but does
not supply cone-region admission or lateral-generator witnesses. Reuse or
extract proven shared leaf operations; do not copy wide limbs, a root isolator,
normalization or rounding into a new cone implementation.

Keep one clear cone contact entry point. Split analytic selection, stationary
features or witnesses only when they form cohesive responsibilities. Keep each
non-private type in its own file. Avoid both oversized owners and one-method
forwarding files. Rename an existing helper only if its actual shared use makes
the old name misleading; migrate all callers without compatibility wrappers.

## Gravitas Integration

Replace the nearest-center-point and projected-support branches in
`CollisionDetection.Cone.cs` with the selected upstream contact. Consume its
normal, depth, clamp flag and two anchors together.

Preserve the existing BVH candidate traversal, minimum-depth triangle reduction,
earlier-candidate tie behavior, pair orientation, materials and manifold
ownership. Keep the closed-convex fallback: a cone wholly inside a solid mesh
need not intersect a surface triangle. No manifold-enrichment redesign is needed.

Delete the obsolete local-center conversion, unchecked cone-frame conversion,
unrelated surface-distance/normal combination and any now-unreferenced helper
after checking all callers. Do not change cone/sphere or cone/convex behavior as
an incidental consequence of cleanup.

## Acceptance And Execution Outline

### 0. Regressions, Proof And Baselines

- [x] Reproduce the three recorded failures as behavioral .NET 8 tests using
  existing fixtures; keep the true separated full-domain control.
- [x] Replace the old mesh-triangle positive-epsilon-gap expectation with an
  independently proved separated case; retain separate convex-fallback tests.
- [x] Derive cone-specific feature completeness, admission, coefficient-width
  and simultaneous scratch bounds; obtain independent correctness/Ponytail review
  before implementing the solver.
- [x] Capture matching cone baselines before production changes. Include base,
  side, apex, oblique rim, touch/gap and full-domain cases in existing benchmark
  projects. Label old incorrect outcomes rather than presenting them as an
  equivalent-correctness baseline. Final preflights enforce corrected behavior.

### 1. Complete Upstream Geometry

- [x] Implement the relation with shared exact arithmetic and winner-only final
  materialization; prove any fast path before bypassing complete feature work.
- [x] Verify base, side, rim, apex and triangle face/edge/vertex winners;
  containment, exact tangency, raw-neighbor separation/penetration, zero radius,
  degeneracy thresholds, winding/reindexing, rotations, odd-raw heights, extreme
  relative frames and unmaterializable world witnesses.
- [x] Check known depth/normal/anchor results, not only hit booleans. Account for
  independently rounded output coordinates with derived bounds where needed.
- [x] Preserve warmed zero allocation and bounded worker-stack behavior using
  existing resource-test patterns, including large extents and distinct frames.

### 2. Downstream Cutover

- [x] Consume the complete relation and remove superseded code.
- [x] Verify actual mesh dispatch in both pair orders, convex/concave surfaces,
  compound routing, multiple-triangle selection and closed-convex containment.
- [x] Verify matched anchors, normals, depth/clamp propagation, repeated results
  and warmed allocations. Keep CCD and unrelated cone policies unchanged.

### 3. Verification, Performance And Closeout

- [x] Build FixedMathSharp and Gravitas in `Release` and `ReleaseLean` for
  `netstandard2.1` and `net8.0`; run both full .NET 8 test suites and retain
  100% reachable line, branch and method coverage without new exclusions.
- [x] Rerun identical cone workloads and shared cylinder/capsule-slab controls.
  Report per-case correctness and cost honestly; investigate material ordinary
  path regressions and record accepted expensive features in the benchmark
  tracker. Do not weaken geometry to reach an arbitrary speed target.
- [x] Obtain final independent correctness and Ponytail review; resolve findings.
- [x] Update affected XML/wiki/API guidance, benchmark notes and the complexity
  register where needed. Keep public docs evergreen and build both DocFX sites
  with warnings as errors.
- [x] Resolve GRV-Issue-086 only after all required gates pass. Keep GRV-Issue-087
  separate. Condense final evidence here and move this document to `done`.

Use `UseLocalLsfStack=true` for coordinated validation; this is not evidence of
released-package readiness. Run only one heavy workload at a time, using the
established two-core, BelowNormal launcher and `DOTNET_PROCESSOR_COUNT=2`.
Review agents must not launch competing builds, tests or benchmarks.

Leave all changes unstaged/uncommitted for user review. Do not release packages
or add release-workaround configuration. Include concise commit suggestions
for each changed repository in the implementation handoff.

## Execution Evidence

- Fresh `Release`, `UseLocalLsfStack=true`, .NET 8 regression run: all four
  focused cases fail for their intended behavioral reasons. Both actual
  intersections return false, the central wall reports depth approximately
  0.447214 instead of 1, and the genuinely separated epsilon-gap case returns
  true. Evidence: `artifacts/grv086/red.log` and `red/red.trx` in Gravitas.
- The eight-row mesh/cone baseline builds without warnings. It records old
  hit/depth outcomes outside measurement; corrected semantic preflights replace
  that temporary diagnostic after integration. Cases are base face, side face,
  apex face, missed side intrusion, oblique rim, rim touch/gap and the
  unrepresentable relative center.
- Cone-specific proof review identified two required degeneracies before
  implementation: zero radius reuses the identical axial-segment cylinder
  relation; positive-radius vertical edges retain their radial stationary
  directions even when the ordinary quartic degenerates. Public regressions
  pin both behaviors.
- The independently reviewed enumeration uses the triangle's two face normals,
  the base pole, lateral-generator normal circle stationary directions and edge
  crossings, and admitted edge/base-rim stationary roots. Constant seam and
  vertical-edge degeneracies have explicit owners. Linear support on pointed
  normal regions moves apex/vertex-rim interior minima to these boundaries,
  avoiding separate solvers for them.
- Generator witnesses use an exact supported-feature slice and a common
  generator parameter. A straddling slice retains the known midpoint parameter
  directly, avoiding unnecessary coefficient growth. Compact generator
  candidates own seam equality before oversized auxiliary directions. Reviewed
  conservative bounds fit the existing 40-word arithmetic and 56-word value
  polynomial owners; source-level and worker-stack checks remain required.
- Old implementation baseline completed successfully: two launches, five
  warmups and fifteen measured iterations, affinity mask 3, .NET 8.0.29.
  The single generated executable was built completely before runtime edits;
  all eight workloads ran against those frozen old binaries. Results and full
  logs are under Gravitas `artifacts/grv086/baseline` and `baseline.log`.

| Geometry | Old mean | Old result | Correct expected result |
| --- | ---: | --- | --- |
| Base face | 19.983 us | Hit, depth 1/4 | Same |
| Side face | 17.525 us | Hit, depth 1/4 | Same |
| Apex face | 28.838 us | Hit, depth about 0.055902 | Hit, depth 1/8 |
| Side intrusion | 15.407 us | Miss | Hit, depth 1 - authored 4/5 |
| Oblique rim | 17.403 us | Miss | Hit, depth 5/16 |
| Rim touch | 17.932 us | Miss | Hit, depth 0 |
| Rim gap | 17.652 us | Miss | Same |
| Unrepresentable relative center | 6.069 us | Miss | Hit |

All rows reported zero managed allocations. Five rows have incorrect old
results, so their timings are repair costs, not equivalent-correctness speed
comparisons. Corrected benchmark preflights now enforce the right hit and
independently established depth results before measurement.

- Initial complete implementation: 34 focused upstream `Release` contact
  cases pass, including one-raw boundaries, odd half-height, radius-zero
  parity, independent oblique and generator witness oracles, rotated authored
  frames, unrepresentable points, warmed allocation and the two 1 MiB worker
  tests with live dirty 64 KiB caller buffers. A further exact generator-touch
  midpoint regression is included in the full-suite pass.
- Gravitas's 23 focused cone tests pass, including the original red cases,
  compound material/anchor orientation, first-BVH-candidate exact ties,
  multi-triangle minimum-depth selection, repeated allocation-free dispatch,
  true full-domain separation and the unchanged closed-convex fallback.
- Independent source review rechecked exact admission before ranking,
  zero-depth winner handling, compact seam ownership and the shared arithmetic
  extractions. Two implementation findings (literal midpoint units and the
  rational-normal span boundary) were corrected and covered before these green
  results. The final full-suite and Lean results are recorded below.
- The first full upstream `Release` run passed 3,874 tests. Further public
  regressions now bring the focused cone suite to 46 passing cases with every
  cone line and branch covered: interior stationary-rim depth and raw neighbors,
  generator endpoint touches, a repeated normal-circle root and an admitted
  zero-factor stationary root. Forty-eight seeded fixtures also agree with the
  independent minimum-axial intersection query and the derived paired-witness
  rounding bound. No coverage exclusions were added.
- The initial complete solver measured 169.3 / 152.1 / 567.4 us for apex /
  base / side faces. Independent review approved reusing the existing
  contained-segment face-disk certificate: axial faces use the cone axis;
  local X/Z faces use a temporary base-centered frame and a base-disk diameter.
  Both signs of axial and face support are admitted first. Only a proven global
  lower bound skips further search; original winner, tie order and witness
  geometry are unchanged. Matching measurements are recorded below.
- Reused the existing cylinder principal-axis depth reducer as one shared
  `TriangleCircularGeometry` operation, without a forwarding wrapper or copied
  arithmetic. Non-generator cone contacts can cancel the normal's scale before
  exact ratio rounding; quadratic generator normals explicitly retain the
  general depth calculation. Independent review verified odd raw dimensions,
  pre-round clamp semantics and the unchanged cylinder call's selected frame.

### Final Correctness And Resource Gates

Both full solutions build without warnings or errors for `netstandard2.1` and
`net8.0` in both configurations, using `UseLocalLsfStack=true`. Both DocFX sites
build with `--warningsAsErrors`: zero warnings and errors. This is source-stack
validation, not published-package availability evidence.

| Repository/configuration | Passing tests | Covered/coverable lines | Covered/total branches | Covered/total methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 3,886 core + 49 integration | 52,585 / 52,585 | 11,808 / 11,808 | 3,925 / 3,925 |
| FixedMathSharp ReleaseLean | 3,865 core + 49 integration | 52,678 / 52,678 | 11,808 / 11,808 | 3,921 / 3,921 |
| Gravitas Release | 4,317 | 56,417 / 56,417 | 16,294 / 16,294 | 5,382 / 5,382 |
| Gravitas ReleaseLean | 4,258 | 56,417 / 56,417 | 16,294 / 16,294 | 5,382 / 5,382 |

Counts use ReportGenerator with the existing CI file filters and canonical
collector attachments, excluding duplicate TRX attachment copies. FixedMathSharp
reports include the core, FluentAssertions and Chronicler assemblies. Exact
uncovered line, branch and method counts are zero; no new exclusions were added.
All test runs have zero failed or skipped tests. Release coverage ran before one
additional fully-contained-triangle case; the final full Release rerun includes
it, with runtime sources unchanged. Lean coverage includes that case directly.

The 47 cone cases include warmed zero allocations for both analytic and
stationary-root contacts, plus ordinary and distinct-frame/large-extent 1 MiB
worker tests with 64 KiB live dirty caller buffers. Full suites also cover every
shared extraction. The focused complexity/CRAP audit confirms all nine registered
cone owners at their full-coverage complexity floor; no metric-only splitting
or coverage waivers were needed. Final correctness, mathematical and Ponytail
reviews found no outstanding implementation or acceptance omissions.

Evidence is retained in each repository under `artifacts/grv086`: the
`Release-coverage` / `ReleaseLean-coverage` collector outputs, matching `*-report`
directories, build/test logs and `docfx.log`. FixedMathSharp's final full Release
rerun is in `Release-final-tests` and its corresponding log.

### Final Cone Performance

The identical two-launch/five-warmup/fifteen-iteration protocol above measures
one dispatched query, not a simulation frame. All nine final rows report zero
managed allocation. All eighteen child launches pass their setup preflights
and exit successfully.

| Geometry | Initial complete mean | Final mean | Final standard deviation |
| --- | ---: | ---: | ---: |
| Base face | 152.1 us | 33.89 us | 0.676 us |
| Side face | 567.4 us | 62.57 us | 1.193 us |
| Apex face | 169.3 us | 55.67 us | 1.331 us |
| Side intrusion | 301.5 us | 283.08 us | 6.321 us |
| Oblique rim | 905.3 us | 850.98 us | 31.440 us |
| Interior stationary rim | Not captured | 1,407.39 us | 32.889 us |
| Rim touch | 703.1 us | 692.58 us | 13.452 us |
| Rim gap | 122.0 us | 120.78 us | 2.479 us |
| Unrepresentable relative center | 1,708.7 us | 1,652.05 us | 31.686 us |

The two retained proof-based reductions cut base/side/apex costs by approximately
78% / 89% / 67% versus the first complete solver. They do not restore the old
heuristic's cost: the known-correct old base/side/gap fixtures remain cheaper by
factors of 1.70 / 3.57 / 6.84. Five other old fixtures gave wrong answers, and the
interior-rim row was added later with its independent 13/256 depth oracle.

BenchmarkDotNet flags multimodal apex and oblique-rim distributions. Small curved
row differences are descriptive, not established gains; in particular, the
interior-rim, rim-touch and rim-gap intervals overlap the certificate-only
checkpoint. Ordinary and general curved costs remain a measured optimization
follow-up in **GRV-Benchmark-019**, not an excuse to retain incomplete geometry.

Reports: Gravitas `artifacts/grv086/initial-complete`, `final` (certificate-only)
and `optimized-and-controls` (final source), with matching launcher logs.

### Shared-Owner Controls

All twelve downstream cylinder/circle-slab cases and all six upstream
capsule-slab cases pass their unchanged preflights at zero managed allocation.
The downstream means are 56.93 / 22.92 us for cylinder/circle-slab cap faces,
32.16 / 32.06 us for side faces, 995.32 / 1,007.06 us for cap intrusion,
1,156.00 / 1,157.18 us for oblique rims, 779.82 / 799.98 us for rim touches,
and 44.03 / 43.60 us for rim gaps.

Those downstream values are 5–16% above the older FMS-Issue-027 control capture.
Do not label that historical comparison an isolated source regression or claim
all shared paths are faster. Source review finds unchanged cylinder geometry
and feature work on ordinary cap/side paths. To separate capture variation from
the shared extraction, a pristine FixedMathSharp `3f7a606` checkout reruns the
existing controls in the same session with the identical protocol. Its 177
cylinder/capsule-slab regressions pass before measurement.

| Capsule-slab row | Fresh old-source mean | New shared-source mean | Change |
| --- | ---: | ---: | ---: |
| Cap face | 60.89 us | 62.27 us | +2.3% |
| Straight side seam | 99.17 us | 101.06 us | +1.9% |
| Rounded end, odd core | 228.98 us | 221.08 us | -3.5% |
| Oblique rim overlap | 1,650.34 us | 1,655.56 us | +0.3% |
| Certified rim gap | 63.62 us | 62.96 us | -1.0% |
| Unmaterialized scalar face | 100.67 us | 103.58 us | +2.9% |

All six reported 99.9% intervals overlap; this comparison does not establish a
material regression. The shared slab slice uses general quadratic products
where its previous private implementation used scalar products. A rational-only
multiply specialization is possible, but was not added without a demonstrated
benefit. Keep one shared slice/arithmetic owner rather than duplicate the old
solver to chase an uncertain small timing difference.

The existing direct `ProjectedTriangleSweepBenchmarks.TriangleCircleSlabContact`
row also reruns against both sources: **16.090 +/- 0.230 us** old versus
**16.151 +/- 0.230 us** new (mean +/- 99.9% interval half-width), a +0.4%
difference with overlapping intervals and zero allocations. This isolates one
ordinary shared cylinder path, not every downstream workload. Together with the
source audit and slab controls, it reveals no material shared-path regression
in the fresh matched checks; it does not attribute all historical drift to a
specific machine condition.

Controls are under Gravitas `artifacts/grv086/optimized-and-controls` and
FixedMathSharp `artifacts/grv086/slab-controls`, `slab-baseline-refresh` and
`control-baseline-tests`. The earlier `fms027-final-benchmarks` is a pre-certificate
checkpoint; `fms027-retained-benchmarks` is that repair's retained baseline.
The final direct control captures are `cylinder-direct-current-final` and
`cylinder-direct-baseline`. Two preceding direct attempts produced no measurements
because BenchmarkDotNet found the nested baseline project's duplicate name;
moving the disposable checkout outside the repository resolved the tooling
failure without runtime or benchmark changes. The temporary checkout is removed
after verification; logs and reports remain in the owning repository.
