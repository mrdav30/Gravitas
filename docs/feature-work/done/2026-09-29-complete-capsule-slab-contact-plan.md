# Complete Capsule/Slab Contact Plan

**Status:** Complete; correctness repair and measured cost accepted on 2026-09-30.
**Created:** 2026-09-29.  
**Repositories:** FixedMathSharp and Gravitas.  
**Issue:** [GRV-Issue-088](../issue-tracker.md#grv-issue-088---mixed-capsulenonzero-core-capsule-slab-contact-retains-incomplete-directions).

## Goal And Scope

Replace the incomplete mixed 3D-capsule/nonzero-core-2D-capsule-slab relation
with complete fixed-point geometry. Determine separation, contact normal and
minimum penetration from the whole shape before rounding. Preserve deterministic
lockstep behavior, canonical contact anchors and allocation-free warmed queries.

FixedMathSharp owns the geometry and exact arithmetic. Gravitas consumes one
narrow internal penetration query and keeps its existing contact, material and
response ownership. This is the single cross-repository design and progress
record; keep proof notes and final evidence here rather than creating parallel
plans or progress reports. The execution outline is gated by the mathematical
proof in phase 0, not a claim that the new candidate enumeration is already proved.

Included:

- Complete 3D capsule versus a planar stadium extruded through flat Y caps.
- Focused reuse of existing cylinder/capsule and constrained-feature arithmetic.
- Gravitas dispatch, canonical support-contact construction, regression tests,
  resource checks and matched benchmarks in the existing projects.

Not included:

- Other mixed cylinder/cone pairs, CCD, broad phase, response or serialization
  redesign; pure 2D and ordinary 3D geometry retain their own contracts.
- A generic collision framework, sampled-direction fallback, new dependency,
  persistent cache, public compatibility wrapper, benchmark project or CI system.
- Optimizing [GRV-Benchmark-020](../benchmark-signal-hardening-backlog.md) or resolving
  GRV-Issue-083. Existing cylinder/circle-slab costs are regression controls here.
- Package releases. Coordinated source validation does not prove package readiness.

## Pre-Repair Evidence

Planning inspection started at Gravitas `5ad457d` and FixedMathSharp `535578e`.
Both worktrees were clean. The tracker retains the earlier diagnostic reproduction;
phase 0 converted that evidence into an enabled .NET 8 regression.

- 3D capsule: radius 1, total height 22, center `(83/4,7/4,0)`, rotation
  `(0,0,-q,q)`, where `q=Fixed64.FromRaw(3037000500)`.
- Slab: `LSCapsuleCollider2D(radius:10,height:22)`, at the origin, unrotated,
  half-thickness 1. Its planar core has length 2 along Z.
- Closest 3D core endpoint: `(43/4,7/4,0)`. Nearest slab point: `(10,1,0)`.
  The squared gap is `9/8 > 1`, so these shapes are separated despite overlapping
  bounds. Extending the slab along Z cannot close that X/Y gap.
- The pre-repair mixed query incorrectly reports contact, depth approximately
  `0.13923784578219056`. The zero-core case already rejects through GRV-Issue-085.

The removed `TryTestCapsuleCapsuleSlab` tested a limited direction set against
the whole slab and kept the shallowest result. It did **not** combine cylinder contacts.
Exact projection along each chosen direction does not make the selection complete.

A middle box plus two end cylinders can represent the slab as a union and support
an overlap-only decomposition. Combining their independent contact depths does
not give whole-shape minimum penetration: clearing one piece can leave another
intersecting. That shortcut is rejected, not part of the proposed fix.

## Geometry And Contact Contract

Retain authoritative centers, rotations, core lengths, radii and half-thickness.
Do not reconstruct the geometry from rounded world axes, scalar endpoint positions
or a saturated relative-center subtraction. The planar local core is +Z after
X/Z embedding, with the existing planar-rotation convention.

- The slab has flat caps, straight sides and rounded ends. It is neither a
  single cylinder nor a rounded 3D capsule.
- An exactly zero slab core reuses the existing complete cylinder/capsule route.
  Every positive core remains a stadium, including one/two-raw and odd-raw lengths;
  `Fixed64.Epsilon` must not decide a shape change or discard its rotation.
- Preserve existing admitted dimensions and frames. Handle a zero 3D capsule
  core and the zero-radius degeneracies admitted by the reused geometry owners;
  do not add new public shape validation or expand collider admission.
- Classify exact geometry first using the signed contact depth defined below:
  a negative value rejects, zero is exact touching, and a positive value is
  overlap (which may round to zero depth). Adding a tolerance or treating a
  failed proof as separation is not acceptable.
- Choose the exact minimum signed support gap before adding the 3D capsule
  radius and materializing depth. Round ordinary outputs nearest-even; retain
  `DepthIsClamped` for conceptual overflow, distinct from an exact maximum value.
- Return a slab-to-capsule normal upstream; Gravitas reverses it to its established
  3D-to-2D convention. Separation returns default outputs.
- Define stable feature/root order and keep the earlier candidate on exact ties.
  Unique winners must respect admissible rigid transforms; tied normals need not
  be invariant under a different enumeration. Zero-core outputs retain existing ties.
- Gravitas retains `BuildCanonicalSupportContact`: anchors remain in the authored
  frames, with the selected normal/depth/clamp and unchanged material propagation.
  This work does not redesign support-anchor selection into a manifold generator.

### Complete Candidate Regions

For a unit world direction `n`, let `u` be world Up, `s` the exact planar slab
half-core, `b` the exact 3D capsule half-core, and `d` the capsule center minus
slab center. With slab radius `R`, half-thickness `H`, and capsule radius `r`:

```text
g(n) = R * sqrt(1 - (u dot n)^2)
       + H * abs(u dot n) + abs(s dot n) + abs(b dot n) - d dot n
signed contact depth = r + min_over_unit_n(g(n))
```

This is a mathematical specification, not a scalar implementation recipe.
Use the existing exact homogeneous/rational representations. Negative signed
contact depth means separation; a negative core gap alone does not, because
the capsule radius can bridge it.

Partition the direction domain into:

1. **Each rounded-end region:** `s dot n > 0` and `< 0`. Each is a shifted
   cylinder/capsule problem, but every candidate must satisfy that region.
   Keep endpoint offsets exact even when no scalar endpoint is representable.
2. **The straight-side boundary:** `s dot n = 0`. On this normal plane the
   disk projects to a line; the cylinder plus capsule core becomes the sum of
   three planar segments (a 2D zonotope). Enumerate its normal-region boundaries
   and admissible interior stationary directions analytically.
3. **Shared boundaries and degeneracies:** poles, cap/side/rim changes, capsule
   endpoint/interior changes, parallel/perpendicular axes, collapsed projections,
   repeated or zero roots, and exact ties must have explicit owners.

The existing cylinder ellipse solver independently reflects basis directions
and retains one largest-positive stationary root using an unrestricted-domain
proof. Those shortcuts cannot simply be applied inside an end region: reflection
may leave that region, and its global winner may be inadmissible. Enumerate all
needed admitted roots, or prove a narrower regional reduction first. Compare
analytic and curved candidates across regions exactly, including gap signs;
never compare already rounded regional depths.

Early exits require a whole-shape certificate. An endpoint-cylinder closest-point
certificate additionally needs the slab-core support-sign condition. Likewise,
triangle/slab code's negative-gap rejection cannot be copied before the capsule
radius has been included. No iteration cap may silently produce an approximate
answer or a false miss.

## Existing Owners And File Boundaries

Paths in this table are relative to the named repository. Prefer a focused slab
partial and the minimum genuinely shared extraction; do not make every cylinder
query carry a new generic feature framework.

| Repository / files | Responsibility |
| --- | --- |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CylinderCapsuleFeatures.cs` | Reuse exact rigid-axis geometry, analytic candidate construction/ranking and complete-depth materialization; preserve existing consumers. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CylinderCapsuleEllipse.cs` | Reuse stationary-polynomial mechanics only where their scale, sign and domain contracts hold. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/ConvexContactValueRoot.cs` and `WideConvexPrismRelations.CylinderPairSideFeatures.cs` | Reuse exact value-root mapping/comparison, not the unrestricted winner/root-ordinal assumptions. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Triangles/TriangleCylinderAnalyticFeatures.cs` and `TriangleCylinderEdgeContacts.cs` | Existing examples of explicit seam ownership and endpoint-region root admission; not a replacement solver for a degenerate triangle. |
| FixedMathSharp new `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CapsuleSlab.cs` | Own the complete slab/capsule relation. Split further only by a demonstrated cohesive responsibility. |
| FixedMathSharp new `tests/FixedMathSharp.Tests/Geometry/Primitives/CenteredCapsuleSlabContact.Tests.cs` | Test the policy-neutral geometry directly, including exact outputs, wide-domain and resource behavior. |
| Gravitas `src/Gravitas/CollisionHandling/Detection/Mixed/CollisionDetectionMixed.cs` | Replace nonzero-core direction selection with the upstream query and existing canonical-contact construction. |
| Gravitas new `tests/Gravitas.Tests/MixedDimensions/MixedNarrowPhaseTests.CapsuleSlab.cs` | Exercise initialized colliders through actual mixed dispatch; reuse existing test helpers. |
| Gravitas new `tests/Gravitas.Benchmarks/CollisionHandling/CapsuleSlabContactBenchmarks.cs` | Measure the same nonzero-core fixtures before and after the repair, with semantic preflight and memory diagnostics. |

The narrow internal entry point should accept slab center, authored planar
rotation, core length, radius and half-thickness; capsule center, authored rigid
rotation, core length and radius; and return `bool` with normal, depth and clamp
outputs. Use the existing +Z planar / +Y capsule local-axis conventions of these
colliders. Prefer `TryGetCenteredCapsuleSlabCapsulePenetration` on
`WideConvexPrismRelations`; finalize its exact representation alongside the phase-0
width proof, not by narrowing inputs for convenience. No new public API is needed.

FixedMathSharp internals remain out of Gravitas public/protected signatures,
serialization, tests and benchmarks. Do not add friendship or downstream wide
arithmetic wrappers. Audit all callers before deleting capsule-specific direction
helpers; retain shared cylinder/cone helpers unless separately proved obsolete.
Keep each non-private type in its own file and avoid tiny forwarding partials.

## Acceptance And Execution Outline

### Phase 0 - Regression, Completeness Proof And Baseline

- [x] Add `CapsuleCapsuleSlab_WithSeparatedRim_ShouldRejectOverlappingBounds`
  in the Gravitas test file above, using the exact recorded fixture. Assert
  bounds overlap, `TryCollide == false` and default/no-contact output. Run it
  against unchanged source and retain the intended assertion failure.
- [x] Build independent expected-result fixtures for straight side, flat cap,
  straight rim, both rounded ends, oblique capsule-interior rim and containment.
  Include a constrained end-region winner missed by unrestricted-root selection
  and a true straight-side-boundary interior winner. Derive support/separation
  certificates or independently bounded exact roots; do not use the new solver
  or a sampled direction search as its own oracle.
- [x] Complete the candidate-enumeration/admission proof, stable tie order,
  coefficient-width bounds and peak live scratch calculation. Cover extreme
  relative centers, exact half-core offsets and wide full slab height; the
  cylinder solver's current 40-word proof is not automatically sufficient.
- [x] Obtain independent correctness and Ponytail review of that derivation
  before implementation. Resolve missing regions, unproved pruning or resource
  limits here; amend this plan if a simpler complete approach is demonstrated.
- [x] Capture a nonzero-core baseline in the existing Gravitas benchmark project
  before production changes. Keep the same fixtures, runtime, configuration,
  affinity and measurement settings for comparison. Record old hit/depth results
  separately and label incorrect rows as repair-cost baselines. Temporary baseline
  diagnostics must not become a retained switch accepting incorrect answers.
- [x] Capture fresh unchanged `CapsuleCircleContactBenchmarks` mixed/3D controls
  and affected upstream shared-owner controls. Preserve full logs and binaries
  used for the baseline; do not rebuild them against changed siblings mid-run.

**Exit:** a reproducible red test, independently reviewed complete design and
resource bounds, and a comparable measured baseline. This gate does not close
the issue or make a no-go decision based on old incorrect performance.

### Phase 1 - Complete FixedMathSharp Geometry

- [x] Add direct upstream regressions first, then implement the reviewed region
  selection with existing exact arithmetic, value ranking and final materializers.
  Keep the zero-core reduction on the existing complete owner.
- [x] Verify all phase-0 feature fixtures with known normal/depth results, not
  only booleans. Include a containment case where a constituent's minimum exit
  does not clear the whole slab.
- [x] Pin strict separation, exact touch and one-raw overlap at a straight rim:
  use the recorded X coordinate with capsule center Y=2, radius `5/4 + raw(k)`
  for `k=-1,0,1`, and unchanged core length 20. Expected hit is `k >= 0`, depth
  is `raw(k)` on contact, and slab-to-capsule normal is `(3/5,4/5,0)`.
- [x] Cover zero and tiny positive/odd-raw slab cores, rotated tiny cores, zero
  capsule core, admitted zero radii, parallel/perpendicular/oblique frames,
  repeated roots and deterministic ties. Assert zero-core cylinder parity.
- [x] Cover opposite-limit origins, unrepresentable endpoints/full thickness,
  sub-raw exact frame terms, midpoint depth rounding and conceptual overflow.
  Use only transformations that preserve vertical-slab embedding for parity tests.
- [x] Reuse warmed-allocation helpers and the existing 1 MiB worker-stack test
  pattern with a live dirty 64 KiB caller buffer. Exercise the actual deepest
  region/root-comparison path; do not assume sequential candidate buffers are
  the peak if called helpers keep scratch live.
- [x] Rerun affected cylinder/capsule, cylinder-pair and triangle/slab tests
  after shared extraction. Review exact admission, ranking and fast-path proofs
  before downstream cutover.

### Phase 2 - Gravitas Cutover And Cleanup

- [x] Route nonzero-core slabs to the new query and reverse its normal exactly
  once for `BuildCanonicalSupportContact`. Preserve zero-core behavior.
- [x] Verify the original regression and raw-neighbor cases through initialized
  runtime colliders. Check anchor origins/rotations, normal orientation, depth,
  clamp/default outputs, material propagation and repeated deterministic results.
- [x] Exercise existing compound and mixed-pair/manifold consumers with a
  meaningful contact; preserve response and dimensional ownership. Confirm pure
  2D and ordinary 3D callers are unchanged except proven shared leaf reuse.
- [x] Delete superseded capsule-only direction checks and unreferenced wrappers
  after the caller audit. Update misleading tests to check correct behavior;
  do not retain assertions whose only purpose is to preserve removed scaffolding.
- [x] Verify warmed zero allocation through Gravitas-owned behavior. Its tests
  and benchmarks must not call FixedMathSharp internals directly.

### Phase 3 - Performance, Coverage And Closeout

- [x] Run matching before/after nonzero-core rows with corrected semantic
  preflights, and rerun unchanged mixed/3D/shared-owner controls. Report units
  explicitly as microseconds per query, plus allocations and run uncertainty.
- [x] Investigate repeatable ordinary-path or shared-owner regressions before
  acceptance. Explain expensive curved cases separately; no accuracy trade-off
  or silently accepted slowdown. Seek user review of material remaining costs,
  recording accepted signals without reopening unrelated optimization work.
- [x] Build both library TFMs and run both complete .NET 8 solution test suites
  in `Release` and `ReleaseLean`. Preserve 100% reachable line, branch and method
  coverage per repository/configuration without new exclusions or hollow tests.
- [x] Check Debug assertions on the focused geometry/resource tests; obtain
  final independent correctness and Ponytail review and resolve all findings.
- [x] Update affected XML, evergreen shape guidance, benchmark notes and any
  changed complexity exceptions. Build affected DocFX sites with warnings as
  errors; never link feature-work plans from evergreen wiki pages.
- [x] Resolve GRV-Issue-088 only after the required gates pass. Consolidate final
  evidence here, move this plan to `done`, and update tracker/overview links.
  Keep unrelated issues and benchmark signals separate.

## Validation And Handoff Rules

Use C# 11, `netstandard2.1` / `net8.0`, and `UseLocalLsfStack=true` for coordinated
source builds. Do not alter package versions or project references to bypass
unreleased dependencies. Released-package validation remains a later release gate:
release FixedMathSharp first, then validate Gravitas against that exact package.

Run only one heavy workload at a time with `DOTNET_PROCESSOR_COUNT=2`, two-core
affinity and BelowNormal priority. Review agents are read-only and must not launch
competing builds, tests or benchmarks. Use serialized MSBuild with
`-m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false`.

Focused commands, from each owning repository, after applying those limits:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~CapsuleCapsuleSlab' -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~CenteredCapsuleSlabContact' -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
```

Repeat focused checks in Lean, then use the existing solution, coverage and
benchmark workflows in each repository's `AGENTS.md` and benchmark README.
Retain logs and coverage summaries under ignored `artifacts/grv088` directories;
summarize exact commands, source revisions, test counts, coverage counts, resource
checks and benchmark comparisons in this document. Distinguish locally verified
platforms from pending Windows/Linux CI; do not infer cross-runtime validation.

Leave changes unstaged and uncommitted for user review. No tag, push or release.
Include concise commit suggestions for each changed repository at implementation
handoff. Planning approval authorizes the scoped work, not unrelated cleanup.

## Execution Evidence And Decisions

- Started from Gravitas `a15f630` (approved plan committed by the user) and
  FixedMathSharp `535578e`, on the existing `develop` checkouts. Preserve the
  user's in-place workflow and leave all implementation changes uncommitted.
- Use this canonical plan as the execution ledger rather than adding a second
  progress document or committing task checkpoints. The upstream query and
  downstream consumer share one signed normal/depth/clamp contract; review its
  representation during phase 0 before either production implementation starts.
- Fresh .NET 8 `Release` local-stack regression run: four intended failures,
  one existing positive-core test passes. The reported separated rim and the
  one-raw-separated neighbor return contact; exact touching and one-raw overlap
  report an extra 1/4 depth. Logs and TRX: `artifacts/grv088/red.log` and `red/`.
- The seven-case nonzero-core benchmark builds with zero warnings/errors.
  Its temporary baseline diagnostic records old admission/depth next to
  independently derived expected results outside measurement; the final
  implementation must replace it with strict corrected semantic preflight.
- Nonzero-core baseline means (microseconds/query, two launches, 15 measured
  250 ms iterations, five warmups, CPU mask 3): Side 40.64; Cap 34.99;
  StraightRim 51.27; SeparatedRim 51.09; EndRegion 40.78; Containment 30.92;
  ObliqueInteriorRim 62.86. All measured zero allocation. The incorrect rim
  answers are repair-cost baselines, not performance targets for equivalent
  behavior. Full distributions and diagnostics: `artifacts/grv088/baseline/`.
- Fresh cylinder/circle-slab control means, mixed / ordinary 3D respectively:
  Cap 21.20 / 21.23; EndpointGap 26.87 / 27.14; EndpointRim 110.79 / 111.90;
  ObliqueRim 1231.73 / 1232.21; Side 27.25 / 28.13; ZeroCore 26.63 / 26.90
  microseconds/query. All measured zero allocation; same runtime and settings.

### Reviewed Implementation Derivation

Use a slab-local exact rational frame, reusing `CylinderPolytopeFrame`.
Its relative rotation column `e` represents the capsule's authored +Y axis;
`|e| = D` exactly. Coordinates share raw scale `2D`. Preserve odd half-core
offsets, wide full height and relative centers without materializing endpoints.

An equivalent, simpler partition of the support function uses the planes
`n.Y=0`, `n.Z=0`, and `e dot n=0`. Enumerate full-support rational candidates
on their planar fans; extra valid directions cannot undercut the global minimum.
On an open sign cell, write `g(n)=R*|n.XZ|+p dot n`. Azimuthal stationarity
forces the direction opposite `p.XZ`; a polar minimum requires `|p.XZ|>R`.
The resulting outward disk-rim residual is a whole-shape closest-point
certificate only after strict cap, slab-end and capsule-end support admission.
Equality belongs to the explicit boundary planes. No positive-gap open-cell
minimum is missing.

On the capsule-interior plane, vertical `e` reduces to the horizontal fan.
Horizontal `e` projects the disk to a segment: include its generator
`R*(Up cross e)/D` in the rational residual fan. Projecting the cap/end
offset alone is incomplete. Genuinely oblique axes retain all admitted quartic
stationary roots in signed/swapped charts `N(t)=f+t*h`, `0<t<=1`.
Explicit chart axes own `t=0`; principal axes and projected linear residuals
own simultaneous `K=L=0`, zero-radius and collapsed cases.

Reuse the circular-rim parameter/value polynomial builders, with geometric
inputs rather than a triangle-only wrapper. Admission checks the original
unsquared equation, cap/end signs and nonzero radial term. Rank signed gaps
exactly; reverse squared-magnitude order for negative gaps. Add capsule radius
only in the final signed threshold comparison, before nearest-even rounding.
Never copy triangle negative-gap rejection or unrestricted-cylinder largest-root
selection. Maintain one raw/value scale across compared endpoint regions.

Independent correctness review confirms completeness with the horizontal-plane
correction. Parameter coefficient bounds are `M<267`, `P<334`, `Q<659`,
`K<603`, `L<929`, and `W/N/D<1900` bits, fitting 40 words. With value shift
at most 410, value invariant heights remain below 682 bits and quartic
coefficients below 2800 bits, fitting 56 words. Projected residual directions
can exceed 320 bits and must remain in wide spans.

The concrete planned peak budget reserves 48 KiB for simultaneously live
solver-owned buffers (enumerated layout 42,892 bytes), plus a live 64 KiB
caller. The largest root comparison uses a 319,136-byte arena and 11,296
bytes of copied cells: 445,120 bytes total before a 16 KiB control/JIT reserve.
The mapping path totals 402,445 bytes before that reserve. Both fit 512 KiB;
verify the actual call graph and 1 MiB worker test after implementation rather
than treating this layout calculation as measured stack usage.

The oblique raw-neighbor fixture must use exactly authored centers: scale the
  proposed non-dyadic center by five to `(24,25,37)`, with slab radius 25,
half-thickness 5, half-core 5, capsule radius `25+raw(k)`, and proportional
quaternion `(13,-4,-1,8)`. Its exact residual `(9,20,12)` has length 25.

### Implementation Checks

- Extracted `CircularRimContactAlgebra` from the existing triangle helper by
  changing concrete input signatures; both triangle consumers preserve their
  original arguments. No second polynomial/root engine or compatibility wrapper.
- The initial upstream 34-case focused suite passes, including positive,
  negative and zero core gaps, raw neighbors, odd-core rounding, exact maximum
  versus clamped depth, horizontal disk projection and an oblique winner.
- The actual oblique path passes a 1 MiB worker-stack test with a live dirty
  64 KiB caller and a warmed zero-allocation check. Independent source review
  bounds simultaneously live solver buffers by 37,908 bytes before structs and
  control state, within the reviewed 48 KiB outer allowance.
- Gravitas's 242-test `MixedNarrowPhase` suite passes after cutover. Superseded
  capsule-only direction traversal and its forwarding contact builder are
  removed; cylinder/cone helpers and canonical support contacts remain.
- Upstream unchanged triangle/slab control means: CapFace 62.39,
  StraightSideSeam 101.27, RoundedEndOddCore 238.22, ObliqueRimOverlap 1714.45,
  CertifiedRimGap 63.63, UnmaterializedScalarFace 104.48 microseconds/query,
  all zero allocation. Same settings as downstream; artifacts are under
  FixedMathSharp `artifacts/grv088/baseline/`.
- The first complete seven-case run passed strict semantic setup and measured
  Cap 95.43, Containment 101.72, EndRegion 93.64, ObliqueInteriorRim 1326.22,
  SeparatedRim 182.87, Side 111.90 and StraightRim 208.38 microseconds/query,
  all zero allocation. This exposed ordinary-path regressions rather than
  establishing acceptance. Frozen run artifacts: Gravitas
  `artifacts/grv088/first-complete/`.
- Two independently reviewed global certificates address that measured cost.
  For parallel cores the core is a Cartesian product. If `a` is the planar
  stadium gap and `h` the vertical interval gap, its minimum is `min(a,h)`
  when either is nonnegative; both-outside cases continue through the full
  solver. For a straight rim, the retained endpoint residual must have the
  radial/cap support signs, a Z coordinate inside the closed slab core, and
  the capsule support sign. Its feasible support point is then the whole
  core's closest point. Neither certificate treats failed proof as separation.
- Removed duplicate straight-side fan work across endpoint regions. Also,
  the concrete chart basis has exactly one nonzero coefficient per Y/Z
  component, so cap/end signs are constant throughout `(0,1]`. Whole-chart
  admission replaces four region-polynomial slots and per-root sign queries.
  Independent proof review confirmed that excluded `t=0` remains owned by
  the analytic fan. No approximation, cache or generic framework was added.
- Expanded focused upstream validation passes 55 tests, including an
  independently bounded irrational depth/normal, a double stationary root,
  two admitted roots in one chart, exact half-raw offset rounding, zero-gap
  chart ties and sign-crossing curved separation. At this checkpoint every
  curved-owner line and branch is exercised; final coverage still requires
  the complete Release/ReleaseLean matrices after all refinements.

### Refined Matched Performance

Measured on Windows 11 / i7-9700K, .NET 8.0.29, SDK 10.0.302, with CPU mask 3,
two launches, five warmups and fifteen 250 ms measured iterations. All values
below are **microseconds per dispatched query**; every row allocates **0 B**.
The error column is BenchmarkDotNet's half-width of the 99.9% confidence interval.

| Geometry | Old incomplete solver | First complete solver | Refined complete solver | Error |
| --- | ---: | ---: | ---: | ---: |
| Cap | 34.99 | 95.43 | 20.53 | 0.285 |
| Containment | 30.92 | 101.72 | 29.96 | 0.490 |
| End region | 40.78 | 93.64 | 25.27 | 0.363 |
| Oblique interior rim | 62.86 | 1326.22 | 942.95 | 20.152 |
| Separated rim | 51.09 | 182.87 | 55.97 | 1.108 |
| Side | 40.64 | 111.90 | 25.61 | 0.423 |
| Straight rim | 51.27 | 208.38 | 80.81 | 1.157 |

The ordinary cap/side/end rows improve approximately 37-41% versus the old path;
containment is approximately unchanged. The old rim answers were incomplete:
their times quantify the repair cost, not equivalent correct throughput. Exact
certificates and chart pruning cut the first complete straight-rim cost by 61%
and oblique-rim cost by 29%. The user accepted the remaining curved cost for now
on 2026-09-30; wrong fast answers are not an acceptable alternative.

Unchanged mixed circle-slab / ordinary 3D controls (respectively) are now:
Cap 20.40 / 21.77; EndpointGap 27.24 / 27.75; EndpointRim 112.85 / 110.90;
ObliqueRim 1244.73 / 1251.47; Side 27.57 / 27.83; ZeroCore 26.89 / 28.15.
Most changes versus the fresh baseline are within a few percent; the largest
is the unchanged 3D ZeroCore control at +4.6%. Its mixed counterpart is +1.0%.
These control changes are not evidence of a new shared-query regression.

Reproduce after a serialized Release local-stack benchmark build:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll capsule-slab-contact capsule-circle-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json --keepFiles --artifacts artifacts/grv088/refined
```

Strict benchmark setup rejects wrong classification, depth or clamping before
measurement. Frozen reports and per-iteration measurements are in
`artifacts/grv088/refined/`; source-mode binary/package fixtures are not release
artifacts. The final dispatch cleanup only removes a duplicate zero-slab-core
branch; these positive-slab-core and circle controls do not take that branch.

### Final Review And Validation

- The final 62-case upstream focused suite passes. Both new solver partials
  have 100% line/branch coverage. Resource tests include distinct authored
  rotations, near-maximum extents, a 1 MiB worker stack and live dirty 64 KiB
  caller storage. Repeated results and warmed allocations are checked.
- Independent correctness review found no production defect. Its missing
  integration check is now an initialized runtime-pair regression: a rotated
  positive-core slab retains its authored anchor frame, both participants
  receive contact-enter notification, the capsule moves upward, and the 2D
  participant's position and yaw remain unchanged.
- Ponytail review removed duplicate downstream zero-core dispatch and reused
  the upstream allocation helper. The first downstream allocation test exposed
  boxing in its own `ContactAnchor.Equals` comparisons; direct typed component
  comparisons remove that test-owned allocation. All eight focused downstream
  cases now pass, including repeated oblique zero-allocation contacts and
  compound-child material propagation. No runtime workaround was introduced.
- FixedMathSharp Release solution build passes with zero warnings/errors for
  both library TFMs. The complete core suite passes 3,994 tests and its separate
  Chronicler suite passes 49. Core/FluentAssertions coverage is exact:
  52,602/52,602 lines, 12,044/12,044 branches and 3,935/3,935 Cobertura methods.
  Logs and reports: FixedMathSharp `artifacts/grv088/final-release*`.
- Gravitas Release solution build passes with zero warnings/errors for both
  library TFMs. All 4,342 tests pass; coverage is exact at 44,360/44,360 lines,
  13,222/13,222 branches and 4,561/4,561 Cobertura methods. Logs and reports:
  Gravitas `artifacts/grv088/final-release*`. The remaining curved-case cost is
  recorded as GRV-Benchmark-021; the user accepted that trade-off on 2026-09-30.
- FixedMathSharp ReleaseLean solution build passes with zero warnings/errors
  for both TFMs. All 3,973 core tests and 49 Chronicler tests pass; coverage is
  52,695/52,695 lines, 12,044/12,044 branches and 3,931/3,931 Cobertura methods.
  Release/Lean use their existing exclusion settings unchanged. Logs and
  reports: FixedMathSharp `artifacts/grv088/final-lean*`.
- Gravitas ReleaseLean solution build passes with zero warnings/errors for
  both TFMs. All 4,283 tests pass; coverage is 44,358/44,358 lines,
  13,222/13,222 branches and 4,560/4,560 Cobertura methods. Logs and reports:
  Gravitas `artifacts/grv088/final-lean*`. Both repository/configuration matrices
  have no uncovered measured line, branch or method; no exclusions were added.
- Final unchanged upstream triangle/slab controls pass semantic setup and
  allocate zero: CapFace 60.23 +/- 1.057; StraightSideSeam 95.06 +/- 0.821;
  RoundedEndOddCore 217.39 +/- 5.471; ObliqueRimOverlap 1577.43 +/- 12.998;
  CertifiedRimGap 57.34 +/- 1.886; UnmaterializedScalarFace 98.68 +/- 1.332
  microseconds/query (mean +/- 99.9% confidence half-width). These show no
  observed regression from the shared extraction. They are controls, not a
  claim of a separately isolated optimization. BenchmarkDotNet flags the
  CertifiedRimGap distribution as bimodal; retain that uncertainty and the
  per-iteration data in FixedMathSharp `artifacts/grv088/final-controls/`.
  Reproduce with the same settings using
  `FixedMathSharp.Benchmarks.dll all --filter '*TriangleCapsuleSlabContactBenchmarks*'`.
- All 62 focused upstream tests also pass in Debug, including both dirty
  worker-stack cases. Debug assertions remain enabled; no proof checks were
  disabled to obtain the result. Log: FixedMathSharp `artifacts/grv088/final-debug.log`.
- All eight focused downstream cases also pass in Debug. Both DocFX sites
  build with `--warningsAsErrors`, zero warnings and zero errors. Logs:
  each repository's `artifacts/grv088/final-docfx.log`, plus Gravitas
  `artifacts/grv088/final-debug.log`.
- Local validation is Windows x64 / .NET 8 against the coordinated source
  stack. Linux CI and released-package consumption are later gates, not claims
  established by these local runs. No package version, coverage exclusion,
  production dependency, CI project or public API was added.

### Closeout Decision

On 2026-09-30 the user accepted the complete-query repair and its remaining
942.95 microsecond oblique and 80.81 microsecond straight-rim costs for now;
the old cheaper results were incorrect. With the recorded technical gates
complete, GRV-Issue-088 is resolved and this plan moves to `done`.

GRV-Benchmark-021 remains open alongside GRV-Benchmark-020. Their
[shared investigation context](../benchmark-signal-hardening-backlog.md#capsule-rim-cost-relationship-grv-benchmark-020-and-021)
distinguishes capsule-endpoint versus slab-edge naming, confirmed analytic
materializer reuse and related but distinct oblique root paths. Profile both
before claiming a shared CPU bottleneck; the accepted repair cost does not
establish optimal throughput or close either performance signal.

The user committed FixedMathSharp as `787afae` (`fix: compute complete
capsule-slab penetration`). Gravitas remains for user commit; existing staged
implementation changes are preserved and this documentation closeout is
unstaged. Suggested Gravitas commit: `fix: use complete mixed capsule-slab contacts`.
