# Complete Capsule/Stadium-Slab Cost Refinement

**Status:** Complete; GRV-Benchmark-021 closed 2026-10-05.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This records the 2026-10-04 investigation of GRV-Benchmark-021. The
[complete-contact repair](2026-09-29-complete-capsule-slab-contact-plan.md)
remains the correctness baseline. The cheaper, incomplete direction subset is
not a valid performance target.

## Shared Owner Changes

FixedMathSharp retains certified root refinement in `CompareRootSquared`
instead of discarding updated root metadata after an exact sign query. All five
consumers forward the root by reference: stadium, cylinder pair, oriented
box/cylinder, triangle/cylinder and triangle/cone. The existing sign/refinement
owner supplies the arithmetic.

The stadium winner retains its independently copied parameter and value roots.
Normal rounding consumes the admitted parameter directly, removing quartic
reconstruction and a second isolation of the winning parameter. A single
`FiniteAxisValueRoot.CopyTo` operation also replaces the cylinder-pair owner's
duplicate copy logic. Copies preserve the active polynomial spans, ordinal,
dyadic denominator, rational state and numerator, with independent backing
storage. No public API or additional friendship is introduced.

Positive analytic gaps provide an exact nonwinning-root certificate before
value mapping. If a positive gap rounds to an unclamped raw integer `d`, its
exact value is at most `d + 1/2` raw. A positive curved gap at or above this
threshold cannot improve the analytic candidate or a later, better winner.
The existing triangle/box squared-gap comparison supplies this test. Equality
retains the earlier feature; negative and zero gaps keep their existing exact
selection paths, and clamped analytic gaps supply no bound. `2*d + 1` is safe
through `d = long.MaxValue` and remains nonzero when `d = 0`.

Every signed and reciprocal chart remains present. Classification, minimum
depth, nearest-even rounding, conceptual clamping, stable ties, canonical
anchors and deterministic traversal are preserved. Gravitas runtime code is
unchanged; its benchmark now includes positive and irrational curved winners
and an analytic winner whose two admitted curved roots lose.

## Profile And Experiment Decisions

The initial stadium `WorkloadActual` profile attributes 83.0% inclusive sampled
CPU to curved-feature improvement, 28.1% to parameter-root acquisition, 26.9%
to root refinement, 18.8% to Sturm construction and 16.4% to final normal
materialization. These are overlapping inclusive samples, not additive costs.
Winner reconstruction alone includes another 3.7% in single-root isolation.

The circle-slab/3D cylinder controls follow a distinct ellipse owner: their
magnitude-to-raw comparison occupies about 53-55% and largest-positive-root
selection 34-36%. Its unrestricted-domain proof does not justify reflection or
largest-root selection inside stadium end-region constraints. GRV-Benchmark-020
therefore retains its own measurements and next step.

A certificate applied to both positive and negative gaps was rejected: the
original negative-gap fixture rose from 767.3 to 791.1 microseconds. The retained
version rounds an analytic magnitude only for positive gaps. Retained winner
parameters then reduced the original fixture to 740.7 microseconds in that
experiment. Expanded fixtures exposed a 3.4 ms analytic-winner case and justified
testing the positive-only certificate against more than one curved winner.

The early retained-root capture overlapped an unassigned test/build and was
discarded. A failed expanded capture selected unrelated benchmarks because of
PowerShell argument expansion and supplied no measurements. Another fixture
preflight rejected an unnormalized proportional quaternion; matching the
upstream authored normalization fixed the fixture before measurement. Those
failed or contaminated captures supply no performance claims.

## Matched Curved-Contact Measurements

Windows 11, Intel i7-9700K, .NET 8.0.29, SDK 10.0.302, Release, local stack,
`DOTNET_PROCESSOR_COUNT=2`, BelowNormal priority, affinity mask 3, one heavy
workload at a time. Both captures use two launches, five warmups, fifteen
measured iterations per launch and **512 invocations per iteration**. Error is
the half-width of the 99.9% confidence interval. These are complete mixed
contact-wrapper costs, not isolated polynomial-operation costs.

| Geometry | Baseline, microseconds | Refined, microseconds | Reduction | Allocation |
| --- | ---: | ---: | ---: | ---: |
| Original oblique interior rim | 812.93 ± 13.251 | 755.13 ± 13.560 | 7.1% | 0 B/op |
| Positive curved winner | 1163.09 ± 19.692 | 881.33 ± 14.390 | 24.2% | 0 B/op |
| Irrational curved winner | 1260.85 ± 19.224 | 981.46 ± 12.907 | 22.2% | 0 B/op |
| Analytic winner, nonwinning curved roots | 3358.56 ± 49.919 | 2898.86 ± 44.634 | 13.7% | 0 B/op |

The final fixed-invocation captures have no minimum-iteration-time warnings.
The baseline retains one multimodal-distribution warning; preserve its raw
measurements and intervals rather than treating these timings as universal host
budgets. Earlier adaptive captures included a 55 ms minimum interval after
JIT warmup, so the longer matched pair supplies the headline comparison.

Baseline sources are Gravitas `942d80f` and FixedMathSharp
`c07fd761cce6d3a85c4f96f352ef02fb3674e9bc`, with the expanded benchmark fixture
present on both sides. The refined source is the coordinated working diff.
Baseline measurement temporarily replaces only modified math production files
with their committed contents, then restores and verifies the exact working
bytes. No source is staged, reset or committed by this capture.

After building the benchmark project in Release source mode, reproduce both
sides with the corresponding source revision and this selection:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
(Get-Process -Id $PID).PriorityClass = 'BelowNormal'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:BuildInParallel=false -m:1
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleSlabContactBenchmarks*Oblique*' --launchCount 2 --warmupCount 5 --iterationCount 15 --invocationCount 512 --affinity 3 --exporters json --artifacts artifacts/grv-benchmark-021/reproduce
```

Raw distributions and generated projects remain under
`artifacts/grv-benchmark-021/fixed-baseline/` and `fixed-current/`; earlier
experiments remain in their separately named directories.

## Parity And Ordinary Contact Controls

Controls use two launches, five warmups, fifteen measured iterations and a
250 ms target interval, with the same environment and source pair as above.
Values are microseconds per query with 99.9% confidence half-widths; all rows
report 0 B/op. The adaptive control captures retain multimodal warnings.
Their small timing differences do not establish a broad cross-stack speedup.

Ordinary stadium:

| Fixture / path | Baseline | Refined |
| --- | ---: | ---: |
| Cap / Contact | 17.23 ± 0.218 | 16.94 ± 0.204 |
| Containment / Contact | 24.95 ± 0.383 | 25.21 ± 0.404 |
| EndRegion / Contact | 21.33 ± 0.238 | 20.83 ± 0.252 |
| SeparatedRim / Contact | 48.97 ± 0.733 | 48.72 ± 0.554 |
| Side / Contact | 21.29 ± 0.315 | 20.75 ± 0.239 |
| StraightRim / Contact | 74.56 ± 1.152 | 74.22 ± 0.903 |

Mixed circle-slab and 3D cylinder:

| Fixture / path | Baseline | Refined |
| --- | ---: | ---: |
| Cap / Contact | 18.72 ± 0.437 | 18.52 ± 0.233 |
| Cap / CylinderContact | 18.08 ± 0.213 | 18.43 ± 0.234 |
| EndpointGap / Contact | 23.50 ± 0.289 | 23.47 ± 0.324 |
| EndpointGap / CylinderContact | 23.92 ± 0.335 | 23.97 ± 0.504 |
| EndpointRim / Contact | 87.85 ± 1.350 | 87.06 ± 1.209 |
| EndpointRim / CylinderContact | 88.07 ± 1.460 | 88.25 ± 1.723 |
| ObliqueRim / Contact | 1180.41 ± 19.868 | 1190.35 ± 18.786 |
| ObliqueRim / CylinderContact | 1136.61 ± 19.448 | 1140.77 ± 22.314 |
| Side / Contact | 23.95 ± 0.408 | 23.47 ± 0.316 |
| Side / CylinderContact | 24.45 ± 0.333 | 23.85 ± 0.314 |
| ZeroCore / Contact | 23.35 ± 0.426 | 23.29 ± 0.280 |
| ZeroCore / CylinderContact | 24.44 ± 0.402 | 23.54 ± 0.303 |

Upstream triangle/stadium:

| Fixture / path | Baseline | Refined |
| --- | ---: | ---: |
| CapFace | 48.12 ± 0.705 | 48.47 ± 0.652 |
| StraightSideSeam | 75.80 ± 1.504 | 75.76 ± 1.196 |
| RoundedEndOddCore | 172.21 ± 2.273 | 173.30 ± 2.908 |
| ObliqueRimOverlap | 869.73 ± 13.608 | 845.02 ± 12.326 |
| CertifiedRimGap | 51.09 ± 0.693 | 50.68 ± 0.653 |
| UnmaterializedScalarFace | 77.50 ± 1.899 | 76.85 ± 0.976 |

The repeated adaptive oblique stadium results (749.13, 879.55, 984.26 and
2907.87 microseconds for original, positive, irrational and analytic-winner
cases) agree with the longer matched captures. The upstream oblique triangle
row improves by 2.8%; the other triangle rows remain near their prior costs.
Raw control distributions remain under `baseline/`, `baseline-controls/`,
`final-controls/`, `triangle-baseline/` and `triangle-final/` in the same artifact
root. Mixed and 3D paths are shown separately because their wrappers differ.

## Workspace And Correctness Gates

The retained parameter buffers add 6,693 bytes. The stadium outer-buffer budget
is now 44,601 bytes, within its existing 48 KiB reserve. Using stadium-specific
coefficient bounds, the conservative live value-comparison and mapped-parameter
sign-query budgets are 462,224 and 418,669 bytes respectively, including the
64 KiB live caller and 16 KiB control/spill reserve. These source-derived bounds
remain below 512 KiB; the separate 1 MiB dirty-caller worker test supplies
runtime evidence. The broader shared field-capacity arena is not the stadium
budget.

Tests cover retained rational discovery, irrational refinement without identity
changes, independent root copies after source-buffer reuse, raw-neighbor
classification, half-raw ties, zero/positive/negative gaps, repeated roots,
conceptual overflow and strict warmed allocation.

The first full Release capture passed its tests but exposed two missing branch
outcomes after the certificate bypassed older nonwinning-root fixtures: chart
value reuse and retained-winner comparison. Geometry-backed tests restore
those paths without weakening the certificate. One places an exact negative
winner at both reciprocal-chart endpoints. The other uses raw dimensions
`R=18`, `H=4`, slab core `2`, capsule core `200`, center `(12,0,10)` and
proportional quaternion `(9,-2,-2,6)`. Its exact axis is `(-12,-45,116)/125`;
the winning normal is `(12,20,9)/25`, with depth five raw units. A later root
in the same chart has gap about 5.299; both pass the 5.5-raw analytic bound
and round to five, requiring exact ranking to preserve the first normal.

For an independent whole-minimum proof, undo its rational yaw to obtain cylinder
`R=18`, `H=4`, center `(0,0,15)` and capsule axis `(-20,-9,12)/25`. In the
potentially winning perpendicular-plane sector, let `s=nz/ny`,
`Q=81-216s+544s²`, `M=Q+400` and `P=80-300s`. The gap is
`(P+18*sqrt(Q))/sqrt(M)`. For `P>=0`, `299Q>10000` proves it exceeds five.
For `P<0`, the two-squared comparison has positive intermediate
`B=7819-16584s+72656s²` and factors exactly as
`B²-100P²M=(4s-3)²(-27411471+169980456s+23930896s²)`. The quadratic is
positive for `s>=4/15`; equality occurs only at `s=3/4`. Other normal sectors
have strict bounds above five. Capsule half-length 100 dominates off-plane
support variation `sqrt(340)+15<34`; adding the stadium core and shifting
the center by its positive endpoint adds `|nz|-nz>=0`, preserving equality
at the stated normal. This proof uses the authored geometry rather than the
solver under test.

Final fresh captures completed 2026-10-05 with `UseLocalLsfStack=true`.
Owning suites retain exact 100% reachable line, branch and method coverage in
both configurations, checked independently in ReportGenerator and OpenCover.
No exclusions were changed. FixedMathSharp core and FluentAssertions are
validated together; Chronicler is validated through its own module rather
than merging its partial core-dependency capture over the complete core suite.

| Owning suite | Release tests | Lean tests |
| --- | ---: | ---: |
| FixedMathSharp core / FluentAssertions | 4150 | 4129 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4537 | 4476 |

| Coverage owner / format | Lines or sequence points | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp-Release / ReportGenerator | 52675/52675 | 12152/12152 | 3936/3936 |
| FixedMathSharp-Release / OpenCover | 52675/52675 | 12152/12152 | 3936/3936 |
| FixedMathSharp.Chronicler-Release / ReportGenerator | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler-Release / OpenCover | 85/85 | 12/12 | 18/18 |
| FixedMathSharp-ReleaseLean / ReportGenerator | 52768/52768 | 12152/12152 | 3932/3932 |
| FixedMathSharp-ReleaseLean / OpenCover | 52768/52768 | 12152/12152 | 3932/3932 |
| FixedMathSharp.Chronicler-ReleaseLean / ReportGenerator | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler-ReleaseLean / OpenCover | 85/85 | 12/12 | 18/18 |
| Gravitas-Release / ReportGenerator | 56272/56272 | 16302/16302 | 5413/5413 |
| Gravitas-Release / OpenCover | 44542/44542 | 13296/13296 | 4603/4603 |
| Gravitas-ReleaseLean / ReportGenerator | 56270/56270 | 16302/16302 | 5412/5412 |
| Gravitas-ReleaseLean / OpenCover | 44540/44540 | 13296/13296 | 4602/4602 |

Gravitas rendered and raw totals use different inherited-method mappings;
each independently has exact covered/total equality. Debug focused suites
passed 521 upstream and 28 downstream cases, including dirty-stack and warmed
allocation checks. Both solution configurations built all declared targets
with zero warnings/errors. Both DocFX builds passed with warnings as errors,
and both API sites passed local-link and existing branding/resource checks.
Independent code and workspace reviews found no actionable correctness issue.

Capture root:
`artifacts/grv-benchmark-021/final-gates-20261005T042039547Z-a3984dee13964f9aae8fbe38ee6e2c00/`.
It retains raw XML, generated reports, test/build logs, source revisions and
`coverage-summary.json` / `result.json`. Earlier failed captures are separate.
Release FixedMathSharp first, then validate Gravitas against that released
package before releasing Gravitas; this evidence uses the unreleased local stack.

## Status At The End Of This Phase

At the end of this phase, GRV-Benchmark-021 remained open. The analytic-winner case still costs about
2.90 ms per contact despite its improvement, so this phase does not establish
a real-time contact budget for large workloads.

Final `WorkloadActual` EventPipe profiles show:

| Fixture | Remaining inclusive sampled CPU |
| --- | --- |
| Analytic winner | Curved chart traversal 92.4%; sign refinement 55.3%; positive rejection comparison 39.6%; parameter-root acquisition 29.9%. |
| Original negative-gap winner | Parameter-root acquisition 28.0%; normal materialization 20.8%; Sturm construction 17.8%. |
| Irrational winner | Parameter-root acquisition 34.0%; value mapping 16.7%; Sturm construction 15.1%. |
| Positive winner | Parameter-root acquisition 40.9%; Sturm construction 23.0%; value mapping 14.1%. |

Percentages overlap and must not be summed. In the analytic-winner trace,
wide magnitude multiplication is the largest exclusive leaf at 28.5%; quartic
polynomial evaluation is 15.6%. The rejection certificate avoids downstream
mapping but still pays for an exact sign at an algebraic parameter.

The next experiment should first reduce or reuse exact threshold-sign
refinement, then examine parameter-root isolation/count reuse where chart
equivalence can be proved. Keep all four oblique fixtures, ordinary contacts,
circle/3D and affected triangle controls. A cheaper rejection operation must
preserve equality, positive/negative separation, full-width intermediates and
bounded scratch; removing constrained charts requires a completeness proof.

Profiler commands use one launch, three warmups, eight 500 ms iterations,
affinity 3 and `--profiler EP` over the same four oblique fixtures. Raw traces
remain in `artifacts/grv-benchmark-021/final-profile/`; summaries use the existing
`artifacts/grv-benchmark-022/profile-summary.ps1 -Activity WorkloadActual` helper.
It counts recursive inclusive frames once and attributes exclusive samples to
the real frame above EventPipe's synthetic `CPU_TIME` leaf. Profiled timings
are not substituted for the unprofiled benchmark results.

Subsequent status on 2026-10-05: the
[root-isolation refinement](2026-10-05-capsule-slab-root-isolation-refinement.md)
closes GRV-Benchmark-021 with further verified gains and published remaining
cost. The measurements and open-status assessment above describe this earlier phase.
