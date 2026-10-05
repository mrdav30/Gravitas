# Capsule/Stadium-Slab Certified Sign Refinement

This continues [GRV-Benchmark-021](benchmark-signal-hardening-backlog.md#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost)
after the [retained-root phase](2026-10-04-capsule-slab-cost-refinement.md).
The committed baseline is Gravitas `b38373e` and FixedMathSharp `e3bdea3`.
All development, child benchmark builds and validation use
`UseLocalLsfStack=true`; no package references are edited.

## Changes And Exactness

The existing value-root sign owner first tries its certified Horner evaluator
on the retained cell. With guarded variable shift `k`, query degree `m` and
`c=ceilLog2(m+1)`, the trial precision is
`min(32, DenominatorShift-k-2*c)`. Only positive precision is attempted.
The normalized cell therefore satisfies `effectiveShift>=precision+2*c`.
Coefficient/Horner truncation contributes less than `2*m+1` units and cell
variation less than half a unit. A returned magnitude above `2*(m+1)` proves
the exact sign. An uncertain zero reaches the unchanged resultant/equality
bound. A decisive trial neither mutates nor copies the retained cell.

Crossing refinement now uses the same endpoint evaluations to classify an
odd crossing and obtain byte-step prediction hints. Nonzero Horner signs
are certified point signs; uncertain endpoints use full integer evaluation.
Only opposite, nonzero endpoint signs admit crossing refinement. Equal signs
or a zero parent endpoint retain the original cell for Sturm refinement.
An excluded endpoint can be another polynomial root and must never become
the selected rational singleton. Temporary upper-numerator changes are
restored before this fallback. Existing known-crossing callers retain their
short-step and prediction rules.

Point-certificate precision has a measured guard budget shared by endpoint,
byte-step and bisection evaluation. Increasing this work budget does not
change the sign acceptance threshold, target root precision, root identity,
rounding or exact fallback. Full polynomial evaluation remains the authority
whenever a point certificate is uncertain. No root fields, caches, solver,
public API or additional internal friendship are introduced.

## Experiments

Matched four-fixture captures use two launches, five warmups, fifteen measured
iterations per launch and 512 invocations per iteration. The host is Windows
11 / i7-9700K, .NET 8.0.29, SDK 10.0.302, Release, affinity mask 3,
`DOTNET_PROCESSOR_COUNT=2`, BelowNormal priority and one heavy workload at a
time. Values are complete mixed contact costs in microseconds. Errors are
99.9% confidence half-widths; profiled costs are not substituted for them.

| Experiment | Analytic winner | Original rim | Irrational rim | Positive rim |
| --- | ---: | ---: | ---: | ---: |
| Fresh committed baseline | 2853.07 ± 16.519 | 737.83 ± 1.347 | 969.48 ± 8.397 | 890.84 ± 7.845 |
| Retained-cell trial alone | 2922.42 ± 11.118 | 706.28 ± 2.481 | 894.15 ± 1.208 | 849.67 ± 6.006 |
| Trial plus shared endpoint/hints | 2816.14 ± 18.184 | 714.51 ± 5.921 | 883.67 ± 7.358 | 837.45 ± 7.739 |
| Local crossing-sign return experiment | 2830.72 ± 29.333 | 722.17 ± 7.689 | 895.02 ± 6.440 | 839.76 ± 7.367 |
| 128-bit point guard | 1368.80 ± 3.209 | 716.97 ± 5.476 | 905.08 ± 7.471 | 858.49 ± 12.159 |

Every completed row reports 0 B/op. The trial alone worsened the target
analytic-winner case by 2.4%, so it was not sufficient on its own. Reusing a
crossing sign locally across precision passes supplied no measured benefit
and was removed, including its return-contract changes and experiment tests.
The direct bound-polynomial specialization was not implemented: its removed
multiplications by one do not account for the expensive sign/refinement
children of the bound comparison.

The shared endpoint/hint profile, before increasing the point budget, shows
60.999% inclusive CPU in crossing refinement. Full polynomial evaluation
accounts for 38.948% inclusive / 15.041% exclusive, and all of its sampled
CPU is directly beneath the crossing refiner. Approximate evaluation beneath
that refiner is 13.918% inclusive. This supports increasing certificate work
precision; sampled stacks alone do not count failed byte predictions or
distinguish them from other uncertified point evaluations.

Raw captures are retained under `artifacts/grv-benchmark-021/phase2-*`,
including the rejected local-sign experiment and pre-budget EventPipe profile.
The point-guard tuning launcher restores original source bytes in a
`finally` block. One-launch / five-warmup / fifteen-iteration / 512-invocation
tuning of the analytic-winner case measured 128 / 192 / 256-bit guards at
1409.13 ± 2.825 / 1452.46 ± 3.975 / 1471.15 ± 4.159 microseconds respectively.
The 128-bit guard is retained. Larger guards cost more without another gain in
this fixture; this is a measured choice rather than a universal optimum.

## Final Matched Confirmation

The selected 128-bit guard, retained-cell trial and shared endpoint/hint path
were rebuilt and confirmed with the same longer four-fixture configuration.
After all validation gates, the committed baseline was also rebuilt and run
again with exactly those settings. All four changed production files were
restored byte-for-byte in `finally`, hash-checked against the measured source,
and the refined benchmark project rebuilt with zero warnings/errors. The
complete intervals and distributions are in `phase2-baseline/`,
`phase2-baseline-confirm/` and `phase2-confirm/`, rather than the shorter tuning
runs. Table values are microseconds; reduction ranges compare the refined
mean with both baseline means, not confidence limits.

| Geometry | Initial baseline | Repeat baseline | Refined | Mean reduction range |
| --- | ---: | ---: | ---: | ---: |
| Analytic winner, nonwinning curved roots | 2853.07 ± 16.519 | 2776.21 ± 31.695 | 1390.83 ± 18.954 | 49.9–51.3% |
| Original oblique interior rim | 737.83 ± 1.347 | 700.89 ± 5.144 | 697.57 ± 8.695 | 0.5–5.5% |
| Irrational curved winner | 969.48 ± 8.397 | 924.06 ± 7.789 | 866.84 ± 7.594 | 6.2–10.6% |
| Positive curved winner | 890.84 ± 7.845 | 847.12 ± 10.286 | 829.38 ± 10.057 | 2.1–6.9% |

All four final and repeated-baseline rows have non-null statistics and zero
measured allocation. The repeated baseline is 2.7–5.0% lower than the initial
baseline, confirming host timing drift. The slow analytic-winner and irrational
winner intervals remain separated from both baselines. Original and positive
rim intervals overlap the repeated baseline, so this phase does not claim a
firm improvement for those smaller changes.
The initial 128-bit capture independently measured the slow fixture at
1368.80 microseconds. The final confirmation retains one detected outlier;
raw measurements and BenchmarkDotNet's filtering remain available. These
host measurements do not establish a universal frame or contact budget.

## Workspace And Regression Evidence

The point guard adds 64 precision bits to the old point-evaluation work budget.
Each active approximate result/product pair grows by one word each, or
16 bytes total. Returning call frames do not accumulate that increment.
Conservatively adding it to the prior comparison / mapped-sign budgets gives
462,240 / 418,685 bytes, including caller/control reserves, below 512 KiB.
No root storage, target shift or Sturm arena grows. The dirty-caller / 1 MiB
worker tests supplement these source-derived bounds with runtime evidence.

New tests require decisive retained cells to answer separated signs without
extra refinement, including 4096-bit common content, finer cells and tiny-root
variable normalization. Independent bracket checks retain the exact selected
root. Cubic fixtures `(2t-1)(4t²-3)` and `(t-1)(4t²-3)` place another root at
either excluded endpoint of `(1/2,1)`. Refinement must preserve the interior
root `sqrt(3)/2` and its ordinal, verified by
`4*N² < 3*2^(2*shift) < 4*(N+1)²`, rather than selecting an endpoint singleton.
Existing cases cover equality, dyadic discovery, odd/even multiplicities,
ill-conditioned roots, dirty buffers, small retained capacity and stable ties.
A new independent rational bracket also covers `alpha=1-2^-256`: exact
evaluation of the uncertain upper endpoint must restore a cell whose low word
carried to zero. The final 128-bit cell must still contain that same root.

Final controls use two launches, five warmups and fifteen 250 ms measured
iterations, with the same host/local-stack restrictions. All 28 rows have
non-null statistics and report 0 B/op. The four adaptive oblique rows are
separate confirmation controls, not replacements for the matched fixed-count
comparison above. Remaining ordinary/downstream controls are:

| Geometry | Stadium mixed, microseconds | Circle mixed, microseconds | Circle equivalent 3D, microseconds |
| --- | ---: | ---: | ---: |
| Cap | 15.86 ± 0.091 | 17.06 ± 0.229 | 17.04 ± 0.178 |
| Side | 19.34 ± 0.094 | 22.04 ± 0.261 | 22.23 ± 0.244 |
| Containment | 23.40 ± 0.234 | — | — |
| End region | 19.54 ± 0.214 | — | — |
| Straight / endpoint rim | 68.74 ± 0.298 | 82.02 ± 1.107 | 82.15 ± 0.348 |
| Separated rim | 45.08 ± 0.216 | 21.70 ± 0.064 | 22.30 ± 0.211 |
| Zero capsule core | — | 21.81 ± 0.175 | 21.78 ± 0.201 |
| Oblique interior rim | See matched table | 1102.67 ± 12.655 | 1046.62 ± 2.215 |

Straight stadium and circular endpoint rims are different geometries, not a
cross-solver comparison. The paired mixed/3D circle rows measure complete
wrapper costs and preserve parity coverage. These controls are lower than
the earlier phase's captures, including circle paths untouched by this change;
do not attribute that broad reduction to this refinement.

| Triangle/stadium control | Microseconds |
| --- | ---: |
| Cap face | 46.66 ± 0.205 |
| Straight side seam | 70.15 ± 1.532 |
| Rounded end, odd core | 159.46 ± 1.406 |
| Oblique rim overlap | 765.49 ± 2.115 |
| Certified rim gap | 47.26 ± 0.191 |
| Unmaterialized scalar face | 74.15 ± 0.584 |

Raw controls remain in `phase2-final-controls/` and `phase2-triangle-final/`.
Independent production, workspace and final regression reviews found no
actionable correctness issue. The final production files match the exact
source snapshot used for the confirmation and controls; subsequent edits
only add the carry regression and documentation.

Fresh serial owning-suite validation completed with `UseLocalLsfStack=true`.
Both configurations retain exact 100% reachable line, branch and method
coverage, independently checked in ReportGenerator and raw OpenCover. No
exclusions changed. Chronicler's owning module is checked independently of
its partially exercised core dependency.

| Owning suite | Release tests | Lean tests |
| --- | ---: | ---: |
| FixedMathSharp core / FluentAssertions | 4157 | 4136 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4537 | 4476 |

| Coverage owner / format | Lines or sequence points | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp-Release / ReportGenerator | 52684/52684 | 12162/12162 | 3936/3936 |
| FixedMathSharp-Release / OpenCover | 52684/52684 | 12162/12162 | 3936/3936 |
| FixedMathSharp.Chronicler-Release / ReportGenerator | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler-Release / OpenCover | 85/85 | 12/12 | 18/18 |
| FixedMathSharp-ReleaseLean / ReportGenerator | 52777/52777 | 12162/12162 | 3932/3932 |
| FixedMathSharp-ReleaseLean / OpenCover | 52777/52777 | 12162/12162 | 3932/3932 |
| FixedMathSharp.Chronicler-ReleaseLean / ReportGenerator | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler-ReleaseLean / OpenCover | 85/85 | 12/12 | 18/18 |
| Gravitas-Release / ReportGenerator | 56272/56272 | 16302/16302 | 5413/5413 |
| Gravitas-Release / OpenCover | 44542/44542 | 13296/13296 | 4603/4603 |
| Gravitas-ReleaseLean / ReportGenerator | 56270/56270 | 16302/16302 | 5412/5412 |
| Gravitas-ReleaseLean / OpenCover | 44540/44540 | 13296/13296 | 4602/4602 |

Gravitas rendered and raw totals use different inherited-method mappings;
each independently has exact covered/total equality. Debug focused suites
passed 528 upstream and 28 downstream cases, including dirty-stack/1 MiB and
warmed-allocation checks. Both solution configurations built all declared
targets with zero warnings/errors. The shared refinement/classifier/sign-query
OpenCover complexities are 16/46/38 in both configurations; the upstream
complexity register and workspace proof are updated accordingly.
Both DocFX builds passed with warnings as errors, and both API sites passed
the existing local-link, branding and resource checks. Release FixedMathSharp
first, then validate Gravitas against that released package before releasing
Gravitas; these development gates select the unreleased local stack.

Capture root:
`artifacts/grv-benchmark-021/final-gates-20261005T154851161Z-61242ac51b9c43faa6fe3d5bc733936e/`.
It retains coverage XML, generated reports, test/build logs, source revisions
and `coverage-summary.json` / `result.json`. Earlier failed captures remain
separate. The first full Release run passed its tests but exposed the missing
upper-endpoint borrow outcome; the independent rational bracket above restores that coverage
without changing production or weakening the certificate.

## Remaining Signal

GRV-Benchmark-021 remains open. The slow fixture is substantially cheaper,
but about 1.39 ms per complete contact still warrants work before claiming a
large-workload budget. GRV-Benchmark-020 retains its distinct ellipse owner.

Final `WorkloadActual` EventPipe profiles use one launch, three warmups, eight
500 ms iterations and affinity 3. The accepted source now gives the following
inclusive sampled shares; they overlap and must not be summed:

| Fixture | Root acquisition | Sign query | Root refinement | Other selected work |
| --- | ---: | ---: | ---: | --- |
| Analytic winner | 55.6% | 23.1% | 17.8% | Rejection comparison 10.4%; Sturm construction 11.6%. |
| Original rim | 34.0% | 34.3% | 16.3% | Value mapping 13.7%; Sturm construction 19.6%. |
| Irrational rim | 33.7% | 35.2% | 19.4% | Value mapping 15.0%; Sturm construction 19.3%. |
| Positive rim | 33.2% | 35.3% | 22.0% | Value mapping 13.1%; Sturm construction 17.5%. |

Root acquisition uses the complete `TryGetFiniteValueRoots` owner, including
parameter and mapped-value calls where present. In the slow fixture, sign-query
share falls from the pre-budget profile's 64.2% to 23.1%, and bound-comparison
share from 38.1% to 10.4%. These percentages describe sampled call paths rather
than operation counts or an additive budget. Raw traces and summaries remain
in `phase2-final-profile/` and `phase2-final-profile.json`.

The next investigation should isolate root acquisition/count/refinement,
including whether paired reciprocal charts can reuse exact isolation work.
Any reuse needs a proof for the transformed polynomial, endpoint inclusion,
ordinal mapping, repeated roots and constrained chart admission. Retain all
charts and the same four oblique fixtures, ordinary contacts, mixed/3D circle
controls and triangle controls; do not import the unrestricted ellipse
largest-root shortcut. No speculative root cache or second solver is added.
