# Capsule/Circle-Slab Output Refinement

This records the first GRV-Benchmark-020 refinement after closing
GRV-Benchmark-021. The starting committed revisions are Gravitas `989c7f9`
and FixedMathSharp `bd8f6a2`. The complete shared cylinder/capsule query is
the correctness baseline; the former sampled mixed direction subset is not
a valid performance target.

## Retained Design

The retained positive ellipse root already encloses the winning exact value.
Its existing signed interval Horner evaluator now also supplies bounds for
`N(alpha)/D(alpha)`. Both polynomials keep the same padded degree and therefore
the same homogeneous denominator scale. A strictly positive denominator
minimum permits independent lower and upper ratio bounds. An uncertain
denominator returns the original complete search range. A nonpositive
numerator minimum is bounded by zero under the caller's nonnegative-value
proof; it is never treated as a negative square-root input.

The existing wide integer division and square-root owners yield bounds on
the **floor** of the magnitude. Division uses full-width quotient, remainder
and scratch buffers. Comparison against `cap^2*D` precedes packing the quotient
into two words; the unclipped quotient is strictly below `2^128`. Constants
do not copy root coordinates, including after deep repeated-root refinement.
Rational roots use a zero-width interval.

The depth caller still performs its exact conceptual-clamping comparison
first. For a negative gap, contact admission proves magnitude at most the
capsule radius. A magnitude floor upper bound `u` gives
`floor(radius-magnitude) >= max(0,radius-u-1)`: the extra raw unit accounts
for the unknown fractional part. For a positive winning gap, the admitted
major-axis candidate bounds the magnitude by the cylinder radius. Normal
components retain their unit-magnitude bound. The original exact binary
threshold and final half-raw nearest-even comparisons decide every result.

Depth materialization then reduces its numerator and denominator once at
the retained root. Common low powers of the strictly positive parameter
cancel together. Each positive pseudo-step multiplies **both** polynomial
values by `abs(leading(P))`, even when one leading coefficient is zero.
Joint degree trimming and shared power-of-two normalization preserve their
relative scale. The previous single-query reducer reuses the same arithmetic
step; there is no second division or root-sign solver.

The original structured ratio supplies the initial interval bounds before
reduction: reduced coefficients can have a weaker enclosure. For nonzero
ellipse `Q`, the defining root is quartic and one step leaves degree at most
three. When `Q=0`, the root owner removes stationary zero factors and retains
a quadratic. Canceling the common numerator/denominator `t^2` first leaves
one shared step and a constant ratio. Normal-component queries already have
degree at most two and keep their original representation.

This is reusable deterministic arithmetic in FixedMathSharp. Gravitas retains
contact, manifold, anchor, material and mixed response policy. Both mixed
circle slabs and 3D cylinders consume the same improvement. No new public API,
friendship, configuration, approximate contact decision or retained cache is
introduced.

## Resources And Correctness

For interval width `W`, the four live bounds and returning division frame need
at most `9W+7` words. Under the existing ellipse bounds, `W<=1071`, hence
**77,168 bytes** of explicit scratch. The non-inlined bounds owner returns
before exact sign or reduced-pair work begins.

Paired pseudo-division increases coefficient height by at most `Bp+1` bits
per common step. The production ellipse uses one step after common-factor
cancellation, so coefficients stay below 4,309 bits; the sizing rule reserves
69 words. The retained pair adds at most **5,530 bytes**, including signs.
Depth threshold slots become 72 words; normal threshold slots remain 43.
The shared pseudo-step's three product buffers return before the next step
or sign query. The existing complete ellipse bound remains below 448 KiB
of explicit stack scratch; the unchanged analytic-winner comparison remains
larger than the new materialization path. This is a source-derived buffer
bound, not a total JIT stack measurement. Existing 1 MiB dirty-stack tests
with 64 KiB of live caller storage supplement the width and lifetime proof.

Independent tests exercise rational and irrational cells, negative denominator
shifts, loose or uncertain interval bounds, a zero numerator, unsigned caps,
wide division, constant ratios after deep repeated-root refinement, both
defining-polynomial leading signs, a zero leading numerator, common root
powers, unequal degree drops, repeated roots, differing coefficient widths
and dirty output tails. Paired quotient identity is checked independently in
the quadratic field rather than by comparing two production routes. Existing
complete-contact tests retain classification, canonical ordering, reflected
and permuted normals, raw-neighbor and half-raw ties, conceptual clamping,
full-domain rigid frames and strict allocation guards.

## Measurements And Validation

Captures use `UseLocalLsfStack=true` in MSBuild and the environment, Windows 11,
an i7-9700K, .NET 8.0.29 and SDK 10.0.302. One BelowNormal heavy job runs at
a time, with `DOTNET_PROCESSOR_COUNT=2` and serial builds. The requested CLI
option is `--affinity 3`; job metadata reports `Affinity=11` in both matched
captures. These are recorded separately without inferring a different mask.
Each row uses two launches, five warmups and fifteen measured 250 ms adaptive
iterations per launch. Values are microseconds per dispatched query; errors
are 99.9% confidence half-widths. Setup asserts complete classification,
known quarter-unit depths and mixed/3D depth agreement.

The initial committed oblique baseline is **1,158.03 ± 17.39 / 1,128.60 ±
18.11 µs** for mixed / 3D. Bounds alone reduce it to **976.53 ± 15.30 /
967.80 ± 17.62 µs**. The shared-ratio smoke is **763.0 ± 13.11 / 737.4 ±
17.80 µs**. Its 3D distribution warning is retained in the raw log; smoke
is not substituted for the complete matched confirmation. All measured rows
report **0 B/op**. Initial twelve-row captures, stage backups, source hashes
and logs are under `artifacts/grv-benchmark-020`.

The `retained-confirm` and `retained-rebuilt-confirm` captures and their
profiles are **excluded**. Restoring the candidate also restored older file
timestamps. The generated child used a different intermediate directory and
reused its baseline upstream DLL even after the launcher was rebuilt.
Reflection confirmed that the timed child lacked both new helpers; checking
only the launcher was insufficient. The corrected `retained-verified-confirm`
refreshes restored timestamps, uses `-t:Rebuild`, verifies source hashes and
launcher DLL equality, and retains the generated child with `--keepFiles`.
Fresh-process checks verify helper presence in the **actual timed and profiled
child DLLs**, whose SHA-256 hashes and MVIDs are retained in
`retained-verified-child-assembly.json` and
`retained-verified-profile-child-assembly.json`. Use only
`retained-verified-profile` for the candidate profile.

| Geometry | Committed mixed | Retained mixed | Committed 3D | Retained 3D |
| --- | ---: | ---: | ---: | ---: |
| Cap | 17.49 ± 0.214 | 17.63 ± 0.270 | 17.96 ± 0.245 | 18.51 ± 0.259 |
| Endpoint gap | 23.61 ± 0.307 | 23.66 ± 0.283 | 23.98 ± 0.312 | 23.69 ± 0.357 |
| Endpoint rim | 86.61 ± 1.195 | 86.39 ± 1.174 | 87.01 ± 1.583 | 87.18 ± 1.296 |
| Oblique interior rim | 1,186.47 ± 22.205 | **745.01 ± 12.541** | 1,124.91 ± 17.824 | **724.62 ± 15.125** |
| Side | 23.54 ± 0.325 | 23.98 ± 0.385 | 24.07 ± 0.312 | 24.49 ± 0.539 |
| Zero capsule core | 23.45 ± 0.298 | 23.46 ± 0.294 | 23.55 ± 0.315 | 23.96 ± 0.275 |

These are `baseline-confirm` versus `retained-verified-confirm`. The oblique
means improve **37.2% mixed / 35.6% 3D** against the fresh committed control,
and **35.7% / 35.8%** against the initial committed capture. This establishes
the curved improvement across repeated controls, not an ordinary-path speedup.

The full matrix's ordinary means shift between approximately -1.2% and +3.1%.
The 3D cap row's +3.1% prompted a targeted back-to-back repeat, with the same
two-launch settings and actual-child assembly checks:

| Cap repeat | Committed | Retained |
| --- | ---: | ---: |
| Mixed | 17.50 ± 0.222 | 18.09 ± 0.235 |
| 3D | 18.19 ± 0.270 | 18.36 ± 0.348 |

The repeated 3D difference is +0.9%, with overlapping confidence intervals;
the mixed repeat moves +3.4%, while its full-matrix difference was +0.8%.
Those small ordinary shifts are not fully isolated. Publish them rather than
claiming zero regression or an improvement in code paths that do not invoke
the new root materializer. All repeat rows remain **0 B/op**. Captures and
child identities are under `cap-baseline-control` / `cap-retained-control`.

Fresh owning-suite validation is retained under
`artifacts/grv-benchmark-020/final-gates-20261006T011442939Z-0be29b185d644823977d584f4ccc0350`.
The invocation's `result.json` confirms all serial gates passed:

- FixedMathSharp: **4,239 Release / 4,218 Lean** tests, plus **49** Chronicler
  owning tests per configuration; **867** focused Debug tests.
- Gravitas: **4,537 Release / 4,476 Lean** tests; **28** focused Debug tests.
- Release/Lean builds cover both library target frameworks, standard and Lean
  variants, tests and benchmarks. Both DocFX builds pass with warnings as
  errors, local API links and branding/resource checks.
- Every owning capture has exactly **100% reachable line, branch and method
  coverage**, checked independently in rendered and paired raw reports.
  Coverage exclusions remain unchanged. Chronicler's partially exercised core
  dependency is not unioned into the independently complete core capture.

| Rendered owning capture | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp Release | 52,958 / 52,958 | 12,266 / 12,266 | 3,949 / 3,949 |
| FixedMathSharp Lean | 53,051 / 53,051 | 12,266 / 12,266 | 3,945 / 3,945 |
| FixedMathSharp.Chronicler Release / Lean, each | 85 / 85 | 12 / 12 | 18 / 18 |
| Gravitas Release | 56,272 / 56,272 | 16,302 / 16,302 | 5,413 / 5,413 |
| Gravitas Lean | 56,270 / 56,270 | 16,302 / 16,302 | 5,412 / 5,412 |

FixedMathSharp's paired raw totals equal those rendered totals. Gravitas's raw
OpenCover summaries independently report **44,542 / 44,542 sequence points,
13,296 / 13,296 branches, 4,603 / 4,603 methods** in Release and **44,540 /
44,540, 13,296 / 13,296, 4,602 / 4,602** in Lean. Each reporting format is
checked against its own complete totals rather than substituting a rounded
percentage or combining unrelated captures. Source reviews found no actionable
correctness or simplicity issues; reviewers ran no concurrent heavy jobs.

## Remaining Cost And Next Investigation

Four matched `WorkloadActual` profiles use one launch, three warmups and eight
500 ms iterations for endpoint and oblique fixtures, with identical runtime,
local-stack and affinity settings. The retained timed/profiled child MVID is
`9a5f65c9-9ce9-4ba5-aeaa-d7f30eb6634e`; the verified baseline cap child MVID is
`c38ca9cd-d4dc-42d5-9b98-e87ab67a96a6`. The new helpers also appear in the
retained oblique traces. Inclusive sampled CPU percentages below overlap and
must not be added together:

| Named frame | Baseline mixed / 3D | Retained mixed / 3D |
| --- | ---: | ---: |
| Ellipse magnitude-to-raw comparison | 58.666% / 58.722% | 39.436% / 38.609% |
| Exact retained-root sign | 49.757% / 50.270% | 39.640% / 36.802% |
| Largest-positive root acquisition | 30.335% / 29.859% | 43.029% / 42.943% |
| Ellipse depth materialization | Not separately sampled | 40.103% / 39.687% |
| Shared polynomial interval bounds | Old interval-sign wrapper: 19.745% / 19.826% | 23.724% / 21.447% |
| Endpoint analytic materialization | 79.947% / 78.345% | 81.431% / 79.313% |

The larger retained interval percentage is not evidence of a wall-time
regression: the complete query is substantially shorter and the extracted
frame now also serves the initial magnitude bounds. Profiles localize work;
the twelve-row timing capture establishes the complete-query cost.

GRV-Benchmark-020 remains **open**. Root acquisition and exact depth sign work
still account for substantial oblique cost, while endpoint output retains its
analytic rounding searches. Next, investigate reuse of existing certified
point evaluation and exact rational-square materialization before adding a
new root representation, cache or solver. Keep the full mixed/3D matrix,
signed/zero-`Q` correctness families, ordinary controls, shared analytic
consumers, zero-allocation gates and stack proofs. Any shared-owner refinement
must preserve all consumers, not merely the timed quarter-unit fixture.

The representative oblique query remains about **0.72–0.75 ms** on this host;
an endpoint rim remains about **86–87 µs**. These costs do not establish a
fixed-rate simulation capacity or cross-platform timing guarantee. Budget
complete representative world steps and expected curved-contact frequency.
The accepted implementation exposes no accuracy setting or approximate route.

To reproduce the candidate, build with local source selection and fresh source
timestamps after any controlled restoration, then verify the generated child:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -t:Rebuild -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleCircleContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-020/reproduction
```

The ignored `measure-verified.ps1`, `verify-assembly.ps1` and
`cap-controls.ps1` retain the complete assembly/source checks used here.
Changing restoration timestamps does not change source content or its hash.
No benchmark-only instrumentation is added to runtime library code.
