# Triangle/Cone Normal Cost Refinement

**Status:** Complete; GRV-Benchmark-019 closed 2026-10-06.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This is the third GRV-Benchmark-019 refinement phase. Starting revisions are
Gravitas `01b9e1e` and FixedMathSharp `fbd167d`. The complete exact contact
query remains the correctness baseline.

## Retained Design

The shared root-normal owner narrows its existing integer search before exact
materialization. It squares each component once, then encloses that square and
the squared gradient length at the retained root using the existing compact
signed Horner evaluator. Both polynomials share their maximum coefficient-height
normalization. Radius scaling happens afterward, preserving useful work bits.

The evaluator uses at most 59 fractional work bits and 17 coefficients. Every
partial magnitude is below `count*2^precision`; the full compact result and its
`2*count` outward error allowance fit unsigned 64-bit endpoints. Open root cells
reserve `2*ceilLog2(count)` additional denominator bits to bound cell movement;
exact dyadic singletons need no cell-width margin. This reuses the existing
quantization and product-truncation proof rather than adding interval arithmetic.

For component C and squared length L, the existing clipped ratio/square-root
owner bounds `floor(scale*sqrt(C²/L))`. Lower numerator/upper denominator gives
the lower floor; upper numerator/lower denominator gives the upper floor.
Unsigned endpoints are represented without narrowing to signed longs, and
their products with at most 126-bit scale squared fit the existing Signed320
owner. Insufficient cell precision or an uncertain positive denominator retains
the original full search range.

The lower floor already passes its integer threshold, so the search starts
strictly above it and finds the first failing integer. Coincident floor bounds
skip integer queries. The existing exact half-raw sign query still decides
nearest-even rounding in every case. Component signs, orientation, canonical
zero components and root refinement retain their existing authority. Preparation
returns before threshold work; no retained cache, extra persistent scratch,
public API, friendship or assertion-only production work is added.

The shared owner serves cone, cylinder and slab contacts, including higher-degree
cylinder-pair gradients. This is reusable deterministic geometry inside
FixedMathSharp; Gravitas needs no production change or math facade.

## Discarded Experiments

A certified edge-parameter enclosure narrowed witness-coordinate searches when
both cell-endpoint denominators had the same nonzero sign. A one-launch interior
rim result looked promising, but confirmation measured 1,081.98 +/- 14.16 us
against 1,074.84 +/- 11.50 us for its fresh baseline; oblique and touch rows also
showed no repeatable benefit. The implementation was removed.

An exact dyadic witness shortcut likewise failed confirmation: its preliminary
rim-touch result was 599.24 us, while two-launch confirmation measured 629.92 us
against 623.54 us for its corresponding baseline. It was removed. These
comparisons have different baselines from the retained normal matrix below.
The initial 32-bit normal enclosure gave mixed results; retaining all compact
work bits improved the useful enclosure without a second evaluator.

## Evidence

All captures use `UseLocalLsfStack=true` in MSBuild and the environment,
Windows 11/i7-9700K, SDK 10.0.302/.NET 8.0.29, BelowNormal priority,
`DOTNET_PROCESSOR_COUNT=2` and one heavy job at a time. Fresh baseline and
retained unprofiled captures use two launches, five warmups and fifteen measured
250 ms adaptive iterations per launch. CLI affinity is 3; exported job metadata
reports 11. No different effective affinity is inferred.

The baseline temporarily restores only the three affected production files from
FixedMathSharp `fbd167d`, then restores byte-preserved experiment snapshots.
Compilation-input timestamps are refreshed before builds. Twenty source hashes
are checked across each capture, and the launcher private IL is compared with
the actual generated child's IL, including every changed normal method.
Post-validation cleanup restored the discarded witness file's committed line
endings after an empty content diff; the three affected production files retain
their captured byte hashes.

All 34 new normal cases pass against both the committed baseline and retained
source. Independent BigInteger ratio/square-root midpoint oracles cover exact
half-raw ties, irrational components, both orientations, zero and maximum scales,
shallow and refined root cells, dyadic cancellation, wide common-factor
cancellation, degree-sixteen squares and compact values wider than 32 bits.
Borrowed coefficients remain unchanged. The final focused Debug filter passes
507 normal/cone/cylinder/slab cases.

Final two-launch results are **microseconds per dispatched query**, mean +/-
standard deviation. Every row is **0 B/op**.

| Geometry | Fresh committed baseline | Retained confirmation |
| --- | ---: | ---: |
| Apex face | 22.37 +/- 0.21 | 22.21 +/- 0.27 |
| Base face | 21.66 +/- 0.15 | 21.97 +/- 0.32 |
| Side face | 27.66 +/- 0.40 | 27.36 +/- 0.39 |
| Side intrusion | 202.09 +/- 2.03 | 199.26 +/- 1.88 |
| Interior stationary rim | 1,052.85 +/- 9.92 | 918.32 +/- 16.56 |
| Oblique rim | 658.04 +/- 9.57 | 638.65 +/- 9.30 |
| Rim gap | 86.44 +/- 1.04 | 86.29 +/- 1.85 |
| Rim touch | 604.29 +/- 4.34 | 605.42 +/- 8.78 |
| Unrepresentable relative center | 464.59 +/- 2.46 | 453.21 +/- 3.11 |

Interior-rim cost falls 12.8%. Oblique and unrepresentable-center means fall
3.0% and 2.4%; those smaller changes are descriptive. Ordinary paths and touch
remain broadly comparable; no speedup is attributed to paths that do not reach
the changed normal owner. These are complete dispatched queries, not isolated
normal timings or simulation-frame capacity measurements.

Captures are `artifacts/grv-benchmark-019/phase3-normal-baseline` and
`phase3-normal-final`, with paired source/launcher/child identities. Preliminary
`phase3-normal-confirmed` predates the final proven-lower-threshold removal;
only the final capture above supports the retained result.

Shared-owner controls use the same two-launch settings. Both paired captures
are retained below, in us/query, mean +/- standard deviation; all rows are
0 B/op. The first pair's oblique means rose about 2%, prompting a fresh pair
rather than silently attributing the difference to noise.

| Consumer / geometry | First baseline | First retained | Recheck baseline | Recheck retained |
| --- | ---: | ---: | ---: | ---: |
| Cylinder cap | 50.54 +/- 0.65 | 48.46 +/- 0.75 | 52.92 +/- 1.32 | 50.01 +/- 0.37 |
| Circle slab cap | 18.68 +/- 0.26 | 18.93 +/- 0.53 | 19.56 +/- 0.30 | 19.80 +/- 0.30 |
| Cylinder side | 23.59 +/- 0.27 | 24.14 +/- 0.30 | 24.96 +/- 0.13 | 25.05 +/- 0.45 |
| Circle slab side | 23.46 +/- 0.18 | 23.70 +/- 0.14 | 24.30 +/- 0.15 | 24.26 +/- 0.17 |
| Cylinder oblique rim | 607.97 +/- 3.03 | 620.46 +/- 10.70 | 636.18 +/- 13.01 | 647.18 +/- 16.18 |
| Circle slab oblique rim | 607.46 +/- 4.94 | 618.52 +/- 3.72 | 632.30 +/- 5.74 | 640.46 +/- 9.26 |

The unchanged baseline itself moved by about 4.6%/4.1% on the oblique rows
between captures. Recheck retained means are 1.7%/1.3% higher, with overlapping
99.9% confidence intervals. These runs do not establish a material shared-path
regression or speedup; the small positive differences remain visible. Captures
are `phase3-normal-baseline-controls`, `phase3-normal-final-controls`,
`phase3-normal-recheck-baseline-controls` and `phase3-normal-recheck-final-controls`;
all pass source/private child IL gates. No shared-control comparison is used
to quantify the cone normal improvement.

The final EventPipe capture uses one launch, three warmups and eight 500 ms
iterations for seven cone fixtures. Its timings are not mixed with the
unprofiled matrix. Actual-only sampled CPU identifies the remaining work:

| Fixture / owner | Inclusive CPU | Exclusive CPU |
| --- | ---: | ---: |
| Interior rim: root sign authority | 58.35% | 2.19% |
| Interior rim: winner materialization | 40.36% | 0.08% |
| Interior rim: witness-coordinate search | 30.00% | 0.89% |
| Interior rim: rounded normal components | 4.69% | 0.08% |
| Interior rim: conservative normal bounds | 1.07% | 0.07% |
| Rim gap: analytic traversal | 87.86% | 0.00% |
| Rim gap: generator fan | 71.64% | 0.59% |
| Rim gap: generator candidate evaluation | 40.96% | 3.03% |

Inclusive samples overlap and do not sum to stage time. Inlining and sampling
prevent treating changes from the preceding profile as isolated measured
speedups. Normal rounding is now a small sampled share; witness/root-sign work
remains substantial. Captures and identities are `phase3-normal-profile` and
`phase3-normal-profile-summary.json`.

## Complete Validation

Both solutions build in Release and ReleaseLean, including their library
netstandard2.1/net8.0 targets, using `UseLocalLsfStack=true` throughout. The
expanded Debug arithmetic/resource filter passes 842 FixedMathSharp tests;
Gravitas's matching mesh-contact filter passes 25. Existing dirty-stack,
one-MiB caller-headroom and strict allocation tests remain the resource gates.

| Owning test suite | Release | ReleaseLean |
| --- | ---: | ---: |
| FixedMathSharp core | 4,409 | 4,388 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4,537 | 4,476 |

Every suite passes with zero skipped tests. Coverage exclusions are unchanged.
Every count below is both covered and total: reachable line, branch and method
coverage is exactly 100%. Raw OpenCover totals and rendered ReportGenerator
summaries are independently checked.

| Owner/configuration | OpenCover sequence points / branches / methods | Rendered lines / branches / methods |
| --- | ---: | ---: |
| FixedMathSharp Release | 53,096 / 12,322 / 3,956 | 53,096 / 12,322 / 3,956 |
| FixedMathSharp ReleaseLean | 53,189 / 12,322 / 3,952 | 53,189 / 12,322 / 3,952 |
| FixedMathSharp.Chronicler, both | 85 / 12 / 18 | 85 / 12 / 18 |
| Gravitas Release | 44,542 / 13,296 / 4,603 | 56,272 / 16,302 / 5,413 |
| Gravitas ReleaseLean | 44,540 / 13,296 / 4,602 | 56,270 / 16,302 / 5,412 |

Core captures include the source FluentAssertions helper. Chronicler adapter
coverage is checked in its own module rather than merging its partially
exercised core dependency into the independently complete core capture.
Changed normal methods have identical fully covered complexity in both
configurations: component rounding 14, preparation 4, floor bounds 6, threshold
updates 2 and compact nonnegative enclosure 2. The existing complexity-register
entry now explains conservative narrowing and exact final rounding.

Both DocFX builds pass with warnings as errors; API branding/resource and local
link checks pass. Independent source and evidence reviews have no remaining
findings. Fresh test logs, raw/rendered reports and the passing `result.json`
are under
`artifacts/grv-benchmark-019/final-gates-20261006T173022711Z-03af87cd14984973add009a073db5afd`.

## Status At The End Of This Phase

At the end of this phase, GRV-Benchmark-019 remained active. The
[final root-sign refinement](2026-10-06-triangle-cone-final-refinement.md)
subsequently closed the isolated-query investigation; workload frequency and
complete-step capacity remain GRV-Benchmark-024. This pass materially reduces normal
materialization, but exact witness-coordinate searches and root-sign/refinement
remain expensive. The next useful investigation is repeated linear threshold
evaluation within the existing witness owner, with the current exact rounding
and paired anchors as its contract. The discarded edge-parameter and dyadic
experiments are not retained merely because their preliminary runs looked good.
Generator admission remains another measured target; completeness and stable
tie ownership must stay intact. No retained-root cache is justified here.

Representative fixture frequency and complete-step capacity remain
GRV-Benchmark-024. Single-triangle timings do not establish game/server capacity.

Reproduce the unprofiled matrices after a coordinated source build:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
(Get-Process -Id $PID).PriorityClass = 'BelowNormal'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*MeshConeContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*MeshCylinderContactBenchmarks*CapFace*' '*MeshCylinderContactBenchmarks*SideFace*' '*MeshCylinderContactBenchmarks*ObliqueRim*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json
```

Run each command serially. Benchmark setup verifies classification and exact
fixture depths before timing; source and generated-child identity checks remain
necessary when comparing temporarily restored upstream baselines.
