# Triangle/Cone Generator Cost Refinement

**Status:** Complete; GRV-Benchmark-019 closed 2026-10-06.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This is the second GRV-Benchmark-019 refinement phase. Starting revisions are
Gravitas `6ccce9c` and FixedMathSharp `6dd97ca`. The complete exact contact
query remains the correctness baseline.

## Retained Design

Cone selection now retains one complete candidate in cone-local coordinates.
Ranking, face certificates and stationary-root admission read only gap fields.
Analytic depth retains its existing local-axis cancellation; other depth rounding
reads the gap representation. Only the final analytic winner snapshots its local normal
for witnesses and transforms the normal into world coordinates. Root winners
use their existing world-gradient path; separated queries need neither step.
This follows the existing cylinder selection pattern and removes duplicate
copies and repeated transforms without changing candidate order or ties.

Because selection no longer mutates candidate inputs, both vertex orientations
share their prepared coefficients and denominator, changing only rational X/Z
signs. Both edge-circle branches share their coefficients and denominator,
changing only radical X/Z signs. Negative/positive orientation order, repeated
root handling, support comparisons against all three vertices and first-winner
tie ownership remain intact. The opposite-radial direction of a source vertex
does not imply that vertex owns triangle support, so those comparisons remain
necessary.

The fan factors `RawScale²` into `H²+R²` once. Constant, vertex and edge
denominators retain exactly their former values. The compact face/generator seam
prepares its normalized denominator in the existing caller; projected chart
seams still defer equality to the complete fan. The existing bounded magnitude
owners perform every product; no arithmetic facade, cache or new owner is added.

The existing quadratic candidate builder now computes
`(A+B*sqrt(K))² = (A²+B²K) + 2AB*sqrt(K)` directly. It needs one cross-product
rather than separately multiplying A*B and B*A. The nonnegative rational
coefficient and signed radical coefficient receive canonical zero signs.
The original numerator still determines the signed gap. An independent test
first reproduced a baseline contract violation for A=0, B=-2, K=0: the generic
square marked its zero rational coefficient positive. Its numeric value was
correct, and no incorrect public physics result was found. The direct builder
fixes that representation without broadening the shared arithmetic contract.

Scratch storage is reused within the same bounded owners. The fan adds one
40-word slot while each generator evaluation removes two; direct squaring
replaces the generic square's nested scratch, and selection needs no transform
scratch. The final local-normal snapshot is made once for analytic witnesses.
These are source-level lifetime observations, not a measurement of total JIT
stack use. Existing dirty-stack, one-MiB caller-headroom and allocation tests
remain the resource gates.

## Evidence

Captures use local source selection in both MSBuild and the environment,
Windows 11/i7-9700K, SDK 10.0.302/.NET 8.0.29, BelowNormal priority,
`DOTNET_PROCESSOR_COUNT=2` and one heavy job at a time. Fresh baseline and
retained captures each use two launches, five warmups and fifteen measured
250 ms adaptive iterations per launch. CLI affinity is 3; exported job metadata
reports 11. No different effective affinity is inferred.

The baseline temporarily restores only the four affected production files from
FixedMathSharp `6dd97ca`, then restores the experiment from byte-preserved
snapshots. Source hashes are checked across each measurement, and launcher
private IL is compared with the actual generated child's IL. One preliminary
confirmation was rejected because the generated child reused the baseline
dependency after timestamp-preserving source restoration. Its results are
discarded. Refreshing compilation-input timestamps before capture ensures the
generated child rebuilds; the private-IL gate remains authoritative.

The new rotated generator regression passed on the baseline before optimization.
Its exact rational frame maps the proven normal (-4,-3,0)/5 to world down; both
windings and all cyclic orders retain depth 11/10 and the same local witnesses.
The independent quadratic regression covers signed coefficients, zero radicands,
exact cancellation and wide coefficients. The retained implementation passes
485 focused Debug cone/cylinder/slab and quadratic cases.

The final two-launch matrix is **microseconds per dispatched query**, mean +/-
standard deviation. Every row is **0 B/op**.

| Geometry | Fresh committed baseline | Retained confirmation | Observed reduction |
| --- | ---: | ---: | ---: |
| Apex face | 24.91 +/- 0.20 | 22.81 +/- 0.14 | 8.4% |
| Base face | 23.66 +/- 0.12 | 22.85 +/- 0.43 | 3.4% |
| Side face | 31.10 +/- 0.58 | 28.32 +/- 0.38 | 8.9% |
| Side intrusion | 213.79 +/- 1.68 | 210.30 +/- 1.33 | 1.6% |
| Interior stationary rim | 1,102.58 +/- 13.51 | 1,089.96 +/- 13.88 | 1.1% |
| Oblique rim | 686.48 +/- 10.28 | 668.72 +/- 13.17 | 2.6% |
| Rim gap | 94.95 +/- 1.11 | 88.70 +/- 1.62 | 6.6% |
| Rim touch | 624.77 +/- 4.28 | 622.41 +/- 8.00 | 0.4% |
| Unrepresentable relative center | 492.71 +/- 3.58 | 473.24 +/- 7.64 | 4.0% |

The gap and ordinary face improvements support retaining the simpler ownership
and shared arithmetic. The interior-rim and rim-touch differences are small
relative to their distributions; no material improvement is claimed for them.
This phase does not resolve stationary-root materialization cost. Preliminary
one-launch trials are diagnostic rather than an isolated attribution of each
edit: local selection measured rim gap at 92.57 us; fan arithmetic at 87.46 us.
Only the complete verified matrix above supports the retained result.

Captures are `artifacts/grv-benchmark-019/phase2-baseline` and
`phase2-single-winner`, with source/launcher/child identity files alongside them.
The rejected `phase2-confirmed` capture is excluded from all retained claims.
Fresh shared-owner controls use the same two-launch settings and report 0 B/op
in every row. Baseline/retained mean +/- standard deviation, in us/query:

| Consumer / geometry | Committed baseline | Retained confirmation |
| --- | ---: | ---: |
| Cylinder cap | 51.77 +/- 0.79 | 50.78 +/- 0.88 |
| Circle slab cap | 19.45 +/- 0.19 | 19.71 +/- 0.25 |
| Cylinder side | 24.77 +/- 0.45 | 24.90 +/- 0.13 |
| Circle slab side | 24.84 +/- 0.16 | 24.48 +/- 0.31 |
| Cylinder oblique rim | 649.45 +/- 11.84 | 650.99 +/- 11.31 |
| Circle slab oblique rim | 643.85 +/- 12.79 | 635.61 +/- 10.89 |

All means differ by less than 2%; no material shared-path regression or speedup
is claimed. These consumers do not call the changed quadratic square builder.
Refreshing the control baseline avoids attributing the earlier capture's lower
costs to this cone refinement. Captures are `phase2-baseline-controls` and
`phase2-controls`; both pass source and private child IL gates.

The final EventPipe capture uses one launch, three warmups and eight 500 ms
iterations. Its timings are not mixed with the unprofiled matrix. Actual-only
sampled CPU shows the remaining work:

| Fixture / owner | Inclusive CPU | Exclusive CPU |
| --- | ---: | ---: |
| Rim gap: analytic traversal | 83.98% | 0.17% |
| Rim gap: generator fan | 66.81% | 1.42% |
| Rim gap: generator candidate evaluation | 38.22% | 2.81% |
| Interior rim: root sign authority | 48.71% | 2.00% |
| Interior rim: winner materialization | 42.48% | 0.31% |
| Interior rim: rounded normal components | 20.54% | 1.15% |
| Interior rim: witness-coordinate search | 15.70% | 0.57% |

Inclusive samples overlap and do not sum to stage time. Inlining and sampling
also prevent treating differences from the earlier profile as measured phase
speedups. The trace confirms both remaining targets rather than justifying a
new cache or a weaker support rule. Artifacts are `phase2-profile`, its paired
source/launcher/child identity files and `phase2-profile-summary.json`.

## Complete Validation

Both solutions build in Release and ReleaseLean, including their library
netstandard2.1/net8.0 targets, using `UseLocalLsfStack=true` throughout. The
expanded Debug resource/arithmetic filter passes 808 FixedMathSharp tests and
the matching Gravitas mesh-contact filter passes 25 tests.

| Owning test suite | Release | ReleaseLean |
| --- | ---: | ---: |
| FixedMathSharp core | 4,375 | 4,354 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4,537 | 4,476 |

Every suite passes with zero skipped tests. Repository coverage exclusions are
unchanged. Every count below is both covered and total: reachable line, branch
and method coverage is exactly 100%. Raw OpenCover totals and rendered
ReportGenerator summaries are independently checked.

| Owner/configuration | OpenCover sequence points / branches / methods | Rendered lines / branches / methods |
| --- | ---: | ---: |
| FixedMathSharp Release | 53,061 / 12,316 / 3,953 | 53,061 / 12,316 / 3,953 |
| FixedMathSharp ReleaseLean | 53,154 / 12,316 / 3,949 | 53,154 / 12,316 / 3,949 |
| FixedMathSharp.Chronicler, both | 85 / 12 / 18 | 85 / 12 / 18 |
| Gravitas Release | 44,542 / 13,296 / 4,603 | 56,272 / 16,302 / 5,413 |
| Gravitas ReleaseLean | 44,540 / 13,296 / 4,602 | 56,270 / 16,302 / 5,412 |

Core captures include the source FluentAssertions helper. Chronicler adapter
coverage is checked in its own module rather than merging its partially
exercised core dependency into the independently complete core capture.
Collector complexity is identical in both configurations: contact ownership 26,
direction admission 22, generator enumeration 16 and quadratic square building
2, all fully covered. The complexity register reflects the changed seam
dispatch.

Both DocFX builds pass with warnings as errors; API branding/resource and local
link checks pass. Independent source and evidence reviews have no remaining
findings. All 17 production-source hashes in the accepted timing, control and
profile manifests still match the final source. The fresh passing gate capture,
including raw/rendered reports, test logs and `result.json`, is
`artifacts/grv-benchmark-019/final-gates-20261006T152417563Z-5fd87fa9ff604ea7b4163f3a0b6d5bd7`.

## Status At The End Of This Phase

At the end of this phase, GRV-Benchmark-019 remained active. This phase reduces generator work; exact
stationary-root witness and normal materialization still dominate interior-rim
queries. A useful next experiment is to narrow the existing witness-coordinate
search using a certified shared edge-parameter interval, while retaining exact
integer and half-raw sign queries as the rounding authority. Reuse existing
ratio and outward-bound owners; uncertain denominator cells retain the current
full authored-coordinate range. No retained-root cache or approximate fallback
is justified by the present evidence.

Representative frequency and complete-step capacity remain GRV-Benchmark-024;
single-triangle query times do not establish game/server workload capacity.
