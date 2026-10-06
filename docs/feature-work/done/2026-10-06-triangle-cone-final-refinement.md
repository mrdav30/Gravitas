# Triangle/Cone Final Cost Refinement

**Status:** Complete; GRV-Benchmark-019 closed 2026-10-06.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This is the fourth GRV-Benchmark-019 refinement phase. Starting revisions are
Gravitas `01b9e1e` and FixedMathSharp `80f39ff`. The preceding
[normal refinement](2026-10-06-triangle-cone-normal-refinement.md) remains part of
the retained design and evidence. Complete exact contact geometry remains the
correctness baseline.

## Retained Design

The existing retained-cell sign trial uses up to 59 fractional work bits rather
than 32. Its unchanged admission rule requires effective cell shift to cover
precision plus `2*ceilLog2(queryCoefficientCount)` bits. Coefficient quantization,
Horner product truncation and cell movement retain the existing `2*count` error
certificate; uncertain answers still reach the exact resultant/equality fallback.
No tolerance or approximate answer replaces an exact sign.

Production queries have degree at most sixteen. Both precisions use the existing
two-word result workspace for that range, with unchanged product scratch width.
Higher-degree internal queries continue to use the existing count-dependent
workspace formula. Virtual variable scaling, coefficient-height normalization,
dyadic singleton evaluation and paired numerator/metadata mutation are unchanged.

The defining polynomial's coefficient-height scan now occurs only when the
retained trial is uncertain. A decisive trial already returns without using that
height, so this removes dead work from repeated witness and normal thresholds.
No new helper, branch, cache, public API, friendship or assertion-only production
work is introduced. This is policy-neutral arithmetic in FixedMathSharp;
Gravitas needs no production change.

## Experiments And Decision

The precision-only one-launch trial measured interior rim at 943.68 us against
the fresh 966.98 us baseline; other rows were mixed. Adding the deferred height
scan measured 900.41 us in a one-launch trial. Two-launch confirmation of both
changes measured 927.41 us, a 4.1% reduction with separated 99.9% confidence
intervals. The larger preliminary reduction is not the final performance claim.

An undoubled integer witness comparison removed positive scaling from threshold
queries while retaining the exact half-raw comparison. Fourteen characterization
cases passed, including signed half-raw ties, both denominator orientations,
signed limits and nonconstant full-domain one-third crossings with independent
oracles. Its two-launch interior result was 924.32 us versus 927.41 us without
it; all corresponding row intervals overlap. Its preliminary touch improvement
also failed confirmation. The production change and experimental test file were
removed; characterization sources and logs remain in ignored artifacts.

Direct rational-coordinate comparison and a transformed coordinate-root design
would add root ordering, equality, endpoint and materialization obligations.
Neither has an established advantage in the complete path. Previously rejected
dyadic and shared edge-parameter witness experiments are not revived. Existing
owners remain the simplest supported implementation after the measured trials.

## Matched Measurements

All captures use `UseLocalLsfStack=true` in MSBuild and the environment,
Windows 11/i7-9700K, SDK 10.0.302/.NET 8.0.29, a BelowNormal launcher,
`DOTNET_PROCESSOR_COUNT=2` and one heavy job at a time. Unprofiled confirmations
use two launches, five warmups and fifteen measured 250 ms adaptive iterations
per launch. CLI affinity is 3; exported job metadata reports 11. No different
effective affinity is inferred. Benchmark setup checks contact classification
and exact fixture depths before timing.

The fresh baseline is the committed source, including the preceding normal
refinement. Temporary baseline controls restore only the two experiment files
from `80f39ff`, then restore byte-preserved snapshots. Compilation timestamps are
refreshed before rebuilding. Twenty source hashes are checked across each
capture, and 27 private method IL hashes compare the launcher with the actual
generated child, including the root-sign core and compact Horner evaluator.

Cone rows below are **microseconds per dispatched query**, mean +/- standard
deviation. All rows report **0 B/op**.

| Geometry | Fresh committed baseline | Retained | Mean change |
| --- | ---: | ---: | ---: |
| Apex face | 23.118 +/- 0.349 | 23.428 +/- 0.493 | +1.340% |
| Base face | 22.494 +/- 0.319 | 22.668 +/- 0.431 | +0.772% |
| Interior rim | 966.981 +/- 18.499 | 927.410 +/- 18.765 | -4.092% |
| Oblique rim | 663.809 +/- 13.794 | 667.396 +/- 15.051 | +0.540% |
| Rim gap | 89.077 +/- 1.289 | 89.350 +/- 2.204 | +0.307% |
| Rim touch | 625.945 +/- 10.810 | 630.930 +/- 13.158 | +0.796% |
| Side face | 28.271 +/- 0.423 | 28.714 +/- 0.555 | +1.567% |
| Side intrusion | 209.987 +/- 4.506 | 210.712 +/- 4.709 | +0.345% |
| Unrepresentable relative center | 480.550 +/- 10.809 | 480.146 +/- 12.271 | -0.084% |

Interior-rim 99.9% confidence intervals are 954.622-979.340 us for baseline and
914.873-939.947 us for retained. The other eight row intervals overlap baseline;
their small visible differences establish neither gains nor regressions.
Clean captures are `phase4-baseline` and `phase4-sign-only` under
`artifacts/grv-benchmark-019`. `phase4-final` is the discarded combined witness
confirmation, not the retained source. `phase4-sign-only-comparison.json` records
the full raw-report comparison.

Shared controls use the same units and methodology; all report **0 B/op**.

| Consumer / geometry | Fresh baseline | Retained | Mean change |
| --- | ---: | ---: | ---: |
| Cylinder / cap | 50.716 +/- 1.221 | 50.857 +/- 1.401 | +0.277% |
| Circle slab / cap | 19.895 +/- 0.426 | 19.730 +/- 0.280 | -0.825% |
| Cylinder / oblique | 649.069 +/- 14.759 | 637.105 +/- 15.139 | -1.843% |
| Circle slab / oblique | 640.294 +/- 14.815 | 653.509 +/- 9.988 | +2.064% |
| Cylinder / side | 24.814 +/- 0.402 | 24.791 +/- 0.352 | -0.093% |
| Circle slab / side | 24.906 +/- 0.442 | 24.300 +/- 0.380 | -2.431% |

Five control row intervals overlap baseline. Circle-slab side mean is 2.4%
lower, with narrowly separated intervals; this isolated control result is not
a general cylinder/slab speed claim. Circle-slab oblique mean is visibly 2.1%
higher but intervals overlap. No material regression is established by these
controls. Captures are `phase4-baseline-controls` and `phase4-retained-controls`;
`phase4-controls-comparison.json` records raw statistics and uncertainty.

## Actual-Only Profile

The retained seven-row EventPipe capture is `phase4-profile`; its actual-only
summary is `phase4-profile-summary.json`. Attribution includes sampled CPU only
inside `WorkloadActual`, excluding pilot, warmup, overhead and waiting. Inclusive
owners overlap and must not be added. Profiling percentages are diagnostic
distributions, not before/after timing gains; clean matrices above decide speed.

| Fixture / owner | Inclusive sampled CPU |
| --- | ---: |
| Interior rim / root-sign core | 50.639% |
| Interior rim / root refinement | 35.340% |
| Interior rim / final rim materials | 31.057% |
| Interior rim / witness-coordinate search | 18.815% |
| Oblique rim / root-sign core | 22.295% |
| Rim gap / analytic traversal | 82.020% |
| Rim gap / generator fan | 59.926% |

Exact root and witness work remain visible costs. Gap rejection remains chiefly
analytic/fan work and does not justify weakening feature admission. These owners
are concrete candidates for workload-driven investigation if #024 establishes
material step-budget pressure.

## Correctness And Resource Gates

Five new retained-cell regressions construct neighboring linear thresholds with
an independent integer-square-root oracle. They cover 36/48 threshold bits,
4,096-bit non-power-of-two coefficient content and a virtually scaled tiny root.
The committed baseline produces correct signs but unnecessarily advances the
already decisive cells from shifts 64 to 66 or 128 to 130. The retained variant
preserves both the original numerator and its metadata. Exact equality and
uncertain cells retain the existing fallback tests.

Serial focused Debug checks pass 847 FixedMathSharp cases and 25 Gravitas cases,
including existing dirty-stack, 1 MiB stack, full-domain and strict allocation
guardrails. These are broader resource checks than the timed quarter-unit
fixtures. Full solution builds cover `netstandard2.1` and `net8.0`.

| Owner | Release tests | ReleaseLean tests |
| --- | ---: | ---: |
| FixedMathSharp core | 4,414 | 4,393 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4,537 | 4,476 |

All tests pass without skips. Fresh OpenCover and rendered ReportGenerator
captures are exactly **100% reachable line, branch and method coverage** in both
configurations. Existing exclusions are unchanged; only the copied settings'
output formats add OpenCover for paired verification. Every numerator below
equals its denominator, rather than relying on rounded percentages.

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

The changed sign core remains complexity 38, with all 71 sequence points,
38 branch points and its method covered in both configurations. The existing
complexity-register rationale now records the stronger retained certificate and
deferred defining-height scan.

Both DocFX builds pass with warnings as errors; API branding/resource and local
link checks pass. Fresh test logs, raw/rendered reports and the passing
`result.json` are under
`artifacts/grv-benchmark-019/final-gates-20261006T184240940Z-042b85d66be247b88f0f657479e88d3d`.
Independent source and evidence reviews have no remaining findings.

This validation uses the unreleased local stack. Release FixedMathSharp first,
then validate Gravitas against that published package before releasing Gravitas.

## Closure

GRV-Benchmark-019 closes under the user's judgement-based performance bar.
The complete exact query has now received four measured refinement passes,
retaining shared-owner improvements and removing experiments that did not earn
their cost. Ordinary base/apex/side fixtures cost about 23-29 us; the full-domain
relative-center fixture costs about 480 us. These are distinct configurations,
not one uniform contact cost.

Interior rim remains about **0.93 ms/query**, oblique rim about **0.67 ms/query**,
touch about **0.63 ms/query**, and fan-gap rejection about **89 us/query** on this
host. These are material costs. Closure publishes them and preserves exact
admission, stable ties, canonical paired witnesses, raw-neighbor classification,
bounded stack use and zero allocations. It does not assert a fixed host rate,
rare-feature frequency or multiplayer/server capacity.

GRV-Benchmark-024 now explicitly includes cone/triangle and shared triangle/
cylinder workloads alongside capsule/slab fixtures. It must count expensive
candidate evaluation as well as final winning contacts and measure complete
deterministic world steps. Further solver redesign or cross-frame retained state
needs evidence that this work materially consumes a representative step budget.
The existing GRV-Benchmark-018 cylinder investigation remains independent;
improvements in shared owners can continue to benefit cones without reopening
this isolated-query signal automatically.

## Reproduction

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
(Get-Process -Id $PID).PriorityClass = 'BelowNormal'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*MeshConeContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*MeshCylinderContactBenchmarks*CapFace*' '*MeshCylinderContactBenchmarks*SideFace*' '*MeshCylinderContactBenchmarks*ObliqueRim*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json
dotnet test Gravitas.slnx -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false
dotnet test Gravitas.slnx -c ReleaseLean -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false
```

Run commands serially; capture shared-owner coverage in FixedMathSharp's own
suite as well. Profile with `--profiler EP` separately from clean timing.
