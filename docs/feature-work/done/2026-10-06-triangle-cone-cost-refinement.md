# Triangle/Cone Contact Cost Refinement

**Status:** Complete; GRV-Benchmark-019 closed 2026-10-06.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This records the first GRV-Benchmark-019 refinement phase on 2026-10-06.
Starting revisions are Gravitas `5d1406b` and FixedMathSharp `b8996ec`.
The complete triangle/cone query remains the correctness baseline. The old
incomplete sampler is not a correctness-equivalent performance target.

## Retained Changes

FixedMathSharp now omits a horizontal triangle's duplicate face directions:
the previously tested +/-Up axes already have the same exact support masks,
gaps and axial-first tie ownership. Existing face certificates still decide
whether the remaining feature search can be omitted.

The shared triangle/cylinder rim witness owner materializes a coordinate as
the exact rational ratio A/D when its radical coefficient vanishes. It reuses
the existing signed raw-ratio converter for nearest-even rounding. This covers
apex projections and cancellation of nonzero radical barycentric terms.
Zero-radius products now receive zero sign before weight accumulation; retaining
the unscaled projection's sign on a zero magnitude had prevented some apex
coordinates from reaching the shortcut. No incorrect public result was found.
Nonzero radical coordinates retain the existing exact materialization path.
Cylinder and planar slab contacts consume this same owner.

Cone analytic depth is materialized once before stationary rim traversal and
retained when no root wins. Both exact triangle-face orientations bound this
winner by half the cone's support width. The cone diameter is
`max(2R, sqrt(R^2+H^2)) <= 2*MaxValue`, so the analytic winner cannot clamp.
Exact orthogonal frames, common homogeneous scaling and generator-seam
forwarding preserve that proof. An exact `MaxValue` depth remains unclamped.

For rounded analytic depth d, its exact value is at most d+1/2 raw. Positive
stationary gaps at or above that bound cannot improve the earlier feature.
The cone now reuses `ConvexContactValueRoot.CompareSquaredGapToTwiceRaw` before
value-polynomial construction and mapping. Negative-gap separation and zero
selection precede this rejection; later charts still run after a zero winner.
At d=MaxValue, `2*d.raw+1` fits `ulong.MaxValue`, and the existing comparator
squares it into two words. Its 48-word query slots accommodate the existing
40-word numerator/denominator coefficients.

Outward admission is affine, a+t*b, for t in (0,1]. Nonpositive values at both
endpoints exclude the entire chart, including a=0, before polynomial work.
The strict root-level admission and reciprocal chart ordering remain intact.

The chart no longer caches value-polynomial construction behind a mutable
`valuesReady` flag. A classification probe over 2,048 deterministic triangle
candidates, skipping degenerate triangles, observed 94 constructions and no
reuse after pruning. Construction now occurs
for each surviving root. This remains correct for multiple survivors; no
uniqueness assumption is introduced. The final timed matrix checks the cost
of removing this bookkeeping, and the temporary probe is not retained as a
duplicate test suite.

These changes add no public API, approximation, configuration, cache, math
facade or new arithmetic owner. Gravitas continues consuming the complete
authored-frame contact query with its existing mesh traversal and manifold
policy.

## Measurements

Captures use `UseLocalLsfStack=true` in MSBuild and the environment, Windows 11,
an i7-9700K, .NET 8.0.29 and SDK 10.0.302. One BelowNormal heavy job runs at a
time with `DOTNET_PROCESSOR_COUNT=2` and serial builds. The CLI requests
`--affinity 3`; job metadata reports `Affinity=11`. These are recorded separately
without inferring a different mask. The baseline and final captures each use
two launches, five warmups and fifteen measured 250 ms adaptive iterations per
launch. Setup checks complete classification and independently known depths.

Values below are **microseconds per dispatched query**, mean +/- standard
deviation. Every baseline and final row reports **0 B/op**.

| Geometry | Fresh committed baseline | Retained confirmation | Observed reduction |
| --- | ---: | ---: | ---: |
| Base face | 28.95 +/- 0.59 | 22.71 +/- 0.46 | 21.6% |
| Apex face | 49.29 +/- 1.09 | 23.60 +/- 0.36 | 52.1% |
| Side face | 55.37 +/- 1.30 | 30.00 +/- 0.36 | 45.8% |
| Side intrusion | 250.24 +/- 6.94 | 208.90 +/- 2.42 | 16.5% |
| Oblique rim | 826.45 +/- 26.01 | 651.08 +/- 4.82 | 21.2% |
| Interior stationary rim | 1,229.16 +/- 27.81 | 1,046.86 +/- 16.98 | 14.8% |
| Rim touch | 649.77 +/- 17.65 | 603.75 +/- 4.03 | 7.1% |
| Rim gap | 101.39 +/- 2.56 | 93.02 +/- 1.22 | 8.3% |
| Unrepresentable relative center | 1,444.04 +/- 25.04 | 463.18 +/- 3.65 | 67.9% |

One-launch experiments separate the two retained stages. Horizontal-axis and
rational-witness changes measured base/apex/side at 23.37/24.91/32.42 us, with
interior/oblique/unrepresentable-center at 1,194.99/781.69/1,391.47 us. The
subsequent cone root/affine pruning and zero-radius sign correction measured
22.70/24.43/31.69 us and 1,096.42/676.47/494.00 us respectively. These trials
are descriptive; the two-launch table supplies confirmation. Small reductions
on unchanged or lightly affected paths must not be interpreted as isolated
algorithmic gains without the shared controls.

An initial two-launch confirmation before deleting the lazy construction flag
measured apex/side at 23.98/29.88 us and interior/oblique/relative-center at
1,061.55/651.73/464.00 us. The final current-source matrix above retains those
gains without a material regression; the small differences do not establish
an additional speedup from removing the flag.

The gap fixture rejects inside analytic traversal before the new root bound
or output shortcuts. Its lower recorded time is a descriptive run-to-run result,
not evidence that those changes accelerated rejection.

The matched shared controls use the same protocol. Values are mean +/- standard
deviation in us/query; all twelve baseline/final rows report 0 B/op.

| Path | Geometry | Baseline | Final |
| --- | --- | ---: | ---: |
| 3D cylinder | Cap face | 48.76 +/- 0.46 | 47.43 +/- 0.54 |
| Mixed circle slab | Cap face | 19.43 +/- 0.20 | 19.04 +/- 0.20 |
| 3D cylinder | Side face | 24.21 +/- 0.16 | 23.81 +/- 0.51 |
| Mixed circle slab | Side face | 24.16 +/- 0.28 | 23.77 +/- 0.42 |
| 3D cylinder | Oblique rim | 622.50 +/- 6.49 | 605.19 +/- 7.00 |
| Mixed circle slab | Oblique rim | 624.52 +/- 4.73 | 617.39 +/- 3.48 |

There is no material shared-control regression. These are controls, not claimed
new cylinder/slab gains; several unchanged paths also measure slightly lower.
The earlier `final-controls` 3D oblique row has a bimodal-distribution warning;
the current-source `confirmed-controls` capture has no such warning. Original
logs retain distribution, outlier and iteration-time warnings; samples are
not manually filtered or replaced by the one-launch trials.

Fresh rebuilds prevent restored source timestamps from selecting stale child
assemblies. Source hashes remain unchanged during each capture. Private method
IL hashes are recorded from both the launcher and the actual timed/profiled
child, including cone support, chart admission, witness materialization and
shared root/rounding owners. MVIDs may differ for generated builds; the relevant
method bodies must match. The original child assembly is also preserved in
the ignored baseline snapshot.

Raw logs, JSON reports, source hashes, IL identities and reproduction scripts
are under `artifacts/grv-benchmark-019`. Earlier captures are `baseline`,
`analytic`, `pruned`, `final`, `baseline-controls`, `final-controls` and their
profiles. The final current-source captures use `confirmed`,
`confirmed-controls` and `confirmed-profile`. Reproduce the timed cone matrix
after setting launcher limits:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
(Get-Process -Id $PID).PriorityClass = 'BelowNormal'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -t:Rebuild -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll mesh-cone-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-019/recheck
```

## Profile And Resource Evidence

Baseline EventPipe captures use one launch, three warmups and eight measured
500 ms iterations. Only `WorkloadActual` CPU samples are attributed; inclusive
frames overlap and are not added. Apex/side analytic witness materialization
accounts for 44.22%/51.05% of baseline CPU samples. Unrepresentable-center
value mapping and comparison account for 41.83%/18.14%, supporting rejection
before that work.

Interior-rim materialization accounts for 32.239% overall: 20.364% in scaled
normalized directions and 11.382% in triangle witness coordinates. Winning
parameter re-isolation accounts for only 0.493%; reconstruction has no CPU
sample under that frame. Unmanaged-only intervals are not counted as CPU.
Retaining approximately 6.7 KiB of additional parameter storage solely to
avoid re-isolation is not justified by that measured share. Future experiments
must show that retained refinement or narrower exact searches repay the
additional state and live-stack cost.

The final seven-row CPU profile moves apex/side analytic witness shares to
19.92%/16.27%, while unrepresentable-center chart work falls from 83.68% to
48.74%. These are changing shares of different total costs, not additional
timing reductions. Interior-rim materialization still accounts for 37.44%,
with exact triangle witness coordinates at 21.38%. The gap fixture spends
92.03% in analytic traversal, including 68.71% in the lateral generator fan;
it does not reach rim value mapping. This gives the next rejection experiment
a concrete owner rather than attributing the gap cost to the curved root solver.

The unchanged stationary quartic, value polynomial and root owners retain
their existing coefficient-height and resource proofs. The new bound uses the
existing widened query rather than narrowing to a new fixed-width type. The
rational witness branch replaces scratch-heavy search with the existing
bounded division owner. Dirty-stack tests retain a 1 MiB worker with 64 KiB of
live caller storage, including large independently rotated authored frames.
These tests supplement source-derived width/lifetime bounds; they do not
measure total JIT stack use.

## Correctness And Validation

The new independent regressions passed against the committed implementation
before optimization. They cover horizontal axial-first ordering through both
windings and all cyclic vertex orders; positive and negative half-raw even/odd
parity; cancellation of two nonzero radical terms; scalar extrema; exact
maximum depth; and identical minima on reciprocal chart boundaries.
Existing tests preserve one-raw miss/touch/overlap distinctions, generator
features, paired anchors, odd height, unrepresentable centers, overflowed world
witnesses, allocation guards and stack headroom. Both retained stages pass
471 focused Debug cone/cylinder/capsule-slab tests.

Fresh serial validation uses `UseLocalLsfStack=true` throughout. Both solutions
build in Release and ReleaseLean, including their netstandard2.1/net8.0 library
targets. The final Debug resource/arithmetic filter passes 794 FixedMathSharp
tests, and the matching Gravitas mesh-contact filter passes 25 tests.

| Owning test suite | Release | ReleaseLean |
| --- | ---: | ---: |
| FixedMathSharp core | 4,361 | 4,340 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4,537 | 4,476 |

All tests pass with zero skipped. Coverage uses the repository exclusions
unchanged. Each value below is both the covered count and the total count:
reachable line, branch and method coverage is exactly 100%, rather than a
rounded percentage. OpenCover and the rendered ReportGenerator summaries are
checked independently.

| Owner/configuration | OpenCover sequence points / branches / methods | Rendered lines / branches / methods |
| --- | ---: | ---: |
| FixedMathSharp Release | 53,062 / 12,312 / 3,953 | 53,062 / 12,312 / 3,953 |
| FixedMathSharp ReleaseLean | 53,155 / 12,312 / 3,949 | 53,155 / 12,312 / 3,949 |
| FixedMathSharp.Chronicler, both | 85 / 12 / 18 | 85 / 12 / 18 |
| Gravitas Release | 44,542 / 13,296 / 4,603 | 56,272 / 16,302 / 5,413 |
| Gravitas ReleaseLean | 44,540 / 13,296 / 4,602 | 56,270 / 16,302 / 5,412 |

FixedMathSharp core captures include its source FluentAssertions helper.
Chronicler adapter coverage is checked in its own module, keeping its partially
exercised core dependency separate from the independently complete core suite.
The complexity register matches the raw per-method measurements in both
configurations: analytic traversal 18, chart traversal 30 and analytic-coordinate
rounding 12, all fully covered.

Both DocFX builds pass with warnings as errors. API branding/resource and local
link checks pass. The fresh passing gate capture is
`artifacts/grv-benchmark-019/final-gates-20261006T050742764Z-64bdaaa410cf4e3ba6f06b83a63ddba2`,
with raw reports, rendered reports, test logs and `result.json`. The confirmed
cone/control captures match current source hashes and their actual private child
IL bodies. Independent source and evidence reviews have no remaining findings.

## Status At The End Of This Phase

At the end of this phase, GRV-Benchmark-019 remained open for the measured complete-contact cost. Interior
rim is still approximately 1.05 ms/query on this host; gap rejection is also
substantially more expensive than the former correct gap fixture. The next
investigation should measure exact witness searches and generator-fan work
in gap rejection before selecting another experiment. Reuse existing exact
owners and preserve all rounding, classification and tie contracts.
Representative frequency and complete-step capacity remain the separate
GRV-Benchmark-024 experiment. Neither a single-triangle microbenchmark nor
rare-feature assumptions establish game/server workload capacity.
