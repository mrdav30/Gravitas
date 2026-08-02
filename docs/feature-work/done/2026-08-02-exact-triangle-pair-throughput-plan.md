# Exact Triangle-Pair Throughput Optimization

**Created:** 2026-08-02  
**Status:** Complete - focused optimization retained; broader signal narrowed  
**Signal:** Exact triangle-pair contacts regress dense concave-mesh throughput

## Goal

Reduce the measured cost of dense concave mesh/mesh contacts without changing
the exact full-domain triangle relation, deterministic axis and tie ordering,
canonical anchors, clamped-depth behavior, or warmed allocation contract.

Success is evidence-gated. Retain only changes that produce a repeatable gain
on the unchanged benchmark rows and preserve 100% reachable line, branch, and
method coverage in every modified repository. Matching the deleted saturating
scalar implementation is not a correctness-compatible target.

## Baseline And Root Cause

The refreshed 64-pair Short in-process medians are:

| Row | 2026-08-02 median |
| --- | ---: |
| Ordinary convex mesh/mesh | `4.836 ms` |
| Concave mesh/mesh | `70.351 ms` |
| Dense concave mesh/mesh | `405.224 ms` |
| Contact-heavy concave mesh/mesh | `556.972 ms` |
| Closed dense mesh/mesh | `2.566 s` |

These results are materially unchanged from the 2026-08-01 closure. A sampled
profile of the dense row centers the measured collision path in
`FixedTriangle.TryGetContact`. Projection and normalized-depth arithmetic
dominate; BVH traversal, manifold ownership, rational-basis construction,
edge preparation, and exact anchor construction do not.

Within the exact relation, the largest shared costs are
`WideArithmetic.MultiplySigned576`, `MultiplySigned320`, and the projection
loops that call them. The common `Signed576` by `Signed192` path already skips
inactive magnitude words, but raw `Fixed64` coordinates and rational-basis
denominators still pay generic narrowing and bit-length dispatch even though
their factor is a proven signed 64-bit value.

## Design

1. Specialize the existing FixedMathSharp wide multiply owner for signed
   one-limb factors. Keep the same exact result type and full-width fallback;
   do not introduce a second geometry answer path.
2. Prove the specialization across zero, both signs, `long.MinValue`, carry
   propagation, narrow and full-width multiplicands, and the established
   finite-axis fit boundary. Preserve the generic path for wider factors.
3. Rerun the focused FixedMathSharp arithmetic evidence and unchanged Gravitas
   mesh/mesh rows. Revert the specialization if the improvement is not
   repeatable or if another affected wide workload regresses materially.
4. Only if a material signal remains, profile again before considering a
   prepared rigid-triangle representation. Preparation is not part of this
   first change because the current trace gives it a small upper bound and the
   existing relation recomputation is simpler and safer.

## Outcome

FixedMathSharp now multiplies `Signed576` values by proven signed one-word
factors directly. Triangle projection passes raw `Fixed64` coordinate words to
that shared owner instead of widening them to `Signed192` and paying generic
bit-length dispatch. The result remains the same exact nine-word two's-
complement product; no public API or competing geometry path was added.

The unchanged 64-pair Short in-process medians are:

| Row | Refreshed baseline | Retained change | Confirmation |
| --- | ---: | ---: | ---: |
| Ordinary convex mesh/mesh | `4.836 ms` | `4.933 ms` | control only |
| Concave mesh/mesh | `70.351 ms` | `60.213 ms` | `59.761 ms` |
| Dense concave mesh/mesh | `405.224 ms` | `342.682 ms` | `343.474 ms` |
| Contact-heavy concave mesh/mesh | `556.972 ms` | `480.926 ms` | `480.773 ms` |
| Closed dense mesh/mesh | `2.566 s` | `2.170 s` | `2.155 s` |

The four affected rows improved by approximately `13.7-15.4%`, and the
independent confirmation reproduced the gain. The direct FixedMathSharp
`TrianglePairPrimary` row improved from the prior `64.221 us` closure to
`54.33 us`, or `15.4%`, with `0 B` reported.

The post-change workload trace retains the same stack shape:

| Role | Observed owner |
| --- | --- |
| Exact relation authority | `FixedTriangle.TryGetContact` |
| Dominant phase | `WideTriangleRelations.TryKeepPairAxis` |
| Dominant inner arithmetic | `MultiplySigned576`, `MultiplySigned320`, `GetMagnitudeBitLength` |
| Small contributors | BVH traversal, manifold ownership, triangle preparation |

At this phase boundary the broader signal remained active because the exact
concave rows were still materially slower than the deleted saturating scalar
baseline. The final bounded
[`Experimental Exact Triangle-Pair Throughput Pass`](2026-08-02-experimental-triangle-pair-throughput-plan.md)
found no additional local change worth retaining and reclassified the remaining
cost as experimental capacity guidance.

## Rejected Alternatives

- Restore scalar SAT or add a narrowed scalar prefilter: faster but can
  disagree with the exact full-domain authority.
- Cache prepared data across frames or mutate `PhysicsMesh` ownership: adds
  invalidation, memory, and lifecycle complexity before profiling justifies it.
- Rewrite all magnitude multiplication around active spans: that optimization
  already exists and does not address the measured dispatch overhead.
- Hoist a common quaternion denominator: no repeatable direct gain and longer
  signatures.
- Prepare the second triangle's edge data eagerly or lazily: a small direct-row
  movement did not survive the unchanged Gravitas rows, so both experiments
  were reverted.

## Work Plan

- [x] Reproduce the unchanged benchmark signal.
- [x] Capture and inspect a dense-row sampled profile.
- [x] Add focused one-limb multiplication correctness and coverage cases.
- [x] Implement the smallest shared specialization in FixedMathSharp.
- [x] Run FixedMathSharp focused tests, full Release tests, and 100% coverage.
- [x] Rerun identical Gravitas mesh/mesh benchmarks and warmed allocation gates.
- [x] Run Gravitas full Release and ReleaseLean validation plus 100% coverage.
- [x] Request independent review, update the benchmark backlog, and close or
      narrow this plan according to the evidence.

## Closure Gates

- FixedMathSharp: `2,653` Release and `2,632` ReleaseLean tests pass. Coverage
  remains `47,137/47,137` lines, `8,706/8,706` branches, and `3,421/3,421`
  methods. Standard and Lean packages build warning-free for `net8.0` and
  `netstandard2.1`.
- Gravitas: `3,925` Release and `3,870` ReleaseLean tests pass. Coverage remains
  `43,911/43,911` lines, `12,845/12,845` branches, and `4,510/4,510` methods.
  Standard and Lean packages build warning-free for both target frameworks.
- All `18` focused triangle/concave/allocation regressions pass. Direct warmed
  allocation guards remain the authority and report `0 B`; the small, variable
  BenchmarkDotNet allocation readings are not treated as runtime allocations.
- Independent arithmetic and closure reviews found no blocking issue. The
  arithmetic review prompted reuse of `Fixed64.AbsToUInt64(...)` and an exact
  `Signed576.MinValue` oracle case.

## Preserved Evidence

- Benchmark artifacts:
  `artifacts/benchmarks/2026-08-02-triangle-pair-current-short`,
  `artifacts/benchmarks/2026-08-02-triangle-pair-signed64-multiply`, and
  `artifacts/benchmarks/2026-08-02-triangle-pair-signed64-multiply-confirmation`
- Baseline and post-change workload traces:
  `TestResults/traces/2026-08-02-dense-triangle-pair-hot.nettrace` and
  `TestResults/traces/2026-08-02-dense-triangle-pair-signed64-workload.nettrace`
