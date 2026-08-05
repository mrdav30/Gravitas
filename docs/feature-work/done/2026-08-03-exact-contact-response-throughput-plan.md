# Exact 3D Contact Response Throughput Hardening

**Created:** 2026-08-03  
**Status:** Completed  
**Signal:** Exact 3D contact response adds measurable ordinary-domain cost

## Goal

Recover ordinary-domain 3D contact-response throughput without weakening exact
contact anchors, paired atomic application, deterministic impulse ordering,
full-domain fallback behavior, or the zero-allocation runtime contract.

Retain only changes that remove demonstrated work. A timing-only optimization
should normally improve its affected direct and downstream rows by at least `5%`
repeatably. Smaller changes may remain only when they delete meaningful code or
provide an independently useful policy-neutral specialization without regressing
sibling rows.

## Preserved Evidence

The original signal used three-iteration ShortRun samples that crossed the
TieredPGO transition. A matched steady-state run now compares the current source
against the parent of the exact-contact-response implementation with identical
lower-stack binaries and the same BenchmarkDotNet job:

- `InvocationCount=1`
- `IterationCount=20`
- `LaunchCount=1`
- `WarmupCount=100`
- prepared 16- and 64-pair contact matrices

Current response is slower in `23/24` cells. The median gap is `17.6%`; all rows
remain at `0 B/op`. At 64 pairs the representative gaps are:

| Contact shape | Pre-hardening |    Current |    Delta |
| ------------- | ------------: | ---------: | -------: |
| Single        |    `566.5 us` | `736.1 us` | `+29.9%` |
| Face manifold |    `1.826 ms` | `2.098 ms` | `+14.9%` |
| Resting face  |    `1.675 ms` | `1.851 ms` | `+10.5%` |
| Cylinder      |    `375.7 us` | `448.3 us` | `+19.3%` |
| Mesh          |    `1.594 ms` | `1.801 ms` | `+13.0%` |
| Compound part |    `375.3 us` | `433.5 us` | `+15.5%` |

Artifacts:

- `artifacts/benchmarks/2026-08-03-contact-response-pre-hardening-warmed-baseline`
- `artifacts/benchmarks/2026-08-03-contact-response-warmed-baseline`

## Root Cause

A sampled Release trace of the ordinary 64-pair single-contact path attributes
roughly `44%` inclusive time to `WidePointAnchor3d.TryGetRelativeOffset(...)`.
Contact response currently sends each contact anchor and its owning body's
center-of-mass anchor through the general two-frame rational reducer even though
ordinary contacts usually place both anchors in the same rigid frame.

The general reducer constructs two rational quaternion bases, evaluates six wide
rotated projections, forms three wide ratios, and rounds all three final
coordinates. That work is required for unrelated frames and for local
differences that cannot be represented before rotation, but it is redundant when
both anchors share a frame and their exact local difference is compact.

The normal solver and exact ratio machinery remain visible in the trace, but
they are not the first optimization target while anchor reduction dominates the
ordinary path.

Trace artifact:
`artifacts/benchmarks/2026-08-03-contact-response-single-current.nettrace`.

## Approved Design

Specialize the existing FixedMathSharp point-anchor owner rather than adding
physics caches or a Gravitas-local lever representation.

The retained specialization covers the two ordinary frame relationships proven
by the profile and downstream contact fixtures:

1. Identical origins and rotations form the complete representable local
   difference before identity return or one final rotation.
2. Independently translated identity frames form the complete world difference
   with existing exact add/subtract operations.
3. Non-unit identity-frame scaling uses existing fused scalar multiplication so
   each final coordinate rounds once.
4. Any failed compact proof, non-identity independent frame, non-unit rotated
   scale, exact residual, or local translation retains the existing wide
   rational reducer.

This is a policy-neutral FixedMathSharp optimization of an existing public
contract. It adds no API, cache, new arithmetic owner, approximate gate, or
second semantic answer. Exact local residuals and independent local translations
continue through the existing exact reducer.

## Phased Work Plan

Every phase must retain 100% reachable line, branch, and method coverage in each
touched repository. Tests should protect numerical behavior rather than API
shape, and newly unreachable or superseded code should be deleted before a phase
closes.

### Phase 0: Evidence And Design

- [x] Replace the unreliable ShortRun comparison with a matched steady-state
      historical/current matrix.
- [x] Confirm the residual timing gap and zero-allocation behavior.
- [x] Capture a sampled Release trace and identify the dominant owner.
- [x] Audit the anchor and response call graph.
- [x] Approve the same-frame FixedPointAnchor specialization and exact fallback.

### Phase 1: Same-Frame Point-Anchor Specialization

- [x] Add focused tests for identity, rotated, displaced, scaled, cancellation,
      raw-unit round-to-even, final overflow, and
      unrepresentable-local-but-representable-rotated cases.
- [x] Add direct benchmark rows for common same-frame identity and rotated
      offsets without removing the existing general-frame rows.
- [x] Implement the minimum specialization in the existing point-anchor owner.
- [x] Preserve the exact two-frame reducer as the fallback for every case that
      cannot prove the compact local reduction.
- [x] Run FixedMathSharp Release, ReleaseLean, allocation, and 100% coverage
      gates.
- [x] Measure the direct common-frame rows twice and retain only a repeatable
      result.

### Phase 2: Gravitas Response Confirmation

- [x] Rebuild Gravitas through the local FixedMathSharp link.
- [x] Rerun the complete 16/64-pair response matrix with enough warmup to reach
      steady-state TieredPGO.
- [x] Confirm `0 B/op`, deterministic tests, and 100% Gravitas coverage.
- [x] Compare every family against the preserved current baseline.
- [x] Reprofile only if a repeatable `>=5%` residual remains. No such residual
      remained, so no second trace was collected.

### Phase 3: Evidence-Gated Residual Optimization

- [x] If required, decompose the remaining cost among contact construction,
      normal solve, atomic preflight/application, friction, and warm-start
      storage. Not required after Phase 2 cleared every 64-pair family.
- [x] Reuse or hoist existing arithmetic proofs instead of adding parallel
      kernels, response caches, or one-line forwarding owners.
- [x] Preserve exact fallback, paired atomicity, diagnostics, and contact order.
- [x] Delete any experiment that does not clear the retention gate. No solver
      experiment was started because the residual gate did not open.
- [x] Re-run both repositories' tests, coverage, allocation, and affected
      benchmark rows after every retained change.

### Phase 4: Documentation And Closure

- [x] Record final direct and downstream benchmark evidence.
- [x] Update the benchmark-signal backlog and feature-work overview.
- [x] Review the full diff for duplicate arithmetic, zombie branches, public
      surface growth, and lower-stack ownership violations.
- [x] Move this plan to `docs/feature-work/done` only after all release gates
      pass and the signal is either closed or explicitly evidence-deferred.

## Final Evidence

The direct auto-invocation benchmark uses three launches, 20 warmups, and 30
measured iterations. Against the matching pre-change artifact:

| Point-anchor relation              |   Baseline |       Final |     Delta | Allocated |
| ---------------------------------- | ---------: | ----------: | --------: | --------: |
| Same identity frame                | `1.984 us` |  `82.00 ns` |  `-95.9%` |     `0 B` |
| Same rotated frame                 | `2.118 us` | `826.36 ns` |  `-61.0%` |     `0 B` |
| Identity frames, different origins |        n/a |  `78.65 ns` | new guard |     `0 B` |

The unchanged 24-row Gravitas matrix was repeated twice with 100 warmups and 20
measured iterations. The confirmation differs from the first optimized matrix by
only `0.7%` median. Across all 24 rows it is `46.4%` faster than the preserved
exact-response baseline and `36.0%` faster than the older compact pre-hardening
implementation. Representative 64-pair confirmation rows are:

| Contact shape | Pre-hardening | Exact baseline |       Final | Delta vs pre-hardening |
| ------------- | ------------: | -------------: | ----------: | ---------------------: |
| Single        |    `566.5 us` |     `736.1 us` | `497.91 us` |               `-12.1%` |
| Face manifold |    `1.826 ms` |     `2.098 ms` |  `1.123 ms` |               `-38.5%` |
| Resting face  |    `1.675 ms` |     `1.851 ms` | `839.82 us` |               `-49.9%` |
| Cylinder      |    `375.7 us` |     `448.3 us` | `235.62 us` |               `-37.3%` |
| Mesh          |    `1.594 ms` |     `1.801 ms` | `825.91 us` |               `-48.2%` |
| Compound part |    `375.3 us` |     `433.5 us` | `233.65 us` |               `-37.7%` |

Every direct and downstream row reports `0 B/op`. The existing generic point
anchor and exact residual paths remain unchanged fallbacks. Coverage review
deleted two superseded scale-zero branches rather than manufacturing tests for
paths made unreachable by the public zero-scale result.

Independent review found no production defect. Its only test-quality finding was
closed with raw-unit round-to-even coverage and an exact expected vector for the
unrepresentable-local wide fallback.

Validation:

- FixedMathSharp: 2,676 Release and 2,655 ReleaseLean tests, plus eight
  Chronicler tests in each configuration.
- FixedMathSharp coverage: 47,452/47,452 lines, 8,742/8,742 branches, and
  3,427/3,427 methods.
- Gravitas: 3,926 Release and 3,871 ReleaseLean tests through the local stack.
- Gravitas coverage: 55,839/55,839 lines, 15,829/15,829 branches, and
  5,320/5,320 methods.

Final artifacts:

- `../FixedMathSharp/artifacts/benchmarks/2026-08-03-same-frame-anchor-baseline`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-03-same-frame-anchor-confirmation-2`
- `artifacts/benchmarks/2026-08-03-contact-response-pre-hardening-warmed-baseline`
- `artifacts/benchmarks/2026-08-03-contact-response-warmed-baseline`
- `artifacts/benchmarks/2026-08-03-contact-response-identity-frame-first-pass`
- `artifacts/benchmarks/2026-08-03-contact-response-identity-frame-confirmation`
- `../FixedMathSharp/tests/FixedMathSharp.Tests/TestResults/coverage-analysis-contact-response-20260803-review-final`
- `tests/Gravitas.Tests/TestResults/coverage-analysis-contact-response-20260803-final`

## Release Gates

- FixedMathSharp Release and ReleaseLean tests pass.
- Gravitas Release and ReleaseLean tests pass through the coordinated local
  link.
- Both repositories retain 100% reachable line, branch, and method coverage.
- Focused benchmark rows report `0 B/op`.
- Same inputs preserve identical point-anchor and contact-response results.
- No new public API, physics-specific upstream type, general-purpose cache, or
  duplicate wide-arithmetic implementation is introduced.
