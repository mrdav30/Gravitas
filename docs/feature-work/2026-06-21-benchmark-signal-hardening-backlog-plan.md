# Benchmark Signal Hardening Backlog Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture benchmark-derived hardening signals that fall outside the active feature scope, then turn each signal into measured, deterministic runtime improvements or an explicit no-change decision.

**Architecture:** Treat benchmark concerns as an evidence pipeline: reproduce the signal, isolate the contributing runtime phase, add a guardrail test or benchmark row, then optimize only the proven source of cost. This plan is a living bucket for benchmark issues discovered while working other Gravitas hardening plans.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp, SwiftCollections, GridForge, Gravitas benchmark support.

---

**Date:** 2026-06-21
**Status:** Pre-alpha release blocker / evidence closure
**Owner:** Gravitas benchmark and runtime hardening

## Purpose

The continuous-collision benchmark cleanup produced useful evidence beyond the
scope of the active CCD implementation plan. The benchmark suite is now strong
enough to show cost shape, allocation behavior, and dimension-specific
asymmetry. This document preserves those signals so they can be resolved with
the same evidence bar as feature work, without derailing the current CCD plan.

This is not a catch-all wish list. Add a signal here only when it comes from a
measured benchmark, allocation guardrail, profiler trace, or repeated validation
run. Each workstream should either produce a runtime/test/docs improvement or
close with a written no-change decision explaining why the signal is expected.

## Current Signals

These numbers came from the 2026-06-21 short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke. Treat them as direction
finding, not release-grade performance budgets.

| Signal | Evidence | Initial Read | Priority |
| --- | --- | --- | --- |
| 3D full-runtime CCD allocation | Pure 3D full-runtime rows allocated about `172,032 B/op` at `256` bodies and `688,138 B/op` at `1024` bodies. Pure 2D full-runtime rows and CCD attribution rows were effectively allocation-clean. | The allocation is likely outside core CCD query/index/sweep math. Suspect reset, host-transform publish, partition refresh, collision-pair lifecycle, or another 3D full-runtime phase. | High |
| 3D shape-exact false-positive cost | `Pure3DFullRuntimeShapeExactFalsePositiveEvidence` was about `97.3 ms/op` at `1024` bodies, while the 2D equivalent was about `28.7 ms/op`. | The 3D exact-reduction path may be doing too much work per conservative false positive, or the scene may be intentionally expensive. Profile before adding more exact 3D reducers. | Medium |
| Pure 2D dynamic CCD candidate asymmetry | `Pure2DDynamicCandidateIndexAttributionEvidence` was about `4.31 ms/op` at `1024` bodies, compared to about `1.47 ms/op` for 3D. The 2D relative-sweep attribution row was also higher than 3D. | Pure 2D candidate gathering may have avoidable overhead, or the evidence scene may produce more 2D candidates than the comparable 3D scene. Normalize candidate counts before optimizing. | Medium |

## Guiding Rules

- Preserve deterministic replay before improving speed.
- Reproduce a signal with the same benchmark command before changing source.
- Add a focused allocation guardrail, correctness regression, or attribution
  benchmark before optimizing the suspected runtime path.
- Do not optimize against one short-run mean if candidate counts, hit counts, or
  allocation source have not been isolated.
- Keep instrumentation deterministic and benchmark/test-owned unless host-facing
  diagnostics are part of the accepted design.
- Prefer deleting unnecessary work over caching more state. If caching is needed,
  ownership, invalidation, and reset behavior must be explicit.
- Validate `Release` and `ReleaseLean` after runtime changes.

## Baseline Commands

Build the benchmark project before capturing evidence:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0
```

List the current evidence rows:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --list flat
```

Run the full evidence smoke locally:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --filter "*Evidence*" -j Short -i
```

After implementation, run:

```powershell
dotnet test Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration ReleaseLean
```

## Workstream 1: 3D Full-Runtime CCD Allocation RCA

**Problem**

The 3D full-runtime evidence rows allocate linearly with body count, while the
2D rows and CCD attribution rows are effectively allocation-free. Gravitas
should not accept hidden per-body managed allocation in repeated runtime CCD
paths unless the allocation is proven to come from benchmark-only setup.

**Why It Matters**

CCD is a hot-path feature for fast movers. Lockstep simulations need predictable
frame cost and should not introduce GC pressure that scales with body count.
Even if the source is not the CCD sweep math, the user experience is still a
runtime cost because the full-runtime benchmark models the way hosts advance
many moving bodies.

**Initial Scope**

- Pure 3D full-runtime CCD allocation in dense and sparse evidence scenes.
- Reset, transform-publish, `LateSimulate`, partition refresh, and pair lifecycle
  phases that are included by the full-runtime rows.
- Pure 2D rows remain comparison data, not the target of this workstream.

**Research Questions**

- Does the allocation reproduce in an xUnit allocation guardrail using
  `AllocationTestHelper.MeasureSteadyState`?
- Is the allocation caused by benchmark reset helpers, `StiffBody` transform
  publication, 3D partition refresh, collision-pair creation/culling, diagnostic
  hooks, or another late-simulate phase?
- Does the allocation happen only when bodies are reset every frame, only during
  `LateSimulate`, or only when both happen together?
- Is mixed-mode allocation explained by the same 3D participant count?

**Candidate Approach**

Start with a focused guardrail test that recreates the pure 3D evidence layout
outside BenchmarkDotNet. Split measurement into reset-only, late-simulate-only,
and reset-plus-late-simulate actions. If reset-only allocates, inspect
`StiffBody` setters and benchmark matter-agent transform publication. If
late-simulate-only allocates, inspect collision partition refresh and 3D pair
lifecycle. Keep any benchmark-only helpers outside runtime source unless the
runtime needs an actual reusable primitive.

**Tasks**

- [ ] Capture a fresh baseline for the affected rows using the
  `continuous-collision-evidence` command in this plan.
- [ ] Add or extend allocation tests under
  `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
  or a focused sibling file using `tests/Gravitas.Tests/Support/AllocationTestHelper.cs`.
- [ ] Reuse deterministic body placement from
  `tests/Gravitas.SharedBenchmarkSupport/ContinuousCollisionBenchmarkLayout.cs`
  so the test scene matches benchmark scale and ordering.
- [ ] Measure reset-only, `LateSimulate`-only, and reset-plus-`LateSimulate`
  variants before changing source.
- [ ] Fix the first confirmed allocation source in runtime or benchmark support.
- [ ] Keep the guardrail strict after warmup: expected measured allocation is
  `0` bytes for the recurring runtime action being protected.
- [ ] Re-run `Release`, `ReleaseLean`, and the affected evidence benchmark rows.

**Likely Files**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/CollisionHandling/Pairs/*`
- `src/Gravitas/Partitions/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Tests/Support/AllocationTestHelper.cs`
- `tests/Gravitas.SharedBenchmarkSupport/ContinuousCollisionBenchmarkLayout.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`

**Done Criteria**

- The allocation source is identified and either removed or documented as
  benchmark-only.
- A test or benchmark row prevents the same signal from becoming invisible.
- Full-runtime 3D allocation no longer scales linearly with body count unless a
  written no-change decision explains why the allocation is outside Gravitas
  runtime behavior.

## Workstream 2: 3D Shape-Exact False-Positive Cost

**Problem**

The 3D shape-exact false-positive evidence row is materially heavier than the
2D equivalent. This row is intentionally stressful, but it is also the clearest
warning that exact 3D reductions could become expensive if expanded without
profiling.

**Why It Matters**

Shape-exact CCD should reduce conservative false positives without turning
non-sphere 3D movers into a broad hidden cost. If exact reducers are too
expensive, users may experience worse performance precisely in dense scenes
where CCD exists to improve physical quality.

**Initial Scope**

- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`.
- 3D static-style CCD collection and exact candidate reduction.
- Conservative false-positive scenes where exact reduction rejects candidates.
- 2D shape-exact rows remain comparison data.

**Research Questions**

- How much time is spent gathering conservative candidates versus exact
  candidate reduction?
- Are repeated false positives caused by overly broad bounds, missing cheap
  rejection checks, or expected geometry complexity?
- Is the reversed swept-sphere reduction doing repeated work that can be reused
  safely inside one query?
- Which shape pairs dominate the row, and do those pairs represent realistic
  user-authored scenes?

**Candidate Approach**

Add benchmark-only attribution before changing runtime behavior. Split the
existing false-positive row into candidate-gather, exact-reduction, and
full-runtime variants if the existing helper boundaries allow it. Count accepted
hits and rejected conservative candidates so time can be interpreted alongside
work volume. Promote a runtime optimization only if it preserves false-negative
safety and improves a measured reducer phase.

**Tasks**

- [ ] Capture a fresh baseline for
  `Pure3DFullRuntimeShapeExactFalsePositiveEvidence` and
  `Pure2DFullRuntimeShapeExactFalsePositiveEvidence`.
- [ ] Add attribution rows to
  `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs` if
  the current row cannot explain candidate count, accepted hit count, and
  rejection count.
- [ ] Profile or inspect the exact 3D reduction path after attribution identifies
  the expensive phase.
- [ ] Add correctness tests before changing any exact-reduction logic, including
  a no-tunneling case and a near-miss false-positive rejection case.
- [ ] Prefer cheap deterministic rejection before heavier exact work if the
  profiler shows repeated unnecessary reducer calls.
- [ ] Re-run the false-positive rows and relevant CCD tests after the change.

**Likely Files**

- `src/Gravitas/CollisionHandling/Continuous/*`
- `src/Gravitas/Queries/GravitasQuery3DService.*.cs`
- `src/Gravitas/CollisionHandling/Detection/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Done Criteria**

- The row is explained by candidate volume, reducer cost, or a specific runtime
  hotspot.
- If optimized, no false-negative risk is introduced and exact 3D behavior
  remains deterministic.
- If not optimized, the no-change decision records why the cost is expected and
  when it should be revisited.

## Workstream 3: Pure 2D Dynamic CCD Candidate Asymmetry

**Problem**

The pure 2D dynamic candidate-index attribution row was slower than the
corresponding 3D row in the short evidence smoke, and the 2D relative-sweep row
was also higher than the 3D row. Pure planar CCD should not be assumed cheaper
until candidate counts and algorithmic work are normalized.

**Why It Matters**

Dynamic-vs-dynamic 2D CCD is likely to become important for fast top-down
games, projectiles, character controllers, and deterministic RTS-style
simulation. If 2D candidate gathering has avoidable overhead, it should be fixed
before richer pure 2D CCD behavior is layered on top.

**Initial Scope**

- Pure 2D dynamic candidate indexing.
- Pure 2D dynamic relative sweep attribution.
- Candidate count normalization against the 3D evidence layout.
- `DynamicCcdCandidateIndex` and 2D partition traversal behavior.

**Research Questions**

- Do the 2D and 3D evidence scenes produce comparable candidate counts per body?
- Is the 2D path spending more time in partition traversal, candidate-key
  ordering, duplicate suppression, shape checks, or body/collider state access?
- Does the 2D layout create denser X/Z overlap than the 3D layout despite having
  the same body count?
- Can 2D reuse a tighter data layout or cheaper candidate key without weakening
  deterministic ordering?

**Candidate Approach**

Normalize before optimizing. Add benchmark-only counters or focused tests that
compare 2D and 3D candidate counts for the same descriptor set. If 2D simply
has more candidate work, document that and adjust evidence scenes. If candidate
counts are comparable, inspect `DynamicCcdCandidateIndex`, 2D partition
membership refresh, and 2D relative sweep helpers for avoidable repeated work.

**Tasks**

- [ ] Capture fresh baselines for
  `Pure2DDynamicCandidateIndexAttributionEvidence`,
  `Pure3DDynamicCandidateIndexAttributionEvidence`,
  `Pure2DDynamicRelativeSweepAttributionEvidence`, and
  `Pure3DDynamicRelativeSweepAttributionEvidence`.
- [ ] Add benchmark attribution that reports or returns deterministic candidate
  totals for 2D and 3D using the same benchmark descriptor count.
- [ ] Add a focused test if a candidate-count or ordering invariant is missing.
- [ ] Inspect `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
  and the 2D partition traversal path once comparable work volume is confirmed.
- [ ] Optimize only the confirmed 2D overhead, preserving stable candidate
  ordering and duplicate suppression.
- [ ] Re-run the candidate-index and relative-sweep evidence rows.

**Likely Files**

- `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
- `src/Gravitas/Core/GravitasPhysics2DService.cs`
- `src/Gravitas/CollisionHandling/Detection/*2D*`
- `src/Gravitas/Queries/GravitasQuery2DService.cs`
- `tests/Gravitas.Tests/CollisionHandling/DynamicCcdCandidateIndexTests.cs`
- `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Done Criteria**

- The 2D/3D attribution gap is explained by candidate volume or a specific 2D
  hot path.
- Any optimization has correctness coverage for candidate ordering and
  duplicate suppression.
- Benchmark evidence shows whether the gap closed, remained expected, or moved
  to another phase.

## Workstream 4: Signal Bucket Maintenance

**Problem**

Benchmark work can reveal issues that are real but not aligned with the active
implementation plan. Without a consistent intake rule, those concerns either
interrupt focused work or disappear.

**Approach**

Use this document as the intake bucket for measured benchmark concerns. Each new
signal should record the benchmark command, date, affected row, measured value,
why it matters, and the smallest first isolation step. Promote the signal into a
dedicated plan only when it spans a subsystem or needs multi-week design work.

**Current Watch Items**

- Mixed full-runtime CCD rows were heavier than pure 2D or pure 3D rows at
  `1024` bodies. This is currently expected because mixed mode exercises both
  dimensions and the mixed broad phase. Revisit if the gap grows after the 3D
  allocation RCA or if mixed CCD becomes an immediate alpha target.
- Benchmark publishing, external baseline storage, CI gating, and host-visible
  CCD counters are tracked in
  [`2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md).

**Tasks**

- [ ] When a new measured concern appears, add it to `Current Signals` with
  evidence and priority.
- [ ] Add a workstream only when the first isolation step is known.
- [ ] Keep completed signal notes in this document until the corresponding
  source/test/docs change is merged or the no-change decision is written.
- [ ] Move broad platform work, such as CI publishing or cross-repo benchmark
  storage, into its own plan instead of mixing it with runtime RCA.

**Likely Files**

- `docs/feature-work/2026-06-21-benchmark-signal-hardening-backlog-plan.md`
- `tests/Gravitas.Benchmarks/README.md`
- `tests/Gravitas.Benchmarks/Core/*`
- `tests/Gravitas.Benchmarks/Support/*`

## Promotion Criteria

A signal can move from backlog to active implementation when it has:

- reproducible local evidence from a benchmark, allocation guardrail, or profiler
  trace.
- a suspected runtime phase narrow enough for focused tests.
- a deterministic correctness or ordering invariant if runtime behavior changes.
- an allocation and benchmark comparison before and after the change.
- updated docs or plan notes explaining the result.

## Current Recommendation

Start with Workstream 1 before optimizing the other signals. If the 3D
full-runtime allocation source is in transform publishing, partition refresh, or
pair lifecycle, it may also affect the shape-exact and mixed full-runtime rows.
After allocation is explained, revisit Workstream 2 and Workstream 3 with fresh
benchmark evidence so the remaining costs are easier to interpret.
