# Benchmark Signal Hardening Backlog

**Status:** Active
**Owner:** Gravitas benchmark and runtime hardening

## Purpose

This document captures benchmark-derived hardening signals that fall outside the
active feature plan. It is intentionally undated and long-lived: individual
entries carry their own discovery dates, evidence, status, and next isolation
step.

Use this backlog for measured performance, allocation, scaling, and benchmark
evidence concerns. Bugs or correctness risks that are not primarily benchmark
signals belong in [`issue-tracker.md`](issue-tracker.md). Broad feature or
architecture work should be promoted into its own dated plan and referenced
from this backlog.

## Intake Rules

- Add a signal only when it comes from a benchmark, allocation guardrail,
  profiler trace, or repeated validation run.
- Record the command, date, affected row or test, measured value, why it
  matters, and the smallest useful next isolation step.
- Keep benchmark-only instrumentation in tests or benchmark support unless the
  runtime needs a durable diagnostic API.
- Prefer a focused fix when the signal has a narrow cause.
- Promote to a dated feature-work plan when the signal spans multiple
  subsystems, requires API design, or needs staged implementation.
- Close entries only after a runtime/test/docs change lands or after a written
  no-change decision explains why the signal is expected.

## Baseline Commands

Build the benchmark project before capturing evidence:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0
```

List the continuous-collision evidence rows:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --list flat
```

Run the current continuous-collision evidence smoke:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --filter "*Evidence*" -j Short -i
```

After runtime changes, validate the package paths:

```powershell
dotnet test Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration ReleaseLean
```

## Active Signals

| Signal | Status | Priority | Tracking |
| --- | --- | --- | --- |
| 3D full-runtime CCD allocation | Open | High | This backlog |
| 3D shape-exact false-positive cost | Open | Medium | This backlog |
| Pure 2D dynamic CCD candidate asymmetry | Open | Medium | This backlog |

## Signal: 3D Full-Runtime CCD Allocation

**Discovered:** 2026-06-21

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported pure 3D
full-runtime rows allocating about `172,032 B/op` at `256` bodies and
`688,138 B/op` at `1024` bodies. Pure 2D full-runtime rows and CCD attribution
rows were effectively allocation-clean.

**Initial read:** The allocation is likely outside core CCD query/index/sweep
math. Suspect reset, host-transform publish, partition refresh,
collision-pair lifecycle, or another 3D full-runtime phase.

**Why it matters:** CCD is a hot-path feature for fast movers. Lockstep
simulations need predictable frame cost and should not introduce GC pressure
that scales with body count.

**Next isolation step:** Reproduce the allocation in a focused xUnit guardrail
using `AllocationTestHelper.MeasureSteadyState`, then split measurement into
reset-only, `LateSimulate`-only, and reset-plus-`LateSimulate` actions before
changing runtime source.

**Likely files:**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/CollisionHandling/Pairs/*`
- `src/Gravitas/Partitions/*`
- `tests/Gravitas.Tests/Support/AllocationTestHelper.cs`
- `tests/Gravitas.SharedBenchmarkSupport/ContinuousCollisionBenchmarkLayout.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`

**Closure criteria:** The allocation source is identified and either removed or
documented as benchmark-only. A test or benchmark row prevents the same signal
from becoming invisible.

## Signal: 3D Shape-Exact False-Positive Cost

**Discovered:** 2026-06-21

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported
`Pure3DFullRuntimeShapeExactFalsePositiveEvidence` at about `97.3 ms/op` for
`1024` bodies, while the 2D equivalent was about `28.7 ms/op`.

**Initial read:** The 3D exact-reduction path may be doing too much work per
conservative false positive, or the scene may be intentionally expensive.

**Why it matters:** Shape-exact CCD should reduce conservative false positives
without making non-sphere 3D movers a broad hidden cost in dense scenes.

**Next isolation step:** Add or use benchmark attribution that separates
candidate gathering, exact reduction, accepted hits, and rejected conservative
candidates before optimizing reducer logic.

**Likely files:**

- `src/Gravitas/CollisionHandling/Continuous/*`
- `src/Gravitas/Queries/GravitasQuery3DService.*.cs`
- `src/Gravitas/CollisionHandling/Detection/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Closure criteria:** The row is explained by candidate volume, reducer cost, or
a specific runtime hotspot. If optimized, correctness coverage proves no
false-negative risk was introduced.

## Signal: Pure 2D Dynamic CCD Candidate Asymmetry

**Discovered:** 2026-06-21

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported
`Pure2DDynamicCandidateIndexAttributionEvidence` at about `4.31 ms/op` for
`1024` bodies, compared to about `1.47 ms/op` for 3D. The 2D relative-sweep
attribution row was also higher than 3D.

**Initial read:** Pure 2D candidate gathering may have avoidable overhead, or
the evidence scene may produce more 2D candidates than the comparable 3D scene.

**Why it matters:** Dynamic-vs-dynamic 2D CCD is important for fast top-down
games, projectiles, character controllers, and deterministic RTS-style
simulation. Pure planar CCD should be understood before richer 2D CCD behavior
is layered on top.

**Next isolation step:** Normalize candidate counts between the 2D and 3D
evidence scenes before optimizing. If counts are comparable, inspect
`DynamicCcdCandidateIndex`, 2D partition traversal, and 2D relative sweep
helpers for repeated work.

**Likely files:**

- `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
- `src/Gravitas/Core/GravitasPhysics2DService.cs`
- `src/Gravitas/CollisionHandling/Detection/*2D*`
- `src/Gravitas/Queries/GravitasQuery2DService.cs`
- `tests/Gravitas.Tests/CollisionHandling/DynamicCcdCandidateIndexTests.cs`
- `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`

**Closure criteria:** The 2D/3D attribution gap is explained by candidate volume
or a specific 2D hot path. Any optimization preserves stable candidate ordering
and duplicate suppression.

## Watch Items

- Mixed full-runtime CCD rows were heavier than pure 2D or pure 3D rows at
  `1024` bodies. This is currently expected because mixed mode exercises both
  dimensions and the mixed broad phase. Revisit if the gap grows after the 3D
  allocation RCA or if mixed CCD becomes an immediate alpha target.
- Benchmark publishing, external baseline storage, CI gating, and host-visible
  CCD counters are tracked in
  [`2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md).

## Promotion Criteria

Promote a signal from this backlog into a dedicated dated plan when it has:

- reproducible evidence and a suspected runtime phase.
- enough subsystem breadth that a single focused patch would be misleading.
- API, architecture, or multi-workstream design decisions.
- correctness or ordering invariants that need staged implementation.
- benchmark and allocation evidence that should move with the new plan.

## Current Recommendation

Start with the 3D full-runtime CCD allocation signal before optimizing the other
signals. If the allocation source is in transform publishing, partition refresh,
or pair lifecycle, it may also affect the shape-exact and mixed full-runtime
rows. After allocation is explained, revisit the remaining benchmark signals
with fresh evidence so their costs are easier to interpret.
