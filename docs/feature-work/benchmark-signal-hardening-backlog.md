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
| 3D shape-exact false-positive cost | Open | Medium | This backlog |
| 3D dynamic shape-exact BDN allocation signal | Open | Medium | This backlog |
| Pure 2D dynamic CCD candidate asymmetry | Open | Medium | This backlog |
| SwiftCollections sort hot-path allocation | Mitigated in Gravitas, lower-stack open | Medium | This backlog |
| Mixed mesh finite-slab triangle scaling signal | Open | Medium | This backlog |

### Signal: SwiftCollections Sort Hot-Path Allocation

**Discovered:** 2026-06-22

**Evidence:** During the post-SwiftCollections v5.1.0 Workstream 3 cleanup,
replacing Gravitas' local sort helpers with package
`SwiftList<T>.SortInPlace(...)` and `SwiftSparseSet.CopySortedKeysTo(...)`
caused the Release allocation guardrails to fail. The full suite reported
recurring allocations in 3D CCD, pure 2D CCD, pure 2D broad phase, and 2D
query tests. The isolated
`Physics2DQueryTests.RaycastAll_ShouldNotAllocateAfterWarmup` test reproduced
the issue at `128 B` after warmup.

**Mitigation:** Gravitas now centralizes measured hot-path ordering through
`SwiftListSortUtility`, a reusable allocation-free heap sort over caller-owned
`SwiftList<T>` buffers. Partition single-source copies use
`SwiftSparseSet.CopyKeysTo(...)` followed by the same no-allocation sorter;
merged buckets append and sort once. The package sort APIs remain acceptable
for non-simulation setup paths such as lifecycle hook registration.

**Why it matters:** `SwiftCollections` is the lower-stack collection layer for
LSF. Its scratch-buffer sort APIs should be safe for deterministic physics hot
paths so consumers do not need local workarounds.

**Next isolation step:** Fix and measure `SwiftList<T>.SortInPlace(...)` and
`CopySortedKeysTo(...)` in SwiftCollections so they avoid recurring managed
allocation, then replace `SwiftListSortUtility` usages in Gravitas and rerun
the Release and ReleaseLean allocation guardrails.

**Likely files:**

- `../SwiftCollections/src/SwiftCollections/Collection/SwiftList.cs`
- `../SwiftCollections/src/SwiftCollections/Collection/SwiftSparseSet.cs`
- `src/Gravitas/Support/SwiftListSortUtility.cs`
- `src/Gravitas/Core/GravitasCollisionService.cs`
- `src/Gravitas/Core/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/GravitasCollision2DService.cs`
- `src/Gravitas/Core/GravitasMixedCollisionService.cs`
- `src/Gravitas/Partitions/*Partition*.cs`

**Closure criteria:** SwiftCollections provides no-recurring-allocation sort
and sorted-key copy APIs, Gravitas removes `SwiftListSortUtility`, and the
same allocation guardrails pass under Release and ReleaseLean.

### Signal: Mixed Mesh Finite-Slab Triangle Scaling Signal

**Discovered:** 2026-06-23

**Evidence:** During mixed finite-slab reducer close-out review,
`MixedQueryBenchmarks` covered mesh target scaling mostly through collider
candidate count. The mesh fixtures were tiny triangle sets, so dense mesh
triangle candidate scanning, triangle clipping, and per-triangle reducer cost
could regress without a dedicated row making that cost visible.

**Update 2026-06-23:** The mixed mesh target reducer no longer sorts triangle
candidate buffers before scanning. It now preserves deterministic lower
authored triangle-index tie-breaks by tracking the best triangle index directly,
removing the avoidable per-mesh `O(k log k)` step.

**Update 2026-06-23:** A short in-process `mixed-query --filter
"*MeshTargets*" -j Short -i` smoke completed successfully. MemoryDiagnoser
reported no managed allocation for some mesh rows, but tiny per-op values on
some candidate-count and 1024-collider rows (`1 B/op` to `20 B/op`). Treat this
as additional evidence that a focused triangle-level allocation/scaling
guardrail is useful before treating the mixed mesh reducer path as closed.

**Why it matters:** Mesh-backed mixed queries are attractive for level
geometry. Collider-count benchmarks do not necessarily expose triangle-level
cost, especially when one mesh collider can own many candidate triangles.

**Next isolation step:** Add a mixed query benchmark with high-triangle mesh
targets and false-positive-heavy slabs. If runtime diagnostics need to explain
the row, expose a benchmark-only or durable mesh-triangle candidate counter
without adding disabled-path cost.

**Likely files:**

- `src/Gravitas/Queries/GravitasQueryMixedService.cs`
- `tests/Gravitas.Benchmarks/Queries/MixedQueryBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs`

**Closure criteria:** Benchmarks expose mixed mesh finite-slab cost by triangle
candidate volume, not only by collider count. Any future optimization preserves
owner identity, lower triangle-index tie-breaks, and zero managed allocation
after warmup.

### Signal: 3D Shape-Exact False-Positive Cost

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

### Signal: 3D Dynamic Shape-Exact BDN Allocation Signal

**Discovered:** 2026-06-23

**Evidence:** The short in-process
`dynamic-ccd-scaling --filter "*DynamicShapeExact*" -j Short -i` smoke reported
`SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8` allocation scaling with
body count: about `43,008 B/op` at `64` bodies and `172,110 B/op` at `256`
bodies. The matching 2D rows reported only `42 B/op` runner noise.

**Counter-evidence:** The focused xUnit guard
`ContinuousMode_DynamicRelativeShapeExactPath_ShouldNotAllocateAfterWarmup`
passes with `0` allocated bytes after warmup for the same thin-cuboid dynamic
relative false-positive shape family. This suggests the BDN signal may come
from batched fixture reset, first-use candidate/pair growth, or another
full-context phase rather than the exact reducer call itself.

**Why it matters:** Dynamic 3D CCD is an alpha hot path. A scaling BDN
allocation signal should be either explained as benchmark-only setup or removed
from the runtime path before release confidence.

**Next isolation step:** Add an allocation guard or benchmark attribution that
splits the `DynamicShapeExact3D` fixture into reset, dynamic candidate query,
exact reducer validation, body `LateSimulate`, pair cleanup, and sum-return
work. Compare a warmed multi-body xUnit guard against the BDN row to identify
whether MemoryDiagnoser is seeing steady-state runtime allocation or benchmark
fixture churn.

**Likely files:**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Queries/ConvexSweepQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkFixture.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Closure criteria:** The multi-body BDN allocation is traced to benchmark-only
setup and documented, or a runtime/benchmark support fix removes it while the
focused xUnit allocation guard remains green.

### Signal: Pure 2D Dynamic CCD Candidate Asymmetry

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

## Closed Signals

| Signal | Status | Closed | Resolution |
| --- | --- | --- | --- |
| 3D full-runtime CCD allocation | Closed | 2026-06-23 | GridForge allocation-free line tracing plus Gravitas 3D raycast adoption |
| Grounding raycast probe allocation | Closed | 2026-06-23 | Same raycast trace fix removed automatic ray-grounding allocation |

### Signal: 3D Full-Runtime CCD Allocation

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported pure 3D
full-runtime rows allocating about `172,032 B/op` at `256` bodies and
`688,138 B/op` at `1024` bodies. Pure 2D full-runtime rows and CCD attribution
rows were effectively allocation-clean.

**Update 2026-06-21:** Discrete response Workstream 3 reproduced the related
steady-state CCD guardrail failures in xUnit after the 3D full-step phase order
started exercising collision distribution during measured CCD frames. Root cause
was comparer-based `Array.Sort` through package sorting in the collision
distribution/island hot path. Gravitas now uses a centralized allocation-free
runtime sort helper for active partitions, island buffers, and per-partition
collider ID copies.
The focused Release allocation guardrails for 3D substep, shape-exact
translational, and rotational CCD now pass under full `Simulate` +
`LateSimulate` measurement. `simulation-allocation` smoke also reported `0 B/op`
for `CollisionPartitionDistributionOnly` and
`ActivePairProcessingLateSimulate`.

**Initial read:** The allocation is likely outside core CCD query/index/sweep
math. Suspect reset, host-transform publish, partition refresh,
collision-pair lifecycle, or another 3D full-runtime phase.

**Why it matters:** CCD is a hot-path feature for fast movers. Lockstep
simulations need predictable frame cost and should not introduce GC pressure
that scales with body count.

**Completed isolation:** The original rows were rerun after the Workstream 3
sort fix. Temporary attribution rows split reset, force setup, and late
simulation enough to identify the moving-frame raycast grounding path.

**RCA 2026-06-23:** The remaining allocation was not CCD-specific. The
full-runtime 3D CCD evidence bodies moved each frame with automatic ray
grounding enabled, and `StiffBody` grounding called `Query3D.RaycastAll`.
The raycast service still used GridForge's enumerable `GridTracer.TraceLine`
path, which allocated iterator/mapping state per ray. Reset, force setup, and
non-moving late simulation attribution were allocation-clean.

**Resolution 2026-06-23:** GridForge now exposes caller-owned
`GridTracer.TraceLineInto(...)` overloads backed by `GridTraceScratch`.
`GravitasQuery3DService` uses that allocation-free trace buffer for closest-hit
and all-hit raycasts. A focused 3D `RaycastAll` allocation guard now protects
the path.

**Validation 2026-06-23:** Re-running the original
`continuous-collision-evidence --filter "*Pure3DFullRuntime*DynamicCcdEvidence*"
-j Short -i` smoke reduced:

- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `256` bodies:
  `168 KB/op` -> `0 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `256` bodies:
  `168.01 KB/op` -> `15 B/op`.
- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `1024` bodies:
  `672.01 KB/op` -> `15 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `1024` bodies:
  `672.01 KB/op` -> `15 B/op`.

The remaining `15 B/op` values were not reproduced by the focused xUnit
allocation guard and are treated as BenchmarkDotNet in-process measurement
noise unless a future guardrail reproduces them.

**Likely files:**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/CollisionHandling/Pairs/*`
- `src/Gravitas/Partitions/*`
- `tests/Gravitas.Tests/Support/AllocationTestHelper.cs`
- `tests/Gravitas.SharedBenchmarkSupport/ContinuousCollisionBenchmarkLayout.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`

**Closure criteria:** Met. The allocation source was removed, the original
benchmark rows were rerun, and a 3D `RaycastAll` allocation guard protects the
runtime path.

### Signal: Grounding Raycast Probe Allocation

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** During Discrete Response Workstream 3 verification, the Release
`simulation-allocation` BenchmarkDotNet smoke reported
`GroundingRaycastProbeOnly` at about `181.8 us` and `43,008 B/op` for
`64` colliders. The same run reported no managed allocation for
`StiffBodyLateSimulateOnly`, `GroundingSweptSphereProbeOnly`,
`CollisionPartitionDistributionOnly`, and
`ActivePairProcessingLateSimulate`.

**Initial read:** This appears separate from the discrete island work and the
collision distribution sort RCA. It likely belongs to the raycast-backed ground
probe or one of the query result/candidate paths used by that benchmark row.

**Why it matters:** Automatic raycast grounding is a recurring body hot path.
If this allocation is repeatable outside BenchmarkDotNet noise, grounded 3D
bodies can create avoidable GC pressure.

**Completed isolation:** The focused 3D raycast allocation guard and grounding
benchmark confirmed the automatic ray-grounding allocation came from the shared
raycast trace path.

**RCA 2026-06-23:** This was the same root cause as the remaining 3D
full-runtime CCD allocation: automatic ray grounding used `Query3D.RaycastAll`,
which depended on the enumerable GridForge line-trace path.

**Resolution 2026-06-23:** Gravitas 3D raycasts now use GridForge's
caller-owned `TraceLineInto(...)` path. The grounding row no longer allocates
after warmup.

**Validation 2026-06-23:** Re-running
`simulation-allocation --filter "*Grounding*" -j Short -i` reported
`GroundingRaycastProbeOnly` at `164.9 us` and `0 B/op` for `64` colliders.
`GroundingSweptSphereProbeOnly` also remained allocation-clean.

**Likely files:**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Raycast.cs`
- `tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs`
- `tests/Gravitas.Tests/Core/StiffBodyGroundingTests.cs`

**Closure criteria:** Met. The runtime allocation was eliminated and the 3D
raycast path now has a focused xUnit allocation guard.

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

With the shared 3D raycast/grounding allocation root cause closed, revisit the
shape-exact and pure 2D dynamic CCD cost signals with fresh evidence. Capture
candidate counts before optimizing timings so reducer cost, candidate volume,
and benchmark fixture behavior do not get conflated.
