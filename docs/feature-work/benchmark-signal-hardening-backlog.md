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
| SwiftCollections sort hot-path allocation | Mitigated in Gravitas, lower-stack open | Medium | This backlog |

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

## Closed Signals

| Signal | Status | Closed | Resolution |
| --- | --- | --- | --- |
| Mixed mesh finite-slab triangle scaling signal | Closed | 2026-06-24 | Mixed and pure 3D query services now expose mesh-triangle candidate counts, dedicated triangle-volume benchmarks cover dense and false-positive mesh targets, and pure 3D convex-source mesh sweeps use ordered lower-bound triangle candidates |
| Pure 2D dynamic CCD candidate asymmetry | Closed | 2026-06-23 | 2D now uses a planar candidate index, skips mixed CCD indexing outside mixed mode, and benchmark resets use 2D reset parity |
| 3D shape-exact false-positive cost | Closed | 2026-06-23 | Static CCD now uses exact-source sweeps for non-sphere convex movers before conservative sphere fallback refinement |
| 3D dynamic shape-exact BDN allocation signal | Closed | 2026-06-23 | Shared exact-sweep bounds prefilters removed the scaling allocation/time signal from 3D dynamic false-positive rows |
| 3D full-runtime CCD allocation | Closed | 2026-06-23 | GridForge allocation-free line tracing plus Gravitas 3D raycast adoption |
| Grounding raycast probe allocation | Closed | 2026-06-23 | Same raycast trace fix removed automatic ray-grounding allocation |

### Signal: Mixed Mesh Finite-Slab Triangle Scaling Signal

**Discovered:** 2026-06-23

**Status:** Closed 2026-06-24

**Evidence:** During mixed finite-slab reducer close-out review,
`MixedQueryBenchmarks` covered mesh target scaling mostly through collider
candidate count. The mesh fixtures were tiny triangle sets, so dense mesh
triangle candidate scanning, triangle clipping, and per-triangle reducer cost
could regress without a dedicated row making that cost visible.

**RCA 2026-06-24:** The mixed reducer itself was not hiding an obvious stronger
hot-path algorithm. A speculative sorted/lower-bound candidate pass improved
some dense-hit rows but regressed false-positive-heavy slabs, and a more
conservative hybrid pass still added cost without enough pruning. The final
mixed path keeps authored triangle scan order, tracks the best lower triangle
index directly, and uses the new benchmark/counter coverage as the guardrail.

The carryover pure 3D review did find a real mesh reducer bottleneck:
convex-source sweeps against concave mesh targets scanned exact triangle
reducers in raw candidate order. Ordering concave target triangle candidates by
a deterministic sweep lower bound lets the worker stop once the current best
TOI cannot be beaten by remaining triangles.

**Resolution 2026-06-24:**

- `GravitasQueryMixedService` now exposes a context-owned
  `LastMeshTriangleCandidateCount` for mixed mesh-target sweeps.
- `GravitasQuery3DService`, `SweptSphereQueryWorker`, and
  `ConvexSweepQueryWorker` expose matching 3D mesh-triangle candidate counts.
- `MixedMeshTriangleScalingBenchmarks` measures dense and false-positive
  mixed mesh targets by triangle volume rather than collider count.
- `MeshQuery3DTriangleScalingBenchmarks` measures pure 3D swept-sphere mesh
  targets and convex-source sweeps against dense concave mesh targets.
- `ConvexSweepQueryWorker` now sorts concave mesh target triangles by
  deterministic lower-bound TOI and authored triangle index, then exits once
  remaining triangles cannot beat the current best hit.

**Validation 2026-06-24:** Re-running
`mixed-mesh-triangle-scaling --filter "*TriangleMeshTarget*" -j Short -i`
reported predictable triangle-volume scaling with no recurring managed
allocation:

- Dense mixed target: `139.4 us` at `128` triangles, `514.2 us` at `512`
  triangles, and `2.031 ms` at `2048` triangles.
- False-positive mixed target: `136.6 us` at `128` triangles, `520.8 us` at
  `512` triangles, and `2.089 ms` at `2048` triangles.

Re-running
`mesh-query3-d-triangle-scaling --filter "*TriangleMeshTarget*" -j Short -i`
after the pure 3D carryover optimization reported:

- Swept-sphere dense mesh targets remain a tracked linear path: `550.1 us`,
  `2.145 ms`, and `8.638 ms` for `128`, `512`, and `2048` triangles.
- Convex-source sweeps against dense concave mesh targets improved from the
  pre-change baseline of about `12.465 ms`, `207.777 ms`, and `3.137 s` to
  `125.4 us`, `441.1 us`, and `1.658 ms` at the same triangle counts.

Focused mixed/3D query tests and the benchmark project build passed after the
change.

**Likely files:**

- `src/Gravitas/Queries/GravitasQueryMixedService.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Circle.cs`
- `src/Gravitas/Queries/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`
- `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- `tests/Gravitas.Benchmarks/Queries/MixedMeshTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Queries/MeshQuery3DTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs`

**Closure criteria:** Met. Benchmarks expose mesh finite-slab and 3D mesh sweep
cost by triangle candidate volume, not only collider count. Mixed keeps stable
authored triangle tie-breaks without speculative reducer overhead, and pure 3D
now avoids the measured convex-source concave-mesh triangle-order bottleneck.

### Signal: Pure 2D Dynamic CCD Candidate Asymmetry

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported
`Pure2DDynamicCandidateIndexAttributionEvidence` at about `4.31 ms/op` for
`1024` bodies, compared to about `1.47 ms/op` for 3D. The 2D relative-sweep
attribution row was also higher than 3D.

**Fresh baseline 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*Dynamic*AttributionEvidence*" -j
Short -i` reproduced the signal:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies:
  `598.731 us` versus 3D `277.042 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies:
  `4.709 ms` versus 3D `1.524 ms`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies:
  `4.782 ms` versus 3D `3.631 ms`.

**RCA 2026-06-23:** The dense benchmark layout was not accidentally stacking
3D Y layers into the 2D plane; both dense scenes are planar. The signal had
three concrete causes:

- Pure `TwoD` contexts built both the planar 2D candidate index and the mixed
  2D-as-3D slab candidate index even though mixed CCD can only run in
  `PhysicsRuntimeMode.Mixed`.
- Planar 2D candidate gathering reused the 3D `FixedBoundVolume` index, forcing
  a dead Y axis, `Vector3d` bound construction, and extra comparisons into the
  2D hot path.
- The evidence fixture reset 2D bodies with `Sleep()` followed by
  `SetPosition(...)`, which churned partition awake state and collider rebuilds
  before every attribution query. 3D already had a single `ResetPosition(...)`
  fixture reset path.

**Resolution 2026-06-23:** `GravitasPhysics2DService` now builds the mixed
dynamic CCD index only when the runtime mode actually runs mixed contacts.
Pure planar CCD uses `DynamicCcdCandidateIndex2D` and `DynamicCcdPlanarBounds`
instead of projecting circles into a 3D `FixedBoundVolume`. `StiffBody2D`
gained `ResetPosition(...)` parity with 3D, and the continuous-collision
benchmark fixture now uses it for deterministic 2D reset/setup without
sleep/wake churn.

**Validation 2026-06-23:** Re-running the same attribution benchmark reduced:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies:
  `598.731 us` -> `92.281 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies:
  `4.709 ms` -> `615.603 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `256` bodies:
  `590.845 us` -> `139.723 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies:
  `4.782 ms` -> `718.256 us`.

MemoryDiagnoser reported no recurring managed allocation in the 2D rows; the
remaining `1 B/op` at `1024` is treated as in-process runner noise unless a
focused allocation guard reproduces it.

**Likely files:**

- `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
- `src/Gravitas/Core/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/StiffBody2D.cs`
- `src/Gravitas/Core/StiffBody2D.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/StiffBody2D.ContinuousCollision.Kinematic.cs`
- `tests/Gravitas.Tests/CollisionHandling/DynamicCcdCandidateIndexTests.cs`
- `tests/Gravitas.Tests/Core/StiffBody2DAngularDynamicsTests.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Closure criteria:** Met. The gap is explained by mixed-index overwork,
planar-vs-3D index shape, and benchmark reset asymmetry. The runtime path now
keeps 2D planar candidate ordering stable with duplicate suppression preserved,
and the attribution benchmark no longer shows a 2D candidate-gathering penalty.

### Signal: 3D Shape-Exact False-Positive Cost

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process
`continuous-collision-evidence` BenchmarkDotNet smoke reported
`Pure3DFullRuntimeShapeExactFalsePositiveEvidence` as a standout cost compared
with the matching pure 2D row.

**RCA 2026-06-23:** The dominant cost was not the final GJK-style exact
reducer in isolation. Static 3D CCD first gathered hits through the conservative
swept-sphere proxy even for non-sphere exact-capable sources, then refined
every proxy hit afterward. In false-positive-heavy scenes, that made the broad
proxy path manufacture work that the source-shape sweep could reject earlier.
The exact sweep workers also lacked a cheap swept-bounds overlap prefilter, so
obviously disjoint target bounds could still enter the shape reducer.

**Resolution 2026-06-23:** 3D static CCD now collects non-sphere convex-source
hits through `GravitasQuery3DService.SweepExactSourceAgainstStaticAll(...)`.
That keeps source shape information during candidate collection and reserves
the old swept-sphere path for sphere or unsupported sources. Shared swept-bounds
prefilters reject disjoint sphere and convex-source targets before entering
per-shape reducer logic.

**Validation 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*ShapeExactFalsePositiveEvidence*"
-j Short -i` reduced:

- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `256` bodies:
  `37.816 ms` -> `19.744 ms`.
- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `1024` bodies:
  `174.326 ms` -> `99.561 ms`.

The focused `ContinuousCollisionDetectionTests` and
`GravitasQuery3DServiceSweepTests` filter passed after the change.

**Likely files:**

- `src/Gravitas/Core/StiffBody.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/StiffBody.ContinuousCollision.Kinematic.cs`
- `src/Gravitas/Queries/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`

**Closure criteria:** Met. The row is explained by conservative proxy overwork,
the runtime path was optimized without weakening exact-source correctness, and
focused CCD/query tests plus before/after benchmarks validate the change.

### Signal: 3D Dynamic Shape-Exact BDN Allocation Signal

**Discovered:** 2026-06-23

**Status:** Closed 2026-06-23

**Evidence:** The short in-process
`dynamic-ccd-scaling --filter "*DynamicShapeExact*" -j Short -i` smoke reported
`SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8` allocation scaling with
body count: about `43,008 B/op` at `64` bodies and `172,110 B/op` at `256`
bodies. The matching 2D rows reported only tiny in-process runner noise.

**Counter-evidence:** The focused xUnit guard
`ContinuousMode_DynamicRelativeShapeExactPath_ShouldNotAllocateAfterWarmup`
passed with `0` allocated bytes after warmup for the same thin-cuboid dynamic
relative false-positive shape family.

**RCA 2026-06-23:** The dynamic row exercised the same false-positive-heavy
exact-source reducer path as the static signal. The allocation scaling did not
reproduce in the focused runtime guard, and after the exact-source swept-bounds
prefilters landed, the BDN row no longer showed the large per-body allocation
slope. The remaining `78 B/op` at `256` bodies matches the tiny in-process
runner noise also reported by the 2D rows in the same run.

**Resolution 2026-06-23:** `ConvexSweepQueryWorker` now computes a padded
swept-source bounds interval during `Prepare(...)` and skips disjoint collider,
compound-part, and concave-mesh triangle candidates before exact reducer work.
`SweptSphereQueryWorker` uses the same shared bounds utility for sphere-source
sweeps.

**Validation 2026-06-23:** Re-running
`dynamic-ccd-scaling --filter "*DynamicShapeExact*" -j Short -i` reported:

- `SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8`, `64` bodies:
  `7.736 ms`, `0 B/op`.
- `SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8`, `256` bodies:
  `29.332 ms`, `78 B/op`.

The same run reported `42 B/op` and `78 B/op` for the matching 2D rows, so the
remaining values are treated as BenchmarkDotNet in-process measurement noise
unless a future focused allocation guard reproduces them.

**Likely files:**

- `src/Gravitas/Queries/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`

**Closure criteria:** Met. The scaling allocation signal was removed from the
benchmark row, the dynamic false-positive xUnit allocation guard remains green,
and the remaining BDN byte counts match runner noise rather than a runtime
allocation slope.

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
- Pure 3D swept-sphere dense mesh target rows are now visible through
  `MeshQuery3DTriangleScalingBenchmarks` and still scale linearly with triangle
  candidate volume: `550.1 us`, `2.145 ms`, and `8.638 ms` at `128`, `512`,
  and `2048` triangles. The lower-bound ordering optimization was adopted only
  for convex-source sweeps against concave mesh targets, where it proved a real
  bottleneck reduction. Revisit swept-sphere mesh pruning if host workloads need
  many analytic sphere casts against dense single-mesh colliders rather than
  partitioned/decomposed mesh geometry.
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

With the runtime CCD allocation, grounding, shape-exact false-positive, pure 2D
dynamic candidate-asymmetry, and mixed mesh triangle-scaling signals closed,
the remaining active benchmark-facing item is the SwiftCollections lower-stack
sort gap. Keep that item open until the lower-stack sort APIs can replace
`SwiftListSortUtility` without allocation regressions.
