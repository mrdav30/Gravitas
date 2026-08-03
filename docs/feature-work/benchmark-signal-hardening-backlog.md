# Benchmark Signal Hardening Backlog

## Purpose

This document captures benchmark-derived hardening signals that fall outside the
active feature plan. It is intentionally undated and long-lived: individual
entries carry their own discovery dates, evidence, status, and next isolation
step.

Use this backlog for measured performance, allocation, scaling, and benchmark
evidence concerns. Bugs or correctness risks that are not primarily benchmark
signals belong in [`issue-tracker.md`](issue-tracker.md). Broad feature or
architecture work should be promoted into its own dated plan and referenced from
this backlog.

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

| Signal                                                              | Status   | Priority | Tracking                                                                                  |
| ------------------------------------------------------------------- | -------- | -------- | ----------------------------------------------------------------------------------------- |
| Mixed public sweep traversal stalls on extreme sparse-grid spans    | Observed | Medium   | Isolate GridTracer clipping and cell-visit scaling independently of narrow phase          |
| Mixed discrete broad-phase refresh allocates at 32 moving CCD pairs | Isolated | Low      | Reproduce capacity-growth threshold independently of rotational CCD                       |

### Signal: Mixed Public Sweep Traversal Stalls On Extreme Sparse-Grid Spans

**Discovered:** 2026-07-19  
**Source:** focused mixed swept-circle public-query regression  
**Status:** Observed; terminated run, no completed timing sample

A temporary focused public mixed-query diagnostic swept from `-200,000` to
`+200,000` with radius `100,000` through a sparse grid configured with
`100,000`-unit rectangular cells. Its isolated Release run did not complete
within approximately 30 seconds and was terminated. The diagnostic was not
retained as a unit test because its dominant behavior was broad-phase traversal,
not the finite-axis narrow-phase contract it was intended to verify.

The same finite-axis reducer completes promptly when invoked below public
candidate gathering, so the signal is in broad-phase traversal rather than the
exact narrow-phase solve. The run was terminated rather than benchmarked to
completion; do not treat it as a stable latency measurement. The smallest next
step is a bounded GridTracer profiler harness that records visited cells and
active-grid clipping for long sparse spans, then determines whether traversal
should skip unoccupied world space or whether the public query needs an explicit
world-span contract.

### Signal: Mixed Discrete Broad-Phase Refresh Allocation At 32 Pairs

**Discovered:** 2026-07-19  
**Source:** `RotationalMovingPairCcdBenchmarks` mixed 3D-to-2D ShortRun  
**Status:** Isolated; no CCD runtime defect confirmed

The mixed 3D-to-2D end-to-end rows measured `0 B/op` at 1 and 8 pairs. Repeated
short runs reported a small, run-dependent 32-pair signal: initially `48 B/op`
and finally `10 B/op`. Focused warmed guards for CCD preparation, interval
search, response, handoff, reset, and completion all remain allocation-free.
Temporary test-side phase instrumentation localized the recurring sample to the
pre-existing mixed discrete `GravitasMixedCollisionService.LateSimulate`
partition refresh and broad-phase capacity-growth path after CCD completes.

Keep the honest 1/8/32 benchmark row. The smallest next step is to reproduce the
same threshold without rotational motion, identify which retained partition or
candidate buffer grows, and decide whether an explicit warm-capacity policy is
justified by representative world churn. Do not add speculative production
preallocation merely to hide this benchmark sample.

## Experimental Signals

| Signal                                                             | Status                                           | Revisit When                                                                                                            |
| ------------------------------------------------------------------ | ------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| Exact triangle-pair contacts regress dense concave-mesh throughput | Capacity-sensitive; local optimization exhausted | A topology or exact classifier design can reduce complete triangle-pair SAT evaluations without a competing answer path |

### Signal: Exact Triangle-Pair Contacts Regress Dense Concave-Mesh Throughput

**Discovered:** 2026-08-01  
**Source:** full-domain triangle-pair Phase 2 comparison against its preserved
scalar mesh/mesh baseline  
**Status:** Experimental capacity guidance. Shared exact-projection and
depth-ranking duplication plus the retained signed one-limb specialization
recovered substantial throughput. A final bounded pass found no further local
change worth retaining; dense dynamic concave mesh/mesh contact is not a
competitive release path.

The unchanged 64-pair Short in-process rows reported:

| Row                             | Scalar baseline |  Initial exact | Final optimized exact | Closure confirmation |
| ------------------------------- | --------------: | -------------: | --------------------: | -------------------: |
| Ordinary convex mesh/mesh       |      `5.168 ms` |     `4.921 ms` |            `4.839 ms` |           `4.900 ms` |
| Concave mesh/mesh               |     `16.120 ms` |    `98.489 ms` |           `70.553 ms` |          `70.005 ms` |
| Dense concave mesh/mesh         |    `105.139 ms` |   `570.378 ms` |          `400.501 ms` |         `397.087 ms` |
| Contact-heavy concave mesh/mesh |    `163.956 ms` |   `804.559 ms` |          `564.147 ms` |         `556.138 ms` |
| Closed dense mesh/mesh          |    `747.173 ms` | `3,641.633 ms` |        `2,532.822 ms` |       `2,519.288 ms` |

FixedMathSharp now computes each triangle's basis-axis projections once per axis
and cancels identical positive common denominators during normalized-depth
ranking. Those policy-neutral deletions recovered roughly `28-30%` of the
initial exact dense-row cost without changing axis order, contact results, or
warmed `0 B` behavior. The ordinary convex row remains comparable because it
uses the existing convex-hull relation rather than the concave triangle-pair
generator.

The remaining gap is the measured cost of invoking the complete wide
triangle/triangle relation for every BVH-admitted candidate; candidate counts
and traversal complexity did not change. That evidence led to the per-candidate
profile recorded in the 2026-08-02 follow-up below. Do not restore the deleted
scalar SAT, add a narrowed prefilter, or create a second answer path that can
disagree with the full-domain authority. Preserved artifacts are under
`artifacts/benchmarks/2026-07-31-triangle-pair-baseline`,
`artifacts/benchmarks/2026-07-31-triangle-pair-gravitas-after`, and
`artifacts/benchmarks/2026-07-31-triangle-pair-after-denominator-cancellation`.

The 2026-08-01 closure rerun used the same 64-pair Short in-process job. Its
point estimates stayed within `-1.42%` to `+1.26%` of the final optimized run,
so it confirms the retained signal without supporting another performance claim.
MemoryDiagnoser reported fixed `78 B` / `624 B` readings on the longer
in-process rows; all 72 direct warmed Gravitas allocation guards, including the
concave and dense mesh paths, measured exactly `0 B`, so the direct guards
remain the runtime allocation authority; this document does not assign a cause
to the differing in-process MemoryDiagnoser readings. The closure artifacts are
under `artifacts/benchmarks/2026-08-01-triangle-pair-closure`.

The 2026-08-02 follow-up profiled the unchanged dense row and isolated generic
wide-multiply dispatch inside exact projection as the next shared cost. Raw
`Fixed64` coordinates were widened to `Signed192` even though each operand is a
proven signed one-word factor. FixedMathSharp now owns an exact
`Signed576`-by-`long` specialization, and triangle projection calls that owner
directly without changing the result width, axis order, tie behavior, contact
anchors, or public API.

| Row                             | Refreshed baseline | Retained change | Confirmation |
| ------------------------------- | -----------------: | --------------: | -----------: |
| Ordinary convex mesh/mesh       |         `4.836 ms` |      `4.933 ms` | control only |
| Concave mesh/mesh               |        `70.351 ms` |     `60.213 ms` |  `59.761 ms` |
| Dense concave mesh/mesh         |       `405.224 ms` |    `342.682 ms` | `343.474 ms` |
| Contact-heavy concave mesh/mesh |       `556.972 ms` |    `480.926 ms` | `480.773 ms` |
| Closed dense mesh/mesh          |          `2.566 s` |       `2.170 s` |    `2.155 s` |

The direct FixedMathSharp `TrianglePairPrimary` row improved from the prior
`64.221 us` closure to `54.33 us`, or `15.4%`, with `0 B` reported. All `18`
focused Gravitas triangle/concave/allocation regressions pass, and the direct
warmed guards remain the allocation authority at `0 B`; the small, variable
BenchmarkDotNet allocation readings are not treated as runtime allocations.

Common-denominator hoisting and eager/lazy second-edge preparation were also
measured and reverted because they did not produce a repeatable end-to-end gain
on the unchanged Gravitas rows. The optimized exact rows remain approximately
`2.9-3.7x` slower than the deleted scalar baseline, so the signal remained
material after the retained work. A final experimental pass tested an exact
signed two-limb multiplication specialization and invocation-local rigid frame
preparation. The direct specialization improved only `0.6%`; frame preparation
left the affected Gravitas rows between `0.28%` and `1.04%` slower. Both changes
were reverted exactly.

Evidence now favors reducing complete exact SAT evaluations; the tested two-limb
dispatch and frame preparation were not material. Revisit only through a
separate topology or exact-classifier design; do not grow the current relation
with more local special cases. The focused plans and evidence are preserved in
[`2026-08-02-exact-triangle-pair-throughput-plan.md`](done/2026-08-02-exact-triangle-pair-throughput-plan.md)
and
[`2026-08-02-experimental-triangle-pair-throughput-plan.md`](done/2026-08-02-experimental-triangle-pair-throughput-plan.md).

## Closed Signals

| Signal                                                      | Status | Closed     | Resolution                                                                                                                                                                                                                                  |
| ----------------------------------------------------------- | ------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Mesh scale rebuild allocation                               | Closed | 2026-08-03 | Convex support topology is built once and scale changes refit transactional node bounds in linear time; subdivision 8/16 rows fall from 4,032/16,320 B to 0 B and improve by 7.9%/7.8%                                                        |
| Exact 3D contact-response ordinary throughput               | Closed | 2026-08-03 | Exact aligned-frame point anchors improve direct rows by 61.0-95.9% and the unchanged 24-row Gravitas matrix by 46.4% median versus the exact baseline; confirmation remains within 0.7% median at 0 B and 100% coverage                    |
| Exact canonical OBB ordinary throughput                     | Closed | 2026-08-03 | One exact relative-frame kernel per relation improves matched direct rows by 35.3-64.0% and Gravitas rows by 30.9-55.7%; full DefaultJob confirmations remain at 0 B and 100% reachable coverage                                            |
| Physics-material combine numeric hardening                  | Closed | 2026-07-13 | Overflow-safe average and geometric-mean edge handling preserve deterministic coefficient semantics; the default geometric-material response benchmark remains allocation-free with no credible timing regression                           |
| Replay hash collider-ID churn scaling                       | Closed | 2026-07-05 | 2D and 3D collider registration now uses a shared reusable-slot registry; authoritative replay hashes traverse canonical live registration order with dense replay ordinals, while deleted ID history remains outside replay identity       |
| Pure 2D response position-correction repartition allocation | Closed | 2026-06-28 | Gravitas reuses empty retained partitions for immediate repartitioning; GridForge stores the common single voxel partition inline and keeps diagnostic names off success paths                                                              |
| SwiftCollections sort hot-path allocation                   | Closed | 2026-06-24 | SwiftCollections owns allocation-free sort and sorted-key APIs; Gravitas removed `SwiftListSortUtility`                                                                                                                                     |
| Mixed mesh finite-slab triangle scaling signal              | Closed | 2026-06-24 | Mixed and pure 3D query services expose mesh-triangle candidate counts, dedicated triangle-volume benchmarks cover dense and false-positive mesh targets, and pure 3D convex-source mesh sweeps use ordered lower-bound triangle candidates |
| Pure 2D dynamic CCD candidate asymmetry                     | Closed | 2026-06-23 | 2D uses a planar candidate index, skips mixed CCD indexing outside mixed mode, and benchmark resets use 2D reset parity                                                                                                                     |
| 3D shape-exact false-positive cost                          | Closed | 2026-06-23 | Static CCD uses exact-source sweeps for non-sphere convex movers before conservative sphere fallback refinement                                                                                                                             |
| 3D dynamic shape-exact BDN allocation signal                | Closed | 2026-06-23 | Shared exact-sweep bounds prefilters removed the scaling allocation/time signal from 3D dynamic false-positive rows                                                                                                                         |
| 3D full-runtime CCD allocation                              | Closed | 2026-06-23 | GridForge allocation-free line tracing plus Gravitas 3D raycast adoption                                                                                                                                                                    |
| Grounding raycast probe allocation                          | Closed | 2026-06-23 | Same raycast trace fix removed automatic ray-grounding allocation                                                                                                                                                                           |

### Closed Signal: Mesh Scale Rebuild Allocation

**Discovered:** 2026-07-28 **Closed:** 2026-08-03

**Initial evidence:** A refreshed focused ShortRun of
`MeshMassPropertyBenchmarks.UpdateNonUniformMeshScaleAndCalculateSurfaceInertia`
reported:

| Subdivision | Mean | Allocated |
| ---: | ---: | ---: |
| 1 | `38.394 us` | `0 B/op` |
| 8 | `2.166 ms` | `4,032 B/op` |
| 16 | `8.822 ms` | `16,320 B/op` |

**RCA:** Triangle-BVH rebuilding, scaled face data, and surface mass properties
remain allocation-free. Convex support-tree preparation instead sorted every
non-leaf vertex range after every scale change through a retained reference
comparer. Each `Array.Sort(...)` call allocated 64 bytes; the subdivision-8 and
subdivision-16 trees have 63 and 255 non-leaf nodes, exactly accounting for the
measured totals.

**Resolution:** `PhysicsMesh` now builds its support-vertex partition once.
Subsequent scale candidates refit leaf bounds from that immutable partition and
branch bounds bottom-up into the existing prepared node buffer. Publication
still swaps complete committed/prepared node buffers transactionally. The
second support-index array, its publication swap, repeated sorting, and retained
construction comparer were deleted. Exact support selection and authored-order
ties are unchanged.

The unchanged command now reports:

| Subdivision | Baseline | Confirmation | Delta | Allocated |
| ---: | ---: | ---: | ---: | ---: |
| 1 | `38.394 us` | `37.755 us` | `-1.7%` | `0 B/op` |
| 8 | `2.166 ms` | `1.994 ms` | `-7.9%` | `0 B/op` |
| 16 | `8.822 ms` | `8.131 ms` | `-7.8%` | `0 B/op` |

Gravitas passes 3,928 Release and 3,873 ReleaseLean tests. Coverage remains
55,869/55,869 lines, 15,833/15,833 branches, and 5,321/5,321 methods. The
focused plan and complete evidence are preserved in
[`2026-08-03-mesh-scale-rebuild-throughput-plan.md`](done/2026-08-03-mesh-scale-rebuild-throughput-plan.md).

Artifacts:

- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-baseline`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-first-pass`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-confirmation`
- `tests/Gravitas.Tests/TestResults/coverage-analysis-mesh-scale-rebuild-20260803`

### Closed Signal: Replay Hash Collider-ID Churn Scaling

**Discovered:** 2026-07-05 **Closed:** 2026-07-05

**Initial evidence:** A focused replay-hash benchmark row,
`ReplayHashBenchmarks.ReplayHash3DChurnedIds`, created an 8x deleted-collider
history by registering and deactivating bodyless 3D static colliders, then
leaving only the final live tail active.

Initial command:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*ReplayHash3DChurnedIds*" --warmupCount 1 --iterationCount 3
```

Initial measurement on 2026-07-05:

| Row                                        |       Mean | Allocated |
| ------------------------------------------ | ---------: | --------: |
| `ColliderCount=64` live, 512 created IDs   | `119.0 us` |     `0 B` |
| `ColliderCount=256` live, 2048 created IDs | `483.9 us` |     `0 B` |

**RCA:** 2D and 3D services had separate collider ownership structures: compact
live lists, ID dictionaries, and manual next-ID/high-water counters. Replay
hashing walked the high-water ID range and emitted deleted-hole state, while
mixed replay hashing crossed 3D and 2D high-water ranges before checking whether
a mixed pair existed. That made deleted context-local allocation history part of
authoritative replay identity even though serialization treats context-local
collider IDs, service indices, partitions, and pair tables as runtime-owned
state.

**Resolution:** 2D and 3D collider registration now goes through a shared
registry backed by reusable `SwiftBucket` slots plus compact live iteration.
`-1` is the unregistered collider sentinel across dimensions; `0` is a valid
context-local collider ID. Runtime IDs remain lookup and pair keys, while
authoritative replay hashes traverse canonical live registration order and write
dense replay ordinals for collider, hierarchy, and pair identity. Deleted ID
holes, free-list ordering, and allocator history are excluded from authoritative
hashes, while registry peak counts remain cache diagnostics for
`AuthoritativeWithSolverCaches`.

Post-fix command:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*ChurnedIds*" --warmupCount 1 --iterationCount 3
```

Post-fix measurement on 2026-07-05:

| Row                                                  |       Mean | Allocated |
| ---------------------------------------------------- | ---------: | --------: |
| `replay-hash-3d-churned-ids`, `ColliderCount=64`     | `114.7 us` |     `0 B` |
| `replay-hash-2d-churned-ids`, `ColliderCount=64`     | `124.3 us` |     `0 B` |
| `replay-hash-mixed-churned-ids`, `ColliderCount=64`  | `124.3 us` |     `0 B` |
| `replay-hash-3d-churned-ids`, `ColliderCount=256`    | `489.2 us` |     `0 B` |
| `replay-hash-2d-churned-ids`, `ColliderCount=256`    | `537.9 us` |     `0 B` |
| `replay-hash-mixed-churned-ids`, `ColliderCount=256` | `550.9 us` |     `0 B` |

**Verification:** Added replay-hash tests proving deleted 3D, 2D, and mixed
collider churn and free-list ordering do not affect authoritative hashes, live
collider ordering still affects authoritative hashes, and steady-state replay
hashing remains allocation-free after churn. Added registry tests proving
reusable IDs, context-local lookup, compact service indices, and `-1` inactive
sentinels.

### Closed Signal: Pure 2D Response Position-Correction Repartition Allocation

**Discovered:** 2026-06-26 **Closed:** 2026-06-28

**Evidence:** During the physics material model validation pass, a focused
allocation guard for `CollisionResponse2D.Resolve(...)` initially measured a
stable `2712 B` allocation when the prepared manifold used non-zero depth and
the measured action reset velocity and resolved response in one pass. Re-running
the material solver path with zero penetration depth and measuring only the
prepared response call reported `0 B`, which pointed away from material
resolution and toward the 2D position-correction/repartition path.

**RCA 2026-06-28:** A dedicated guard reproduced the hot path by forcing
non-zero 2D position correction across GridForge voxel partitions. The first
focused repro measured `18,936 B` over four measured iterations. Gravitas kept
empty `PhysicsPartition2D` instances retained on old voxels, but when a moving
collider crossed into fresh voxels and the inactive partition pool was empty,
the service allocated new partition objects and first-use sparse sets instead of
retiring an empty retained partition for immediate reuse. After adding
retained-empty reuse, the repro dropped to `7,712 B`.

The remaining allocation was lower-stack metadata. GridForge's
`PartitionProvider` always allocated a `SwiftDictionary<Type, IVoxelPartition>`
for the first partition attached to a voxel. Physics partitions are commonly the
only partition type on a voxel, so this made every fresh voxel attach pay a
general dictionary allocation. After moving the common single-partition case
inline, the final `224 B` repro remainder came from eager `Type.Name` diagnostic
strings in `Voxel.TryAddPartition(...)` and `Voxel.TryRemovePartition<T>()`;
those names were only needed on failure but were built on the success path.

**Resolution 2026-06-28:** `GravitasCollision2DService`,
`GravitasCollisionService`, and `GravitasMixedCollisionService` now retire an
empty retained partition for immediate reuse when their inactive pool is empty.
GridForge's `PartitionProvider<TPartitionBase>` stores the first partition
inline, upgrades to `SwiftDictionary` only when a second concrete partition type
is attached, keeps multi-partition storage reusable after `Clear()`, and exposes
an internal allocation-free enumerator for voxel reset. `Voxel` now creates
partition type names only on error paths.

**Validation 2026-06-28:** Focused GridForge allocation guards pass for first
single-partition provider attach and voxel add/remove success paths. Focused
Gravitas guards pass for:

- 2D response position correction crossing partitions.
- 3D response position correction followed by collider repartition refresh.
- mixed 3D collider refresh crossing mixed partitions.

The `physics-2d --filter "*Resolve*" --job Short` benchmark smoke reports no
managed allocation across the selected 64-body and 1024-body 2D response rows.

**Touched files:**

- `../GridForge/src/GridForge/Spatial/PartitionProvider.cs`
- `../GridForge/src/GridForge/Grids/Nodes/Voxel.cs`
- `../GridForge/tests/GridForge.Tests/Spatial/SpatialTypes.Tests.cs`
- `../GridForge/tests/GridForge.Tests/Grids/Voxel.Tests.cs`
- `src/Gravitas/Core/2D/GravitasCollision2DService.cs`
- `src/Gravitas/Core/3D/GravitasCollisionService.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Partitioning.cs`
- `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DManifoldTests.cs`
- `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedBroadPhaseTests.cs`

### Closed Signal: SwiftCollections Sort Hot-Path Allocation

**Discovered:** 2026-06-22 **Closed:** 2026-06-24

**Evidence:** During the post-SwiftCollections v5.1.0 Workstream 3 cleanup,
replacing Gravitas' local sort helpers with package
`SwiftList<T>.SortInPlace(...)` and `SwiftSparseSet.CopySortedKeysTo(...)`
caused the Release allocation guardrails to fail. The full suite reported
recurring allocations in 3D CCD, pure 2D CCD, pure 2D broad phase, and 2D query
tests. The isolated
`Physics2DQueryTests.RaycastAll_ShouldNotAllocateAfterWarmup` test reproduced
the issue at `128 B` after warmup.

**Resolution:** SwiftCollections owns the allocation-free sort primitive.
`SwiftList<T>.SortInPlace(...)` routes default ordering through the optimized
BCL default-comparer path, custom class/interface comparers through an
allocation-free introsort, and struct comparers through a no-boxing generic
introsort. `SwiftSortedList<T>` bulk-load, known-count `IReadOnlyCollection<T>`
range insertion, and `SetComparer(...)` use the same lower-stack helper without
recurring managed sort allocations. Gravitas removed
`src/Gravitas/Support/SwiftListSortUtility.cs` and calls
`SwiftList<T>.SortInPlace(...)` or `SwiftSparseSet.CopySortedKeysTo(...)`
directly from runtime ordering paths.

**Why it matters:** `SwiftCollections` is the lower-stack collection layer for
LSF. Its scratch-buffer sort APIs should be safe for deterministic physics hot
paths so consumers do not need local workarounds.

**Benchmark signal:** Short-run comparer benchmarks show the tradeoff:
`List<T>.Sort(custom class comparer)` remains faster but allocates `64 B/op`,
while `SwiftList<T>.SortInPlace(custom class comparer)` allocates `0 B/op`. For
struct comparers, `List<T>.Sort(struct comparer)` measured about `12.136 ms` at
`100000` integers with `88 B/op`; the Swift struct-comparer path measured about
`13.089 ms` with `0 B/op`, closing most of the CPU gap while preserving the
allocation contract.

**Touched files:**

- `../SwiftCollections/src/SwiftCollections/Collection/SwiftList.cs`
- `../SwiftCollections/src/SwiftCollections/Collection/SwiftSortedList.cs`
- `../SwiftCollections/src/SwiftCollections/Utility/SwiftArraySortHelper.cs`
- `../SwiftCollections/src/SwiftCollections/Collection/SwiftSparseSet.cs`
- `src/Gravitas/Core/3D/GravitasCollisionService.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/2D/GravitasCollision2DService.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.cs`
- `src/Gravitas/Partitions/*/*Partition*.cs`

**Closure evidence:** SwiftCollections focused allocation guardrails and full
Release/ReleaseLean test suites pass, GridForge and Gravitas validate through
local project references, Gravitas Release/ReleaseLean allocation guardrails
pass, and the Gravitas simulation allocation benchmark smoke rows remain at
`0 B/op`.

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
a deterministic sweep lower bound lets the worker stop once the current best TOI
cannot be beaten by remaining triangles.

**Resolution 2026-06-24:**

- `GravitasQueryMixedService` exposes a context-owned
  `LastMeshTriangleCandidateCount` for mixed mesh-target sweeps.
- `GravitasQuery3DService`, `SweptSphereQueryWorker`, and
  `ConvexSweepQueryWorker` expose matching 3D mesh-triangle candidate counts.
- `MixedMeshTriangleScalingBenchmarks` measures dense and false-positive mixed
  mesh targets by triangle volume rather than collider count.
- `MeshQuery3DTriangleScalingBenchmarks` measures pure 3D swept-sphere mesh
  targets and convex-source sweeps against dense concave mesh targets.
- `ConvexSweepQueryWorker` sorts concave mesh target triangles by deterministic
  lower-bound TOI and authored triangle index, then exits once remaining
  triangles cannot beat the current best hit.

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

- `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`
- `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- `tests/Gravitas.Benchmarks/Queries/MixedMeshTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Queries/MeshQuery3DTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs`

**Closure criteria:** Met. Benchmarks expose mesh finite-slab and 3D mesh sweep
cost by triangle candidate volume, not only collider count. Mixed keeps stable
authored triangle tie-breaks without speculative reducer overhead, and pure 3D
avoids the measured convex-source concave-mesh triangle-order bottleneck.

### Signal: Pure 2D Dynamic CCD Candidate Asymmetry

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported `Pure2DDynamicCandidateIndexAttributionEvidence`
at about `4.31 ms/op` for `1024` bodies, compared to about `1.47 ms/op` for 3D.
The 2D relative-sweep attribution row was also higher than 3D.

**Fresh baseline 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*Dynamic*AttributionEvidence*" -j Short -i`
reproduced the signal:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies: `598.731 us`
  versus 3D `277.042 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies: `4.709 ms`
  versus 3D `1.524 ms`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies: `4.782 ms`
  versus 3D `3.631 ms`.

**RCA 2026-06-23:** The dense benchmark layout was not accidentally stacking 3D
Y layers into the 2D plane; both dense scenes are planar. The signal had three
concrete causes:

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

**Resolution 2026-06-23:** `GravitasPhysics2DService` builds the mixed dynamic
CCD index only when the runtime mode actually runs mixed contacts. Pure planar
CCD uses `DynamicCcdCandidateIndex2D` and `DynamicCcdPlanarBounds` instead of
projecting circles into a 3D `FixedBoundVolume`. `SolidBody2D` gained
`ResetPosition(...)` parity with 3D, and the continuous-collision benchmark
fixture uses it for deterministic 2D reset/setup without sleep/wake churn.

**Validation 2026-06-23:** Re-running the same attribution benchmark reduced:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies: `598.731 us`
  -> `92.281 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies: `4.709 ms` ->
  `615.603 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `256` bodies: `590.845 us` ->
  `139.723 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies: `4.782 ms` ->
  `718.256 us`.

MemoryDiagnoser reported no recurring managed allocation in the 2D rows; the
remaining `1 B/op` at `1024` is treated as in-process runner noise unless a
focused allocation guard reproduces it.

**Likely files:**

- `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/2D/SolidBody2D.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`
- `tests/Gravitas.Tests/CollisionHandling/DynamicCcdCandidateIndexTests.cs`
- `tests/Gravitas.Tests/Core/SolidBody2DAngularDynamicsTests.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Closure criteria:** Met. The gap is explained by mixed-index overwork,
planar-vs-3D index shape, and benchmark reset asymmetry. The runtime path keeps
2D planar candidate ordering stable with duplicate suppression preserved, and
the attribution benchmark no longer shows a 2D candidate-gathering penalty.

### Signal: 3D Shape-Exact False-Positive Cost

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported
`Pure3DFullRuntimeShapeExactFalsePositiveEvidence` as a standout cost compared
with the matching pure 2D row.

**RCA 2026-06-23:** The dominant cost was not the final GJK-style exact reducer
in isolation. Static 3D CCD first gathered hits through the conservative
swept-sphere proxy even for non-sphere exact-capable sources, then refined every
proxy hit afterward. In false-positive-heavy scenes, that made the broad proxy
path manufacture work that the source-shape sweep could reject earlier. The
exact sweep workers also lacked a cheap swept-bounds overlap prefilter, so
obviously disjoint target bounds could still enter the shape reducer.

**Resolution 2026-06-23:** 3D static CCD collects non-sphere convex-source hits
through `GravitasQuery3DService.SweepExactSourceAgainstStaticAll(...)`. That
keeps source shape information during candidate collection and reserves the old
swept-sphere path for sphere or unsupported sources. Shared swept-bounds
prefilters reject disjoint sphere and convex-source targets before entering
per-shape reducer logic.

**Validation 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*ShapeExactFalsePositiveEvidence*" -j Short -i`
reduced:

- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `256` bodies: `37.816 ms`
  -> `19.744 ms`.
- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `1024` bodies:
  `174.326 ms` -> `99.561 ms`.

The focused `ContinuousCollisionDetectionTests` and
`GravitasQuery3DServiceSweepTests` filter passed after the change.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
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

**Resolution 2026-06-23:** `ConvexSweepQueryWorker` computes a padded
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

- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`

**Closure criteria:** Met. The scaling allocation signal was removed from the
benchmark row, the dynamic false-positive xUnit allocation guard remains green,
and the remaining BDN byte counts match runner noise rather than a runtime
allocation slope.

### Signal: 3D Full-Runtime CCD Allocation

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported pure 3D full-runtime rows allocating about
`172,032 B/op` at `256` bodies and `688,138 B/op` at `1024` bodies. Pure 2D
full-runtime rows and CCD attribution rows were effectively allocation-clean.

**Update 2026-06-21:** Discrete response Workstream 3 reproduced the related
steady-state CCD guardrail failures in xUnit after the 3D full-step phase order
started exercising collision distribution during measured CCD frames. Root cause
was comparer-based `Array.Sort` through package sorting in the collision
distribution/island hot path. Gravitas uses a centralized allocation-free
runtime sort helper for active partitions, island buffers, and per-partition
collider ID copies. The focused Release allocation guardrails for 3D substep,
shape-exact translational, and rotational CCD pass under full `Simulate` +
`LateSimulate` measurement. `simulation-allocation` smoke also reported `0 B/op`
for `CollisionPartitionDistributionOnly` and `ActivePairProcessingLateSimulate`.

**Initial read:** The allocation is likely outside core CCD query/index/sweep
math. Suspect reset, host-transform publish, partition refresh, collision-pair
lifecycle, or another 3D full-runtime phase.

**Why it matters:** CCD is a hot-path feature for fast movers. Lockstep
simulations need predictable frame cost and should not introduce GC pressure
that scales with body count.

**Completed isolation:** The original rows were rerun after the Workstream 3
sort fix. Temporary attribution rows split reset, force setup, and late
simulation enough to identify the moving-frame raycast grounding path.

**RCA 2026-06-23:** The remaining allocation was not CCD-specific. The
full-runtime 3D CCD evidence bodies moved each frame with automatic ray
grounding enabled, and `SolidBody` grounding called `Query3D.RaycastAll`. The
raycast service still used GridForge's enumerable `GridTracer.TraceLine` path,
which allocated iterator/mapping state per ray. Reset, force setup, and
non-moving late simulation attribution were allocation-clean.

**Resolution 2026-06-23:** GridForge exposes caller-owned
`GridTracer.TraceLineInto(...)` overloads backed by `GridTraceScratch`.
`GravitasQuery3DService` uses that allocation-free trace buffer for closest-hit
and all-hit raycasts. A focused 3D `RaycastAll` allocation guard protects the
path.

**Validation 2026-06-23:** Re-running the original
`continuous-collision-evidence --filter "*Pure3DFullRuntime*DynamicCcdEvidence*" -j Short -i`
smoke reduced:

- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `256` bodies: `168 KB/op` ->
  `0 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `256` bodies: `168.01 KB/op` ->
  `15 B/op`.
- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `1024` bodies: `672.01 KB/op` ->
  `15 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `1024` bodies: `672.01 KB/op`
  -> `15 B/op`.

The remaining `15 B/op` values were not reproduced by the focused xUnit
allocation guard and are treated as BenchmarkDotNet in-process measurement noise
unless a future guardrail reproduces them.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService.cs`
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
`GroundingRaycastProbeOnly` at about `181.8 us` and `43,008 B/op` for `64`
colliders. The same run reported no managed allocation for
`SolidBodyLateSimulateOnly`, `GroundingSweptSphereProbeOnly`,
`CollisionPartitionDistributionOnly`, and `ActivePairProcessingLateSimulate`.

**Initial read:** This appears separate from the discrete island work and the
collision distribution sort RCA. It likely belongs to the raycast-backed ground
probe or one of the query result/candidate paths used by that benchmark row.

**Why it matters:** Automatic raycast grounding is a recurring body hot path. If
this allocation is repeatable outside BenchmarkDotNet noise, grounded 3D bodies
can create avoidable GC pressure.

**Completed isolation:** The focused 3D raycast allocation guard and grounding
benchmark confirmed the automatic ray-grounding allocation came from the shared
raycast trace path.

**RCA 2026-06-23:** This was the same root cause as the remaining 3D
full-runtime CCD allocation: automatic ray grounding used `Query3D.RaycastAll`,
which depended on the enumerable GridForge line-trace path.

**Resolution 2026-06-23:** Gravitas 3D raycasts use GridForge's caller-owned
`TraceLineInto(...)` path. The grounding row no longer allocates after warmup.

**Validation 2026-06-23:** Re-running
`simulation-allocation --filter "*Grounding*" -j Short -i` reported
`GroundingRaycastProbeOnly` at `164.9 us` and `0 B/op` for `64` colliders.
`GroundingSweptSphereProbeOnly` also remained allocation-clean.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs`
- `tests/Gravitas.Tests/Core/SolidBodyGroundingTests.cs`

**Closure criteria:** Met. The runtime allocation was eliminated and the 3D
raycast path has a focused xUnit allocation guard.

### Closed Signal: Checked Mesh Scale And Thin-Shell Cache Cost

**Status:** Closed 2026-07-12

**Evidence:** Task 46 extended `MeshMassPropertyBenchmarks` with matching
scale-only and scale-plus-surface-inertia rows. The final post-change command
was:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    mesh-mass-property --filter "*UpdateNonUniformMeshScale*" --job short
```

The checked scale/cache rebuild measured about `5.530 us`, `282.282 us`, and
`1.122 ms` at subdivisions `1`, `8`, and `16`. Scale plus lazy physical
thin-shell integration measured `16.087 us`, `907.933 us`, and `3.609 ms`. Every
row reported no managed allocation.

A cache-focused follow-up used:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    mesh-mass-property --filter "*CalculateCachedClosedVolumeInertiaTensor*" `
    --job short --artifacts `
    "artifacts/benchmarks/2026-07-12-task46-mesh-scale-cache-fix2"
```

Cached closed-volume inertia measured `61.14 ns`, `64.79 ns`, and `64.33 ns` at
subdivisions `1`, `8`, and `16`, with no managed allocation.

**Resolution:** Scale changes pay deterministic O(triangle-count) validation and
scaled face-cache rebuilding. Surface integration stays lazy and its
successfully prevalidated candidate is promoted into the live cache, so callers
that do not select `SurfaceApproximation` do not pay the shell moment pass.
Closed-volume properties are likewise cached by committed scale, including the
default-scale initialization path; checked prevalidation is promoted at commit
and pose-only updates retain the cache. Retain both transform and cached-read
rows as regression signals for future mesh transform or mass-property work.

### Closed Signal: Physics-Material Combine Numeric Hardening

**Status:** Closed 2026-07-13

**Evidence:** Task 48 compared the existing default-material response row at the
pre-change `577cdb1` checkpoint and the final arithmetic implementation. This
row exercises the dominant `GeometricMean` friction policy rather than the
distinct-material `Maximum` path.

| Body count |     Baseline |        Final | Allocated |
| ---------: | -----------: | -----------: | --------: |
|         64 | `148.785 us` | `149.243 us` |     `0 B` |
|       1024 |  `2.8145 ms` |  `2.6088 ms` |     `0 B` |

The 64-body intervals overlap (`+0.31%` point estimate), while the 1024-body
measurement was multimodal and is retained only as a no-regression signal, not
as a speedup claim. Artifacts:

- `artifacts/benchmarks/2026-07-13-task48-geometric-material-baseline`
- `artifacts/benchmarks/2026-07-13-task48-geometric-material-after`

**Resolution:** `Average` now computes an overflow-safe raw midpoint with
ties-to-even rounding. Positive equal geometric inputs retain exact identity;
positive unequal inputs multiply separately rounded square roots so coefficient
products cannot saturate or quantize away before the root. The revised path is
deterministic, allocation-free, symmetric in sampled review, and showed no
credible regression in the default contact-response benchmark.

## Watch Items

- Mixed full-runtime CCD rows were heavier than pure 2D or pure 3D rows at
  `1024` bodies. This is expected because mixed mode exercises both dimensions
  and the mixed broad phase. Revisit if the gap grows after the 3D allocation
  RCA or if mixed CCD becomes an immediate release-critical target.
- Pure 3D swept-sphere dense mesh target rows are visible through
  `MeshQuery3DTriangleScalingBenchmarks` and still scale linearly with triangle
  candidate volume: `550.1 us`, `2.145 ms`, and `8.638 ms` at `128`, `512`, and
  `2048` triangles. The lower-bound ordering optimization was adopted only for
  convex-source sweeps against concave mesh targets, where it proved a real
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

Exact 3D contact-response and canonical OBB throughput are closed with shared
full-domain owners and no competing answer paths. Dense concave mesh/mesh
throughput remains experimental capacity guidance; prefer primitive, convex,
compound, or partitioned static-concave authoring. The mixed discrete
broad-phase refresh threshold remains a lower-priority capacity signal because
the CCD-owned preparation, search, response, handoff, reset, and completion
paths remain allocation-free. Keep this document as the intake bucket for future
measured signals; promote broader work into a dated feature plan when the scope
outgrows a focused patch.
