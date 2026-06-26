# Runtime Allocation Hardening Plan

Status: Done.

## Purpose

The Phase 9 BenchmarkDotNet baseline gave Gravitas its first allocation view of the context-first runtime. Context creation and body/collider construction naturally allocate, but steady-state simulation and query paths should move toward zero avoidable allocations before alpha.

## Baseline Evidence

Captured with:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all -j Short -i --exporters json
```

Short-run artifacts are written under `BenchmarkDotNet.Artifacts/results/`.

Observed steady-state allocation signals:

| Benchmark | Allocated |
| --- | ---: |
| `CollisionPartitionBenchmarks.SimulatePartitionedDynamicSpheres` | 20.78 KB/op |
| `QueryServiceBenchmarks.RaycastAllAcrossPopulatedContext` | 1,448 B/op |
| `QueryServiceBenchmarks.CircleCastAllAcrossPopulatedContext` | 296 B/op |
| `QueryServiceBenchmarks.RaycastAcrossTwoOverlappingContexts` | 2,896 B/op |

## Original Suspected Sources

- Query APIs currently expose `IEnumerable<LSRaycastHit>` and use `yield return`, which allocates enumerator state for hot query paths.
- Query hit sorting uses `Comparer<LSRaycastHit>.Create(...)` per call. The comparison delegate is static, but the comparer object is still created repeatedly.
- Simulation allocation likely includes per-body grounding/query work, collision distribution, or temporary GridForge/SwiftCollections buffers. It needs measurement before changing behavior.

## Completed Phases

### Phase 1: Query API Allocation

- Added allocation-focused benchmarks for `RaycastAll`, `CircleCastAll`, and directional `CircleCast`.
- Replaced enumerable hot-path APIs with caller-provided `SwiftList<LSRaycastHit>` result buffers.
- Removed convenience enumerable wrappers from the authoritative query surface.
- Replaced comparer-based sorting with an allocation-free in-place distance sorter.
- Verified query benchmarks report no managed allocation in the short allocation smoke run.

### Phase 2: Grounding And Simulation Allocation

- Split benchmarks for `SolidBody.LateSimulate`, grounding `CircleCast`, collision partition distribution, and active pair processing.
- Identified active-pair late simulation repartitioning as the remaining allocation source.
- Replaced `PartitionObject`'s `GridTracer.GetCoveredVoxels(...)` enumeration with direct spatial-cell and voxel scanning backed by reusable context sets.
- Added steady-state allocation benchmarks for grounded and moving-body scenarios.

### Phase 3: Guardrails

- Added benchmark documentation with expected no-allocation smoke targets for query and simulation allocation aliases.
- Confirmed existing CI build coverage compiles `tests/Gravitas.Benchmarks` because `Gravitas.slnx` includes the benchmark project.
- Deferred allocation assertion tests until the narrow deterministic cases are stable enough to avoid runtime noise.

## 2026-05-23 Evidence

Captured with:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- query-service simulation-allocation --filter "*" -j Short -i --exporters json
```

Short-run BenchmarkDotNet output reported no managed allocation for the hardened
steady-state paths:

| Benchmark | Allocated |
| --- | ---: |
| `QueryServiceBenchmarks.RaycastAllAcrossPopulatedContext` | 0 B/op |
| `QueryServiceBenchmarks.CircleCastAllAcrossPopulatedContext` | 0 B/op |
| `QueryServiceBenchmarks.DirectionalCircleCastAcrossPopulatedContext` | 0 B/op |
| `QueryServiceBenchmarks.RaycastAcrossTwoOverlappingContexts` | 0 B/op |
| `SimulationAllocationBenchmarks.SolidBodyLateSimulateOnly` | 0 B/op |
| `SimulationAllocationBenchmarks.GroundingCircleCastOnly` | 0 B/op |
| `SimulationAllocationBenchmarks.CollisionPartitionDistributionOnly` | 0 B/op |
| `SimulationAllocationBenchmarks.ActivePairProcessingLateSimulate` | 0 B/op |

## Acceptance Criteria

- [x] Query all-style APIs offer an allocation-free hot path.
- [x] Grounding and partitioned simulation do not allocate per body in the steady state.
- [x] Benchmark docs record the baseline and the improved allocation budgets.
- [x] `Release` and `ReleaseLean` build/test remain green.

## Verification

2026-05-23:

- `dotnet build Gravitas.slnx --configuration Release`
- `dotnet test Gravitas.slnx --configuration Release --no-build`
- `dotnet build Gravitas.slnx --configuration ReleaseLean`
- `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build`
