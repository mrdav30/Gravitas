# Runtime Allocation Hardening Plan

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

## Suspected Sources

- Query APIs currently expose `IEnumerable<LSRaycastHit>` and use `yield return`, which allocates enumerator state for hot query paths.
- Query hit sorting uses `Comparer<LSRaycastHit>.Create(...)` per call. The comparison delegate is static, but the comparer object is still created repeatedly.
- Simulation allocation likely includes per-body grounding/query work, collision distribution, or temporary GridForge/SwiftCollections buffers. It needs measurement before changing behavior.

## Proposed Phases

### Phase 1: Query API Allocation

- Add allocation-focused benchmarks for `RaycastAll`, `CircleCastAll`, and directional `CircleCast`.
- Replace enumerable hot-path APIs with context-owned or caller-provided result buffers, likely `SwiftList<LSRaycastHit>`.
- Keep any convenience enumerable wrappers out of authoritative simulation paths.
- Make raycast and circlecast comparers static readonly instances.
- Verify query benchmarks trend toward zero steady-state allocation.

### Phase 2: Grounding And Simulation Allocation

- Split benchmarks for `StiffBody.LateSimulate`, grounding `CircleCast`, collision partition distribution, and active pair processing.
- Identify the allocation source before changing algorithm shape.
- Remove per-body/per-frame temporary allocations by reusing context-owned buffers or caller-provided buffers.
- Add regression benchmarks for grounded and airborne bodies.

### Phase 3: Guardrails

- Add benchmark documentation with expected allocation budgets once the hot paths stabilize.
- Add CI build coverage for benchmark compilation and optional smoke execution.
- Consider a small allocation assertion test only for narrow deterministic cases where runtime variance will not make it flaky.

## Acceptance Criteria

- Query all-style APIs offer an allocation-free hot path.
- Grounding and partitioned simulation do not allocate per body in the steady state.
- Benchmark docs record the baseline and the improved allocation budgets.
- `Release` and `ReleaseLean` build/test remain green.
