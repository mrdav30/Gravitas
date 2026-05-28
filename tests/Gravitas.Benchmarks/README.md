# Gravitas Benchmarks

This project is the BenchmarkDotNet scaffold for Gravitas physics hot paths.

The runner, alias catalog, and deterministic fixture helpers are in place. Initial benchmark classes cover context lifecycle, registration/partitioning, simulation, query-service paths, and diagnostics.

## Requirements

- .NET 8 SDK
- `Release` configuration for meaningful measurements

Avoid measuring `Debug` builds except when diagnosing benchmark setup failures.

## Running

### List available benchmark selections

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
```

### Run all benchmarks

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all
```

### Run a selection by alias

Aliases are derived from benchmark class names. `Benchmarks` or `Benchmark` is stripped, and the remaining words are joined with `-`.

For a class named `CollisionDetectionBenchmarks`, the selection alias is `collision-detection`:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collision-detection
```

Multiple aliases can run together:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collision-detection partitioning
```

### Forward BenchmarkDotNet arguments

Arguments after the selection are forwarded to BenchmarkDotNet:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --list flat
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collision-detection --filter "*Sphere*"
```

### Fast development check

Use BenchmarkDotNet's short in-process job for quick local smoke runs. This verifies benchmark code compiles and produces plausible output without a full run:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all -j Short -i
```

Do not treat short-run numbers as canonical measurements.

## Suggested Benchmark Areas

Start with hot paths that can be isolated and repeated deterministically:

- `GravitasWorldContext` simulation phases and service phase ordering.
- `GravitasPhysicsService` body/collider registration and collision-pair ownership.
- `GravitasCollisionService` partitioning and partition cleanup.
- `CollisionDetection` shape-pair checks.
- `CollisionResponse` contact resolution.
- continuous collision detection policy and swept movement cost.
- collider shape-state rebuilds, capsule derived state, compound aggregate bounds, and mesh validation/BVH construction.
- `GravitasRaycastService` and `GravitasCircleQueryService` query gathering, filtering, and result ordering.
- Mesh collider preprocessing and convex mesh limits.
- Pooling and allocation behavior for collision pairs, partitions, and temporary collections.
- Diagnostics disabled overhead and enabled capture cost for event hooks and debug draw commands.

## Authoring Guidelines

- Put benchmark classes in the `Gravitas.Benchmarks` namespace.
- Prefer one benchmark class per subsystem or scenario group.
- Apply `[MemoryDiagnoser]` to benchmark classes unless there is a specific reason not to.
- Use deterministic fixtures and fixed seeds. Do not use ambient randomness in measured paths.
- Create isolated `GravitasWorldContext` instances for measured scenarios, or use `BenchmarkEnvironment.PrepareWorld(...)` when a benchmark only needs raw GridForge setup.
- Reset or dispose context/world state between benchmark cases so measurements do not depend on previous cases.
- Capture both throughput and allocation impact when changing hot-path collections, pooling, collision dispatch, or broad-phase behavior.

Keep support helpers physics-specific. Remove copied template helpers when they stop serving a Gravitas benchmark scenario.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by default. Archive the JSON or markdown reports before changing algorithms so regressions can be compared against known results.

## Allocation Smoke Targets

For quick allocation checks around the current steady-state hot paths, run:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- query-service simulation-allocation continuous-collision collision-detection collision-response partition-culling diagnostics --filter "*" -j Short -i --exporters json
```

The short in-process job is not canonical timing evidence, but it is useful for
catching obvious managed allocations. These scenarios should report no managed
allocation in steady-state paths unless noted below:

BenchmarkDotNet short in-process runs can occasionally report `1 B/op` noise on
otherwise allocation-guarded paths. Treat repeatable non-zero values as a reason
to add or tighten explicit allocation tests before changing the algorithm.

| Alias | Covered paths |
| --- | --- |
| `query-service` | `RaycastAll`, `OverlapCircleAll`, directional `OverlapCircleInDirection`, and overlapping-context queries. |
| `simulation-allocation` | `StiffBody.LateSimulate`, grounding raycast probes, collision partition distribution, and active-pair late simulation. |
| `continuous-collision` | Discrete fast body movement baseline and opt-in CCD sweep/clamp against thin static geometry. |
| `collision-detection` | prepared primitive pairs, non-SAT primitive pairs, primitive manifold generation, cuboid face-manifold generation, cuboid SAT, mesh/cylinder, mesh/cuboid, mesh/mesh, and compound/primitive checks. |
| `collision-response` | manifold response solver cost across single-contact and face-manifold cases, with pair-count scaling. |
| `diagnostics` | Disabled/enabled force and torque event hooks plus disabled/enabled collider debug draw capture. |
| `partition-culling` | dynamic collider repartitioning after teleports, direct partition add/remove churn, and culled-pair invalidation after movement. |

Collider shape work has a focused selection:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- collider-shape --filter "*" -j Short -i --exporters json
```

## CI Guidance

CI should at minimum compile the benchmark project in `Release`. The normal
`Gravitas.slnx` build already includes `tests/Gravitas.Benchmarks`; use this
direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood. When performance gates are introduced, prefer BenchmarkDotNet comparison support or stored baseline artifacts over raw timing thresholds, which are sensitive to runner hardware.
