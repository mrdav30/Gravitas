# Gravitas Benchmarks

This project is the BenchmarkDotNet scaffold for Gravitas physics hot paths.

The runner, alias catalog, short-run config, and deterministic fixture helpers are in place. There are currently no authored benchmark classes, so `list` may produce an empty selection list until the first `[Benchmark]` types are added.

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

`InProcessShortRunConfig` is available for quick local smoke runs. Use it to verify benchmark code compiles and produces plausible output without a full run:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --config InProcessShortRunConfig
```

Do not treat short-run numbers as canonical measurements.

## Suggested Benchmark Areas

Start with hot paths that can be isolated and repeated deterministically:

- `PhysicsManager` simulation phases and body/collider registration.
- `CollisionManager` partitioning and partition cleanup.
- `CollisionDetection` shape-pair checks.
- `CollisionResponse` contact resolution.
- `Raycaster` and `Circlecaster` query gathering, filtering, and result ordering.
- Mesh collider preprocessing and convex mesh limits.
- Pooling and allocation behavior for collision pairs, partitions, and temporary collections.

## Authoring Guidelines

- Put benchmark classes in the `Gravitas.Benchmarks` namespace.
- Prefer one benchmark class per subsystem or scenario group.
- Apply `[MemoryDiagnoser]` to benchmark classes unless there is a specific reason not to.
- Use deterministic fixtures and fixed seeds. Do not use ambient randomness in measured paths.
- Use `BenchmarkEnvironment.PrepareWorld(...)` to create isolated `GridWorld` instances and suppress logging noise.
- Reset or dispose world state between benchmark cases so measurements do not depend on previous cases.
- Capture both throughput and allocation impact when changing hot-path collections, pooling, collision dispatch, or broad-phase behavior.

Some support helpers were carried over from earlier templates. Retire or rename those helpers when the first physics-specific benchmark classes are added.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by default. Archive the JSON or markdown reports before changing algorithms so regressions can be compared against known results.

## CI Guidance

CI should at minimum compile the benchmark project in `Release`:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood. When performance gates are introduced, prefer BenchmarkDotNet comparison support or stored baseline artifacts over raw timing thresholds, which are sensitive to runner hardware.
