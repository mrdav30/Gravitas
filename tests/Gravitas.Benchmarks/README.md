# Gravitas Benchmarks

This project benchmarks the path-request and navigation hot paths using [BenchmarkDotNet](https://benchmarkdotnet.org/).

The suite is layered so that regression in a high-level steering or cache benchmark can be diagnosed
by running the lower-level surveyor or guide-resolution benchmark in isolation.

## Requirements

- .NET 8 SDK
- `Release` configuration (mandatory — avoid `Debug` for performance measurements)

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

Aliases are derived from the benchmark class name. `Benchmarks` is stripped and the remaining words
are joined with `-`. Pass one or more aliases as leading arguments before any BenchmarkDotNet flags.

| Alias | Benchmark class |
| --- | --- |
| N/A | N/A |

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- a-star-path-request
```

Multiple aliases run together:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- a-star-path-request guide-cache
```

### Filter to specific methods

Add `--filter` after the alias. The filter pattern is forwarded to BenchmarkDotNet's method-name
filter and supports `*` wildcards.

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --list flat
```

### Fast development check (InProcessShortRunConfig)

`InProcessShortRunConfig` is registered for quick local smoke runs. Use it during development to
verify benchmark code compiles and produces plausible numbers without a full benchmark run.

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --config InProcessShortRunConfig
```

Do not treat results from short-run mode as canonical measurements.

## Benchmark Suite Structure

N/A

## Design Principles

- Each benchmark class uses its own `BenchmarkPathFixture` instances to prevent cross-contamination
  between benchmark groups.
- `[MemoryDiagnoser]` is applied to all benchmark classes.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by default. Archive the
JSON or markdown reports before making hot-path changes so regressions can be compared against
a known state.

## CI Guidance

CI should at minimum compile the benchmark project in `Release`:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood. When you are ready to
add performance gates, use BenchmarkDotNet's `--compare` or a stored baseline artifact rather than
raw timing thresholds, which are sensitive to runner hardware.
