# Gravitas

![Gravitas Icon](https://raw.githubusercontent.com/mrdav30/gravitas/main/icon.png)

[![Build](https://github.com/mrdav30/Gravitas/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/mrdav30/Gravitas/actions/workflows/build-and-test.yml)
[![Coverage](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fmrdav30.github.io%2FGravitas%2FSummary.json&query=%24.summary.linecoverage&suffix=%25&label=coverage&color=brightgreen)](https://mrdav30.github.io/Gravitas/)
[![NuGet](https://img.shields.io/nuget/v/Gravitas.svg)](https://www.nuget.org/packages/Gravitas)
[![NuGet Lean](https://img.shields.io/nuget/v/Gravitas.Lean.svg?label=nuget%20lean)](https://www.nuget.org/packages/Gravitas.Lean)
[![License](https://img.shields.io/github/license/mrdav30/Gravitas.svg)](https://github.com/mrdav30/Gravitas/blob/main/LICENSE)
[![Frameworks](https://img.shields.io/badge/frameworks-netstandard2.1%20%7C%20net8.0-512BD4.svg)](https://github.com/mrdav30/Gravitas)

**Deterministic physics engine for lockstep simulations and games.**

Gravitas gives simulation-heavy .NET projects a fixed-point physics stack without tying them to a renderer, ECS, or game framework.

The README is the front door. The deeper integration notes live in the [wiki](docs/wiki/Home.md), starting with the [architecture overview](docs/wiki/Overview.md).

## Why Gravitas?

- Deterministic runtime math through `FixedMathSharp` types such as `Fixed64`, `Vector3d`, and `FixedQuaternion`.
- Voxel-backed world representation through `GridForge`, with explicit chart registration and context-owned runtime state.
- Multi-targeted builds for `netstandard2.1` and `net8.0`.

## Install

```bash
dotnet add package Gravitas
```

Gravitas targets `netstandard2.1` and `net8.0`.

### Package Variants

Gravitas is published in two build variants so you can choose between built-in `MemoryPack` support and a leaner dependency set:

- `Gravitas`: Includes `MemoryPack` and depends on the standard `FixedMathSharp`, `SwiftCollections`, `GridForge`, and `Chronicler.Core` packages. This is the best default choice for most .NET applications, especially if you want the MemoryPack-backed Chronicler transport available out of the box.
- `Gravitas.Lean`: Excludes the `MemoryPack` package, swaps to `FixedMathSharp.NoMemoryPack`, `SwiftCollections.Lean`, `GridForge.Lean`, and `Chronicler.Core.Lean`, and omits MemoryPack-specific source files. Choose this when you do not need built-in MemoryPack serialization, when you prefer a different serializer, or when you want the leanest dependency surface.

Both variants expose the same core pathing and navigation API. The main difference is whether `MemoryPack` and the standard dependency chain are included.

Install via NuGet:

- Standard package:

  ```bash
  dotnet add package Gravitas
  ```

- Lean package:

  ```bash
  dotnet add package Gravitas.Lean
  ```

If you build from source, the repository provides matching release configurations:

- `Release` builds the standard `Gravitas` package.
- `ReleaseLean` builds the `Gravitas.Lean` package.

For local development against the repository, reference the project directly:

```xml
<ItemGroup>
  <ProjectReference Include="path/to/Gravitas/src/Gravitas/Gravitas.csproj" />
</ItemGroup>
```

## Mental Model

N/A

## Quick Start

N/A

## Main Systems

| Area | What it does | Start here |
| --- | --- | --- |
| N/A | N/A| N/A |

## Repository Map

| Path | Purpose |
| --- | --- |
| N/A | N/A |

## Build And Test

```bash
dotnet restore Gravitas.slnx
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

For focused work, run the matching test area first:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~N/A
```

Release builds generate NuGet packages because `GeneratePackageOnBuild` is enabled.

## Benchmarks

The benchmark suite measures path-request and navigation hot paths: N/A

List available benchmark selections:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
```

Run a specific group:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- guide-cache
```

See the [benchmark README](tests/Gravitas.Benchmarks/README.md) for the full command reference and suite design notes.

## Documentation

Start with the [wiki home](docs/wiki/Home.md) if you are evaluating the project, or jump straight into:

- [Overview](docs/wiki/Overview.md) for the runtime model

The wiki is intentionally more detailed than this README. If behavior changes, keep code, tests, README, and the relevant wiki page aligned.

## Compatibility

- `netstandard2.1`
- `net8.0`
- Windows, Linux, and macOS host environments supported by .NET

## Contributing

Contributions are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request, and prefer focused changes with release-mode validation.

For issues, feature requests, or questions, use the repository issue tracker. Community discussion is also available on the official [Discord server](https://discord.gg/mhwK2QFNBA).

## License

Gravitas is licensed under the MIT License. See [LICENSE](LICENSE), [NOTICE](NOTICE), and [COPYRIGHT](COPYRIGHT) for the project terms and attribution details.
