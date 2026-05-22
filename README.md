# Gravitas

![Gravitas Icon](https://raw.githubusercontent.com/mrdav30/gravitas/main/icon.png)

[![Build](https://github.com/mrdav30/Gravitas/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/mrdav30/Gravitas/actions/workflows/build-and-test.yml)
[![Coverage](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fmrdav30.github.io%2FGravitas%2FSummary.json&query=%24.summary.linecoverage&suffix=%25&label=coverage&color=brightgreen)](https://mrdav30.github.io/Gravitas/)
[![NuGet](https://img.shields.io/nuget/v/Gravitas.svg)](https://www.nuget.org/packages/Gravitas)
[![NuGet Lean](https://img.shields.io/nuget/v/Gravitas.Lean.svg?label=nuget%20lean)](https://www.nuget.org/packages/Gravitas.Lean)
[![License](https://img.shields.io/github/license/mrdav30/Gravitas.svg)](https://github.com/mrdav30/Gravitas/blob/main/LICENSE)
[![Frameworks](https://img.shields.io/badge/frameworks-netstandard2.1%20%7C%20net8.0-512BD4.svg)](https://github.com/mrdav30/Gravitas)

**Deterministic physics for lockstep simulations and games.**

Gravitas is an engine-agnostic fixed-point physics prototype for simulation-heavy .NET projects. It is designed to sit above the LSF stack:

- `FixedMathSharp` for deterministic fixed-point math.
- `SwiftCollections` for low-allocation collections and pools.
- `GridForge` for explicit voxel worlds and spatial partitioning.
- `Chronicler.Core` for deterministic state transfer.

## Prototype Status

Gravitas is preparing for alpha. The current library is intentionally experimental, 3D-focused, and not API-stable. Heavy redesigns are expected where they improve deterministic behavior, physics correctness, runtime complexity, or engine-agnostic integration.

The unit test project now has focused runtime, settings, query, partition, and coroutine coverage. The benchmark project has initial context lifecycle, registration/partitioning, and query-service benchmarks. Use this README as current orientation, and use [AGENTS.md](AGENTS.md) for detailed contributor guidance.

## Why Gravitas?

- Deterministic runtime math through `Fixed64`, `Vector2d`, `Vector3d`, and `FixedQuaternion`.
- Engine-agnostic host boundary through `IMatterAgent` instead of direct renderer, ECS, or Unity coupling.
- Grid-backed broad-phase partitioning through `GridForge` `GridWorld`, voxel tracing, and `PhysicsPartition`.
- Runtime systems for bodies, colliders, collision pairs, collision detection/response, raycasts, circlecasts, and physics settings.
- A future direction toward first-class 2D physics and mixed 2D/3D simulations where 2D and 3D bodies can interact through explicit dimensional rules.

## Install

```bash
dotnet add package Gravitas
```

Gravitas targets `netstandard2.1` and `net8.0`.

### Package Variants

Gravitas is configured for two package variants:

- `Gravitas`: Includes `MemoryPack` and depends on the standard `FixedMathSharp`, `SwiftCollections`, `SwiftCollections.FixedMathSharp`, `GridForge`, and `Chronicler.Core` packages.
- `Gravitas.Lean`: Excludes the direct `MemoryPack` package and swaps to the lean dependency chain: `FixedMathSharp.Lean`, `SwiftCollections.Lean`, `SwiftCollections.FixedMathSharp.Lean`, `GridForge.Lean`, and `Chronicler.Core.Lean`.

Both variants are intended to expose the same core physics API. The difference is whether built-in MemoryPack support and the standard dependency chain are present.

Install via NuGet:

```bash
dotnet add package Gravitas
dotnet add package Gravitas.Lean
```

If you build from source, the repository provides matching configurations:

- `Release` builds the standard `Gravitas` package.
- `ReleaseLean` builds the `Gravitas.Lean` package.

For local development against the repository, reference the project directly:

```xml
<ItemGroup>
  <ProjectReference Include="path/to/Gravitas/src/Gravitas/Gravitas.csproj" />
</ItemGroup>
```

## Mental Model

Gravitas is now centered around explicit world-context ownership:

1. A host creates or attaches a `GravitasWorldContext`, which owns an explicit `GridForge.Grids.GridWorld`.
2. Host objects expose deterministic transform and world context access through `IMatterAgent`.
3. `GravitasWorldContext` owns fixed-step clock state, settings, physical environment values, lifecycle hooks, and context-local services.
4. `GravitasPhysicsService` owns body/collider registration, collider ID lookup, collision-pair pooling, and physics lifecycle work for one context.
5. `GravitasCollisionService` maps colliders into GridForge voxels and activates `PhysicsPartition` instances for collision checks.
6. `GravitasRaycastService`, `GravitasCirclecastService`, and `GravitasCoroutineService` own query and coroutine state per context.
7. `StiffBody` owns simulated body state such as position, rotation, velocity, acceleration, mass, drag, friction, grounding, and Chronicler state recording.
8. `LSCollider` and primitive collider types own shape data, bounds, layers, trigger/contact events, and GridForge partition coordinates.

Typical integration creates or attaches a context, initializes bodies and colliders against agents bound to that context, then advances the simulation through `Simulate()`, `LateSimulate()`, `Visualize()`, and `LateVisualize()` according to the host's fixed-frame loop.

## Main Systems

| Area | What it does | Start here |
| --- | --- | --- |
| Core runtime | Context-owned physics service, body state, and host agent boundary | [`src/Gravitas/Core`](src/Gravitas/Core) and [`src/Gravitas/Runtime`](src/Gravitas/Runtime) |
| Colliders | Collider base class, primitive shapes, mesh support, bounds, and layer behavior | [`src/Gravitas/Colliders`](src/Gravitas/Colliders) |
| Collision handling | Shape-pair checks, contact data, collision pairs, and response logic | [`src/Gravitas/CollisionHandling`](src/Gravitas/CollisionHandling) |
| Partitions | GridForge-backed physics partitions used by collision distribution | [`src/Gravitas/Partitions`](src/Gravitas/Partitions) |
| Raycasting | Raycast and circlecast query support | [`src/Gravitas/Raycasting`](src/Gravitas/Raycasting) |
| Settings | Frame rate, collision matrix, pooling switch, and settings save helpers | [`src/Gravitas/Settings`](src/Gravitas/Settings) |
| Support | Fixed transforms, layers, lifecycle hooks, coroutines, and transient state helpers | [`src/Gravitas/Support`](src/Gravitas/Support) |

## Repository Map

| Path | Purpose |
| --- | --- |
| [`src/Gravitas`](src/Gravitas) | Main library project. |
| [`tests/Gravitas.Tests`](tests/Gravitas.Tests) | xUnit v3 test project with focused runtime/settings/query coverage. |
| [`tests/Gravitas.Benchmarks`](tests/Gravitas.Benchmarks) | BenchmarkDotNet project scaffold and benchmark runner. |
| [`docs/feature-work/prototype`](docs/feature-work/prototype) | Historical Unity-oriented prototype/reference code. Not the source of truth. |
| [`.github/workflows`](.github/workflows) | CI, coverage, release, NuGet publish, Discord, and wiki-sync workflows. |

## Build And Test

```bash
dotnet restore Gravitas.slnx
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

Validate the lean package path when changing package references, serialization, or conditional MemoryPack behavior:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

For focused unit-test work:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release
```

Release builds generate NuGet packages because `GeneratePackageOnBuild` is enabled.

## Benchmarks

The benchmark project includes initial physics hot-path measurements for context lifecycle, body/collider registration, partitioning, simulation, and query services.

List available benchmark selections:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
```

Run all benchmarks once benchmark classes exist:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all
```

See the [benchmark README](tests/Gravitas.Benchmarks/README.md) for runner details and benchmark authoring notes.

## Documentation

- [AGENTS.md](AGENTS.md) is the main contributor guide for deterministic, performance-sensitive, and physics-design work.
- [`docs/feature-work/prototype`](docs/feature-work/prototype) contains historical prototype code and Unity-oriented reference material.

If behavior changes, keep code, tests, this README, and benchmark documentation aligned.

## Compatibility

- `netstandard2.1`
- `net8.0`
- Windows, Linux, and macOS host environments supported by .NET

## Contributing

Contributions are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) and [AGENTS.md](AGENTS.md) before opening a pull request.

Prefer focused changes with release-mode validation. Determinism, physics correctness, low time complexity, and allocation behavior are first-order design constraints.

## License

Gravitas is licensed under the MIT License. See [LICENSE](LICENSE), [NOTICE](NOTICE), and [COPYRIGHT](COPYRIGHT) for the project terms and attribution details.
