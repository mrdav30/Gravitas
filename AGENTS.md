# Gravitas Contributor Guide

## Purpose

Gravitas is a framework-agnostic deterministic physics library for lockstep
simulations and games. It sits in the LSF stack after:

1. `FixedMathSharp` - deterministic Q32.32 fixed-point math.
2. `SwiftCollections` - low-allocation collections, pools, and spatial data
   structures.
3. `GridForge` - deterministic voxel worlds, spatial queries, occupants, and
   partition backing.
4. `Gravitas` - deterministic physics, collision, raycasting, and simulation
   orchestration.

This repository is still an experimental prototype preparing for alpha. Expect
heavy redesigns where correctness, deterministic behavior, physical plausibility,
or hot-path complexity demands them. Backward compatibility is not required
unless the user explicitly asks for it. Do not preserve weak APIs with adapters
or band-aid layers when a clean redesign is the right engineering move.

Current priorities:

1. Preserve deterministic behavior first.
2. Correct physics and collision behavior before broad refactors.
3. Keep runtime hot paths low-complexity and allocation-conscious from the start.
4. Support engine-agnostic hosts without Unity, renderer, or ECS coupling.
5. Build toward first-class 3D and 2D support, with an eventual mixed 2D/3D
   simulation model.
6. Establish unit tests and benchmarks before alpha hardening.

## Start Here

Read these in order before making non-trivial changes:

1. [`README.md`](README.md), while remembering that it is sparse and currently
   contains placeholder/stale text.
2. The stack context, when sibling repositories are available:
   - `../FixedMathSharp/AGENTS.md` and `../FixedMathSharp/README.md`
   - `../SwiftCollections/AGENTS.md` and `../SwiftCollections/README.md`
   - `../GridForge/AGENTS.md` and `../GridForge/README.md`
   - `../Chronicler/AGENTS.md` and `../Chronicler/README.md` when serialization
     behavior is involved.
3. [`src/Gravitas/Core/PhysicsManager.cs`](src/Gravitas/Core/PhysicsManager.cs)
   and [`src/Gravitas/Core/StiffBody.cs`](src/Gravitas/Core/StiffBody.cs).
4. The relevant source folder under [`src/Gravitas`](src/Gravitas).
5. The matching test or benchmark area under [`tests`](tests). The unit test
   project currently has no authored tests, so new behavior usually needs new
   tests.
6. [`src/Gravitas/Gravitas.csproj`](src/Gravitas/Gravitas.csproj),
   [`tests/Gravitas.Tests/Gravitas.Tests.csproj`](tests/Gravitas.Tests/Gravitas.Tests.csproj),
   and [`tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj`](tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj).

## Source Of Truth

When code, README, and generated docs disagree, prefer the code and project
files. The README references wiki pages and pathing/navigation concepts that do
not yet match this prototype.

Keep these aligned whenever behavior, public API, package shape, or developer
workflow changes:

- [`README.md`](README.md)
- [`tests/Gravitas.Tests`](tests/Gravitas.Tests)
- [`tests/Gravitas.Benchmarks`](tests/Gravitas.Benchmarks) when performance
  claims or hot paths change.
- Relevant workflow files under [`.github/workflows`](.github/workflows).

## Repository Map

| Path | Purpose | Notes |
| --- | --- | --- |
| [`src/Gravitas`](src/Gravitas) | Main library project | Multi-targets `netstandard2.1` and `net8.0`. |
| [`src/Gravitas/Core`](src/Gravitas/Core) | Simulation manager, body state, collision manager, host agent interface | Start here for architecture changes. |
| [`src/Gravitas/Colliders`](src/Gravitas/Colliders) | Collider base type, primitive colliders, physics mesh helpers | Shape logic is currently 3D-focused. |
| [`src/Gravitas/CollisionHandling`](src/Gravitas/CollisionHandling) | Collision detection, response, pairs, contact data | Determinism and ordering are high risk here. |
| [`src/Gravitas/Raycasting`](src/Gravitas/Raycasting) | Raycast and circlecast support | Keep result ordering stable. |
| [`src/Gravitas/Partitions`](src/Gravitas/Partitions) | GridForge-backed physics partitions | Tied to voxel ownership and pooling. |
| [`src/Gravitas/Settings`](src/Gravitas/Settings) | Physics settings and save helpers | Includes frame rate and layer collision matrix behavior. |
| [`src/Gravitas/Support`](src/Gravitas/Support) | Fixed transforms, layers, lifecycle hooks, coroutine scaffolding, transient state helpers | Keep engine-specific assumptions out. |
| [`tests/Gravitas.Tests`](tests/Gravitas.Tests) | xUnit v3 test project | Scaffold exists; no authored tests yet. |
| [`tests/Gravitas.Benchmarks`](tests/Gravitas.Benchmarks) | BenchmarkDotNet project | Scaffold exists; some docs/support names still reflect earlier pathing templates. |
| [`docs/feature-work/prototype`](docs/feature-work/prototype) | Historical/prototype Unity-oriented reference code | Useful context, not the source of truth. |

Ignore generated output when reviewing structure:

- `bin/`
- `obj/`
- `TestResults/`
- `artifacts/`
- `.vs/`
- `BenchmarkDotNet.Artifacts/`

## Runtime Architecture Snapshot

The current runtime is static-manager based and prototype-stage:

- `PhysicsManager` owns setup/initialize flow, frame count, fixed delta time,
  dynamic body storage, collider IDs, collision-pair pooling, simulation phases,
  and global physics settings.
- `StiffBody` owns simulated body state: position, rotation, visual
  interpolation state, velocity, acceleration, drag, friction, grounding,
  transforms, and Chronicler state recording.
- `IMatterAgent` is the host boundary. Hosts provide a `GridWorld`, a
  `FixedTransform`, hierarchy information, and interaction state without tying
  Gravitas to a game engine.
- `LSCollider` and primitive subclasses own shape state, bounds, layers,
  trigger/contact events, GridForge partition coordinates, and collision-pair
  references.
- `CollisionManager` maps colliders into GridForge voxels via `GridWorld`,
  `GridTracer`, `WorldVoxelIndex`, and `PhysicsPartition`, using
  `SwiftCollections` pools and buckets.
- `CollisionDetection`, `CollisionResponse`, `CollisionPair`, `ContactPoint`,
  and context structs form the narrow-phase and response layer.
- `Raycaster` and `Circlecaster` are query systems layered on top of collider
  state and manager versions.

Treat this as a working prototype, not final architecture. Static manager state,
global counters, partition reuse, collision-pair ownership, and simulation phase
ordering are high-risk areas.

## 2D, 3D, And Mixed-Dimension Direction

Gravitas is currently 3D-focused. Some body state already separates a 2D ground
position from height, but that is not the same as a complete 2D physics model.

When adding or redesigning dimension-sensitive behavior:

- Do not bake in permanent assumptions that all simulations are 3D, y-up, or
  XZ-ground-plane unless the API explicitly says so.
- Model 2D as first-class physics behavior, not as accidental 3D with one axis
  ignored.
- Before implementing mixed 2D/3D collision, define the embedding rule: plane,
  layer, thickness, projection volume, contact manifold shape, and how 3D bodies
  exchange impulses with 2D bodies.
- Keep dimensional choices explicit in public APIs and tests.
- Avoid naming that implies Unity-specific `Rigidbody` behavior. `StiffBody`
  is the current prototype term; future redesigns may rename or split it if that
  clarifies body semantics.

## Physics Quality Bar

Future agents should bring senior simulation-physics judgment, not just general
C# instincts.

Prefer:

- physically explainable models for mass, inertia, impulse, torque, drag,
  friction, restitution, damping, and sleep/rest behavior.
- known collision and response algorithms adapted to deterministic fixed-point
  math when they fit the problem, rather than bespoke approximations.
- explicit units and invariants for constants and thresholds.
- stable contact ordering, pair identity, and island/partition traversal.
- correctness tests for edge cases before optimizing the same behavior.

Avoid:

- arbitrary magic constants without documentation or tests.
- silent clamping that hides unstable physics instead of fixing the model.
- visual-only corrections in runtime simulation state.
- fixes that only handle the current example while leaving the general collision
  or integration issue unresolved.
- changes that improve one scenario by making another shape pair or dimension
  mode less physically coherent.

## External Dependencies

The main external packages shape how this project should be changed:

- `FixedMathSharp`: use `Fixed64`, `Vector2d`, `Vector3d`, `FixedQuaternion`,
  `Fixed3x3`, bounds, and deterministic math helpers.
- `SwiftCollections`: prefer `SwiftBucket`, `SwiftList`, `SwiftQueue`,
  `SwiftStack`, `SwiftHashSet`, object pools, and related low-allocation types in
  runtime or hot-path code.
- `GridForge`: use explicit `GridWorld` ownership, voxel tracing, world voxel
  identities, partitions, and spatial queries. Do not reintroduce hidden
  process-global grid state.
- `Chronicler.Core`: use explicit `IRecordable.RecordData(...)` for runtime
  state transfer into existing host-created objects.
- `MemoryPack`: standard package support only. Lean builds should avoid direct
  MemoryPack dependencies or isolate them behind `GRAVITAS_DISABLE_MEMORYPACK`
  compatible files.

Do not casually replace these with standard floating-point, general-purpose
collections, or non-deterministic alternatives.

## Determinism Rules

Any change that affects simulation order, iteration order, rounding, collision
pair identity, partition traversal, contact generation, integration, or update
timing is high risk.

Always prefer:

- `Fixed64`, `Vector2d`, `Vector3d`, and `FixedQuaternion` over `float`,
  `double`, and `System.Numerics` in deterministic runtime logic.
- frame-based reasoning through `PhysicsManager.FrameRate`,
  `PhysicsManager.DeltaTime`, and `PhysicsManager.FrameCount`.
- stable and explicit ordering when traversing colliders, partitions, voxels,
  contacts, collision pairs, raycast hits, or pooled collections.
- deterministic seeds with explicit ownership for any randomness.
- host-owned `GridWorld` state over hidden ambient world lookups.

Avoid introducing:

- floating-point math in simulation logic.
- time-dependent APIs such as `DateTime.Now`, timers, or wall-clock scheduling
  in runtime code.
- iteration behavior that depends on platform-specific hash ordering.
- LINQ or iterator allocations in per-frame, per-collider, per-voxel, or
  per-contact paths.
- background threading that changes observable simulation order.

## Performance Guidance

Always prefer optimized, low time-complexity code. No band-aid solutions.

Optimization work should focus on proven hot paths and data-structure behavior,
but new runtime systems should start lean rather than assuming a later cleanup.

Likely hotspots:

- `PhysicsManager.Simulate`, `LateSimulate`, `Visualize`, and collider
  assimilation/dessimilation.
- GridForge partitioning in `CollisionManager.PartitionObject` and
  `ClearPartitionedObject`.
- collision-pair creation, culling, notification, deactivation, and pooling.
- narrow-phase shape checks in `CollisionDetection`.
- contact resolution in `CollisionResponse`.
- raycast/circlecast candidate gathering, ordering, and filtering.
- mesh collider preprocessing and convex mesh limits.

Optimization rules:

- Preserve physics correctness before reducing allocations.
- Choose data structures by complexity and access pattern, not habit.
- Pool only when lifetime and ownership are obvious and testable.
- Clear or return pooled collections on every path, including early exits.
- Avoid resize spikes in hot paths; if growth is unavoidable, make capacity
  strategy explicit and benchmark-sensitive.
- Do not broaden a hot-path refactor across unrelated systems in one change set.
- Add or update benchmarks for meaningful changes to collision dispatch,
  partitioning, raycasting, pooling, or broad-phase behavior.

## Serialization Status

Serialization is experimental and incomplete.

Current behavior observed in source:

- Runtime body and collider state use explicit Chronicler
  `IRecordable.RecordData(...)` methods.
- Settings and layer helpers currently use MemoryPack attributes directly;
  verify the Lean build when touching them.
- The project has `Release` and `ReleaseLean` configurations. `ReleaseLean`
  defines `GRAVITAS_DISABLE_MEMORYPACK` and is expected to keep the same core
  physics API without built-in MemoryPack dependency.

Important rules:

- Gravitas should load into host-created runtime shells. Do not turn Chronicler
  into a construct-from-data object factory.
- Host bindings such as engine objects, renderers, and external transforms should
  remain host-owned. Serialize stable state or explicit links, not framework
  objects.
- Keep JSON and MemoryPack behavior aligned when both are supported.
- If serialized fields, defaults, or load semantics change, add tests that cover
  save and populate flows.
- When adding MemoryPack-specific code, ensure the Lean package still compiles.

## Coding Style And Documentation

Observed project conventions:

- `LangVersion` is `11.0`.
- `ImplicitUsings` are disabled.
- Library nullable context is enabled.
- Tests use nullable context enabled; benchmarks currently disable nullable.
- XML documentation output is generated for the library, while `.editorconfig`
  silences `CS1591`.
- Namespace-folder matching is not enforced.
- Source files currently mix region-heavy prototype style with direct static
  manager code. Match nearby style for focused edits.

Contributor expectations:

- Add or improve XML `<summary>` tags for public and externally meaningful
  internal APIs when touching them.
- Add brief comments only where logic, invariants, or edge conditions are hard
  to infer from the code alone.
- Preserve ASCII unless the file already requires otherwise.
- Keep comments factual. Explain why the physics, ordering, or lifetime rule
  exists, not what a simple assignment does.
- Split reusable or generic infrastructure into focused files instead of burying
  it inside unrelated runtime classes.
- Prefer `SwiftCollections` over `System.Collections*` in library runtime code
  when a suitable SwiftCollections type exists. If a BCL collection is kept, the
  reason should be obvious from locality, API needs, or non-hot-path usage.
- Keep engine-specific and editor-only ideas in docs, samples, or host adapters,
  not in the core library.

## Testing Workflow

Use these baseline commands:

```bash
dotnet restore Gravitas.slnx
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

Also validate the lean package path when touching package references,
serialization, MemoryPack usage, or conditional compilation:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

For focused test work, prefer the unit test project:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release
```

Important notes:

- `tests/Gravitas.Tests` currently contains only the project file and coverage
  settings. Add the first authored tests alongside any behavior change.
- Building the library produces NuGet packages because `GeneratePackageOnBuild`
  is enabled.
- CI builds and tests `Release` and `ReleaseLean` on Ubuntu and Windows.
- Coverage workflow runs the xUnit project with `XPlat Code Coverage`.

## Benchmark Workflow

The benchmark project is scaffolded, but benchmark docs/support code still
contain pathing/navigation template names. Verify or update the benchmark target
before relying on a result.

List available benchmark selections:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
```

Run all benchmarks once meaningful benchmark classes exist:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all
```

For optimization work, capture a baseline before changing the algorithm and
compare the same benchmark after the change. Do not treat short-run benchmark
results as canonical performance evidence.

## Test Design Expectations

Because there are no authored unit tests yet, new tests should establish the
foundation for deterministic physics behavior.

Prioritize tests for:

- `PhysicsManager` setup, initialize, frame count, fixed delta time, settings,
  assimilation, dessimilation, and reset behavior.
- `StiffBody` force integration, velocity changes, drag/friction, grounding,
  rotation, transform helpers, rest state, and serialization.
- collider bounds, local/world transforms, layer filtering, trigger/contact
  events, parent/child collision exclusion, and partition lifecycle.
- collision detection for every supported shape pair, including edge-touching,
  full overlap, separated, degenerate, and rotated cases.
- collision response invariants such as conservation expectations, restitution,
  immovable bodies, kinematic bodies, and angular effects.
- GridForge partition interactions, voxel snapping, partition reuse, and stale
  partition cleanup.
- raycast/circlecast ordering and filtering.
- serialization round trips and populate-existing-instance semantics.
- deterministic replay: same initial state and inputs must produce the same
  state across repeated runs.

Use fixed-point expected values. Avoid fuzzy assertions unless the tolerance is
itself deterministic, documented, and justified by the algorithm.

## Recommended Change Workflow

For both humans and AI agents, use this order:

1. Read the relevant docs, source files, and project files.
2. Identify deterministic invariants, simulation phase effects, global/static
   state, and pooling ownership.
3. Decide whether the current design should be preserved or redesigned. Since
   alpha compatibility is not required, prefer the clean deterministic design
   over compatibility scaffolding.
4. Add or update focused tests that pin the intended behavior.
5. Make the smallest coherent code change that solves the real issue.
6. Add XML docs or clarifying comments while the code is open.
7. Run focused tests or at least compile the affected project.
8. Run the full `Release` suite before closing behavior work.
9. Run `ReleaseLean` validation when package shape, serialization, or
   MemoryPack-related code changed.
10. Update `README.md`, benchmark docs, or workflow docs if public behavior or
    developer workflow changed.

## Guidance For AI Agents

If you are an automated coding agent working in this repository:

- Do not trust high-level docs blindly; validate against code, project files,
  and tests.
- Do not broaden scope from one subsystem into another unless the change truly
  requires it.
- Call out any build or test failures explicitly, with exact file references.
- Treat static manager state, collider IDs, collision-pair ownership, partition
  reuse, pooled collections, settings, frame ordering, and GridForge world
  ownership as high-risk areas.
- Treat serialization boundaries and load semantics as high-risk areas. Avoid
  silently broadening populate-existing-instance loads into construct-from-data
  behavior.
- Prefer focused redesigns with tests over patches that preserve flawed
  behavior.
- If you change a public API or behavior, update tests and docs in the same
  pass.
- If you add comments, comment the invariant or reason, not the syntax.
- Do not leave generic helpers buried inside unrelated classes when they can
  stand alone as reusable support types.
- Reach for `SwiftCollections` first before introducing `System.Collections`,
  `System.Collections.Generic`, or `System.Collections.Concurrent` into library
  runtime code.

## Guidance For Human Contributors

This codebase is small enough that local consistency matters, but early enough
that clean redesigns are welcome when they improve the physics foundation.

Prefer:

- mirror source/test naming when adding files.
- focused patches over broad folder-wide rewrites.
- release-mode verification for simulation behavior.
- explicit notes about units, coordinate systems, body dimensionality, and
  collision assumptions.
- documenting any deliberate divergence from real-world physics.

Be especially careful when changing:

- frame rate, delta time, and accumulation behavior.
- collider ID allocation and reuse.
- collision pair activation, culling, and deactivation.
- partition ownership and GridForge voxel lookups.
- transform, scale, rotation, and bounds calculations.
- force, impulse, torque, drag, friction, and restitution logic.
- line/ray/circle query ordering.
- 2D/3D dimension assumptions.
- code guarded by `GRAVITAS_DISABLE_MEMORYPACK` or `#if DEBUG`.
