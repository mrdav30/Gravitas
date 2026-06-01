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

1. [`README.md`](README.md) for current package orientation.
2. The stack context, when sibling repositories are available:
   - `../FixedMathSharp/AGENTS.md` and `../FixedMathSharp/README.md`
   - `../SwiftCollections/AGENTS.md` and `../SwiftCollections/README.md`
   - `../GridForge/AGENTS.md` and `../GridForge/README.md`
   - `../Chronicler/AGENTS.md` and `../Chronicler/README.md` when serialization
     behavior is involved.
3. [`docs/wiki/OVERVIEW.md`](docs/wiki/OVERVIEW.md), then the matching wiki
   page for the area being changed:
   [`HOST_INTEGRATION.md`](docs/wiki/HOST_INTEGRATION.md),
   [`RUNTIME_ARCHITECTURE.md`](docs/wiki/RUNTIME_ARCHITECTURE.md),
   [`COLLISION_PIPELINE.md`](docs/wiki/COLLISION_PIPELINE.md), or
   [`QUERY_SERVICES.md`](docs/wiki/QUERY_SERVICES.md), or
   [`DIAGNOSTICS.md`](docs/wiki/DIAGNOSTICS.md).
4. [`src/Gravitas/Runtime/GravitasWorldContext.cs`](src/Gravitas/Runtime/GravitasWorldContext.cs),
   [`src/Gravitas/Core/GravitasPhysicsService.cs`](src/Gravitas/Core/GravitasPhysicsService.cs),
   and [`src/Gravitas/Core/StiffBody.cs`](src/Gravitas/Core/StiffBody.cs).
5. The relevant source folder under [`src/Gravitas`](src/Gravitas).
6. The matching test or benchmark area under [`tests`](tests). The unit test
   project now has focused runtime/settings coverage, so new behavior usually
   needs matching tests.
7. [`src/Gravitas/Gravitas.csproj`](src/Gravitas/Gravitas.csproj),
   [`tests/Gravitas.Tests/Gravitas.Tests.csproj`](tests/Gravitas.Tests/Gravitas.Tests.csproj),
   and [`tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj`](tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj).

## Source Of Truth

When code, README, and generated docs disagree, prefer the code and project
files. Keep docs honest as the prototype sheds older copied scaffolding.

Keep these aligned whenever behavior, public API, package shape, or developer
workflow changes:

- [`README.md`](README.md)
- [`docs/wiki`](docs/wiki), especially when runtime ownership, host
  integration, collision behavior, query behavior, lifecycle order, or known
  prototype limitations change.
- [`tests/Gravitas.Tests`](tests/Gravitas.Tests)
- [`tests/Gravitas.Benchmarks`](tests/Gravitas.Benchmarks) when performance
  claims or hot paths change.
- Relevant workflow files under [`.github/workflows`](.github/workflows).

## Repository Map

| Path | Purpose | Notes |
| --- | --- | --- |
| [`src/Gravitas`](src/Gravitas) | Main library project | Multi-targets `netstandard2.1` and `net8.0`. |
| [`src/Gravitas/Core`](src/Gravitas/Core) | Context-owned physics service, body state, and host agent interface | Start here for body/registration architecture changes. |
| [`src/Gravitas/Runtime`](src/Gravitas/Runtime) | Explicit world context, deterministic clock, and lifecycle hooks | Start here for host integration changes. |
| [`src/Gravitas/Colliders`](src/Gravitas/Colliders) | Collider base type, primitive colliders, physics mesh helpers | Shape logic is currently 3D-focused. |
| [`src/Gravitas/CollisionHandling`](src/Gravitas/CollisionHandling) | Collision detection, response, pairs, contact data | Determinism and ordering are high risk here. |
| [`src/Gravitas/Queries`](src/Gravitas/Queries) | 2D/3D raycast, swept-sphere, and overlap query support | Keep result ordering stable. |
| [`src/Gravitas/Diagnostics`](src/Gravitas/Diagnostics) | Context-owned diagnostic events and engine-agnostic debug draw commands | Keep disabled paths allocation-free and renderer-neutral. |
| [`src/Gravitas/Partitions`](src/Gravitas/Partitions) | GridForge-backed physics partitions | Tied to voxel ownership and pooling. |
| [`src/Gravitas/Settings`](src/Gravitas/Settings) | Physics settings and save helpers | Includes frame rate and layer collision matrix behavior. |
| [`src/Gravitas/Support`](src/Gravitas/Support) | Fixed transforms, layers, lifecycle hooks, coroutine scaffolding, transient state helpers | Keep engine-specific assumptions out. |
| [`tests/Gravitas.Tests`](tests/Gravitas.Tests) | xUnit v3 test project | Contains focused runtime/settings coverage; expand it alongside behavior changes. |
| [`tests/Gravitas.Benchmarks`](tests/Gravitas.Benchmarks) | BenchmarkDotNet project | Covers context lifecycle, registration/partitioning, simulation, queries, and diagnostics. |
| [`docs/wiki`](docs/wiki) | Developer-facing architecture and usage notes | Keep current with runtime, host integration, collision, query, and diagnostics changes. |
| [`docs/feature-work/prototype`](docs/feature-work/prototype) | Historical/prototype Unity-oriented reference code | Useful context, not the source of truth. |

Ignore generated output when reviewing structure:

- `bin/`
- `obj/`
- `TestResults/`
- `artifacts/`
- `.vs/`
- `BenchmarkDotNet.Artifacts/`

## Runtime Architecture Snapshot

The current runtime uses explicit world-context ownership:

- `GravitasWorldContext` is the host-facing runtime shell. It owns the
  `GridWorld`, context settings, physical environment, deterministic clock,
  lifecycle hooks, and context-local runtime services.
- `GravitasPhysicsService` owns context-local dynamic body registration,
  collider IDs, collider lookup, collision-pair pooling, active pair processing,
  and physics lifecycle phases.
- `GravitasPhysics2DService` owns pure 2D body and collider registration,
  collider IDs, 2D pair pooling, response/event processing, visualization
  transform publishing for one context.
- `GravitasMixedCollisionService` owns the Phase 10 mixed 2D/3D lifecycle path,
  GridForge-backed mixed broad phase, stable 3D/2D candidate keys, awake
  gating, and retained `PhysicsMixedPartition` cleanup. `PhysicsRuntimeMode.Mixed`
  reaches it; `PhysicsRuntimeMode.Both` deliberately runs pure 2D and pure 3D
  side by side without cross-dimensional contacts.
- `GravitasCollisionService` maps colliders into GridForge voxels through
  `GridWorld` spatial hash and active-grid access, `WorldVoxelIndex`, and
  `PhysicsPartition`, using `SwiftCollections` pools and duplicate-check sets.
- `GravitasCollision2DService` maps pure 2D X/Z bounds into GridForge voxels
  through `PhysicsPartition2D`, using the internal Y=0 storage plane as
  deterministic broad-phase identity rather than physical thickness.
- `GravitasQuery3DService` owns 3D raycast, swept-sphere, and X/Z
  overlap/proximity query workers, intersection state, candidate gathering,
  filtering, and result ordering for one context. X/Z circle queries are not
  swept casts.
- `GravitasQuery2DService` owns pure 2D overlap-circle and segment raycast
  query buffers and hit ordering for one context.
- All-hit query paths should write into caller-owned hit buffers.
- `GravitasCoroutineService` owns lockstep coroutine state and context-bound
  wait instructions for one context.
- `GravitasDiagnosticSink` owns disabled-by-default diagnostic event and debug
  draw buffers for one context. It should expose deterministic data that host
  adapters can render or log without engine dependencies.
- `StiffBody` owns simulated body state: position, rotation, visual
  interpolation state, velocity, acceleration, drag, friction, grounding,
  transforms, and Chronicler state recording.
- `StiffBody2D` owns pure 2D body state: X/Z-projected position, scalar yaw,
  linear velocity, force integration, sleep/wake state, visualization transform
  publishing, and Chronicler state recording.
- `IMatterAgent` is the host boundary. Hosts provide a `GravitasWorldContext`,
  a `FixedTransform`, hierarchy information, and interaction state without tying
  Gravitas to a game engine.
- `LSCollider` and `LSCollider2D` primitive subclasses own shape state, bounds,
  layers, trigger/contact events, GridForge partition coordinates, and
  collision-pair references.
- `CollisionDetection`, `CollisionResponse`, `CollisionPair`, `ContactPoint`,
  and context structs form the narrow-phase and response layer.

Treat this as a working prototype, not final architecture. Context ownership,
collider IDs, partition reuse, collision-pair ownership, and simulation phase
ordering are high-risk areas.

## Lockstep Host Lifecycle

The old LSF Unity prototype mapped the lockstep loop onto Unity callbacks, but
the important contract is engine-agnostic. Hosts own the outer loop; Gravitas
should expose deterministic phases that can be called from Unity, a server, a
test harness, or another simulation runner.

| Phase | Typical host timing | Gravitas expectation |
| --- | --- | --- |
| `Setup` | Once per run before instance initialization | Configure package/static defaults only. Prefer moving new state to explicit contexts instead of adding more ambient setup. |
| `Initialize` | Once per context/session or when an agent is unpooled | Bind worlds, agents, bodies, colliders, settings, and pools. Do not advance simulation state here. |
| `GameStart` | First simulation frame after commands can advance | Usually a host/game-layer hook. Add a Gravitas equivalent only for a real physics invariant. |
| `Execute` | When deterministic commands or network frames are received | Apply ordered input into host-owned state before `Simulate`. Gravitas runtime code should not read wall-clock input timing. |
| `Simulate` | Fixed-rate simulation step | Perform authoritative deterministic mutation only here or in `LateSimulate`. |
| `LateSimulate` | End of the same fixed-rate step | Process deterministic deferred queues, pair cleanup, body late simulation, and post-step bookkeeping. |
| `Visualize` | Render/update frame | Interpolate or publish presentation state. Do not mutate authoritative simulation state. |
| `LateVisualize` | Late render/update frame | Finish presentation-only work after host transforms/animation have run. |
| `UpdateGUI` | Host UI/debug draw pass | Keep this outside core physics unless it is diagnostics-only and non-authoritative. |
| `Deactivate` | Session end, object disable, or object pooling | Release registrations and pooled runtime state so the object/context can be reused or disposed. |
| `Quit` | Application shutdown | Host concern. Core physics should not require application-lifetime callbacks for correctness. |

When changing lifecycle code, preserve the fixed-step boundary: deterministic
state changes belong in `Simulate`/`LateSimulate`, while render-frame phases are
for visualization and host-facing presentation only.

## 2D, 3D, And Mixed-Dimension Direction

Gravitas still has deeper 3D coverage, but pure 2D now has a first-class
runtime path through `StiffBody2D`, `LSCollider2D`,
`GravitasPhysics2DService`, `GravitasCollision2DService`, and
`PhysicsPartition2D`. `PhysicsRuntimeMode` is a validated bitmask:
`TwoD`, `ThreeD`, `Both`, and `Mixed` are valid settings values. `Both` runs
pure 2D and pure 3D without mixed contacts; `Mixed` enables the dedicated mixed
lifecycle path. `LSCollider2D` also caches a mixed `BoundingBox` using
`PhysicsSettings.Mixed2DHalfThickness`, optional per-collider
`MixedHalfThicknessOverride`, and the host transform's Y position. The mixed
broad phase now emits deterministic candidate keys through `PhysicsMixedPartition`;
contacts, response, diagnostics, mixed queries, and CCD remain later Phase 10
work.

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

## Experimental Design And Evidence Bar

Gravitas is the core value target of the LSF stack. Much of the current physics
logic is experimental by design: it follows useful patterns from other physics
systems where they help, but it should not copy another engine just because that
engine's approach is familiar.

Thinking outside the box is welcome when the evidence is strong. Novel
algorithms, data layouts, collision strategies, integration models, or
partition/query approaches are acceptable if they:

- preserve deterministic replay across repeated runs.
- have explicit units, invariants, ordering rules, and failure modes.
- are covered by focused unit tests for correctness and edge cases.
- are measured against a baseline with benchmarks when they touch hot paths.
- improve or preserve time complexity, allocation behavior, and physical
  coherence.
- are documented in `docs/wiki` or a feature-work plan when the design changes
  how the system should be understood.

Do not reject a better design just because it is unusual. Do reject clever
changes that cannot be explained, tested, benchmarked, or made deterministic.

## External Dependencies

The main external packages shape how this project should be changed:

- `FixedMathSharp`: use `Fixed64`, `Vector2d`, `Vector3d`, `Vector4d`,
  `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`, deterministic bounds, and
  geometry primitives. Before hand-rolling spatial math, review
  `../FixedMathSharp/src/FixedMathSharp/Geometry`, especially `BoundingBox`,
  `BoundingArea`, `BoundingSphere`, `BoundingFrustum`, `FixedRay`,
  `FixedPlane`, `ContainmentType`, and `FixedPlaneIntersectionType`.
- `SwiftCollections`: prefer `SwiftBucket`, `SwiftList`, `SwiftQueue`,
  `SwiftStack`, `SwiftHashSet`, object pools, and related low-allocation types in
  runtime or hot-path code. For broad-phase or spatial-query experiments,
  review `SwiftCollections.FixedMathSharp` first: `SwiftFixedBVH<T>`,
  `SwiftFixedOctree<T>`, `SwiftFixedSpatialHash<T>`, and `FixedBoundVolume`.
  The generic `SwiftCollections.Query` types can be useful, but avoid
  `System.Numerics`/floating-point query helpers in deterministic runtime paths.
  Use `SwiftCollections.Observable` for host-facing diagnostics or tooling only,
  not authoritative per-frame simulation paths unless tests and benchmarks prove
  the notification cost and ordering are acceptable.
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
- frame-based reasoning through `GravitasWorldContext.FrameRate`,
  `GravitasWorldContext.DeltaTime`, and `GravitasWorldContext.FrameCount`.
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

- `GravitasWorldContext.Simulate`, `LateSimulate`, `Visualize`, and service
  phase ordering.
- body/collider assimilation and dessimilation in `GravitasPhysicsService`.
- GridForge partitioning in `GravitasCollisionService.PartitionObject` and
  `ClearPartitionedObject`.
- collision-pair creation, culling, notification, deactivation, and pooling.
- narrow-phase shape checks in `CollisionDetection`.
- contact resolution in `CollisionResponse`.
- raycast/circlecast candidate gathering, ordering, and filtering.
- mesh collider preprocessing and convex mesh limits.

Optimization rules:

- Preserve physics correctness before reducing allocations.
- Choose data structures by complexity and access pattern, not habit.
- Check FixedMathSharp geometry and `SwiftCollections.FixedMathSharp` query
  structures before creating custom bounds, ray, plane, BVH, octree, or spatial
  hash code. If existing primitives are skipped, document why they do not fit.
- For novel hot-path algorithms, capture a benchmark baseline and explain why
  the new approach is measurably better or complexity-safer.
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
- Source files currently mix region-heavy prototype style with newer
  context-owned service code. Match nearby style for focused edits.

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

- `tests/Gravitas.Tests` contains focused runtime/settings coverage. Mirror the
  source area being changed and add regression tests alongside behavior changes.
- Building the library produces NuGet packages because `GeneratePackageOnBuild`
  is enabled.
- CI builds and tests `Release` and `ReleaseLean` on Ubuntu and Windows.
- Coverage workflow runs the xUnit project with `XPlat Code Coverage`.

## Benchmark Workflow

The benchmark project contains initial physics-specific coverage for context
lifecycle, registration/partitioning, simulation, and query services.

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

The existing tests are still early and focused on runtime shell, settings, and
physics-service behavior. New tests should keep building the foundation for
deterministic physics behavior.

Prioritize tests for:

- `GravitasWorldContext` setup, frame count, fixed delta time, settings,
  service ownership, and reset behavior.
- `GravitasPhysicsService` assimilation, dessimilation, collider lookup, pair
  ownership, and reset behavior.
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
10. Update `docs/wiki`, `README.md`, benchmark docs, or workflow docs if public
    behavior, developer workflow, system architecture, collision behavior, or
    query behavior changed.

## Guidance For AI Agents

If you are an automated coding agent working in this repository:

- Do not trust high-level docs blindly; validate against code, project files,
  and tests.
- Do not broaden scope from one subsystem into another unless the change truly
  requires it.
- Call out any build or test failures explicitly, with exact file references.
- Treat context ownership, collider IDs, collision-pair ownership, partition
  reuse, pooled collections, settings, frame ordering, and GridForge world
  ownership as high-risk areas.
- Treat serialization boundaries and load semantics as high-risk areas. Avoid
  silently broadening populate-existing-instance loads into construct-from-data
  behavior.
- Prefer focused redesigns with tests over patches that preserve flawed
  behavior.
- If you change a public API or behavior, update tests and docs in the same
  pass.
- If you change runtime architecture, host integration, collision flow, query
  behavior, lifecycle order, or known prototype limitations, update the matching
  page under `docs/wiki`.
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
