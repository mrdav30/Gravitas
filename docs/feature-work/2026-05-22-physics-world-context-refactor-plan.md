# Gravitas World Context Refactor Battle Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Gravitas's process-wide physics state with explicit world-owned runtime contexts aligned with GridForge's primitive `GridWorld` model.

**Architecture:** Introduce a `GravitasWorldContext` that owns one active `GridWorld`, deterministic clock state, physics registration, collision partitioning, query buffers, coroutine state, and lifecycle hooks. Keep pure stateless helpers static, but move all mutable simulation state out of static classes and into context-owned services.

**Tech Stack:** C# 11, `FixedMathSharp`, `SwiftCollections`, `GridForge`, xUnit v3, BenchmarkDotNet, Chronicler.

---

## Source Context Reviewed

- `src/Gravitas/Core/PhysicsManager.cs`
- `src/Gravitas/Core/CollisionManager.cs`
- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/IMatterAgent.cs`
- `src/Gravitas/Colliders/LSCollider.cs`
- `src/Gravitas/Partitions/PhysicsPartition.cs`
- `src/Gravitas/Raycasting/Raycaster.cs`
- `src/Gravitas/Raycasting/Circlecaster.cs`
- `src/Gravitas/Raycasting/RayCasterWorker.cs`
- `src/Gravitas/Support/Coroutines/CoroutineManager.cs`
- `src/Gravitas/Support/Coroutines/LockedYieldInstructions/*.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Runtime\TrailblazerWorldContext.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Runtime\TrailblazerClock.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Pathing\Partition\SolidChartPartition.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Pathing\Partition\VolumeChartPartition.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Pathing\PathingWorldState.cs`
- `F:\gamedevrepos\Trailblazer\src\Trailblazer\Pathing\PathManager.cs`

## Recommendation

Use the Trailblazer pattern as the spine, but make the Gravitas split more explicit because physics has more mutable runtime state than pathing:

- Add `GravitasWorldContext` with `Attach(GridWorld world, bool takeOwnership = false)` and `CreateOwned(...)`.
- Add `GravitasClock`, modeled after `TrailblazerClock`, and expose clock values through the context.
- Add context-owned services:
  - `GravitasPhysicsService` for body/collider registration, simulation phases, settings, and pair pooling.
  - `GravitasCollisionService` for partition activation, partition pooling, collision version, and partition distribution.
  - `GravitasRaycastService` and `GravitasCirclecastService` for query buffers and versioning.
  - `GravitasCoroutineService` for lockstep coroutine storage.
  - `GravitasLifecycleHooks` for ordered simulate, late-simulate, visualize, reset, and settings-change callbacks.
- Change `IMatterAgent` to expose `GravitasWorldContext Context` instead of a raw `GridWorld`. Agents can still reach the world through `Context.World`, but the physics owner is no longer ambiguous.
- Avoid a long-lived process-wide default context. If a temporary facade is needed to keep the tree compiling during migration, make it short-lived and remove it in the final static-removal phase.

Trailblazer's post-bridge partition model should be the direct precedent for Gravitas partition ownership:

- A partition must receive its owning world/service before it is added to a voxel.
- Removal callers should detach the partition from the voxel; the partition's `OnRemoveFromVoxel(...)` should perform owner-local cleanup and return itself to the owner-local pool.
- A missing owner during removal is a broken invariant, not a cue to fall back to process-wide state.
- Pool release should have one path. Mixing caller-side release with voxel removal creates double-release and stale-activation hazards.

The only static state that should survive this refactor is either immutable domain data or safe process-wide metadata:

- Good static candidates: unit conversion constants, pure math helpers, mesh geometry helpers, reflection delegate caches, diagnostic logger configuration.
- Bad static candidates: frame count, settings, body/collider registries, collider ID allocation, active collision pairs, active partitions, partition pools, raycast/circlecast buffers, coroutine storage, query version counters, raycast worker cache fields.

## Target Ownership Model

```text
GravitasWorldContext
  GridWorld World
  GravitasClock Clock
  PhysicsSettings Settings
  PhysicsEnvironment Environment
  GravitasPhysicsService Physics
    body registry
    collider registry
    collider ID allocator
    active collision-pair queue
    collision-pair pool
  GravitasCollisionService Collisions
    active partitions
    partition pool
    redundancy checker
    collision version
  GravitasRaycastService Raycasts
    hit buffers
    intersection buffers
    redundant collider checker
    raycast worker state
  GravitasCirclecastService Circlecasts
    hit buffers
    redundant collider checker
  GravitasCoroutineService Coroutines
    active coroutines
  GravitasLifecycleHooks Hooks
```

## Static-State Inventory

| Current type | Current mutable static state | Target owner |
| --- | --- | --- |
| `PhysicsManager` | settings, frame count, dynamic bodies, collider array, collider ID stack, pair pool, active pair queue, accumulation state, cull distributor | `GravitasWorldContext`, `GravitasClock`, `GravitasPhysicsService` |
| `CollisionManager` | version, active partitions, partition pool, redundancy checker | `GravitasCollisionService` |
| `PhysicsPartition` | `_id1`, `_id2`, `_pair` scratch fields and calls back into static managers | local variables plus required owning `GravitasCollisionService` |
| `Raycaster` | intersection buffers, hit buffer, redundant checker, ignore layer, version | `GravitasRaycastService` |
| `Circlecaster` | hit buffer, redundant checker, ignore layer, version | `GravitasCirclecastService` |
| `RayCasterWorker` | cached ray axis fields | context-owned worker struct/class used by `GravitasRaycastService` |
| `CoroutineManager` | active coroutine bucket | `GravitasCoroutineService` |
| `WaitForFrames`, `WaitForNextSimulate`, `WaitForRealSeconds` | reads static clock | context or clock-bound yield instructions |
| `PhysicsSettings` | static `PoolingEnabled` | context settings or context environment |
| `PhysicsSettingsSaver` | writes static `PhysicsManager.Settings` | context settings apply/record flow |
| `ColliderSettings` | public mutable static dictionaries | immutable shape-policy helper or context-independent readonly lookup |
| `CollisionDetection`, `CollisionResponse`, `AxisProjectionHelper`, `MeshUtils` | no runtime ownership state | can remain static if kept pure |
| `TransientStateUtility` | reflection delegate caches | can remain static as process-wide metadata cache |
| `GravitasLogger` | diagnostic configuration | can remain static unless a later host logging design requires context-local diagnostics |

## Non-Negotiable Invariants

- One active `GridWorld` can be attached to at most one active `GravitasWorldContext`.
- A `StiffBody`, `LSCollider`, `PhysicsPartition`, collision pair, coroutine, raycast, and circlecast belong to exactly one `GravitasWorldContext`.
- Collider IDs are context-local. A partition stores IDs meaningful only to its owning context.
- Collision pairs never cross contexts.
- Simulation order remains deterministic: body order, collider ID assignment, partition traversal, pair ordering, and query result ordering must be explicit and test-pinned.
- Query services are not reentrant unless the implementation explicitly supports it. Per-context state removes cross-world contamination, not necessarily concurrent mutation hazards.
- `GridWorld` lifetime is explicit: context-created worlds are disposed by the context; host-owned worlds are not.

## Phase 0: Baseline And Test Harness

**Purpose:** Lock down current behavior before moving state.

**Files:**

- Create: `tests/Gravitas.Tests/Runtime/PhysicsManagerLegacyTests.cs`
- Create: `tests/Gravitas.Tests/Support/TestMatterAgent.cs`
- Create: `tests/Gravitas.Tests/Support/PhysicsTestWorld.cs`
- Modify: `tests/Gravitas.Tests/Gravitas.Tests.csproj` only if helper folders need compile metadata.

**Tasks:**

- [ ] Add a `TestMatterAgent` that implements the current `IMatterAgent` with a host-owned `GridWorld`, `FixedTransform`, `IsParent`, and `IsInteracting`.
- [ ] Add `PhysicsTestWorld` helper that creates a `GridWorld`, calls `PhysicsManager.Setup()` and `PhysicsManager.Initialize()`, and disposes the world.
- [ ] Add tests for `PhysicsManager.FrameCount`, `DeltaTime`, `Settings`, body assimilation, collider assimilation, and `Deactivate()`.
- [ ] Add tests proving the current static manager cannot isolate two worlds. This can be an explicit skipped/failing characterization if needed, but it should document the bug the refactor fixes.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~PhysicsManagerLegacyTests
```

**Exit criteria:**

- We have focused tests that describe current manager lifecycle and at least one test demonstrating why global state is insufficient for multiple worlds.

## Phase 1: Add Context, Clock, And Lifecycle Shell

**Purpose:** Introduce the new owner without moving physics behavior yet.

**Files:**

- Create: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Create: `src/Gravitas/Runtime/GravitasClock.cs`
- Create: `src/Gravitas/Runtime/GravitasLifecycleHooks.cs`
- Create: `tests/Gravitas.Tests/Runtime/GravitasWorldContextTests.cs`
- Create: `tests/Gravitas.Tests/Runtime/GravitasClockTests.cs`

**Design:**

- `GravitasWorldContext` mirrors Trailblazer:
  - `Attach(GridWorld world, bool takeOwnership = false)`
  - `CreateOwned(Fixed64? voxelSize = null, int spatialGridCellSize = GridWorld.DefaultSpatialGridCellSize)`
  - static weak ownership map keyed by `GridWorld`
  - `World`, `IsDisposed`, `FrameRate`, `DeltaTime`, `FrameCount`, `TotalTime`, `AccumulatedTime`, `ExpectedAccumulation`, `ResetAccumulation`
  - `Simulate()`, `LateSimulate()`, `Visualize()`, `LateVisualize()`, `Reset()`, `SetFrameRate(int)`
  - `Dispose()`
- `GravitasClock` mirrors `TrailblazerClock`, but uses `PhysicsSettings.DefaultFrameRate`.
- `GravitasLifecycleHooks` should reuse the existing `LifecycleHookHandler`, `OrderedLifecycleHook`, and `LifecycleHookRegistration` types.

**Tasks:**

- [x] Write ownership tests: attach active world, reject inactive world, reject attaching the same active world twice, allow reattach after disposal.
- [x] Write owned-world disposal tests: `CreateOwned()` disposes its world; `Attach(takeOwnership: false)` leaves the host-owned world active.
- [x] Write clock tests: simulate increments `FrameCount` and `TotalTime`; late simulate marks accumulation reset; visualize resets then increments accumulation; setting frame rate changes `DeltaTime`.
- [x] Implement the shell with no dependency on `PhysicsManager`.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasWorldContextTests|FullyQualifiedName~GravitasClockTests"
```

**Exit criteria:**

- Context lifecycle and timing are tested independently of the old static manager.

**Status:** Complete for Phase 1. Added `GravitasWorldContext`, `GravitasClock`, `GravitasLifecycleHooks`, and focused runtime tests. Also added the MemoryPack-disabled shim so Lean builds keep the normal serialization attributes without compiler-condition noise. Verified with focused Phase 1 tests plus `Release` and `ReleaseLean` solution build/test runs.

## Phase 2: Move Settings And Environment State

**Purpose:** Stop reading mutable physics configuration from `PhysicsManager`.

**Files:**

- Create: `src/Gravitas/Settings/PhysicsEnvironment.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Create: `tests/Gravitas.Tests/Settings/PhysicsEnvironmentTests.cs`
- Create: `tests/Gravitas.Tests/Settings/PhysicsSettingsTests.cs`

**Design:**

- `PhysicsSettings` remains the context's frame/layer settings object.
- Move `PhysicsSettings.PoolingEnabled` into an instance setting or `PhysicsEnvironment`.
- Move mutable environment values out of `PhysicsManager`:
  - gravity
  - air density
  - min/max speeds
  - max fall speed
  - friction transition speed
  - deceleration multiplier
  - damping factor
  - culling thresholds
- Keep unit conversions such as pounds-to-newtons and kilograms-to-pounds as immutable static constants, ideally under `PhysicsUnits`.

**Tasks:**

- [x] Write tests for context-specific settings: two contexts can have different frame rates and layer matrices.
- [x] Write tests for context-specific environment: two contexts can have different gravity and culling values without affecting each other.
- [x] Introduce `PhysicsEnvironment.Default()` with current values.
- [x] Add `Settings` and `Environment` properties to `GravitasWorldContext`.
- [x] Update `GravitasClock.SetFrameRate` call path so changing context settings updates clock deterministically.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PhysicsEnvironmentTests|FullyQualifiedName~PhysicsSettingsTests|FullyQualifiedName~GravitasClockTests"
```

**Exit criteria:**

- Configuration needed by simulation is context-owned and test-pinned.

**Status:** Complete for Phase 2. Added `PhysicsEnvironment`, moved pooling into instance `PhysicsSettings`, added context-owned `Settings` and `Environment`, and added `ApplySettings(...)` so replacing settings synchronizes the context clock. Verified with focused Phase 2 tests plus `Release` and `ReleaseLean` solution build/test runs.

## Phase 3: Introduce Context-Owned Physics Registry

**Purpose:** Move body/collider registration, IDs, active pair queue, and pair pool out of `PhysicsManager`.

**Files:**

- Create: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Create: `tests/Gravitas.Tests/Core/GravitasPhysicsServiceTests.cs`

**Design:**

- `GravitasPhysicsService` owns:
  - `SwiftBucket<StiffBody>` dynamic bodies
  - `LSCollider?[]` collider table
  - free collider ID stack
  - active collision-pair queue
  - collision-pair pool
  - `SimulatePhysics`
  - body/collider counters
- `GravitasWorldContext.Physics` exposes lifecycle methods:
  - `Initialize()`
  - `LateInitialize()`
  - `Simulate()`
  - `LateSimulate()`
  - `Visualize()`
  - `LateVisualize()`
  - `Reset()`
  - `Deactivate()`
- `StiffBody` stores `GravitasWorldContext Context`.
- `LSCollider` stores `GravitasWorldContext Context`.
- `CollisionPair` should be initialized with the owning context and should not resolve colliders through static state.

**Tasks:**

- [x] Write tests proving collider IDs are context-local: two contexts can each allocate collider ID `1` without cross-resolution.
- [x] Write tests proving `TryGetColliderById` resolves only within its context.
- [x] Write tests proving a pair created in one context cannot use colliders from another context.
- [x] Add `GravitasPhysicsService` and wire it into `GravitasWorldContext`.
- [x] Replace body/collider assimilation calls in `StiffBody` and `LSCollider` with `Context.Physics`.
- [x] Keep a temporary internal adapter only if necessary to make incremental compilation possible. Remove the adapter by Phase 8.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~GravitasPhysicsServiceTests
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Body and collider ownership no longer depends on process-wide static registries.

**Status:** Complete for Phase 3. Added `GravitasPhysicsService`, moved body and collider registration through `GravitasWorldContext.Physics`, made collider IDs context-local, verified reset clears the context-local collider table without body-count underflow, and made `CollisionPair` reject colliders from different contexts. The temporary simulation bridge still calls the static collision manager, and cull distribution still uses the static distributor, until the collision step moves in Phase 5. Verified with focused Phase 3 tests plus `Release` and `ReleaseLean` solution build/test runs.

## Phase 4: Bind Agents To Context Instead Of Raw World

**Purpose:** Make host ownership explicit at the API boundary.

**Files:**

- Modify: `src/Gravitas/Core/IMatterAgent.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `tests/Gravitas.Tests/Support/TestMatterAgent.cs`
- Create: `tests/Gravitas.Tests/Core/MatterAgentContextTests.cs`

**Design:**

- Replace `GridWorld World { get; }` on `IMatterAgent` with `GravitasWorldContext Context { get; }`.
- All world access flows through `agent.Context.World`.
- `StiffBody.Setup(...)` verifies the agent and collider bind to the same context.
- `LSCollider.InitializeWithNoBody(...)` binds static colliders to `agent.Context`.
- Static collider APIs should take a context-bound agent, not a raw world.

**Tasks:**

- [ ] Write tests that constructing a body/collider with an agent from one context registers with that context.
- [ ] Write tests that attempting to combine body/collider/agent ownership across contexts fails with a clear exception.
- [ ] Update `IMatterAgent`.
- [ ] Update all body and collider world access to `Context.World`.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~MatterAgentContextTests|FullyQualifiedName~GravitasPhysicsServiceTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- `GridWorld` handling is explicit through `GravitasWorldContext`, and agents cannot accidentally point physics at a world without its owning runtime state.

## Phase 5: Move Collision Partitioning Into A Context Service

**Purpose:** Isolate active partitions, partition pools, collision versioning, and partition distribution by world context.

**Files:**

- Create: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/Core/CollisionManager.cs`
- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Create: `tests/Gravitas.Tests/Core/GravitasCollisionServiceTests.cs`
- Create: `tests/Gravitas.Tests/Partitions/PhysicsPartitionTests.cs`

**Design:**

- `GravitasCollisionService` owns:
  - collision version
  - active partition bucket
  - partition object pool
  - redundancy checker
- `PhysicsPartition` stores a required owning `GravitasCollisionService`, set immediately after rent and before `voxel.TryAddPartition(partition)`.
- `PhysicsPartition.OnRemoveFromVoxel(...)` is the only release path. It should deactivate from its owner service, detach voxel event/state if added later, and release itself back to the owner service's partition pool.
- Missing owner during removal is an invariant violation. Do not add a static fallback pool or compatibility bridge.
- `PhysicsPartition.Distribute()` uses local variables instead of static scratch fields.
- `PhysicsPartition` calls its owning service to activate/deactivate itself and to resolve collision pairs through `Context.Physics`.
- Remove the current manual `CollisionManager.PoolNodePartition(...)` pattern. Callers should remove the partition from the voxel and let the partition's `OnRemoveFromVoxel(...)` release through the owner, avoiding double-release hazards.

**Tasks:**

- [ ] Write tests for partition activation/deactivation within one context.
- [ ] Write tests for two contexts with overlapping world coordinates: partition activation in one context does not affect the other.
- [ ] Write tests proving `PhysicsPartition.Distribute()` does not reuse static scratch state.
- [ ] Write tests proving `PhysicsPartition.OnRemoveFromVoxel(...)` releases through the owner service and throws or logs a clear invariant violation when no owner exists.
- [ ] Write tests proving empty partition cleanup removes the partition from the voxel exactly once and does not double-release it.
- [ ] Create `GravitasCollisionService`.
- [ ] Convert `CollisionManager` methods into instance methods on the service.
- [ ] Convert `PhysicsPartition` to service-owned behavior with a required `SetOwner(GravitasCollisionService owner)` or owner-taking rent helper.
- [ ] Replace direct pool release calls with `voxel.TryRemovePartition<PhysicsPartition>()` so `OnRemoveFromVoxel(...)` owns the release flow.
- [ ] Remove or empty the old static `CollisionManager` after all callers use `Context.Collisions`.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasCollisionServiceTests|FullyQualifiedName~PhysicsPartitionTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Collision partitioning is world-local, pair resolution is context-local, and static collision service state is gone.

## Phase 6: Move Raycast And Circlecast State Into Query Services

**Purpose:** Remove static query buffers and make ray/circle queries context-local and deterministic.

**Files:**

- Create: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Create: `src/Gravitas/Raycasting/GravitasCirclecastService.cs`
- Create: `src/Gravitas/Raycasting/RaycastAxisWorker.cs`
- Modify: `src/Gravitas/Raycasting/Raycaster.cs`
- Modify: `src/Gravitas/Raycasting/Circlecaster.cs`
- Modify: `src/Gravitas/Raycasting/RayCasterWorker.cs`
- Create: `tests/Gravitas.Tests/Raycasting/GravitasRaycastServiceTests.cs`
- Create: `tests/Gravitas.Tests/Raycasting/GravitasCirclecastServiceTests.cs`

**Design:**

- `GravitasWorldContext.Raycasts` owns raycast buffers and worker state.
- `GravitasWorldContext.Circlecasts` owns circlecast buffers and versioning.
- Query methods should no longer accept raw `GridWorld` when called through context services. The world is `Context.World`.
- If static convenience methods remain briefly, they must delegate to an explicit context parameter, not hidden global state.
- `RaycastAxisWorker` should be an instance or ref struct carrying the ray-axis cache currently held in static fields.

**Tasks:**

- [ ] Write tests for raycast result ordering in one context.
- [ ] Write tests for circlecast duplicate suppression in one context.
- [ ] Write tests proving query buffers do not leak across two contexts.
- [ ] Convert `Raycaster` into `GravitasRaycastService`.
- [ ] Convert `Circlecaster` into `GravitasCirclecastService`.
- [ ] Convert `RayCasterWorker` cached fields into instance-owned worker state.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasRaycastServiceTests|FullyQualifiedName~GravitasCirclecastServiceTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Query state is context-local and static query buffers are gone.

## Phase 7: Move Coroutines And Yield Instructions To The Context Clock

**Purpose:** Remove coroutine and wait-instruction dependence on static frame timing.

**Files:**

- Create: `src/Gravitas/Support/Coroutines/GravitasCoroutineService.cs`
- Modify: `src/Gravitas/Support/Coroutines/CoroutineManager.cs`
- Modify: `src/Gravitas/Support/Coroutines/LockedYieldInstructions/WaitForFrames.cs`
- Modify: `src/Gravitas/Support/Coroutines/LockedYieldInstructions/WaitForNextSimulate.cs`
- Modify: `src/Gravitas/Support/Coroutines/LockedYieldInstructions/WaitForRealSeconds.cs`
- Create: `tests/Gravitas.Tests/Support/Coroutines/GravitasCoroutineServiceTests.cs`
- Create: `tests/Gravitas.Tests/Support/Coroutines/LockedYieldInstructionTests.cs`

**Design:**

- `GravitasCoroutineService` owns the coroutine bucket for a context.
- Yield instructions accept either a `GravitasWorldContext` or `GravitasClock` at construction.
- Prefer context factories for ergonomics:
  - `context.Coroutines.WaitForFrames(int frames)`
  - `context.Coroutines.WaitForNextSimulate()`
  - `context.Coroutines.WaitForRealSeconds(Fixed64 seconds)`
- Do not keep yield instructions reading `PhysicsManager.FrameCount` or `PhysicsManager.DeltaTime`.

**Tasks:**

- [ ] Write tests for frame waits against one context clock.
- [ ] Write tests for two contexts advancing at different frame counts.
- [ ] Create `GravitasCoroutineService`.
- [ ] Convert `CoroutineManager` behavior to instance service methods.
- [ ] Update yield instructions to context/clock-bound constructors.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasCoroutineServiceTests|FullyQualifiedName~LockedYieldInstructionTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Coroutines and wait instructions are deterministic per context.

## Phase 8: Remove Static Facades And Clean Public API

**Purpose:** Finish the ownership inversion instead of leaving a disguised global manager.

**Files:**

- Delete or radically reduce: `src/Gravitas/Core/PhysicsManager.cs`
- Delete or radically reduce: `src/Gravitas/Core/CollisionManager.cs`
- Delete or radically reduce: `src/Gravitas/Raycasting/Raycaster.cs`
- Delete or radically reduce: `src/Gravitas/Raycasting/Circlecaster.cs`
- Delete or radically reduce: `src/Gravitas/Support/Coroutines/CoroutineManager.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettingsSaver.cs`
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `tests/Gravitas.Benchmarks/README.md`

**Design:**

- Remove static manager APIs unless they are pure helpers.
- If class names remain for discoverability, they should be stateless factories or extension helpers requiring explicit `GravitasWorldContext`.
- Settings serialization should apply to an explicit context. Chronicler should still populate existing host-created objects.
- `ColliderSettings` should stop exposing mutable public dictionaries. Convert to immutable lookup arrays or private readonly mappings with accessor methods.

**Tasks:**

- [ ] Search for `PhysicsManager.`, `CollisionManager.`, `Raycaster.`, `Circlecaster.`, and `CoroutineManager.` references.
- [ ] Replace all runtime references with context-owned services.
- [ ] Remove temporary adapters introduced during earlier phases.
- [ ] Update docs to show context-first usage.
- [ ] Run:

```bash
rg -n "PhysicsManager\.|CollisionManager\.|Raycaster\.|Circlecaster\.|CoroutineManager\." src/Gravitas tests/Gravitas.Tests
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

**Exit criteria:**

- Runtime code no longer relies on mutable static manager state.

## Phase 9: Benchmarks And Regression Hardening

**Purpose:** Prove the refactor did not introduce hidden allocation or complexity regressions.

**Files:**

- Create: `tests/Gravitas.Benchmarks/Runtime/WorldContextBenchmarks.cs`
- Create: `tests/Gravitas.Benchmarks/Core/CollisionPartitionBenchmarks.cs`
- Create: `tests/Gravitas.Benchmarks/Raycasting/QueryServiceBenchmarks.cs`
- Modify: `tests/Gravitas.Benchmarks/README.md`

**Benchmark scenarios:**

- Create and dispose owned context.
- Attach host-owned world.
- Register dynamic bodies and colliders.
- Partition static and dynamic colliders.
- Simulate active collision partitions.
- Raycast and circlecast through a populated context.
- Two-context isolation with overlapping world coordinates.

**Tasks:**

- [ ] Add benchmark fixtures that create deterministic contexts and dispose them per benchmark case.
- [ ] Add `[MemoryDiagnoser]` to each benchmark class.
- [ ] Capture baseline JSON after the implementation stabilizes.
- [ ] Run:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all --config InProcessShortRunConfig
```

**Exit criteria:**

- The new architecture has benchmark coverage for lifecycle, registration, partitioning, and query services.

## Suggested Commit Sequence

1. `test: characterize legacy physics manager state`
2. `feat: add gravitas world context and clock`
3. `feat: move physics settings into world context`
4. `feat: add context-owned physics registry`
5. `refactor: bind matter agents to gravitas contexts`
6. `feat: move collision partitions into context service`
7. `feat: move raycast and circlecast state into context services`
8. `feat: move coroutines onto context clock`
9. `refactor: remove static physics manager facades`
10. `perf: add context runtime benchmarks`
11. `docs: document context-first gravitas runtime`

## Main Risks

- `PhysicsPartition` currently stores collider IDs only. Once IDs become context-local, every partition must have a required owner service set before voxel attachment.
- Partition cleanup currently removes the partition and releases it manually. The refactor should use a single release path through `PhysicsPartition.OnRemoveFromVoxel(...)`; mixing manual service release with voxel removal risks double-release and stale activation IDs.
- `Raycaster` and `Circlecaster` accept `GridWorld` today but resolve colliders through global IDs. This is the exact cross-world leak the query phase must close.
- `RayCasterWorker` static cache makes overlapping or nested raycasts unsafe. Moving it to per-query worker state is necessary even if the public query API is context-owned.
- `WaitForFrames`, `WaitForNextSimulate`, and `WaitForRealSeconds` currently read static clock state. They will silently behave incorrectly with multiple contexts until fixed.
- `PhysicsSettingsSaver` writes global settings. Serialization must become context-applied before save/load behavior can be trusted in multi-world hosts.
- Public mutable dictionaries in `ColliderSettings` allow runtime mutation of global shape policy. This is not the same class of bug as manager state, but it should be cleaned during static facade removal.

## Acceptance Criteria

- Two active `GravitasWorldContext` instances can run against two active `GridWorld` instances in the same process.
- Each context can use different frame rates, physics settings, gravity/environment values, and collision state.
- Dynamic bodies and colliders register with their owning context and are not visible to other contexts.
- Collision pairs and partitions never cross context boundaries.
- Raycasts and circlecasts query only the context world and resolve only context-local colliders.
- Coroutines and wait instructions advance against the owning context clock.
- No mutable runtime simulation state remains in static manager classes.
- `dotnet test Gravitas.slnx --configuration Release` passes.
- `dotnet test Gravitas.slnx --configuration ReleaseLean` passes.
- README and AGENTS describe context-first runtime usage.
