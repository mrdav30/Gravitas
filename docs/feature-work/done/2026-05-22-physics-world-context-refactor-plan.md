# Gravitas World Context Refactor Battle Plan

> **Archive status:** Completed and moved to `docs/feature-work/done` on 2026-05-22. The only carry-forward item is allocation hardening, tracked separately in [`2026-05-22-runtime-allocation-hardening-plan.md`](2026-05-22-runtime-allocation-hardening-plan.md).

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Gravitas's process-wide physics state with explicit world-owned runtime contexts aligned with GridForge's primitive `GridWorld` model.

**Architecture:** Introduce a `GravitasWorldContext` that owns one active `GridWorld`, deterministic clock state, physics registration, collision partitioning, query buffers, coroutine state, and lifecycle hooks. Keep pure stateless helpers static, but move all mutable simulation state out of static classes and into context-owned services.

**Tech Stack:** C# 11, `FixedMathSharp`, `SwiftCollections`, `GridForge`, xUnit v3, BenchmarkDotNet, Chronicler.

---

## Source Context Reviewed

- `src/Gravitas/Core/PhysicsManager.cs`
- `src/Gravitas/Core/CollisionManager.cs`
- `src/Gravitas/Core/SolidBody.cs`
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
- A `SolidBody`, `LSCollider`, `PhysicsPartition`, collision pair, coroutine, raycast, and circlecast belong to exactly one `GravitasWorldContext`.
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

- [x] Add a shared `TestMatterAgent` for context-bound test agents.
- [x] Supersede the legacy `PhysicsTestWorld` helper path by moving directly to `GravitasWorldContext` ownership and deleting `PhysicsManager` in Phase 8.
- [x] Add focused context runtime, clock, settings, assimilation, dessimilation, reset, and lifecycle tests across Phases 1-8.
- [x] Add tests proving two contexts isolate worlds, settings, body/collider IDs, partitions, raycasts, circlecasts, and coroutines.
- [x] Run focused tests as each replacement phase landed, then verify the full `Release` and `ReleaseLean` solution runs.

The original legacy-manager test command was intentionally superseded once the
static manager was removed instead of preserved behind a compatibility bridge.

**Exit criteria:**

- We have focused tests that describe current context lifecycle and demonstrate why process-global physics state is insufficient for multiple worlds.

**Status:** Superseded and complete. The original Phase 0 legacy-characterization path was replaced by context-first runtime tests as the static manager was decomposed. This avoided preserving a deleted API just for compatibility characterization.

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
- Modify: `src/Gravitas/Core/SolidBody.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Create: `tests/Gravitas.Tests/Core/GravitasPhysicsServiceTests.cs`

**Design:**

- `GravitasPhysicsService` owns:
  - `SwiftBucket<SolidBody>` dynamic bodies
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
- `SolidBody` stores `GravitasWorldContext Context`.
- `LSCollider` stores `GravitasWorldContext Context`.
- `CollisionPair` should be initialized with the owning context and should not resolve colliders through static state.

**Tasks:**

- [x] Write tests proving collider IDs are context-local: two contexts can each allocate collider ID `1` without cross-resolution.
- [x] Write tests proving `TryGetColliderById` resolves only within its context.
- [x] Write tests proving a pair created in one context cannot use colliders from another context.
- [x] Add `GravitasPhysicsService` and wire it into `GravitasWorldContext`.
- [x] Replace body/collider assimilation calls in `SolidBody` and `LSCollider` with `Context.Physics`.
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
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/SolidBody.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Create: `tests/Gravitas.Tests/Support/TestMatterAgent.cs`
- Create: `tests/Gravitas.Tests/Core/MatterAgentContextTests.cs`

**Design:**

- Replace `GridWorld World { get; }` on `IMatterAgent` with `GravitasWorldContext Context { get; }`.
- All world access flows through `agent.Context.World`.
- `SolidBody.Setup(...)` verifies the agent and collider bind to the same context.
- `LSCollider.InitializeWithNoBody(...)` binds static colliders to `agent.Context`.
- Static collider APIs should take a context-bound agent, not a raw world.

**Tasks:**

- [x] Write tests that constructing a body/collider with an agent from one context registers with that context.
- [x] Write tests that attempting to combine body/collider/agent ownership across contexts fails with a clear exception.
- [x] Update `IMatterAgent`.
- [x] Update all body and collider world access to `Context.World`.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~MatterAgentContextTests|FullyQualifiedName~GravitasPhysicsServiceTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- `GridWorld` handling is explicit through `GravitasWorldContext`, and agents cannot accidentally point physics at a world without its owning runtime state.

**Status:** Complete for Phase 4. Replaced `IMatterAgent.World` with `IMatterAgent.Context`, moved body and static-collider binding through `agent.Context`, added shared test-agent support, added context-ownership tests, and removed the now-unused raw `GridWorld`-to-context lookup bridge from `GravitasWorldContext`. Verified with focused Phase 4/Phase 3 regression tests plus `Release` and `ReleaseLean` solution build/test runs.

## Phase 5: Move Collision Partitioning Into A Context Service

**Purpose:** Isolate active partitions, partition pools, collision versioning, and partition distribution by world context.

**Files:**

- Create: `src/Gravitas/Core/GravitasCollisionService.cs`
- Delete: `src/Gravitas/Core/CollisionManager.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/Core/PhysicsManager.cs`
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

- [x] Write tests for partition activation/deactivation within one context.
- [x] Write tests for two contexts with overlapping world coordinates: partition activation in one context does not affect the other.
- [x] Write tests proving `PhysicsPartition.Distribute()` does not reuse static scratch state.
- [x] Write tests proving `PhysicsPartition.OnRemoveFromVoxel(...)` releases through the owner service and throws or logs a clear invariant violation when no owner exists.
- [x] Write tests proving empty partition cleanup removes the partition from the voxel exactly once and does not double-release it.
- [x] Create `GravitasCollisionService`.
- [x] Convert `CollisionManager` methods into instance methods on the service.
- [x] Convert `PhysicsPartition` to service-owned behavior with a required `SetOwner(GravitasCollisionService owner)` or owner-taking rent helper.
- [x] Replace direct pool release calls with `voxel.TryRemovePartition<PhysicsPartition>()` so `OnRemoveFromVoxel(...)` owns the release flow.
- [x] Remove or empty the old static `CollisionManager` after all callers use `Context.Collisions`.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasCollisionServiceTests|FullyQualifiedName~PhysicsPartitionTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Collision partitioning is world-local, pair resolution is context-local, and static collision service state is gone.

**Status:** Complete for Phase 5. Added `GravitasCollisionService`, exposed it through `GravitasWorldContext.Collisions`, moved partition activation, versioning, pooling, cull distribution, and collision distribution behind the context, converted `PhysicsPartition` to owner-required behavior, and deleted the old static `CollisionManager`. Empty partition cleanup now removes the partition from the voxel and lets `PhysicsPartition.OnRemoveFromVoxel(...)` release through the owning service. Verified with focused Phase 5 tests plus `Release` and `ReleaseLean` solution build/test runs.

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

- [x] Write tests for raycast result ordering in one context.
- [x] Write tests for circlecast duplicate suppression in one context.
- [x] Write tests proving query buffers do not leak across two contexts.
- [x] Convert `Raycaster` into `GravitasRaycastService`.
- [x] Convert `Circlecaster` into `GravitasCirclecastService`.
- [x] Convert `RayCasterWorker` cached fields into instance-owned worker state.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasRaycastServiceTests|FullyQualifiedName~GravitasCirclecastServiceTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Query state is context-local and static query buffers are gone.

**Status:** Complete for Phase 6. Added context-owned `GravitasRaycastService` and `GravitasCirclecastService`, exposed them through `GravitasWorldContext`, removed the old query facades, and moved `RayCasterWorker` cached state into the context-owned `RaycastAxisWorker`. `SolidBody.CheckGround()` now uses the owning context's clock, settings, and circlecast service. Also normalized the raycast worker axis during preparation after focused tests exposed that the old raw-vector projection treated non-normalized ray directions as normalized. Verified with focused Phase 6 tests plus `Release` and `ReleaseLean` solution build/test runs.

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

- [x] Write tests for frame waits against one context clock.
- [x] Write tests for two contexts advancing at different frame counts.
- [x] Create `GravitasCoroutineService`.
- [x] Convert `CoroutineManager` behavior to instance service methods.
- [x] Update yield instructions to context/clock-bound constructors.
- [x] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasCoroutineServiceTests|FullyQualifiedName~LockedYieldInstructionTests"
dotnet build Gravitas.slnx --configuration Release
```

**Exit criteria:**

- Coroutines and wait instructions are deterministic per context.

**Status:** Complete for Phase 7. Added context-owned `GravitasCoroutineService`, exposed it through `GravitasWorldContext.Coroutines`, and made `GravitasWorldContext.Simulate()` advance context-local coroutines after the context clock and physics step. `WaitForFrames`, `WaitForNextSimulate`, and `WaitForRealSeconds` now bind to an explicit context through service factories, and `SolidBody.SkipGrounding()` uses its owning context coroutine service. Removed the old static `CoroutineManager` instead of leaving a compatibility facade. Verified with focused Phase 7 tests plus `Release` and `ReleaseLean` solution build/test runs.

**Pre-Phase 8 follow-up:** Converted live context-owned `SwiftBucket` loops in `GravitasCoroutineService`, `GravitasPhysicsService`, and `GravitasCollisionService` to captured-peak `TryGetValue(...)` iteration. This avoids the `IsAllocated(...)` plus indexer double probe, tolerates removals during callback-driven simulation loops, and prevents newly added coroutines from running in the same simulation tick. At the time, the only remaining old `SwiftBucket` loops were isolated to the legacy static `PhysicsManager` slated for removal/reduction in Phase 8.

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

- [x] Search for `PhysicsManager.`, `CollisionManager.`, `Raycaster.`, `Circlecaster.`, and `CoroutineManager.` references.
- [x] Replace all runtime references with context-owned services.
- [x] Remove temporary adapters introduced during earlier phases.
- [x] Update docs to show context-first usage.
- [x] Run:

```bash
rg -n "PhysicsManager\.|CollisionManager\.|Raycaster\.|Circlecaster\.|CoroutineManager\." src/Gravitas tests/Gravitas.Tests
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

**Exit criteria:**

- Runtime code no longer relies on mutable static manager state.

**Status:** Complete for Phase 8. Added tests pinning context-local impulse timing and explicit settings-saver application, moved `SolidBody` timing/environment reads to its owning `GravitasWorldContext`, added `PhysicsSettingsSaver.ApplyTo(...)`, removed the legacy static `PhysicsManager` file, and converted `ColliderSettings` priority lookup away from a mutable public dictionary. README, AGENTS, and benchmark docs now describe the context-owned runtime API. Verified with the static-reference search plus `Release` and `ReleaseLean` solution build/test runs.

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

- [x] Add benchmark fixtures that create deterministic contexts and dispose them per benchmark case.
- [x] Add `[MemoryDiagnoser]` to each benchmark class.
- [x] Capture baseline JSON after the implementation stabilizes.
- [x] Run:

```bash
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- list
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- all -j Short -i
```

**Exit criteria:**

- The new architecture has benchmark coverage for lifecycle, registration, partitioning, and query services.

**Status:** Complete for Phase 9. Added deterministic benchmark fixtures plus `WorldContextBenchmarks`, `CollisionPartitionBenchmarks`, and `QueryServiceBenchmarks`, covering owned/attached context lifecycle, grid creation, empty simulation frames, dynamic/static sphere registration and partitioning, partitioned simulation, raycasts, circlecasts, and overlapping-coordinate two-context query isolation. Removed stale copied benchmark helpers from earlier template code and fixed the benchmark runner's `all` command so it selects all benchmarks without an interactive prompt. Captured JSON baseline artifacts via the short in-process benchmark run. Follow-up allocation hardening is tracked in [`2026-05-22-runtime-allocation-hardening-plan.md`](2026-05-22-runtime-allocation-hardening-plan.md).

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

## Closure Risk Review

- Resolved: `PhysicsPartition` now has owner-required behavior through `GravitasCollisionService`.
- Resolved: partition cleanup uses owner-local release through the partition removal path instead of mixed manual release.
- Resolved: raycasts and circlecasts are context-owned services that resolve context-local collider IDs.
- Resolved: raycast worker state moved out of the static cache and into the query service.
- Resolved: wait instructions bind to the owning context clock through `GravitasCoroutineService`.
- Resolved: `PhysicsSettingsSaver` applies settings to an explicit `GravitasWorldContext`.
- Resolved: `ColliderSettings` no longer exposes mutable public priority dictionaries.
- Follow-up: benchmarked steady-state query/simulation allocations are tracked separately in [`2026-05-22-runtime-allocation-hardening-plan.md`](2026-05-22-runtime-allocation-hardening-plan.md).

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

**Closure status:** Satisfied. No additional follow-up tasks were found in this plan beyond the allocation hardening work already moved to a separate plan.
