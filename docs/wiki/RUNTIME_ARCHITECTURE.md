# Runtime Architecture

The runtime architecture is context-first. `GravitasWorldContext` is the single
host-facing object that ties together one explicit `GridWorld`, deterministic
clock state, settings, physical environment values, and the mutable services
needed to run one simulation.

## Context Ownership

`GravitasWorldContext` can be created in two ways:

- `CreateOwned(...)` creates a new `GridWorld` and disposes it with the context.
- `Attach(world, takeOwnership: false)` binds to a host-created active
  `GridWorld`.

Internally, contexts use a weak ownership registry to prevent one active
`GridWorld` from being attached to multiple active contexts. This registry is
process-wide metadata, not simulation state. Collider IDs, bodies, partitions,
pairs, queries, and coroutines remain context-local.

## Services

| Service | Owned state |
| --- | --- |
| `GravitasPhysicsService` | Dynamic body bucket, collider ID table, reusable collider IDs, collision-pair pool, active collision-pair queue, simulation switch. |
| `GravitasCollisionService` | Active partition bucket, inactive partition pool, duplicate voxel checker, collision distribution version, cull distributor. |
| `GravitasRaycastService` | Raycast worker, intersection buffer, hit buffer, duplicate collider checker, query version. |
| `GravitasCirclecastService` | Hit buffer, duplicate collider checker, query version. |
| `GravitasCoroutineService` | Active lockstep coroutine bucket and context-bound wait instruction factories. |
| `GravitasLifecycleHooks` | Ordered callbacks for simulate, late simulate, visualize, late visualize, reset, and frame-rate change. |

The split is intentional: host code should mostly see the context and a few
domain objects, while mutable implementation details stay inside services.

## Frame Phases

Current context methods run in this order:

```text
Simulate
  Clock.Simulate
  Physics.Simulate
    Collisions.CheckAndDistributeCollisions
  Coroutines.Simulate
  Hooks.InvokeSimulate

LateSimulate
  Clock.LateSimulate
  Physics.LateSimulate
    ProcessActiveCollisionPairs
    StiffBody.LateSimulate for dynamic bodies
  Hooks.InvokeLateSimulate

Visualize
  Clock.Visualize
  Physics.Visualize
    StiffBody.OnVisualize for dynamic bodies
  Hooks.InvokeVisualize

LateVisualize
  Physics.LateVisualize
    StiffBody.LateVisualize for dynamic bodies
  Hooks.InvokeLateVisualize
```

`Reset` clears the clock and all context-local service state, then invokes reset
hooks. `SetFrameRate` and `ApplySettings` update the clock's frame rate and
invoke frame-rate-change hooks.

## Clock State

`GravitasClock` stores:

- `FrameRate`
- `DeltaTime`
- `InvDeltaTime`
- `FrameCount`
- `TotalTime`
- `AccumulatedTime`
- `ExpectedAccumulation`
- `ResetAccumulation`

Simulation code should use context time values rather than wall-clock APIs.
`Visualize` advances deterministic accumulation by the fixed delta, not by
elapsed real time.

## Registration Model

Dynamic bodies are stored in a `SwiftBucket<StiffBody>`. Their `DynamicId` is
the bucket index returned by `GravitasPhysicsService.AssimilateBody(...)`.

Colliders are stored in a context-local `LSCollider?[]` table. Collider IDs
start at `1`; released IDs are pushed into `_cachedColliderIds` for reuse.
Because IDs are context-local, two different contexts can both have collider ID
`1` without ambiguity. All lookups must go through the owning context's
`GravitasPhysicsService.TryGetColliderById(...)`.

`StiffBody.Setup(...)` requires the agent and collider to belong to the same
context. `CollisionPair.Initialize(...)` rejects colliders from different
contexts. These checks are core invariants.

## Body State

`StiffBody` owns:

- position as `Vector2d` ground position plus height, exposed as `Position3d`.
- rotation and derived basis vectors.
- visual position/rotation interpolation buffers.
- linear and angular velocity, acceleration, impulses, drag, friction, and
  restitution inputs.
- mass and inverse mass.
- inertia tensor and inverse inertia tensor.
- grounding state and ground probe settings.
- Chronicler record data.

Body movement currently happens in `StiffBody.LateSimulate()`, called by
`GravitasPhysicsService.LateSimulate()`. Non-kinematic movable bodies process
forces, update velocities, apply position/rotation changes, run a grounding
circlecast, and then update collider partition state through `Collider.Simulate()`.

Kinematic bodies read their host transforms during `LateSimulate`, update
authoritative body position/rotation from those transforms, and then update
visual values.

## Collider State

`LSCollider` owns:

- context binding and context-local ID.
- optional `StiffBody` binding or host-only `IMatterAgent`.
- active/trigger state.
- layer index.
- shape type and shape priority.
- local offset, scale-derived size, radius, area, bounds, and partition
  coordinates.
- contact and trigger events.
- parent/child metadata used to suppress sibling collisions.
- collision-pair references and holders.
- raycast and spherecast query version markers.

Dynamic colliders are updated by their bodies. Bodyless/static colliders are
not owned by the dynamic body bucket, so a host that moves one after
initialization must call `collider.Simulate()` to refresh bounds and partition
membership.

## Settings And Environment

`PhysicsSettings` holds the frame rate, collision matrix, pooling switch, and
ground-check layer mask. The collision matrix uses `true` for collide and
`false` for ignore.

`PhysicsEnvironment` holds physical and culling values such as gravity, air
density, speed caps, friction transition speed, damping, and culling scores.
Environment values are mutable per context.

## Lifecycle Hooks

Lifecycle hooks are internal context registrations. Each hook has an owner name
and order. Hooks are sorted by order, then owner name. Invocations snapshot the
hook list before callbacks run, which keeps iteration stable if a callback
registers or unregisters another hook.

## Coroutines

`GravitasCoroutineService` runs lockstep coroutines during `context.Simulate()`.
The supported wait instructions are context-bound:

- `WaitForFrames`
- `WaitForNextSimulate`
- `WaitForRealSeconds`

They use the context clock, so coroutine behavior stays frame-deterministic and
multi-context safe.

## Runtime Invariants

- A live `GridWorld` has at most one live `GravitasWorldContext`.
- Every collider belongs to one context for its active lifetime.
- Collision pairs cannot cross context boundaries.
- Partition ownership is through `GravitasCollisionService`; partitions are
  returned to the owning service pool through voxel removal.
- Query services resolve collider IDs through their owning context only.
- Simulation state changes belong in fixed-step phases, not visualization
  phases.
