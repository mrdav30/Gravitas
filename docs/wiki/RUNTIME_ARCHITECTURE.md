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
| `GravitasPhysics2DService` | Pure 2D dynamic body bucket, monotonic collider ID table, 2D pair pool, pair-reference cleanup, layer/hierarchy-filtered narrow-phase/response processing, visualization publishing, simulation switch. |
| `GravitasMixedCollisionService` | Phase 10 mixed 2D/3D lifecycle owner, GridForge-backed mixed broad phase, stable mixed candidate-key buffer, mixed hierarchy filtering, duplicate suppression, awake dynamic membership refresh, mixed pair/response ownership, retained `PhysicsMixedPartition` cleanup, and lifecycle counters. |
| `GravitasCollisionService` | Active partition bucket, inactive partition pool, duplicate voxel checker, partition awake-state refresh, collision distribution version, cull distributor. |
| `GravitasCollision2DService` | GridForge-backed pure 2D partition bucket, inactive partition pool, duplicate voxel checker, awake dynamic membership refresh, 2D collision distribution version, retained partition cleanup. |
| `GravitasQuery2DService` | Pure 2D query candidate buffer, overlap-circle queries, segment raycasts, swept-circle queries, collider-stamped duplicate suppression, hit ordering. |
| `GravitasQuery3DService` | 3D segment worker, swept-sphere worker, X/Z circle overlap/proximity queries, intersection buffer, duplicate voxel checker, duplicate collider checker, raycast and circle query versions. |
| `GravitasCoroutineService` | Active lockstep coroutine bucket and context-bound wait instruction factories. |
| `GravitasDiagnosticSink` | Disabled-by-default diagnostic event buffer and engine-agnostic debug draw command buffer. |
| `GravitasLifecycleHooks` | Ordered callbacks for simulate, late simulate, visualize, late visualize, reset, and frame-rate change. |

The split is intentional: host code should mostly see the context and a few
domain objects, while mutable implementation details stay inside services.

## Frame Phases

Current context methods run in this order:

```text
Simulate
  Clock.Simulate
  If RuntimeMode includes ThreeD:
    Physics.Simulate
    PrepareCollisionPartitions for dynamic-body colliders
    Collisions.CheckAndDistributeCollisions
  If RuntimeMode includes TwoD:
    Physics2D.Simulate
    Collisions2D.CheckAndDistributeCollisions
    Process overlapping 2D partition candidate pairs
  If RuntimeMode == Mixed:
    MixedCollisions.Simulate
  Coroutines.Simulate
  Hooks.InvokeSimulate

LateSimulate
  Clock.LateSimulate
  If RuntimeMode includes ThreeD:
    Physics.LateSimulate
    ProcessActiveCollisionPairs
    StiffBody.LateSimulate for dynamic bodies
  If RuntimeMode includes TwoD:
    Physics2D.LateSimulate
    StiffBody2D.LateSimulate for dynamic 2D bodies
  If RuntimeMode == Mixed:
    MixedCollisions.LateSimulate
  Hooks.InvokeLateSimulate

Visualize
  Clock.Visualize
  If RuntimeMode includes ThreeD:
    Physics.Visualize
    StiffBody.OnVisualize for dynamic bodies
  If RuntimeMode includes TwoD:
    Physics2D.Visualize
    StiffBody2D.OnVisualize for dynamic 2D bodies
  If RuntimeMode == Mixed:
    MixedCollisions.Visualize
  Hooks.InvokeVisualize

LateVisualize
  If RuntimeMode includes ThreeD:
    Physics.LateVisualize
    StiffBody.LateVisualize for dynamic bodies
  If RuntimeMode includes TwoD:
    Physics2D.LateVisualize
    StiffBody2D.LateVisualize for dynamic 2D bodies
  If RuntimeMode == Mixed:
    MixedCollisions.LateVisualize
  Hooks.InvokeLateVisualize
```

This order is an alpha contract, not just an implementation detail. Ordered host
commands should be applied before `Simulate()`. Transform teleports made before
`Simulate()` are reflected in the same collision distribution pass because
dynamic-body colliders are refreshed before collisions are checked. Force and
acceleration commands made before `Simulate()` are stored on the body and
integrated during `LateSimulate()`. Collision response can mutate authoritative
body state during `Simulate()`, including the pure 2D response service. Body
force integration, 3D grounding, and post-integration collider refresh happen
during `LateSimulate()`.

Lifecycle hooks run after the built-in work for their phase. `Visualize()` and
`LateVisualize()` are presentation phases: they may update visual interpolation
state, but they must not mutate authoritative position, rotation, velocity,
partition membership, or collision state.

The replay expectation is: the same initial context, settings, world data,
ordered command sequence, and frame count should produce the same authoritative
body, collider, clock, and contact state across repeated runs. The current
contract is pinned by `GravitasSimulationPhaseOrderTests`.

`PhysicsSettings.RuntimeMode` is a validated bitmask with exact public settings
values: `ThreeD`, `TwoD`, `Both`, and `Mixed`. `Both` runs pure 2D and pure 3D
side by side without cross-dimensional contacts. `Mixed` runs both pure paths
plus the dedicated mixed lifecycle and broad-phase path. Mixed narrow phase
currently supports 3D spheres, cuboids, capsules, finite cylinders, compound
colliders, and mesh colliders against embedded 2D circle, AABB, and convex
polygon slabs. Mixed pair ownership and constrained impulse exchange are
implemented through `CollisionPairMixed` and `CollisionResponseMixed`; later
Phase 10 work fills in diagnostics, mixed queries, and CCD policy.

`Reset` clears the clock and all context-local service state, then invokes reset
hooks. `SetFrameRate` and `ApplySettings` update the clock's frame rate and
invoke frame-rate-change hooks.

Diagnostics are context-local and disabled by default. Runtime hooks can emit
force, query, ground-probe, contact, response, and velocity-delta events through
`context.Diagnostics` when enabled. Hosts can also capture colliders or simple
line/ray/point draw commands into the same context-owned sink for visualization
without adding renderer dependencies to core physics.

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
- `ResetAccumulationThisVisualize`

Simulation code should use context time values rather than wall-clock APIs.
`Visualize` advances deterministic accumulation by the fixed delta, not by
elapsed real time. `ExpectedAccumulation` is clamped to one simulation frame so
frame-accumulated visual interpolation cannot overshoot its target.

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
- deterministic sleep state, sleep thresholds, and wake handling.
- Chronicler record data.

Body movement currently happens in `StiffBody.LateSimulate()`, called by
`GravitasPhysicsService.LateSimulate()`. Non-kinematic movable bodies process
forces, update velocities, apply position/rotation changes, run their selected
ground probe through the context query services, and then update collider
partition state through `Collider.Simulate()`. `GravitasPhysicsService.Simulate()`
also performs a pre-distribution collider refresh for dynamic-body colliders so
host commands that teleport or reposition bodies before `Simulate()` are
reflected in the same collision distribution pass.

Body initialization does not assume grounded state. After the body collider is
registered and partitioned, initialization performs an explicit ground probe.
If no configured ground layer is hit, the body starts airborne. Ground probes
use `PhysicsSettings.GroundCheckLayerMask`, write hits into a body-owned buffer,
and ignore the body's own collider before accepting the closest hit. `Ray`
ground probes use `RaycastAll`; `SweptSphere` probes use `SweepSphereAll`;
`Auto` derives the mode from collider shape and probe radius. Stationary
grounded bodies can skip repeated simulation probes for a short frame window,
but movement of the last hit platform invalidates that guard. Ground probes
accept bodyless colliders, immovable bodies, and kinematic bodies as ground;
ordinary movable dynamic bodies are ignored.

`StiffBody` is the 3D body model. Pure 2D behavior uses `StiffBody2D` instead
of a dimension flag on `StiffBody`. This prevents temporary 3D colliders from
becoming the hidden implementation path for pure 2D bodies. The existing
position-as-ground-plus-height fields still belong to the current 3D y-up body
model.

Sleeping is body-owned and deterministic. A dynamic non-kinematic body can sleep
after linear and angular speed remain below configured thresholds for the body
sleep window. Sleep clears accumulated runtime motion state but leaves the
collider partitioned. Wake stimuli include force, impulse, collision with an
awake body, kinematic motion, host teleports, shape mutation, and explicit host
wake; waking updates awake membership in every current partition for that
collider.

Kinematic bodies read their host transforms during `LateSimulate`, update
authoritative body position/rotation from those transforms, and then update
visual values.

Visual rotation has two modes. With no rotation interpolation speed, Gravitas
uses clamped frame accumulation between the last visual rotation and the current
authoritative rotation. With a positive interpolation speed, each visualize call
speed-limits from the current presentation rotation toward the authoritative
target.

`StiffBody2D` owns the pure 2D body model. It is constructed from an
`IMatterAgent`, uses the agent context and transform bridge, and stores
`Vector2d` position, `Vector2d` linear velocity, scalar rotation, 2D gravity,
2D force integration, sleep/wake state, and Chronicler record data. Pure 2D
positions use world X/Z projection: `Vector2d.x = Vector3d.x` and
`Vector2d.y = Vector3d.z`. Kinematic 2D bodies read their agent transform during
`LateSimulate`. It intentionally has no y-up ground probe, height split, visual
interpolation state, or 3D inertia tensor. `ContinuousCollisionMode` is shared
with the 3D body path: `StiffBody2D` resolves body, hierarchy, then context
settings before committing movement, and uses `Query2D.SweepCircle` to clip fast
circle-proxy movement against static or kinematic 2D targets. Dynamic-vs-dynamic
2D CCD remains deferred until deterministic relative-velocity ordering is
specified. `GravitasPhysics2DService.Simulate()` runs 2D contact response and
events; `GravitasPhysics2DService.LateSimulate()` integrates active movable 2D
bodies; `GravitasPhysics2DService.Visualize()` publishes dynamic 2D position and
yaw rotation back to the host transform while preserving host vertical height.

## Collider State

`LSCollider` owns:

- context binding and context-local ID.
- optional `StiffBody` binding or host-only `IMatterAgent`.
- active/trigger state.
- layer index.
- shape type and shape priority.
- local offset, scale-derived size, radius, area, bounds, and runtime-shape
  versioning.
- partition coordinates, last grid bounds, partition-change flags, and
  broad-phase versioning.
- raycast and circle-query version markers.
- explicit parent/child metadata used to suppress parent-child and sibling
  collisions.
- collision-pair references and holder references.
- contact and trigger events.

The dense mutable groups inside `LSCollider` are split into focused internal
state helpers: runtime shape, partition, query, hierarchy, and pair state. The
public collider remains the host-facing shape object, while the helpers keep
ownership rules local enough for manifold and solver work to evolve without
turning the base collider into a bigger conditional path.
Runtime-shape snapshot commits are the source of truth for collider
position/rotation/scale/shape invalidation. Partition state advances the
broad-phase version from those commits, and collision pairs use broad-phase
version changes instead of maintaining a second position/rotation dirty path.

Dynamic colliders are updated by their bodies during the simulation phases.
Bodyless/static colliders are not owned by the dynamic body bucket, so a host
that moves one after initialization must call `collider.Simulate()` to refresh
bounds and partition membership.

Current 3D primitive colliders derive from `LSCollider`. Pure 2D colliders
derive from `LSCollider2D`; `LSCircleCollider2D`, `LSAABBoxCollider2D`, and
`LSPolygonCollider2D` own their own 2D bounds, support points, closest-point
math, and shape validation. They are registered with `GravitasPhysics2DService`,
partition through `GravitasCollision2DService`, and do not register with the 3D
`GravitasPhysicsService`. Bodyless 2D colliders bind through
`InitializeWithNoBody(IMatterAgent)` and use the same X/Z transform projection
for query and trigger visibility. Pure 2D partition storage uses GridForge
voxels on the internal Y=0 storage plane; that is a deterministic broad-phase
identity, not physical 3D thickness.
`LSCollider2D` shares the dimension-free collider helper pattern used by the 3D
path: query stamps, generic pair references, generic hierarchy state, and
runtime-shape dirty/version commits. Its runtime snapshot is 2D-specific only
where the payload is actually dimensional, and unchanged colliders skip bounds
and partition refresh work.

## Settings And Environment

`PhysicsSettings` holds the frame rate, runtime mode, collision matrix, pooling
switch, ground-check layer mask, and default continuous-collision mode. The
collision matrix uses `true` for collide and `false` for ignore.

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
- Pure 2D bodies and colliders are simulated by `GravitasPhysics2DService`;
  2D/3D contacts are produced only by `GravitasMixedCollisionService` when
  `PhysicsRuntimeMode.Mixed` is active.
- Partition ownership is through `GravitasCollisionService` for 3D and
  `GravitasCollision2DService` for pure 2D; partitions are returned to the
  owning service pool through voxel removal.
- Query services resolve collider IDs through their owning context only.
- Diagnostic events and draw commands describe one context only; body and
  collider IDs are not global.
- Simulation state changes belong in fixed-step phases, not visualization
  phases.
