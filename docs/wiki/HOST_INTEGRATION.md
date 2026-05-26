# Host Integration

Gravitas does not own the application loop. A game engine, server, deterministic
simulation harness, or unit test owns the outer lifecycle and calls Gravitas
phases at deterministic points.

## Lifecycle Contract

Use the old LSF lifecycle names as a mental model, not as Unity-specific API:

| Phase | Host responsibility | Gravitas call |
| --- | --- | --- |
| `Setup` | Create package defaults, host resources, and worlds. | Create or attach `GravitasWorldContext`. |
| `Initialize` | Bind agents, transforms, colliders, bodies, settings, and grids. | Initialize colliders and bodies. |
| `Execute` | Apply deterministic commands or network input in ordered frame batches. | No direct call; mutate host-owned state before simulation. |
| `Simulate` | Advance the authoritative fixed step. | `context.Simulate()`. |
| `LateSimulate` | Finish deterministic end-of-frame work. | `context.LateSimulate()`. |
| `Visualize` | Interpolate or publish presentation state. | `context.Visualize()`. |
| `LateVisualize` | Finish presentation-only work. | `context.LateVisualize()`. |
| `Deactivate` | Pool/despawn agents and release registrations. | `body.Deactivate()` or `collider.Deactivate()`. |
| `Quit` | Shut down the host process/session. | `context.Dispose()` when the context is no longer needed. |

Authoritative simulation state belongs in `Simulate` and `LateSimulate`.
`Visualize` and `LateVisualize` are for interpolation and presentation; they
should not be used to apply gameplay commands or physics corrections.

## Minimal Host Agent

The host provides `IMatterAgent` so Gravitas can bind an object to a context and
fixed transform without depending on Unity, ECS, rendering, or a specific object
model.

```csharp
using Gravitas;
using Gravitas.Support;

internal sealed class HostMatterAgent : IMatterAgent
{
    public HostMatterAgent(
        GravitasWorldContext context,
        FixedTransform transform,
        bool isParent = true)
    {
        Context = context;
        Transform = transform;
        IsParent = isParent;
    }

    public GravitasWorldContext Context { get; }

    public FixedTransform Transform { get; }

    public bool IsParent { get; }

    public bool IsInteracting { get; set; }
}
```

`IsParent` and the collider parent/child fields are used to suppress sibling
collisions in host hierarchies. This is the engine-agnostic replacement for the
old prototype's reliance on transform parenting.

## Creating A Context

Use `CreateOwned(...)` when Gravitas should own the `GridWorld` lifetime:

```csharp
using Gravitas;

using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
```

Use `Attach(...)` when the host creates and owns the `GridWorld`:

```csharp
using Gravitas;
using GridForge.Grids;

using GridWorld world = new();
using GravitasWorldContext context = GravitasWorldContext.Attach(world);
```

Pass `takeOwnership: true` to `Attach(...)` only when disposing the context
should also dispose the supplied world.

One active `GridWorld` can be attached to only one active
`GravitasWorldContext`. This prevents collider IDs, partitions, query buffers,
and pair state from being ambiguously shared across simulations.

## Configure Grid Coverage First

Colliders partition into existing GridForge voxels. Add grids that cover the
simulation area before initializing bodies and colliders.

```csharp
using FixedMathSharp;
using GridForge.Configuration;

context.World.TryAddGrid(
    new GridConfiguration(
        new Vector3d(-16, -4, -16),
        new Vector3d(16, 8, 16)),
    out _);
```

If no voxel exists for a collider's bounds, the collider cannot be distributed
into partitions and will not participate in partition-backed collision/query
work for that area.

## Dynamic Body Setup

Dynamic matter usually has a host agent, one collider, and one `StiffBody`.

```csharp
using FixedMathSharp;
using Gravitas;
using Gravitas.Colliders;
using Gravitas.Support;

FixedTransform transform = new(
    Vector3d.Zero,
    FixedQuaternion.Identity,
    Vector3d.One);

HostMatterAgent agent = new(context, transform);
LSSphereCollider collider = new();
StiffBody body = new(agent, collider)
{
    Mass = Fixed64.One
};

body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, isDynamic: true);
```

Initialization binds the body and collider to `agent.Context`, allocates a
context-local body slot, allocates a context-local collider ID, calculates shape
runtime data, and partitions the collider.

## Static Collider Setup

Use `InitializeWithNoBody(...)` for bodyless host geometry. This registers and
partitions the collider but does not create a simulated body.

```csharp
using Gravitas.Colliders;

LSCuboidCollider floor = new();
floor.InitializeWithNoBody(agent);
```

An immovable `StiffBody` is different from a bodyless collider. Immovable bodies
are placed in the partition static list. Bodyless colliders are still registered
as colliders and can participate in queries and candidate generation, but pair
creation still requires at least one collider in the pair to have a body.
If a bodyless collider moves after initialization, the host must call
`floor.Simulate()` after mutating its transform so bounds and partition
membership are refreshed.

## Fixed Loop

A simple deterministic loop looks like this:

```csharp
while (running)
{
    ApplyOrderedCommandsForFrame();

    context.Simulate();
    context.LateSimulate();

    context.Visualize();
    context.LateVisualize();
}
```

Most real hosts call `Simulate` and `LateSimulate` from a fixed-rate scheduler
and call the visualization phases from the render/update loop.

Current service order matters:

- `context.Simulate()` advances the clock, distributes collisions, advances
  lockstep coroutines, then invokes simulate hooks.
- `context.LateSimulate()` marks visualization accumulation for reset, processes
  active collision-pair notifications/culling, advances dynamic bodies, updates
  collider partitions through body simulation, then invokes late-simulate hooks.
- `context.Visualize()` advances interpolation accumulation, updates body visual
  transforms, then invokes visualize hooks.
- `context.LateVisualize()` runs body late-visualize hooks and context hooks.

Do not assume an engine-style integrate-then-collide order. The current
prototype checks/distributes collisions during `Simulate` and advances bodies in
`LateSimulate`.

## Replay Contract

For deterministic runs, hosts should treat command application as a separate
ordered input phase before `context.Simulate()`. Given the same initial context,
settings, world state, command order, and frame count, Gravitas should replay to
the same authoritative body, collider, clock, and contact state.

The current alpha order has two important consequences:

- Teleports or transform mutations made before `Simulate()` refresh dynamic
  collider bounds and can create contacts in that same `Simulate()` call.
- Forces and accelerations queued before `Simulate()` do not move the body until
  `LateSimulate()`.

Visualization phases are non-authoritative. Use them to publish interpolated
positions, rotations, and diagnostic draw data to a renderer or host adapter,
not to change physics state that must replay.

## Settings And Environment

Each context owns its own `PhysicsSettings` and `PhysicsEnvironment`.

```csharp
context.SetFrameRate(60);

PhysicsSettings settings = PhysicsSettings.DefaultSettings();
settings.PoolingEnabled = true;
context.ApplySettings(settings);

context.Environment.Gravity = (Fixed64)9.8f;
```

Different contexts can run at different frame rates and with different settings
in the same process. Frame-derived values such as `DeltaTime`, `FrameCount`, and
`TotalTime` are read through the context.

## Queries

Raycasts and circle-overlap queries are context services:

```csharp
using Gravitas.Raycasting;
using Gravitas.Support;
using SwiftCollections;

PhysicsLayerMask layerMask = PhysicsLayerMask.FromLayer(0);
SwiftList<LSRaycastHit> circleHits = new();

bool hit = context.Raycasts.Raycast(
    origin,
    direction,
    maxDistance,
    out LSRaycastHit rayHit,
    layerMask);

int circleHitCount = context.CircleQueries.OverlapCircleAll(origin, radius, layerMask, circleHits);
for (int i = 0; i < circleHitCount; i++)
{
    LSRaycastHit circleHit = circleHits[i];
    // Consume circle-overlap hits.
}

SwiftList<LSRaycastHit> sweepHits = new();
int sweepHitCount = context.Raycasts.SweepSphereAll(
    origin,
    origin + direction * maxDistance,
    radius,
    layerMask,
    sweepHits,
    excludedCollider: null);
```

All-hit query APIs use caller-owned `SwiftList<LSRaycastHit>` buffers. They
clear the supplied list, write sorted hits into it, and return the count so hot
query loops do not allocate enumerators or temporary hit lists.

Query APIs use `PhysicsLayerMask` as an include mask. Use
`PhysicsLayerMask.FromLayer(...)` for a single layer,
`PhysicsLayerMask.FromLayers(...)` for several layers,
`PhysicsLayerMask.All` for every layer, and `PhysicsLayerMask.None` when no
collider should be included.

Ground checks use `context.Settings.GroundCheckLayerMask`. The default preserves
old prototype example exclusions only as a starting point; hosts should set this
explicitly for their own layer model before relying on grounding behavior.
`StiffBody.Initialize(...)` performs an initial ground probe after the collider
is registered, so bodies only start grounded when the configured ground mask
actually hits suitable geometry.

Each body selects its probe shape through `GroundProbeMode`:

- `Ray` preserves the sorted raycast/self-exclusion behavior.
- `SweptSphere` uses the true swept-sphere query and the body collider as the
  excluded collider.
- `Auto` uses swept spheres for sphere, capsule, cylinder, and wide cuboid
  bodies, and ray probes for point-like or unsupported bodies.

`GroundProbeRadius` can override the derived swept radius. Leave it at zero to
derive radius from the collider shape. Ground probes ignore the body's own
collider and ordinary movable dynamic bodies; valid ground targets are bodyless
colliders, immovable bodies, or kinematic bodies.

## Deactivation And Disposal

Deactivate runtime objects before pooling or destroying their host wrappers:

```csharp
body.Deactivate();
floor.Deactivate();
```

`StiffBody.Deactivate()` deactivates the collider, removes the body from the
physics service, and clears the dynamic ID. `LSCollider.Deactivate()` clears
partition membership, removes collision-pair references, returns active pairs to
the pool when enabled, and releases the collider ID.

Use `context.Reset()` for a reusable session context. Use `context.Dispose()`
when the context is finished. If the context owns its world, dispose will also
dispose the world.

## Serialization Boundary

`StiffBody` and `LSCollider` implement Chronicler record methods for state
transfer into existing host-created objects. Treat serialization as populate
existing runtime shells, not construct arbitrary engine objects from data.
Host-specific bindings, renderers, and external transforms should remain
host-owned.
