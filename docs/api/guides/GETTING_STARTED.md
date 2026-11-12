# Getting Started

Build a world with one sphere at the origin. Give it a push along +X, advance
one fixed step, and read its new position. You will create one
`GravitasWorldContext` for the simulation, one GridForge grid for spatial
coverage, and one host adapter for the sphere.

Your host creates the objects and owns input, rendering, networking, and
scene lifetime. Gravitas advances the sphere's authoritative physics state
inside the context.

## 1. Install A Package

Use the standard package when you want built-in MemoryPack support:

```bash
dotnet add package Gravitas
```

Use the Lean package when serialization is provided elsewhere and you do not
want a direct MemoryPack runtime dependency:

```bash
dotnet add package Gravitas.Lean
```

Both variants expose the same core physics API and target `netstandard2.1` and
`net10.0`.

## 2. Give Gravitas A Host Object

`IMatterAgent` is the engine-neutral bridge between a host object and Gravitas.
It provides the object's physics context, fixed-point transform, hierarchy
intent, and interaction state.

```csharp
using FixedMathSharp;
using Gravitas;

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

An engine adapter can wrap a scene object, entity, or server-side simulation
record behind this interface. Keep engine-native rigid-body simulation disabled
for objects whose authoritative motion comes from Gravitas.

## 3. Create A Context And Spatial Coverage

Create one context per isolated simulation. `CreateOwned()` also creates a
`GridWorld` whose lifetime belongs to the context.

```csharp
using System;
using FixedMathSharp;
using Gravitas;
using GridForge.Configuration;

using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
context.SetFrameRate(60);

if (!context.World.TryAddGrid(
        new GridConfiguration(
            new Vector3d(-16, -4, -16),
            new Vector3d(16, 8, 16)),
        out _))
{
    throw new InvalidOperationException("Could not create spatial coverage.");
}
```

Gravitas uses existing GridForge voxels for collision partitions and spatial
queries. Create coverage for the simulation area before registering colliders. A
collider outside every grid cannot participate in partition-backed collision or
query work there.

If your host already owns a `GridWorld`, bind it instead:

```csharp
using Gravitas;
using GridForge.Grids;

using GridWorld world = new();
using GravitasWorldContext context = GravitasWorldContext.Attach(world);
```

The default `takeOwnership: false` leaves disposal of `world` with the host.
These are alternative context-creation examples; use one ownership path per
world.

## 4. Create A Body And Collider

Start the sphere at the origin with a radius of 0.5 and a mass of 1. The
collider defines its shape and material. The body stores its simulated motion.
Bind both to the host agent so they share its context and transform.

```csharp
using FixedMathSharp;
using Gravitas;
using Gravitas.Colliders;
using Gravitas.Materials;

FixedTransform transform = new(
    Vector3d.Zero,
    FixedQuaternion.Identity,
    Vector3d.One);

HostMatterAgent agent = new(context, transform);

LSSphereCollider collider = new(
    ColliderShapeDefinition.Sphere(
        Fixed64.Half,
        new PhysicsMaterial(
            staticFriction: Fixed64.One,
            dynamicFriction: Fixed64.Half,
            restitution: Fixed64.FromFraction(1, 4))));

SolidBody body = new(agent, collider)
{
    Mass = Fixed64.One
};

body.Initialize(
    Vector3d.Zero,
    FixedQuaternion.Identity,
    PhysicsMobilityType.Dynamic);
```

Initialization assigns context-local runtime identity, prepares the shape and
mass properties, and partitions the collider. Runtime motion is explicit:

- `Dynamic` bodies are solver controlled.
- `Kinematic` bodies are host controlled and sampled deterministically.
- `Static` bodies are immobile until repositioned through their explicit pose
  APIs.

For engine authoring, you can create Gravitas-owned runtime colliders from
data-only definitions instead of branching over concrete types:

```csharp
ColliderShapeDefinition shape =
    ColliderShapeDefinition.Sphere(Fixed64.Half);
LSCollider authoredCollider = shape.CreateCollider();
```

## 5. Advance The Fixed Step

Apply the impulse before the fixed step. The impulse points along +X, so the
sphere's X position is positive when the step finishes.

```csharp
body.AddLinearImpulse(Vector3d.Right * body.Mass);
context.Simulate();
context.LateSimulate();

Console.WriteLine(body.Position3d.X > Fixed64.Zero); // True
```

`Simulate()` advances the context clock and primary services. `LateSimulate()`
completes deferred collision, constraint, pair, cleanup, and post-step work.
Keep them in the same deterministic fixed-frame transaction.

### Publish The Sphere's Pose

On a render update, publish the sphere's visual pose to the host:

```csharp
context.Visualize();
context.LateVisualize();
```

These phases publish presentation state. Do not apply authoritative gameplay or
physics corrections from them.

### Check Replay State

For lockstep or replay validation, hash the context after the authoritative
step:

```csharp
var replayHash = context.ComputeReplayHash();
```

The hash covers replay-relevant physics state and excludes host identity,
delegates, query scratch, diagnostics buffers, and interpolation caches.

## 6. Choose A Runtime Mode

The default path is 3D. Set the mode before registering dimensional runtime
objects when you need another path:

```csharp
context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
```

| Mode     | Behavior                                                         |
| -------- | ---------------------------------------------------------------- |
| `ThreeD` | Runs the 3D body, collision, query, and constraint services      |
| `TwoD`   | Runs the first-class X/Z planar physics services                 |
| `Both`   | Runs 2D and 3D side by side without cross-dimensional contacts   |
| `Mixed`  | Enables deliberate 2D/3D contacts, queries, CCD, and diagnostics |

Pure 2D uses `SolidBody2D` and `LSCollider2D`. Mixed mode embeds 2D geometry as
finite slabs centered on the host transform's world Y; it is a distinct physical
model, not an alias for `Both`.

## 7. Release Runtime Ownership

Deactivate registered objects before pooling or destroying their host wrappers:

```csharp
body.Deactivate();
```

Disposing the context releases its context-local services and, for an owned
context, its `GridWorld`. Use `context.Reset()` when the context itself should
survive into another session.

## Where To Go Next

- [Host Integration](HOST_INTEGRATION.md) covers bodyless geometry, hierarchy
  filtering, materials, constraints, ragdolls, queries, replay, and teardown.
- [2D, 3D, And Runtime Modes](DIMENSIONS.md) explains planar coordinates, mixed
  embedding, and dimensional service ownership.
- [Collision Pipeline](COLLISION_PIPELINE.md) follows a collider from GridForge
  partitioning through narrow phase, response, events, and cleanup.
- [Query Services](QUERY_SERVICES.md) shows closest-hit, all-hit, overlap,
  swept-shape, mixed, and batch workflows.
- [Serialization And Replay](SERIALIZATION.md) defines the populate-existing-
  shell boundary and replay contract.
- [API Reference](https://mrdav30.github.io/Gravitas/api/Gravitas.html) lists
  the complete public surface.
