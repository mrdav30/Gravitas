# Host Integration

Gravitas does not own the application loop. A game engine, server, deterministic
simulation harness, or unit test owns the outer lifecycle and calls Gravitas
phases at deterministic points.

## Lifecycle Contract

Use the LSF lifecycle names as a mental model, not as engine-specific APIs:

| Phase           | Host responsibility                                                     | Gravitas call                                              |
| --------------- | ----------------------------------------------------------------------- | ---------------------------------------------------------- |
| `Setup`         | Create package defaults, host resources, and worlds.                    | Create or attach `GravitasWorldContext`.                   |
| `Initialize`    | Bind agents, transforms, colliders, bodies, settings, and grids.        | Initialize colliders and bodies.                           |
| `Execute`       | Apply deterministic commands or network input in ordered frame batches. | No direct call; mutate host-owned state before simulation. |
| `Simulate`      | Advance the authoritative fixed step.                                   | `context.Simulate()`.                                      |
| `LateSimulate`  | Finish deterministic end-of-frame work.                                 | `context.LateSimulate()`.                                  |
| `Visualize`     | Interpolate or publish presentation state.                              | `context.Visualize()`.                                     |
| `LateVisualize` | Finish presentation-only work.                                          | `context.LateVisualize()`.                                 |
| `Deactivate`    | Pool/despawn agents and release registrations.                          | `body.Deactivate()` or `collider.Deactivate()`.            |
| `Quit`          | Shut down the host process/session.                                     | `context.Dispose()` when the context is no longer needed.  |

Authoritative simulation state belongs in `Simulate` and `LateSimulate`.
`Visualize` and `LateVisualize` are for interpolation and presentation; they
should not be used to apply gameplay commands or physics corrections.

## Minimal Host Agent

The host provides `IMatterAgent` so Gravitas can bind an object to a context and
fixed transform without depending on an engine, ECS, rendering, or a specific
object model.

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

`IsParent` marks whether an agent is intended to be a top-level hierarchy owner,
but hierarchy collision filtering is bound explicitly on colliders. Hosts that
need parent-child or sibling collision suppression should initialize the
colliders first, then call `childCollider.SetParent(parentCollider)`.
`SetParent(...)` walks the collider-parent chain to the top collider and stores
the top parent as a dimension-tagged collider key on the child, so sibling
filtering does not depend on an engine `transform.parent` or any other engine
hierarchy object. Mixed mode can bind a 2D collider under a 3D collider, or a 3D
collider under a 2D collider, without aliasing the separate collider ID tables.

```csharp
weaponCollider.SetParent(characterCollider);
leftFootCollider.SetParent(characterCollider);
rightFootCollider.SetParent(characterCollider);
```

Use `ClearParent()` when a collider leaves that host hierarchy without being
deactivated.

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

Dynamic matter usually has a host agent, one collider, and one `SolidBody`.

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
LSSphereCollider collider = new();
collider.Material = new PhysicsMaterial(
    staticFriction: Fixed64.One,
    dynamicFriction: Fixed64.Half,
    restitution: Fixed64.FromFraction(1, 4));
SolidBody body = new(agent, collider)
{
    Mass = Fixed64.One
};

body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, isDynamic: true);
```

Initialization binds the body and collider to `agent.Context`, allocates a
context-local body slot, allocates a context-local collider ID, calculates shape
runtime data, and partitions the collider.

Pure 2D scenes use the same host-agent shape, but select the 2D runtime path and
create 2D body/collider types:

```csharp
context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;

LSCircleCollider2D collider = new(Fixed64.Half);
SolidBody2D body = new(agent, collider)
{
    Mass = Fixed64.One
};

body.Initialize(agent.Transform.Position.ToVector2d(), isDynamic: true);
```

The 2D projection uses the LSF X/Z convention: world X maps to 2D X and world Z
maps to 2D Y. World Y is height or future embedding metadata.

## Surface Materials

`PhysicsMaterial` is deterministic collider surface data. Assign it on
`LSCollider` or `LSCollider2D` before simulation when a surface needs explicit
static friction, dynamic friction, restitution, or combine policies.

```csharp
collider.Material = PhysicsMaterial.Frictionless;
```

Shape definitions and compound parts can also carry materials for authored
setup:

```csharp
var compound = new LSCompoundCollider(
    CompoundColliderPart.Sphere(
        Fixed64.Half,
        -Vector3d.Right,
        PhysicsMaterial.Bouncy),
    CompoundColliderPart.Cuboid(
        Vector3d.One,
        Vector3d.Right,
        PhysicsMaterial.Default));
```

Compound parts without an explicit material use the owning compound collider's
material when the private part colliders are materialized. Query hits still
identify colliders; hosts can read `hit.Collider.Material` or the mixed hit's
dimension-specific collider reference instead of duplicating material data in
every hit payload.

## Local Physical Filtering

`IgnoredCollisionLayers` is a collider-owned physical filter. Assign it on
`LSCollider` or `LSCollider2D` when one collider should ignore selected physical
layers without changing the context-wide collision matrix.

```csharp
projectileCollider.Layer = new PhysicsLayer(3);
ownerCollider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(projectileCollider.Layer);
```

The rule is symmetric at pair time: if either collider ignores the other
collider's layer, the physical interaction is rejected. The mask affects
discrete collision pairs, trigger pairs, internal CCD target eligibility, and
grounding/support acceptance. Public query services do not use this mask; query
results are controlled by the caller's query `PhysicsLayerMask`, trigger option,
and excluded-collider argument.

## Static Collider Setup

Use `InitializeWithNoBody(...)` for bodyless host geometry. This registers and
partitions the collider but does not create a simulated body.

```csharp
using Gravitas.Colliders;

LSCuboidCollider floor = new();
floor.InitializeWithNoBody(agent);
```

A body with all translation axes frozen is different from a bodyless collider.
Set `SolidBody.FreezeAxes = BodyFreezeAxes3D.Position` or
`SolidBody2D.FreezeAxes = BodyFreezeAxes2D.Position` when the object should keep
body-owned state but behave as static-equivalent for solver and partition
mobility. Bodyless 3D colliders are still registered as colliders and can
participate in queries and candidate generation, but 3D pair creation still
requires at least one collider in the pair to have a body. Bodyless 2D colliders
bind through `LSCollider2D.InitializeWithNoBody(IMatterAgent)` and can
participate in queries, trigger events, layer filtering, cleanup, and static
collision response.

Freeze axes are authoritative body state. Partial position freezes, such as
`BodyFreezeAxes3D.PositionY` or `BodyFreezeAxes2D.PositionX`, remain dynamic
members and only constrain the matching solver and integration axis. Rotation
freezes are explicit through `BodyFreezeAxes3D.RotationX/Y/Z` or
`BodyFreezeAxes2D.Rotation`.

## 3D Constraints And Ragdolls

`context.Constraints3D` owns deterministic 3D joints and ragdoll runtimes. A
joint links two active `SolidBody` instances through explicit local frames,
optional angular limits, optional motor payloads, and a linked-collider
collision policy:

```csharp
using Gravitas.Constraints;

Joint3D shoulder = context.Constraints3D.RegisterJoint(new JointDefinition3D(
    upperArmBody,
    torsoBody,
    upperArmLocalFrame,
    torsoLocalFrame,
    JointType3D.ConeTwist,
    JointLimit3D.ConeTwist(maxConeAngle, maxTwistAngle),
    JointMotor3D.Disabled,
    JointCollisionPolicy.SuppressLinked));
```

Enabled joints are solved in the same 3D discrete islands as contacts during
`LateSimulate()`, using `PhysicsSettings.DiscreteSolverIterations`. Linked
sleep/wake behavior follows the island graph, so pushing one awake link wakes
the connected dynamic articulation.

`Joint3D.LastSolveMetrics` exposes the latest solver row count, anchor error,
angular limit error, motor error, impulse, and clamped-row counters. Hosts can
read these values directly or enable diagnostics and consume
`GravitasJointDiagnosticView` when debugging ragdoll stability, motor drive
strength, or authoring mistakes. Stabilization is still controlled by the
discrete solver iteration count and by physically named motor values; Gravitas
does not expose extra public stiffness/compliance knobs until stress evidence
shows a clear API shape.

Ragdolls are authoring conveniences over the same joint model. Hosts provide
stable link IDs, the linked bodies/colliders, authored joint definitions, and a
self-collision policy:

```csharp
RagdollRuntime3D ragdoll = context.Constraints3D.RegisterRagdoll(
    new RagdollDefinition3D(links, joints, RagdollSelfCollisionPolicy.SuppressAdjacentLinks));

ragdoll.ActivateDynamic();
// Later, when deterministic animation or host control takes over again:
ragdoll.DeactivateToKinematic();
```

Animation systems remain outside Gravitas. A deterministic animation library can
compute target joint rotations, then pass caller-owned motor payloads before the
fixed step:

```csharp
context.Constraints3D.SetRagdollPoseTargets(ragdoll, jointMotors);
context.Simulate();
context.LateSimulate();
```

Foot IK, hand IK, animation events, blending, and engine animator hooks belong
in host or animation packages. Gravitas owns only the deterministic physical
constraints, collision filtering, activation state, diagnostics, and
serialization boundary.

## Pure 2D Constraints And Ragdolls

`context.Constraints2D` owns deterministic pure 2D joints and ragdoll runtimes.
A 2D joint links two active `SolidBody2D` instances through local planar
anchors, scalar local angles, optional scalar limits, optional motor payloads,
and the same linked-collider collision policy used by 3D articulations:

```csharp
using Gravitas.Constraints;

Joint2D hinge = context.Constraints2D.RegisterJoint(new JointDefinition2D(
    forearmBody2D,
    upperArmBody2D,
    new JointFrame2D(Vector2d.Right * Fixed64.Half, Fixed64.Zero),
    new JointFrame2D(-Vector2d.Right * Fixed64.Half, Fixed64.Zero),
    JointType2D.Pin,
    JointLimit2D.Angular(-Fixed64.HalfPi, Fixed64.HalfPi),
    JointMotor2D.Disabled,
    JointCollisionPolicy.SuppressLinked));
```

Enabled 2D joints solve in the same pure 2D response islands as contact rows
during `LateSimulate()`. They use `PhysicsSettings.DiscreteSolverIterations`,
respect `SolidBody2D.FreezeAxes`, and write deterministic metrics to
`Joint2D.LastSolveMetrics`. Current joint types are distance, pin/revolute,
weld/fixed, and prismatic/slider. Angular values are radians; distance and
slider limits are world units in the X/Z simulation plane.

2D ragdolls follow the same data-first authoring shape as 3D ragdolls, but the
payload is planar:

```csharp
RagdollRuntime2D ragdoll2D = context.Constraints2D.RegisterRagdoll(
    new RagdollDefinition2D(links2D, joints2D, RagdollSelfCollisionPolicy.SuppressAdjacentLinks));

ragdoll2D.ActivateDynamic();
ragdoll2D.DeactivateToKinematic();
```

Animation, IK, pose selection, and engine-specific skeleton state stay outside
Gravitas. A deterministic animation package can compute `JointMotor2D` values
and pass them through `context.Constraints2D.SetRagdollPoseTargets(...)` before
the fixed step.

If a bodyless 3D collider moves after initialization, the host must call
`floor.Simulate()` after mutating its transform so bounds and partition
membership are refreshed. Pure 2D bodyless colliders currently rebuild from
their agent transform during the 2D broad-phase pass.

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

- `context.Simulate()` advances the clock, runs enabled simulate-phase services,
  runs the mixed lifecycle path only in `Mixed`, advances lockstep coroutines,
  then invokes simulate hooks.
- `context.LateSimulate()` marks visualization accumulation for reset. In the 3D
  path it prepares CCD frame state, integrates dynamic bodies, evaluates
  host-driven kinematic active sweeps, refreshes dynamic collider partitions,
  distributes and solves discrete contacts, updates active pair/contact
  maintenance, and updates sleep state after response. It also runs each enabled
  2D/mixed late-simulate path, then invokes late-simulate hooks.
- `context.Visualize()` advances interpolation accumulation, updates enabled 2D
  and/or 3D body visual transforms, runs the mixed lifecycle path only in
  `Mixed`, then invokes visualize hooks.
- `context.LateVisualize()` invokes context hooks only. Add built-in late
  presentation work only when there is a real runtime invariant for it.

For the 3D path, the fixed-step order is integrate-then-collide inside
`LateSimulate`: queued forces affect motion before the discrete collision pass
for that same frame.

For kinematic CCD, hosts must write deterministic target transforms before
`context.LateSimulate()`. Gravitas captures the body pose at the start of the
late step, reads the host transform as the requested target, and sweeps between
those two poses when `ContinuousCollisionMode` resolves to `Continuous` or
`Auto`. Dynamic targets crossed before the first static-style blocker are woken
and position-corrected as if hit by an infinite-mass source. The first
static-style blocker clips the kinematic pose and updates the bound transform to
the clipped value so render and physics state do not diverge.

## Replay Contract

For deterministic runs, hosts should treat command application as a separate
ordered input phase before `context.Simulate()`. Given the same initial context,
settings, world state, command order, and frame count, Gravitas should replay to
the same authoritative body, collider, clock, and contact state.

Hosts can compute a compact deterministic frame hash after a fixed step and
compare that value across peers, servers, replay runners, or restored snapshots:

```csharp
using Chronicler;

context.Simulate();
context.LateSimulate();

ChronicleHash hash = context.ComputeReplayHash();
SendFrameHashToLockstepPeer(context.FrameCount, hash);
```

`ComputeReplayHash()` hashes context settings, physical environment values,
clock state, body state, collider shape/filter state, retained
continuation-affecting pair/contact state, and active CCD handoff state. It does
not hash host object identity, delegates, diagnostics buffers, debug draw
commands, query scratch buffers, or visualization interpolation caches. Use
`GravitasReplayHashMode.AuthoritativeWithSolverCaches` when investigating drift
inside solver/cache state that is useful for RCA but not part of the ordinary
authoritative continuation contract.

The returned `ChronicleHash` is a deterministic conformance signal, not a
cryptographic hash and not a compatibility promise across package version
changes.

The current runtime order has two important consequences:

- Teleports or transform mutations made before `Simulate()` refresh dynamic
  collider bounds and can create contacts in that same fixed step's
  `LateSimulate()` call.
- Forces and accelerations queued before `Simulate()` move the body during
  `LateSimulate()` before the 3D discrete collision pass runs.

Visualization phases are non-authoritative. Use them to publish interpolated
positions, rotations, and diagnostic draw data to a renderer or host adapter,
not to change physics state that must replay. In pure 2D mode,
`context.Visualize()` publishes dynamic `SolidBody2D` X/Z position and yaw
rotation back to each agent transform while preserving the host transform's
vertical height.

## Settings And Environment

Each context owns its own `PhysicsSettings` and `PhysicsEnvironment`.

```csharp
context.SetFrameRate(60);

PhysicsSettings settings = PhysicsSettings.DefaultSettings();
settings.PoolingEnabled = true;
settings.RestitutionVelocityThreshold = Fixed64.FromFraction(1, 4);
context.ApplySettings(settings);

context.Environment.Gravity = (Fixed64)9.8f;
```

Different contexts can run at different frame rates and with different settings
in the same process. Frame-derived values such as `DeltaTime`, `FrameCount`, and
`TotalTime` are read through the context.

Per-body gravity tuning lives on the body. `SolidBody.GravityScale` multiplies
the context environment gravity for that body; `Fixed64.Zero` disables
environment-gravity acceleration and grounded weight for the body.
`SolidBody2D.GravityScale` applies the same policy to that body's planar
`Gravity` vector.

## Queries

2D and 3D queries are context services:

```csharp
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

PhysicsLayerMask layerMask = PhysicsLayerMask.FromLayer(0);
SwiftList<Physics3DHit> circleHits = new();

bool hit = context.Query3D.Raycast(
    origin,
    direction,
    maxDistance,
    out Physics3DHit rayHit,
    layerMask);

int circleHitCount = context.Query3D.OverlapCircleAll(origin, radius, layerMask, circleHits);
for (int i = 0; i < circleHitCount; i++)
{
    Physics3DHit circleHit = circleHits[i];
    // Consume circle-overlap hits.
}

SwiftList<Physics3DHit> sweepHits = new();
int sweepHitCount = context.Query3D.SweepSphereAll(
    origin,
    origin + direction * maxDistance,
    radius,
    layerMask,
    sweepHits,
    excludedCollider: null);

SwiftList<Physics2DHit> planarHits = new();
int planarHitCount = context.Query2D.RaycastAll(
    start2D,
    end2D,
    layerMask,
    planarHits);

SwiftList<PhysicsMixedHit> mixedHits = new();
int mixedHitCount = context.QueryMixed.SweepSphereAgainst2DAll(
    origin,
    origin + direction * maxDistance,
    radius,
    layerMask,
    mixedHits,
    excludedCollider: null);
```

High-volume lockstep systems should prefer batch APIs when issuing many related
queries in one frame. Requests, closest-hit outputs, all-hit buffers, and range
buffers remain caller-owned and reusable:

```csharp
PhysicsRaycast3DRequest[] rayRequests = new PhysicsRaycast3DRequest[agentCount];
Physics3DHit[] closestRayHits = new Physics3DHit[agentCount];
SwiftList<Physics3DHit> allRayHits = new(agentCount * 4);
PhysicsQueryHitRange[] rayRanges = new PhysicsQueryHitRange[agentCount];

for (int i = 0; i < agentCount; i++)
{
    rayRequests[i] = new PhysicsRaycast3DRequest(
        sensorOrigins[i],
        sensorTargets[i],
        layerMask);
}

int closestHitCount = context.Query3D.RaycastBatch(rayRequests, closestRayHits);
int allHitCount = context.Query3D.RaycastAllBatch(rayRequests, allRayHits, rayRanges);

for (int requestIndex = 0; requestIndex < agentCount; requestIndex++)
{
    PhysicsQueryHitRange range = rayRanges[requestIndex];
    for (int hitIndex = 0; hitIndex < range.Count; hitIndex++)
    {
        Physics3DHit queryHit = allRayHits[range.Start + hitIndex];
        // Consume this request's sorted hits.
    }
}
```

The same pattern applies to pure 2D and mixed batch queries. Polygon batch
queries take a separate flat vertex span plus per-request vertex ranges, so the
host owns polygon vertex storage explicitly for the duration of the batch call.

All-hit query APIs use caller-owned buffers. The 3D query service writes
`Physics3DHit` values, pure 2D queries write `Physics2DHit` values, and mixed
queries write `PhysicsMixedHit` values. They clear the supplied list, write
sorted hits into it, and return the count so hot query loops do not allocate
enumerators or temporary hit lists.

Query APIs use `PhysicsLayerMask` as an include mask. Use
`PhysicsLayerMask.FromLayer(...)` for a single layer,
`PhysicsLayerMask.FromLayers(...)` for several layers, `PhysicsLayerMask.All`
for every layer, and `PhysicsLayerMask.None` when no collider should be
included.

Ground checks use `context.Settings.GroundCheckLayerMask`. Hosts need to set
this explicitly for their own layer model before relying on grounding behavior.
`SolidBody.Initialize(...)` performs an initial ground probe after the collider
is registered, so bodies only start grounded when the configured ground mask
actually hits suitable geometry.

`SolidBody.GroundingMode` controls who owns grounded state:

- `Automatic` is the default. Gravitas updates `IsGrounded`, `HitPoint`,
  `WasGrounded`, `GroundNormal`, `HitPlatform`, and normal-force cache from
  deterministic ground probes.
- `Manual` disables automatic probes. Hosts can call `UseManualGrounding(...)`,
  `SetManualGrounding(...)`, `ClearManualGrounding()`, and
  `UseAutomaticGrounding(...)` when deterministic heightmaps or another
  host-owned ground source should drive grounded state without paying query
  cost. While in manual mode, the host is responsible for keeping `IsGrounded`
  state current. `UseManualGrounding()` and `ClearManualGrounding()` leave the
  body airborne until the host supplies manual support or returns ownership to
  Gravitas.

Each body selects its probe shape through `GroundProbeMode`:

- `Ray` preserves the sorted raycast/self-exclusion behavior.
- `SweptSphere` uses the true swept-sphere query and the body collider as the
  excluded collider.
- `Auto` uses swept spheres for sphere, capsule, cylinder, and wide cuboid
  bodies, and ray probes for point-like or unsupported bodies.

`GroundProbeRadius` can override the derived swept radius. Leave it at zero to
derive radius from the collider shape. Ground probes ignore the body's own
collider, collider-local ignored physical layers, and ordinary movable dynamic
bodies; valid ground targets are bodyless colliders, position-frozen bodies, or
kinematic bodies. `WasGrounded` stores the grounded value captured before the
latest authoritative simulation refresh or explicit manual grounding change, so
hosts can distinguish landing, remaining grounded, and leaving support without
deriving that transition from visual-frame state.

`SolidBody2D` exposes the same ownership model through `GroundingMode`,
`UseManualGrounding(...)`, `SetManualGrounding(...)`, `ClearManualGrounding()`,
and `UseAutomaticGrounding(...)`, but the values are planar support state rather
than world-height state. `GroundPoint`, `GroundNormal`, `LastGroundedPosition`,
and `GroundUpDirection` are `Vector2d` values where 2D X maps to world X and 2D
Y maps to world Z. Automatic 2D grounding first uses current-frame 2D contact
manifolds against bodyless, position-frozen, or kinematic support colliders
included by `GroundCheckLayerMask`; if no contact candidate is valid, it runs a
deterministic `GroundProbeMode2D.Ray` or `GroundProbeMode2D.SweptCircle` probe
through `context.Query2D`. Grounded 2D integration removes acceleration and
velocity components that push into the support normal while preserving
tangential planar motion.

## Deactivation And Disposal

Deactivate runtime objects before pooling or destroying their host wrappers:

```csharp
body.Deactivate();
floor.Deactivate();
```

`SolidBody.Deactivate()` deactivates the collider, removes the body from the
physics service, and clears the dynamic ID. `LSCollider.Deactivate()` clears
partition membership, removes collision-pair references, clears explicit parent
binding, returns active pairs to the pool when enabled, and releases the
collider ID.

Use `context.Reset()` for a reusable session context. Reset detaches Gravitas
partition payloads from GridForge voxels and clears context-local runtime state
while preserving the world and its grids. Use `context.Dispose()` when the
context is finished. If the context owns its world, dispose will also dispose
the world.

## Serialization Boundary

`SolidBody`, `SolidBody2D`, `LSCollider`, and `LSCollider2D` implement
Chronicler record methods for state transfer into existing host-created objects.
Treat serialization as populate-existing-runtime-shell behavior, not
construct-from-data behavior.

Read [Serialization And Replay](SERIALIZATION.md) before changing serialized
fields, load defaults, runtime cache rebuilds, or replay tests.
