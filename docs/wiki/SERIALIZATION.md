# Serialization And Replay

Gravitas uses Chronicler for deterministic state transfer into host-created
runtime shells. The goal is not to materialize a full engine object graph from
data. The host creates the context, world, agents, transforms, body instances,
and collider shape types; Chronicler populates deterministic physics state into
those existing objects.

This contract matters for lockstep debugging, rollback-style validation, save
state testing, and future replay tools.

## Ownership Contract

Serialize authoritative simulation state. Keep host bindings and runtime caches
owned by the runtime that created them.

Host-created shell:

- `GravitasWorldContext` and its `GridWorld`.
- `IMatterAgent` implementations.
- `FixedTransform` instances and any engine transform wrappers.
- `StiffBody`, `StiffBody2D`, `LSCollider`, and `LSCollider2D` instances.
- the concrete collider shape type, such as sphere, cuboid, circle, AABB, or
  polygon.
- renderer, ECS, engine object, networking, pooling, editor, and event
  subscription state.

Serialized state:

- body position, rotation, velocities, acceleration stores, pending force,
  torque, impulse, and position-correction accumulators.
- mass, friction, restitution, sleep state, sleep thresholds, CCD mode, and
  movement flags.
- 3D grounding state and ground probe configuration.
- collider active/trigger state, layer, local offset, shape dimensions, 2D
  mixed half-thickness override, and shape-derived inputs.
- settings that affect deterministic execution, through
  `PhysicsSettingsSaver`.

Runtime-owned state that should not be serialized:

- context-local collider IDs and service indices.
- GridForge partition coordinate lists and active partition payloads.
- collision pairs, pair holder references, warm runtime pair caches, query
  buffers, diagnostic buffers, and pooled collections.
- lifecycle hooks, delegates, renderer callbacks, and event subscribers.
- host transform object identity.
- 3D visual interpolation buffers and presentation-only rotation speed state.

On load, bodies publish restored authoritative position and rotation into their
existing host transform. 3D visual interpolation buffers are reset from that
authoritative state instead of treated as replay truth.

## Current Recordable Types

`StiffBody` records 3D authoritative body state, including position,
height, rotation, linear/angular motion, pending force and torque state, mass,
response coefficients, sleep state, CCD mode, and 3D ground probe state. It
does not record the `FixedTransform` binding.

`StiffBody2D` records pure 2D authoritative body state, including X/Z-projected
position, scalar rotation, linear motion, pending force state, mass, response
coefficients, gravity, sleep state, CCD mode, and its owned collider state.

`LSCollider` records 3D collider filter and shape state. Runtime IDs are
context-owned and intentionally excluded from snapshots. Loading a bound
collider rebuilds runtime shape state and refreshes partition membership where
needed.

`LSCollider2D` records pure 2D collider filter and shape state. Circle, AABB,
and convex polygon colliders record their shape-specific values through
shape-local hooks rather than a central type switch. Loading shape data validates
the input and rebuilds bounds without waking a sleeping body just because state
was populated.

`PhysicsSettingsSaver` records frame rate, collision matrix, ground-check layer
mask, default CCD mode, retained-partition cleanup settings, runtime mode, and
mixed 2D half-thickness. Applying it owns a new `PhysicsSettings` instance for
the target context and synchronizes the context clock.

`PhysicsLayer` and `PhysicsLayerMask` still use direct JSON/MemoryPack-friendly
field annotations because they are small value helpers, not Chronicler graphs.

## Replay Workflow

A deterministic replay or rollback restore should follow this shape:

1. Create or attach a `GravitasWorldContext` with matching settings and
   GridForge world setup.
2. Create host agents, transforms, bodies, and concrete collider shapes in the
   same stable order the host expects.
3. Populate settings first when the snapshot includes `PhysicsSettingsSaver`.
4. Populate body/collider state into those existing shells.
5. Continue fixed-step simulation from the restored frame using the same ordered
   input commands.

For dynamic replay tests, compare the uninterrupted simulation against a fresh
shell restored at frame N and advanced with the same subsequent inputs. Do not
compare runtime-owned service IDs or partition list identities; compare
authoritative body/collider values and externally observable collision/query
behavior.

## Transport Notes

Standard `Release` builds include MemoryPack support through the standard
dependency chain. `ReleaseLean` defines `GRAVITAS_DISABLE_MEMORYPACK`, excludes
the direct MemoryPack package, and relies on the shim attributes needed for the
same core API to compile without built-in MemoryPack support.

When changing serialized fields, defaults, or load behavior:

- add or update save/populate tests under `tests/Gravitas.Tests/Serialization`.
- cover JSON and MemoryPack in standard builds.
- keep the same source compiling under `ReleaseLean`.
- run focused replay tests that serialize, restore into a fresh shell, and
  continue simulation.
- document whether the field is authoritative simulation state, host-owned
  binding, or runtime cache.
