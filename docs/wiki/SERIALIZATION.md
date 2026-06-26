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
- `SolidBody`, `SolidBody2D`, `LSCollider`, and `LSCollider2D` instances.
- the concrete collider shape type, such as sphere, cuboid, circle, AABB,
  polygon, or compound.
- private runtime part colliders materialized by `LSCompoundCollider` and
  `LSCompoundCollider2D`.
- renderer, ECS, engine object, networking, pooling, editor, and event
  subscription state.

Serialized state:

- body position, rotation, velocities, acceleration stores, pending force,
  torque, impulse, and position-correction accumulators.
- mass, local center-of-mass offset, friction, restitution, sleep state, sleep
  thresholds, CCD mode, and movement flags.
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

`SolidBody` records 3D authoritative body state, including position,
height, rotation, linear/angular motion, pending force and torque state, mass,
local center-of-mass offset, response coefficients, sleep state, CCD mode, and
3D ground probe state. It does not record the `FixedTransform` binding.
Collider geometry can derive a default COM for new shells, but populated
snapshots restore the body-owned COM state directly.

`SolidBody2D` records pure 2D authoritative body state, including X/Z-projected
position, scalar rotation, linear motion, pending force state, scalar angular
velocity, applied and queued angular acceleration, angular-force policy, mass,
shape-refreshed scalar moment policy, body-local center-of-mass offset, response
coefficients, gravity, sleep state plus linear and angular sleep thresholds, CCD
mode, and its owned collider state. Populated snapshots restore explicit COM
state and then refresh scalar moment/inverse moment from the loaded collider
shape so deterministic replay continues with the same effective solver mass.

`LSCollider` records 3D collider filter and shape state. Runtime IDs are
context-owned and intentionally excluded from snapshots. Loading a bound
collider rebuilds runtime shape state and refreshes partition membership where
needed.

`ColliderShapeDefinition` is a data-only authoring/import surface for creating
runtime 3D colliders and compound parts. It is not a bound runtime shell: it has
no body, context, collider ID, partition coordinates, pairs, hierarchy state, or
events. Offline authored compound assets should serialize shape definitions and
stable part transforms, then let the host create `LSCompoundCollider` runtime
shells from that data before simulation or replay state is populated.

`ColliderShapeDefinition2D` is the matching data-only authoring/import surface
for pure 2D circle, AABB, and convex polygon shapes. Offline authored 2D
compound assets should serialize `ColliderShapeDefinition2D` plus
`CompoundColliderPart2D` local transforms, then let the host create
`LSCompoundCollider2D` runtime shells before Chronicler populates state.

`LSCollider2D` records pure 2D collider filter and shape state. Circle, AABB,
convex polygon, and compound colliders record their shape-specific values
through shape-local hooks rather than a central type switch. Compound part
definitions are host-created shell data, not runtime pair/partition state.
Loading shape data validates the input and rebuilds bounds without waking a
sleeping body just because state was populated.

`PhysicsSettingsSaver` records frame rate, collision matrix, ground-check layer
mask, default CCD mode, CCD TOI iteration limit, retained-partition cleanup settings,
runtime mode, and mixed 2D half-thickness. Applying it owns a new
`PhysicsSettings` instance for the target context and synchronizes the context
clock.

`PhysicsLayer` and `PhysicsLayerMask` still use direct JSON/MemoryPack-friendly
field annotations because they are small value helpers, not Chronicler graphs.

## Replay Workflow

A deterministic replay or rollback restore should follow this shape:

1. Create or attach a `GravitasWorldContext` with matching settings and
   GridForge world setup.
2. Create host agents, transforms, bodies, and concrete collider shapes in the
   same stable order the host expects. Authored 3D and 2D compound assets can
   materialize those shapes from `ColliderShapeDefinition` or
   `ColliderShapeDefinition2D` parts before binding.
3. Populate settings first when the snapshot includes `PhysicsSettingsSaver`.
4. Populate body/collider state into those existing shells.
5. Continue fixed-step simulation from the restored frame using the same ordered
   input commands.

For dynamic replay tests, compare the uninterrupted simulation against a fresh
shell restored at frame N and advanced with the same subsequent inputs. Do not
compare runtime-owned service IDs or partition list identities; compare
authoritative body/collider values and externally observable collision/query
behavior.

`GravitasWorldContext.ComputeReplayHash()` is the preferred compact conformance
signal for replay and rollback tests. After Chronicler populates existing
runtime shells, the restored context should produce the same per-frame
`GravitasReplayHash` sequence as the uninterrupted context when both receive
the same subsequent inputs. The authoritative hash follows the same boundary as
`IRecordable.RecordData(...)`: serialized continuation state is included, while
host-owned bindings and rebuildable runtime caches are excluded. Active
cross-frame CCD handoff state remains authoritative because it can affect the
next fixed step. Rebuildable CCD frame snapshots, query scratch data, diagnostic
buffers, visual interpolation state, and drift-debug counters are available
only through `GravitasReplayHashMode.AuthoritativeWithSolverCaches` when they
are useful for RCA.

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
