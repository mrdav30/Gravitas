# Serialization And Replay

Gravitas uses Chronicler for deterministic state transfer into host-created
runtime shells. The goal is not to materialize a full engine object graph from
data. The host creates the context, world, agents, transforms, body instances,
and concrete collider shape types; Chronicler populates deterministic physics
state into those existing objects.

This contract supports lockstep debugging, rollback-style validation, save-state
testing, and replay tools.

## Quick Read

- Serialize authoritative simulation state, not host object identity.
- Load into existing runtime shells created by the host.
- Rebuild context-owned runtime caches after load.
- Populate settings before body/collider state when snapshots include
  `PhysicsSettingsSaver`.
- Use `GravitasWorldContext.ComputeReplayHash()` for compact conformance checks.
- Treat hash strings as deterministic non-cryptographic signals, not
  cross-version compatibility values.
- Keep `ReleaseLean` compiling when serialization-related fields or attributes
  change.

## Ownership Contract

| Host-created shell                                         | Serialized state                                                                                                                                              |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GravitasWorldContext` and `GridWorld`                     | Settings that affect deterministic execution.                                                                                                                 |
| `IMatterAgent`, `FixedTransform`, engine wrappers          | Body position, rotation, motion type, velocities, force/torque stores, gravity scale, sleep, CCD, freeze axes.                                                |
| `SolidBody`, `SolidBody2D`                                 | 3D grounding state and 2D planar support state.                                                                                                               |
| Concrete `LSCollider` and `LSCollider2D` types             | Active/trigger state for bodyless trigger volumes, layer, local ignored physical layers, material, local offset, shape inputs, mixed half-thickness override. |
| Compound runtime shells and private part colliders         | Authored shape/part values needed to rebuild deterministic geometry.                                                                                          |
| Existing registered `Joint3D`, `Joint2D`, ragdoll runtimes | Joint enabled state, type, frames, limits, motors, linked collision policy, ragdoll activation state.                                                         |
| Renderer, ECS, networking, pooling, editor state           | Nothing. These remain host-owned.                                                                                                                             |

Runtime-owned state that should not be serialized:

- context-local collider IDs and service indices.
- GridForge partition coordinate lists and active partition payloads.
- collision pairs, pair holder references, warm runtime pair caches, query
  buffers, diagnostic buffers, and pooled collections.
- derived collider bounds and transformed mesh caches. Absolute endpoints,
  corners, and contact/query witnesses are materialized on demand from canonical
  rigid-frame and local geometry rather than stored as runtime authority.
- context-local joint IDs, ragdoll IDs, articulation suppression tables, and
  service-owned joint/ragdoll arrays.
- lifecycle hooks, delegates, renderer callbacks, and event subscribers.
- host transform object identity.
- 3D visual interpolation buffers and presentation-only rotation speed state.

On load, bodies publish restored authoritative position and rotation into their
existing host transform. A restored 3D quaternion is scale-safely normalized
before any runtime shape or host state observes it; a zero quaternion resolves
to identity. 3D visual interpolation buffers reset from that authoritative state
instead of being treated as replay truth.

Continuous-collision modes are validated at the settings and body publication
boundaries. `Inherit`, `Discrete`, `Continuous`, and `Auto` load normally;
undefined byte-cast values throw `ArgumentOutOfRangeException`. A rejected body
value does not replace its previously valid mode, and a rejected
`PhysicsSettingsSaver` does not replace the context's existing settings.

`BodyMotionType` is authoritative serialized state and defaults to `Dynamic`
when the field is absent. Motion type and freeze masks are validated before any
loaded body state is published. Loading a different role into an already
registered shell reconciles simulated-body membership, partitions, CCD state,
and solver caches through the same atomic lifecycle invariants as a public role
transition. Undefined roles or freeze bits fail without partially publishing the
new state.

Mass retains its existing `"Mass"` record key. Loading writes backing state and
refreshes derived inertia and awake membership at the final load commit; it
preserves recorded sleep and motion rather than invoking the public mass
setter's wake policy. Restoring an active body clears contact and connected-joint
impulse caches and prepared CCD state while retaining pair and joint identity.

## Recordable Types

The 3D body record starts with `BodySchemaVersion = 1` and a signed 64-bit
`LastGroundCheckFrame`. A nonnegative value is an absolute simulation frame;
`-1` means no cached probe. Missing or unsupported schema versions and invalid
negative stamps reject before any body fields are changed, consistently in
JSON and MemoryPack. Unversioned narrow-frame body snapshots are not accepted.

The 2D body does not persist its ground-check frame: its support lookup cache is
invalidated on population and rebuilt by simulation. Its record shape is
unchanged. Neither body record restores the containing world's clock. Hosts
must coordinate matching timeline and runtime-shell state before continuation;
there is no live-world rewind API.

| Type                                    | What it records                                                                                                                                                                                         | What it does not own                                             |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| `SolidBody`                             | 3D position/height, rotation, motion type, freeze axes, motion stores, mass, COM, gravity scale, sleep, CCD, grounding/probe state, owned collider state.                                               | `FixedTransform` identity, service IDs, partitions, pairs.       |
| `SolidBody2D`                           | X/Z position, scalar rotation, motion type, freeze axes, planar motion stores, scalar angular state, mass, COM, scalar moment policy, gravity, grounding/probe state, sleep, CCD, owned collider state. | Host transform identity, runtime service IDs, query buffers.     |
| `LSCollider`                            | 3D active state, bodyless trigger state, layer/filter state, local ignored physical mask, material, shape state.                                                                                        | Context-owned collider ID, partition identity, pairs/events.     |
| `LSCollider2D`                          | 2D active state, bodyless trigger state, layer/filter state, material, shape-local values, mixed half-thickness override.                                                                               | Context-owned collider ID, private runtime pair/partition state. |
| `ColliderShapeDefinition`               | Data-only 3D authoring/import values for primitive, mesh, and compound part inputs.                                                                                                                     | Runtime body, context, collider ID, pairs, hierarchy, events.    |
| `ColliderShapeDefinition2D`             | Data-only 2D authoring/import values for circle, capsule, AABB, convex polygon, triangle convenience, and compound parts.                                                                               | Runtime body, context, collider ID, pairs, hierarchy, events.    |
| `PhysicsSettingsSaver`                  | Frame rate, collision matrix, ground mask, CCD settings, restitution threshold, retained partition cleanup, runtime mode, mixed 2D thickness.                                                           | Runtime service state.                                           |
| `Joint3D` / `Joint2D`                   | Mutable joint continuation state: enabled flag, type, frames/anchors, limits, motors, linked-collider policy.                                                                                           | Body link construction, service joint IDs, solver caches.        |
| `RagdollRuntime3D` / `RagdollRuntime2D` | Runtime activation state for existing handles.                                                                                                                                                          | Definitions, link bodies, colliders, joint ownership.            |
| `PhysicsLayer` / `PhysicsLayerMask`     | JSON/MemoryPack-friendly value fields.                                                                                                                                                                  | Chronicler graph identity.                                       |

Collider geometry can derive default COM/mass properties for new shells, but
populated snapshots restore body-owned COM state directly where that state is
authoritative.

A successful `SolidBody.TryReconfigureCollider(...)` becomes ordinary
authoritative body/collider state: the published body root, local offset, and
primitive geometry are recorded by the existing serialization paths. The
operation itself, detached fit candidate, returned blocker, invalidated pairs,
and refreshed partition coordinates are not serialized. A blocked or invalid
attempt publishes nothing, so its serialized payload and authoritative replay
hash remain unchanged.

Pair-local contact caches and joint solver caches are rebuildable runtime data
unless a drift investigation explicitly hashes them through
`GravitasReplayHashMode.AuthoritativeWithSolverCaches`.

When solver caches are included, contact and query-adjacent witnesses are hashed
as the canonical `Origin`, `Rotation`, `LocalPoint`, and `LocalDisplacement`
components of their `ContactAnchor` or `ContactAnchor2D` frame. A clipped or
unavailable rotated offset or absolute world point is not replay truth. Derived
bounds and other rebuildable shape caches remain outside the authoritative shape
hash and are covered only by their dedicated cache section.

Joint and ragdoll handles are valid serialization targets only while registered
with their owning constraint service. Endpoint teardown removes dependent joints
and any owning ragdoll before collider identity release; removed handles reject
save and load operations instead of applying state to a later pooled body or
collider lifetime. Context reset and disposal invalidate those handles by the
same rule.

## Replay Workflow

A deterministic replay or rollback restore should follow this shape:

1. Create or attach a `GravitasWorldContext` with matching settings and
   GridForge world setup.
2. Create host agents, transforms, bodies, and concrete collider shapes in the
   same stable order the host expects.
3. Materialize authored 3D and 2D compound assets from
   `ColliderShapeDefinition`, `ColliderShapeDefinition2D`, and compound part
   data before binding.
4. Populate settings first when the snapshot includes `PhysicsSettingsSaver`.
5. Populate body/collider/joint/ragdoll state into existing shells.
6. Continue fixed-step simulation from the restored frame using the same ordered
   input commands.

For dynamic replay tests, compare the uninterrupted simulation against a fresh
shell restored at frame N and advanced with the same subsequent inputs. Do not
compare runtime-owned service IDs or partition list identities; compare
authoritative body/collider values and externally observable collision/query
behavior.

## Replay Hashes

`GravitasWorldContext.ComputeReplayHash()` is the preferred compact conformance
signal for checking replay and rollback continuation.

Apply ordered host commands before each fixed step, call `Simulate()` and then
`LateSimulate()` exactly once, and compute the frame hash after both phases
complete. `Simulate()` advances the clock, coroutines and simulate hooks;
`LateSimulate()` completes physics and late hooks. A late-only trace omits part
of the authoritative host lifecycle. Presentation phases may run between fixed
steps and must leave the authoritative hash unchanged.

Compare relevant host events and coroutine effects separately: a physics-state
hash does not include arbitrary host state.

Compare peers using identical initial state, settings, ordered commands and
replay-hash schema versions. Record OS/CPU architecture, runtime and dependency
versions separately from authoritative state so differences can be diagnosed
without changing the equality contract. Native execution provides evidence for
the workloads and platforms exercised; repeated agreement on one machine does
not establish agreement across all platforms.

After Chronicler populates existing runtime shells, the restored context should
produce the same per-frame `ChronicleHash` sequence as the uninterrupted context
when both receive the same subsequent inputs.

The authoritative hash follows the same boundary as
`IRecordable.RecordData(...)`:

- serialized continuation state is included.
- host-owned bindings are excluded.
- rebuildable runtime caches are excluded.
- active cross-frame CCD handoff state is included because it can affect the
  next fixed step.
- runtime collider IDs are context-local lookup and pair keys; replay hashes use
  canonical live registration order with dense replay ordinals for collider,
  hierarchy, and pair identity, so deleted collider ID history and allocator
  holes are excluded.
- solver caches and diagnostic counters are included only in
  `AuthoritativeWithSolverCaches` mode for RCA.

Replay hashes use `Chronicler.Hashing.ChronicleHash` and Chronicler's
hash-writer mechanics.
Gravitas owns the physics-specific inclusion policy and deterministic ordering.
The root `gravitas.replay` section is version 2: frame and late-phase stamps
are signed 64-bit values, and elapsed time is ordered whole seconds (`long`)
then fractional seconds (`uint`). Affected body, pair, manifold, and cache
sections are versioned independently for their widened stamps. The `body.3d`
section is version 7 and includes the automatic support-normal threshold;
`body.2d` remains version 4. Hashes from older schema versions are not comparable.
Sparse transports that omit a default-valued `MotionType` deterministically
resolve it to `Dynamic`.
Both body recorders validate `GroundMinNormalDot` in `[0, 1]` before publishing
loaded state and default missing values to one half. 2D restores gravity and
support policy through backing state to preserve saved sleep, then rebuilds its
transient support ownership and probe timing. Its recorded fields and replay
hash layout remain unchanged.

```mermaid
flowchart LR
    Shells["Host-created shells"] --> Populate["Chronicler populate"]
    Populate --> Sim["Continue fixed-step simulation"]
    Sim --> Hash["ComputeReplayHash"]
    Hash --> Compare["Compare peers/replay runners"]
```

## Transport Notes

Import `Chronicler.Serialization` for `JsonRecordSerializer`,
`MemoryPackRecordSerializer`, converters, and `SerializationPayloadEditor`.
Recording contracts and helpers remain in `Chronicler`; replay hash values and
writers use `Chronicler.Hashing`. Rebuild consuming assemblies together when
updating these imports. See the [migration guide](../MIGRATION.md#chronicler-v100)
for the payload-editor replacements and hash compatibility notes.

Standard `Release` builds include MemoryPack support through the standard
dependency chain. `ReleaseLean` defines `GRAVITAS_DISABLE_MEMORYPACK`, excludes
the direct MemoryPack package, and relies on shim attributes needed for the same
core API to compile without built-in MemoryPack support.

When changing serialized fields, defaults, or load behavior:

- add or update save/populate tests under `tests/Gravitas.Tests/Serialization`.
- cover JSON and MemoryPack in standard builds.
- keep the same source compiling under `ReleaseLean`.
- run focused replay tests that serialize, restore into a fresh shell, and
  continue simulation.
- document whether the field is authoritative simulation state, host-owned
  binding, or runtime cache.

## Rules That Matter

- Do not turn Chronicler loading into a construct-from-data object factory.
- Do not serialize host bindings such as engine objects, renderers, or external
  transform object identity.
- Do not serialize context-local IDs as portable identity.
- Keep JSON and MemoryPack behavior aligned when both are supported.
- Treat presentation-only data and runtime caches as rebuildable.
- Add replay-continuation tests when state affects deterministic continuation.

## Source Map

| Area                            | Source                                                                                                                 |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| 3D body serialization           | [`src/Gravitas/Core/3D/SolidBody.Serialization.cs`](../../src/Gravitas/Core/3D/SolidBody.Serialization.cs)             |
| 2D body serialization           | [`src/Gravitas/Core/2D/SolidBody2D.Serialization.cs`](../../src/Gravitas/Core/2D/SolidBody2D.Serialization.cs)         |
| 3D collider replay/record state | [`src/Gravitas/Colliders/3D/LSCollider.ReplayHash.cs`](../../src/Gravitas/Colliders/3D/LSCollider.ReplayHash.cs)       |
| 2D collider replay/record state | [`src/Gravitas/Colliders/2D/LSCollider2D.ReplayHash.cs`](../../src/Gravitas/Colliders/2D/LSCollider2D.ReplayHash.cs)   |
| Settings saver                  | [`src/Gravitas/Settings/PhysicsSettingsSaver.cs`](../../src/Gravitas/Settings/PhysicsSettingsSaver.cs)                 |
| Replay hash service             | [`src/Gravitas/Determinism/GravitasReplayHashService.cs`](../../src/Gravitas/Determinism/GravitasReplayHashService.cs) |
| Serialization tests             | [`tests/Gravitas.Tests/Serialization`](../../tests/Gravitas.Tests/Serialization)                                       |
| Replay conformance tests        | [`tests/Gravitas.Tests/Determinism`](../../tests/Gravitas.Tests/Determinism)                                           |
