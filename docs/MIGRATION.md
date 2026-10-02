# Gravitas Migration Guide

## Partition membership ownership

`PhysicsPartition`, `PhysicsPartition2D` and `PhysicsMixedPartition` no longer
expose mutable `Contained*Objects` fields. Local membership storage now scales
with the number of IDs in a voxel instead of the highest context-wide collider
ID. Storage is internal so direct collection edits cannot bypass activation,
awake membership or retained-empty bookkeeping. Rebuild consuming assemblies.

Use the existing partition `Add*Object`, `Remove*Object` and `SetDynamic*Awake`
methods for an intentional membership change; normal collider registration and
movement remain owned by the collision service. For inspection:

| Previous access                                                                                          | Replacement                                                                                                                                     |
| -------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| 2D/3D `ContainedDynamicObjects.Count`, `ContainedKinematicObjects.Count`, `ContainedStaticObjects.Count` | `DynamicObjectCount`, `KinematicObjectCount`, `StaticObjectCount`                                                                               |
| 2D/3D `ContainedAwakeDynamicObjects.Count`                                                               | `AwakeDynamicObjectCount`                                                                                                                       |
| Mixed `ContainedDynamic3DObjects.Count`, etc.                                                            | `Dynamic3DObjectCount`, `Kinematic3DObjectCount`, `Static3DObjectCount` and the corresponding `2D` properties                                   |
| Mixed awake membership counts                                                                            | `AwakeDynamic3DObjectCount`, `AwakeDynamic2DObjectCount`, or total `AwakeDynamicObjectCount`                                                    |
| Membership enumeration                                                                                   | `CopyAllColliderIds(destination)` for pure dimensions; `Copy3DColliderIds(destination)` / `Copy2DColliderIds(destination)` for mixed partitions |

Copy methods clear and fill a caller-owned `SwiftList<int>` in ascending
collider ID order. Reuse that buffer for allocation-free inspection after its
capacity has warmed. Role counts include sleeping dynamic bodies; awake counts
do not.

Partition refresh retains memberships in voxels that remain covered, while
geometry and version updates still occur. Empty-payload reuse now selects the
last entry in a deterministic dense eligibility list instead of searching
through occupied payloads. Swap-back removal does not preserve chronological
emptying order. Payload identity and reuse order are runtime cache details;
candidate traversal and query hit selection retain explicit spatial/ID ordering.
Recorded body and collider schemas are unchanged. Cache-inclusive replay hashes
and hashes from different library versions should not be compared as
compatibility guarantees.

## Chronicler v1.0.0

When building Gravitas with Chronicler v1.0.0, import `Chronicler.Hashing` for
`ChronicleHash` returned by `GravitasWorldContext.ComputeReplayHash()` and for
`ChronicleHashWriter` extensions. Import `Chronicler.Serialization` for JSON and
MemoryPack serializers, converters, and payload editing. Recording contracts,
helpers, and `DefaultSaver` remain in `Chronicler`; timing remains in
`Chronicler.Timing`. Rebuild consuming assemblies together because the moved
types have new CLR identities.

Replace removed `SerializationPayloadEditor` convenience methods with typed
transport calls: `JsonRecordSerializer.Serialize` / `Populate`,
`MemoryPackRecordSerializer.Serialize` / `Populate`, and the editor's
`SetJsonValue`, `RemoveJsonProperty`, `SetMemoryPackValue`, or
`RemoveMemoryPackEntry`. JSON payloads are strings; MemoryPack payloads are byte
arrays and require the standard package. Pass `writeIndented: true` to JSON
serialization to preserve the old convenience method's formatting.

This namespace migration preserves Gravitas's recorded schemas, type names, and
replay hash stream. Existing payloads and unchanged replay vectors need no
conversion. The timing schema changes described below are a separate migration.
See the
[Chronicler migration guide](https://github.com/mrdav30/Chronicler/blob/main/docs/MIGRATION.md)
for the complete API mapping.

## Shared simulation timing

When upgrading from the narrow-clock API, update host code and stored data
together. The integration step remains Fixed64; the absolute timeline does not.

| Previous contract                                | Current contract                                                             |
| ------------------------------------------------ | ---------------------------------------------------------------------------- |
| `int FrameCount` and absolute frame/phase stamps | `long`; retain the full width in host logs, diagnostics and replay data      |
| `Fixed64 TotalTime`                              | `Chronicler.Timing.ChronicleTimestamp ElapsedTime`                           |
| `GetFrameFromTime(timestamp)`                    | `GetFrameCountForDuration(duration)`, a long count of complete current steps |

Subtract timestamps from the same simulation origin before converting a bounded
duration through `FixedMathSharp.Chronicler.FixedChronicleTime`. Do not cast
absolute time or frame counts back to narrow types. Ordinary integration still
uses `DeltaTime` and `InvDeltaTime`. Duration counts use exact raw-unit
division, rounded down, not reciprocal multiplication or a historical frame
lookup.

Rate changes affect future steps without recalculating elapsed history. Built-in
seconds waits preserve their elapsed-time deadline across a rate change; frame
waits count advances. Reset invalidates old waits even when the new run reaches
the same numeric frame. Recreate them for the new lifetime.

The 3D body requires `BodySchemaVersion = 1` and a long `LastGroundCheckFrame`.
Unversioned narrow-frame snapshots reject; recreate them using the new schema
rather than guessing an epoch. The 2D body rebuilds its grounding cache after
population and does not persist that frame stamp. Neither body restores the
containing world's clock. Coordinate runtime state and time origins through the
host lifecycle; standalone Chronicler clock recording is not a live-world rewind
API.

Replay hashes include the widened timing fields under the new domain schema. Old
and new hashes are not comparable; regenerate expected traces from verified
simulation inputs, not by replacing failing hash literals blindly.

See [Runtime Architecture](wiki/RUNTIME_ARCHITECTURE.md#clock-state),
[Host Integration](wiki/HOST_INTEGRATION.md), and
[Serialization And Replay](wiki/SERIALIZATION.md) for the current contracts.
