# Diagnostics

Gravitas diagnostics are context-owned, deterministic, and engine-agnostic.
They are meant to expose physics state for debug drawing, logs, replay tooling,
or server-side inspection without linking the core library to an engine, an editor,
or a renderer.

Diagnostics are disabled by default. When disabled, runtime hooks return before
touching diagnostic buffers. When enabled, events and draw commands are appended
to pre-sized `SwiftList` buffers owned by the active `GravitasWorldContext`.

## Entry Point

Use `context.Diagnostics`:

```csharp
context.Diagnostics.Enable(eventCapacity: 512, drawCommandCapacity: 256);
context.Simulate();
context.LateSimulate();

foreach (GravitasDiagnosticEvent diagnosticEvent in context.Diagnostics.Events)
{
    // Translate to logs, overlays, replay markers, or host telemetry.
}

foreach (GravitasDebugDrawCommand command in context.Diagnostics.DrawCommands)
{
    // Translate to host-specific lines, meshes, gizmos, or debug shapes.
}

context.Diagnostics.Clear();
```

`Enable(...)` reserves capacity so enabled diagnostics can run without resize
spikes when the expected event count is known. `Clear()` resets captured data
and per-frame sequence values while keeping the allocated buffers. `Disable()`
clears and stops capture.

## Event Stream

`GravitasDiagnosticEvent` is a compact generic payload. The common fields are:

| Field | Meaning |
| --- | --- |
| `Frame` | Owning context frame count when the event was captured. |
| `Sequence` | Capture order inside the current buffer. |
| `Kind` | Event payload type. |
| `BodyId` | Context-local dynamic body ID, or `-1` when not applicable. |
| `ColliderAId` / `ColliderBId` | Context-local collider IDs, or `-1` when not applicable. |
| `ColliderADimension` / `ColliderBDimension` | Collider runtime surface: `ThreeD`, `TwoD`, or `None`. |
| `ColliderAType` / `ColliderBType` | Collider shape types when present. |
| `ColliderA2DType` / `ColliderB2DType` | 2D collider shape types when present. |
| `Start` / `End` | Query segment, previous/current velocity, or other pair of vector values. |
| `PointA` / `PointB` | Contact points, hit point, acceleration delta, or shape-specific point data. |
| `Vector` | Force, torque, velocity delta, query normal, contact normal, or impulse direction. |
| `ScalarA` / `ScalarB` | Event-specific fixed-point scalar values. |
| `DataA` / `DataB` | Event-specific integer values, such as layer mask bits or hit count. |
| `Hit` | Whether the event represents a successful hit/contact. |

Current event kinds:

| Kind | Captured from | Payload notes |
| --- | --- | --- |
| `ForceDelta` | `StiffBody.AddForce(...)` | `Vector` is the force, `PointA` is acceleration delta, `ScalarA` is force magnitude. |
| `TorqueDelta` | `StiffBody.AddTorque(...)` | `Vector` is torque, `ScalarA` is torque magnitude. |
| `LinearVelocityDelta` | Collision response velocity change | `Start` and `End` are previous/current velocity, `Vector` is the delta. |
| `AngularVelocityDelta` | Collision response angular velocity change | Same shape as linear velocity delta. |
| `GroundProbe` | `StiffBody.CheckGround(...)` | `Start`/`End` are probe segment, `ScalarA` is probe radius, `DataA` is `GroundProbeMode`. |
| `RayQuery` | Raycast and swept-sphere queries | `ScalarA` is sweep radius, `DataA` is layer mask bits, `DataB` is hit count. |
| `CircleQuery` | Circle overlap queries | `Start` is center, `End` is directional extent when used, `ScalarA` is radius. |
| `QuerySummary` | Query reducer quality diagnostics | `DataA` is eligible top-level exact reducer attempts, `DataB` is accepted hits, `ScalarA` is fallback hits, and `ScalarB` is rejected conservative candidates. |
| `Contact` | `CollisionPair.ProcessCollision()` | Contact points, normal, and depth from narrow phase. |
| `ResponseImpulse` | `CollisionResponse` | `Vector` is normal impulse, `ScalarA` is impulse magnitude, `ScalarB` is normal velocity. |
| `MixedQuery` | `GravitasQueryMixedService` | Explicit mixed query segment, layer mask bits, hit count, mixed hit points, normal, and distance. |
| `MixedContact` | `CollisionPairMixed.MarkColliding(...)` | Dimension-tagged 3D/2D collider IDs, mixed contact points, `Normal3DTo2D`, and penetration depth. |
| `MixedResponseImpulse` | `CollisionResponseMixed` | Dimension-tagged mixed impulse, impulse magnitude, normal velocity, solve iteration, and iteration cap. |
| `MixedResponseIsland` | `GravitasMixedCollisionService` | Mixed island root key, constraint count, iterations used, and whether the configured cap was reached. |

The stream is scoped to one context. Collider and body IDs are not global and
must be resolved through the same context that produced the event.

## CCD Service Counters

Continuous-collision handoff diagnostics are exposed as service counters rather
than diagnostic events. `GravitasPhysicsService` reports
`LastContinuousCollisionIslandCount`,
`LastContinuousCollisionIslandIterationCount`, and
`LastContinuousCollisionIslandLimitReached`; `GravitasPhysics2DService` mirrors
the same counters internally for pure 2D tests and service-level validation.
The body-owned bounded solver still reports
`LastContinuousCollisionToiIterationCount` and
`LastContinuousCollisionToiIterationLimitReached` on `StiffBody` and
`StiffBody2D`. These counters are deterministic frame-local state intended for
tuning, tests, and host diagnostics without adding event-buffer traffic to the
hot path.

## Diagnostic Dispatch And Typed Views

`GravitasDiagnosticEvent` remains the compact capture format. Host adapters
should usually consume events through `GravitasDiagnosticEventVisitor` so they
do not need to know which `TryAs...` helper matches each
`GravitasDiagnosticEventKind`:

```csharp
public sealed class MyDiagnosticAdapter : GravitasDiagnosticEventVisitor
{
    public override void VisitMixedContact(in GravitasMixedContactDiagnosticView contact)
    {
        // contact.Collider3DId, contact.Collider2DId,
        // contact.Point3D, contact.Point2D, contact.Normal3DTo2D, contact.Depth
    }
}

context.Diagnostics.DispatchEventsTo(adapter);
```

Available views cover every current event kind:
`GravitasForceDeltaDiagnosticView`, `GravitasTorqueDeltaDiagnosticView`,
`GravitasVelocityDeltaDiagnosticView`, `GravitasGroundProbeDiagnosticView`,
`GravitasRayQueryDiagnosticView`, `GravitasCircleQueryDiagnosticView`,
`GravitasQuerySummaryDiagnosticView`, `GravitasContactDiagnosticView`,
`GravitasResponseImpulseDiagnosticView`, `GravitasMixedQueryDiagnosticView`,
`GravitasMixedContactDiagnosticView`,
`GravitasMixedResponseImpulseDiagnosticView`, and
`GravitasMixedResponseIslandDiagnosticView`.

`GravitasQuerySummaryDiagnosticView` is emitted by mixed query paths
when diagnostics are enabled. It reports eligible top-level exact reducer
candidate attempts, accepted hits, fallback hits, and rejected conservative
fallback candidates so hosts can inspect exact-versus-conservative query quality
beside the ordinary `MixedQuery` hit event. Exact attempts are counted after
eligibility filtering; private compound part attempts are folded into the owning
candidate and are not counted independently.

The views are read-only wrappers over the existing event value. Visitors and
views do not change capture storage, event ordering, diagnostic buffering, or
disabled-path cost. The lower-level `TryAs...` helpers remain available for
one-off filters over a known event kind, but visitors are the preferred adapter
shape.

## Draw Commands

`GravitasDebugDrawCommand` is the renderer-facing stream. Gravitas emits
primitive draw descriptions; hosts translate them into their own debug drawing
API.

Current draw kinds:

| Kind | Required payload |
| --- | --- |
| `Line` | `Start`, `End`, `Color` |
| `Ray` | `Start`, `End`, `Color` |
| `Point` | `Center`, `Radius`, `Color` |
| `WireSphere` | `Center`, `Radius`, `Color` |
| `WireBox` | `Center`, `Size`, `Rotation`, `Color` |
| `WireCapsule` | `Center`, `Radius`, `Height`, `Rotation`, `Color` |
| `WireCylinder` | `Center`, `Radius`, `Height`, `Rotation`, `Color` |
| `WireTriangle` | `PointA`, `PointB`, `PointC`, `Color` |

Host renderers can consume draw commands through
`GravitasDebugDrawCommandVisitor` and `context.Diagnostics.DispatchDrawCommandsTo(...)`.
That visitor exposes typed draw views for lines, rays, points, wire spheres,
boxes, capsules, cylinders, and triangles so adapters do not need to repeat a
manual command-kind switch.

Use the explicit capture helpers for host-driven overlays:

```csharp
context.Diagnostics.CaptureCollider(collider, GravitasDiagnosticColor.Cyan);
context.Diagnostics.CaptureMixedCollider(collider2D, GravitasDiagnosticColor.Cyan);
context.Diagnostics.CaptureLine(start, end, GravitasDiagnosticColor.Yellow);
context.Diagnostics.CaptureRay(origin, direction, maxDistance, GravitasDiagnosticColor.Green);
context.Diagnostics.CapturePoint(point, Fixed64.Half, GravitasDiagnosticColor.Red);
```

`CaptureCollider(...)` emits one command for primitive colliders and one
`WireTriangle` command per mesh triangle. Compound colliders emit one command
per internal part using the owning compound collider ID and
`ColliderType.Compound`, so host renderers can draw the approximation without
treating parts as registered colliders. Large meshes can therefore generate a
large command buffer; hosts should reserve capacity or choose filtered capture
when inspecting dense mesh scenes.

`CaptureMixedCollider(LSCollider2D, ...)` emits the finite 2D slab/prism used
by mixed collision rather than only the pure 2D outline. Circles draw as
vertical wire cylinders, AABBs draw as wire boxes, and polygons draw the top,
bottom, and vertical slab edges as line commands. These commands set
`ColliderDimension` to `TwoD` and populate `Collider2DType` so host adapters can
style embedded 2D debug geometry separately from pure 3D colliders.

## Host Adapter Pattern

A Host adapter might translate draw commands into `Debug.DrawLine`, `Gizmos`,
or `Handles`. A server adapter might ignore draw commands and only export the
event stream as structured logs. A replay/debugger adapter might store both
streams beside lockstep frame data.

Keep adapters outside `src/Gravitas`. Core runtime code should emit fixed-point
values, context-local IDs, stable ordering, and shape metadata only.

See [Diagnostic Adapters](DIAGNOSTIC_ADAPTERS.md) for renderer-neutral adapter
shapes for debug draw, server logs, and replay timeline capture.

## Performance Rules

- Leave diagnostics disabled in normal hot-path measurements unless the
  measurement is specifically about diagnostics.
- Call `Enable(...)` with realistic capacities before a capture-heavy run.
- Call `Clear()` once the host has consumed a frame or diagnostic window.
- Do not project diagnostics through `SwiftCollections.Observable` in
  authoritative simulation paths unless tests and benchmarks prove the ordering
  and notification cost are acceptable.
- Add benchmarks when new event hooks or draw commands touch collision,
  partitioning, queries, body integration, or response paths.

## Current Limits

- Diagnostics are same-thread context buffers, matching the current lockstep
  runtime model.
- Event storage is intentionally generic. When a subsystem needs richer
  diagnostics, add a documented event kind or typed view instead of overloading
  fields in a way hosts cannot decode.
- Draw commands are wire/debug descriptions, not mesh generation utilities.
  Hosts remain responsible for actual rendering.
