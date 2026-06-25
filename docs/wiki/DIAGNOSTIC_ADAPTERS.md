# Diagnostic Adapters

Diagnostics in Gravitas are data streams, not renderer calls. Core runtime code
emits deterministic `GravitasDiagnosticEvent` values and
`GravitasDebugDrawCommand` values into `context.Diagnostics`; host adapters
translate those streams into engine-specific overlays, logs, captures, or replay
tools outside `src/Gravitas`.

Keep adapter code in the host, samples, or tooling projects. Do not add engine-specific,
renderer, file-system, networking, or editor dependencies to the Gravitas core
library to make diagnostics easier to display.

## Adapter Boundary

An adapter should:

- consume diagnostics after the deterministic frame or debug capture window has
  finished.
- preserve `Frame` and `Sequence` as the ordering keys.
- resolve context-local body and collider IDs only against the same
  `GravitasWorldContext` that produced the payload.
- translate fixed-point values at the edge, if the host renderer or log format
  requires floating-point or text output.
- call `context.Diagnostics.Clear()` after the frame is consumed when the host
  wants a per-frame stream.

An adapter should not:

- mutate `StiffBody`, `StiffBody2D`, collider, partition, or query state while
  consuming diagnostics.
- feed diagnostics back into authoritative simulation decisions.
- assume collider IDs are global, stable across contexts, or valid after the
  owning context is reset.
- project diagnostics through observable/event APIs in authoritative hot paths
  unless ordering and notification cost have dedicated tests and benchmarks.

## Debug Draw Adapter Shape

Renderer adapters usually need a small command sink that mirrors the host's
debug drawing API. The adapter owns any conversion to host units, colors, and
duration.

```csharp
public interface IHostDebugDrawSink
{
    void DrawLine(Vector3d start, Vector3d end, GravitasDiagnosticColor color);
    void DrawPoint(Vector3d center, Fixed64 radius, GravitasDiagnosticColor color);
    void DrawWireSphere(Vector3d center, Fixed64 radius, GravitasDiagnosticColor color);
    void DrawWireBox(Vector3d center, Vector3d size, FixedQuaternion rotation, GravitasDiagnosticColor color);
    void DrawWireCylinder(Vector3d center, Fixed64 radius, Fixed64 height, FixedQuaternion rotation, GravitasDiagnosticColor color);
    void DrawWireCapsule(Vector3d center, Fixed64 radius, Fixed64 height, FixedQuaternion rotation, GravitasDiagnosticColor color);
    void DrawWireTriangle(Vector3d a, Vector3d b, Vector3d c, GravitasDiagnosticColor color);
}
```

```csharp
public sealed class HostDebugDrawAdapter : GravitasDebugDrawCommandVisitor
{
    private readonly IHostDebugDrawSink _sink;

    public HostDebugDrawAdapter(IHostDebugDrawSink sink)
    {
        _sink = sink;
    }

    public override void VisitLine(in GravitasLineDebugDrawView view) =>
        _sink.DrawLine(view.Start, view.End, view.Color);

    public override void VisitRay(in GravitasRayDebugDrawView view) =>
        _sink.DrawLine(view.Start, view.End, view.Color);

    public override void VisitPoint(in GravitasPointDebugDrawView view) =>
        _sink.DrawPoint(view.Center, view.Radius, view.Color);

    public override void VisitWireSphere(in GravitasWireSphereDebugDrawView view) =>
        _sink.DrawWireSphere(view.Center, view.Radius, view.Color);

    public override void VisitWireBox(in GravitasWireBoxDebugDrawView view) =>
        _sink.DrawWireBox(view.Center, view.Size, view.Rotation, view.Color);

    public override void VisitWireCylinder(in GravitasWireCylinderDebugDrawView view) =>
        _sink.DrawWireCylinder(view.Center, view.Radius, view.Height, view.Rotation, view.Color);

    public override void VisitWireCapsule(in GravitasWireCapsuleDebugDrawView view) =>
        _sink.DrawWireCapsule(view.Center, view.Radius, view.Height, view.Rotation, view.Color);

    public override void VisitWireTriangle(in GravitasWireTriangleDebugDrawView view) =>
        _sink.DrawWireTriangle(view.PointA, view.PointB, view.PointC, view.Color);
}

public static void FlushDebugDraw(GravitasWorldContext context, HostDebugDrawAdapter adapter) =>
    context.Diagnostics.DispatchDrawCommandsTo(adapter);
```

2D debug draw in mixed mode is emitted as finite 3D slab geometry. Circles draw
as wire cylinders, axis-aligned boxes draw as wire boxes, and polygons draw
top, bottom, and vertical slab edges. Use `ColliderDimension` and
`Collider2DType` to style embedded 2D geometry differently from normal 3D
colliders.

Mesh and compound capture can emit many commands. Reserve draw-command capacity
before a capture-heavy run and avoid enabling full mesh capture every frame in
normal gameplay.

## Structured Log Adapter Shape

Server or headless hosts can ignore draw commands and write diagnostic events to
a deterministic log sink. Prefer `GravitasDiagnosticEventVisitor` so decoding
stays centralized in Gravitas instead of repeating generic-field mappings in
every adapter.

```csharp
public interface IHostDiagnosticLogSink
{
    void Write(int frame, int sequence, GravitasDiagnosticEventKind kind, string payload);
}
```

```csharp
public sealed class HostDiagnosticLogAdapter : GravitasDiagnosticEventVisitor
{
    private readonly IHostDiagnosticLogSink _sink;

    public HostDiagnosticLogAdapter(IHostDiagnosticLogSink sink)
    {
        _sink = sink;
    }

    public override void VisitForceDelta(in GravitasForceDeltaDiagnosticView view) =>
        _sink.Write(view.Frame, view.Sequence, view.Event.Kind, $"body={view.BodyId} force={view.Force} accelDelta={view.AccelerationDelta}");

    public override void VisitRayQuery(in GravitasRayQueryDiagnosticView view) =>
        _sink.Write(view.Frame, view.Sequence, view.Event.Kind, $"hit={view.Hit} collider={view.HitColliderId} distance={view.Distance}");

    public override void VisitContact(in GravitasContactDiagnosticView view) =>
        _sink.Write(view.Frame, view.Sequence, view.Event.Kind, $"a={view.ColliderAId} b={view.ColliderBId} depth={view.Depth}");

    public override void VisitMixedContact(in GravitasMixedContactDiagnosticView view) =>
        _sink.Write(view.Frame, view.Sequence, view.Event.Kind, $"3d={view.Collider3DId} 2d={view.Collider2DId} depth={view.Depth}");
}

public static void FlushEvents(GravitasWorldContext context, HostDiagnosticLogAdapter adapter) =>
    context.Diagnostics.DispatchEventsTo(adapter);
```

For production logs, prefer a structured payload object over ad hoc strings.
The important rule is that adapters should consume semantic typed views instead
of decoding `ScalarA`, `ScalarB`, `DataA`, and `DataB` directly.

## Replay Timeline Adapter Shape

Replay tooling should keep diagnostics beside the authoritative replay frame,
not inside the authoritative snapshot. A simple frame capture can store events
and draw commands in order:

```csharp
public readonly struct DiagnosticFrameCapture
{
    public DiagnosticFrameCapture(
        int frame,
        GravitasDiagnosticEvent[] events,
        GravitasDebugDrawCommand[] drawCommands)
    {
        Frame = frame;
        Events = events;
        DrawCommands = drawCommands;
    }

    public int Frame { get; }

    public GravitasDiagnosticEvent[] Events { get; }

    public GravitasDebugDrawCommand[] DrawCommands { get; }
}
```

```csharp
public static DiagnosticFrameCapture CaptureTimelineFrame(GravitasWorldContext context)
{
    ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
    ReadOnlySpan<GravitasDebugDrawCommand> drawCommands = context.Diagnostics.DrawCommands;

    return new DiagnosticFrameCapture(
        context.FrameCount,
        events.ToArray(),
        drawCommands.ToArray());
}
```

The array allocation above belongs to the tooling edge. Do not move that pattern
into runtime hooks. Long-running replay tools should pool or stream their own
capture storage if diagnostics are enabled for many frames.

## Generic Payload Policy

The current event struct intentionally uses generic numeric fields:

- `ScalarA` and `ScalarB` for fixed-point payload values.
- `DataA` and `DataB` for integer payload values.

That shape keeps the core event stream compact and predictable. The generic
payload table in [`DIAGNOSTICS.md`](DIAGNOSTICS.md) remains the storage
contract, while `GravitasDiagnosticEventVisitor` and typed event views are the
preferred adapter-facing decode surface. The lower-level `TryAs...` helpers are
useful for one-off filters over known event kinds, not for full adapter
dispatch.

Draw commands follow the same pattern: `GravitasDebugDrawCommand` stays compact,
while `GravitasDebugDrawCommandVisitor` exposes typed draw views for renderer
adapters. Do not overload the same event or draw kind with new meanings that
are not documented and tested.
