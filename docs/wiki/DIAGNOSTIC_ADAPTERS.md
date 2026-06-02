# Diagnostic Adapters

Diagnostics in Gravitas are data streams, not renderer calls. Core runtime code
emits deterministic `GravitasDiagnosticEvent` values and
`GravitasDebugDrawCommand` values into `context.Diagnostics`; host adapters
translate those streams into engine-specific overlays, logs, captures, or replay
tools outside `src/Gravitas`.

Keep adapter code in the host, samples, or tooling projects. Do not add Unity,
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
public static void FlushDebugDraw(GravitasWorldContext context, IHostDebugDrawSink sink)
{
    foreach (GravitasDebugDrawCommand command in context.Diagnostics.DrawCommands)
    {
        switch (command.Kind)
        {
            case GravitasDebugDrawKind.Line:
            case GravitasDebugDrawKind.Ray:
                sink.DrawLine(command.Start, command.End, command.Color);
                break;
            case GravitasDebugDrawKind.Point:
                sink.DrawPoint(command.Center, command.Radius, command.Color);
                break;
            case GravitasDebugDrawKind.WireSphere:
                sink.DrawWireSphere(command.Center, command.Radius, command.Color);
                break;
            case GravitasDebugDrawKind.WireBox:
                sink.DrawWireBox(command.Center, command.Size, command.Rotation, command.Color);
                break;
            case GravitasDebugDrawKind.WireCylinder:
                sink.DrawWireCylinder(command.Center, command.Radius, command.Height, command.Rotation, command.Color);
                break;
            case GravitasDebugDrawKind.WireCapsule:
                sink.DrawWireCapsule(command.Center, command.Radius, command.Height, command.Rotation, command.Color);
                break;
            case GravitasDebugDrawKind.WireTriangle:
                sink.DrawWireTriangle(command.PointA, command.PointB, command.PointC, command.Color);
                break;
        }
    }
}
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
a deterministic log sink. Keep event decoding table-driven by
`GravitasDiagnosticEventKind` so generic fields do not become ambiguous.

```csharp
public interface IHostDiagnosticLogSink
{
    void Write(int frame, int sequence, GravitasDiagnosticEventKind kind, string payload);
}
```

```csharp
public static void FlushEvents(GravitasWorldContext context, IHostDiagnosticLogSink sink)
{
    foreach (GravitasDiagnosticEvent diagnosticEvent in context.Diagnostics.Events)
    {
        string payload = diagnosticEvent.Kind switch
        {
            GravitasDiagnosticEventKind.ForceDelta =>
                $"body={diagnosticEvent.BodyId} force={diagnosticEvent.Vector} accelDelta={diagnosticEvent.PointA}",
            GravitasDiagnosticEventKind.RayQuery =>
                $"hit={diagnosticEvent.Hit} collider={diagnosticEvent.ColliderAId} distance={diagnosticEvent.ScalarB}",
            GravitasDiagnosticEventKind.Contact =>
                $"a={diagnosticEvent.ColliderAId} b={diagnosticEvent.ColliderBId} depth={diagnosticEvent.ScalarA}",
            GravitasDiagnosticEventKind.MixedContact =>
                $"3d={diagnosticEvent.ColliderAId} 2d={diagnosticEvent.ColliderBId} depth={diagnosticEvent.ScalarA}",
            _ => $"hit={diagnosticEvent.Hit} a={diagnosticEvent.ColliderAId} b={diagnosticEvent.ColliderBId}"
        };

        sink.Write(diagnosticEvent.Frame, diagnosticEvent.Sequence, diagnosticEvent.Kind, payload);
    }
}
```

For production logs, prefer a structured payload object over ad hoc strings.
The important rule is that every adapter branch must treat the event kind as
the decoder for `ScalarA`, `ScalarB`, `DataA`, and `DataB`.

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

That shape keeps the core event stream compact and predictable. It is
sufficient for alpha as long as each event kind has documented field meanings in
[`DIAGNOSTICS.md`](DIAGNOSTICS.md) and adapters decode by kind.

If host adapters start repeating error-prone switch logic, add typed adapter
helpers or typed view structs outside the capture hot path. Do not overload the
same event kind with new meanings that are not documented and tested.
