using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Diagnostics;

public sealed class GravitasDebugDrawCommandViewTests
{
    [Fact]
    public void DispatchTo_ShouldRouteEveryDrawKindToTypedVisitorMethod()
    {
        var visitor = new RecordingDebugDrawVisitor();
        Vector3d start = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d end = new((Fixed64)2, Fixed64.Zero, Fixed64.Zero);
        Vector3d center = new((Fixed64)3, Fixed64.Zero, Fixed64.Zero);
        Vector3d size = new((Fixed64)4, (Fixed64)5, (Fixed64)6);
        Vector3d pointA = Vector3d.Right;
        Vector3d pointB = Vector3d.Up;
        Vector3d pointC = Vector3d.Forward;

        CreateCommand(GravitasDebugDrawKind.Line, start: start, end: end).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Ray, start: start, end: end).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Point, center: center, radius: Fixed64.Half).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireSphere, center: center, radius: Fixed64.One).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireBox, center: center, size: size, rotation: FixedQuaternion.Identity).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCapsule, center: center, radius: Fixed64.Half, height: (Fixed64)2).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCylinder, center: center, radius: Fixed64.Half, height: (Fixed64)3).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireTriangle, pointA: pointA, pointB: pointB, pointC: pointC).DispatchTo(visitor);
        CreateCommand((GravitasDebugDrawKind)250).DispatchTo(visitor);

        visitor.Route.Should().Equal(
            nameof(RecordingDebugDrawVisitor.VisitLine),
            nameof(RecordingDebugDrawVisitor.VisitRay),
            nameof(RecordingDebugDrawVisitor.VisitPoint),
            nameof(RecordingDebugDrawVisitor.VisitWireSphere),
            nameof(RecordingDebugDrawVisitor.VisitWireBox),
            nameof(RecordingDebugDrawVisitor.VisitWireCapsule),
            nameof(RecordingDebugDrawVisitor.VisitWireCylinder),
            nameof(RecordingDebugDrawVisitor.VisitWireTriangle),
            nameof(RecordingDebugDrawVisitor.VisitUnknown));
        visitor.LastLine.Start.Should().Be(start);
        visitor.LastLine.End.Should().Be(end);
        visitor.LastWireBox.Size.Should().Be(size);
        visitor.LastWireTriangle.PointA.Should().Be(pointA);
        visitor.LastWireTriangle.PointB.Should().Be(pointB);
        visitor.LastWireTriangle.PointC.Should().Be(pointC);
        visitor.LastUnknown.Kind.Should().Be((GravitasDebugDrawKind)250);
    }

    [Fact]
    public void DispatchDrawCommandsTo_ShouldVisitCapturedDrawCommandsInBufferOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var visitor = new RecordingDebugDrawVisitor();

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 4);
        scenario.Context.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Yellow);
        scenario.Context.Diagnostics.CapturePoint(Vector3d.Up, Fixed64.Half, GravitasDiagnosticColor.Red);

        scenario.Context.Diagnostics.DispatchDrawCommandsTo(visitor);

        visitor.Route.Should().Equal(
            nameof(RecordingDebugDrawVisitor.VisitLine),
            nameof(RecordingDebugDrawVisitor.VisitPoint));
        visitor.LastLine.Sequence.Should().Be(0);
        visitor.LastPoint.Sequence.Should().Be(1);
        visitor.LastPoint.Center.Should().Be(Vector3d.Up);
    }

    private static GravitasDebugDrawCommand CreateCommand(
        GravitasDebugDrawKind kind,
        int colliderId = 17,
        GravitasColliderDimension colliderDimension = GravitasColliderDimension.ThreeD,
        ColliderType colliderType = ColliderType.Sphere,
        ColliderType2D collider2DType = ColliderType2D.None,
        Vector3d start = default,
        Vector3d end = default,
        Vector3d center = default,
        Vector3d size = default,
        Vector3d pointA = default,
        Vector3d pointB = default,
        Vector3d pointC = default,
        FixedQuaternion rotation = default,
        Fixed64 radius = default,
        Fixed64 height = default,
        GravitasDiagnosticColor color = default)
    {
        return new GravitasDebugDrawCommand(
            frame: 5,
            sequence: 9,
            kind: kind,
            colliderId: colliderId,
            colliderDimension: colliderDimension,
            colliderType: colliderType,
            collider2DType: collider2DType,
            start: start,
            end: end,
            center: center,
            size: size,
            pointA: pointA,
            pointB: pointB,
            pointC: pointC,
            rotation: rotation,
            radius: radius,
            height: height,
            color: color);
    }

    private sealed class RecordingDebugDrawVisitor : GravitasDebugDrawCommandVisitor
    {
        public readonly List<string> Route = new();
        public GravitasLineDebugDrawView LastLine;
        public GravitasPointDebugDrawView LastPoint;
        public GravitasWireBoxDebugDrawView LastWireBox;
        public GravitasWireTriangleDebugDrawView LastWireTriangle;
        public GravitasDebugDrawCommand LastUnknown;

        public override void VisitLine(in GravitasLineDebugDrawView view)
        {
            Route.Add(nameof(VisitLine));
            LastLine = view;
        }

        public override void VisitRay(in GravitasRayDebugDrawView view) =>
            Route.Add(nameof(VisitRay));

        public override void VisitPoint(in GravitasPointDebugDrawView view)
        {
            Route.Add(nameof(VisitPoint));
            LastPoint = view;
        }

        public override void VisitWireSphere(in GravitasWireSphereDebugDrawView view) =>
            Route.Add(nameof(VisitWireSphere));

        public override void VisitWireBox(in GravitasWireBoxDebugDrawView view)
        {
            Route.Add(nameof(VisitWireBox));
            LastWireBox = view;
        }

        public override void VisitWireCapsule(in GravitasWireCapsuleDebugDrawView view) =>
            Route.Add(nameof(VisitWireCapsule));

        public override void VisitWireCylinder(in GravitasWireCylinderDebugDrawView view) =>
            Route.Add(nameof(VisitWireCylinder));

        public override void VisitWireTriangle(in GravitasWireTriangleDebugDrawView view)
        {
            Route.Add(nameof(VisitWireTriangle));
            LastWireTriangle = view;
        }

        public override void VisitUnknown(in GravitasDebugDrawCommand command)
        {
            Route.Add(nameof(VisitUnknown));
            LastUnknown = command;
        }
    }
}
