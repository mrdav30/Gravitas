using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
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
        Vector3d halfExtents = new((Fixed64)4, (Fixed64)5, (Fixed64)6);
        Vector3d pointA = Vector3d.Right;
        Vector3d pointB = Vector3d.Up;
        Vector3d pointC = Vector3d.Forward;
        GravitasDiagnosticColor color = new(1, 2, 3, 4);

        CreateCommand(GravitasDebugDrawKind.Line, start: start, end: end, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Ray, start: start, end: end, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Point, center: center, radius: Fixed64.Half, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireSphere, center: center, radius: Fixed64.One, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireBox, center: center, halfExtents: halfExtents, rotation: FixedQuaternion.Identity, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCapsule, center: center, rotation: FixedQuaternion.Identity, radius: Fixed64.Half, axisLength: (Fixed64)2, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCylinder, center: center, rotation: FixedQuaternion.Identity, radius: Fixed64.Half, height: (Fixed64)3, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireTriangle, pointA: pointA, pointB: pointB, pointC: pointC, color: color).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCone, center: center, rotation: FixedQuaternion.Identity, radius: Fixed64.One, height: (Fixed64)4, color: color).DispatchTo(visitor);
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
            nameof(RecordingDebugDrawVisitor.VisitWireCone),
            nameof(RecordingDebugDrawVisitor.VisitUnknown));
        AssertExpectedMetadata(
            visitor.LastLine.Frame,
            visitor.LastLine.Sequence,
            visitor.LastLine.ColliderId,
            visitor.LastLine.ColliderDimension,
            visitor.LastLine.ColliderType,
            visitor.LastLine.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastRay.Frame,
            visitor.LastRay.Sequence,
            visitor.LastRay.ColliderId,
            visitor.LastRay.ColliderDimension,
            visitor.LastRay.ColliderType,
            visitor.LastRay.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastPoint.Frame,
            visitor.LastPoint.Sequence,
            visitor.LastPoint.ColliderId,
            visitor.LastPoint.ColliderDimension,
            visitor.LastPoint.ColliderType,
            visitor.LastPoint.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireSphere.Frame,
            visitor.LastWireSphere.Sequence,
            visitor.LastWireSphere.ColliderId,
            visitor.LastWireSphere.ColliderDimension,
            visitor.LastWireSphere.ColliderType,
            visitor.LastWireSphere.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireBox.Frame,
            visitor.LastWireBox.Sequence,
            visitor.LastWireBox.ColliderId,
            visitor.LastWireBox.ColliderDimension,
            visitor.LastWireBox.ColliderType,
            visitor.LastWireBox.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireCapsule.Frame,
            visitor.LastWireCapsule.Sequence,
            visitor.LastWireCapsule.ColliderId,
            visitor.LastWireCapsule.ColliderDimension,
            visitor.LastWireCapsule.ColliderType,
            visitor.LastWireCapsule.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireCylinder.Frame,
            visitor.LastWireCylinder.Sequence,
            visitor.LastWireCylinder.ColliderId,
            visitor.LastWireCylinder.ColliderDimension,
            visitor.LastWireCylinder.ColliderType,
            visitor.LastWireCylinder.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireTriangle.Frame,
            visitor.LastWireTriangle.Sequence,
            visitor.LastWireTriangle.ColliderId,
            visitor.LastWireTriangle.ColliderDimension,
            visitor.LastWireTriangle.ColliderType,
            visitor.LastWireTriangle.Collider2DType);
        AssertExpectedMetadata(
            visitor.LastWireCone.Frame,
            visitor.LastWireCone.Sequence,
            visitor.LastWireCone.ColliderId,
            visitor.LastWireCone.ColliderDimension,
            visitor.LastWireCone.ColliderType,
            visitor.LastWireCone.Collider2DType);
        visitor.LastLine.Start.Should().Be(start);
        visitor.LastLine.End.Should().Be(end);
        visitor.LastLine.Color.Should().Be(color);
        visitor.LastRay.Start.Should().Be(start);
        visitor.LastRay.End.Should().Be(end);
        visitor.LastRay.Color.Should().Be(color);
        visitor.LastPoint.Center.Should().Be(center);
        visitor.LastPoint.Radius.Should().Be(Fixed64.Half);
        visitor.LastPoint.Color.Should().Be(color);
        visitor.LastWireSphere.Center.Should().Be(center);
        visitor.LastWireSphere.Radius.Should().Be(Fixed64.One);
        visitor.LastWireSphere.Color.Should().Be(color);
        visitor.LastWireBox.Center.Should().Be(center);
        visitor.LastWireBox.HalfExtents.Should().Be(halfExtents);
        visitor.LastWireBox.Rotation.Should().Be(FixedQuaternion.Identity);
        visitor.LastWireBox.Color.Should().Be(color);
        visitor.LastWireCapsule.Center.Should().Be(center);
        visitor.LastWireCapsule.Rotation.Should().Be(FixedQuaternion.Identity);
        visitor.LastWireCapsule.Radius.Should().Be(Fixed64.Half);
        visitor.LastWireCapsule.AxisLength.Should().Be((Fixed64)2);
        visitor.LastWireCapsule.Color.Should().Be(color);
        visitor.LastWireCylinder.Center.Should().Be(center);
        visitor.LastWireCylinder.Rotation.Should().Be(FixedQuaternion.Identity);
        visitor.LastWireCylinder.Radius.Should().Be(Fixed64.Half);
        visitor.LastWireCylinder.Height.Should().Be((Fixed64)3);
        visitor.LastWireCylinder.Color.Should().Be(color);
        visitor.LastWireTriangle.PointA.Should().Be(pointA);
        visitor.LastWireTriangle.PointB.Should().Be(pointB);
        visitor.LastWireTriangle.PointC.Should().Be(pointC);
        visitor.LastWireTriangle.Color.Should().Be(color);
        visitor.LastWireCone.Center.Should().Be(center);
        visitor.LastWireCone.Rotation.Should().Be(FixedQuaternion.Identity);
        visitor.LastWireCone.Radius.Should().Be(Fixed64.One);
        visitor.LastWireCone.Height.Should().Be((Fixed64)4);
        visitor.LastWireCone.Color.Should().Be(color);
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

    [Fact]
    public void DispatchTo_ShouldRejectNullDebugDrawVisitors()
    {
        GravitasDebugDrawCommand command = CreateCommand(GravitasDebugDrawKind.Line);

        Action dispatchCommand = () => command.DispatchTo(null!);

        dispatchCommand.Should().Throw<ArgumentNullException>().WithParameterName("visitor");
    }

    [Fact]
    public void DispatchDrawCommandsTo_ShouldRejectNullVisitor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        Action dispatch = () => scenario.Context.Diagnostics.DispatchDrawCommandsTo(null!);

        dispatch.Should().Throw<ArgumentNullException>().WithParameterName("visitor");
    }

    [Fact]
    public void PartialDebugDrawVisitor_ShouldIgnoreUnimplementedKindsAndPreserveOverriddenRoute()
    {
        var visitor = new PartialDebugDrawVisitor();

        CreateCommand(GravitasDebugDrawKind.Line, start: Vector3d.Right).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Ray).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.Point).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireSphere).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireBox).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCapsule).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCylinder).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireTriangle).DispatchTo(visitor);
        CreateCommand(GravitasDebugDrawKind.WireCone).DispatchTo(visitor);
        CreateCommand((GravitasDebugDrawKind)250).DispatchTo(visitor);

        visitor.LineVisitCount.Should().Be(1);
        visitor.LastLine.Start.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void DiagnosticColor_ShouldPackChannelsAndExposeNamedColors()
    {
        var color = new GravitasDiagnosticColor(1, 2, 3, 4);

        color.R.Should().Be(1);
        color.G.Should().Be(2);
        color.B.Should().Be(3);
        color.A.Should().Be(4);
        color.Rgba.Should().Be(0x01020304);
        GravitasDiagnosticColor.White.Rgba.Should().Be(0xFFFFFFFF);
        GravitasDiagnosticColor.Red.Rgba.Should().Be(0xFF0000FF);
        GravitasDiagnosticColor.Green.Rgba.Should().Be(0x00FF00FF);
        GravitasDiagnosticColor.Blue.Rgba.Should().Be(0x0000FFFF);
        GravitasDiagnosticColor.Yellow.Rgba.Should().Be(0xFFFF00FF);
        GravitasDiagnosticColor.Cyan.Rgba.Should().Be(0x00FFFFFF);
    }

    private static GravitasDebugDrawCommand CreateCommand(
        GravitasDebugDrawKind kind,
        int colliderId = 17,
        GravitasColliderDimension colliderDimension = GravitasColliderDimension.ThreeD,
        ColliderType colliderType = ColliderType.Sphere,
        ColliderType2D collider2DType = ColliderType2D.Circle,
        Vector3d start = default,
        Vector3d end = default,
        Vector3d center = default,
        Vector3d halfExtents = default,
        Vector3d pointA = default,
        Vector3d pointB = default,
        Vector3d pointC = default,
        FixedQuaternion rotation = default,
        Fixed64 radius = default,
        Fixed64 axisLength = default,
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
            halfExtents: halfExtents,
            pointA: pointA,
            pointB: pointB,
            pointC: pointC,
            rotation: rotation,
            radius: radius,
            axisLength: axisLength,
            height: height,
            color: color);
    }

    private static void AssertExpectedMetadata(
        int frame,
        int sequence,
        int colliderId,
        GravitasColliderDimension colliderDimension,
        ColliderType colliderType,
        ColliderType2D collider2DType)
    {
        frame.Should().Be(5);
        sequence.Should().Be(9);
        colliderId.Should().Be(17);
        colliderDimension.Should().Be(GravitasColliderDimension.ThreeD);
        colliderType.Should().Be(ColliderType.Sphere);
        collider2DType.Should().Be(ColliderType2D.Circle);
    }

    private sealed class RecordingDebugDrawVisitor : GravitasDebugDrawCommandVisitor
    {
        public readonly List<string> Route = new();
        public GravitasLineDebugDrawView LastLine;
        public GravitasRayDebugDrawView LastRay;
        public GravitasPointDebugDrawView LastPoint;
        public GravitasWireSphereDebugDrawView LastWireSphere;
        public GravitasWireBoxDebugDrawView LastWireBox;
        public GravitasWireCapsuleDebugDrawView LastWireCapsule;
        public GravitasWireCylinderDebugDrawView LastWireCylinder;
        public GravitasWireTriangleDebugDrawView LastWireTriangle;
        public GravitasWireConeDebugDrawView LastWireCone;
        public GravitasDebugDrawCommand LastUnknown;

        public override void VisitLine(in GravitasLineDebugDrawView view)
        {
            Route.Add(nameof(VisitLine));
            LastLine = view;
        }

        public override void VisitRay(in GravitasRayDebugDrawView view)
        {
            Route.Add(nameof(VisitRay));
            LastRay = view;
        }

        public override void VisitPoint(in GravitasPointDebugDrawView view)
        {
            Route.Add(nameof(VisitPoint));
            LastPoint = view;
        }

        public override void VisitWireSphere(in GravitasWireSphereDebugDrawView view)
        {
            Route.Add(nameof(VisitWireSphere));
            LastWireSphere = view;
        }

        public override void VisitWireBox(in GravitasWireBoxDebugDrawView view)
        {
            Route.Add(nameof(VisitWireBox));
            LastWireBox = view;
        }

        public override void VisitWireCapsule(in GravitasWireCapsuleDebugDrawView view)
        {
            Route.Add(nameof(VisitWireCapsule));
            LastWireCapsule = view;
        }

        public override void VisitWireCylinder(in GravitasWireCylinderDebugDrawView view)
        {
            Route.Add(nameof(VisitWireCylinder));
            LastWireCylinder = view;
        }

        public override void VisitWireTriangle(in GravitasWireTriangleDebugDrawView view)
        {
            Route.Add(nameof(VisitWireTriangle));
            LastWireTriangle = view;
        }

        public override void VisitWireCone(in GravitasWireConeDebugDrawView view)
        {
            Route.Add(nameof(VisitWireCone));
            LastWireCone = view;
        }

        public override void VisitUnknown(in GravitasDebugDrawCommand command)
        {
            Route.Add(nameof(VisitUnknown));
            LastUnknown = command;
        }
    }

    private sealed class PartialDebugDrawVisitor : GravitasDebugDrawCommandVisitor
    {
        public int LineVisitCount;
        public GravitasLineDebugDrawView LastLine;

        public override void VisitLine(in GravitasLineDebugDrawView view)
        {
            base.VisitLine(view);
            LineVisitCount++;
            LastLine = view;
        }
    }
}
