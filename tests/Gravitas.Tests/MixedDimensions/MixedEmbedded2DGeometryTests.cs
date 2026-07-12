using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedEmbedded2DGeometryTests
{
    [Fact]
    public void GetClosestPointOnEmbeddedVolume_WithUnsupportedContainingShape_ShouldKeepLowerSlabFaceOnTie()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 hostY = (Fixed64)4;
        SolidBody2D body = CreateBody(
            context,
            new UnsupportedTestCollider2D(containsPoints: true),
            new Vector2d((Fixed64)2, (Fixed64)(-3)),
            hostY);
        Vector3d point = new(body.Position.X, hostY, body.Position.Y);

        Vector3d closest = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(body.Collider, point);

        closest.Should().Be(new Vector3d(
            point.X,
            hostY - context.Settings.Mixed2DHalfThickness,
            point.Z));
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    public void TryGetPlanarBoundaryPoint_WithCoincidentRoundShapePoint_ShouldUseRightFallback(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D collider = shape == ColliderType2D.Circle
            ? new LSCircleCollider2D(Fixed64.Half)
            : new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        SolidBody2D body = CreateBody(context, collider, new Vector2d((Fixed64)2, (Fixed64)(-3)));

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            body.Position,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(body.Position + Vector2d.Right * Fixed64.Half);
        distance.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithAabbCenter_ShouldKeepMinXFaceOnTie()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            new Vector2d((Fixed64)3, (Fixed64)4));

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            body.Position,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d((Fixed64)2, (Fixed64)4));
        distance.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithPolygonCenter_ShouldKeepFirstAuthoredEdgeOnTie()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body = CreateBody(context, CreateSquare(), new Vector2d((Fixed64)3, (Fixed64)4));

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            body.Position,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d((Fixed64)3, (Fixed64)3));
        distance.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithPointOnPolygonBoundary_ShouldReportExactZeroDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body = CreateBody(context, CreateSquare(), new Vector2d((Fixed64)3, (Fixed64)4));
        var point = new Vector2d((Fixed64)4, (Fixed64)4);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            point,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(point);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithEqualDistanceCompoundParts_ShouldKeepFirstAuthoredPart()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-2), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));
        SolidBody2D body = CreateBody(context, compound, Vector2d.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            Vector2d.Zero,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d(-Fixed64.FromFraction(3, 2), Fixed64.Zero));
        distance.Should().Be(Fixed64.FromFraction(3, 2));
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithMaximumDistanceCompoundPart_ShouldAdmitFirstCandidate()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.MaxValue, Vector2d.Zero));

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            compound,
            Vector2d.Zero,
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        distance.Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void ConvexAuthoring_ShouldRejectZeroVertices()
    {
        Action act = () => _ = new LSPolygonCollider2D(Array.Empty<Vector2d>());

        act.Should().Throw<ArgumentException>().WithParameterName("vertices");
    }

    [Fact]
    public void CompoundAuthoring_ShouldRejectZeroParts()
    {
        Action act = () => _ = new LSCompoundCollider2D(Array.Empty<CompoundColliderPart2D>());

        act.Should().Throw<ArgumentException>().WithParameterName("parts");
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-8), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)8, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 hostY = default)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(position.X, hostY, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }

    private static LSPolygonCollider2D CreateSquare() =>
        new(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
}
