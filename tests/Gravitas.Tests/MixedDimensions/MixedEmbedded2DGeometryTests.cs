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
    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void GetSupportAnchor_ShouldKeepPrimitiveSupportRelativeToEmbeddedCenter(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D collider = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            ColliderType2D.AABox => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            _ => CreateSquare()
        };
        SolidBody2D body = CreateBody(
            context,
            collider,
            new Vector2d((Fixed64)3, (Fixed64)(-2)),
            hostY: (Fixed64)4);

        ContactAnchor anchor = MixedEmbedded2DGeometry.GetSupportAnchor(
            body.Collider,
            new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One));

        anchor.Origin.Should().Be(new Vector3d((Fixed64)3, (Fixed64)4, (Fixed64)(-2)));
        anchor.Offset.Y.Should().Be(context.Settings.Mixed2DHalfThickness);
        anchor.TryGetWorldPoint(out Vector3d worldPoint).Should().BeTrue();
        worldPoint.Y.Should().Be(body.Collider.MixedBounds3D.Max.Y);
    }

    [Fact]
    public void GetSupportAnchor_WithVerticalAndPlanarTies_ShouldUseStableRepresentatives()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D circle = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        SolidBody2D polygon = CreateBody(
            context,
            CreateSquare(),
            new Vector2d((Fixed64)3, Fixed64.Zero));

        ContactAnchor lowerCircle = MixedEmbedded2DGeometry.GetSupportAnchor(
            circle.Collider,
            -Vector3d.Up);
        ContactAnchor tiedPolygon = MixedEmbedded2DGeometry.GetSupportAnchor(
            polygon.Collider,
            Vector3d.Zero);

        lowerCircle.Offset.Should().Be(new Vector3d(
            Fixed64.Zero,
            -context.Settings.Mixed2DHalfThickness,
            Fixed64.Zero));
        tiedPolygon.Offset.Y.Should().Be(Fixed64.Zero);
        tiedPolygon.Offset.X.Should().Be(Fixed64.One);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    public void GetSupportAnchor_WithSmallRepresentableDirection_ShouldHonorItsSign(
        ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D collider = shape == ColliderType2D.Circle
            ? new LSCircleCollider2D(Fixed64.Half)
            : new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        SolidBody2D body = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            rotation: Fixed64.PiOver4);

        ContactAnchor tinyDirection = MixedEmbedded2DGeometry.GetSupportAnchor(
            body.Collider,
            new Vector3d(Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero));
        ContactAnchor unitDirection = MixedEmbedded2DGeometry.GetSupportAnchor(
            body.Collider,
            Vector3d.Right);

        tinyDirection.TryGetWorldPoint(out Vector3d tinyPoint).Should().BeTrue();
        unitDirection.TryGetWorldPoint(out Vector3d unitPoint).Should().BeTrue();
        tinyPoint.Should().Be(unitPoint);
    }

    [Fact]
    public void GetSupportAnchor_WithCompoundInput_ShouldRejectMissingPartContext()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));

        Action getAnchor = () => MixedEmbedded2DGeometry.GetSupportAnchor(
            compound,
            Vector3d.Right);

        getAnchor.Should().Throw<InvalidOperationException>()
            .WithMessage("*no representable owner-relative support anchor*");
    }

    [Fact]
    public void GetClosestAnchorOnEmbeddedVolume_WithUnsupportedContainingShape_ShouldKeepLowerSlabFaceOnTie()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 hostY = (Fixed64)4;
        SolidBody2D body = CreateBody(
            context,
            new UnsupportedTestCollider2D(containsPoints: true),
            new Vector2d((Fixed64)2, (Fixed64)(-3)),
            hostY);
        Vector3d point = new(body.Position.X, hostY, body.Position.Y);

        ContactAnchor anchor =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                body.Collider,
                point);

        anchor.TryGetWorldPoint(out Vector3d closest).Should().BeTrue();
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
    public void PlanarBoundaryAnchor_WithRotatedPolygonAtScalarFace_ShouldRemainSemantic()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)8,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        var polygon = CreateSquare();
        polygon.MixedHalfThicknessOverride = Fixed64.Two;
        Vector2d center = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        SolidBody2D body = CreateBody(
            context,
            polygon,
            center,
            rotation: Fixed64.PiOver4);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryAnchor(
            body.Collider,
            center,
            out ContactAnchor2D boundary,
            out Fixed64 distance);
        ContactAnchor closest =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                body.Collider,
                new Vector3d(center.X, Fixed64.Zero, center.Y));

        found.Should().BeTrue();
        distance.Should().Be(Fixed64.One);
        boundary.TryGetWorldPoint(out _).Should().BeFalse();
        closest.TryGetWorldPoint(out _).Should().BeFalse();
        closest.TryGetOffsetFrom(
            new Vector3d(center.X, Fixed64.Zero, center.Y),
            out Vector3d difference).Should().BeTrue();
        new Vector2d(difference.X, difference.Z).Magnitude
            .Should().Be(Fixed64.One);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    public void PlanarBoundaryAnchor_WithRoundShapeAtScalarFace_ShouldRemainSemantic(
        ColliderType2D shape)
    {
        using GravitasWorldContext context = CreatePositiveXScalarFaceContext();
        LSCollider2D collider = shape == ColliderType2D.Circle
            ? new LSCircleCollider2D(Fixed64.One)
            : new LSCapsuleCollider2D(Fixed64.One, (Fixed64)3);
        Vector2d center = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        SolidBody2D body = CreateBody(context, collider, center);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryAnchor(
            body.Collider,
            center,
            out ContactAnchor2D boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().Be(Fixed64.One);
        boundary.TryGetWorldPoint(out _).Should().BeFalse();
        boundary.TryGetOffsetFrom(center, out Vector2d offset).Should().BeTrue();
        offset.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void PlanarBoundaryAnchor_WithAabbAtScalarFace_ShouldRemainSemantic()
    {
        using GravitasWorldContext context = CreatePositiveXScalarFaceContext();
        Vector2d center = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        SolidBody2D body = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.Two)),
            center);
        Vector2d queryPoint = new(Fixed64.MaxValue, Fixed64.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryAnchor(
            body.Collider,
            queryPoint,
            out ContactAnchor2D boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().Be(Fixed64.FromFraction(3, 4));
        boundary.TryGetWorldPoint(out _).Should().BeFalse();
        boundary.LocalPoint.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void ClosestEmbeddedAnchor_WithSlabAtScalarFace_ShouldRemainSemantic()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    (Fixed64)(-4),
                    Fixed64.MaxValue - (Fixed64)8,
                    (Fixed64)(-4)),
                new Vector3d(
                    (Fixed64)4,
                    Fixed64.MaxValue,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        Fixed64 slabCenterY =
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        SolidBody2D body = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            slabCenterY);
        Vector3d queryPoint = new(
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.Zero);

        ContactAnchor closest =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                body.Collider,
                queryPoint);

        closest.TryGetWorldPoint(out _).Should().BeFalse();
        closest.TryGetOffsetFrom(queryPoint, out Vector3d offset).Should().BeTrue();
        offset.Should().Be(
            Vector3d.Up * Fixed64.FromFraction(3, 4));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ClosestEmbeddedAnchor_WithUnrepresentableCenterDifference_ShouldSelectNearestSlabFace(
        bool positiveSlab)
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 slabCenterY = positiveSlab
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 queryY =
            positiveSlab ? Fixed64.MinValue : Fixed64.MaxValue;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    (Fixed64)(-4),
                    positiveSlab
                        ? Fixed64.MaxValue - (Fixed64)8
                        : Fixed64.MinValue,
                    (Fixed64)(-4)),
                new Vector3d(
                    (Fixed64)4,
                    positiveSlab
                        ? Fixed64.MaxValue
                        : Fixed64.MinValue + (Fixed64)8,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        SolidBody2D body = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            slabCenterY);

        ContactAnchor closest =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                body.Collider,
                new Vector3d(Fixed64.Zero, queryY, Fixed64.Zero));

        closest.Origin.Y.Should().Be(slabCenterY);
        closest.Offset.Should().Be(
            (positiveSlab ? -Vector3d.Up : Vector3d.Up));
        closest.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
        point.Y.Should().Be(
            slabCenterY
            + (positiveSlab ? -Fixed64.One : Fixed64.One));
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

    private static GravitasWorldContext CreatePositiveXScalarFaceContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)8,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        return context;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 hostY = default,
        Fixed64 rotation = default)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(position.X, hostY, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(
            position,
            rotation,
            BodyMotionType.Static);
        return body;
    }

    private static LSPolygonCollider2D CreateSquare() =>
        new(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
}
