using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Collider2DShapeTests
{
    [Fact]
    public void CircleCollider2D_ShouldOwnPure2DBounds()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSCircleCollider2D(Fixed64.One);
        var transform = new FixedTransform(new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)3), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)2, (Fixed64)3));

        collider.Shape.Should().Be(ColliderType2D.Circle);
        collider.Bounds.Min.Should().Be(new Vector2d(Fixed64.One, (Fixed64)2));
        collider.Bounds.Max.Should().Be(new Vector2d((Fixed64)3, (Fixed64)4));
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 0)]
    public void AABBoxCollider2D_ShouldRejectNonPositiveSizeComponents(int x, int y)
    {
        Action create = () => _ = new LSAABBoxCollider2D(new Vector2d((Fixed64)x, (Fixed64)y));

        create.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Fact]
    public void CapsuleCollider2D_GetSupportPoint_ShouldUseDirectionAndSegmentSign()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector2d.Zero);

        collider.GetSupportPoint(Vector2d.Forward).Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        collider.GetSupportPoint(Vector2d.Forward * (Fixed64)2)
            .Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        collider.GetSupportPoint(-Vector2d.Forward).Should().Be(new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(3, 2)));
        collider.GetSupportPoint(Vector2d.Zero).Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void CapsuleCollider2D_WithOddRawAxisLength_ShouldRoundOnlyTheFinalSurfacePoint()
    {
        using GravitasWorldContext context = Create2DContext();
        Fixed64 radius = Fixed64.FromRaw(1);
        var collider = new LSCapsuleCollider2D(radius, Fixed64.FromRaw(3));
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero);

        collider.AxisLength.Should().Be(Fixed64.FromRaw(1));
        collider.GetSupportPoint(Vector2d.Forward)
            .Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromRaw(2)));
        collider.GetClosestPoint(Vector2d.Forward)
            .Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromRaw(2)));
    }

    [Fact]
    public void CapsuleCollider2D_ExactDimensionAdmission_ShouldRejectSaturatingDiameter()
    {
        Fixed64 oversizedRadius = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue / 2L) + 1L);

        Action create = () =>
            _ = new LSCapsuleCollider2D(
                oversizedRadius,
                Fixed64.MaxValue);

        create.Should()
            .Throw<ArgumentException>()
            .WithParameterName("height");
    }

    [Fact]
    public void CapsuleCollider2D_ExactDimensionAdmission_ShouldRejectInvalidScaledDiameter()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSCapsuleCollider2D(
            Fixed64.One,
            Fixed64.Two);
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Two,
                Fixed64.One,
                Fixed64.One));
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            collider)
        {
            Mass = Fixed64.One
        };

        Action initialize = () => body.Initialize(Vector2d.Zero);

        initialize.Should()
            .Throw<ArgumentException>()
            .WithParameterName("snapshot");
    }

    [Fact]
    public void CapsuleCollider2D_WithBothAxisEndpointsOutsideDomain_ShouldUseCenteredTieSupport()
    {
        using GravitasWorldContext context = Create2DContext();
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        Vector2d center = new(
            Fixed64.MaxValue - (Fixed64)5,
            Fixed64.MaxValue - (Fixed64)5);
        var collider = new LSCapsuleCollider2D(radius, (Fixed64)20 + radius * Fixed64.Two);
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(center, FixedMath.DegToRad((Fixed64)45));
        Vector2d normal = collider.WorldAxis.RightHandNormal;

        FixedSegment2d.TryGetCenteredAxisEndpoint(
                collider.Center,
                collider.WorldAxis,
                collider.AxisLength,
                positive: false,
                out _)
            .Should()
            .BeFalse();
        FixedSegment2d.TryGetCenteredAxisEndpoint(
                collider.Center,
                collider.WorldAxis,
                collider.AxisLength,
                positive: true,
                out _)
            .Should()
            .BeFalse();
        Fixed64.TryMultiplyAdd(normal.X, radius, center.X, out Fixed64 expectedX)
            .Should()
            .BeTrue();
        Fixed64.TryMultiplyAdd(normal.Y, radius, center.Y, out Fixed64 expectedY)
            .Should()
            .BeTrue();

        collider.GetSupportPoint(normal).Should().Be(new Vector2d(expectedX, expectedY));
    }

    [Fact]
    public void CapsuleCollider2D_WithSurfaceOutsideDomain_ShouldRejectTheWitness()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSCapsuleCollider2D(Fixed64.One, Fixed64.Two);
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = Fixed64.One
        };
        Vector2d center = new(Fixed64.MaxValue, Fixed64.Zero);
        body.Initialize(center);

        Action getSupport = () => collider.GetSupportPoint(Vector2d.Right);
        Action getClosest = () => collider.GetClosestPoint(center);

        getSupport.Should().Throw<InvalidOperationException>();
        getClosest.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void PolygonCollider2D_WithConcaveVertices_ShouldThrow()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)2, Fixed64.Zero),
            new(Fixed64.One, Fixed64.Half),
            new((Fixed64)2, (Fixed64)2),
            new(Fixed64.Zero, (Fixed64)2)
        };

        Action create = () => _ = new LSPolygonCollider2D(vertices);

        create.Should()
            .Throw<ArgumentException>()
            .WithMessage("*convex*");
    }

    [Fact]
    public void PolygonCollider2D_ShouldUpdateWorldVerticesFromBodyRotation()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        var transform = new FixedTransform(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)5), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)4, (Fixed64)5), Fixed64.Pi / (Fixed64)2);

        collider.GetWorldVertex(0).Should().Be(new Vector2d((Fixed64)5, (Fixed64)4));
        collider.GetWorldVertex(1).Should().Be(new Vector2d((Fixed64)5, (Fixed64)6));
        collider.GetWorldVertex(2).Should().Be(new Vector2d((Fixed64)3, (Fixed64)5));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PolygonCollider2D_WhenScaleCollapsesBoundary_ShouldRejectBeforeBinding(bool compoundPart)
    {
        using GravitasWorldContext context = Create2DContext();
        Vector2d[] vertices = { Vector2d.Left, Vector2d.Right, new(Fixed64.Zero, Fixed64.FromRaw(1)) };
        LSCollider2D collider = compoundPart
            ? new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * Fixed64.Two),
                CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero, Fixed64.Zero,
                    new Vector2d(Fixed64.One, Fixed64.Half)))
            : new LSPolygonCollider2D(vertices);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.One, compoundPart ? Fixed64.One : Fixed64.Half));

        Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithMessage("*convex*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void PolygonCollider2D_WhenRefreshCollapsesBoundary_ShouldRetainCommittedShapeAndRecover()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSPolygonCollider2D(
            Vector2d.Left, Vector2d.Right, new Vector2d(Fixed64.Zero, Fixed64.FromRaw(1)));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        int id = collider.Id;
        var bounds = collider.Bounds;
        Vector2d apex = collider.GetWorldVertex(2);
        int membershipCount = collider.PartitionCoordinates!.Count;
        transform.LocalScale = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Half);

        Action refresh = () => collider.Simulate();

        refresh.Should().Throw<ArgumentException>().WithMessage("*convex*");
        collider.Id.Should().Be(id);
        collider.Bounds.Should().Be(bounds);
        collider.GetWorldVertex(2).Should().Be(apex);
        collider.IsPartitioned.Should().BeTrue();
        collider.PartitionCoordinates!.Count.Should().Be(membershipCount);
        context.Physics2D.ColliderCount.Should().Be(1);
        transform.LocalScale = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Two);
        collider.Simulate();
        collider.GetWorldVertex(2).Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromRaw(2)));
        context.Query2D.Raycast(Vector2d.Zero, Vector2d.Right, out var hit).Should().BeTrue();
        hit.Collider.Should().BeSameAs(collider);
        hit.Normal.Should().Be(-Vector2d.Forward);
    }

    [Fact]
    public void PolygonCollider2D_WhenLoadedVertexCountChanges_ShouldValidateCollapsedReplacement()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(-1, -1), new Vector2d(1, -1), new Vector2d(1, 1), new Vector2d(-1, 1));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Half));
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        var bounds = collider.Bounds;
        Vector2d firstVertex = collider.GetWorldVertex(0);
        var payload = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Vertices"] = new Vector2d[]
            {
                new(Fixed64.FromRaw(-1), Fixed64.Zero),
                new(Fixed64.FromRaw(1), Fixed64.Zero),
                new(Fixed64.Zero, Fixed64.FromRaw(1))
            }
        });

        Action load = () => collider.RecordData(payload);

        load.Should().Throw<ArgumentException>().WithMessage("*convex*");
        collider.Count.Should().Be(4);
        collider.Bounds.Should().Be(bounds);
        collider.GetWorldVertex(0).Should().Be(firstVertex);
        context.Physics2D.ColliderCount.Should().Be(1);

        collider.RecordData(new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Vertices"] = new[] { Vector2d.Left, Vector2d.Right, Vector2d.Forward }
        }));
        collider.Simulate();
        collider.Count.Should().Be(3);
        collider.GetWorldVertex(2).Should().Be(Vector2d.Forward * Fixed64.Half);
    }

    private static GravitasWorldContext Create2DContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }
}
