//=======================================================================
// LSPolygonCollider2DCanonicalGeometryTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSPolygonCollider2DCanonicalGeometryTests
{
    [Fact]
    public void Construction_ShouldValidateFullDomainConvexityExactly()
    {
        Action convex = () => _ = new LSPolygonCollider2D(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        Action concave = () => _ = new LSPolygonCollider2D(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        Action collinear = () => _ = new LSPolygonCollider2D(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));

        convex.Should().NotThrow();
        concave.Should().Throw<ArgumentException>()
            .WithParameterName("vertices");
        collinear.Should().Throw<ArgumentException>()
            .WithParameterName("vertices");
    }

    [Fact]
    public void RotatedLocalOffsetOutsideScalarDomain_ShouldRemainCanonicalWhenWorldPointIsRepresentable()
    {
        Fixed64 inset = (Fixed64)2;
        Vector2d[] localVertices =
        {
            new(Fixed64.MaxValue - inset, Fixed64.MaxValue - inset),
            new(Fixed64.MaxValue, Fixed64.MaxValue - inset),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MaxValue - inset, Fixed64.MaxValue),
        };
        Fixed64 rotation = Fixed64.PiOver4;
        Vector2d center = new(Fixed64.Zero, Fixed64.MinValue);
        Vector2d.TryTransformPoint(
            Vector2d.Zero,
            localVertices[2],
            rotation,
            out _).Should().BeFalse();
        var expected = new Vector2d[localVertices.Length];
        for (int i = 0; i < localVertices.Length; i++)
        {
            Vector2d.TryTransformPoint(
                center,
                localVertices[i],
                rotation,
                out expected[i]).Should().BeTrue();
        }

        Fixed64 minX = expected[0].X;
        Fixed64 maxX = expected[0].X;
        Fixed64 minY = expected[0].Y;
        Fixed64 maxY = expected[0].Y;
        for (int i = 1; i < expected.Length; i++)
        {
            minX = FixedMath.Min(minX, expected[i].X);
            maxX = FixedMath.Max(maxX, expected[i].X);
            minY = FixedMath.Min(minY, expected[i].Y);
            maxY = FixedMath.Max(maxY, expected[i].Y);
        }

        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(minX - Fixed64.One, -Fixed64.One, minY - Fixed64.One),
                new Vector3d(maxX + Fixed64.One, Fixed64.One, maxY + Fixed64.One)),
            out _).Should().BeTrue();
        var collider = new LSPolygonCollider2D(localVertices);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);

        body.Initialize(center, rotation: rotation);

        collider.ScaledLocalVertices.ToArray().Should().Equal(localVertices);
        for (int i = 0; i < expected.Length; i++)
        {
            collider.TryGetWorldVertex(i, out Vector2d actual).Should().BeTrue();
            actual.Should().Be(expected[i]);
            collider.Bounds.Contains(actual).Should().BeTrue();
        }
        collider.ContainsPoint(expected[0]).Should().BeTrue();
    }

    [Fact]
    public void GroundProbe_WhenALaterRotatedVertexExceedsTheLocalDomain_ShouldRemainConservative()
    {
        Vector2d[] localVertices =
        {
            Vector2d.Zero,
            new(Fixed64.MaxValue, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.Zero, Fixed64.MaxValue)
        };
        var collider = new LSPolygonCollider2D(localVertices);
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var body = new SolidBody2D(new TestMatterAgent(context), collider);

        body.Initialize(
            new Vector2d(Fixed64.Zero, Fixed64.MinValue),
            rotation: Fixed64.PiOver4,
            motionType: BodyMotionType.Static);

        collider.CanonicalGroundProbeRadius.Should().Be(Fixed64.MaxValue);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ScalarFace_ShouldPublishCanonicalOffsetsWithoutFabricatingWorldVertices(
        bool positiveFace)
    {
        Vector2d[] localVertices =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half)
        };
        var collider = new LSPolygonCollider2D(localVertices);
        using GravitasWorldContext context = CreateScalarFaceContext(positiveFace);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);

        body.Initialize(new Vector2d(centerX, Fixed64.Zero));

        collider.ScaledLocalVertices.ToArray().Should().Equal(localVertices);
        collider.Bounds.Min.X.Should().Be(
            positiveFace ? centerX - Fixed64.Half : Fixed64.MinValue);
        collider.Bounds.Max.X.Should().Be(
            positiveFace ? Fixed64.MaxValue : centerX + Fixed64.Half);
        int inwardIndex = positiveFace ? 0 : 1;
        int outwardIndex = positiveFace ? 1 : 0;
        collider.TryGetWorldVertex(inwardIndex, out Vector2d inward).Should().BeTrue();
        inward.Should().Be(new Vector2d(
            centerX + localVertices[inwardIndex].X,
            localVertices[inwardIndex].Y));
        collider.TryGetWorldVertex(outwardIndex, out _).Should().BeFalse();
        FluentActions.Invoking(() => collider.GetWorldVertex(outwardIndex))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*TryGetWorldVertex*");
        collider.ContainsPoint(collider.Center).Should().BeTrue();
        collider.GetClosestPoint(collider.Center).Should().Be(collider.Center);
        Vector2d inwardDirection = positiveFace ? Vector2d.Left : Vector2d.Right;
        Vector2d outwardDirection = -inwardDirection;
        collider.GetSupportPoint(inwardDirection).Should().Be(new Vector2d(
            centerX + localVertices[inwardIndex].X,
            localVertices[inwardIndex].Y));
        FluentActions.Invoking(() => collider.GetSupportPoint(outwardDirection))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");

        using GravitasWorldContext equivalentContext = CreateScalarFaceContext(positiveFace);
        var equivalentCollider = new LSPolygonCollider2D(localVertices);
        var equivalentBody = new SolidBody2D(
            new TestMatterAgent(equivalentContext),
            equivalentCollider);
        equivalentBody.Initialize(new Vector2d(centerX, Fixed64.Zero));

        context.ComputeReplayHash().Should().Be(equivalentContext.ComputeReplayHash());
    }

    [Fact]
    public void TryGetWorldVertex_WithInvalidIndex_ShouldRejectTheIndex()
    {
        var collider = new LSPolygonCollider2D(
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward);

        FluentActions.Invoking(() => collider.TryGetWorldVertex(3, out _))
            .Should().Throw<IndexOutOfRangeException>();
    }

    private static GravitasWorldContext CreateScalarFaceContext(bool positiveFace)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 minX = positiveFace
            ? Fixed64.MaxValue - (Fixed64)4
            : Fixed64.MinValue;
        Fixed64 maxX = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue + (Fixed64)4;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(minX, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(maxX, (Fixed64)2, (Fixed64)2)),
            out _).Should().BeTrue();
        return context;
    }
}
