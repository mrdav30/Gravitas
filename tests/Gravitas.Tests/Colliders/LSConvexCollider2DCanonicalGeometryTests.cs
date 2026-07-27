//=======================================================================
// LSConvexCollider2DCanonicalGeometryTests.cs
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

public sealed class LSConvexCollider2DCanonicalGeometryTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void AxisAlignedBoxAtScalarFace_ShouldHashCanonicalOffsets(
        bool positiveFace)
    {
        using GravitasWorldContext first = CreateScalarFaceContext(positiveFace);
        using GravitasWorldContext second = CreateScalarFaceContext(positiveFace);
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        var firstCollider = new LSAABBoxCollider2D(
            new Vector2d(Fixed64.One, Fixed64.One));
        var secondCollider = new LSAABBoxCollider2D(
            new Vector2d(Fixed64.One, Fixed64.One));
        Initialize(first, firstCollider, centerX);
        Initialize(second, secondCollider, centerX);

        firstCollider.GetScaledLocalVertexUnchecked(0)
            .Should().Be(new Vector2d(-Fixed64.Half, -Fixed64.Half));
        first.ComputeReplayHash().Should().Be(second.ComputeReplayHash());
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void AxisAlignedBoxAtScalarFace_ShouldNotFabricateClippedSupport(
        bool positiveFace)
    {
        using GravitasWorldContext context = CreateScalarFaceContext(positiveFace);
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        var collider = new LSAABBoxCollider2D(Vector2d.One);
        Initialize(context, collider, centerX);
        Vector2d inward = positiveFace ? Vector2d.Left : Vector2d.Right;
        Vector2d outward = -inward;

        collider.GetSupportPoint(inward).Should().Be(new Vector2d(
            centerX + inward.X * Fixed64.Half,
            Fixed64.Half));
        FluentActions.Invoking(() => collider.GetSupportPoint(outward))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CircleAtScalarFace_ShouldNotFabricateClippedSupport(
        bool positiveFace)
    {
        using GravitasWorldContext context = CreateScalarFaceContext(positiveFace);
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        Initialize(context, collider, centerX);
        Vector2d outward = positiveFace ? Vector2d.Right : Vector2d.Left;

        collider.ContainsPoint(collider.Center).Should().BeTrue();
        FluentActions.Invoking(() => collider.GetSupportPoint(outward))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
    }

    private static void Initialize(
        GravitasWorldContext context,
        LSCollider2D collider,
        Fixed64 centerX)
    {
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(
            new Vector2d(centerX, Fixed64.Zero),
            motionType: BodyMotionType.Static);
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
