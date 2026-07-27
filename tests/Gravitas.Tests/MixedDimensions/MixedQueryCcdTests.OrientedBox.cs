//=======================================================================
// MixedQueryCcdTests.OrientedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SweptCircleAgainstRotatedCuboid_AtScalarFace_ShouldMatchTranslatedDistance(
        bool positiveFace)
    {
        PhysicsMixedHit baseline = SweepCircleAgainstRotatedCuboid(
            cuboidCenterX: Fixed64.Zero,
            positiveFace,
            scalarFaceGrid: false);
        Fixed64 cuboidCenterX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        PhysicsMixedHit translated = SweepCircleAgainstRotatedCuboid(
            cuboidCenterX,
            positiveFace,
            scalarFaceGrid: true);

        translated.Distance.Should().Be(baseline.Distance);
        translated.Normal3DTo2D.Should().Be(baseline.Normal3DTo2D);
        translated.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweptCircleStartingInsideCuboid_ShouldUseNearestFaceAnchor()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateBody3D(
            context,
            new LSCuboidCollider(),
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.FromFraction(1, 4),
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Anchor3D.TryGetOffsetFrom(
                cuboid.Collider.Center,
                out Vector3d surfaceOffset)
            .Should().BeTrue();
        surfaceOffset.Should().Be(Vector3d.Right * Fixed64.Half);
        hit.Normal3DTo2D.Should().Be(Vector3d.Left);
    }

    private static PhysicsMixedHit SweepCircleAgainstRotatedCuboid(
        Fixed64 cuboidCenterX,
        bool positiveFace,
        bool scalarFaceGrid)
    {
        using GravitasWorldContext context = CreateOrientedBoxMixedContext(
            positiveFace,
            scalarFaceGrid);
        _ = CreateBody3D(
            context,
            new LSCuboidCollider(),
            new Vector3d(cuboidCenterX, Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)45,
                Fixed64.Zero));
        Fixed64 pathX = positiveFace
            ? cuboidCenterX + Fixed64.FromFraction(1, 4)
            : cuboidCenterX - Fixed64.FromFraction(1, 4);
        Vector2d start = new(pathX, (Fixed64)(-3));
        Vector2d end = new(pathX, Fixed64.Zero);

        context.QueryMixed.SweepCircleAgainst3D(
                start,
                end,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();
        return hit;
    }

    private static GravitasWorldContext CreateOrientedBoxMixedContext(
        bool positiveFace,
        bool scalarFaceGrid)
    {
        if (!scalarFaceGrid)
            return CreateMixedContext();

        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 minX = positiveFace
            ? Fixed64.MaxValue - (Fixed64)8
            : Fixed64.MinValue;
        Fixed64 maxX = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue + (Fixed64)8;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(minX, (Fixed64)(-4), (Fixed64)(-4)),
                new Vector3d(maxX, (Fixed64)4, (Fixed64)4)),
            out _).Should().BeTrue();
        return context;
    }
}
