//=======================================================================
// MixedQuerySupportTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedQuerySupportTests
{
    [Fact]
    public void TrySweepBox_WhenStartIsInside_ShouldReturnZeroDistance()
    {
        bool hit = MixedSweepBoxUtility.TrySweepBox(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepBox_WhenParallelAxesAreInside_ShouldClipMovingAxis()
    {
        bool hit = MixedSweepBoxUtility.TrySweepBox(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void TrySweepBox_WhenParallelAxisStartsOutside_ShouldReject()
    {
        bool hit = MixedSweepBoxUtility.TrySweepBox(
            new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TrySweepBox_WhenAxisIntervalsDoNotOverlap_ShouldReject()
    {
        bool hit = MixedSweepBoxUtility.TrySweepBox(
            new Vector3d((Fixed64)(-3), (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.One, -Fixed64.FromFraction(1, 10), Fixed64.Zero),
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TrySweepBox_WhenMovingFromPositiveSide_ShouldSwapAxisEntryAndExit()
    {
        bool hit = MixedSweepBoxUtility.TrySweepBox(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ClassifySweepSphereAgainst2D_ShouldMarkSupportedShapesExact()
    {
        LSCollider2D[] colliders =
        {
            new LSCircleCollider2D(Fixed64.Half),
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2),
            new LSAABBoxCollider2D(Vector2d.One),
            new LSPolygonCollider2D(CreateTriangle2D()),
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Right))
        };

        for (int i = 0; i < colliders.Length; i++)
        {
            MixedQueryReducerClassifier
                .ClassifySweepSphereAgainst2D(colliders[i])
                .Should()
                .Be(PhysicsQueryReducerKind.Exact);
        }
    }

    [Fact]
    public void ClassifySweepSphereAgainst2D_WhenShapeIsUnsupported_ShouldMarkConservativeFallback()
    {
        MixedQueryReducerClassifier
            .ClassifySweepSphereAgainst2D(new UnsupportedTestCollider2D())
            .Should()
            .Be(PhysicsQueryReducerKind.ConservativeFallback);
    }

    [Fact]
    public void ClassifySweepCircleAgainst3D_ShouldMarkSupportedShapesExact()
    {
        LSCollider[] colliders =
        {
            new LSSphereCollider(),
            new LSCuboidCollider(),
            new LSCapsuleCollider(),
            new LSCylinderCollider(),
            new LSConeCollider(),
            MeshTestFixtures.CreateConvexCube(),
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
                CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Right))
        };

        for (int i = 0; i < colliders.Length; i++)
        {
            MixedQueryReducerClassifier
                .ClassifySweepCircleAgainst3D(colliders[i])
                .Should()
                .Be(PhysicsQueryReducerKind.Exact);
        }
    }

    [Fact]
    public void ClassifySweepCircleAgainst3D_WhenShapeIsUnsupported_ShouldMarkConservativeFallback()
    {
        MixedQueryReducerClassifier
            .ClassifySweepCircleAgainst3D(new UnsupportedTestCollider3D())
            .Should()
            .Be(PhysicsQueryReducerKind.ConservativeFallback);
    }

    private static Vector2d[] CreateTriangle2D() =>
        new[]
        {
            Vector2d.Forward,
            Vector2d.Right,
            Vector2d.Left
        };
}
