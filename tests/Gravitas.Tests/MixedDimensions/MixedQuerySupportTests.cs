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
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedQuerySupportTests
{
    [Fact]
    public void TryClipSegment_WhenStartIsInside_ShouldReturnZeroEntryAndBoundaryExit()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 entry,
            out Fixed64 exit);

        hit.Should().BeTrue();
        entry.Should().Be(Fixed64.Zero);
        exit.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryClipSegment_WhenParallelAxesAreInside_ShouldClipMovingAxis()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance,
            out _);

        hit.Should().BeTrue();
        distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void TryClipSegment_WhenParallelAxisStartsOutside_ShouldReject()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TryClipSegment_WhenAxisIntervalsDoNotOverlap_ShouldReject()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.One, -Fixed64.FromFraction(1, 10), Fixed64.Zero),
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TryClipSegment_WhenMovingFromPositiveSide_ShouldSwapAxisEntryAndExit()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance,
            out _);

        hit.Should().BeTrue();
        distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void TryClipSegment_WhenMovingAwayFromFirstAxis_ShouldReject()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TryClipSegment_WhenParallelAxisStartsBelowMinimum_ShouldReject()
    {
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)(-2), Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)6,
            -Vector3d.One,
            Vector3d.One,
            out _,
            out _);

        hit.Should().BeFalse();
    }

    [Fact]
    public void TryClipSegment_WhenLaterAxisEntersEarlier_ShouldKeepLatestEntry()
    {
        Fixed64 directionX = Fixed64.FromFraction(3, 5);
        bool hit = SweepBoundsUtility.TryClipSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(directionX, Fixed64.FromFraction(4, 5), Fixed64.Zero),
            (Fixed64)10,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance,
            out _);

        hit.Should().BeTrue();
        distance.Should().Be((Fixed64)2 / directionX);
    }

    [Fact]
    public void TryClipSegment_WhenSmallestDirectionComponentReachesBoundaryAtEndpoint_ShouldHit()
    {
        Fixed64 smallestIncrement = Fixed64.FromRaw(1);
        Vector3d start = new(-Fixed64.One - smallestIncrement, Fixed64.Zero, -Fixed64.Half);
        Vector3d end = new(-Fixed64.One, Fixed64.Zero, Fixed64.Half);
        Vector3d segment = end - start;
        Fixed64 length = segment.Magnitude;

        bool hit = SweepBoundsUtility.TryClipSegment(
            start,
            segment / length,
            length,
            -Vector3d.One,
            Vector3d.One,
            out Fixed64 distance,
            out _);

        hit.Should().BeTrue();
        distance.Should().Be(length);
    }

    [Fact]
    public void TryClipSegment_WhenAxisIntervalsDifferByOneRawUnit_ShouldUseSharedBoundary()
    {
        Fixed64 oneRawBelowOne = Fixed64.One - Fixed64.Epsilon;

        bool hit = SweepBoundsUtility.TryClipSegment(
            Vector3d.Zero,
            new Vector3d(1, 1, 0),
            Fixed64.Two,
            new Vector3d(Fixed64.One, oneRawBelowOne, -Fixed64.One),
            new Vector3d(Fixed64.One, oneRawBelowOne, Fixed64.One),
            out Fixed64 entry,
            out Fixed64 exit);

        hit.Should().BeTrue();
        entry.Should().Be(exit);
        entry.Should().Be(FixedMath.Midpoint(Fixed64.One, oneRawBelowOne));
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
    public void ClassifySweepSphereAgainst2D_WhenShapeIsUnsupported_ShouldFailExplicitly()
    {
        Action classify = () => MixedQueryReducerClassifier
            .ClassifySweepSphereAgainst2D(new UnsupportedTestCollider2D());

        classify.Should().Throw<InvalidOperationException>()
            .WithMessage("*not supported*");
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
