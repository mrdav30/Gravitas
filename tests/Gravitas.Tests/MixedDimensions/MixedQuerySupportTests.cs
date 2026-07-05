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
using SwiftCollections;
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
            .ClassifySweepSphereAgainst2D(new UnsupportedCollider2D())
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
            .ClassifySweepCircleAgainst3D(new UnsupportedCollider3D())
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

    private sealed class UnsupportedCollider2D : LSCollider2D
    {
        public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

        public override bool ContainsPoint(Vector2d point) => false;

        public override Vector2d GetClosestPoint(Vector2d point) => Center;

        public override Vector2d GetSupportPoint(Vector2d direction) => Center;

        internal override int VertexCount => 0;

        internal override Vector2d GetVertexUnchecked(int index) => Center;

        public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint) => Fixed64.Zero;

        internal override Fixed64 CalculateAreaForMassProperties() => Fixed64.Zero;

        protected override void RebuildShape() =>
            SetBoundsFromMinMax(Center - Vector2d.One, Center + Vector2d.One);
    }

    private sealed class UnsupportedCollider3D : LSCollider
    {
        public override ColliderType Shape => (ColliderType)byte.MaxValue;

        public override int Priority => 0;

        protected override void BuildShape()
        {
            Area = Fixed64.One;
            SetBoundsMinMax(Center - Vector3d.One, Center + Vector3d.One);
        }

        public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset) =>
            Fixed3x3.Zero;

        public override Vector3d ClosestPointOnSurface(Vector3d other) => Center;

        public override Vector3d GetNormalAtPoint(Vector3d point) => Vector3d.Up;

        public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
            false;
    }
}
