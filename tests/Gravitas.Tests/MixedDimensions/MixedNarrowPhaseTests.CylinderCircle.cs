//=======================================================================
// MixedNarrowPhaseTests.CylinderCircle.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void CylinderCircleSlab_WithSkewRimSeparation_ShouldRejectOverlappingBounds(int sign)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinderCircleRegressionBody(
            context, Fixed64.One,
            new Vector3d(Fixed64.FromFraction(7 * sign, 4),
                Fixed64.FromFraction(7 * sign, 4), Fixed64.FromFraction(11 * sign, 8)));
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.One },
            Vector2d.Zero);

        // Each constrained rim can reach at most sqrt(7)/4 along Z;
        // their combined reach is strictly less than the 11/8 separation.
        cylinder.Collider.Bounds.Intersects(circle.Collider.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider,
            out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(-1L << 30)]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    [InlineData(1L << 30)]
    public void CylinderCircleSlab_AroundExactSkewRimTangency_ShouldClassifySignedGap(long offsetRaw)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinderCircleRegressionBody(
            context, (Fixed64)5,
            new Vector3d((Fixed64)4, (Fixed64)4, (Fixed64)8 + Fixed64.FromRaw(offsetRaw)));
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D((Fixed64)5) { MixedHalfThicknessOverride = Fixed64.One },
            Vector2d.Zero);

        // At offset zero the rim offsets are (0,3,4) and (3,0,4),
        // both radius five, with common supporting normal (3,3,4).
        cylinder.Collider.Bounds.Intersects(circle.Collider.MixedBounds3D).Should().BeTrue();
        bool collided = CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider,
            out MixedContact contact);
        collided.Should().Be(offsetRaw <= 0);
        contact.HasContact.Should().Be(offsetRaw <= 0);
        if (offsetRaw > 0)
            return;

        if (offsetRaw == 0)
            contact.Depth.Should().Be(Fixed64.Zero);
        else
            contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Normal3DTo2D.X.Should().BeLessThan(Fixed64.Zero);
        contact.Normal3DTo2D.Y.Should().BeLessThan(Fixed64.Zero);
        contact.Normal3DTo2D.Z.Should().BeLessThan(Fixed64.Zero);
        contact.Anchor3D.Origin.Should().Be(cylinder.Collider.Center);
        contact.Anchor3D.Rotation.Should().Be(cylinder.Collider.Rotation);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        contact.DepthIsClamped.Should().BeFalse();
        contact.HasMaterialOverride.Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void CylinderCircleSlab_WithParallelCapContact_ShouldKeepPairDirectionAndDepth(int overlap)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateBody3D(context,
            new LSCylinderCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) },
            new Vector3d(Fixed64.Zero, (Fixed64)2 - Fixed64.FromFraction(overlap, 4), Fixed64.Zero),
            FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.One },
            Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider,
            out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(overlap, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Up);
        contact.Point3D.Y.Should().Be(Fixed64.One - Fixed64.FromFraction(overlap, 4));
        contact.Point2D.Y.Should().Be(Fixed64.One);
        contact.HasMaterialOverride.Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderCircleSlab_WithUnrepresentableFullSlabHeight_ShouldPreserveAdmittedHalfThickness(bool perpendicular)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = perpendicular
            ? CreateCylinderCircleRegressionBody(context, Fixed64.One, Vector3d.Zero)
            : CreateBody3D(context,
                new LSCylinderCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) },
                Vector3d.Zero, FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.MaxValue },
            Vector2d.Zero);
        Fixed64 centerY = Fixed64.MaxValue - (perpendicular ? Fixed64.Half : Fixed64.One);
        cylinder.Collider.LocalOffset = perpendicular
            ? new Vector3d(-centerY, Fixed64.Zero, Fixed64.Zero)
            : new Vector3d(Fixed64.Zero, centerY, Fixed64.Zero);
        cylinder.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        cylinder.Collider.Center.Y.Should().Be(centerY);

        CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider,
            out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(perpendicular ? Fixed64.FromFraction(3, 2) : (Fixed64)2);
        if (perpendicular)
            contact.Normal3DTo2D.Should().Be(-Vector3d.Up);
        contact.DepthIsClamped.Should().BeFalse();
        contact.Anchor3D.Origin.Should().Be(cylinder.Collider.Center);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CylinderCircleSlab_AtScalarFace_ShouldKeepCanonicalAnchors()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);
        cylinder.Collider.LocalOffset = new Vector3d(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero);
        circle.Collider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        cylinder.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        circle.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider,
            out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        contact.Normal3DTo2D.Should().Be(Vector3d.Right);
        contact.Anchor3D.Origin.Should().Be(cylinder.Collider.Center);
        contact.TryGetPoint3D(out _).Should().BeFalse();
        contact.Anchor2D.Origin.X.Should().Be(Fixed64.MaxValue);
        contact.DepthIsClamped.Should().BeFalse();
    }

    [Fact]
    public void CylinderCircleSlab_AfterWarmup_ShouldNotAllocate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);
        bool allContacts = true;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
            allContacts &= CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider, out _));

        allContacts.Should().BeTrue();
        allocated.Should().Be(0);
    }

    private static ScenarioBody<LSCylinderCollider> CreateCylinderCircleRegressionBody(
        GravitasWorldContext context, Fixed64 radius, Vector3d center)
    {
        Fixed64 q = Fixed64.FromRaw(3037000500L);
        return CreateBody3D(context,
            new LSCylinderCollider { Radius = radius, Size = new Vector3d(2, 2, 2) },
            center, new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q));
    }
}
