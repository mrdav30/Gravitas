//=======================================================================
// MixedNarrowPhaseTests.RelativeAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(ColliderType.Capsule)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    public void FiniteAxisCapsuleSlab_With3DAxisSeparation_ShouldRejectContact(
        ColliderType shape3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape3D,
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero),
            FixedQuaternion.Identity);
        LSCollider2D capsule2D = CreatePrimitive2D(
            context,
            ColliderType2D.Capsule,
            Fixed64.Zero);

        CollisionDetectionMixed.TryCollide(
                collider3D,
                capsule2D,
                out MixedContact contact)
            .Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(ColliderType.Capsule, ColliderType2D.Circle)]
    [InlineData(ColliderType.Cylinder, ColliderType2D.Capsule)]
    [InlineData(ColliderType.Cone, ColliderType2D.Capsule)]
    public void FiniteAxisSlab_WithCrossAxisSeparation_ShouldRejectContact(
        ColliderType shape3D,
        ColliderType2D shape2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape3D,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4),
            Euler(0, 0, 90));
        LSCollider2D collider2D = CreatePrimitive2D(
            context,
            shape2D,
            Fixed64.Zero);

        CollisionDetectionMixed.TryCollide(
                collider3D,
                collider2D,
                out MixedContact contact)
            .Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CapsuleCircleSlab_AtPositiveScalarFace_ShouldKeepRelativeContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsule3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        Fixed64 capsuleCenter = Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        capsule.Collider.LocalOffset = new Vector3d(capsuleCenter, Fixed64.Zero, Fixed64.Zero);
        circle.Collider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        capsule.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        circle.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(
                capsule.Collider,
                circle.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.HasContact.Should().BeTrue();
        contact.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        contact.Anchor3D.Offset.X.Should().BeGreaterThan(Fixed64.Zero);
        contact.TryGetPoint3D(out _).Should().BeFalse();
        contact.Anchor2D.Origin.Should().Be(new Vector3d(
            circle.Collider.Center.X,
            Fixed64.Zero,
            circle.Collider.Center.Y));
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Fact]
    public void RotatedCapsuleCircleSlab_AtPositiveScalarFace_ShouldKeepCanonicalRigidFrame()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = new(
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        ScenarioBody<LSCapsuleCollider> capsule =
            CreateCapsule3D(context, Vector3d.Zero, rotation);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        Fixed64 capsuleCenter =
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        capsule.Collider.LocalOffset = new Vector3d(
            -capsuleCenter,
            Fixed64.Zero,
            Fixed64.Zero);
        circle.Collider.LocalOffset = new Vector2d(
            Fixed64.MaxValue,
            Fixed64.Zero);
        capsule.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        circle.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        capsule.Collider.WorldAxis.Should().Be(Vector3d.Up);

        CollisionDetectionMixed.TryCollide(
                capsule.Collider,
                circle.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        contact.Anchor3D.Rotation.Should().Be(capsule.Collider.Rotation);
        contact.Anchor3D.TryGetWorldPoint(out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(ColliderType.Capsule)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    public void FiniteAxisCircleSlab_NonIdentityFrame_ShouldKeepCanonicalRigidFrame(
        ColliderType shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.PiOver4);
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape,
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            rotation);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(
                collider3D,
                circle.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Anchor3D.Origin.Should().Be(collider3D.Center);
        contact.Anchor3D.Rotation.Should().Be(collider3D.Rotation);
    }

    [Theory]
    [InlineData(ColliderType.Capsule, true)]
    [InlineData(ColliderType.Capsule, false)]
    [InlineData(ColliderType.Cylinder, true)]
    [InlineData(ColliderType.Cylinder, false)]
    [InlineData(ColliderType.Cone, true)]
    [InlineData(ColliderType.Cone, false)]
    public void FiniteAxisCapsuleSlab_AtScalarFace_ShouldRetainExactProjection(
        ColliderType shape,
        bool positive)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        SolidBody2D capsule2D = CreateBody2D(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            Vector2d.Zero);
        Fixed64 shapeCoordinate = positive
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 slabCoordinate =
            positive ? Fixed64.MaxValue : Fixed64.MinValue;
        collider3D.LocalOffset = new Vector3d(
            shapeCoordinate,
            Fixed64.Zero,
            Fixed64.Zero);
        capsule2D.Collider.LocalOffset = new Vector2d(
            slabCoordinate,
            Fixed64.Zero);
        collider3D.RebuildRuntimeShapeOnly().Should().BeTrue();
        capsule2D.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(
                collider3D,
                capsule2D.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        contact.Normal3DTo2D.Should().Be(
            positive ? Vector3d.Right : Vector3d.Left);
        contact.Anchor3D.Origin.Should().Be(collider3D.Center);
        contact.Anchor2D.Origin.Should().Be(
            MixedEmbedded2DGeometry.GetCenter3D(
                capsule2D.Collider));
    }

    [Theory]
    [InlineData(ColliderType.Capsule, true)]
    [InlineData(ColliderType.Capsule, false)]
    [InlineData(ColliderType.Cylinder, true)]
    [InlineData(ColliderType.Cylinder, false)]
    public void FiniteAxisCircleSlab_AtScalarFace_ShouldRetainExactProjection(
        ColliderType shape,
        bool positive)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        var circleCollider = new LSCircleCollider2D(Fixed64.Half)
        {
            MixedHalfThicknessOverride = Fixed64.Two
        };
        SolidBody2D circle2D = CreateBody2D(
            context,
            circleCollider,
            Vector2d.Zero);
        Fixed64 shapeCoordinate = positive
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 slabCoordinate =
            positive ? Fixed64.MaxValue : Fixed64.MinValue;
        collider3D.LocalOffset = new Vector3d(
            shapeCoordinate,
            Fixed64.Zero,
            Fixed64.Zero);
        circle2D.Collider.LocalOffset = new Vector2d(
            slabCoordinate,
            Fixed64.Zero);
        collider3D.RebuildRuntimeShapeOnly().Should().BeTrue();
        circle2D.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(
                collider3D,
                circle2D.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        contact.Normal3DTo2D.Should().Be(
            positive ? Vector3d.Right : Vector3d.Left);
        contact.Anchor3D.Origin.Should().Be(collider3D.Center);
        contact.Anchor2D.Origin.Should().Be(
            MixedEmbedded2DGeometry.GetCenter3D(
                circle2D.Collider));
    }

    [Fact]
    public void SphereCircleSlab_AtPositiveScalarFace_ShouldKeepSemanticBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere =
            CreateSphere3D(context, Vector3d.Zero);
        var circleCollider = new LSCircleCollider2D(Fixed64.Half)
        {
            MixedHalfThicknessOverride = Fixed64.Two
        };
        SolidBody2D circle =
            CreateBody2D(context, circleCollider, Vector2d.Zero);
        Fixed64 center =
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        sphere.Collider.LocalOffset =
            new Vector3d(center, Fixed64.Zero, Fixed64.Zero);
        circle.Collider.LocalOffset =
            new Vector2d(center, Fixed64.Zero);
        sphere.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        circle.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(
                sphere.Collider,
                circle.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Anchor2D.TryGetWorldPoint(out _).Should().BeFalse();
        contact.Anchor2D.TryGetOffsetFrom(
            new Vector3d(center, Fixed64.Zero, Fixed64.Zero),
            out Vector3d embeddedOffset).Should().BeTrue();
        embeddedOffset.Should().Be(Vector3d.Right * Fixed64.Half);
        contact.Normal3DTo2D.Should().Be(Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void CapsuleCircleSlab_RigidTranslation_ShouldPreserveCanonicalAnchorOffsets()
    {
        using GravitasWorldContext firstContext = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> firstCapsule = CreateCapsule3D(
            firstContext,
            -Vector3d.Right * Fixed64.FromFraction(1, 4));
        SolidBody2D firstCircle = CreateBody2D(
            firstContext,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        CollisionDetectionMixed.TryCollide(
                firstCapsule.Collider,
                firstCircle.Collider,
                out MixedContact first)
            .Should()
            .BeTrue();

        using GravitasWorldContext secondContext = CreateMixedContext();
        Vector3d translation = new((Fixed64)5, (Fixed64)2, (Fixed64)(-3));
        ScenarioBody<LSCapsuleCollider> secondCapsule = CreateCapsule3D(
            secondContext,
            translation - Vector3d.Right * Fixed64.FromFraction(1, 4));
        SolidBody2D secondCircle = CreateBody2D(
            secondContext,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(translation.X, translation.Z));
        secondCircle.Agent.Transform.TrySetWorldPosition(translation).Should().BeTrue();
        secondCircle.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        CollisionDetectionMixed.TryCollide(
                secondCapsule.Collider,
                secondCircle.Collider,
                out MixedContact second)
            .Should()
            .BeTrue();

        first.Anchor3D.TryGetOffsetFrom(
                firstCapsule.Collider.Center,
                out Vector3d first3DOffset)
            .Should()
            .BeTrue();
        second.Anchor3D.TryGetOffsetFrom(
                secondCapsule.Collider.Center,
                out Vector3d second3DOffset)
            .Should()
            .BeTrue();
        first.Anchor2D.TryGetOffsetFrom(
                MixedEmbedded2DGeometry.GetCenter3D(firstCircle.Collider),
                out Vector3d first2DOffset)
            .Should()
            .BeTrue();
        second.Anchor2D.TryGetOffsetFrom(
                MixedEmbedded2DGeometry.GetCenter3D(secondCircle.Collider),
                out Vector3d second2DOffset)
            .Should()
            .BeTrue();

        second3DOffset.Should().Be(first3DOffset);
        second2DOffset.Should().Be(first2DOffset);
        second.Normal3DTo2D.Should().Be(first.Normal3DTo2D);
        second.Depth.Should().Be(first.Depth);
    }

    [Fact]
    public void CompoundParts_ShouldRetainExactPartFrames()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound3D = CreateCompound3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * Fixed64.FromFraction(3, 4));

        CollisionDetectionMixed.TryCollide(
                compound3D.Collider,
                circle.Collider,
                out MixedContact from3DCompound)
            .Should()
            .BeTrue();

        Vector3d partCenter3D =
            Vector3d.Right * Fixed64.FromFraction(3, 4);
        from3DCompound.Anchor3D.Origin.Should().Be(partCenter3D);
        from3DCompound.Anchor3D.TryGetOffsetFrom(
                compound3D.Collider.Center,
                out Vector3d ownerRelative3D)
            .Should()
            .BeTrue();
        ownerRelative3D.X.Should().BeGreaterThan(Fixed64.Zero);

        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                Vector2d.Right * Fixed64.FromFraction(3, 4),
                PhysicsMaterial.Default));
        SolidBody2D compoundBody2D = CreateBody2D(context, compound2D, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            Vector3d.Right * Fixed64.FromFraction(3, 4));

        CollisionDetectionMixed.TryCollide(
                sphere.Collider,
                compoundBody2D.Collider,
                out MixedContact from2DCompound)
            .Should()
            .BeTrue();

        Vector3d partCenter2D = new(
            Fixed64.FromFraction(3, 4),
            compoundBody2D.Agent.Transform.WorldPosition.Y,
            Fixed64.Zero);
        from2DCompound.Anchor2D.Origin.Should().Be(partCenter2D);
        from2DCompound.Anchor2D.TryGetOffsetFrom(
                new Vector3d(
                    compoundBody2D.Collider.Center.X,
                    compoundBody2D.Agent.Transform.WorldPosition.Y,
                    compoundBody2D.Collider.Center.Y),
                out Vector3d ownerRelative2D)
            .Should()
            .BeTrue();
        ownerRelative2D.X.Should().BeGreaterThan(Fixed64.Zero);
    }
}
