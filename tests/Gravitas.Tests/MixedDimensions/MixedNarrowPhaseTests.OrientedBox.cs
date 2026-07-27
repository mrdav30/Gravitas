//=======================================================================
// MixedNarrowPhaseTests.OrientedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle, true)]
    [InlineData(ColliderType2D.Circle, false)]
    [InlineData(ColliderType2D.AABox, true)]
    [InlineData(ColliderType2D.AABox, false)]
    [InlineData(ColliderType2D.Capsule, true)]
    [InlineData(ColliderType2D.Capsule, false)]
    [InlineData(ColliderType2D.ConvexPolygon, true)]
    [InlineData(ColliderType2D.ConvexPolygon, false)]
    public void RotatedCuboidEmbeddedSlab_AtScalarFace_ShouldMatchTranslatedCanonicalGeometry(
        ColliderType2D embeddedShape,
        bool positiveFace)
    {
        MixedContact baseline = GetRotatedCuboidEmbeddedContact(
            embeddedShape,
            cuboidCenterX: Fixed64.Zero,
            embeddedCenterX: positiveFace
                ? -Fixed64.FromFraction(9, 10)
                : Fixed64.FromFraction(9, 10),
            positiveFace,
            scalarFaceGrid: false);
        Fixed64 cuboidCenterX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 embeddedCenterX = positiveFace
            ? cuboidCenterX - Fixed64.FromFraction(9, 10)
            : cuboidCenterX + Fixed64.FromFraction(9, 10);
        MixedContact translated = GetRotatedCuboidEmbeddedContact(
            embeddedShape,
            cuboidCenterX,
            embeddedCenterX,
            positiveFace,
            scalarFaceGrid: true);

        translated.Normal3DTo2D.Should().Be(baseline.Normal3DTo2D);
        translated.Depth.Should().Be(baseline.Depth);
        translated.DepthIsClamped.Should().Be(baseline.DepthIsClamped);
        baseline.Anchor3D.TryGetOffsetFrom(Vector3d.Zero, out Vector3d baseline3DOffset)
            .Should().BeTrue();
        translated.Anchor3D.TryGetOffsetFrom(
                new Vector3d(cuboidCenterX, Fixed64.Zero, Fixed64.Zero),
                out Vector3d translated3DOffset)
            .Should().BeTrue();
        translated3DOffset.Should().Be(baseline3DOffset);
        baseline.Anchor2D.TryGetOffsetFrom(
                new Vector3d(
                    positiveFace
                        ? -Fixed64.FromFraction(9, 10)
                        : Fixed64.FromFraction(9, 10),
                    Fixed64.Zero,
                    Fixed64.Zero),
                out Vector3d baseline2DOffset)
            .Should().BeTrue();
        translated.Anchor2D.TryGetOffsetFrom(
                new Vector3d(embeddedCenterX, Fixed64.Zero, Fixed64.Zero),
                out Vector3d translated2DOffset)
            .Should().BeTrue();
        translated2DOffset.Should().Be(baseline2DOffset);
    }

    private static MixedContact GetRotatedCuboidEmbeddedContact(
        ColliderType2D embeddedShape,
        Fixed64 cuboidCenterX,
        Fixed64 embeddedCenterX,
        bool positiveFace,
        bool scalarFaceGrid)
    {
        using GravitasWorldContext context = CreateOrientedBoxNarrowPhaseContext(
            positiveFace,
            scalarFaceGrid);
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(cuboidCenterX, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)45,
                Fixed64.Zero));
        SolidBody2D embedded = CreateBody2D(
            context,
            CreateOrientedBoxEmbeddedCollider(embeddedShape),
            new Vector2d(embeddedCenterX, Fixed64.Zero),
            GetOrientedBoxEmbeddedRotation(embeddedShape));

        CollisionDetectionMixed.TryCollide(
                cuboid.Collider,
                embedded.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();
        return contact;
    }

    private static LSCollider2D CreateOrientedBoxEmbeddedCollider(ColliderType2D shape) =>
        shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(
                Fixed64.FromFraction(1, 4),
                Fixed64.One),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)),
            _ => throw new System.ArgumentOutOfRangeException(nameof(shape), shape, null)
        };

    private static Fixed64 GetOrientedBoxEmbeddedRotation(ColliderType2D shape) =>
        shape switch
        {
            ColliderType2D.AABox => FixedMath.DegToRad((Fixed64)15),
            ColliderType2D.Capsule => FixedMath.DegToRad((Fixed64)30),
            ColliderType2D.ConvexPolygon => FixedMath.DegToRad((Fixed64)(-20)),
            _ => Fixed64.Zero
        };

    private static GravitasWorldContext CreateOrientedBoxNarrowPhaseContext(
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
