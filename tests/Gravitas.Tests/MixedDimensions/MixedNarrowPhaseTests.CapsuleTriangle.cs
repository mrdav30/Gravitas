using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(1L)]
    [InlineData(4294967296L)]
    public void CapsuleSlabTriangle_ObliqueRimGap_ShouldRejectDespiteOverlappingBounds(long coreRaw)
    {
        using GravitasWorldContext context = CreateMixedContext();
        // Swap X/Z from the upstream separator fixture to use the collider's
        // authored Forward core without introducing a rounded rotation.
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(9, 2), Fixed64.FromFraction(99, 16)),
            new Vector3d(Fixed64.FromFraction(-5, 4), (Fixed64)6, Fixed64.FromFraction(67, 16)),
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(37, 4), Fixed64.FromFraction(131, 16))), Vector3d.Zero);
        var capsule = new LSCapsuleCollider2D((Fixed64)5, (Fixed64)10 + Fixed64.FromRaw(coreRaw))
        {
            MixedHalfThicknessOverride = (Fixed64)5
        };
        CreateBody2D(context, capsule, Vector2d.Zero);

        // Along (0,4/5,3/5), the gap is 5/16-3*coreLength/10 > 0.
        mesh.Collider.Bounds.Intersects(capsule.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(mesh.Collider, capsule, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CapsuleSlabTriangle_InteriorRimRoot_ShouldPreserveUpstreamDepthAndAnchors()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(Fixed64.FromFraction(231, 64), Fixed64.FromFraction(309, 64), Fixed64.FromFraction(1021, 256)),
            new Vector3d(Fixed64.FromFraction(279, 64), Fixed64.FromFraction(325, 64), Fixed64.FromFraction(509, 256)),
            new Vector3d(Fixed64.FromFraction(271, 64), Fixed64.FromFraction(365, 64), Fixed64.FromFraction(813, 256))), Vector3d.Forward);
        var capsule = new LSCapsuleCollider2D((Fixed64)5, (Fixed64)12)
        {
            MixedHalfThicknessOverride = (Fixed64)5
        };
        CreateBody2D(context, capsule, Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(mesh.Collider, capsule, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(13, 256));
        contact.DepthIsClamped.Should().BeFalse();
        contact.Normal3DTo2D.Should().Be(new Vector3d(-Fixed64.FromFraction(4, 13),
            -Fixed64.FromFraction(12, 13), -Fixed64.FromFraction(3, 13)));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.FromFraction(255, 64),
            Fixed64.FromFraction(317, 64), Fixed64.FromFraction(1021, 256)));
        contact.Point2D.Should().Be(new Vector3d(4, 5, 4));
        contact.Anchor3D.Origin.Should().Be(mesh.Collider.Mesh.Origin);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CapsuleSlabTriangle_MiddleCap_ShouldPairWitnessesOutsideBothEndDisks()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Two),
            new Vector3d(-Fixed64.Half, Fixed64.Half, (Fixed64)3),
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Two)), Vector3d.Zero);
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)12)
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        CreateBody2D(context, capsule, Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(mesh.Collider, capsule, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Half);
        contact.Normal3DTo2D.Should().Be(Vector3d.Down);
        contact.Point3D.Y.Should().Be(Fixed64.Half);
        contact.Point2D.Y.Should().Be(Fixed64.One);
        contact.Point3D.X.Should().Be(contact.Point2D.X);
        contact.Point3D.Z.Should().Be(contact.Point2D.Z);
        contact.Point3D.Z.Should().BeInRange(Fixed64.Two, (Fixed64)3);
        contact.Point3D.X.Should().BeInRange(-Fixed64.Half, Fixed64.Half);
    }
}
