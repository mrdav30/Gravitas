using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Fact]
    public void CircleSlabTriangle_ObliqueRimGap_ShouldRejectDespiteOverlappingCandidateAxes()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(Fixed64.FromFraction(99, 16), Fixed64.FromFraction(9, 2), Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(67, 16), (Fixed64)6, Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(131, 16), Fixed64.FromFraction(37, 4), Fixed64.Zero)), Vector3d.Zero);
        var circle = new LSCircleCollider2D((Fixed64)5) { MixedHalfThicknessOverride = (Fixed64)5 };
        CreateBody2D(context, circle, Vector2d.Zero);

        // The triangle is separated from the rim along (3,4,0) by 5/16;
        // face, side-cross and vertex-to-axis directions do not detect the gap.
        CollisionDetectionMixed.TryCollide(mesh.Collider, circle, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void CircleSlabTriangle_ObliqueRim_ShouldUseTheSameGeometricContact(int yaw)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero)), Vector3d.Zero);
        var circle = new LSCircleCollider2D((Fixed64)5) { MixedHalfThicknessOverride = (Fixed64)5 };
        CreateBody2D(context, circle, Vector2d.Zero, (Fixed64)yaw);

        CollisionDetectionMixed.TryCollide(mesh.Collider, circle, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(5, 16));
        contact.DepthIsClamped.Should().BeFalse();
        contact.Anchor3D.Origin.Should().Be(mesh.Collider.Mesh.Origin);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        contact.Anchor2D.Rotation.Should().Be(FixedQuaternion.FromAxisAngle(Vector3d.Up, -(Fixed64)yaw));
        contact.Normal3DTo2D.X.m_rawValue.Should().BeInRange(
            Fixed64.FromFraction(-3, 5).m_rawValue - 2, Fixed64.FromFraction(-3, 5).m_rawValue + 2);
        contact.Normal3DTo2D.Y.m_rawValue.Should().BeInRange(
            Fixed64.FromFraction(-4, 5).m_rawValue - 2, Fixed64.FromFraction(-4, 5).m_rawValue + 2);
        contact.Normal3DTo2D.Z.Should().Be(Fixed64.Zero);
        contact.Point3D.X.m_rawValue.Should().BeInRange(
            Fixed64.FromFraction(77, 16).m_rawValue - 2, Fixed64.FromFraction(77, 16).m_rawValue + 2);
        contact.Point3D.Y.m_rawValue.Should().BeInRange(
            Fixed64.FromFraction(19, 4).m_rawValue - 2, Fixed64.FromFraction(19, 4).m_rawValue + 2);
        contact.Point3D.Z.m_rawValue.Should().BeInRange(-2, 2);
        contact.Point2D.X.m_rawValue.Should().BeInRange(((Fixed64)5).m_rawValue - 2, ((Fixed64)5).m_rawValue + 2);
        contact.Point2D.Y.Should().Be((Fixed64)5);
        contact.Point2D.Z.m_rawValue.Should().BeInRange(-2, 2);
    }
}
