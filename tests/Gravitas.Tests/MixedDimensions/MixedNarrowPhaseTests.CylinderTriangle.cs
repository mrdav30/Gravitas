using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void CircleSlabQuad_ShouldMatchTheWholeCylinderExitAcrossCoveredDiagonal(
        bool stadium, bool reverseWinding, bool vertical)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var vertices = new[] { new Vector3d(-1, 0, -1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, -Fixed64.One),
            new Vector3d(-1, 0, 1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.One) };
        int[] indices = reverseWinding ? new[] { 1, 2, 0, 3, 2, 1 } : new[] { 0, 2, 1, 1, 2, 3 };
        Vector3d normal = vertical ? Vector3d.Right : Vector3d.Up;
        Vector3d origin = -normal * Fixed64.FromFraction(1, 10);
        FixedQuaternion rotation = vertical
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized
            : FixedQuaternion.Identity;
        var mesh = CreateMesh3D(context,
            new LSMeshCollider(vertices, indices, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation),
            origin, rotation);
        LSCollider2D embedded = stadium ? new LSCapsuleCollider2D(Fixed64.Quarter, Fixed64.Half)
            : new LSCircleCollider2D(Fixed64.Quarter);
        embedded.MixedHalfThicknessOverride = Fixed64.Half;
        CreateBody2D(context, embedded, Vector2d.Zero);
        if (embedded is LSCapsuleCollider2D capsule) capsule.AxisLength.Should().Be(Fixed64.Zero);
        var cylinder = CreateBody3D(context,
            new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One },
            Vector3d.Zero, FixedQuaternion.Identity);
        var native = new CollisionPair(mesh.Collider, cylinder.Collider);
        var mixed = new ContactManifold();

        CollisionDetection.DoCollisionCheck(native).Should().BeTrue();
        CollisionDetectionMixed.TryBuildManifold(mesh.Collider, embedded, mixed).Should().BeTrue();
        Fixed64 depth = (vertical ? Fixed64.Quarter : Fixed64.Half) - Fixed64.FromFraction(1, 10);
        foreach (ContactManifold manifold in new[] { native.Manifold, mixed })
        {
            manifold.Count.Should().BeGreaterThan(0);
            foreach (ManifoldContact row in manifold)
            {
                row.Normal.Should().Be(normal);
                row.Depth.Should().Be(depth);
                row.DepthIsClamped.Should().BeFalse();
                row.TryGetPointA(out Vector3d source).Should().BeTrue();
                row.TryGetPointB(out Vector3d exit).Should().BeTrue();
                Vector3d.Dot(source - origin, normal).Should().Be(Fixed64.Zero);
                (vertical ? source.Y : source.X).Should().BeInRange(-Fixed64.One, Fixed64.FromFraction(5, 4));
                source.Z.Should().BeInRange(-Fixed64.One, Fixed64.One);
                exit.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half);
                (exit.X * exit.X + exit.Z * exit.Z).Should().BeLessThanOrEqualTo(
                    Fixed64.FromFraction(1, 16) + Fixed64.MinIncrement);
                Vector3d.Dot(exit, normal).Should().Be(vertical ? -Fixed64.Quarter : -Fixed64.Half);
                (source - exit).Should().Be(normal * depth);
            }
        }
        // Moving the finite patch beyond the analytic bottom/side support
        // separates both representations, independently of triangle selection.
        mesh.Body.SetPosition(origin - normal * (depth + Fixed64.MinIncrement));
        mesh.Collider.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(native).Should().BeFalse();
        CollisionDetectionMixed.TryBuildManifold(mesh.Collider, embedded, mixed).Should().BeFalse();
    }

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
    public void CircleSlabTriangle_ObliqueRim_ShouldRetainTheSameGeometricConstraint(int yaw)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = CreateMesh3D(context, CreateTriangleMesh(
            new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero)), Vector3d.Zero);
        var circle = new LSCircleCollider2D((Fixed64)5) { MixedHalfThicknessOverride = (Fixed64)5 };
        CreateBody2D(context, circle, Vector2d.Zero, (Fixed64)yaw);

        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh.Collider, circle, manifold).Should().BeTrue();
        manifold.Should().Contain(row => row.Depth == Fixed64.FromFraction(5, 16));
        MixedContact contact = MixedContact.FromManifoldContact(manifold.First(row => row.Depth == Fixed64.FromFraction(5, 16)));
        CollisionDetectionMixed.TryCollide(mesh.Collider, circle, out MixedContact representative).Should().BeTrue();
        representative.Depth.Should().Be(manifold.PrimaryContact.Depth);
        representative.Anchor3D.Should().Be(manifold.PrimaryContact.AnchorA);
        representative.Anchor2D.Should().Be(manifold.PrimaryContact.AnchorB);
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
