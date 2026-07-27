using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class FiniteSurfaceContactAnchorTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatedFiniteSurfaceSphere_RetainsTheSelectedLocalFeature(
        bool cone)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)5);
        LSCollider finiteSurface = cone
            ? new LSConeCollider
            {
                Radius = (Fixed64)2,
                Size = new Vector3d((Fixed64)4, (Fixed64)4, (Fixed64)4)
            }
            : new LSCylinderCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(Fixed64.Two, (Fixed64)4, Fixed64.Two)
            };
        finiteSurface.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                rotation,
                Vector3d.One)));
        Vector3d localSphereCenter = new(
            (Fixed64)5 / (Fixed64)4,
            cone ? Fixed64.Zero : Fixed64.One,
            Fixed64.Zero);
        finiteSurface.Rotation.TryRotate(
            localSphereCenter,
            out Vector3d sphereCenter).Should().BeTrue();
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        sphere.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                sphereCenter,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Right,
                    Fixed64.Pi / (Fixed64)7),
                Vector3d.One)));
        var pair = new CollisionPair(finiteSurface, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(finiteSurface.Center);
        contact.AnchorA.Rotation.Should().Be(finiteSurface.Rotation);
        Fixed64 expectedAxialCoordinate = cone
            ? Fixed64.FromFraction(-1, 10)
            : Fixed64.One;
        contact.AnchorA.LocalPoint.Y.m_rawValue.Should().BeInRange(
            expectedAxialCoordinate.m_rawValue - 2L,
            expectedAxialCoordinate.m_rawValue + 2L);
        contact.AnchorB.Origin.Should().Be(sphere.Center);
        contact.AnchorB.Rotation.Should().Be(sphere.Rotation);
    }

    [Fact]
    public void ScalarFaceCylinderSphere_PreservesAuthoritativeContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateAtScalarFace(context, new LSCylinderCollider());
        LSSphereCollider sphere = CreateAtScalarFace(context, new LSSphereCollider());
        var pair = new CollisionPair(cylinder, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(cylinder.Center);
        contact.AnchorB.Origin.Should().Be(sphere.Center);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out _).Should().BeFalse();
    }

    [Fact]
    public void ScalarFaceConeSphere_PreservesAuthoritativeContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateAtScalarFace(context, new LSConeCollider());
        LSSphereCollider sphere = CreateAtScalarFace(context, new LSSphereCollider());
        var pair = new CollisionPair(cone, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(cone.Center);
        contact.AnchorB.Origin.Should().Be(sphere.Center);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, false)]
    [InlineData(true, true)]
    [InlineData(false, true)]
    public void ScalarFaceMeshFiniteSurface_PreservesTriangleAndShapeAnchors(
        bool positive,
        bool cone)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 face = positive ? Fixed64.MaxValue : Fixed64.MinValue;
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)2),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One)));

        Fixed64 centerX = positive
            ? Fixed64.MaxValue - Fixed64.One
            : Fixed64.MinValue + Fixed64.One;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            positive ? (Fixed64)(-90) : (Fixed64)90);
        LSCollider finiteSurface = cone
            ? new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(Fixed64.Two, (Fixed64)4, Fixed64.Two)
            }
            : new LSCylinderCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(Fixed64.Two, (Fixed64)4, Fixed64.Two)
            };
        finiteSurface.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero),
                rotation,
                Vector3d.One)));
        var pair = new CollisionPair(mesh, finiteSurface);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(mesh.Center);
        contact.AnchorB.Origin.Should().Be(finiteSurface.Center);
        contact.TryGetPointA(out Vector3d pointOnMesh).Should().BeTrue();
        pointOnMesh.X.Should().Be(face);
        contact.TryGetPointB(out _).Should().BeFalse();
    }

    private static TCollider CreateAtScalarFace<TCollider>(
        GravitasWorldContext context,
        TCollider collider)
        where TCollider : LSCollider
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.TrySetWorldPosition(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero)).Should().BeTrue();
        collider.RebuildRuntimeShapeOnly(refreshMassProperties: false).Should().BeTrue();
        return collider;
    }
}
