using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
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
        // The contained sphere's inward support is representable even when
        // the solid's outward surface exceeds the world-coordinate boundary.
        contact.TryGetPointB(out Vector3d spherePoint).Should().BeTrue();
        spherePoint.X.Should().BeLessThan(sphere.Center.X);
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
        // The contained sphere's inward support is representable even when
        // the solid's outward surface exceeds the world-coordinate boundary.
        contact.TryGetPointB(out Vector3d spherePoint).Should().BeTrue();
        spherePoint.X.Should().BeLessThan(sphere.Center.X);
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
        LSMeshCollider mesh = CreateScalarFaceMesh();
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

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeshCone_WholeUnitTranslationPreservesContactsAndResponseAtScalarFaces(bool positive)
    {
        using PhysicsScenarioBuilder baselineScenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder translatedScenario = PhysicsScenarioBuilder.Create();
        // A whole-unit shift preserves the global lattice's half-tie parity.
        // Both scenes have a valid body root and an outward cone point that
        // crosses the scalar face; the contact is authoritative in its frame.
        Fixed64 face = positive
            ? Fixed64.MaxValue - (Fixed64.One - Fixed64.FromRaw(1))
            : Fixed64.MinValue;
        CollisionPair baseline = CreateConePair(baselineScenario, Fixed64.Zero);
        CollisionPair translated = CreateConePair(translatedScenario, face);
        SolidBody baselineBody = baseline.ColliderB.Body!;
        SolidBody translatedBody = translated.ColliderB.Body!;
        Vector3d originalPosition = translatedBody.Position3d;
        Vector3d originalBaselinePosition = baselineBody.Position3d;

        CollisionDetection.DoCollisionCheck(baseline).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(translated).Should().BeTrue();
        translatedBody.Position3d.Should().Be(originalPosition,
            "sampling coordinates must not move the simulated body");
        translatedBody.LinearVelocity.Should().Be(Vector3d.Zero);
        translated.Manifold.Count.Should().Be(baseline.Manifold.Count);
        translated.Manifold.GroupCount.Should().Be(baseline.Manifold.GroupCount);
        for (int index = 0; index < baseline.Manifold.Count; index++)
        {
            ManifoldContact expected = baseline.Manifold[index];
            ManifoldContact actual = translated.Manifold[index];
            actual.Depth.Should().Be(expected.Depth);
            actual.Normal.Should().Be(expected.Normal);
            actual.ContactId.Should().Be(expected.ContactId);
            actual.AnchorA.Origin.Should().Be(translated.ColliderA.Center);
            actual.AnchorB.Origin.Should().Be(translated.ColliderB.Center);
            actual.AnchorA.Rotation.Should().Be(expected.AnchorA.Rotation);
            actual.AnchorB.Rotation.Should().Be(expected.AnchorB.Rotation);
            actual.AnchorA.TryGetOffsetFrom(actual.AnchorA.Origin, out Vector3d meshOffset).Should().BeTrue();
            expected.AnchorA.TryGetOffsetFrom(expected.AnchorA.Origin, out Vector3d expectedMeshOffset).Should().BeTrue();
            meshOffset.Should().Be(expectedMeshOffset);
            actual.AnchorB.TryGetOffsetFrom(actual.AnchorB.Origin, out Vector3d coneOffset).Should().BeTrue();
            expected.AnchorB.TryGetOffsetFrom(expected.AnchorB.Origin, out Vector3d expectedConeOffset).Should().BeTrue();
            coneOffset.Should().Be(expectedConeOffset);
        }
        translated.Manifold.PrimaryContact.TryGetPointB(out _).Should().BeFalse();

        Vector3d incoming = positive ? Vector3d.Right : Vector3d.Left;
        baselineBody.AddLinearImpulse(incoming);
        translatedBody.AddLinearImpulse(incoming);
        CollisionResponse.CalculateImpulse(baseline);
        CollisionResponse.CalculateImpulse(translated);
        translatedBody.Position3d.Should().Be(originalPosition + (baselineBody.Position3d - originalBaselinePosition));
        translatedBody.LinearVelocity.Should().Be(baselineBody.LinearVelocity);
        translatedBody.AngularVelocity.Should().Be(baselineBody.AngularVelocity);

        CollisionPair CreateConePair(PhysicsScenarioBuilder scenario, Fixed64 plane)
        {
            LSMeshCollider mesh = CreateScalarFaceMesh();
            mesh.Material = PhysicsMaterial.Frictionless;
            scenario.CreateBody(mesh, new Vector3d(plane, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity, immovable: true);
            var cone = new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(Fixed64.Two, (Fixed64)4, Fixed64.Two),
                Material = PhysicsMaterial.Frictionless
            };
            FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero, Fixed64.Zero, positive ? (Fixed64)(-90) : (Fixed64)90);
            scenario.CreateBody(cone, new Vector3d(plane + (positive ? -Fixed64.One : Fixed64.One),
                Fixed64.Zero, Fixed64.Zero), rotation, preventAngularForces: true).Body.UseManualGrounding();
            return scenario.CreatePair(mesh, cone);
        }
    }

    private static LSMeshCollider CreateScalarFaceMesh() => new(
        new[]
        {
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)2),
            new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);

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
