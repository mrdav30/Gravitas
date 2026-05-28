using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConcaveMeshCollisionTests
{
    [Fact]
    public void ConcaveMeshSphere_ShouldHitInteriorExteriorAndTouchingFeatures()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> corner = scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);

        CollisionPair wallHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateSphere(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Sphere);
        wallHit.Manifold.PrimaryContact.Normal.x.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair floorHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Fraction(1, 4), (Fixed64)2)).Collider,
            CollisionType.Mesh_Sphere);
        floorHit.Manifold.PrimaryContact.Normal.y.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair touching = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Sphere);
        touching.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);

        AssertNoCollision(
            scenario,
            corner.Collider,
            scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Sphere);
    }

    [Fact]
    public void ConcaveMeshCapsule_ShouldUseTriangleFeaturesInsteadOfMeshCenter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> corner = scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);

        CollisionPair wallHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCapsule(), new Vector3d(Fixed64.Fraction(1, 4), Fixed64.One, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Capsule);
        wallHit.Manifold.PrimaryContact.Normal.x.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair floorHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCapsule(), new Vector3d((Fixed64)2, Fixed64.Fraction(1, 4), (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Capsule);
        floorHit.Manifold.PrimaryContact.Normal.y.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair touching = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCapsule(), new Vector3d(Fixed64.Half, Fixed64.One, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Capsule);
        touching.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);

        AssertNoCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCapsule(), new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Capsule);
    }

    [Fact]
    public void ConcaveMeshCuboid_ShouldNotTreatOpenChannelAsConvexHull()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> channel = scenario.CreateBody(
            MeshTestFixtures.CreateUChannel(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSMeshCollider> corner = scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(8, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);

        AssertNoCollision(
            scenario,
            channel.Collider,
            scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Cuboid);

        CollisionPair wallHit = AssertCollision(
            scenario,
            channel.Collider,
            scenario.CreateCuboid(new Vector3d(Fixed64.Fraction(7, 4), Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Cuboid);
        wallHit.Manifold.PrimaryContact.Normal.x.Should().BeLessThan(Fixed64.Zero);

        CollisionPair floorHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateCuboid(new Vector3d((Fixed64)10, Fixed64.Fraction(1, 4), (Fixed64)2)).Collider,
            CollisionType.Mesh_Cuboid);
        floorHit.Manifold.PrimaryContact.Normal.y.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair touching = AssertCollision(
            scenario,
            channel.Collider,
            scenario.CreateCuboid(new Vector3d(Fixed64.Fraction(3, 2), Fixed64.One, (Fixed64)2)).Collider,
            CollisionType.Mesh_Cuboid);
        touching.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ConcaveMeshCylinder_ShouldHitInteriorFeaturesAndSeparatedCases()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> corner = scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);

        CollisionPair wallHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCylinder(), new Vector3d(Fixed64.Fraction(1, 4), Fixed64.One, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Cylinder);
        wallHit.Manifold.PrimaryContact.Normal.x.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair floorHit = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCylinder(), new Vector3d((Fixed64)2, Fixed64.Fraction(3, 4), (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Cylinder);
        floorHit.Manifold.PrimaryContact.Normal.y.Should().BeGreaterThan(Fixed64.Zero);

        CollisionPair touching = AssertCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCylinder(), new Vector3d(Fixed64.Half, Fixed64.One, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Cylinder);
        touching.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);

        AssertNoCollision(
            scenario,
            corner.Collider,
            scenario.CreateBody(CreateTallCylinder(), new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)2), FixedQuaternion.Identity).Collider,
            CollisionType.Mesh_Cylinder);
    }

    [Fact]
    public void ConcaveMeshMesh_ShouldSupportConcaveConvexAndConcaveConcaveDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> concave = scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSMeshCollider> convex = scenario.CreateBody(
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, Fixed64.One, (Fixed64)3),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSMeshCollider> otherConcave = scenario.CreateBody(
            MeshTestFixtures.CreateUChannel(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);

        CollisionPair concaveConvex = AssertCollision(scenario, concave.Collider, convex.Collider, CollisionType.Mesh_Mesh);
        CollisionPair convexConcave = AssertCollision(scenario, convex.Collider, concave.Collider, CollisionType.Mesh_Mesh);
        CollisionPair concaveConcave = AssertCollision(scenario, concave.Collider, otherConcave.Collider, CollisionType.Mesh_Mesh);
        ulong[] firstIds = concaveConcave.Manifold.Select(contact => contact.ContactId).ToArray();

        CollisionDetection.DoCollisionCheck(concaveConcave).Should().BeTrue();

        concaveConvex.Manifold.HasContact.Should().BeTrue();
        convexConcave.Manifold.HasContact.Should().BeTrue();
        concaveConcave.Manifold.Select(contact => contact.ContactId).Should().Equal(firstIds);
    }

    [Fact]
    public void DynamicConcaveMeshAndPrimitive_ShouldProduceDeterministicContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> channel = scenario.CreateBody(
            MeshTestFixtures.CreateUChannel(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.Fraction(7, 4), Fixed64.One, (Fixed64)2),
            immovable: true);
        CollisionPair pair = scenario.CreatePair(channel.Collider, cuboid.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact first = pair.Manifold.PrimaryContact;

        channel.Body.SetPosition(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        channel.Collider.Simulate();
        cuboid.Body.SetPosition(new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        cuboid.Collider.Simulate();

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.x.Should().Be(first.Normal.x);
        pair.Manifold.PrimaryContact.ContactId.Should().NotBe(0);
    }

    [Fact]
    public void ConcaveMeshChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> channel = scenario.CreateBody(
            MeshTestFixtures.CreateUChannel(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        CollisionPair cuboidPair = scenario.CreatePair(
            channel.Collider,
            scenario.CreateCuboid(new Vector3d(Fixed64.Fraction(7, 4), Fixed64.One, (Fixed64)2)).Collider);
        CollisionPair cylinderPair = scenario.CreatePair(
            channel.Collider,
            scenario.CreateBody(CreateTallCylinder(), new Vector3d(Fixed64.Fraction(7, 4), Fixed64.One, (Fixed64)2), FixedQuaternion.Identity).Collider);
        CollisionPair meshPair = scenario.CreatePair(
            channel.Collider,
            scenario.CreateBody(MeshTestFixtures.CreateInsideCorner(), PhysicsScenarioBuilder.Vector(0, 0, 0), FixedQuaternion.Identity, immovable: true).Collider);

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            EnsureCollision(cuboidPair);
            EnsureCollision(cylinderPair);
            EnsureCollision(meshPair);
        });

        allocatedBytes.Should().Be(0);
    }

    private static CollisionPair AssertCollision(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB,
        CollisionType expectedType)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);

        pair.CollisionType.Should().Be(expectedType);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        return pair;
    }

    private static void AssertNoCollision(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB,
        CollisionType expectedType)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);

        pair.CollisionType.Should().Be(expectedType);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    private static LSCapsuleCollider CreateTallCapsule() =>
        new()
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };

    private static LSCylinderCollider CreateTallCylinder() =>
        new()
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };

    private static long MeasureAllocatedBytes(Action action)
    {
        for (int i = 0; i < 128; i++)
            action();

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 64; i++)
            action();

        return GC.GetAllocatedBytesForCurrentThread() - before;
    }

    private static void EnsureCollision(CollisionPair pair)
    {
        if (!CollisionDetection.DoCollisionCheck(pair))
            throw new InvalidOperationException("Expected the prepared collision pair to collide.");
    }
}
