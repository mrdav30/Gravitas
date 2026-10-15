using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class MeshTrianglePairContactTests
{
    [Fact]
    public void ConvexMeshMesh_ShouldForwardCanonicalContactInBothDispatchDirections()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d secondOrigin =
            Fixed64.FromFraction(3, 4) * Vector3d.Right;
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            secondOrigin,
            FixedQuaternion.Identity);
        CollisionPair forward = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair reversed = scenario.CreatePair(second.Collider, first.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        Fixed64 half = Fixed64.Half;
        ManifoldContact contact = forward.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(Vector3d.Zero);
        contact.AnchorA.Rotation.Should().Be(FixedQuaternion.Identity);
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(half, -half, -half));
        contact.AnchorB.Origin.Should().Be(secondOrigin);
        contact.AnchorB.Rotation.Should().Be(FixedQuaternion.Identity);
        contact.AnchorB.LocalPoint.Should().Be(new Vector3d(-half, -half, -half));
        contact.Normal.Should().Be(Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.DepthIsClamped.Should().BeFalse();

        ManifoldContact reversedContact = reversed.Manifold.PrimaryContact;
        reversedContact.ContactId.Should().Be(contact.ContactId);
        reversedContact.AnchorA.Should().Be(contact.AnchorB);
        reversedContact.AnchorB.Should().Be(contact.AnchorA);
        reversedContact.Normal.Should().Be(-contact.Normal);
        reversedContact.Depth.Should().Be(contact.Depth);
        reversedContact.DepthIsClamped.Should().Be(contact.DepthIsClamped);
    }

    [Fact]
    public void MeshMesh_WithUnsupportedCanonicalExtent_ShouldRejectBeforePairCreation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.FromRaw(4611686011448066045L);
        LSMeshCollider mesh = CreateSeparatedTriangleMesh(extent);
        Action initialize = () => InitializeStaticMesh(scenario.Context, mesh,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), Vector3d.One);
        initialize.Should().Throw<ArgumentException>().WithMessage("*geometry reach*");
        mesh.Id.Should().Be(-1);
    }

    [Fact]
    public void MeshMesh_SupportedLargeEdgeCrossSeparatorShouldNotProduceContact()
    {
        Fixed64 scale = (Fixed64)512;
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d(1, -2, 2) * scale,
            new Vector3d(0, 3, 1) * scale,
            new Vector3d(-2, 2, 1) * scale);
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d(2, 2, -2) * scale,
            new Vector3d(-2, 0, -3) * scale,
            new Vector3d(-2, -1, 2) * scale);
        InitializeStaticMesh(context, first, Vector3d.One);
        InitializeStaticMesh(context, second, Vector3d.One);
        var manifold = new ContactManifold();
        var workItem = new CollisionWorkItem(
            context,
            first,
            second,
            CollisionType.Mesh_Mesh,
            manifold);

        first.Bounds.Intersects(second.Bounds).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(workItem).Should().BeFalse();
        manifold.Count.Should().Be(0);
    }

    [Fact]
    public void MeshMesh_ShouldPreserveExactCanonicalLocalAnchors()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d(0, 0, 0),
            new Vector3d(3, 0, 0),
            new Vector3d(0, 3, 0));
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d(0, 0, -3),
            new Vector3d(0, 0, 3),
            new Vector3d(0, 3, 0));
        ContactManifold manifold = Collide(context, first, second);

        ManifoldContact contact = manifold.PrimaryContact;
        Fixed64 roundedThird = Fixed64.Half + Fixed64.MinIncrement;
        contact.AnchorA.Origin.Should().Be(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2), Fixed64.Zero));
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(-Fixed64.FromFraction(3, 2), -roundedThird, Fixed64.Zero));
        contact.AnchorB.Origin.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.Zero));
        contact.AnchorB.LocalPoint.Should().Be(new Vector3d(
            Fixed64.Zero,
            -roundedThird,
            Fixed64.MinIncrement * (Fixed64)3));
        contact.PointA.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero));
        contact.PointB.Should().Be(new Vector3d(
            Fixed64.Zero,
            Fixed64.One - Fixed64.MinIncrement,
            Fixed64.MinIncrement * (Fixed64)3));
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void MeshMesh_ShouldRetainCanonicalAnchorsWhenWorldPointsAreUnrepresentable()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 2), Fixed64.Zero));
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        scenario.InitializeStaticCollider(first, origin);
        scenario.InitializeStaticCollider(second, origin);

        CollisionPair pair = scenario.CreatePair(first, second);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(origin);
        contact.AnchorB.Origin.Should().Be(origin);
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(
            Fixed64.One - Fixed64.MinIncrement * (Fixed64)4,
            -Fixed64.Half - Fixed64.MinIncrement,
            Fixed64.Zero));
        contact.AnchorB.LocalPoint.Should().Be(contact.AnchorA.LocalPoint);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out _).Should().BeFalse();
    }

    [Fact]
    public void MeshMesh_ReversedDispatchShouldPreserveStableContactOrdering()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider first = CreateQuadMesh(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0),
            new Vector3d(2, 2, 0));
        LSMeshCollider second = CreateQuadMesh(
            new Vector3d(0, -2, -2),
            new Vector3d(0, -2, 2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2));
        scenario.InitializeStaticCollider(first, Vector3d.Zero);
        scenario.InitializeStaticCollider(
            second,
            Fixed64.FromFraction(1, 4) * Vector3d.Right);
        CollisionPair forward = scenario.CreatePair(first, second);
        CollisionPair reversed = scenario.CreatePair(second, first);
        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        ManifoldContact[] expectedContacts = forward.Manifold.ToArray();
        ManifoldContact[] expectedReversedContacts = reversed.Manifold.ToArray();
        ulong[] expectedIds = expectedContacts.Select(contact => contact.ContactId).ToArray();
        ulong[] expectedReversedIds = expectedReversedContacts.Select(contact => contact.ContactId).ToArray();

        expectedContacts.Should().HaveCountGreaterThan(1);
        expectedIds.Should().BeInAscendingOrder();
        expectedReversedIds.Should().BeInAscendingOrder();
        expectedReversedIds.Should().Equal(expectedIds);
        for (int index = 0; index < expectedContacts.Length; index++)
        {
            ManifoldContact contact = expectedContacts[index];
            ManifoldContact reversedContact = expectedReversedContacts[index];
            reversedContact.AnchorA.Should().Be(contact.AnchorB);
            reversedContact.AnchorB.Should().Be(contact.AnchorA);
            reversedContact.Normal.Should().Be(-contact.Normal);
            reversedContact.Depth.Should().Be(contact.Depth);
            reversedContact.DepthIsClamped.Should().Be(contact.DepthIsClamped);
        }

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        forward.Manifold.Select(contact => contact.ContactId).Should().Equal(expectedIds);
        reversed.Manifold.Select(contact => contact.ContactId).Should().Equal(expectedReversedIds);
        forward.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        reversed.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
    }

    [Fact]
    public void MeshMesh_WithUnsupportedDepthGeometry_ShouldRejectBeforePairCreation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 half = Fixed64.MaxValue / Fixed64.Two - Fixed64.One;
        LSMeshCollider mesh = CreateTriangleMesh(new Vector3d(-half, half, -half),
            new Vector3d(-half, -half, half), new Vector3d(half, half, half), MeshInertiaPolicy.RequireClosedVolume);
        Action initialize = () => InitializeStaticMesh(scenario.Context, mesh, Vector3d.One * Fixed64.Two);
        initialize.Should().Throw<ArgumentException>().WithMessage("*geometry reach*");
        mesh.Id.Should().Be(-1);
    }

    [Fact]
    public void MeshMesh_WarmedChecksShouldNotAllocate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(CreateQuadMesh(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0),
            new Vector3d(2, 2, 0)), Vector3d.Zero, FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(CreateQuadMesh(
            new Vector3d(0, -2, -2),
            new Vector3d(0, -2, 2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2)), Vector3d.Zero, FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair))
                throw new InvalidOperationException("Expected the prepared mesh pair to collide.");
        });

        allocatedBytes.Should().Be(0);
    }

    private static LSMeshCollider CreateTriangleMesh(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.SurfaceApproximation) =>
        new(
            new[] { first, second, third },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            inertiaPolicy);

    private static LSMeshCollider CreateSeparatedTriangleMesh(Fixed64 extent) =>
        new(
            new[]
            {
                new Vector3d(-extent, -extent, -extent),
                new Vector3d(-extent, -extent, extent),
                new Vector3d(extent, -extent, -extent),
                new Vector3d(-extent, extent, -extent),
                new Vector3d(extent, extent, -extent),
                new Vector3d(-extent, extent, extent)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.RequireClosedVolume);

    private static LSMeshCollider CreateQuadMesh(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d fourth) =>
        new(
            new[] { first, second, third, fourth },
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

    private static ContactManifold Collide(
        GravitasWorldContext context,
        LSMeshCollider first,
        LSMeshCollider second)
    {
        var manifold = new ContactManifold();
        CollisionDetection.DoCollisionCheck(CreateWorkItem(context, first, second, manifold)).Should().BeTrue();
        return manifold;
    }

    private static CollisionWorkItem CreateWorkItem(
        GravitasWorldContext context,
        LSMeshCollider first,
        LSMeshCollider second,
        ContactManifold manifold) =>
        new(context, first, second, CollisionType.Mesh_Mesh, manifold);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d scale) =>
        InitializeStaticMesh(context, collider, Vector3d.Zero, scale);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        Vector3d scale) =>
        InitializeStaticMesh(
            context,
            collider,
            position,
            FixedQuaternion.Identity,
            scale);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d scale)
    {
        var transform = new FixedTransform(position, rotation, scale);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
    }

}
