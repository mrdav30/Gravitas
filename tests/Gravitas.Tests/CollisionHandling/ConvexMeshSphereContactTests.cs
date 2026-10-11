using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexMeshSphereContactTests
{
    private static readonly Fixed64 Radius = Fixed64.FromFraction(1, 8);
    // Unit-sized fixtures: nearest-even depth, normalized direction and final
    // coordinate products contribute at most eight raw units to exit checks.
    private static readonly Fixed64 RoundingBound = Fixed64.FromRaw(8);

    [Theory]
    [InlineData(0, 0, true, 10)]
    [InlineData(4, 0, true, 6)]
    [InlineData(9, 0, true, 1)]
    [InlineData(10, 0, true, 0)]
    [InlineData(11, 0, false, 0)]
    [InlineData(9, 9, true, -1)]
    [InlineData(10, 10, false, 0)]
    public void SphereAndZeroCoreCapsule_ShouldAgreeOnCompleteHullContact(
        int xSixteenths, int ySixteenths, bool overlap, int depthSixteenths)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        Vector3d center = new(Fixed64.FromFraction(xSixteenths, 16), Fixed64.FromFraction(ySixteenths, 16), Fixed64.Zero);
        LSSphereCollider sphere = new() { Radius = Radius };
        LSCapsuleCollider capsule = new() { Radius = Radius, Size = Vector3d.One * (Radius * Fixed64.Two) };
        scenario.InitializeStaticCollider(sphere, center);
        scenario.InitializeStaticCollider(capsule, center);
        capsule.AxisLength.Should().Be(Fixed64.Zero);

        foreach (bool reverse in new[] { false, true })
        {
            CollisionPair spherePair = reverse ? scenario.CreatePair(sphere, mesh) : scenario.CreatePair(mesh, sphere);
            CollisionPair capsulePair = reverse ? scenario.CreatePair(capsule, mesh) : scenario.CreatePair(mesh, capsule);
            CollisionDetection.DoCollisionCheck(spherePair).Should().Be(overlap);
            CollisionDetection.DoCollisionCheck(capsulePair).Should().Be(overlap);
            spherePair.Manifold.Count.Should().Be(capsulePair.Manifold.Count);
            if (!overlap) continue;
            spherePair.Manifold.Count.Should().Be(1);
            for (int i = 0; i < spherePair.Manifold.Count; i++)
            {
                ManifoldContact contact = spherePair.Manifold[i];
                ManifoldContact equivalent = capsulePair.Manifold[i];
                contact.Normal.Should().Be(equivalent.Normal);
                contact.Depth.Should().Be(equivalent.Depth);
                contact.PointA.Should().Be(equivalent.PointA);
                contact.PointB.Should().Be(equivalent.PointB);
                if (depthSixteenths >= 0) contact.Depth.Should().Be(Fixed64.FromFraction(depthSixteenths, 16));
                (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
                    .Abs().Should().BeLessThanOrEqualTo(RoundingBound);
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(4)]
    [InlineData(9)]
    public void ReportedExit_ShouldClearTheWholeSphereAtTheFirstTouch(int xSixteenths)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        Vector3d center = Vector3d.Right * Fixed64.FromFraction(xSixteenths, 16);
        var sphere = new LSSphereCollider { Radius = Radius };
        scenario.InitializeStaticCollider(sphere, center);
        CollisionPair pair = scenario.CreatePair(mesh, sphere);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        foreach (int beyondRaw in new[] { -1, 0, 1 })
        {
            var moved = new LSSphereCollider { Radius = Radius };
            scenario.InitializeStaticCollider(moved, center + contact.Normal * (contact.Depth + Fixed64.FromRaw(beyondRaw)));
            CollisionPair movedPair = scenario.CreatePair(mesh, moved);
            CollisionDetection.DoCollisionCheck(movedPair).Should().Be(beyondRaw <= 0);
            if (beyondRaw <= 0) movedPair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromRaw(-beyondRaw));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveClosedCube_ShouldRemainASurface(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = capsule ? new LSCapsuleCollider { Radius = Radius, Size = Vector3d.One * (Radius * Fixed64.Two) }
            : new LSSphereCollider { Radius = Radius };
        scenario.InitializeStaticCollider(curved, Vector3d.Zero);
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, curved)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void TransformedScaledReversedHull_ShouldRetainMatchedWholeSphereExit(bool reverseWinding, bool rotated)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider source = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = source.Mesh.LocalVertices.ToArray();
        int[] indices = source.Mesh.Triangles.ToArray();
        for (int i = 0; i < vertices.Length; i++) vertices[i] += Vector3d.Right * 2;
        if (reverseWinding)
            for (int i = 0; i < indices.Length; i += 3) (indices[i], indices[i + 1]) = (indices[i + 1], indices[i]);
        var mesh = new LSMeshCollider(vertices, indices);
        FixedQuaternion rotation = rotated ? new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity;
        mesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(new Vector3d(3, -2, 4), rotation, new Vector3d(2, 3, 4))));
        Vector3d normal = rotation.Rotate(Vector3d.Right);
        Vector3d center = mesh.Bounds.Center + normal * Fixed64.Half;
        var sphere = new LSSphereCollider { Radius = Radius };
        var capsule = new LSCapsuleCollider { Radius = Radius, Size = Vector3d.One * (Radius * Fixed64.Two) };
        foreach (LSCollider curved in new LSCollider[] { sphere, capsule })
            curved.InitializeWithNoBody(new TestMatterAgent(scenario.Context, new FixedTransform(center, rotation, Vector3d.One)));

        foreach (LSCollider curved in new LSCollider[] { sphere, capsule })
        {
            CollisionPair pair = scenario.CreatePair(curved, mesh);
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            pair.Manifold.Count.Should().Be(1);
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            contact.Normal.Should().Be(normal);
            contact.Depth.Should().Be(Fixed64.FromFraction(5, 8));
            contact.PointA.Should().Be(mesh.Bounds.Center + normal);
            contact.PointB.Should().Be(center - normal * Radius);
            var moved = new LSSphereCollider { Radius = Radius };
            scenario.InitializeStaticCollider(moved, center + normal * (contact.Depth + Fixed64.FromRaw(1)));
            CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, moved)).Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundSphere_ShouldPreserveLeafMaterialAndContactOwnership(bool meshInCompound)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube();
        mesh.Material = PhysicsMaterial.Frictionless;
        LSCollider first = meshInCompound
            ? new LSCompoundCollider(CompoundColliderPart.ConvexMesh(mesh.Mesh.LocalVertices.ToArray(), mesh.Mesh.Triangles.ToArray(),
                Vector3d.Zero, PhysicsMaterial.Frictionless))
            : new LSCompoundCollider(CompoundColliderPart.Sphere(Radius, Vector3d.Zero, PhysicsMaterial.Bouncy));
        LSCollider second = meshInCompound ? new LSSphereCollider { Radius = Radius, Material = PhysicsMaterial.Bouncy } : mesh;
        scenario.InitializeStaticCollider(first, Vector3d.Zero);
        scenario.InitializeStaticCollider(second, Vector3d.Right * Fixed64.Quarter);
        CollisionPair pair = scenario.CreatePair(second, first);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.ColliderA.Should().BeSameAs(first);
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.FeatureNamespaceA.Should().Be(1);
        contact.FeatureNamespaceB.Should().Be(0);
        contact.MaterialA.Should().Be(meshInCompound ? PhysicsMaterial.Frictionless : PhysicsMaterial.Bouncy);
        contact.MaterialB.Should().Be(meshInCompound ? PhysicsMaterial.Bouncy : PhysicsMaterial.Frictionless);
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 8));
        contact.Normal.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FullHostLoop_ShouldCorrectContainmentOutwardAndPreserveEscape(bool capsule, bool escaping)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        var mesh = MeshTestFixtures.CreateConvexCube();
        mesh.Material = PhysicsMaterial.Frictionless;
        SolidBody solid = scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true).Body;
        LSCollider curved = capsule ? new LSCapsuleCollider { Radius = Radius, Size = Vector3d.One * (Radius * Fixed64.Two) }
            : new LSSphereCollider { Radius = Radius };
        curved.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scenario.CreateBody(curved, Vector3d.Right * Fixed64.Quarter,
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        Fixed64 velocity = escaping ? Fixed64.One : -Fixed64.One;
        body.AddLinearImpulse(Vector3d.Right * velocity);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Position3d.X.Should().BeGreaterThan(Fixed64.Quarter);
        body.Position3d.Y.Should().Be(Fixed64.Zero);
        body.Position3d.Z.Should().Be(Fixed64.Zero);
        body.LinearVelocity.X.Should().Be(escaping ? Fixed64.One : Fixed64.Zero);
        solid.Position3d.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void WarmedHullContacts_ShouldAllocateNothing()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube();
        var sphere = new LSSphereCollider { Radius = Radius };
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        scenario.InitializeStaticCollider(sphere, Vector3d.Right * Fixed64.Quarter);
        CollisionPair pair = scenario.CreatePair(mesh, sphere);
        for (int i = 0; i < 32; i++) CollisionDetection.DoCollisionCheck(pair);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++) CollisionDetection.DoCollisionCheck(pair);
        (GC.GetAllocatedBytesForCurrentThread() - before).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueExit_ShouldClearEverySphereCoordinateWithinRoundingBound(bool vertex)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        Fixed64 coordinate = Fixed64.FromFraction(9, 16);
        Vector3d center = new(coordinate, coordinate, vertex ? coordinate : Fixed64.Zero);
        var sphere = new LSSphereCollider { Radius = Radius };
        scenario.InitializeStaticCollider(sphere, center);
        CollisionPair pair = scenario.CreatePair(mesh, sphere);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        foreach (bool beyond in new[] { false, true })
        {
            var moved = new LSSphereCollider { Radius = Radius };
            scenario.InitializeStaticCollider(moved,
                center + contact.Normal * (contact.Depth + (beyond ? RoundingBound : -RoundingBound)));
            CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, moved)).Should().Be(!beyond);
        }
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedSphere_ShouldRetainRelativeAnchorsAtBothDomainLimits(bool maximum)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 limit = maximum ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d normal = maximum ? Vector3d.Right : Vector3d.Left;
        Vector3d center = Vector3d.Right * limit;
        var mesh = MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(mesh, center - normal * Fixed64.Quarter);
        var sphere = new LSSphereCollider { Radius = Radius };
        scenario.InitializeStaticCollider(sphere, center);
        var capsule = new LSCapsuleCollider { Radius = Radius, Size = Vector3d.One * (Radius * Fixed64.Two) };
        scenario.InitializeStaticCollider(capsule, center);
        foreach (LSCollider curved in new LSCollider[] { sphere, capsule })
        {
            CollisionPair pair = scenario.CreatePair(mesh, curved);
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            pair.Manifold.Count.Should().Be(1);
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            contact.Depth.Should().Be(Fixed64.FromFraction(3, 8));
            contact.Normal.Should().Be(normal);
            contact.TryGetPointA(out _).Should().BeFalse();
            contact.TryGetPointB(out Vector3d point).Should().BeTrue();
            point.Should().Be(center - normal * Radius);
            contact.AnchorA.TryGetOffsetFrom(contact.AnchorB, out Vector3d offset).Should().BeTrue();
            offset.Should().Be(normal * contact.Depth);
        }
    }

}
