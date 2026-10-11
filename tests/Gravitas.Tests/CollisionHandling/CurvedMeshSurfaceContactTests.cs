using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CurvedMeshSurfaceContactTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, true, true)]
    public void InteriorFootprint_ShouldRetainOnlySurfaceNormals(bool capsule, bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().BeGreaterThan(0);
        for (int i = 0; i < pair.Manifold.Count; i++)
            pair.Manifold[i].Normal.Should().Be(Vector3d.Down);

        CollisionPair reversed = scenario.CreatePair(curved, mesh);
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        reversed.Manifold.Count.Should().Be(pair.Manifold.Count);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            reversed.Manifold[i].Normal.Should().Be(pair.Manifold[i].Normal);
            reversed.Manifold[i].ContactId.Should().Be(pair.Manifold[i].ContactId);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FrictionlessFullLoop_ShouldPreserveMotionAcrossCoveredDiagonal(bool capsule, bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateQuad(reverse, false);
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        LSCollider curved = CreateShape(capsule);
        curved.Material = PhysicsMaterial.Frictionless;
        var body = scenario.CreateBody(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)),
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        body.AddLinearImpulse(new(-Fixed64.One, Fixed64.Half, -Fixed64.One));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.LinearVelocity.X.Should().Be(-Fixed64.One);
        body.LinearVelocity.Z.Should().Be(-Fixed64.One);
        body.LinearVelocity.Y.Should().BeLessThanOrEqualTo(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExposedEdge_ShouldRetainRadialConstraint(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(false, false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new((Fixed64)2 + Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void WorldChart_ShouldRejectCoveredSeamWithoutNarrowingRelativeCenter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[] { new Vector3d(4, -2, -2), new Vector3d(4, 2, -2),
            new Vector3d(4, -2, 2), new Vector3d(4, 2, 2), new Vector3d(-4, -2, -2), new Vector3d(-4, 2, -2),
            new Vector3d(-4, -2, 2), new Vector3d(-4, 2, 2) }, new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));
        var curved = new LSSphereCollider { Radius = Fixed64.MaxValue - Fixed64.One };
        scenario.InitializeStaticCollider(curved, new(Fixed64.Zero, Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        for (int i = 0; i < pair.Manifold.Count; i++) pair.Manifold[i].Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void WorldChart_WithUnrepresentableNeighborVertex_ShouldPreserveAdmittedContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[] { new Vector3d(0, 0, -2), new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2), new Vector3d(0, -2, 2) }, new[] { 0, 1, 2, 0, 2, 3 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero));
        var sphere = new LSSphereCollider { Radius = Fixed64.MaxValue };
        scenario.InitializeStaticCollider(sphere, new(Fixed64.Quarter, Fixed64.Zero, (Fixed64)3));
        CollisionPair pair = scenario.CreatePair(mesh, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().BeGreaterThan(0);
        pair.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CompoundPart_ShouldRemapSurfaceNormalAndMaterials(bool capsule, bool reverseArguments)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(false, true);
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        CompoundColliderPart part = capsule
            ? CompoundColliderPart.Capsule(Fixed64.Half, (Fixed64)2, Vector3d.Zero, PhysicsMaterial.Bouncy)
            : CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, PhysicsMaterial.Bouncy);
        var compound = new LSCompoundCollider(part);
        scenario.InitializeStaticCollider(compound, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = reverseArguments ? scenario.CreatePair(mesh, compound) : scenario.CreatePair(compound, mesh);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.ColliderA.Should().BeSameAs(compound);
        pair.Manifold.GroupCount.Should().Be(1);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.Should().Be(Vector3d.Up);
            contact.FeatureNamespaceA.Should().Be(1);
            contact.FeatureNamespaceB.Should().Be(0);
            contact.MaterialA.Should().Be(PhysicsMaterial.Bouncy);
            contact.MaterialB.Should().Be(PhysicsMaterial.Frictionless);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatedCrossingCore_ShouldOrientNormalAndRadialAnchorTogether(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)180);
        Vector3d origin = new(4, 3, 2);
        LSMeshCollider mesh = CreateQuad(reverse, false);
        scenario.CreateBody(mesh, origin, rotation, immovable: true);
        LSCollider capsule = CreateShape(true);
        Vector3d localCenter = new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10));
        scenario.CreateBody(capsule, origin + rotation.Rotate(localCenter), rotation, immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.Should().Be(Vector3d.Up);
            contact.PointA.Y.Should().Be((Fixed64)3);
            contact.PointB.Y.Should().Be((Fixed64)3 - Fixed64.Half);
            contact.Depth.Should().Be(Fixed64.Half);
        }
    }

    [Theory]
    [InlineData(false, -1, -1)]
    [InlineData(false, -1, 1)]
    [InlineData(false, 1, -1)]
    [InlineData(false, 1, 1)]
    [InlineData(true, -1, -1)]
    [InlineData(true, -1, 1)]
    [InlineData(true, 1, -1)]
    [InlineData(true, 1, 1)]
    public void ExposedCorner_ShouldRetainBothRadialComponents(bool capsule, int x, int z)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(false, false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new((Fixed64)x * ((Fixed64)2 + Fixed64.Quarter),
            -Fixed64.Quarter, (Fixed64)z * ((Fixed64)2 + Fixed64.Quarter)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        (pair.Manifold.PrimaryContact.Normal.X * (Fixed64)x).Should().BeGreaterThan(Fixed64.Zero);
        (pair.Manifold.PrimaryContact.Normal.Z * (Fixed64)z).Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WarmedSurfaceReduction_ShouldAllocateNothing(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(false, true);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);
        for (int i = 0; i < 32; i++) CollisionDetection.DoCollisionCheck(pair);
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++) CollisionDetection.DoCollisionCheck(pair);
        (System.GC.GetAllocatedBytesForCurrentThread() - before).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleBoundary_ShouldRemainExposed(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half), new(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Zero, Fixed64.Half), new(-Fixed64.Half, Fixed64.Zero, Fixed64.Half) };
        var mesh = new LSMeshCollider(vertices, new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool left = false;
        for (int i = 0; i < pair.Manifold.Count; i++) left |= pair.Manifold[i].Normal.X < Fixed64.Zero;
        left.Should().BeTrue("the inner rim is a real boundary, not a covered seam");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentSurfaces_ShouldRetainMoreThanFourConstraints(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices = new Vector3d[18];
        int[] indices = new int[18];
        Vector3d[] normals = { Vector3d.Right, Vector3d.Left, Vector3d.Up, Vector3d.Down, Vector3d.Forward, -Vector3d.Forward };
        for (int face = 0; face < 6; face++)
        {
            Vector3d normal = normals[face];
            Vector3d tangent = face < 2 ? Vector3d.Up : Vector3d.Right;
            Vector3d other = Vector3d.Cross(normal, tangent);
            Vector3d center = normal * Fixed64.Quarter;
            vertices[face * 3] = center - tangent * (Fixed64)2 - other * (Fixed64)2;
            vertices[face * 3 + 1] = center + tangent * (Fixed64)2 - other * (Fixed64)2;
            vertices[face * 3 + 2] = center + other * (Fixed64)2;
            for (int i = 0; i < 3; i++) indices[face * 3 + i] = face * 3 + i;
        }
        var mesh = new LSMeshCollider(vertices, indices, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.GroupCount.Should().Be(6);
        foreach (Vector3d normal in normals)
        {
            bool found = false;
            for (int i = 0; i < pair.Manifold.Count; i++) found |= pair.Manifold[i].Normal == normal;
            found.Should().BeTrue("every independent face must survive reduction");
        }
    }

    private static LSCollider CreateShape(bool capsule) => capsule
        ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
        : new LSSphereCollider { Radius = Fixed64.Half };

    private static LSMeshCollider CreateQuad(bool reverse, bool subdivide)
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(-2, 0, 2), new(2, 0, 2), Vector3d.Zero };
        if (!subdivide) System.Array.Resize(ref vertices, 4);
        int[] triangles = subdivide ? new[] { 0, 2, 4, 2, 3, 4, 3, 1, 4, 1, 0, 4 } : new[] { 0, 2, 1, 1, 2, 3 };
        if (reverse)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i], triangles[i + 2]) = (triangles[i + 2], triangles[i]);
        return new(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
