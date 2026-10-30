using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexMeshCylinderContactTests
{
    [Fact]
    public void ContainedCylinderSweep_ReportsTheExistingInitialOverlap()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateCube(MeshColliderMode.Convex);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cylinder = new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half };
        scene.CreateBody(cylinder, Vector3d.Zero, FixedQuaternion.Identity);
        foreach (Vector3d travel in new[] { Vector3d.Up, Vector3d.Down, Vector3d.Right, Vector3d.Left })
        {
            Assert.True(scene.Context.Query3D.SweepCylinder(cylinder, travel, PhysicsLayerMask.All, out var hit));
            Assert.Same(mesh, hit.Collider);
            Assert.Equal(Fixed64.Zero, hit.Distance);
        }
    }

    [Fact]
    public void RepeatedHostSteps_KeepMovingTheContainedCylinderOutward()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateCube(MeshColliderMode.Convex);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        SolidBody body = scene.CreateBody(new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half },
            Vector3d.Zero, FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.SleepEnabled = false; body.UseManualGrounding();
        Vector3d direction = default;
        Fixed64 previous = Fixed64.Zero;
        for (int step = 0; step < 16; step++)
        {
            scene.Context.Simulate(); scene.Context.LateSimulate();
            if (step == 0) direction = body.Position3d.Normalized;
            Fixed64 progress = Vector3d.Dot(body.Position3d, direction);
            Assert.True(progress >= previous, $"step {step}: {progress} < {previous}");
            previous = progress;
        }
        Assert.True(previous > (Fixed64)1.2m);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, true, true)]
    public void ContainedAndBoundaryCenters_PreserveScaledWitnessesAndReversal(bool scaledMesh, bool scaledCylinder, bool boundary)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateCube(MeshColliderMode.Convex);
        Vector3d origin = new(2, 1, -3);
        Vector3d meshScale = scaledMesh ? Vector3d.One * (Fixed64)1.5m : Vector3d.One;
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(origin, FixedQuaternion.Identity, meshScale)));
        var cylinder = new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half };
        Vector3d center = origin + Vector3d.Right * (boundary ? meshScale.X : Fixed64.Quarter);
        Vector3d cylinderScale = scaledCylinder ? Vector3d.One * Fixed64.Two : Vector3d.One;
        cylinder.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(center, FixedQuaternion.Identity, cylinderScale)));
        foreach (bool reverse in new[] { false, true })
        {
            CollisionPair pair = reverse ? scene.CreatePair(cylinder, mesh) : scene.CreatePair(mesh, cylinder);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            Fixed64 expected = meshScale.X - (boundary ? meshScale.X : Fixed64.Quarter) + cylinder.ScaledRadius;
            Assert.Equal(expected, contact.Depth);
            Assert.True(contact.AnchorA.TryGetOffsetFrom(contact.AnchorB, out Vector3d offset));
            Assert.Equal(contact.Normal * contact.Depth, offset);
            Vector3d normal = ReferenceEquals(pair.ColliderB, cylinder) ? contact.Normal : -contact.Normal;
            Assert.Equal(Vector3d.Right, normal);
            foreach (int raw in new[] { -1, 0, 1 })
            {
                var moved = new LSCylinderCollider { Radius = cylinder.Radius, Size = cylinder.Size };
                moved.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(center + normal * (contact.Depth + Fixed64.FromRaw(raw)), FixedQuaternion.Identity, cylinderScale)));
                CollisionPair movedPair = scene.CreatePair(mesh, moved);
                Assert.Equal(raw <= 0, CollisionDetection.DoCollisionCheck(movedPair));
                if (raw <= 0) Assert.Equal(Fixed64.FromRaw(-raw), movedPair.Manifold.PrimaryContact.Depth);
            }
        }
    }

    [Fact]
    public void WarmContainmentHostSteps_AllocateNothingAndReplayIdentically()
    {
        using var first = PhysicsScenarioBuilder.Create();
        using var second = PhysicsScenarioBuilder.Create();
        SolidBody a = Create(first), b = Create(second);
        for (int i = 0; i < 16; i++)
        {
            Step(first, a); Step(second, b);
            Assert.Equal(first.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches),
                second.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
            Assert.NotEqual(Vector3d.Zero, a.Position3d);
        }
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(() => Step(first, a), warmupIterations: 16));

        static SolidBody Create(PhysicsScenarioBuilder scene)
        {
            scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
                scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            scene.InitializeStaticCollider(CreateCube(MeshColliderMode.Convex), Vector3d.Zero);
            SolidBody body = scene.CreateBody(new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half },
                Vector3d.Zero, FixedQuaternion.Identity, preventAngularForces: true).Body;
            body.SleepEnabled = false; body.UseManualGrounding();
            return body;
        }
        static void Step(PhysicsScenarioBuilder scene, SolidBody body)
        {
            body.SetPosition(Vector3d.Zero);
            scene.Context.Simulate(); scene.Context.LateSimulate();
        }
    }
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, false, true)]
    [InlineData(true, false, true)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    public void CompletedHostStep_ResolvesConvexContainmentAndExteriorOverlap(bool cylinder, bool surface, bool exterior)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateCube(surface ? MeshColliderMode.Concave : MeshColliderMode.Convex);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider mover = cylinder
            ? new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half }
            : new LSSphereCollider { Radius = Fixed64.Quarter };
        mover.Material = PhysicsMaterial.Frictionless;
        Vector3d start = exterior ? Vector3d.Right * (Fixed64)1.125m : Vector3d.Zero;
        SolidBody body = scene.CreateBody(mover, start, FixedQuaternion.Identity,
            preventAngularForces: true).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();

        scene.Context.Simulate(); scene.Context.LateSimulate();

        if (surface) Assert.Equal(start, body.Position3d);
        else Assert.NotEqual(start, body.Position3d);
    }

    private static LSMeshCollider CreateCube(MeshColliderMode mode)
    {
        LSMeshCollider fixture = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = fixture.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++) vertices[i] *= Fixed64.Two;
        return new LSMeshCollider(vertices, fixture.Mesh.Triangles.ToArray(), mode,
            MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
    }
}
