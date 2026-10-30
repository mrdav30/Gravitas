using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexMeshCurvedSlabContainmentTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void ContinuousHostStep_AllowsEscapeAndStopsDeeperTravel(int shape, bool escaping)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        scene.Context.SetFrameRate(1);
        scene.InitializeStaticCollider(Cube(), Vector3d.Zero);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), Slab(shape))
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        Vector2d start = Vector2d.Right * Fixed64.Half;
        body.Initialize(start);
        body.UseManualGrounding();
        Vector2d velocity = Vector2d.Right * (escaping ? Fixed64.Two : -Fixed64.Two);
        body.AddLinearImpulse(velocity);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(escaping ? velocity : Vector2d.Zero, body.LinearVelocity);
        if (escaping) Assert.Equal(start + velocity, body.Position);
        else Assert.True(body.Position.X >= start.X);
        Assert.Equal(escaping ? 0 : 1, body.LastContinuousCollisionToiIterationCount);
        Assert.Equal(Fixed64.Zero, body.Agent.Transform.WorldPosition.Y);
    }

    [Fact]
    public void ContainedCompoundLeaves_RetainTheirNamespacesAndMaterials()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube();
        mesh.Material = PhysicsMaterial.Default;
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var slab = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Quarter, -Vector2d.Right * Fixed64.Half, PhysicsMaterial.Bouncy),
            CompoundColliderPart2D.Capsule(Fixed64.Quarter, Fixed64.One, Vector2d.Right * Fixed64.Half, PhysicsMaterial.Frictionless))
        { MixedHalfThicknessOverride = Fixed64.Quarter };
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, slab, manifold));
        Assert.Equal(2, manifold.GroupCount);
        for (int group = 0; group < manifold.GroupCount; group++)
        {
            ref var contacts = ref manifold.GetGroup(group);
            Assert.Equal(0, contacts.Key.NamespaceA);
            int part = -contacts.Key.NamespaceB - 1;
            Assert.InRange(part, 0, 1);
            Assert.Equal(1, contacts.Count);
            Assert.Equal(mesh.Material, contacts[0].MaterialA);
            Assert.Equal(slab.GetPartCollider(part).Material, contacts[0].MaterialB);
        }
    }

    [Theory]
    [InlineData(0, false, false)]
    [InlineData(0, true, false)]
    [InlineData(0, false, true)]
    [InlineData(0, true, true)]
    [InlineData(2, false, false)]
    [InlineData(2, true, false)]
    [InlineData(2, false, true)]
    [InlineData(2, true, true)]
    public void ScaledAndBoundaryCenters_PreserveFiniteAnchorsAndMaterials(int shape, bool scaled, bool boundary)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube();
        Vector3d origin = new(2, 1, -3);
        Vector3d scale = scaled ? Vector3d.One * (Fixed64)1.5m : Vector3d.One;
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(origin, FixedQuaternion.Identity, scale)));
        Vector3d center = origin + Vector3d.Right * (boundary ? scale.X : Fixed64.Half);
        LSCollider2D slab = Slab(shape);
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(center,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -(Fixed64).4m), scaled ? Vector3d.One * Fixed64.Two : Vector3d.One)));
        Assert.True(CollisionDetectionMixed.TryCollide(mesh, slab, out MixedContact contact));
        Assert.True(contact.Anchor3D.TryGetOffsetFrom(contact.Anchor2D, out Vector3d offset));
        Assert.True((offset - contact.Normal3DTo2D * contact.Depth).Magnitude <= Fixed64.FromRaw(64));
        Assert.Equal(Vector3d.Right, contact.Normal3DTo2D);
        Assert.Equal(mesh.Material, contact.Material3D);
        Assert.Equal(slab.Material, contact.Material2D);
        Assert.True(FixedMath.Abs(contact.Point3D.X - origin.X - scale.X) <= Fixed64.FromRaw(64));
        Vector2d point = new(contact.Point2D.X, contact.Point2D.Z);
        Assert.True(slab.TryGetClosestBoundaryAnchor(point, out var anchor));
        Assert.True(anchor.TryGetPoint(out Vector2d nearest));
        Assert.True((nearest - point).Magnitude <= Fixed64.FromRaw(64));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircleSweep_ReportsInitialHullContainment(bool scaled)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube();
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity, scaled ? Vector3d.One * (Fixed64)1.5m : Vector3d.One)));
        foreach (Vector2d end in new[] { Vector2d.Right, -Vector2d.Right, Vector2d.Forward, -Vector2d.Forward })
        {
            Assert.True(scene.Context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, end, Fixed64.Quarter, Fixed64.Zero,
                Fixed64.Quarter, PhysicsLayerMask.All, out var hit));
            Assert.Same(mesh, hit.Collider3D);
            Assert.Equal(Fixed64.Zero, hit.Distance);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ContinuousAdmission_IncludesContainedCurvedSlabs(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube();
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider2D slab = Slab(shape);
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, slab));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ImmobileVerticalMinimum_PreservesTheMixedResponseConstraint(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube();
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider2D slab = Slab(shape);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), slab)
        { Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes2D.Rotation };
        body.Initialize(Vector2d.Zero);
        body.UseManualGrounding();
        Assert.True(CollisionDetectionMixed.TryCollide(mesh, slab, out MixedContact contact));
        Assert.True(FixedMath.Abs(contact.Normal3DTo2D.Y) == Fixed64.One);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(Vector2d.Zero, body.Position);
        Assert.True(CollisionDetectionMixed.TryCollide(mesh, slab, out _));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void ContainedCurvedSlab_UsesFilledConvexVolume(int shape, bool exterior)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = Cube();
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider2D slab = Slab(shape);
        Vector3d center = exterior ? Vector3d.Right * (Fixed64)1.125m : Vector3d.Zero;
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(center, FixedQuaternion.Identity, Vector3d.One)));

        Assert.True(CollisionDetectionMixed.TryCollide(mesh, slab, out MixedContact contact));
        Assert.True(contact.Depth > Fixed64.Zero);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void CompletedMixedSteps_MoveAContainedPlanarBodyOutward(int shape, bool yawed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        scene.InitializeStaticCollider(Cube(), Vector3d.Zero);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), Slab(shape))
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes2D.Rotation
        };
        Vector2d start = Vector2d.Right * Fixed64.Half;
        body.Initialize(start, yawed ? (Fixed64).4m : Fixed64.Zero);
        body.UseManualGrounding();
        Fixed64 previous = start.X;
        for (int step = 0; step < 16; step++)
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
            Assert.True(body.Position.X >= previous);
            Assert.Equal(start.Y, body.Position.Y);
            previous = body.Position.X;
        }
        Assert.True(previous > (Fixed64)1.2m);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void CompletedMixedSteps_MoveThe3DBodyAlongAVerticalExit(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        SolidBody body = scene.CreateBody(Cube(), Vector3d.Zero, FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        LSCollider2D slab = Slab(shape);
        slab.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 8);
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        for (int step = 0; step < 16; step++)
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
        }
        Assert.Equal(Fixed64.Zero, body.Position3d.X);
        Assert.Equal(Fixed64.Zero, body.Position3d.Z);
        Assert.True(FixedMath.Abs(body.Position3d.Y) > (Fixed64)1.1m);
        Assert.Equal(Vector2d.Zero, slab.Center);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ConcaveClosedShell_PreservesItsSurfaceMeaning(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider mesh = Cube(MeshColliderMode.Concave);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider2D slab = Slab(shape);
        slab.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        Assert.False(CollisionDetectionMixed.TryCollide(mesh, slab, out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void WarmContainedSteps_AllocateNothingAndReplayIdentically(int shape)
    {
        using var first = PhysicsScenarioBuilder.Create();
        using var second = PhysicsScenarioBuilder.Create();
        SolidBody2D a = Create(first), b = Create(second);
        for (int i = 0; i < 16; i++)
        {
            Step(first, a); Step(second, b);
            Assert.Equal(first.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches),
                second.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
            Assert.True(a.Position.X > Fixed64.Half);
        }
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(() => Step(first, a), warmupIterations: 16));

        SolidBody2D Create(PhysicsScenarioBuilder scene)
        {
            Configure(scene.Context);
            scene.InitializeStaticCollider(Cube(), Vector3d.Zero);
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), Slab(shape))
            { Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes2D.Rotation };
            body.Initialize(Vector2d.Right * Fixed64.Half, (Fixed64).4m);
            body.UseManualGrounding();
            return body;
        }
        static void Step(PhysicsScenarioBuilder scene, SolidBody2D body)
        {
            body.SetPosition(Vector2d.Right * Fixed64.Half);
            scene.Context.Simulate(); scene.Context.LateSimulate();
        }
    }

    private static LSCollider2D Slab(int shape)
    {
        LSCollider2D slab = shape == 0 ? new LSCircleCollider2D(Fixed64.Quarter)
            : new LSCapsuleCollider2D(Fixed64.Quarter, shape == 1 ? Fixed64.Half : Fixed64.One);
        slab.MixedHalfThicknessOverride = Fixed64.Quarter;
        slab.Material = PhysicsMaterial.Frictionless;
        return slab;
    }

    private static LSMeshCollider Cube(MeshColliderMode mode = MeshColliderMode.Convex)
    {
        LSMeshCollider fixture = MeshTestFixtures.CreateConvexCube();
        var vertices = new Vector3d[fixture.Mesh.GeometryVertices.Length];
        for (int i = 0; i < vertices.Length; i++) vertices[i] = fixture.Mesh.GeometryVertices[i] * Fixed64.Two;
        return new LSMeshCollider(vertices, fixture.Mesh.Triangles.ToArray(), mode, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
    }

    private static void Configure(GravitasWorldContext context)
    {
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(16);
        context.Environment.Gravity = context.Environment.AirDensity =
            context.Environment.DampingFactor = context.Environment.MinSpeed = Fixed64.Zero;
    }
}
