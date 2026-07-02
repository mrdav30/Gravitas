using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class GravitasReplayConformanceTests
{
    public static TheoryData<GravitasSerializationTransport> Transports =>
        GravitasSerializationTransportCases.All();

    [Fact]
    public void ReplayHashTrace_ShouldMatchForRepeated3DContinuousCollisionRuns()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            Create3DContinuousCollisionReplayScenario,
            frameCount: 10);
    }

    [Fact]
    public void ReplayHashTrace_ShouldMatchForRepeated3DMeshAndCompoundRuns()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            Create3DMeshAndCompoundReplayScenario,
            frameCount: 8);
    }

    [Fact]
    public void ReplayHashTrace_ShouldMatchForRepeatedPure2DRuns()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            CreatePure2DReplayScenario,
            frameCount: 12);
    }

    [Fact]
    public void ReplayHashTrace_ShouldMatchForRepeatedMixedRunsWithQueryCacheMutation()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            CreateMixedReplayScenario,
            frameCount: 10,
            beforeFrame: static (context, frame) =>
            {
                if ((frame & 1) == 0)
                {
                    _ = context.QueryMixed.SweepSphereAgainst2D(
                        new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
                        new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
                        Fixed64.Half,
                        PhysicsLayerMask.All,
                        out _);
                }
            });
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void ReplayHashTrace_ShouldMatchAfterChroniclerRestore3D(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = Create3DScenario();
        ScenarioBody<LSCuboidCollider> source = sourceScenario.CreateCuboid(
            Vector3d.Zero,
            mass: (Fixed64)3);

        using PhysicsScenarioBuilder restoredScenario = Create3DScenario();
        ScenarioBody<LSCuboidCollider> restored = restoredScenario.CreateCuboid(
            Vector3d.Zero,
            mass: Fixed64.One);

        source.Body.AddForce(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        restored.Body.AddForce(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        AdvanceBoth(sourceScenario.Context, restoredScenario.Context, frameCount: 4);

        source.Body.AddTorque(new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        object payload = GravitasSerializationHarness.Serialize(source.Body, transport);
        restored.Body.AddForce(Vector3d.Left);
        GravitasSerializationHarness.Populate(restored.Body, payload, transport);

        ReplayConformanceHarness.AssertNextFramesMatch(
            sourceScenario.Context,
            restoredScenario.Context,
            frameCount: 16);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void ReplayHashTrace_ShouldMatchAfterChroniclerRestore2D(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = CreatePure2DReplayScenario();
        SolidBody2D source = GetBody2D(sourceContext, colliderId: 1);

        using GravitasWorldContext restoredContext = CreatePure2DReplayScenario();
        SolidBody2D restored = GetBody2D(restoredContext, colliderId: 1);

        AdvanceBoth(sourceContext, restoredContext, frameCount: 4);

        source.AddTorque((Fixed64)2);
        object payload = GravitasSerializationHarness.Serialize(source, transport);
        restored.AddForce(Vector2d.Left);
        GravitasSerializationHarness.Populate(restored, payload, transport);

        ReplayConformanceHarness.AssertNextFramesMatch(
            sourceContext,
            restoredContext,
            frameCount: 16);
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario3D = Create3DScenario();
        scenario3D.CreateSphere(Vector3d.Zero);
        scenario3D.Context.LateSimulate();

        using PhysicsScenarioBuilder dense3D = Create3DScenario();
        dense3D.CreateSphere(Vector3d.Zero);
        dense3D.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        dense3D.Context.Simulate();

        using GravitasWorldContext context2D = CreatePure2DReplayScenario();
        context2D.LateSimulate();

        using GravitasWorldContext mixedContext = CreateMixedReplayScenario();
        mixedContext.LateSimulate();

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                _ = scenario3D.Context.ComputeReplayHash();
                _ = scenario3D.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
                _ = dense3D.Context.ComputeReplayHash();
                _ = dense3D.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
                _ = context2D.ComputeReplayHash();
                _ = context2D.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
                _ = mixedContext.ComputeReplayHash();
                _ = mixedContext.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
            },
            warmupIterations: 16,
            stabilizationIterations: 8,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    private static PhysicsScenarioBuilder Create3DScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(8);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }

    private static GravitasWorldContext Create3DContinuousCollisionReplayScenario()
    {
        PhysicsScenarioBuilder scenario = Create3DScenario();

        var wall = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.FromFraction(1, 8), (Fixed64)4, (Fixed64)4)
        };
        scenario.InitializeStaticCollider(wall, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.One);
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);

        return scenario.Context;
    }

    private static GravitasWorldContext Create3DMeshAndCompoundReplayScenario()
    {
        PhysicsScenarioBuilder scenario = Create3DScenario();

        Vector3d[] vertices =
        {
            new(-Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
            new(Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
            new(-Fixed64.Half, Fixed64.Half, Fixed64.Zero),
            new(Fixed64.Half, Fixed64.Half, Fixed64.Zero)
        };
        int[] triangles = { 0, 2, 1, 1, 2, 3 };
        var mesh = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Cuboid(
                Vector3d.One,
                new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            mass: (Fixed64)2);
        body.Body.AddForce(Vector3d.Right * (Fixed64)2);
        body.Body.AddTorque(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        return scenario.Context;
    }

    private static GravitasWorldContext CreatePure2DReplayScenario()
    {
        GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 8);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D mover = CreateCircle2D(context, Vector2d.Zero);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.AddForce(new Vector2d((Fixed64)5, Fixed64.Zero));
        mover.AddTorque(Fixed64.One);
        _ = CreateBox2D(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        return context;
    }

    private static GravitasWorldContext CreateMixedReplayScenario()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(8);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        EnsureGrid(context, extent: 24);

        var agent3D = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var body3D = new SolidBody(agent3D, new LSSphereCollider())
        {
            Mass = (Fixed64)2,
            FreezeAxes = BodyFreezeAxes3D.Position
        };
        body3D.Initialize(agent3D.Transform.Position, FixedQuaternion.Identity);

        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)(-1), Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)6, Fixed64.Zero));
        return context;
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = (Fixed64)2,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateBox2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSAABBoxCollider2D(Vector2d.One))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D GetBody2D(GravitasWorldContext context, int colliderId)
    {
        context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
            .Should()
            .BeTrue();
        collider!.Body.Should().NotBeNull();
        return collider.Body!;
    }

    private static void AdvanceBoth(
        GravitasWorldContext first,
        GravitasWorldContext second,
        int frameCount)
    {
        for (int frame = 0; frame < frameCount; frame++)
        {
            first.LateSimulate();
            second.LateSimulate();
        }
    }

    private static void EnsureGrid(GravitasWorldContext context, int extent)
    {
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-extent), (Fixed64)(-extent), (Fixed64)(-extent)),
                new Vector3d((Fixed64)extent, (Fixed64)extent, (Fixed64)extent)),
            out _).Should().BeTrue();
    }
}
