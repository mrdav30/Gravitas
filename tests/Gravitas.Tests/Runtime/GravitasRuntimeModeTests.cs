using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasRuntimeModeTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.Both)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void RuntimeMode_WithValidBitmaskMode_ShouldAcceptValue(PhysicsRuntimeMode mode)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.Settings.RuntimeMode = mode;

        context.Settings.RuntimeMode.Should().Be(mode);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.None)]
    [InlineData((PhysicsRuntimeMode)4)]
    [InlineData((PhysicsRuntimeMode)5)]
    [InlineData((PhysicsRuntimeMode)6)]
    [InlineData((PhysicsRuntimeMode)byte.MaxValue)]
    public void RuntimeMode_WithInvalidBitmaskMode_ShouldThrow(PhysicsRuntimeMode mode)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        Action setMode = () => context.Settings.RuntimeMode = mode;

        setMode.Should()
            .Throw<ArgumentException>()
            .WithParameterName("value")
            .WithMessage("*Physics runtime mode*");
    }

    [Fact]
    public void LateSimulate_WithTwoDMode_ShouldSkipThreeDBodiesAndRunTwoDBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        SolidBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        body2D.Position.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void LateSimulate_WithBothMode_ShouldRunTwoDAndThreeDBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        SolidBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        body3D.Body.Position3d.X.Should().Be(Fixed64.FromFraction(1, 4));
        body2D.Position.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        scenario.Context.MixedCollisions.SimulateCount.Should().Be(0);
        scenario.Context.MixedCollisions.LateSimulateCount.Should().Be(0);
    }

    [Fact]
    public void LateSimulate_WithBothServicesDisabled_ShouldAdvanceContextOnly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);
        int lateSimulateHooks = 0;
        using IDisposable hook = scenario.Context.RegisterOnLateSimulate(
            "RuntimeMode.DisabledServices",
            0,
            () => lateSimulateHooks++);
        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body3D.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Zero,
            Vector3d.Right,
            scenario.Context.DeltaTime);
        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(1);
        scenario.Context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body2D.ApplyContinuousCollisionHandoff(
            Vector2d.Zero,
            Vector2d.Right,
            scenario.Context.DeltaTime);
        scenario.Context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(1);
        Vector3d position3D = body3D.Body.Position3d;
        Vector2d position2D = body2D.Position;
        scenario.Context.Physics.SimulatePhysics = false;
        scenario.Context.Physics2D.SimulatePhysics = false;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        scenario.Context.ResetAccumulation.Should().BeTrue();
        lateSimulateHooks.Should().Be(1);
        body3D.Body.Position3d.Should().Be(position3D);
        body2D.Position.Should().Be(position2D);
        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
        scenario.Context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        scenario.Context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void RuntimePhases_WithMixedMode_ShouldRunTwoDThreeDAndMixedLifecycle()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        SolidBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();
        scenario.Context.LateVisualize();

        body3D.Body.Position3d.X.Should().Be(Fixed64.FromFraction(1, 4));
        body2D.Position.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        scenario.Context.MixedCollisions.SimulateCount.Should().Be(1);
        scenario.Context.MixedCollisions.LateSimulateCount.Should().Be(1);
        scenario.Context.MixedCollisions.VisualizeCount.Should().Be(1);

        scenario.Context.Reset();

        scenario.Context.MixedCollisions.SimulateCount.Should().Be(0);
        scenario.Context.MixedCollisions.LateSimulateCount.Should().Be(0);
        scenario.Context.MixedCollisions.VisualizeCount.Should().Be(0);
    }

    [Fact]
    public void LateVisualize_ShouldRemainContextHookOnly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        int lateVisualizeHooks = 0;
        using IDisposable hook = scenario.Context.RegisterOnLateVisualize(
            "RuntimeMode.LateVisualize",
            0,
            () => lateVisualizeHooks++);

        scenario.Context.LateVisualize();

        lateVisualizeHooks.Should().Be(1);
        typeof(GravitasPhysicsService).GetMethod("LateVisualize").Should().BeNull();
        typeof(GravitasPhysics2DService).GetMethod("LateVisualize").Should().BeNull();
        typeof(SolidBody).GetMethod("LateVisualize").Should().BeNull();
        typeof(SolidBody2D)
            .GetMethod("LateVisualize", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .Should()
            .BeNull();
        typeof(GravitasMixedCollisionService)
            .GetMethod("LateVisualize", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .Should()
            .BeNull();
    }

    [Fact]
    public void LateSimulate_WithThreeDMode_ShouldSkipTwoDBodiesAndRunThreeDBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        SolidBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        body3D.Body.Position3d.X.Should().Be(Fixed64.FromFraction(1, 4));
        body2D.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Visualize_WithTwoDMode_ShouldSkipThreeDVisualInterpolationButKeepHooks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        FixedTransform transform = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var body = new SolidBody(agent, new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        int visualizeHooks = 0;
        using IDisposable hook = scenario.Context.RegisterOnVisualize("test", 0, () => visualizeHooks++);

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        scenario.Context.LateSimulate();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        scenario.Context.Visualize();

        visualizeHooks.Should().Be(1);
        transform.Position.Should().Be(Vector3d.Zero);
    }

    private static SolidBody2D Create2DBody(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position.ToVector2d());
        return body;
    }
}
