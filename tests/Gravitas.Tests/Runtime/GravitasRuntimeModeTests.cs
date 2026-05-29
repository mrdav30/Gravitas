using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasRuntimeModeTests
{
    [Fact]
    public void LateSimulate_WithTwoDMode_ShouldSkipThreeDBodiesAndRunTwoDBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        StiffBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        body2D.Position.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void LateSimulate_WithThreeDMode_ShouldSkipTwoDBodiesAndRunThreeDBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        StiffBody2D body2D = Create2DBody(scenario.Context, Vector3d.Zero);

        body3D.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body2D.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        scenario.Context.FrameCount.Should().Be(1);
        body3D.Body.Position3d.x.Should().Be(Fixed64.Fraction(1, 4));
        body2D.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Visualize_WithTwoDMode_ShouldSkipThreeDVisualInterpolationButKeepHooks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        FixedTransform transform = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var body = new StiffBody(agent, new LSSphereCollider())
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

    private static StiffBody2D Create2DBody(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position.ToVector2d());
        return body;
    }
}
