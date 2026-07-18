using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyImpulseContractTests
{
    [Fact]
    public void AddLinearImpulse_ShouldApplyImmediateFrameRateInvariantVelocityWithoutAdvancingPose()
    {
        using PhysicsScenarioBuilder slowScenario = Create3DScenario(frameRate: 10);
        using PhysicsScenarioBuilder fastScenario = Create3DScenario(frameRate: 100);
        ScenarioBody<LSSphereCollider> slow = slowScenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        ScenarioBody<LSSphereCollider> fast = fastScenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        Vector3d impulse = new((Fixed64)6, (Fixed64)4, (Fixed64)(-2));
        Vector3d expectedVelocity = impulse * slow.Body.InverseMass;

        slow.Body.AddLinearImpulse(impulse);
        fast.Body.AddLinearImpulse(impulse);

        slow.Body.LinearVelocity.Should().Be(expectedVelocity);
        fast.Body.LinearVelocity.Should().Be(expectedVelocity);
        slow.Body.Position3d.Should().Be(Vector3d.Zero);
        fast.Body.Position3d.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void AddAngularImpulse_ShouldApplyImmediateFrameRateInvariantVelocityWithoutAdvancingPose()
    {
        using PhysicsScenarioBuilder slowScenario = Create3DScenario(frameRate: 10);
        using PhysicsScenarioBuilder fastScenario = Create3DScenario(frameRate: 100);
        ScenarioBody<LSCuboidCollider> slow = slowScenario.CreateCuboid(Vector3d.Zero, mass: (Fixed64)2);
        ScenarioBody<LSCuboidCollider> fast = fastScenario.CreateCuboid(Vector3d.Zero, mass: (Fixed64)2);
        Vector3d impulse = new((Fixed64)3, (Fixed64)(-2), Fixed64.One);
        Vector3d expectedVelocity = impulse * slow.Body.InverseInertiaTensor;

        slow.Body.AddAngularImpulse(impulse);
        fast.Body.AddAngularImpulse(impulse);

        slow.Body.AngularVelocity.Should().Be(expectedVelocity);
        fast.Body.AngularVelocity.Should().Be(expectedVelocity);
        slow.Body.Rotation.Should().Be(FixedQuaternion.Identity);
        fast.Body.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void AddLinearImpulse2D_ShouldMatchImmediateAngularImpulseContract()
    {
        using GravitasWorldContext slowContext = Physics2DTestWorld.CreateContext(frameRate: 10);
        using GravitasWorldContext fastContext = Physics2DTestWorld.CreateContext(frameRate: 100);
        SolidBody2D slow = Create2DBody(slowContext);
        SolidBody2D fast = Create2DBody(fastContext);
        Vector2d impulse = new((Fixed64)6, (Fixed64)(-2));
        Vector2d expectedVelocity = impulse * slow.InverseMass;

        slow.AddLinearImpulse(impulse);
        fast.AddLinearImpulse(impulse);

        slow.LinearVelocity.Should().Be(expectedVelocity);
        fast.LinearVelocity.Should().Be(expectedVelocity);
        slow.Position.Should().Be(Vector2d.Zero);
        fast.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void AddLinearImpulse_ShouldAdvancePoseOnlyDuringTheNextFixedStep()
    {
        using PhysicsScenarioBuilder scenario = Create3DScenario(frameRate: 8);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);
        Vector3d impulse = Vector3d.Right * (Fixed64)4;
        Vector3d expectedVelocity = impulse * body.Body.InverseMass;

        body.Body.AddLinearImpulse(impulse);
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(expectedVelocity);
        body.Body.Position3d.Should().Be(expectedVelocity * scenario.Context.DeltaTime);
    }

    private static PhysicsScenarioBuilder Create3DScenario(int frameRate)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(frameRate);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        return scenario;
    }

    private static SolidBody2D Create2DBody(GravitasWorldContext context)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context),
            new LSCircleCollider2D(Fixed64.One))
        {
            Mass = (Fixed64)2
        };
        body.Initialize(Vector2d.Zero);
        return body;
    }
}
