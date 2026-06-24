using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class StiffBody2DAngularDynamicsTests
{
    [Fact]
    public void AddAngularImpulse_ShouldChangeAngularVelocityImmediatelyWhenBodyCanRotate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 8);
        StiffBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);

        body.AddAngularImpulse((Fixed64)3);

        body.AngularVelocity.Should().Be((Fixed64)3 * body.EffectiveInverseMomentOfInertia);
        body.AngularSpeed.Should().Be(body.AngularVelocity.Abs());
    }

    [Fact]
    public void AddTorque_ShouldIntegrateAngularVelocityAndRotationDuringLateSimulate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        StiffBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);

        body.AddTorque((Fixed64)8);

        context.LateSimulate();

        body.AngularAcceleration.Should().Be((Fixed64)8);
        body.AngularVelocity.Should().Be((Fixed64)2);
        body.AngularSpeed.Should().Be((Fixed64)2);
        body.Rotation.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void AngularForces_ShouldBeIgnoredWhenBodyCannotRotate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        StiffBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);
        body.PreventAngularForces = true;

        body.AddAngularImpulse((Fixed64)3);
        body.AddTorque((Fixed64)8);
        context.LateSimulate();

        body.AngularVelocity.Should().Be(Fixed64.Zero);
        body.AngularAcceleration.Should().Be(Fixed64.Zero);
        body.AngularSpeed.Should().Be(Fixed64.Zero);
        body.Rotation.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_ShouldKeepBodyAwakeWhileAngularSpeedExceedsSleepThreshold()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        StiffBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);
        body.SleepFrameThreshold = 1;
        body.SleepLinearSpeedThreshold = Fixed64.Zero;
        body.SleepAngularSpeedThreshold = Fixed64.Half;
        body.AddAngularImpulse(Fixed64.One);

        context.LateSimulate();

        body.IsSleeping.Should().BeFalse();

        body.SleepAngularSpeedThreshold = (Fixed64)2;
        context.LateSimulate();

        body.IsSleeping.Should().BeTrue();
        body.AngularVelocity.Should().Be(Fixed64.Zero);
        body.AngularSpeed.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ShapeMutation_ShouldRefreshMomentUsedByAngularImpulse()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        var collider = new LSCircleCollider2D(Fixed64.One);
        StiffBody2D body = CreateBody(context, collider, mass: (Fixed64)2);

        body.AddAngularImpulse(Fixed64.One);
        collider.Radius = (Fixed64)2;
        body.AddAngularImpulse((Fixed64)4);

        body.MomentOfInertia.Should().Be((Fixed64)4);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.FromFraction(1, 4));
        body.AngularVelocity.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ResetPosition_ShouldReturnMovingBodyToRest()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        StiffBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);

        body.AddForce(new Vector2d((Fixed64)4, Fixed64.Zero));
        body.AddAngularImpulse(Fixed64.One);
        context.LateSimulate();

        body.LinearSpeed.Should().BeGreaterThan(Fixed64.Zero);
        body.AngularSpeed.Should().BeGreaterThan(Fixed64.Zero);

        body.ResetPosition(new Vector2d((Fixed64)3, (Fixed64)4), Fixed64.Half);

        body.Position.Should().Be(new Vector2d((Fixed64)3, (Fixed64)4));
        body.Rotation.Should().Be(Fixed64.Half);
        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.LinearSpeed.Should().Be(Fixed64.Zero);
        body.AngularVelocity.Should().Be(Fixed64.Zero);
        body.AngularSpeed.Should().Be(Fixed64.Zero);
        body.IsSleeping.Should().BeFalse();
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Fixed64 mass)
    {
        var body = new StiffBody2D(new TestMatterAgent(context), collider)
        {
            Mass = mass
        };

        body.Initialize(Vector2d.Zero);
        return body;
    }
}
