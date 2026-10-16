using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class BodyDissipationParityTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, 4, 1)]
    [InlineData(PhysicsRuntimeMode.TwoD, 64, 8)]
    [InlineData(PhysicsRuntimeMode.Mixed, 4, 1)]
    [InlineData(PhysicsRuntimeMode.Mixed, 64, 8)]
    public void LargeAmbientDissipation_ShouldPreservePlanarFreeFlightAndDrivenMotion(
        PhysicsRuntimeMode mode, int rate, int sizeDivisor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        SolidBody2D body = CreateBody(scene.Context, mode, rate, sizeDivisor);
        Fixed64 inertia = body.MomentOfInertia;
        Vector2d velocity = new(Fixed64.One, Fixed64.Half);
        body.AddLinearImpulse(velocity);
        body.AddAngularImpulse(inertia);
        StepAndExpect(scene.Context, body, velocity, Fixed64.One);

        // Driving input can cancel existing motion exactly, without a passive
        // 3D term introducing a new direction in the planar integration owner.
        body.AddForce(-velocity * (Fixed64)rate);
        body.AddTorque(-inertia * (Fixed64)rate);
        StepAndExpect(scene.Context, body, Vector2d.Zero, Fixed64.Zero);

        body.AddForce(new Vector2d(4, 2));
        body.AddTorque(inertia * (Fixed64)4);
        Vector2d drivenVelocity = new Vector2d(4, 2) * scene.Context.DeltaTime;
        Fixed64 drivenSpin = (Fixed64)4 * scene.Context.DeltaTime;
        StepAndExpect(scene.Context, body, drivenVelocity, drivenSpin);

        body.AddForce(new Vector2d(-8, -4));
        body.AddTorque(-inertia * (Fixed64)8);
        StepAndExpect(scene.Context, body, -drivenVelocity, -drivenSpin);
        StepAndExpect(scene.Context, body, -drivenVelocity, -drivenSpin);

        (body.Collider.CollisionPairCount + body.Collider.CollisionPairHolderCount).Should().Be(0);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void LargeAmbientDissipation_ShouldPreserveFrozenAxesAndAllowedDrivenMotion(PhysicsRuntimeMode mode)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        SolidBody2D body = CreateBody(scene.Context, mode, rate: 64, sizeDivisor: 8);
        body.FreezeAxes = BodyFreezeAxes2D.PositionX | BodyFreezeAxes2D.Rotation;
        body.AddLinearImpulse(Vector2d.One);
        body.AddAngularImpulse(Fixed64.One);
        body.AddForce(new Vector2d(1024, 4));
        body.AddTorque(Fixed64.One);

        Vector2d expected = Vector2d.Forward * (Fixed64.One + (Fixed64)4 * scene.Context.DeltaTime);
        StepAndExpect(scene.Context, body, expected, Fixed64.Zero);
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, PhysicsRuntimeMode mode, int rate, int sizeDivisor)
    {
        context.SetFrameRate(rate);
        context.Settings.RuntimeMode = mode;
        context.Environment.AirDensity = (Fixed64)1024;
        context.Environment.DampingFactor = (Fixed64)1024;
        context.Environment.MinSpeed = Fixed64.Zero;
        var collider = new LSCircleCollider2D(Fixed64.FromFraction(1, 2 * sizeDivisor))
        {
            Material = PhysicsMaterial.Frictionless
        };
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = Fixed64.One, Gravity = Vector2d.Zero, SleepEnabled = false
        };
        body.Initialize(Vector2d.Zero);
        return body;
    }

    private static void StepAndExpect(GravitasWorldContext context, SolidBody2D body, Vector2d velocity, Fixed64 spin)
    {
        Vector2d before = body.Position;
        context.Simulate();
        context.LateSimulate();
        body.LinearVelocity.Should().Be(velocity);
        body.AngularVelocity.Should().Be(spin);
        body.Position.Should().Be(before + velocity * context.DeltaTime);
        body.IsGrounded.Should().BeFalse();
    }
}
