using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedContinuous3DSource_MovingDiscrete2DTargetShouldNotLeaveStaleStaticHit()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            Vector2d.Zero,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.Body.AddLinearImpulse(Vector3d.Right * (Fixed64)6);
        target.Agent.Transform.LocalPosition = Vector3d.Right * (Fixed64)6;

        context.LateSimulate();

        source.Body.Position3d.Should().Be(Vector3d.Right * (Fixed64)3);
        source.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)6);
    }

    [Fact]
    public void MixedContinuous2DSource_MovingDiscrete3DTargetShouldNotLeaveStaleStaticHit()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            Vector3d.Zero,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.AddLinearImpulse(Vector2d.Right * (Fixed64)6);
        target.Body.Agent.Transform.LocalPosition = Vector3d.Right * (Fixed64)6;

        context.LateSimulate();

        source.Position.Should().Be(Vector2d.Right * (Fixed64)3);
        source.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)6);
    }
}
