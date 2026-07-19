using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void ContinuousSource_MovingDiscreteKinematicTargetShouldNotLeaveStaleStaticHit2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.AddLinearImpulse(Vector2d.Right * (Fixed64)6);
        target.Agent.Transform.LocalPosition = Vector3d.Right * (Fixed64)10;

        context.LateSimulate();

        source.Position.Should().Be(Vector2d.Right * (Fixed64)3);
        source.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)6);
    }

    [Fact]
    public void ContinuousSource_BelowThresholdAutoKinematicTargetShouldUseMovingHitTime2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        source.AddLinearImpulse(Vector2d.Right * (Fixed64)6);
        target.Agent.Transform.LocalPosition = Vector3d.Right * Fixed64.FromFraction(1, 4);

        context.LateSimulate();

        source.Position.X.Should().NotBe(-Fixed64.One);
        source.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
    }
}
