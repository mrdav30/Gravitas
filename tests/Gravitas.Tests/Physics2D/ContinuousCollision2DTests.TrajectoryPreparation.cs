using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void PreparedKinematicTrajectory2D_ShouldConsumeCapturedHostEndpoint()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        Vector2d capturedPosition = new(Fixed64.One, Fixed64.Two);
        Fixed64 requestedRotation = FixedMath.DegToRad((Fixed64)45);
        mover.Agent.Transform.LocalPosition = capturedPosition.ToVector3d(Fixed64.Zero);
        mover.Agent.Transform.LocalRotationXZRadians = requestedRotation;
        Fixed64 capturedRotation = mover.Agent.Transform.WorldRotationXZRadians;

        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        mover.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)90);

        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: true)
            .Should()
            .BeTrue();

        mover.Position.Should().Be(capturedPosition);
        (mover.Rotation - capturedRotation).Abs()
            .Should()
            .BeLessThanOrEqualTo(Fixed64.FromRaw(64));
        mover.Agent.Transform.WorldPositionXZ.Should().Be(capturedPosition);
        (mover.Agent.Transform.WorldRotationXZRadians - capturedRotation).Abs()
            .Should()
            .BeLessThanOrEqualTo(Fixed64.FromRaw(64));
    }

    [Fact]
    public void HandoffTrajectory2D_ShouldReplaceSupersededTailAndRetainIncreasingHistory()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        mover.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        mover.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.FromFraction(1, 4),
            Vector2d.Right,
            Fixed64.FromFraction(3, 4));
        mover.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.FromFraction(3, 4),
            Vector2d.Right,
            Fixed64.FromFraction(1, 4));
        mover.ContinuousCollisionTrajectoryCount.Should().Be(3);

        mover.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Left,
            Fixed64.Half);
        mover.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.SampleContinuousCollisionPosition(Fixed64.Half)
            .Should()
            .Be(Vector2d.Right * Fixed64.Half);

        mover.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);
        mover.ContinuousCollisionTrajectoryCount.Should().Be(3);
    }

    [Fact]
    public void SubthresholdHandoffVelocity2D_ShouldPublishStationaryTrajectory()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        mover.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        Vector2d impactPosition = Vector2d.Right;

        mover.ApplyContinuousCollisionHandoff(
            impactPosition,
            Vector2d.Right * Fixed64.FromRaw(2),
            Fixed64.Half);

        mover.SampleContinuousCollisionPosition(Fixed64.One)
            .Should()
            .Be(impactPosition);
    }
}
