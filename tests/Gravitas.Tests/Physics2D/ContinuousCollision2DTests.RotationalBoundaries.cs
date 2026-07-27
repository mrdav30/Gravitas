using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void KinematicRotationalResponse_WithZeroMassFrozenRotationTarget_ShouldRejectZeroEffectiveMass()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        target.Mass = Fixed64.Zero;
        target.FreezeAxes = BodyFreezeAxes2D.Rotation;
        target.Sleep();

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(RotationalMovingPairQuarterTurn2D);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DynamicRotationalResponse_WhenSourceTrajectoryIsFull_ShouldRejectPairAtomically()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.ApplyContinuousCollisionHandoffState(
                blade.Position,
                blade.Rotation,
                blade.LinearVelocity,
                blade.AngularVelocity,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector2d targetPosition = target.Position;

        blade.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();

        blade.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        target.Position.Should().Be(targetPosition);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void KinematicRotationalResponse_WhenTargetTrajectoryIsFull_ShouldLeaveTargetAtomic()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                target.Position,
                target.Rotation,
                Vector2d.Zero,
                Fixed64.Zero,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector2d targetPosition = target.Position;

        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Rotation.Should().BeLessThan(RotationalMovingPairQuarterTurn2D);
        target.Position.Should().Be(targetPosition);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnrepresentableRotationalRadius_ShouldClassifyMovingRegisteredTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var sourceCollider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero)
        };
        SolidBody2D source = CreateBody(
            context,
            sourceCollider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.GatherRotationalContinuousCollisionCandidates(
                Vector2d.Zero,
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.MaxValue)
            .Should()
            .Be(1);
    }

    [Fact]
    public void CircleSeparationGap_UnrepresentableCenterDistance_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.MinIncrement),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            immovable: true);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.MinIncrement),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            immovable: true);

        SolidBody2D.TryGetCircleSeparationGap(
                (LSCircleCollider2D)source.Collider,
                target.Collider,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CircleSeparationGap_ContainedCenter_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: true);
        SolidBody2D target = CreateBody(
            context,
            new LSAABBoxCollider2D(Vector2d.One * Fixed64.Two),
            Vector2d.Zero,
            immovable: true);

        SolidBody2D.TryGetCircleSeparationGap(
                (LSCircleCollider2D)source.Collider,
                target.Collider,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CircleSeparationGap_UnrepresentableBoundaryDistance_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.MinIncrement),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            immovable: true);
        SolidBody2D target = CreateBody(
            context,
            new LSAABBoxCollider2D(Vector2d.One),
            new Vector2d(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero),
            immovable: true);

        SolidBody2D.TryGetCircleSeparationGap(
                (LSCircleCollider2D)source.Collider,
                target.Collider,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CircleSeparationGap_UnrepresentableCombinedRadius_ShouldRemainUncertified()
    {
        Fixed64 radius = Fixed64.MaxValue / Fixed64.Two
            + Fixed64.MinIncrement;
        var source = new LSCircleCollider2D(radius);
        var target = new LSCircleCollider2D(radius);

        SolidBody2D.TryGetCircleSeparationGap(
                source,
                target,
                out _)
            .Should()
            .BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalSweep_BetweenNonCircularShapes_ShouldUseBoundsSeparationFallback(
        bool isKinematic)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(
            context,
            isKinematic);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(
                Fixed64.FromFraction(2, 5),
                Fixed64.FromFraction(2, 5))),
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            immovable: true);
        Fixed64 startRotation = FixedMath.DegToRad((Fixed64)(-5));
        Fixed64 endRotation = -startRotation;
        blade.SetRotation(startRotation);
        if (isKinematic)
            blade.Agent.Transform.LocalRotationXZRadians = endRotation;
        else
            blade.ApplyCollisionAngularVelocityDelta(endRotation - startRotation);

        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(endRotation);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
        if (!isKinematic)
            blade.AngularVelocity.Should().Be(Fixed64.Zero);
    }
}
