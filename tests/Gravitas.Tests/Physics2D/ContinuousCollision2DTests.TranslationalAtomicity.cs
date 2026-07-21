using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void RotationalDynamicResponse_WithFullyLockedDynamicTarget_ShouldApplyOnlySourceState2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        target.FreezeAxes = BodyFreezeAxes2D.All;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4);
        var contact = new Contact2D(
            -Vector2d.Right * Fixed64.Half,
            -Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Zero);

        bool applied = source.TryApplyRotationalContinuousCollisionResponse(
            target,
            contact,
            Fixed64.Half,
            source.Position,
            Fixed64.Zero,
            Vector2d.Right * Fixed64.Two,
            Fixed64.Zero,
            Fixed64.Zero,
            context.DeltaTime);

        applied.Should().BeTrue();
        source.LinearVelocity.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TranslationalDynamicResponse_WithFullyLockedDynamicTarget_ShouldApplyOnlySourceState2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        target.FreezeAxes = BodyFreezeAxes2D.All;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4);

        bool applied = InvokeTranslationalDynamicResponse2D(
            source,
            target,
            -Vector2d.Right,
            -Vector2d.Right,
            context.DeltaTime * Fixed64.Half,
            context.DeltaTime * Fixed64.Half);

        applied.Should().BeTrue();
        source.LinearVelocity.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TranslationalDynamicResponse_WhenTargetTrajectoryIsFull_ShouldRejectAtomically2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4);
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        context.Diagnostics.Enable(eventCapacity: 8, drawCommandCapacity: 0);

        Vector2d sourcePosition = source.Position;
        Vector2d sourceVelocity = source.LinearVelocity;
        Vector2d targetPosition = target.Position;
        Fixed64 targetRotation = target.Rotation;
        Vector2d targetVelocity = target.LinearVelocity;
        Fixed64 targetAngularVelocity = target.AngularVelocity;
        bool sourceSleeping = source.IsSleeping;
        bool targetSleeping = target.IsSleeping;
        int sourceTrajectoryCount = source.ContinuousCollisionTrajectoryCount;
        int targetTrajectoryCount = target.ContinuousCollisionTrajectoryCount;
        int diagnosticCount = context.Diagnostics.EventCount;
        var replayHash = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        bool applied = InvokeTranslationalDynamicResponse2D(
            source,
            target,
            -Vector2d.Right,
            -Vector2d.Right,
            context.DeltaTime * Fixed64.Half,
            context.DeltaTime * Fixed64.Half);

        applied.Should().BeFalse();
        source.Position.Should().Be(sourcePosition);
        source.LinearVelocity.Should().Be(sourceVelocity);
        target.Position.Should().Be(targetPosition);
        target.Rotation.Should().Be(targetRotation);
        target.LinearVelocity.Should().Be(targetVelocity);
        target.AngularVelocity.Should().Be(targetAngularVelocity);
        source.IsSleeping.Should().Be(sourceSleeping);
        target.IsSleeping.Should().Be(targetSleeping);
        source.ContinuousCollisionTrajectoryCount.Should().Be(sourceTrajectoryCount);
        target.ContinuousCollisionTrajectoryCount.Should().Be(targetTrajectoryCount);
        context.Diagnostics.EventCount.Should().Be(diagnosticCount);
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(replayHash);
    }

    private static bool InvokeTranslationalDynamicResponse2D(
        SolidBody2D source,
        SolidBody2D target,
        Vector2d normal,
        Vector2d sourcePositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Fixed64 frameFraction = FixedMath.Clamp01(hitElapsedTime / source.Context.DeltaTime);
        return source.TryApplyContinuousCollisionDynamicResponse(
            target,
            normal,
            sourcePositionAtImpact,
            target.SampleContinuousCollisionPosition(frameFraction),
            hitElapsedTime,
            remainingTime);
    }
}
