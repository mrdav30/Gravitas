using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void ContinuousHandoff_WhenConsumptionThrows_ShouldCloseBatch()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D throwing = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D unread = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)8),
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        throwing.ApplyContinuousCollisionHandoff(
            throwing.Position,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            Fixed64.One);
        unread.ApplyContinuousCollisionHandoff(unread.Position, Vector2d.Right, Fixed64.Half);

        Action process = () => context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 4);

        process.Should().Throw<ArgumentOutOfRangeException>();
        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 4).Should().Be(0);
        throwing.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        unread.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();

        var replayHash = context.ComputeReplayHash();
        throwing.DiscardContinuousCollisionHandoff();
        unread.DiscardContinuousCollisionHandoff();
        context.ComputeReplayHash().Should().Be(replayHash);
    }

    [Fact]
    public void ContinuousHandoff_WhenQueuedStateIsSupersededByTerminalUpdates_ShouldNotConsumeStaleContinuation()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D noTime = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D noVelocity = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        noTime.ApplyContinuousCollisionHandoff(Vector2d.Zero, Vector2d.Right, Fixed64.One);
        noTime.ApplyContinuousCollisionHandoff(Vector2d.Right * Fixed64.Half, Vector2d.Right, Fixed64.Zero);
        noVelocity.ApplyContinuousCollisionHandoff(noVelocity.Position, Vector2d.Right, Fixed64.One);
        noVelocity.ApplyContinuousCollisionHandoff(noVelocity.Position, Vector2d.Left, Fixed64.Half);

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 2).Should().Be(0);

        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
        noTime.Position.Should().Be(Vector2d.Right * Fixed64.Half);
        noTime.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)2);
        noVelocity.Position.Should().Be(new Vector2d(Fixed64.Zero, (Fixed64)4));
        noVelocity.LinearVelocity.Should().Be(Vector2d.Zero);
        var replayHash = context.ComputeReplayHash();
        noTime.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        noVelocity.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        noTime.DiscardContinuousCollisionHandoff();
        noVelocity.DiscardContinuousCollisionHandoff();
        context.ComputeReplayHash().Should().Be(replayHash);
    }

    [Theory]
    // Piecewise target histories resolve the causal relay in four dequeues;
    // only smaller shared budgets discard the terminal continuation.
    [InlineData(5, 4, false)]
    [InlineData(4, 4, false)]
    [InlineData(3, 3, true)]
    public void ContinuousHandoff_WhenRelayReturnsToConsumedBody_ShouldRequeueOrDiscardAtBudget(
        int iterationBudget,
        int expectedIterations,
        bool expectedLimitReached)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        SolidBody2D returning = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D middle = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D opposing = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * (Fixed64)4,
            immovable: false);
        returning.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        middle.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        opposing.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        // Perfectly elastic A -> B <- C impacts return the handoff to A after its first dequeue.
        returning.ApplyContinuousCollisionHandoff(
            returning.Position,
            Vector2d.Right * (Fixed64)6,
            Fixed64.One);
        opposing.ApplyContinuousCollisionHandoff(
            opposing.Position,
            Vector2d.Left * (Fixed64)6,
            Fixed64.One);

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget)
            .Should().Be(expectedIterations);

        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(expectedIterations);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().Be(expectedLimitReached);
        var replayHash = context.ComputeReplayHash();
        returning.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
        returning.DiscardContinuousCollisionHandoff();
        context.ComputeReplayHash().Should().Be(replayHash);
    }
}
