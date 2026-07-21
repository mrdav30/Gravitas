using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void ContinuousHandoff_WhenMovementCallbackRequeuesAndThrows_ShouldCloseBatch()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> throwing = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> unread = scenario.CreateSphere(Vector3d.Up * (Fixed64)8);
        DisableGroundQueries(throwing.Body);
        DisableGroundQueries(unread.Body);
        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        throwing.Body.ApplyContinuousCollisionHandoff(Vector3d.Zero, Vector3d.Right, Fixed64.Half);
        unread.Body.ApplyContinuousCollisionHandoff(unread.Body.Position3d, Vector3d.Right, Fixed64.Half);
        var expected = new InvalidOperationException("movement callback failure");
        throwing.Body.OnMoved += () =>
        {
            throwing.Body.ApplyContinuousCollisionHandoff(
                throwing.Body.Position3d,
                Vector3d.Right,
                Fixed64.Half);
            throw expected;
        };

        Action process = () => scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 4);

        process.Should().Throw<InvalidOperationException>().Which.Should().BeSameAs(expected);
        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();

        throwing.Body.OnMoved = null;
        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 4).Should().Be(0);
        throwing.Body.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        unread.Body.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();

        var replayHash = scenario.Context.ComputeReplayHash();
        throwing.Body.DiscardContinuousCollisionHandoff();
        unread.Body.DiscardContinuousCollisionHandoff();
        scenario.Context.ComputeReplayHash().Should().Be(replayHash);
    }

    [Fact]
    public void ContinuousHandoff_WhenQueuedStateIsSupersededByTerminalUpdates_ShouldNotConsumeStaleContinuation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> noTime = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> noVelocity = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);
        DisableGroundQueries(noTime.Body);
        DisableGroundQueries(noVelocity.Body);
        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        noTime.Body.ApplyContinuousCollisionHandoff(Vector3d.Zero, Vector3d.Right, Fixed64.One);
        noTime.Body.ApplyContinuousCollisionHandoff(Vector3d.Right * Fixed64.Half, Vector3d.Right, Fixed64.Zero);
        noVelocity.Body.ApplyContinuousCollisionHandoff(noVelocity.Body.Position3d, Vector3d.Right, Fixed64.One);
        noVelocity.Body.ApplyContinuousCollisionHandoff(noVelocity.Body.Position3d, Vector3d.Left, Fixed64.Half);

        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 2).Should().Be(0);

        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(0);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
        noTime.Body.Position3d.Should().Be(Vector3d.Right * Fixed64.Half);
        noTime.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)2);
        noVelocity.Body.Position3d.Should().Be(Vector3d.Up * (Fixed64)4);
        noVelocity.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        var replayHash = scenario.Context.ComputeReplayHash();
        noTime.Body.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        noVelocity.Body.TryConsumeContinuousCollisionHandoff(false, false).Should().BeFalse();
        noTime.Body.DiscardContinuousCollisionHandoff();
        noVelocity.Body.DiscardContinuousCollisionHandoff();
        scenario.Context.ComputeReplayHash().Should().Be(replayHash);
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
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 4;
        scenario.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> returning = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> opposing = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);
        returning.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        middle.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        opposing.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        DisableGroundQueries(returning.Body);
        DisableGroundQueries(middle.Body);
        DisableGroundQueries(opposing.Body);
        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        // Perfectly elastic A -> B <- C impacts return the handoff to A after its first dequeue.
        returning.Body.ApplyContinuousCollisionHandoff(
            returning.Body.Position3d,
            Vector3d.Right * (Fixed64)6,
            Fixed64.One);
        opposing.Body.ApplyContinuousCollisionHandoff(
            opposing.Body.Position3d,
            Vector3d.Left * (Fixed64)6,
            Fixed64.One);

        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget)
            .Should().Be(expectedIterations);

        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(expectedIterations);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().Be(expectedLimitReached);
        var replayHash = scenario.Context.ComputeReplayHash();
        returning.Body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
        returning.Body.DiscardContinuousCollisionHandoff();
        scenario.Context.ComputeReplayHash().Should().Be(replayHash);
    }
}
