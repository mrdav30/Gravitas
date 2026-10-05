using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SolidBodyMassMutationContinuousCollisionTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void MassAssignment3D_ShouldPreserveCcdForNoOpsAndDiscardOldTrajectoryCandidatesAndHandoffForChanges(
        bool mixed, bool noOp)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(4);
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.UseManualGrounding();
        body.LinearDragCoefficient = Fixed64.Zero;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)4);
        context.Physics.PrepareContinuousCollisionFrame();

        Vector3d impactPosition = Vector3d.Right * (Fixed64)10;
        Fixed64 remainingTime = context.DeltaTime * Fixed64.Half;
        body.ApplyContinuousCollisionHandoff(impactPosition, Vector3d.Zero, remainingTime)
            .Should().BeTrue();
        body.ContinuousCollisionTrajectoryCount.Should().Be(2);
        var firstSegment = body.GetContinuousCollisionTrajectorySegment(0);
        var lastSegment = body.GetContinuousCollisionTrajectorySegment(1);
        var oldBounds = new FixedBoundVolume(-Vector3d.One, Vector3d.One);
        context.Physics.QueryContinuousCollisionCandidates(oldBounds)
            .Should().ContainSingle().Which.Should().Be(body.DynamicId);

        body.Mass = noOp ? Fixed64.One : Fixed64.Two;

        body.Position3d.Should().Be(impactPosition);
        body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)4);
        body.ContinuousCollisionTrajectoryCount.Should().Be(noOp ? 2 : 0);
        context.Physics.TryGetContinuousCollisionCandidate(body.DynamicId, out _).Should().Be(noOp);
        if (noOp)
        {
            body.GetContinuousCollisionTrajectorySegment(0).Should().Be(firstSegment);
            body.GetContinuousCollisionTrajectorySegment(1).Should().Be(lastSegment);
        }

        // The old excursion must disappear when the invalidated index rebuilds
        // within the same token; retained dirty candidates must not resurrect it.
        context.Physics.QueryContinuousCollisionCandidates(oldBounds).Count.Should().Be(noOp ? 1 : 0);
        var currentBounds = new FixedBoundVolume(impactPosition - Vector3d.One, impactPosition + Vector3d.One);
        context.Physics.QueryContinuousCollisionCandidates(currentBounds)
            .Should().ContainSingle().Which.Should().Be(body.DynamicId);
        body.ContinuousCollisionTrajectoryCount.Should().Be(noOp ? 2 : 1);
        if (!noOp)
            body.ContinuousCollisionFrameStart.Should().Be(impactPosition);

        body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false)
            .Should().Be(noOp);
        body.Position3d.Should().Be(noOp
            ? impactPosition + Vector3d.Right * (Fixed64)4 * remainingTime
            : impactPosition);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void MassAssignment2D_ShouldPreserveCcdForNoOpsAndDiscardOldPlanarMixedCandidatesAndHandoffForChanges(
        bool mixed, bool noOp)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(4);
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD;
        context.Environment.Gravity = Fixed64.Zero;
        var body = new SolidBody2D(new TestMatterAgent(context), new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        body.Initialize(Vector2d.Zero);
        body.UseManualGrounding();
        body.AddLinearImpulse(Vector2d.Right * (Fixed64)4);
        context.Physics2D.PrepareContinuousCollisionFrame();

        Vector2d impactPosition = Vector2d.Right * (Fixed64)10;
        Fixed64 remainingTime = context.DeltaTime * Fixed64.Half;
        body.ApplyContinuousCollisionHandoff(impactPosition, Vector2d.Zero, remainingTime)
            .Should().BeTrue();
        body.ContinuousCollisionTrajectoryCount.Should().Be(2);
        var firstSegment = body.GetContinuousCollisionTrajectorySegment(0);
        var lastSegment = body.GetContinuousCollisionTrajectorySegment(1);
        var oldPlanarBounds = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One);
        var oldMixedBounds = new FixedBoundVolume(-Vector3d.One, Vector3d.One);
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(oldPlanarBounds)
            .Should().ContainSingle().Which.Should().Be(body.DynamicId);
        if (mixed)
            context.Physics2D.QueryMixedContinuousCollisionCandidates(oldMixedBounds)
                .Should().ContainSingle().Which.Should().Be(body.DynamicId);

        body.Mass = noOp ? Fixed64.One : Fixed64.Two;

        body.Position.Should().Be(impactPosition);
        body.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)4);
        body.ContinuousCollisionTrajectoryCount.Should().Be(noOp ? 2 : 0);
        context.Physics2D.TryGetContinuousCollisionCandidate(body.DynamicId, out _).Should().Be(noOp);
        if (noOp)
        {
            body.GetContinuousCollisionTrajectorySegment(0).Should().Be(firstSegment);
            body.GetContinuousCollisionTrajectorySegment(1).Should().Be(lastSegment);
        }

        context.Physics2D.QueryPlanarContinuousCollisionCandidates(oldPlanarBounds).Count.Should().Be(noOp ? 1 : 0);
        if (mixed)
            context.Physics2D.QueryMixedContinuousCollisionCandidates(oldMixedBounds).Count.Should().Be(noOp ? 1 : 0);
        var currentPlanarBounds = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            impactPosition, impactPosition, Fixed64.One);
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(currentPlanarBounds)
            .Should().ContainSingle().Which.Should().Be(body.DynamicId);
        if (mixed)
        {
            Vector3d center = impactPosition.ToVector3d(Fixed64.Zero);
            context.Physics2D.QueryMixedContinuousCollisionCandidates(
                    new FixedBoundVolume(center - Vector3d.One, center + Vector3d.One))
                .Should().ContainSingle().Which.Should().Be(body.DynamicId);
        }
        body.ContinuousCollisionTrajectoryCount.Should().Be(noOp ? 2 : 1);
        if (!noOp)
            body.ContinuousCollisionFrameStart.Should().Be(impactPosition);

        body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false)
            .Should().Be(noOp);
        body.Position.Should().Be(noOp
            ? impactPosition + Vector2d.Right * (Fixed64)4 * remainingTime
            : impactPosition);
    }
}
