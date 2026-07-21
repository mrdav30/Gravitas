using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void TrajectorySampling3D_ShouldUseAuthoritativePoseUntilMotionIsPrepared()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Right);
        FixedQuaternion authoredRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Half);
        body.Body.SetRotation(authoredRotation);

        body.Body.SampleContinuousCollisionPosition(Fixed64.Half).Should().Be(Vector3d.Right);
        body.Body.SampleContinuousCollisionRotation(Fixed64.Half).Should().Be(authoredRotation);

        body.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        body.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up);
        scenario.Context.AdvanceLateSimulateToken();
        body.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);

        body.Body.SampleContinuousCollisionPosition(Fixed64.One).Should().NotBe(Vector3d.Right);
        body.Body.SampleContinuousCollisionRotation(Fixed64.One).Should().NotBe(authoredRotation);
    }

    [Fact]
    public void DirtyCandidateRefreshAndQuery3D_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        body.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        var queryBounds = new FixedBoundVolume(
            Vector3d.Right - Vector3d.One,
            Vector3d.Right + Vector3d.One);

        void RefreshAndQuery()
        {
            scenario.Context.Physics.RefreshContinuousCollisionCandidate(body.Body);
            scenario.Context.Physics.QueryContinuousCollisionCandidates(queryBounds);
        }

        RefreshAndQuery();
        AllocationTestHelper.MeasureSteadyState(RefreshAndQuery)
            .Should()
            .Be(0);
    }

    [Fact]
    public void ClippedTrajectory3D_ShouldHaveStableReplayHashAfterSupersededTailReplacement()
    {
        using PhysicsScenarioBuilder directScenario = CreateCcdScenario();
        using PhysicsScenarioBuilder replacedScenario = CreateCcdScenario();
        directScenario.Context.Environment.MaxSpeed = (Fixed64)100;
        replacedScenario.Context.Environment.MaxSpeed = (Fixed64)100;
        directScenario.Context.Settings.ContinuousCollisionMaxToiIterations = 2;
        replacedScenario.Context.Settings.ContinuousCollisionMaxToiIterations = 2;
        ScenarioBody<LSSphereCollider> direct = directScenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> replaced = replacedScenario.CreateSphere(Vector3d.Zero);
        direct.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)100);
        replaced.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)100);
        directScenario.Context.AdvanceLateSimulateToken();
        replacedScenario.Context.AdvanceLateSimulateToken();
        direct.Body.EnsureContinuousCollisionFramePrepared(
            directScenario.Context.LateSimulateToken);
        replaced.Body.EnsureContinuousCollisionFramePrepared(
            replacedScenario.Context.LateSimulateToken);

        direct.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * (Fixed64)25,
            FixedQuaternion.Identity,
            Vector3d.Left * (Fixed64)200,
            Vector3d.Zero,
            Fixed64.FromFraction(3, 4));
        direct.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Left * (Fixed64)25,
            FixedQuaternion.Identity,
            Vector3d.Right * (Fixed64)50,
            Vector3d.Zero,
            Fixed64.Half);
        replaced.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * (Fixed64)25,
            FixedQuaternion.Identity,
            Vector3d.Left * (Fixed64)200,
            Vector3d.Zero,
            Fixed64.FromFraction(3, 4));
        replaced.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * (Fixed64)25,
            FixedQuaternion.Identity,
            Vector3d.Left * (Fixed64)200,
            Vector3d.Zero,
            Fixed64.FromFraction(1, 4));
        replaced.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Left * (Fixed64)25,
            FixedQuaternion.Identity,
            Vector3d.Right * (Fixed64)50,
            Vector3d.Zero,
            Fixed64.Half);

        direct.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
        replaced.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
        directScenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.Authoritative)
            .Should()
            .Be(replacedScenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.Authoritative));
    }

    [Fact]
    public void DirtyTrajectory3D_ShouldShadowPreparedFutureInPureAndMixedQueries()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)100);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        Vector3d impactPosition = Vector3d.Right * (Fixed64)25;

        body.Body.ApplyContinuousCollisionHandoff(
                impactPosition,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();

        var impactBounds = new FixedBoundVolume(
            impactPosition - Vector3d.One,
            impactPosition + Vector3d.One);
        var supersededBounds = new FixedBoundVolume(
            Vector3d.Right * (Fixed64)99 - Vector3d.One,
            Vector3d.Right * (Fixed64)101 + Vector3d.One);

        scenario.Context.Physics.QueryContinuousCollisionCandidates(impactBounds)
            .Contains(body.Body.DynamicId)
            .Should()
            .BeTrue();
        scenario.Context.Physics.QueryContinuousCollisionCandidates(supersededBounds)
            .Contains(body.Body.DynamicId)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void HandoffTrajectory3D_ShouldClipPreparedFutureForStopAndReverse()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        ScenarioBody<LSSphereCollider> stopped = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> reversed = scenario.CreateSphere(Vector3d.Zero);
        Vector3d preparedVelocity = Vector3d.Right * (Fixed64)100;
        stopped.Body.ApplyCollisionLinearVelocityDelta(preparedVelocity);
        reversed.Body.ApplyCollisionLinearVelocityDelta(preparedVelocity);
        scenario.Context.AdvanceLateSimulateToken();
        stopped.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        reversed.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        Fixed64 impactFraction = Fixed64.FromFraction(1, 4);
        Fixed64 remainingTime = Fixed64.One - impactFraction;
        Vector3d impactPosition = Vector3d.Right * (Fixed64)25;

        stopped.Body.ApplyContinuousCollisionHandoff(
                impactPosition,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                remainingTime)
            .Should()
            .BeTrue();
        reversed.Body.ApplyContinuousCollisionHandoff(
                impactPosition,
                FixedQuaternion.Identity,
                -preparedVelocity,
                Vector3d.Zero,
                remainingTime)
            .Should()
            .BeTrue();

        stopped.Body.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 8))
            .Should()
            .Be(Vector3d.Right * Fixed64.FromFraction(25, 2));
        stopped.Body.SampleContinuousCollisionPosition(impactFraction)
            .Should()
            .Be(impactPosition);
        stopped.Body.SampleContinuousCollisionPosition(Fixed64.FromFraction(5, 8))
            .Should()
            .Be(impactPosition);
        reversed.Body.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 8))
            .Should()
            .Be(Vector3d.Right * Fixed64.FromFraction(25, 2));
        reversed.Body.SampleContinuousCollisionPosition(impactFraction)
            .Should()
            .Be(impactPosition);
        reversed.Body.SampleContinuousCollisionPosition(Fixed64.FromFraction(5, 8))
            .Should()
            .Be(Vector3d.Left * Fixed64.FromFraction(25, 2));

        stopped.Body.TryResolveContinuousCollisionMotionBound(
                impactFraction,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 stoppedMotionBound)
            .Should()
            .BeTrue();
        reversed.Body.TryResolveContinuousCollisionMotionBound(
                impactFraction,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 reversedMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 expectedStoppedMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector3d.Left * (Fixed64)75,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 expectedReversedMotionBound)
            .Should()
            .BeTrue();
        stoppedMotionBound.Should().Be(expectedStoppedMotionBound);
        reversedMotionBound.Should().Be(expectedReversedMotionBound);

        FixedBoundVolume stoppedBounds =
            stopped.Body.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero);
        FixedBoundVolume reversedBounds =
            reversed.Body.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero);
        stoppedBounds.Min.X.Should().Be(Fixed64.Zero);
        stoppedBounds.Max.X.Should().Be((Fixed64)25);
        reversedBounds.Min.X.Should().Be((Fixed64)(-50));
        reversedBounds.Max.X.Should().Be((Fixed64)25);
    }

    [Fact]
    public void DirtyCandidateReservations3D_ShouldScaleWithRegisteredBodyCount()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 4;
        var bodies = new SolidBody[12];
        for (int i = 0; i < bodies.Length; i++)
        {
            bodies[i] = scenario.CreateSphere(Vector3d.Right * (Fixed64)(i * 4)).Body;
        }

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        for (int i = 0; i < bodies.Length; i += 2)
        {
            scenario.Context.Physics.TryReserveContinuousCollisionCandidateRefresh(
                    bodies[i],
                    bodies[i + 1])
                .Should()
                .BeTrue();
        }
    }

    [Fact]
    public void DirtyCandidateIndex3D_WhenDynamicIdIsReused_ShouldDiscardRetiredBounds()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> original = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        Vector3d retiredExcursion = Vector3d.Right * (Fixed64)10;
        original.Body.ApplyContinuousCollisionHandoff(
                retiredExcursion,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        int reusedId = original.Body.DynamicId;

        original.Body.Deactivate();
        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(Vector3d.Up * (Fixed64)10);
        replacement.Body.DynamicId.Should().Be(reusedId);
        var retiredBounds = new FixedBoundVolume(
            retiredExcursion - Vector3d.One,
            retiredExcursion + Vector3d.One);

        scenario.Context.Physics.QueryContinuousCollisionCandidates(retiredBounds)
            .Should()
            .NotContain(replacement.Body.DynamicId);
    }

    [Fact]
    public void PreparedKinematicTrajectory3D_ShouldConsumeCapturedHostEndpoint()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        Vector3d capturedPosition = new(Fixed64.One, Fixed64.Two, (Fixed64)3);
        FixedQuaternion capturedRotation = PhysicsScenarioBuilder.Yaw(45);
        mover.Body.Agent.Transform.LocalPosition = capturedPosition;
        mover.Body.Agent.Transform.LocalRotation = capturedRotation;

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        mover.Body.Agent.Transform.LocalPosition = new Vector3d((Fixed64)4, (Fixed64)5, (Fixed64)6);
        mover.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(90);

        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: true)
            .Should()
            .BeTrue();

        mover.Body.Position3d.Should().Be(capturedPosition);
        mover.Body.Rotation.Should().Be(capturedRotation);
        mover.Body.Agent.Transform.LocalPosition.Should().Be(capturedPosition);
        mover.Body.Agent.Transform.LocalRotation.Should().Be(capturedRotation);
    }

    [Fact]
    public void HandoffTrajectory3D_ShouldRetainIncreasingExcursionsAndReplaceEqualOrEarlierTail()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 2;
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        mover.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)100);
        scenario.Context.AdvanceLateSimulateToken();
        mover.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);

        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Left * (Fixed64)200,
                Vector3d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Left * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Right * (Fixed64)50,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.Body.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 4))
            .Should()
            .Be(Vector3d.Right * (Fixed64)25);
        mover.Body.SampleContinuousCollisionPosition(Fixed64.Half)
            .Should()
            .Be(Vector3d.Left * (Fixed64)25);
        mover.Body.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero).Min.X
            .Should()
            .Be((Fixed64)(-25));
        mover.Body.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero).Max.X
            .Should()
            .Be((Fixed64)25);
        mover.Body.TryResolveContinuousCollisionMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 motionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector3d.Right * (Fixed64)25,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 excursionMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector3d.Left * (Fixed64)50,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 returnMotionBound)
            .Should()
            .BeTrue();
        motionBound.Should().Be(excursionMotionBound * Fixed64.Two + returnMotionBound);

        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Left * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right * (Fixed64)25,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One)
            .Should()
            .BeTrue();
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(1);
        mover.Body.SampleContinuousCollisionPosition(Fixed64.One)
            .Should()
            .Be(Vector3d.Zero);
    }

    [Fact]
    public void SubthresholdHandoffVelocity3D_ShouldPublishStationaryTrajectory()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        mover.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        Vector3d impactPosition = Vector3d.Right;

        mover.Body.ApplyContinuousCollisionHandoff(
            impactPosition,
            Vector3d.Right * Fixed64.FromRaw(2),
            Fixed64.Half);

        mover.Body.SampleContinuousCollisionPosition(Fixed64.One)
            .Should()
            .Be(impactPosition);
    }
}
