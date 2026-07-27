using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void ClippedTrajectory2D_ShouldHaveStableReplayHashAfterSupersededTailReplacement()
    {
        using GravitasWorldContext directContext = CreateContext(frameRate: 1);
        using GravitasWorldContext replacedContext = CreateContext(frameRate: 1);
        directContext.Settings.ContinuousCollisionMaxToiIterations = 2;
        replacedContext.Settings.ContinuousCollisionMaxToiIterations = 2;
        SolidBody2D direct = CreateBody(
            directContext,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D replaced = CreateBody(
            replacedContext,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        direct.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)100);
        replaced.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)100);
        directContext.AdvanceLateSimulateToken();
        replacedContext.AdvanceLateSimulateToken();
        direct.EnsureContinuousCollisionFramePrepared(directContext.LateSimulateToken);
        replaced.EnsureContinuousCollisionFramePrepared(replacedContext.LateSimulateToken);

        direct.ApplyContinuousCollisionHandoffState(
            Vector2d.Right * (Fixed64)25,
            Fixed64.Zero,
            Vector2d.Left * (Fixed64)200,
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4));
        direct.ApplyContinuousCollisionHandoffState(
            Vector2d.Left * (Fixed64)25,
            Fixed64.Zero,
            Vector2d.Right * (Fixed64)50,
            Fixed64.Zero,
            Fixed64.Half);
        replaced.ApplyContinuousCollisionHandoffState(
            Vector2d.Right * (Fixed64)25,
            Fixed64.Zero,
            Vector2d.Left * (Fixed64)200,
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4));
        replaced.ApplyContinuousCollisionHandoffState(
            Vector2d.Right * (Fixed64)25,
            Fixed64.Zero,
            Vector2d.Left * (Fixed64)200,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4));
        replaced.ApplyContinuousCollisionHandoffState(
            Vector2d.Left * (Fixed64)25,
            Fixed64.Zero,
            Vector2d.Right * (Fixed64)50,
            Fixed64.Zero,
            Fixed64.Half);

        direct.ContinuousCollisionTrajectoryCount.Should().Be(3);
        replaced.ContinuousCollisionTrajectoryCount.Should().Be(3);
        directContext.ComputeReplayHash(GravitasReplayHashMode.Authoritative)
            .Should()
            .Be(replacedContext.ComputeReplayHash(
                GravitasReplayHashMode.Authoritative));
    }

    [Fact]
    public void DirtyTrajectory2D_ShouldShadowPreparedFutureInPlanarAndMixedQueries()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 128);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)100);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        Vector2d impactPosition = Vector2d.Right * (Fixed64)25;

        body.ApplyContinuousCollisionHandoffState(
                impactPosition,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();

        DynamicCcdPlanarBounds impactPlanarBounds =
            DynamicCcdCandidateIndex2D.CreateBoundsBetween(
                impactPosition,
                impactPosition,
                Fixed64.One);
        DynamicCcdPlanarBounds supersededPlanarBounds =
            DynamicCcdCandidateIndex2D.CreateBoundsBetween(
                Vector2d.Right * (Fixed64)100,
                Vector2d.Right * (Fixed64)100,
                Fixed64.One);
        var impactMixedBounds = new FixedBoundVolume(
            new Vector3d((Fixed64)24, -Fixed64.One, -Fixed64.One),
            new Vector3d((Fixed64)26, Fixed64.One, Fixed64.One));
        var supersededMixedBounds = new FixedBoundVolume(
            new Vector3d((Fixed64)99, -Fixed64.One, -Fixed64.One),
            new Vector3d((Fixed64)101, Fixed64.One, Fixed64.One));

        context.Physics2D.QueryPlanarContinuousCollisionCandidates(impactPlanarBounds)
            .Contains(body.DynamicId)
            .Should()
            .BeTrue();
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(supersededPlanarBounds)
            .Contains(body.DynamicId)
            .Should()
            .BeFalse();
        context.Physics2D.QueryMixedContinuousCollisionCandidates(impactMixedBounds)
            .Contains(body.DynamicId)
            .Should()
            .BeTrue();
        context.Physics2D.QueryMixedContinuousCollisionCandidates(supersededMixedBounds)
            .Contains(body.DynamicId)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void HandoffTrajectory2D_ShouldClipPreparedFutureForStopAndReverse()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D stopped = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D reversed = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        Vector2d preparedVelocity = Vector2d.Right * (Fixed64)100;
        stopped.ApplyCollisionLinearVelocityDelta(preparedVelocity);
        reversed.ApplyCollisionLinearVelocityDelta(preparedVelocity);
        context.AdvanceLateSimulateToken();
        stopped.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        reversed.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        Fixed64 impactFraction = Fixed64.FromFraction(1, 4);
        Fixed64 remainingTime = Fixed64.One - impactFraction;
        Vector2d impactPosition = Vector2d.Right * (Fixed64)25;

        stopped.ApplyContinuousCollisionHandoffState(
                impactPosition,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                remainingTime)
            .Should()
            .BeTrue();
        reversed.ApplyContinuousCollisionHandoffState(
                impactPosition,
                Fixed64.Zero,
                -preparedVelocity,
                Fixed64.Zero,
                remainingTime)
            .Should()
            .BeTrue();

        stopped.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 8))
            .Should()
            .Be(Vector2d.Right * Fixed64.FromFraction(25, 2));
        stopped.SampleContinuousCollisionPosition(impactFraction)
            .Should()
            .Be(impactPosition);
        stopped.SampleContinuousCollisionPosition(Fixed64.FromFraction(5, 8))
            .Should()
            .Be(impactPosition);
        reversed.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 8))
            .Should()
            .Be(Vector2d.Right * Fixed64.FromFraction(25, 2));
        reversed.SampleContinuousCollisionPosition(impactFraction)
            .Should()
            .Be(impactPosition);
        reversed.SampleContinuousCollisionPosition(Fixed64.FromFraction(5, 8))
            .Should()
            .Be(Vector2d.Left * Fixed64.FromFraction(25, 2));

        stopped.TryResolveContinuousCollisionMotionBound(
                impactFraction,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 stoppedMotionBound)
            .Should()
            .BeTrue();
        reversed.TryResolveContinuousCollisionMotionBound(
                impactFraction,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 reversedMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 expectedStoppedMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector2d.Left * (Fixed64)75,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 expectedReversedMotionBound)
            .Should()
            .BeTrue();
        stoppedMotionBound.Should().Be(expectedStoppedMotionBound);
        reversedMotionBound.Should().Be(expectedReversedMotionBound);

        DynamicCcdPlanarBounds stoppedBounds =
            stopped.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero);
        DynamicCcdPlanarBounds reversedBounds =
            reversed.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero);
        stoppedBounds.MinX.Should().Be(Fixed64.Zero);
        stoppedBounds.MaxX.Should().Be((Fixed64)25);
        reversedBounds.MinX.Should().Be((Fixed64)(-50));
        reversedBounds.MaxX.Should().Be((Fixed64)25);
    }

    [Fact]
    public void DirtyCandidateReservations2D_ShouldScaleWithRegisteredBodyCount()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        var bodies = new SolidBody2D[12];
        for (int i = 0; i < bodies.Length; i++)
        {
            bodies[i] = CreateBody(
                context,
                new LSCircleCollider2D(Fixed64.Half),
                Vector2d.Right * (Fixed64)(i * 4),
                immovable: false);
        }

        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        for (int i = 0; i < bodies.Length; i += 2)
        {
            context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(
                    bodies[i],
                    bodies[i + 1])
                .Should()
                .BeTrue();
        }
    }

    [Fact]
    public void FrozenAxisHandoff2D_ShouldPublishProjectedPositionAndCandidateBounds()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D mover = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false);
        mover.FreezeAxes = BodyFreezeAxes2D.PositionX;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        Vector2d rawImpactPosition = new((Fixed64)2, (Fixed64)3);
        Vector2d projectedImpactPosition = new(Fixed64.Zero, (Fixed64)3);

        mover.ApplyContinuousCollisionHandoff(
                rawImpactPosition,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        mover.Position.Should().Be(projectedImpactPosition);
        collider.Center.Should().Be(projectedImpactPosition);
        mover.SampleContinuousCollisionPosition(Fixed64.Half)
            .Should()
            .Be(projectedImpactPosition);
        DynamicCcdPlanarBounds projectedImpactBounds = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            projectedImpactPosition,
            projectedImpactPosition,
            Fixed64.FromFraction(1, 10));
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(projectedImpactBounds)
            .Should()
            .Contain(mover.DynamicId);
        DynamicCcdPlanarBounds rawImpactBounds = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            rawImpactPosition,
            rawImpactPosition,
            Fixed64.FromFraction(1, 10));
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(rawImpactBounds)
            .Should()
            .NotContain(mover.DynamicId);
    }

    [Fact]
    public void Handoff2D_WhenRequestedPoseCannotPublishCollider_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            LocalOffset = Vector2d.Right
        };
        SolidBody2D mover = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false);
        Vector2d originalCenter = collider.Center;
        var originalBounds = collider.Bounds;

        mover.ApplyContinuousCollisionHandoff(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeFalse();

        mover.Position.Should().Be(Vector2d.Zero);
        collider.Center.Should().Be(originalCenter);
        collider.Bounds.Should().Be(originalBounds);
    }

    [Fact]
    public void ProxyRadius2D_WhenPublishedPoseLeadsBody_ShouldRemainCanonicalForOffsetCompound()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                Vector2d.Right))
        {
            LocalOffset = Vector2d.Right * (Fixed64)3
        };
        SolidBody2D mover = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false);
        Fixed64 expectedRadius = Fixed64.FromFraction(9, 2);
        mover.ResolveContinuousCollisionProxyRadius()
            .Should()
            .Be(expectedRadius);

        collider.TryPrepareBodyPose(
                Vector2d.Right * (Fixed64)10,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        collider.PublishPreparedBodyPose();

        mover.Position.Should().Be(Vector2d.Zero);
        collider.Center.Should().Be(Vector2d.Right * (Fixed64)13);
        mover.ResolveContinuousCollisionProxyRadius()
            .Should()
            .Be(expectedRadius);
    }

    [Fact]
    public void DirtyCandidateIndex2D_WhenDynamicIdIsReused_ShouldDiscardRetiredBounds()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D original = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        Vector2d retiredExcursion = Vector2d.Right * (Fixed64)10;
        original.ApplyContinuousCollisionHandoff(
                retiredExcursion,
                Vector2d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        int reusedId = original.DynamicId;

        original.Deactivate();
        SolidBody2D replacement = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)10),
            immovable: false);
        replacement.DynamicId.Should().Be(reusedId);
        DynamicCcdPlanarBounds retiredBounds = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            retiredExcursion,
            retiredExcursion,
            Fixed64.One);

        context.Physics2D.QueryPlanarContinuousCollisionCandidates(retiredBounds)
            .Should()
            .NotContain(replacement.DynamicId);
    }

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
    public void HandoffTrajectory2D_ShouldRetainIncreasingExcursionsAndReplaceEqualOrEarlierTail()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 2;
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        mover.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)100);
        context.AdvanceLateSimulateToken();
        mover.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Right * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Left * (Fixed64)200,
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Left * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Right * (Fixed64)50,
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.SampleContinuousCollisionPosition(Fixed64.FromFraction(1, 4))
            .Should()
            .Be(Vector2d.Right * (Fixed64)25);
        mover.SampleContinuousCollisionPosition(Fixed64.Half)
            .Should()
            .Be(Vector2d.Left * (Fixed64)25);
        mover.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero).MinX
            .Should()
            .Be((Fixed64)(-25));
        mover.ResolveContinuousCollisionTrajectoryBounds(Fixed64.Zero).MaxX
            .Should()
            .Be((Fixed64)25);
        mover.TryResolveContinuousCollisionMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                out Fixed64 motionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector2d.Right * (Fixed64)25,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 excursionMotionBound)
            .Should()
            .BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                Vector2d.Left * (Fixed64)50,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out Fixed64 returnMotionBound)
            .Should()
            .BeTrue();
        motionBound.Should().Be(excursionMotionBound * Fixed64.Two + returnMotionBound);

        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Left * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Right * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Right * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Right * (Fixed64)25,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(2);
        mover.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One)
            .Should()
            .BeTrue();
        mover.ContinuousCollisionTrajectoryCount.Should().Be(1);
        mover.SampleContinuousCollisionPosition(Fixed64.One)
            .Should()
            .Be(Vector2d.Zero);
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
