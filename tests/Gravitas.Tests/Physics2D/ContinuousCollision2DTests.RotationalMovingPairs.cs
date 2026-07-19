using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    private static readonly Fixed64 RotationalMovingPairQuarterTurn2D =
        FixedMath.DegToRad((Fixed64)90);

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldPushDynamic2DTargetAndReachAuthoredRotation()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        target.Sleep();

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.LateSimulate();

        blade.Rotation.Should().Be(RotationalMovingPairQuarterTurn2D);
        target.IsSleeping.Should().BeFalse();
        (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldClampAgainstBodyless2DTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(
            context,
            isKinematic: true);
        Vector2d targetPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        LSCircleCollider2D target = CreateBodylessCircle(context, targetPosition);
        Fixed64 authoredRotation = FixedMath.DegToRad((Fixed64)90);
        blade.Agent.Transform.LocalRotationXZRadians = authoredRotation;

        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(authoredRotation);
        target.Center.Should().Be(targetPosition);
        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_PureRotation_ShouldIgnoreUnsupported2DTarget(
        bool immovable)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(
            context,
            isKinematic: true);
        if (immovable)
        {
            // An unrepresentable pivot radius deliberately takes the registered-
            // collider fallback instead of relying on shape-specific broad phase.
            blade.Collider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        }
        Vector2d targetPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        SolidBody2D target = CreateBody(
            context,
            new UnsupportedTestCollider2D(),
            targetPosition,
            immovable);

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.LateSimulate();

        blade.Rotation.Should().Be(RotationalMovingPairQuarterTurn2D);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(targetPosition);
    }

    [Fact]
    public void ContinuousMode_PureRotation_ShouldPreferEarlierStaticTargetAtEqualBroadPhaseDistance()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(
            context,
            isKinematic: true);
        Fixed64 targetRadius = Fixed64.FromFraction(16, 5);
        _ = CreateBodylessCircle(
            context,
            Vector2d.Rotate(
                new Vector2d(targetRadius, Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)30)));
        _ = CreateBodylessCircle(
            context,
            Vector2d.Rotate(
                new Vector2d(targetRadius, Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)60)));

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(FixedMath.DegToRad((Fixed64)45));
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_DynamicPureRotation_ShouldTransferMomentumToDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        target.Sleep();

        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);
        context.LateSimulate();

        target.IsSleeping.Should().BeFalse();
        (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousMode_DynamicRotationalMovingPair_ShouldReplayAppendedAngularHandoffTrajectoryDeterministically()
    {
        var first = RunDynamicRotationalMovingPairReplay2D();
        var second = RunDynamicRotationalMovingPairReplay2D();

        first.SourceTrajectoryCount.Should().BeGreaterThan(1);
        first.SourceTrajectoryAngularVelocity.Abs()
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        first.Hash.Should().Be(second.Hash);
    }

    [Fact]
    public void ContinuousMode_PureRotation_ShouldResolveMultipleMovingTargetsInTimeOrder()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        SolidBody2D first = CreateRotationalMovingPairTarget2D(
            context,
            FixedMath.DegToRad((Fixed64)30));
        SolidBody2D second = CreateRotationalMovingPairTarget2D(
            context,
            FixedMath.DegToRad((Fixed64)60));
        first.Sleep();
        second.Sleep();

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        context.LateSimulate();

        first.IsSleeping.Should().BeFalse();
        second.IsSleeping.Should().BeFalse();
        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThanOrEqualTo(2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_StaticRotationalClamp_ShouldReplacePreparedFutureTrajectory(
        bool isKinematic)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);

        if (isKinematic)
            blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;
        else
            blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(RotationalMovingPairQuarterTurn2D);
        blade.SampleContinuousCollisionPosition(Fixed64.One).Should().Be(blade.Position);
        blade.SampleContinuousCollisionRotation(Fixed64.One).Should().Be(blade.Rotation);
        blade.SampleContinuousCollisionAngularVelocity(Fixed64.One).Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_CombinedMotion_ShouldTransferContactPointMomentumToDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        target.Sleep();

        blade.AddLinearImpulse(Vector2d.Right * Fixed64.FromFraction(1, 10));
        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);
        context.LateSimulate();

        target.IsSleeping.Should().BeFalse();
        (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousMode_RotationalResponse_ShouldUseSampledWorldCenterOfMass()
    {
        Vector2d positiveOffsetVelocity = RunRotationalMovingPairWithCenterOfMassOffset(Fixed64.One);
        Vector2d negativeOffsetVelocity = RunRotationalMovingPairWithCenterOfMassOffset(-Fixed64.One);

        positiveOffsetVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        negativeOffsetVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        positiveOffsetVelocity.Should().NotBe(negativeOffsetVelocity);
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldBeIndependentOf2DBodyRegistrationOrder()
    {
        var sourceFirst = RunKinematicRotationalMovingPair2D(targetFirst: false);
        var targetFirst = RunKinematicRotationalMovingPair2D(targetFirst: true);

        sourceFirst.TargetLinearVelocity.MagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        sourceFirst.Should().Be(targetFirst);
    }

    [Fact]
    public void ContinuousCollisionHandoff_WithAngularVelocity_ShouldContinueRemainingRotation()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            RotationalMovingPairQuarterTurn2D,
            Fixed64.Half);

        body.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();
        body.Rotation.Should().Be(RotationalMovingPairQuarterTurn2D * Fixed64.Half);
    }

    [Fact]
    public void DirtyCandidateBounds_ShouldRetainEveryEffectiveTrajectorySegment()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * (Fixed64)10,
            Vector2d.Zero,
            Fixed64.FromFraction(3, 4));
        body.ApplyContinuousCollisionHandoff(
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half);

        DynamicCcdPlanarBounds query = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            Vector2d.Right * (Fixed64)10,
            Vector2d.Right * (Fixed64)10,
            Fixed64.FromFraction(1, 10));
        context.Physics2D.QueryPlanarContinuousCollisionCandidates(query)
            .Should()
            .Contain(body.DynamicId);
    }

    [Fact]
    public void DirtyCandidateRefreshAndQuery_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        DynamicCcdPlanarBounds query = DynamicCcdCandidateIndex2D.CreateBoundsBetween(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One);

        void RefreshAndQuery()
        {
            context.Physics2D.RefreshContinuousCollisionCandidate(body);
            context.Physics2D.QueryPlanarContinuousCollisionCandidates(query);
        }

        RefreshAndQuery();
        MeasureAllocatedBytes(RefreshAndQuery).Should().Be(0);
    }

    [Fact]
    public void MovingTargetHandoff_WhenTrajectoryBudgetIsExhausted_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        body.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        body.ApplyContinuousCollisionHandoff(
                Vector2d.Right,
                Vector2d.Right,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector2d position = body.Position;
        Vector2d velocity = body.LinearVelocity;

        body.ApplyContinuousCollisionHandoff(
                Vector2d.Right * Fixed64.Two,
                Vector2d.Right,
                Fixed64.Half)
            .Should()
            .BeFalse();

        body.Position.Should().Be(position);
        body.LinearVelocity.Should().Be(velocity);
        body.ContinuousCollisionTrajectoryCount.Should().Be(2);
    }

    [Fact]
    public void ContinuousMode_DynamicPureRotationAgainstMovingKinematicTarget_ShouldBeOrderIndependent()
    {
        var sourceFirst = RunDynamicRotationalMovingKinematicPair2D(targetFirst: false);
        var targetFirst = RunDynamicRotationalMovingKinematicPair2D(targetFirst: true);

        sourceFirst.BladeLinearVelocity.MagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        sourceFirst.Should().Be(targetFirst);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_NonRotatingSource_ShouldSampleIntermediateDiscreteKinematicTargetRotation(
        bool sourceTranslates)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 startRotation = -FixedMath.DegToRad((Fixed64)45);
        Fixed64 targetRotation = FixedMath.DegToRad((Fixed64)45);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(
            context,
            bladeCollider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        blade.ResetPosition(Vector2d.Zero, startRotation);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d(Fixed64.FromFraction(31, 10), Fixed64.FromFraction(-1, 10)),
            immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.FreezeAxes = BodyFreezeAxes2D.Rotation;
        Vector2d requestedVelocity = sourceTranslates
            ? Vector2d.Forward * Fixed64.FromFraction(1, 5)
            : Vector2d.Zero;

        blade.Agent.Transform.LocalRotationXZRadians = targetRotation;
        source.AddLinearImpulse(requestedVelocity);
        context.LateSimulate();

        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.LinearVelocity.Should().NotBe(requestedVelocity);
    }

    private static (
        Fixed64 BladeRotation,
        Vector2d TargetPosition,
        Vector2d TargetLinearVelocity,
        Fixed64 TargetAngularVelocity) RunKinematicRotationalMovingPair2D(bool targetFirst)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade;
        SolidBody2D target;
        if (targetFirst)
        {
            target = CreateRotationalMovingPairTarget2D(context);
            blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        }
        else
        {
            blade = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
            target = CreateRotationalMovingPairTarget2D(context);
        }

        target.Sleep();
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMovingPairQuarterTurn2D;

        context.LateSimulate();

        return (
            blade.Rotation,
            target.Position,
            target.LinearVelocity,
            target.AngularVelocity);
    }

    private static (
        Vector2d BladePosition,
        Vector2d BladeLinearVelocity,
        Fixed64 BladeAngularVelocity,
        Vector2d TargetPosition) RunDynamicRotationalMovingKinematicPair2D(bool targetFirst)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade;
        SolidBody2D target;
        Vector2d targetStart = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        if (targetFirst)
        {
            target = CreateBody(
                context,
                new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
                targetStart,
                immovable: false,
                isKinematic: true);
            blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        }
        else
        {
            blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
            target = CreateBody(
                context,
                new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
                targetStart,
                immovable: false,
                isKinematic: true);
        }

        Vector2d targetEnd = targetStart - Vector2d.Right * Fixed64.FromFraction(1, 10);
        target.Agent.Transform.LocalPosition = new Vector3d(
            targetEnd.X,
            Fixed64.Zero,
            targetEnd.Y);
        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);

        context.LateSimulate();

        target.Position.Should().Be(targetEnd);
        return (
            blade.Position,
            blade.LinearVelocity,
            blade.AngularVelocity,
            target.Position);
    }

    private static Vector2d RunRotationalMovingPairWithCenterOfMassOffset(
        Fixed64 centerOfMassOffset)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        blade.LocalCenterOfMassOffset = new Vector2d(Fixed64.Zero, centerOfMassOffset);
        target.Sleep();

        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);
        context.LateSimulate();
        return target.LinearVelocity;
    }

    private static SolidBody2D CreateRotationalMovingPairBlade2D(
        GravitasWorldContext context,
        bool isKinematic)
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: isKinematic);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return blade;
    }

    private static (
        ChronicleHash Hash,
        int SourceTrajectoryCount,
        Fixed64 SourceTrajectoryAngularVelocity) RunDynamicRotationalMovingPairReplay2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMovingPairBlade2D(context, isKinematic: false);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        target.Sleep();
        blade.ApplyCollisionAngularVelocityDelta(RotationalMovingPairQuarterTurn2D);

        context.LateSimulate();

        return (
            context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches),
            blade.ContinuousCollisionTrajectoryCount,
            blade.SampleContinuousCollisionAngularVelocity(Fixed64.One));
    }

    private static SolidBody2D CreateRotationalMovingPairTarget2D(
        GravitasWorldContext context,
        Fixed64? angle = null)
    {
        Vector2d targetPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            angle ?? FixedMath.DegToRad((Fixed64)45));
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            targetPosition,
            immovable: false);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return target;
    }
}
