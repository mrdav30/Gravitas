using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.Compound)]
    public void ContinuousMode_ShouldPreventFastCircleTunnelingThroughStaticTargets(ColliderType2D targetShape)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, CreateCollider(targetShape), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldPreventFastCapsuleMoverTunnelingThroughStaticCircle()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2),
            Vector2d.Zero,
            immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().BeLessThan((Fixed64)5);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldConsumeRemainingFrameTimeAfterSlidingIntoSecondStaticContact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(1, 10), (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, Fixed64.FromFraction(1, 10))),
            new Vector2d((Fixed64)(-1), (Fixed64)3),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)4, (Fixed64)4));
        context.LateSimulate();

        mover.Position.X.Should().BeLessThan(Fixed64.Zero);
        mover.Position.Y.Should().BeGreaterThanOrEqualTo(Fixed64.FromFraction(49, 20));
        mover.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContinuousMode_ToiIterationPath_ShouldEvaluateMoverShapeAtIntermediateTimeOfImpact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSAABBoxCollider2D(Vector2d.One),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(1, 10), (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.FromFraction(1, 10))),
            new Vector2d(Fixed64.FromFraction(-11, 20), (Fixed64)3),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)4, (Fixed64)4));
        context.LateSimulate();

        mover.Position.X.Should().BeLessThan(Fixed64.Zero);
        mover.Position.Y.Should().BeGreaterThanOrEqualTo(Fixed64.FromFraction(49, 20));
        mover.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContinuousMode_WithToiIterationLimit_ShouldExposeDeterministicLimitState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(1, 10), (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, Fixed64.FromFraction(1, 10))),
            new Vector2d((Fixed64)(-1), (Fixed64)3),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)4, (Fixed64)4));
        context.LateSimulate();

        mover.LastContinuousCollisionToiIterationCount.Should().Be(1);
        mover.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        mover.Position.Y.Should().BeLessThan(Fixed64.FromFraction(49, 20));
        mover.LinearVelocity.Y.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_ToiIterationPath_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(1, 10), (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        _ = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, Fixed64.FromFraction(1, 10))),
            new Vector2d((Fixed64)(-1), (Fixed64)3),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        void SimulateToiIterationCcd()
        {
            mover.Sleep();
            mover.SetPosition(new Vector2d((Fixed64)(-2), Fixed64.Zero));
            mover.AddForce(new Vector2d((Fixed64)4, (Fixed64)4));
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateToiIterationCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldUseCompoundOwnerProxyRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        SolidBody2D mover = CreateBody(context, compound, Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithTinyPositiveDynamicCircle_ShouldSkipCcdWithoutDiscardingMotion()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 tinyRadius = Fixed64.FromRaw(1);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(tinyRadius),
            Vector2d.Zero,
            immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right,
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)2);
        context.LateSimulate();

        tinyRadius.Should().BeGreaterThan(Fixed64.Zero);
        tinyRadius.Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        mover.Position.Should().Be(Vector2d.Right * (Fixed64)2);
        mover.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)2);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(0);
        mover.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void DiscreteMode_ShouldKeepExistingFastMovementPath()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void AutoMode_ShouldSweepOnlyWhenMovementExceedsProxyRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D slow = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D fast = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(Fixed64.Zero, (Fixed64)3), immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, (Fixed64)3), immovable: true);
        slow.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        fast.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        slow.AddForce(new Vector2d(Fixed64.Half, Fixed64.Zero));
        fast.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        slow.Position.X.Should().Be(Fixed64.Half);
        fast.Position.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_WithEqualDistanceStaticTargets_ShouldUseLowerColliderId()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Half), immovable: true);
        SolidBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.Half), immovable: true);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(Vector2d.Right * (Fixed64)8);
        context.LateSimulate();

        first.Collider.Id.Should().BeLessThan(second.Collider.Id);
        mover.Position.X.Should().BeLessThan((Fixed64)4);
        mover.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithOpposingDynamicBodies_ShouldClampBothAtSharedTimeOfImpact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D left = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        SolidBody2D right = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (right.Position.X - left.Position.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
        left.LinearVelocity.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        right.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_ShouldIgnoreSiblingTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-3), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        target.Collider.SetParent(source.Collider);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source.Position.X.Should().Be(Fixed64.One);
        source.LinearVelocity.X.Should().Be((Fixed64)4);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_WithMatchingVelocity_ShouldNotClampSource()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-3), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)5);
        target.AddForce(Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)2);
        target.Position.X.Should().Be((Fixed64)5);
        source.LinearVelocity.X.Should().Be((Fixed64)5);
        target.LinearVelocity.X.Should().Be((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_WithConfiguredRestitutionThreshold_ShouldSuppressDynamicBounce()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.RestitutionVelocityThreshold = (Fixed64)5;
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-3), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source.AddForce(Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source.LinearVelocity.X.Should().Be((Fixed64)2);
        target.LinearVelocity.X.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ContinuousMode_WithZeroRestitutionThreshold_ShouldBounceLowSpeedDynamicContact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-3), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source.AddForce(Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source.LinearVelocity.X.Should().Be(Fixed64.Zero);
        target.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_WithChainedDynamicBodies_ShouldWakeAndContinueConnectedIsland()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D middle = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D receiver = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false);
        SolidBody2D driver = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        driver.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Sleep();
        receiver.Sleep();

        driver.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        middle.IsSleeping.Should().BeFalse();
        receiver.IsSleeping.Should().BeFalse();
        receiver.Position.X.Should().BeGreaterThan((Fixed64)2);
        middle.Position.X.Should().BeLessThanOrEqualTo(receiver.Position.X - Fixed64.One);
        driver.Position.X.Should().BeLessThanOrEqualTo(middle.Position.X - Fixed64.One);
        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(2);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithChainedDynamicBodiesAndQueueLimit_ShouldExposeServiceLimitState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D middle = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D receiver = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false);
        SolidBody2D driver = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        driver.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Sleep();
        receiver.Sleep();

        driver.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
    }

    [Fact]
    public void ContinuousHandoff_WithFrozenBody_ShouldNotMoveOrQueue()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        body.FreezeAxes = BodyFreezeAxes2D.Position;

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right,
            Vector2d.Right,
            Fixed64.Half);

        body.Position.Should().Be(Vector2d.Zero);
        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WithNoRemainingMotion_ShouldApplyImmediateStateWithoutQueueing()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D noTime = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D noVelocity = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Forward * (Fixed64)4,
            immovable: false);

        noTime.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Zero);
        noVelocity.ApplyContinuousCollisionHandoff(
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(9, 2)),
            Vector2d.Zero,
            Fixed64.Half);

        noTime.Position.Should().Be(Vector2d.Right * Fixed64.Half);
        noTime.LinearVelocity.Should().Be(Vector2d.Right);
        noTime.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
        noVelocity.Position.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.FromFraction(9, 2)));
        noVelocity.LinearVelocity.Should().Be(Vector2d.Zero);
        noVelocity.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WithQueuedMotion_ShouldHonorDirectConsumeFlags()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);

        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeTrue();
        body.Position.X.Should().Be(Fixed64.One);
        body.LinearVelocity.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void ContinuousHandoff_WithQueuedMotion_ShouldConsumeThroughDirectLateSimulate()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);
        body.LateSimulate();

        body.Position.X.Should().Be(Fixed64.One);
        body.LinearVelocity.Should().Be(Vector2d.Right);
        body.Collider.Center.X.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ContinuousHandoff_WhenUpdatedTwice_ShouldQueueOnceAndConsumeLatestState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.FromFraction(1, 4),
            Vector2d.Right,
            Fixed64.FromFraction(3, 4));
        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.FromFraction(1, 4));

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(1);

        body.Position.Should().Be(Vector2d.Right);
        body.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)2);
        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(0);
    }

    [Fact]
    public void QueryMixedContinuousCollisionCandidates_OutsideMixedMode_ShouldClearSharedCandidateBuffer()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        DynamicCcdPlanarBounds planarBounds = DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.One);
        SwiftList<int> planarCandidates = context.Physics2D.QueryPlanarContinuousCollisionCandidates(planarBounds);
        planarCandidates.Should().ContainSingle().Which.Should().Be(body.DynamicId);
        FixedBoundVolume mixedBounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.One);

        SwiftList<int> mixedCandidates = context.Physics2D.QueryMixedContinuousCollisionCandidates(mixedBounds);

        mixedCandidates.Should().BeSameAs(planarCandidates);
        mixedCandidates.Should().BeEmpty();
    }

    [Fact]
    public void ContinuousHandoff_WhenServiceBudgetIsExhausted_ShouldDiscardPendingBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 0).Should().Be(0);

        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
        body.Position.X.Should().Be(Fixed64.Half);
        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WhenPositiveBudgetIsExhausted_ShouldDiscardUnprocessedBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        first.ApplyContinuousCollisionHandoff(Vector2d.Zero, Vector2d.Right, Fixed64.Half);
        second.ApplyContinuousCollisionHandoff(
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            Vector2d.Right,
            Fixed64.Half);

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(1);

        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
        first.Position.X.Should().Be(Fixed64.Half);
        second.Position.X.Should().Be(Fixed64.Zero);
        second.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WhenQueuedBodyDeactivates_ShouldDiscardPendingBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);

        body.Deactivate();
        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(0);

        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WhenPhysicsServiceResets_ShouldDiscardPendingBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);

        context.Physics2D.Reset();

        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WhenDynamicIdIsReused_ShouldNotConsumeReplacementBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D original = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        int originalId = original.DynamicId;
        original.ApplyContinuousCollisionHandoff(Vector2d.Zero, Vector2d.Right, Fixed64.Half);
        original.Deactivate();

        SolidBody2D replacement = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            immovable: false);
        replacement.DynamicId.Should().Be(originalId);
        replacement.ApplyContinuousCollisionHandoff(
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            Vector2d.Right,
            Fixed64.Half);

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(0);

        replacement.Position.X.Should().Be(Fixed64.Zero);
        replacement.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeTrue();
    }

    [Fact]
    public void ContinuousHandoff_WhenConsumedBeforeServiceDrain_ShouldNotCountAnIsland()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        body.ApplyContinuousCollisionHandoff(Vector2d.Zero, Vector2d.Right, Fixed64.Half);
        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeTrue();

        context.Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(0);

        context.Physics2D.LastContinuousCollisionIslandCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WhenFrozenAfterQueue_ShouldConsumeAtImpactAndRefreshCollider()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);
        body.FreezeAxes = BodyFreezeAxes2D.Position;
        body.LateSimulate();

        body.Position.X.Should().Be(Fixed64.Half);
        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.Collider.Center.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ContinuousHandoff_WhenFrozenAfterQueueAndDirectConsumeWithoutColliderRefresh_ShouldRemainAtImpact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half);
        body.FreezeAxes = BodyFreezeAxes2D.Position;

        body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeTrue();

        body.Position.X.Should().Be(Fixed64.Half);
        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.Collider.Center.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ContinuousMode_WithQuantizedTailAfterContact_ShouldStopAtImpact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 remainingTail = Fixed64.Epsilon * (Fixed64)2;
        Fixed64 impactTime = Fixed64.One - remainingTail;
        Vector2d impactPosition = new(impactTime, impactTime * Fixed64.Half);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            impactPosition + Vector2d.Right,
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.One, Fixed64.Half));
        context.LateSimulate();

        mover.Position.Should().Be(impactPosition);
        mover.LinearVelocity.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Half));
        mover.LastContinuousCollisionToiIterationCount.Should().Be(1);
        mover.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithContactAtFrameEndpoint_ShouldStopWithoutTailIteration()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Vector2d endpoint = new(Fixed64.One, Fixed64.Half);
        SolidBody2D mover = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            endpoint + Vector2d.Right,
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.ApplyCollisionLinearVelocityDelta(endpoint);
        context.LateSimulate();

        mover.Position.Should().Be(endpoint);
        mover.LinearVelocity.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Half));
        mover.LastContinuousCollisionToiIterationCount.Should().Be(1);
        mover.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithHugeMassDynamicPair_ShouldResolveFiniteResponseMass()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 hugeMass = (Fixed64)33_554_432;
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        source.Mass = hugeMass;
        target.Mass = hugeMass;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        source.InverseMass.Should().BeGreaterThan(Fixed64.Zero);
        (source.InverseMass + target.InverseMass).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)2);
        context.LateSimulate();

        source.Position.Should().Be(-Vector2d.Right * Fixed64.FromFraction(3, 4));
        source.LinearVelocity.Should().Be(Vector2d.Right * Fixed64.Half);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Position.Should().Be(Vector2d.Right * Fixed64.FromFraction(3, 4));
        target.LinearVelocity.Should().Be(Vector2d.Right * Fixed64.FromFraction(3, 2));
    }

    [Fact]
    public void ContinuousMode_WithNearSingularFrozenTargetMobility_ShouldUseSourceFallback()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 smallOffset = Fixed64.FromFraction(1, 65536);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, smallOffset),
            immovable: false);
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.Sleep();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)2);
        context.LateSimulate();

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Position.Should().Be(new Vector2d(Fixed64.Zero, smallOffset));
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContinuousMode_WithNearSingularFrozenSourceMobility_ShouldStopZeroTimeIteration()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 smallOffset = Fixed64.FromFraction(1, 65536);
        Vector2d sourceStart = new(-Fixed64.One, smallOffset);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            sourceStart,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        source.FreezeAxes = BodyFreezeAxes2D.PositionX;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Forward * Fixed64.Half);
        target.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)2);

        context.LateSimulate();

        source.Position.Should().Be(sourceStart);
        source.LinearVelocity.Should().Be(Vector2d.Forward * Fixed64.Half);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_ShouldNotClampThinAabbWhenProxyCirclesHitButShapesMiss()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)6, Fixed64.FromFraction(1, 5))),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: false);
        target.Sleep();
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        blade.Position.X.Should().Be((Fixed64)10);
        blade.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslationAndShapeExactMiss_ShouldNotPushDynamicTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: false);
        target.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)6, Fixed64.FromFraction(1, 5))),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)10);
        target.Position.Should().Be(new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)));
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldSkipMovableDynamicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D left = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        SolidBody2D right = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        context.Query2D.LastQueryCandidateCount.Should().Be(0);
        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldIncludeKinematicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false, isKinematic: true);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        context.Query2D.LastQueryCandidateCount.Should().Be(1);
        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldClampBeforeStaticTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.One);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void DiscreteMode_WithMovingKinematic2DHost_ShouldReachHostPoseWithoutCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * (Fixed64)2,
            immovable: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;

        source.Agent.Transform.Position = new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.Should().Be(Vector2d.Right * (Fixed64)4);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithTinyPositiveKinematicCircle_ShouldReachHostPoseWithoutCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 tinyRadius = Fixed64.FromRaw(1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(tinyRadius),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right,
            immovable: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = Vector3d.Right * (Fixed64)2;
        context.LateSimulate();

        tinyRadius.Should().BeGreaterThan(Fixed64.Zero);
        tinyRadius.Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)2);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithKinematic2DSourceAndBroadCornerCandidate_ShouldRejectRelativeCircleMiss()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Vector2d sourceStart = new((Fixed64)(-5), (Fixed64)(-5));
        Vector2d hostTarget = new((Fixed64)5, (Fixed64)5);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            immovable: false);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            sourceStart,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        DynamicCcdPlanarBounds sourceBounds = DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
            sourceStart,
            hostTarget - sourceStart,
            source.ResolveContinuousCollisionProxyRadius());
        DynamicCcdPlanarBounds targetBounds = DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
            target.Position,
            Vector2d.Zero,
            target.ResolveContinuousCollisionProxyRadius());
        bool broadBoundsOverlap = !(sourceBounds.MinX > targetBounds.MaxX
            || sourceBounds.MaxX < targetBounds.MinX
            || sourceBounds.MinZ > targetBounds.MaxZ
            || sourceBounds.MaxZ < targetBounds.MinZ);
        broadBoundsOverlap.Should().BeTrue();

        source.Agent.Transform.Position = new Vector3d(hostTarget.X, Fixed64.Zero, hostTarget.Y);
        context.LateSimulate();

        source.Position.Should().Be(hostTarget);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Position.Should().Be(new Vector2d(Fixed64.Zero, (Fixed64)2));
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void AutoMode_WithShortKinematic2DHostTranslation_ShouldSkipContinuousCollision()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        source.Agent.Transform.Position = new Vector3d(Fixed64.FromFraction(-19, 4), Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be(Fixed64.FromFraction(-19, 4));
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldTransferVelocityToDynamicTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        target.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)5);
        target.Position.X.Should().BeGreaterThan(source.Position.X);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldNotTransferVelocityAcrossFrozenTargetAxis()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithKinematic2DSourceAndNearSingularFrozenTargetMobility_ShouldNotPushTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 smallOffset = Fixed64.FromFraction(1, 65536);
        Vector2d targetPosition = new(Fixed64.Zero, smallOffset);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            targetPosition,
            immovable: false);
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d hostTarget = Vector2d.Right * (Fixed64)5;

        source.Agent.Transform.Position = new Vector3d(hostTarget.X, Fixed64.Zero, hostTarget.Y);
        context.LateSimulate();

        source.Position.Should().Be(hostTarget);
        source.Rotation.Should().Be(Fixed64.Zero);
        source.Agent.Transform.Position.Should().Be(new Vector3d(hostTarget.X, Fixed64.Zero, hostTarget.Y));
        source.Agent.Transform.Rotation.Should().Be(FixedQuaternion.Identity);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(targetPosition);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldRelayDynamicHandoffThroughChain()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D receiver = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false);
        SolidBody2D middle = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        receiver.Sleep();
        middle.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        middle.IsSleeping.Should().BeFalse();
        receiver.IsSleeping.Should().BeFalse();
        middle.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.Position.X.Should().BeGreaterThan((Fixed64)2);
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(2);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldStillClampAtStaticAfterDynamicPush()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-1), Fixed64.Zero), immovable: false);
        target.Sleep();
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().BeLessThan((Fixed64)7);
        source.Position.X.Should().BeLessThanOrEqualTo((Fixed64)2);
        target.Position.X.Should().BeGreaterThan((Fixed64)(-1));
        target.Position.X.Should().BeLessThanOrEqualTo((Fixed64)2);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematic2DHostTranslation_ShouldNotPushDynamicTargetBehindEarlierStaticHit()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: false);
        target.Sleep();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        Fixed64 expectedFirstHitX = -Fixed64.One;
        Fixed64 tolerance = Fixed64.FromFraction(1, 1024);
        source.Position.X.Should().BeGreaterThanOrEqualTo(expectedFirstHitX - tolerance);
        source.Position.X.Should().BeLessThanOrEqualTo(expectedFirstHitX + tolerance);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Position.X.Should().Be((Fixed64)3);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithPlanarDynamicsAtDifferentHostY_ShouldStillClampInPure2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D left = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            hostY: (Fixed64)(-8));
        SolidBody2D right = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            immovable: false,
            hostY: (Fixed64)8);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (right.Position.X - left.Position.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
    }

    [Fact]
    public void ContinuousMode_ShouldClampRotatingThinPolygonBeforeAngularTunnelingThroughStaticCircle()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddAngularImpulse(FixedMath.DegToRad((Fixed64)90) / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(FixedMath.DegToRad((Fixed64)90));
        blade.AngularVelocity.Should().Be(Fixed64.Zero);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_ShouldRefineRotatingThinPolygonAngularToiBeyondPreviousSample()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d(Fixed64.FromFraction(29, 10), Fixed64.FromFraction(13, 20)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.AddAngularImpulse(angularVelocity / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().BeGreaterThan(FixedMath.DegToRad((Fixed64)1));
        blade.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AutoMode_ShouldSkipSmallRotationalArc()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)5);
        blade.AddAngularImpulse(angularVelocity / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().Be(angularVelocity);
        blade.AngularVelocity.Should().Be(angularVelocity);
    }

    [Fact]
    public void ContinuousMode_WithKinematic2DHostRotation_ShouldClampBeforeAngularTunnelingThroughStaticCircle()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false, isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(FixedMath.DegToRad((Fixed64)90));
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Theory]
    [InlineData(179, -179)]
    [InlineData(-179, 179)]
    public void ContinuousMode_WithKinematicYawAcrossSignedBoundary_ShouldUseShortestArc(
        int startDegrees,
        int targetDegrees)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
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
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.SetRotation(FixedMath.DegToRad((Fixed64)startDegrees));
        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)targetDegrees,
            Fixed64.Zero);
        Fixed64 targetRotation = FixedMath.DegToRad(blade.Agent.Transform.EulerAngles.Y);

        context.LateSimulate();

        blade.Rotation.Should().Be(targetRotation);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithKinematicEpsilonRadiusCircle_ShouldApplyHostRotationWithoutRotationalCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: true);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Epsilon),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.SetRotation(Fixed64.Pi);

        context.LateSimulate();

        source.Rotation.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithDynamicEpsilonProxyAndOffsetCenterOfMass_ShouldSkipRotationalCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Epsilon),
            Vector2d.Zero,
            immovable: false);
        source.LocalCenterOfMassOffset = Vector2d.Right;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: true);
        Fixed64 requestedAngularVelocity = Fixed64.HalfPi;
        source.AddAngularImpulse(requestedAngularVelocity / source.EffectiveInverseMomentOfInertia);
        Fixed64 appliedAngularVelocity = source.AngularVelocity;
        source.CanRotate.Should().BeTrue();
        source.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.Epsilon);

        context.LateSimulate();

        source.Rotation.Should().Be(appliedAngularVelocity);
        source.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithSubEpsilonRotationalArc_ShouldSkipRotationalCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 proxyRadius = Fixed64.Epsilon + Fixed64.Epsilon;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(proxyRadius),
            Vector2d.Zero,
            immovable: false);
        source.LocalCenterOfMassOffset = Vector2d.Right;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: true);
        Fixed64 requestedAngularVelocity = Fixed64.Epsilon + Fixed64.Epsilon;
        source.AddAngularImpulse(requestedAngularVelocity / source.EffectiveInverseMomentOfInertia);
        Fixed64 appliedAngularVelocity = source.AngularVelocity;
        appliedAngularVelocity.Should().BeGreaterThan(Fixed64.Epsilon);
        (appliedAngularVelocity.Abs() * source.ResolveContinuousCollisionProxyRadius())
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);

        context.LateSimulate();

        source.Rotation.Should().Be(appliedAngularVelocity);
        source.AngularVelocity.Should().Be(appliedAngularVelocity);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithKinematicSubEpsilonArc_ShouldApplyHostRotationWithoutRotationalCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        Fixed64 smallPositiveValue = Fixed64.Epsilon * (Fixed64)2;
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: true);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(smallPositiveValue),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.SetRotation(smallPositiveValue);
        (smallPositiveValue * smallPositiveValue).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        var candidates = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAgainstStaticAll(
            Vector2d.Zero,
            smallPositiveValue,
            PhysicsLayerMask.All,
            candidates,
            source.Collider,
            includeTriggers: false).Should().Be(1);

        context.LateSimulate();

        source.Rotation.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void AutoMode_WithSmallKinematicRotationalArc_ShouldApplyHostRotationWithoutRotationalCcd()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
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
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)3, Fixed64.Half),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)5,
            Fixed64.Zero);
        Fixed64 expectedRotation = FixedMath.DegToRad(blade.Agent.Transform.EulerAngles.Y);
        context.LateSimulate();

        blade.Rotation.Should().Be(expectedRotation);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void AutoMode_WithLargeKinematicRotationalArc_ShouldClampAtStaticCollision()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
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
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        context.LateSimulate();

        blade.Rotation.Should().BeGreaterThan(Fixed64.Zero);
        blade.Rotation.Should().BeLessThan(FixedMath.DegToRad((Fixed64)90));
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void AutoMode_WithLargeKinematicRotationalArcAndNoStaticCandidates_ShouldApplyHostRotation()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
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
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        context.LateSimulate();

        blade.Rotation.Should().Be(FixedMath.DegToRad((Fixed64)90));
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithKinematic2DHostTranslationAndRotationNearMiss_ShouldRotateFully()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(
            context,
            bladeCollider,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.Agent.Transform.Position = Vector3d.Zero;
        blade.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        context.LateSimulate();

        blade.Rotation.Should().Be(FixedMath.DegToRad((Fixed64)90));
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_Kinematic2DActiveSourcePath_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        void SimulateKinematicCcd()
        {
            source.SetPosition(new Vector2d((Fixed64)(-5), Fixed64.Zero));
            source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
            context.Simulate();
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateKinematicCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampRotatingThinPolygonForAngularNearMiss()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)(-2)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.AddAngularImpulse(angularVelocity / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().Be(angularVelocity);
        blade.AngularVelocity.Should().Be(angularVelocity);
    }

    [Fact]
    public void ContinuousMode_RotationalCandidateOnIgnoredPhysicalLayer_ShouldNotClamp()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.UseManualGrounding();
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);
        Fixed64 requestedAngularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.AddAngularImpulse(requestedAngularVelocity / blade.EffectiveInverseMomentOfInertia);
        Fixed64 appliedAngularVelocity = blade.AngularVelocity;

        context.LateSimulate();

        context.Query2D.LastQueryCandidateCount.Should().Be(1);
        blade.Rotation.Should().Be(appliedAngularVelocity);
        blade.AngularVelocity.Should().Be(appliedAngularVelocity);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampThinPolygonWhenBoundsProxyHitsButSweptShapeMisses()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        blade.Position.X.Should().Be((Fixed64)10);
        blade.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampThinAabbWhenBoundsProxyHitsButSweptShapeMisses()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)6, Fixed64.FromFraction(1, 5))),
            Vector2d.Zero,
            immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        blade.Position.X.Should().Be((Fixed64)10);
        blade.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShouldClampThinPolygonWhenSweptShapeHitsStaticCircle()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        blade.Position.X.Should().BeLessThan((Fixed64)10);
        blade.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampCompoundWhenAggregateProxyHitsButPartsMiss()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)3, Fixed64.Zero)));
        SolidBody2D mover = CreateBody(context, compound, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)10);
        mover.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShapeExactTranslationalPath_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        void SimulateShapeExactCcd()
        {
            blade.Sleep();
            blade.SetPosition(Vector2d.Zero);
            blade.SetRotation(Fixed64.Zero);
            blade.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateShapeExactCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_RotationalPath_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Fixed64 angularImpulse = FixedMath.DegToRad((Fixed64)90) / blade.EffectiveInverseMomentOfInertia;

        void SimulateRotationalCcd()
        {
            blade.SetPosition(Vector2d.Zero);
            blade.SetRotation(Fixed64.Zero);
            blade.AddAngularImpulse(angularImpulse);
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateRotationalCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Physics2DLateSimulate_DirectCalls_ShouldRefreshDynamicCcdFrame()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 128);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)100, Fixed64.Zero), immovable: false);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        context.Physics2D.LateSimulate();
        target.SetPosition(new Vector2d((Fixed64)5, Fixed64.Zero));
        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.Physics2D.LateSimulate();

        mover.Position.X.Should().BeLessThan(target.Position.X);
        mover.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        mover.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
        target.Position.X.Should().BeGreaterThan((Fixed64)5);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void InheritMode_ShouldResolveFromContextDefault()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void InheritMode_WithContextInheritSettings_ShouldUseDiscreteIntegration()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Inherit;
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);

        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        mover.Position.Should().Be(Vector2d.Right * (Fixed64)10);
        mover.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)10);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void InheritMode_ShouldUseTopParentContinuousBeforeContextDefault()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D topParent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-8), Fixed64.Zero), immovable: true);
        SolidBody2D middleParent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-7), Fixed64.Zero), immovable: true);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        topParent.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middleParent.Collider.SetParent(topParent.Collider);
        mover.Collider.SetParent(middleParent.Collider);

        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.Should().Be(Vector2d.Zero);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void InheritMode_ShouldUseParentDiscreteBeforeContextContinuousDefault()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-8), Fixed64.Zero), immovable: true);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        parent.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        mover.Collider.SetParent(parent.Collider);

        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        mover.Position.Should().Be(Vector2d.Right * (Fixed64)10);
        mover.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)10);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldClampAgainstBodylessStaticTarget()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBodylessCircle(context, new Vector2d((Fixed64)5, Fixed64.Zero));
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.Should().Be(Vector2d.Zero);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_WithInitialOverlapMovingAway_ShouldNotClamp()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBodylessCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(-Vector2d.Right * (Fixed64)2);
        context.LateSimulate();

        mover.Position.Should().Be(-Vector2d.Right * (Fixed64)2);
        mover.LinearVelocity.Should().Be(-Vector2d.Right * (Fixed64)2);
        mover.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnOrderedHitsAndFilterExcludedHierarchy()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
        SolidBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D far = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true);
        child.Collider.SetParent(parent.Collider);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            child.Collider,
            includeTriggers: false);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(far.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldIncludeMovableKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        SolidBody2D movable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D kinematic = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false, isKinematic: true);
        SolidBody2D immovable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true);
        LSCircleCollider2D bodyless = CreateBodylessCircle(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, movable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void SweepCircleAgainstStaticAll_ShouldSkipMovableDynamicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(0);
        context.Query2D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainstStaticAll_ShouldIncludeKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        SolidBody2D kinematic = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false, isKinematic: true);
        SolidBody2D immovable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        SolidBody2D nonDynamic = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: false, isDynamic: false);
        LSCircleCollider2D bodyless = CreateBodylessCircle(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(4);
        context.Query2D.LastQueryCandidateCount.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, nonDynamic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void OverlapCircleAgainstStaticAll_ShouldApplyExcludedHierarchyAndTriggerFilters()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        SolidBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        LSCircleCollider2D trigger = CreateBodylessCircle(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        SolidBody2D included = CreateBody(context, new LSAABBoxCollider2D(Vector2d.One), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true);
        child.Collider.SetParent(parent.Collider);
        trigger.IsTrigger = true;
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAgainstStaticAll(
            Vector2d.Zero,
            (Fixed64)5,
            PhysicsLayerMask.All,
            hits,
            child.Collider,
            includeTriggers: false);

        count.Should().Be(1);
        hits.Should().ContainSingle(hit => ReferenceEquals(hit.Collider, included.Collider));
    }

    [Fact]
    public void SweepCircleAll_ShouldApplyLayerAndTriggerFilters()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(0));
        LSCircleCollider2D trigger = CreateBodylessCircle(context, new Vector2d((Fixed64)4, Fixed64.Zero), layer: new PhysicsLayer(1));
        SolidBody2D included = CreateBody(context, new LSAABBoxCollider2D(Vector2d.One), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(1));
        trigger.IsTrigger = true;
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(1),
            hits,
            excludedCollider: null,
            includeTriggers: false);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldOrderHitsByDistanceThenColliderId()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.One), immovable: true);
        SolidBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.One), immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            hits);

        count.Should().Be(2);
        hits[0].Distance.Should().Be(hits[1].Distance);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
    }

    [Fact]
    public void SweepCircleAll_WithHexGrid_ShouldOrderHitsByDistanceThenColliderId()
    {
        using GravitasWorldContext context = CreateHexContext(frameRate: 1);
        SolidBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.One), immovable: true);
        SolidBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.One), immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            hits);

        count.Should().Be(2);
        hits[0].Distance.Should().Be(hits[1].Distance);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnNoHitsForZeroDisplacement()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, hits);
        bool hasClosest = context.Query2D.SweepCircle(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, out _);

        count.Should().Be(0);
        hasClosest.Should().BeFalse();
    }

    [Fact]
    public void SweepCircleAll_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 128);
        for (int i = 0; i < 64; i++)
            _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(i * 2), Fixed64.Zero), immovable: true);

        var hits = new SwiftList<Physics2DHit>(64);
        Vector2d start = new((Fixed64)(-4), Fixed64.Zero);
        Vector2d end = new((Fixed64)140, Fixed64.Zero);
        for (int i = 0; i < 3; i++)
            context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits);

        long allocatedBytes = MeasureAllocatedBytes(() => context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits));

        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateContext(int frameRate, int extent = 32) =>
        Physics2DTestWorld.CreateContext(frameRate, extent);

    private static GravitasWorldContext CreateHexContext(int frameRate, int extent = 32)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;

        GridConfiguration configuration = new(
            new Vector3d((Fixed64)(-extent), Fixed64.Zero, (Fixed64)(-extent)),
            new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent),
            topologyKind: GridTopologyKind.HexPrism,
            topologyMetrics: GridTopologyMetrics.Hex(Fixed64.One, Fixed64.One, HexOrientation.PointyTop));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        return context;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable,
        PhysicsLayer layer = default,
        Fixed64 hostY = default,
        bool isKinematic = false,
        bool isDynamic = true)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, hostY, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        body.Collider.Layer = layer;
        body.Initialize(position, isDynamic: isDynamic);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 hostY = default,
        PhysicsLayer layer = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, hostY, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            Layer = layer
        };
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateCollider(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)),
            ColliderType2D.Compound => new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero)),
            _ => new LSCircleCollider2D(Fixed64.Half)
        };

    private static long MeasureAllocatedBytes(System.Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
