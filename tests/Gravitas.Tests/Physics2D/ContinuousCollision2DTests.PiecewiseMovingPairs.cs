using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void DynamicRelativeCircleHit_ShouldNotConstructSyntheticEndpoint()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 targetX = Fixed64.MaxValue - Fixed64.FromFraction(3, 2);
        Fixed64 sourceX = targetX - Fixed64.Two;
        context.World.TryAddGrid(
                new GridConfiguration(
                    new Vector3d(targetX - (Fixed64)16, Fixed64.Zero, (Fixed64)(-4)),
                    new Vector3d(Fixed64.MaxValue, Fixed64.Zero, (Fixed64)4)),
                out _)
            .Should()
            .BeTrue();
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(sourceX, Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(targetX, Fixed64.Zero),
            immovable: false);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ApplyCollisionLinearVelocityDelta(Vector2d.Left * (Fixed64)3);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        source.TryGetDynamicRelativeContinuousCollisionHit(
                target,
                source.Position,
                Vector2d.Right,
                Fixed64.Half,
                Fixed64.One,
                Fixed64.Zero,
                out Physics2DHit hit,
                out _)
            .Should()
            .BeTrue();
        hit.Distance.Should().Be(Fixed64.One / (Fixed64)4);
        hit.Normal.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void DynamicRelativeHits_ShouldPreserveOneRawPhysicalDistanceOrdering()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D farther = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)11 + Fixed64.MinIncrement, Fixed64.Zero),
            immovable: false);
        SolidBody2D nearer = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)11, Fixed64.Zero),
            immovable: false);
        Vector2d displacement = Vector2d.Right * (Fixed64)100_000;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        source.TryGetDynamicRelativeContinuousCollisionHit(
                farther,
                source.Position,
                displacement,
                Fixed64.Half,
                (Fixed64)100_000,
                Fixed64.Zero,
                out Physics2DHit fartherHit,
                out Fixed64 fartherClosingSpeed)
            .Should()
            .BeTrue();
        source.TryGetDynamicRelativeContinuousCollisionHit(
                nearer,
                source.Position,
                displacement,
                Fixed64.Half,
                (Fixed64)100_000,
                Fixed64.Zero,
                out Physics2DHit nearerHit,
                out Fixed64 nearerClosingSpeed)
            .Should()
            .BeTrue();

        (fartherHit.Distance - nearerHit.Distance)
            .Should()
            .Be(Fixed64.MinIncrement);
        farther.Collider.Id.Should().BeLessThan(nearer.Collider.Id);
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                nearerHit,
                nearerClosingSpeed,
                true,
                true,
                fartherHit,
                fartherClosingSpeed)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void ContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldBlockTranslationalSource()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        target.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Position.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldReceiveKinematicHandoff()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            immovable: false);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_TargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                new Vector2d(Fixed64.Zero, Fixed64.One),
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Position.X.Should().Be((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_PiecewiseCandidateRegistrationOrder_ShouldNotChangeOutcome()
    {
        var collisionFirst = RunPiecewiseRegistrationScenario(
            collisionTargetFirst: true);
        var decoyFirst = RunPiecewiseRegistrationScenario(
            collisionTargetFirst: false);

        collisionFirst.SourcePosition.Should().Be(decoyFirst.SourcePosition);
        collisionFirst.SourceVelocity.Should().Be(decoyFirst.SourceVelocity);
        collisionFirst.TargetPosition.Should().Be(decoyFirst.TargetPosition);
        collisionFirst.TargetVelocity.Should().Be(decoyFirst.TargetVelocity);
        collisionFirst.ToiIterations.Should().Be(decoyFirst.ToiIterations);
    }

    [Fact]
    public void ContinuousMode_PiecewiseCandidateReplay_ShouldBeRepeatable()
    {
        var first = RunPiecewiseRegistrationScenario(collisionTargetFirst: false);
        var repeated = RunPiecewiseRegistrationScenario(collisionTargetFirst: false);

        repeated.Should().Be(first);
    }

    [Fact]
    public void PiecewiseMovingPairReduction_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false);
        SolidBody2D target = CreatePiecewiseTarget(context, Fixed64.Zero);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
            Vector2d.Zero,
            Fixed64.Zero,
            new Vector2d(Fixed64.Zero, (Fixed64)6),
            Fixed64.Zero,
            Fixed64.Half);

        void ReduceTrajectory()
        {
            _ = source.TryGetDynamicRelativeContinuousCollisionHit(
                target,
                source.Position,
                Vector2d.Right * (Fixed64)10,
                Fixed64.Half,
                (Fixed64)10,
                Fixed64.Zero,
                out _,
                out _);
        }

        ReduceTrajectory();
        AllocationTestHelper.MeasureSteadyState(ReduceTrajectory)
            .Should()
            .Be(0);
    }

    private static (
        Vector2d SourcePosition,
        Vector2d SourceVelocity,
        Vector2d TargetPosition,
        Vector2d TargetVelocity,
        int ToiIterations,
        ChronicleHash ReplayHash) RunPiecewiseRegistrationScenario(
            bool collisionTargetFirst)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false);
        SolidBody2D collisionTarget;
        SolidBody2D decoy;
        if (collisionTargetFirst)
        {
            collisionTarget = CreatePiecewiseTarget(context, Fixed64.Zero);
            decoy = CreatePiecewiseTarget(context, (Fixed64)(-4));
        }
        else
        {
            decoy = CreatePiecewiseTarget(context, (Fixed64)(-4));
            collisionTarget = CreatePiecewiseTarget(context, Fixed64.Zero);
        }

        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        collisionTarget.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        decoy.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        collisionTarget.ApplyContinuousCollisionHandoffState(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(Fixed64.Zero, (Fixed64)6),
            Fixed64.Zero,
            Fixed64.Half);
        decoy.ApplyContinuousCollisionHandoffState(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(Fixed64.Zero, (Fixed64)6),
            Fixed64.Zero,
            Fixed64.Half);

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        return (
            source.Position,
            source.LinearVelocity,
            collisionTarget.Position,
            collisionTarget.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount,
            context.ComputeReplayHash(GravitasReplayHashMode.Authoritative));
    }

    private static SolidBody2D CreatePiecewiseTarget(
        GravitasWorldContext context,
        Fixed64 x) =>
        CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(x, (Fixed64)3),
            immovable: false);
}
