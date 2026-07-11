using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DSimulationTests
{
    [Fact]
    public void LateSimulate_ShouldIntegratePure2DForceVelocityAndPosition()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        var agent = new TestMatterAgent(context);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = (Fixed64)2
        };
        body.Initialize(Vector2d.Zero);

        body.AddForce(new Vector2d((Fixed64)8, Fixed64.Zero));
        context.LateSimulate();

        body.LinearVelocity.Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        body.Position.Should().Be(new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero));
    }

    [Fact]
    public void LateSimulate_WithDefaultGravityScale_ShouldApplyPlanarGravity()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        body.Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-4));

        context.LateSimulate();

        body.LinearVelocity.Should().Be(new Vector2d(Fixed64.Zero, -Fixed64.One));
        body.Position.Should().Be(new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(1, 4)));
    }

    [Fact]
    public void LateSimulate_WithHalfGravityScale_ShouldApplyScaledPlanarGravity()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        body.Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-4));
        body.GravityScale = Fixed64.Half;

        context.LateSimulate();

        body.LinearVelocity.Should().Be(new Vector2d(Fixed64.Zero, -Fixed64.Half));
        body.Position.Should().Be(new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(1, 8)));
    }

    [Fact]
    public void LateSimulate_WithZeroGravityScale_ShouldIgnorePlanarGravity()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        body.Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-4));
        body.GravityScale = Fixed64.Zero;

        context.LateSimulate();

        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void GravityScale_ShouldRejectNegativeValues()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);

        Action action = () => body.GravityScale = -Fixed64.Epsilon;

        action.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void ContinuousCollisionPrediction_WithGravityScale_ShouldUseScaledPlanarGravity()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        body.Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-4));
        body.GravityScale = Fixed64.Half;

        body.EnsureContinuousCollisionFramePrepared(123);

        body.ContinuousCollisionFrameDisplacement.Should().Be(new Vector2d(
            Fixed64.Zero,
            -Fixed64.FromFraction(1, 8)));
    }

    [Fact]
    public void Simulate_WithOverlapping2DBodies_ShouldResolveContactAndNotifyOnce()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D right = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        int entered = 0;
        int stayed = 0;
        left.Collider.OnContactEnter += _ => entered++;
        left.Collider.OnContact += _ => stayed++;

        Step(context);
        Vector2d resolvedPosition = left.Position;
        Step(context);

        resolvedPosition.X.Should().BeLessThan(Fixed64.Zero);
        left.Position.Should().Be(resolvedPosition);
        entered.Should().Be(1);
        stayed.Should().Be(2);
    }

    [Fact]
    public void LateSimulate_ShouldRefreshMoved2DCollidersAndDistributeContactsAfterIntegration()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D mover = CreateCircle(context, new Vector2d(-Fixed64.FromFraction(5, 4), Fixed64.Zero), immovable: false);
        SolidBody2D target = CreateCircle(context, Vector2d.Zero, immovable: true);
        Vector2d startPosition = mover.Position;
        int entered = 0;
        mover.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(target);
            entered++;
        };

        mover.AddForce(new Vector2d((Fixed64)16, Fixed64.Zero));
        context.Simulate();

        mover.Position.Should().Be(startPosition);
        mover.Collider.TryGetCollisionPair(target.Collider.Id, out _).Should().BeFalse();
        entered.Should().Be(0);

        context.LateSimulate();

        mover.Collider.TryGetCollisionPair(target.Collider.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        pair.LastFrame.Should().Be(context.FrameCount);
        entered.Should().Be(1);
    }

    [Fact]
    public void Deactivate_ShouldRemoveBodyColliderPairsAndQueryVisibility()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        int exited = 0;
        left.Collider.OnContactExit += _ => exited++;

        Step(context);
        left.Deactivate();

        var hits = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAll(Vector2d.Zero, (Fixed64)0.1f, hits).Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(1);
        exited.Should().Be(1);
    }

    [Fact]
    public void Deactivate_NonDynamicBody_ShouldStillRemoveColliderAndPartition()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false, isDynamic: false);

        body.DynamicId.Should().Be(-1);
        context.Physics2D.BodyCount.Should().Be(0);
        body.Collider.IsPartitioned.Should().BeTrue();

        body.Deactivate();

        body.Active.Should().BeFalse();
        body.Collider.IsPartitioned.Should().BeFalse();
        body.Collider.Id.Should().Be(-1);
        context.Physics2D.BodyCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_AfterPhysicsServiceReset_ShouldClearStaleBodyState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);

        context.Physics2D.Reset();

        body.Deactivate();

        body.Active.Should().BeFalse();
        body.Collider.Body.Should().BeNull();
        body.Collider.Id.Should().Be(-1);
        context.Physics2D.BodyCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void DirectSimulation_WhenPhysicsIsDisabled_ShouldPreserveBodyAndBroadPhaseState()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, Fixed64.Half * Vector2d.Right, immovable: true);
        context.LateSimulate();
        body.AddForce(Vector2d.Right);
        Vector2d position = body.Position;
        Vector2d velocity = body.LinearVelocity;
        int candidateCount = context.Physics2D.LastBroadPhaseCandidateCount;
        int lateSimulateToken = context.LateSimulateToken;
        context.Physics2D.SimulatePhysics = false;

        context.Physics2D.Simulate();
        context.Physics2D.LateSimulate();

        candidateCount.Should().BeGreaterThan(0);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(candidateCount);
        body.Position.Should().Be(position);
        body.LinearVelocity.Should().Be(velocity);
        context.LateSimulateToken.Should().Be(lateSimulateToken);
    }

    [Fact]
    public void LateSimulate_DirectCall_ShouldOwnLateStepTokenAdvance()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        body.AddForce(new Vector2d((Fixed64)4, Fixed64.Zero));

        context.Physics2D.LateSimulate();

        context.LateSimulateToken.Should().Be(1);
        body.Position.Should().Be(Fixed64.FromFraction(1, 4) * Vector2d.Right);
    }

    [Fact]
    public void LateSimulate_WithPoolingDisabled_ShouldStillProcessCollisionCandidates()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        context.Settings.PoolingEnabled = false;
        SolidBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D right = CreateCircle(context, Fixed64.Half * Vector2d.Right, immovable: true);

        context.LateSimulate();

        context.Physics2D.LastBroadPhaseCandidateCount.Should().BeGreaterThan(0);
        left.Collider.TryGetCollisionPair(right.Collider.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void CollisionMatrix_ShouldFilter2DCollisionPairs()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        bool[,] matrix =
        {
            { true, false },
            { false, true }
        };
        context.ApplySettings(new PhysicsSettings(4, matrix));

        SolidBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D right = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        left.Collider.Layer = new PhysicsLayer(0);
        right.Collider.Layer = new PhysicsLayer(1);
        int entered = 0;
        left.Collider.OnContactEnter += _ => entered++;

        Step(context);

        left.Position.Should().Be(Vector2d.Zero);
        entered.Should().Be(0);
    }

    [Fact]
    public void TriggerCollider_ShouldNotifyTriggerWithoutResponse()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        LSCircleCollider2D trigger = CreateBodylessCircle(context, Vector2d.Zero);
        SolidBody2D other = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        trigger.IsTrigger = true;
        int triggerEntered = 0;
        int triggerStayed = 0;
        int triggerExited = 0;
        int otherEntered = 0;
        int otherStayed = 0;
        int otherExited = 0;
        int contactEntered = 0;
        trigger.OnTriggerEnter += collider =>
        {
            collider.Should().BeSameAs(other.Collider);
            triggerEntered++;
        };
        trigger.OnTriggerStay += collider =>
        {
            collider.Should().BeSameAs(other.Collider);
            triggerStayed++;
        };
        trigger.OnTriggerExit += collider =>
        {
            collider.Should().BeSameAs(other.Collider);
            triggerExited++;
        };
        other.Collider.OnTriggerEnter += collider =>
        {
            collider.Should().BeSameAs(trigger);
            otherEntered++;
        };
        other.Collider.OnTriggerStay += collider =>
        {
            collider.Should().BeSameAs(trigger);
            otherStayed++;
        };
        other.Collider.OnTriggerExit += collider =>
        {
            collider.Should().BeSameAs(trigger);
            otherExited++;
        };
        trigger.OnContactEnter += _ => contactEntered++;

        Step(context);
        Step(context);
        other.SetPosition(new Vector2d((Fixed64)4, Fixed64.Zero));
        Step(context);

        trigger.Center.Should().Be(Vector2d.Zero);
        triggerEntered.Should().Be(1);
        triggerStayed.Should().Be(2);
        triggerExited.Should().Be(1);
        otherEntered.Should().Be(1);
        otherStayed.Should().Be(2);
        otherExited.Should().Be(1);
        contactEntered.Should().Be(0);
    }

    [Fact]
    public void SleepingBody_RestingAgainstImmovable_ShouldRemainSleepingAndStationary()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        sleeper.Sleep();

        Step(context);

        sleeper.IsSleeping.Should().BeTrue();
        sleeper.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void SleepingBody_ShouldWakeFromForceAndCollision()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D mover = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);

        sleeper.Sleep();
        sleeper.IsSleeping.Should().BeTrue();

        Step(context);
        sleeper.IsSleeping.Should().BeFalse();

        sleeper.Sleep();
        sleeper.AddForce(Vector2d.Right);
        sleeper.IsSleeping.Should().BeFalse();
        mover.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ReplayedPure2DScenario_ShouldProduceSameState()
    {
        (Vector2d position, Vector2d velocity) first = RunReplayScenario();
        (Vector2d position, Vector2d velocity) second = RunReplayScenario();

        second.Should().Be(first);
    }

    private static (Vector2d position, Vector2d velocity) RunReplayScenario()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 8);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);

        for (int i = 0; i < 6; i++)
        {
            body.AddForce(new Vector2d((Fixed64)2, Fixed64.Zero));
            context.Simulate();
            context.LateSimulate();
        }

        return (body.Position, body.LinearVelocity);
    }

    private static GravitasWorldContext CreateContext(int frameRate)
    {
        return Physics2DTestWorld.CreateContext(frameRate);
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static SolidBody2D CreateCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable,
        bool isDynamic = true)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position, isDynamic: isDynamic);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
