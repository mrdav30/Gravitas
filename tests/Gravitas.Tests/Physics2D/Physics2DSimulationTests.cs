using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DSimulationTests
{
    [Fact]
    public void LateSimulate_ShouldIntegratePure2DForceVelocityAndPosition()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        var agent = new TestMatterAgent(context);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
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
    public void Simulate_WithOverlapping2DBodies_ShouldResolveContactAndNotifyOnce()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        StiffBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        StiffBody2D right = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        int entered = 0;
        int stayed = 0;
        left.Collider.OnContactEnter += _ => entered++;
        left.Collider.OnContact += _ => stayed++;

        context.Simulate();
        Vector2d resolvedPosition = left.Position;
        context.Simulate();

        resolvedPosition.X.Should().BeLessThan(Fixed64.Zero);
        left.Position.Should().Be(resolvedPosition);
        entered.Should().Be(1);
        stayed.Should().Be(2);
    }

    [Fact]
    public void Deactivate_ShouldRemoveBodyColliderPairsAndQueryVisibility()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        StiffBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        int exited = 0;
        left.Collider.OnContactExit += _ => exited++;

        context.Simulate();
        left.Deactivate();

        var hits = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAll(Vector2d.Zero, (Fixed64)0.1f, hits).Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(1);
        exited.Should().Be(1);
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

        StiffBody2D left = CreateCircle(context, Vector2d.Zero, immovable: false);
        StiffBody2D right = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        left.Collider.Layer = new PhysicsLayer(0);
        right.Collider.Layer = new PhysicsLayer(1);
        int entered = 0;
        left.Collider.OnContactEnter += _ => entered++;

        context.Simulate();

        left.Position.Should().Be(Vector2d.Zero);
        entered.Should().Be(0);
    }

    [Fact]
    public void TriggerCollider_ShouldNotifyTriggerWithoutResponse()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        StiffBody2D trigger = CreateCircle(context, Vector2d.Zero, immovable: false);
        StiffBody2D other = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        trigger.Collider.IsTrigger = true;
        int triggerEntered = 0;
        int triggerStayed = 0;
        int contactEntered = 0;
        trigger.Collider.OnTriggerEnter += collider =>
        {
            collider.Should().BeSameAs(other.Collider);
            triggerEntered++;
        };
        trigger.Collider.OnContact += _ => triggerStayed++;
        trigger.Collider.OnContactEnter += _ => contactEntered++;

        context.Simulate();
        context.Simulate();

        trigger.Position.Should().Be(Vector2d.Zero);
        triggerEntered.Should().Be(1);
        triggerStayed.Should().Be(2);
        contactEntered.Should().Be(0);
    }

    [Fact]
    public void SleepingBody_RestingAgainstImmovable_ShouldRemainSleepingAndStationary()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        StiffBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        sleeper.Sleep();

        context.Simulate();

        sleeper.IsSleeping.Should().BeTrue();
        sleeper.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void SleepingBody_ShouldWakeFromForceAndCollision()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        StiffBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        StiffBody2D mover = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);

        sleeper.Sleep();
        sleeper.IsSleeping.Should().BeTrue();

        context.Simulate();
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
        StiffBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
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

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position, bool immovable)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }
}
