using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DColliderReconfigurationTests
{
    [Fact]
    public void AcceptedReplacement_ShouldRetireHolderOwnedPair()
    {
        using GravitasWorldContext context = CreateContext();
        var trigger = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        context.Simulate();
        context.LateSimulate();
        GetPair(body.Collider, trigger).IsColliding.Should().BeTrue();
        body.Collider.CollisionPairHolderCount.Should().Be(1);
        body.Collider.CollisionPairCount.Should().Be(0);
        int exits = 0;
        body.Collider.OnTriggerExit += _ => exits++;

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One),
            Vector2d.Zero, new Vector2d(5, 0), out _, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        exits.Should().Be(1);
        trigger.CollisionPairCount.Should().Be(0);
        body.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResetDuringSeparation_ShouldReturnAcceptedResultAndOnlyCallbackFailure(bool throwAfterReset)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var first = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        var second = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        context.Simulate();
        context.LateSimulate();
        GetPair(body.Collider, first).IsColliding.Should().BeTrue();
        GetPair(body.Collider, second).IsColliding.Should().BeTrue();
        int exits = 0;
        var callbackFailure = new InvalidOperationException("exit after reset");
        body.Collider.OnTriggerExit += _ =>
        {
            if (++exits != 1) return;
            context.Reset();
            if (throwAfterReset) throw callbackFailure;
        };
        ColliderReconfigurationStatus result = default;
        Exception? failure = null;

        Action operation = () => result = body.TryReconfigureCollider(
            ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(5, 0), out _, out failure);

        operation.Should().NotThrow("reset during notification must not invalidate the retirement snapshot");
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeSameAs(throwAfterReset ? callbackFailure : null);
        exits.Should().BeGreaterThan(0);
        body.Position.Should().Be(new Vector2d(5, 0));
        ((LSCircleCollider2D)body.Collider).ScaledRadius.Should().Be(Fixed64.One);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void AcceptedReplacement_ShouldPublishBeforeCallbacksAndRetireAllPairsDespiteFailures()
    {
        using GravitasWorldContext context = CreateContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = CreateBody(context, circle);
        var first = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        var second = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        var parent = CreateObstacle(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(-10, 0));
        circle.SetParent(parent);
        context.Simulate();
        context.LateSimulate();
        GetPair(circle, first).IsColliding.Should().BeTrue();
        GetPair(circle, second).IsColliding.Should().BeTrue();
        int published = 0;
        int exits = 0;
        var publisherFailure = new InvalidOperationException("publisher");
        circle.OnTriggerExit += other =>
        {
            published.Should().Be(1);
            circle.ScaledRadius.Should().Be(Fixed64.One);
            body.Position.Should().Be(new Vector2d(5, 0));
            exits++;
            throw new InvalidOperationException(ReferenceEquals(other, first) ? "first" : "second");
        };

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(5, 0), out _, out var failure, () =>
            {
                published++;
                context.Query2D.OverlapCircle(new Vector2d(5, 0), Fixed64.Half, out var hit).Should().BeTrue();
                hit.Collider.Should().BeSameAs(circle);
                throw publisherFailure;
            });

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        var failures = failure.Should().BeOfType<AggregateException>().Which.Flatten().InnerExceptions;
        failures.Should().HaveCount(3);
        failures[0].Should().BeSameAs(publisherFailure);
        exits.Should().Be(2);
        circle.Parent2D.Should().BeSameAs(parent);
        circle.CollisionPairCount.Should().Be(0);
        circle.CollisionPairHolderCount.Should().Be(0);
        first.CollisionPairCount.Should().Be(0);
        first.CollisionPairHolderCount.Should().Be(0);
        second.CollisionPairCount.Should().Be(0);
        second.CollisionPairHolderCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SeparationRecycling_ShouldNotRetireAReusedPairLifetime(bool pooling)
    {
        using GravitasWorldContext context = CreateContext();
        context.Settings.PoolingEnabled = pooling;
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var first = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        var second = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        context.Simulate();
        context.LateSimulate();
        CollisionPair2D retiringSecondPair = GetPair(body.Collider, second);
        SolidBody2D? replacement = null;
        CollisionPair2D? replacementPair = null;
        int exits = 0;
        body.Collider.OnTriggerExit += other =>
        {
            exits++;
            if (!ReferenceEquals(other, first)) return;
            second.Deactivate();
            CreateObstacle(context, second, new Vector2d(10, 0));
            replacement = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(10, 0));
            context.LateSimulate();
            replacementPair = GetPair(replacement.Collider, second);
        };

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(5, 0), out _, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        exits.Should().Be(2);
        replacement.Should().NotBeNull();
        replacementPair.Should().NotBeNull();
        GetPair(replacement!.Collider, second).Should().BeSameAs(replacementPair);
        replacementPair!.IsColliding.Should().BeTrue();
        if (pooling) replacementPair.Should().BeSameAs(retiringSecondPair);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PublisherReinitializingBody_ShouldNotClearFreshPairOwnership(bool resetContext)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var trigger = CreateObstacle(context, new LSCircleCollider2D(Fixed64.One) { IsTrigger = true }, Vector2d.Zero);
        context.Simulate();
        context.LateSimulate();
        CollisionPair2D? freshPair = null;

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(5, 0), out _, out var failure, () =>
            {
                body.Deactivate();
                if (resetContext)
                {
                    trigger.Deactivate();
                    context.Reset();
                    CreateObstacle(context, trigger, Vector2d.Zero);
                }
                body.Initialize(Vector2d.Zero);
                context.LateSimulate();
                freshPair = GetPair(body.Collider, trigger);
            });

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        freshPair.Should().NotBeNull();
        GetPair(body.Collider, trigger).Should().BeSameAs(freshPair);
        freshPair!.IsColliding.Should().BeTrue();
    }

    private static CollisionPair2D GetPair(LSCollider2D first, LSCollider2D second)
    {
        if (first.TryGetCollisionPair(second.Id, out CollisionPair2D? pair)) return pair!;
        second.TryGetCollisionPair(first.Id, out pair).Should().BeTrue();
        return pair!;
    }
}
