using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void ContinuousMode_ShouldPreventFastCircleTunnelingThroughStaticTargets(ColliderType2D targetShape)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, CreateCollider(targetShape), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.x.Should().Be((Fixed64)4);
        mover.LinearVelocity.x.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DiscreteMode_ShouldKeepExistingFastMovementPath()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.x.Should().Be((Fixed64)10);
    }

    [Fact]
    public void AutoMode_ShouldSweepOnlyWhenMovementExceedsProxyRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D slow = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        StiffBody2D fast = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(Fixed64.Zero, (Fixed64)3), immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, (Fixed64)3), immovable: true);
        slow.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        fast.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        slow.AddForce(new Vector2d(Fixed64.Half, Fixed64.Zero));
        fast.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        slow.Position.x.Should().Be(Fixed64.Half);
        fast.Position.x.Should().Be((Fixed64)4);
    }

    [Fact]
    public void InheritMode_ShouldResolveFromContextDefault()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.x.Should().Be((Fixed64)4);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnOrderedHitsAndFilterExcludedHierarchy()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
        StiffBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        StiffBody2D far = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true);
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
    public void SweepCircleAll_ShouldApplyLayerAndTriggerFilters()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(0));
        StiffBody2D trigger = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(1));
        StiffBody2D included = CreateBody(context, new LSAABBoxCollider2D(Vector2d.One), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(1));
        trigger.Collider.IsTrigger = true;
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
        StiffBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.One), immovable: true);
        StiffBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.One), immovable: true);
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

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable,
        PhysicsLayer layer = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.x, Fixed64.Zero, position.y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateCollider(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)),
            _ => new LSCircleCollider2D(Fixed64.Half)
        };

    private static long MeasureAllocatedBytes(System.Action action)
    {
        System.GC.Collect();
        System.GC.WaitForPendingFinalizers();
        System.GC.Collect();
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        action();
        return System.GC.GetAllocatedBytesForCurrentThread() - before;
    }
}
