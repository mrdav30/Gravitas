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

public sealed class Physics2DQueryTests
{
    [Fact]
    public void OverlapCircleAll_ShouldUsePure2DShapeMathAndStableOrdering()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D first = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        StiffBody2D second = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        _ = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAll(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            (Fixed64)3,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
        hits[0].Distance.Should().BeLessThanOrEqualTo(hits[1].Distance);
    }

    [Fact]
    public void OverlapCircleAll_WithLayerMask_ShouldFilter2DHits()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.Zero), new PhysicsLayer(0));
        StiffBody2D included = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero), new PhysicsLayer(1));
        _ = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(2));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAll(
            Vector2d.Zero,
            (Fixed64)4,
            PhysicsLayerMask.FromLayer(1),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
    }

    [Fact]
    public void RaycastAll_ShouldUsePure2DShapeMathAndStableOrdering()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D near = CreateCircle(context, Vector2d.Zero);
        StiffBody2D middle = CreateBox(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        StiffBody2D far = CreatePolygon(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            hits);

        count.Should().Be(3);
        hits[0].Collider.Should().BeSameAs(near.Collider);
        hits[1].Collider.Should().BeSameAs(middle.Collider);
        hits[2].Collider.Should().BeSameAs(far.Collider);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
        hits[1].Distance.Should().BeLessThan(hits[2].Distance);
    }

    [Fact]
    public void Raycast_ShouldReturnZeroDistanceWhenSegmentStartsInsideCollider()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = context.Query2D.Raycast(
            Vector2d.Zero,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(body.Collider);
        rayHit.Distance.Should().Be(Fixed64.Zero);
        rayHit.Point.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Raycast_WithHorizontalSegmentAcrossPolygonEdges_ShouldNotDivideByZero()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D body = CreatePolygon(context, Vector2d.Zero);
        Vector2d start = new((Fixed64)(-3), Fixed64.Zero);
        Vector2d end = new((Fixed64)3, Fixed64.Zero);

        for (int i = 0; i < 256; i++)
        {
            bool hit = QueryDetection2D.TryRaycast(start, end, body.Collider, out Physics2DHit rayHit);

            hit.Should().BeTrue();
            rayHit.Collider.Should().BeSameAs(body.Collider);
        }
    }

    [Fact]
    public void RaycastAll_WithLayerMask_ShouldFilter2DHits()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero, new PhysicsLayer(0));
        StiffBody2D included = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(1));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            PhysicsLayerMask.FromLayer(1),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
    }

    [Fact]
    public void RaycastAll_WithColliderSpanningMultipleVoxels_ShouldReturnOneHit()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D body = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)8, (Fixed64)8));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-8), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void RaycastAll_WithZeroLengthSegment_ShouldReturnNoHits()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(Vector2d.Zero, Vector2d.Zero, hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void RaycastAll_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        for (int i = 0; i < 64; i++)
            _ = CreateCircle(context, new Vector2d((Fixed64)i, Fixed64.Zero));

        var hits = new SwiftList<Physics2DHit>(64);
        Vector2d start = new((Fixed64)(-8), Fixed64.Zero);
        Vector2d end = new((Fixed64)80, Fixed64.Zero);
        for (int i = 0; i < 3; i++)
            context.Query2D.RaycastAll(start, end, hits);

        long allocatedBytes = MeasureAllocatedBytes(() => context.Query2D.RaycastAll(start, end, hits));

        allocatedBytes.Should().Be(0);
    }

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position)
    {
        return CreateCircle(context, position, new PhysicsLayer());
    }

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static StiffBody2D CreateBox(GravitasWorldContext context, Vector2d position)
    {
        return CreateBox(context, position, new PhysicsLayer());
    }

    private static StiffBody2D CreateBox(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        return CreateBox(context, position, layer, Vector2d.One);
    }

    private static StiffBody2D CreateBox(GravitasWorldContext context, Vector2d position, PhysicsLayer layer, Vector2d size)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static StiffBody2D CreatePolygon(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(
            agent,
            new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)))
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(position);
        return body;
    }

    private static GravitasWorldContext Create2DContext()
    {
        return Physics2DTestWorld.CreateContext();
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
