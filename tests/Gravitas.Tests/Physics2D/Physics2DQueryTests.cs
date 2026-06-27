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
        SolidBody2D first = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        SolidBody2D second = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero));
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
        SolidBody2D included = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero), new PhysicsLayer(1));
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
    public void OverlapCircle_ShouldReturnClosestLayerFilteredHit()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero, new PhysicsLayer(0));
        SolidBody2D closest = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero), new PhysicsLayer(1));
        _ = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(1));

        bool hit = context.Query2D.OverlapCircle(
            Vector2d.Zero,
            (Fixed64)4,
            PhysicsLayerMask.FromLayer(1),
            out Physics2DHit queryHit);

        hit.Should().BeTrue();
        queryHit.Collider.Should().BeSameAs(closest.Collider);
    }

    [Fact]
    public void OverlapAabbAll_ShouldUseExactShapeMathAndStableOrdering()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D circle = CreateCircle(context, new Vector2d(-Fixed64.One, Fixed64.Zero));
        SolidBody2D box = CreateBox(context, new Vector2d(Fixed64.One, Fixed64.Zero));
        SolidBody2D polygon = CreatePolygon(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        SolidBody2D capsule = CreateCapsule(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        _ = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapAabbAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)10, (Fixed64)2),
            hits);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, circle.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, box.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, polygon.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, capsule.Collider));
        hits[0].Distance.Should().BeLessThanOrEqualTo(hits[1].Distance);
        hits[1].Distance.Should().BeLessThanOrEqualTo(hits[2].Distance);
        hits[2].Distance.Should().BeLessThanOrEqualTo(hits[3].Distance);
    }

    [Fact]
    public void OverlapAabb_ShouldReturnClosestLayerFilteredHit()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero, new PhysicsLayer(0));
        SolidBody2D included = CreateBox(context, new Vector2d(Fixed64.One, Fixed64.Zero), new PhysicsLayer(1));

        bool hit = context.Query2D.OverlapAabb(
            Vector2d.Zero,
            new Vector2d((Fixed64)4, (Fixed64)4),
            PhysicsLayerMask.FromLayer(1),
            out Physics2DHit queryHit);

        hit.Should().BeTrue();
        queryHit.Collider.Should().BeSameAs(included.Collider);
    }

    [Fact]
    public void OverlapPolygonAll_ShouldIncludeEdgeTouchingAndCompoundOwner()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D edgeTouchingBox = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));
        SolidBody2D compound = CreateCompound(context, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        _ = CreateCircle(context, new Vector2d((Fixed64)5, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapPolygonAll(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            },
            hits);

        count.Should().Be(2);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, edgeTouchingBox.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, compound.Collider));
        hits.Should().OnlyContain(hit => hit.Collider != null);
    }

    [Fact]
    public void OverlapPolygon_ShouldRejectSeparatedTargetsAndReturnClosestHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D inside = CreatePolygon(context, Vector2d.Zero);
        _ = CreateBox(context, new Vector2d((Fixed64)6, Fixed64.Zero));

        bool hit = context.Query2D.OverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.Zero, Fixed64.One)
            },
            out Physics2DHit queryHit);

        hit.Should().BeTrue();
        queryHit.Collider.Should().BeSameAs(inside.Collider);
        queryHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void RaycastAll_ShouldUsePure2DShapeMathAndStableOrdering()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D near = CreateCircle(context, Vector2d.Zero);
        SolidBody2D middle = CreateBox(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        SolidBody2D far = CreatePolygon(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        SolidBody2D capsule = CreateCapsule(context, new Vector2d((Fixed64)9, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)12, Fixed64.Zero),
            hits);

        count.Should().Be(4);
        hits[0].Collider.Should().BeSameAs(near.Collider);
        hits[1].Collider.Should().BeSameAs(middle.Collider);
        hits[2].Collider.Should().BeSameAs(far.Collider);
        hits[3].Collider.Should().BeSameAs(capsule.Collider);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
        hits[1].Distance.Should().BeLessThan(hits[2].Distance);
        hits[2].Distance.Should().BeLessThan(hits[3].Distance);
    }

    [Fact]
    public void OverlapCircleAndPolygon_ShouldIncludeCapsuleTargets()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int circleCount = context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits);
        bool polygonHit = context.Query2D.OverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            },
            out Physics2DHit polygonQueryHit);

        circleCount.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(capsule.Collider);
        polygonHit.Should().BeTrue();
        polygonQueryHit.Collider.Should().BeSameAs(capsule.Collider);
    }

    [Fact]
    public void Raycast_ShouldReturnZeroDistanceWhenSegmentStartsInsideCollider()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

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
        SolidBody2D body = CreatePolygon(context, Vector2d.Zero);
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
        SolidBody2D included = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(1));
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
        SolidBody2D body = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)8, (Fixed64)8));
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

    [Fact]
    public void AreaQueryAll_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        for (int i = 0; i < 64; i++)
            _ = i % 2 == 0
                ? CreatePolygon(context, new Vector2d((Fixed64)i, Fixed64.Zero))
                : CreateCapsule(context, new Vector2d((Fixed64)i, Fixed64.Zero));

        var hits = new SwiftList<Physics2DHit>(64);
        Vector2d center = new((Fixed64)16, Fixed64.Zero);
        Vector2d size = new((Fixed64)32, (Fixed64)4);
        ReadOnlySpan<Vector2d> polygon = stackalloc Vector2d[]
        {
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d((Fixed64)32, -Fixed64.One),
            new Vector2d((Fixed64)32, Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One)
        };
        for (int i = 0; i < 3; i++)
        {
            context.Query2D.OverlapAabbAll(center, size, hits);
            context.Query2D.OverlapPolygonAll(polygon, hits);
        }

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            context.Query2D.OverlapAabbAll(center, size, hits);
            context.Query2D.OverlapPolygonAll(
                stackalloc Vector2d[]
                {
                    new Vector2d(Fixed64.Zero, -Fixed64.One),
                    new Vector2d((Fixed64)32, -Fixed64.One),
                    new Vector2d((Fixed64)32, Fixed64.One),
                    new Vector2d(Fixed64.Zero, Fixed64.One)
                },
                hits);
        });

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void CapsuleQueryPaths_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        for (int i = 0; i < 64; i++)
            _ = CreateCapsule(context, new Vector2d((Fixed64)(i * 2), Fixed64.Zero));

        var hits = new SwiftList<Physics2DHit>(64);
        Vector2d start = new((Fixed64)(-4), Fixed64.Zero);
        Vector2d end = new((Fixed64)140, Fixed64.Zero);
        for (int i = 0; i < 3; i++)
        {
            context.Query2D.RaycastAll(start, end, hits);
            context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits);
            context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)16, Fixed64.Zero), (Fixed64)32, hits);
        }

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            context.Query2D.RaycastAll(start, end, hits);
            context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits);
            context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)16, Fixed64.Zero), (Fixed64)32, hits);
        });

        allocatedBytes.Should().Be(0);
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position)
    {
        return CreateCircle(context, position, new PhysicsLayer());
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateBox(GravitasWorldContext context, Vector2d position)
    {
        return CreateBox(context, position, new PhysicsLayer());
    }

    private static SolidBody2D CreateBox(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        return CreateBox(context, position, layer, Vector2d.One);
    }

    private static SolidBody2D CreateBox(GravitasWorldContext context, Vector2d position, PhysicsLayer layer, Vector2d size)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreatePolygon(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(
            agent,
            new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateCapsule(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateCompound(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(
            agent,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.One, Fixed64.Zero))))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
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
