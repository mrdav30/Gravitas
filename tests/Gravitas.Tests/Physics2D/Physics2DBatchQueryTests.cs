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

public sealed class Physics2DBatchQueryTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void RaycastAndSweepCircleBatches_ShouldPreserveRequestOrderAndRanges()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        PhysicsRaycast2DRequest[] rayRequests =
        {
            new(new Vector2d((Fixed64)4, (Fixed64)(-2)), new Vector2d((Fixed64)4, (Fixed64)2), IncludeLayerZero),
            new(Vector2d.Zero, Vector2d.Zero, IncludeLayerZero),
            new(new Vector2d(Fixed64.Zero, (Fixed64)(-2)), new Vector2d(Fixed64.Zero, (Fixed64)2), IncludeLayerZero)
        };
        PhysicsSweepCircle2DRequest[] sweepRequests =
        {
            new(new Vector2d((Fixed64)(-4), Fixed64.Zero), new Vector2d((Fixed64)2, Fixed64.Zero), Fixed64.Half, IncludeLayerZero),
            new(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, IncludeLayerZero)
        };
        Physics2DHit[] closestRayHits = new Physics2DHit[rayRequests.Length];
        Physics2DHit[] closestSweepHits = new Physics2DHit[sweepRequests.Length];
        var rayHits = new SwiftList<Physics2DHit>();
        var sweepHits = new SwiftList<Physics2DHit>();
        PhysicsQueryHitRange[] rayRanges = new PhysicsQueryHitRange[rayRequests.Length];
        PhysicsQueryHitRange[] sweepRanges = new PhysicsQueryHitRange[sweepRequests.Length];

        int rayClosestCount = context.Query2D.RaycastBatch(rayRequests, closestRayHits);
        int rayAllCount = context.Query2D.RaycastAllBatch(rayRequests, rayHits, rayRanges);
        int sweepClosestCount = context.Query2D.SweepCircleBatch(sweepRequests, closestSweepHits);
        int sweepAllCount = context.Query2D.SweepCircleAllBatch(sweepRequests, sweepHits, sweepRanges);

        rayClosestCount.Should().Be(2);
        closestRayHits[0].Collider.Should().BeSameAs(second.Collider);
        closestRayHits[1].Collider.Should().BeNull();
        closestRayHits[2].Collider.Should().BeSameAs(first.Collider);
        rayAllCount.Should().Be(2);
        rayRanges[0].Count.Should().Be(1);
        rayRanges[1].Count.Should().Be(0);
        rayRanges[2].Count.Should().Be(1);
        rayHits[rayRanges[0].Start].Collider.Should().BeSameAs(second.Collider);
        rayHits[rayRanges[2].Start].Collider.Should().BeSameAs(first.Collider);
        sweepClosestCount.Should().Be(1);
        closestSweepHits[0].Collider.Should().BeSameAs(first.Collider);
        closestSweepHits[1].Collider.Should().BeNull();
        sweepAllCount.Should().Be(1);
        sweepRanges[0].Count.Should().Be(1);
        sweepRanges[1].Count.Should().Be(0);
        sweepHits[sweepRanges[0].Start].Collider.Should().BeSameAs(first.Collider);
    }

    [Fact]
    public void AreaBatches_ShouldSupportCircleAabbAndPolygonRequests()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 16);
        SolidBody2D circle = CreateCircle(context, Vector2d.Zero);
        SolidBody2D box = CreateBox(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        SolidBody2D polygon = CreatePolygon(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        PhysicsOverlapCircle2DRequest[] circleRequests =
        {
            new(Vector2d.Zero, (Fixed64)2, IncludeLayerZero),
            new(new Vector2d((Fixed64)3, Fixed64.Zero), (Fixed64)2, IncludeLayerZero)
        };
        PhysicsOverlapAabb2DRequest[] aabbRequests =
        {
            new(Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2), IncludeLayerZero),
            new(new Vector2d((Fixed64)6, Fixed64.Zero), new Vector2d((Fixed64)2, (Fixed64)2), IncludeLayerZero)
        };
        Vector2d[] vertices =
        {
            new Vector2d((Fixed64)(-1), (Fixed64)(-1)),
            new Vector2d((Fixed64)1, (Fixed64)(-1)),
            new Vector2d((Fixed64)1, (Fixed64)1),
            new Vector2d((Fixed64)(-1), (Fixed64)1),
            new Vector2d((Fixed64)5, (Fixed64)(-1)),
            new Vector2d((Fixed64)7, (Fixed64)(-1)),
            new Vector2d((Fixed64)7, (Fixed64)1),
            new Vector2d((Fixed64)5, (Fixed64)1)
        };
        PhysicsOverlapPolygon2DRequest[] polygonRequests =
        {
            new(0, 4, IncludeLayerZero),
            new(4, 4, IncludeLayerZero)
        };
        Physics2DHit[] closest = new Physics2DHit[2];
        var hits = new SwiftList<Physics2DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[2];

        context.Query2D.OverlapCircleBatch(circleRequests, closest).Should().Be(2);
        closest[0].Collider.Should().BeSameAs(circle.Collider);
        closest[1].Collider.Should().BeSameAs(box.Collider);
        context.Query2D.OverlapAabbAllBatch(aabbRequests, hits, ranges).Should().Be(2);
        hits[ranges[0].Start].Collider.Should().BeSameAs(circle.Collider);
        hits[ranges[1].Start].Collider.Should().BeSameAs(polygon.Collider);
        context.Query2D.OverlapPolygonBatch(polygonRequests, vertices, closest).Should().Be(2);
        closest[0].Collider.Should().BeSameAs(circle.Collider);
        closest[1].Collider.Should().BeSameAs(polygon.Collider);
        context.Query2D.OverlapPolygonAllBatch(polygonRequests, vertices, hits, ranges).Should().Be(2);
        hits[ranges[0].Start].Collider.Should().BeSameAs(circle.Collider);
        hits[ranges[1].Start].Collider.Should().BeSameAs(polygon.Collider);
    }

    [Fact]
    public void BatchQueries_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        for (int i = 0; i < 32; i++)
            _ = CreateCircle(context, new Vector2d((Fixed64)i, Fixed64.Zero));

        PhysicsRaycast2DRequest[] rays =
        {
            new(new Vector2d((Fixed64)(-4), Fixed64.Zero), new Vector2d((Fixed64)40, Fixed64.Zero), IncludeLayerZero),
            new(new Vector2d((Fixed64)(-4), Fixed64.One), new Vector2d((Fixed64)40, Fixed64.One), IncludeLayerZero)
        };
        PhysicsOverlapCircle2DRequest[] circles =
        {
            new(new Vector2d((Fixed64)8, Fixed64.Zero), (Fixed64)8, IncludeLayerZero),
            new(new Vector2d((Fixed64)16, Fixed64.Zero), (Fixed64)8, IncludeLayerZero)
        };
        PhysicsSweepCircle2DRequest[] sweeps =
        {
            new(new Vector2d((Fixed64)(-4), Fixed64.Zero), new Vector2d((Fixed64)40, Fixed64.Zero), Fixed64.Half, IncludeLayerZero)
        };
        Physics2DHit[] closest = new Physics2DHit[2];
        var hits = new SwiftList<Physics2DHit>(64);
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[2];

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            context.Query2D.RaycastBatch(rays, closest);
            context.Query2D.RaycastAllBatch(rays, hits, ranges);
            context.Query2D.OverlapCircleBatch(circles, closest);
            context.Query2D.OverlapCircleAllBatch(circles, hits, ranges);
            context.Query2D.SweepCircleAllBatch(sweeps, hits, ranges);
        });

        allocatedBytes.Should().Be(0);
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateBox(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSAABBoxCollider2D(Vector2d.One))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
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
}
