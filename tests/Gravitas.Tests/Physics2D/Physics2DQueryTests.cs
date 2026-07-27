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
    public void OverlapQueries_ShouldRejectDiagonalBoundsCandidateAndClearCallerResults()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D diagonal = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)));
        var hits = new SwiftList<Physics2DHit>();

        context.Query2D.OverlapCircle(Vector2d.Zero, Fixed64.Half, out Physics2DHit circleHit).Should().BeFalse();
        circleHit.Should().Be(default(Physics2DHit));
        context.Query2D.LastQueryCandidateCount.Should().Be(1);

        hits.Add(new Physics2DHit(diagonal.Collider, diagonal.Position, Vector2d.Right, Fixed64.Zero));
        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.Half, hits).Should().Be(0);
        hits.Should().BeEmpty();
        context.Query2D.LastQueryCandidateCount.Should().Be(1);

        context.Query2D.OverlapAabb(Vector2d.Zero, Vector2d.One, out Physics2DHit aabbHit).Should().BeFalse();
        aabbHit.Should().Be(default(Physics2DHit));
        context.Query2D.LastQueryCandidateCount.Should().Be(1);

        context.Query2D.OverlapAabb(diagonal.Position, Vector2d.One, out aabbHit).Should().BeTrue();
        aabbHit.Collider.Should().BeSameAs(diagonal.Collider);

        hits.Add(new Physics2DHit(diagonal.Collider, diagonal.Position, Vector2d.Right, Fixed64.Zero));
        context.Query2D.OverlapAabbAll(Vector2d.Zero, Vector2d.One, hits).Should().Be(0);
        hits.Should().BeEmpty();
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapAabbAll_WithNonPositiveSize_ShouldThrow()
    {
        using GravitasWorldContext context = Create2DContext();
        var hits = new SwiftList<Physics2DHit>();

        Action zeroX = () => context.Query2D.OverlapAabbAll(
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            hits);
        Action negativeY = () => context.Query2D.OverlapAabbAll(
            Vector2d.Zero,
            new Vector2d(Fixed64.One, -Fixed64.One),
            hits);

        zeroX.Should().Throw<ArgumentException>().WithParameterName("size");
        negativeY.Should().Throw<ArgumentException>().WithParameterName("size");
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
    public void OverlapPolygon_WithClockwiseVertices_ShouldRemainValid()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D inside = CreateCircle(context, Vector2d.Zero);

        bool hit = context.Query2D.OverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One)
            },
            out Physics2DHit queryHit);

        hit.Should().BeTrue();
        queryHit.Collider.Should().BeSameAs(inside.Collider);
    }

    [Fact]
    public void ValidateConvexQueryPolygon_WithFullDomainEdges_ShouldUseExactOrientation()
    {
        Vector2d[] vertices =
        {
            new(-(Fixed64)1000, -(Fixed64)1000),
            Vector2d.Zero,
            new(Fixed64.MaxValue, Fixed64.MaxValue - Fixed64.One)
        };

        Action validate = () =>
            QueryDetection2D.ValidateConvexQueryPolygon(vertices);

        validate.Should().NotThrow();
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
    public void RaycastAll_WithFiniteAxisHitsOneSpatialRawApart_ShouldPreserveDistanceOrder()
    {
        using GravitasWorldContext context = Create2DContext();
        const long RawUnitsPerWhole = 4_294_967_296L;
        const long NearCenterRaw = 11L * RawUnitsPerWhole;
        Fixed64 nearCenterX = Fixed64.FromRaw(NearCenterRaw);
        Fixed64 farCenterX = Fixed64.FromRaw(NearCenterRaw + 1L);

        // Register the farther collider first so collider identity cannot mask a
        // distance tie caused by rounding a normalized segment parameter.
        SolidBody2D far = CreateCapsule(
            context,
            new Vector2d(farCenterX, Fixed64.Zero),
            Fixed64.One,
            Fixed64.Two);
        SolidBody2D near = CreateCapsule(
            context,
            new Vector2d(nearCenterX, Fixed64.Zero),
            Fixed64.One,
            Fixed64.Two);
        var hits = new SwiftList<Physics2DHit>(2);

        int count = context.Query2D.RaycastAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)100, Fixed64.Zero),
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near.Collider);
        hits[1].Collider.Should().BeSameAs(far.Collider);
        hits[0].Distance.Should().Be(Fixed64.FromRaw(10L * RawUnitsPerWhole));
        hits[1].Distance.Should().Be(Fixed64.FromRaw((10L * RawUnitsPerWhole) + 1L));
    }

    [Fact]
    public void RaycastAll_WithCompoundCollider_ShouldReturnOwnerAtEarliestPart()
    {
        using GravitasWorldContext context = Create2DContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)3, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)));
        SolidBody2D body = CreateCompound(context, Vector2d.Zero, compound);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
        hits[0].Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hits[0].Point.Should().Be(new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        hits[0].Normal.Should().Be(-Vector2d.Right);
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
    public void QueryVersionWrap_ShouldNotSuppressColliderFromPreviousVersionOneQueries()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        body.Collider.RaycastVersion = 1;
        body.Collider.CircleQueryVersion = 1;
        context.Query2D.RaycastVersion = uint.MaxValue;
        context.Query2D.OverlapQueryVersion = uint.MaxValue;

        bool rayHit = context.Query2D.Raycast(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            out Physics2DHit raycastHit);
        bool overlapHit = context.Query2D.OverlapCircle(
            Vector2d.Zero,
            Fixed64.One,
            out Physics2DHit overlapCircleHit);

        rayHit.Should().BeTrue();
        raycastHit.Collider.Should().BeSameAs(body.Collider);
        overlapHit.Should().BeTrue();
        overlapCircleHit.Collider.Should().BeSameAs(body.Collider);
        context.Query2D.RaycastVersion.Should().Be(1);
        context.Query2D.OverlapQueryVersion.Should().Be(1);
    }

    [Fact]
    public void Reset_ShouldInvalidateLiveColliderQueryVersionsBeforeCounterReuse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        Vector2d start = new((Fixed64)(-2), Fixed64.Zero);
        Vector2d end = new((Fixed64)2, Fixed64.Zero);

        context.Query2D.Raycast(start, end, out _).Should().BeTrue();
        context.Query2D.OverlapCircle(Vector2d.Zero, Fixed64.One, out _).Should().BeTrue();

        context.Query2D.Reset();

        context.Query2D.Raycast(start, end, out Physics2DHit raycastHit).Should().BeTrue();
        context.Query2D.OverlapCircle(Vector2d.Zero, Fixed64.One, out Physics2DHit overlapHit).Should().BeTrue();
        raycastHit.Collider.Should().BeSameAs(body.Collider);
        overlapHit.Collider.Should().BeSameAs(body.Collider);
    }

    [Fact]
    public void Raycast_WithSegmentBoundsOutsideCollider_ShouldRejectBeforeShapeMath()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)2, (Fixed64)2),
            new Vector2d((Fixed64)4, (Fixed64)2),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCircleTangentSegment_ShouldReportBoundaryHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(-Fixed64.One, Fixed64.Half),
            new Vector2d(Fixed64.One, Fixed64.Half),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(body.Collider);
        rayHit.Distance.Should().Be(Fixed64.One);
        rayHit.Point.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Half));
        rayHit.Normal.Should().Be(Vector2d.Forward);
    }

    [Fact]
    public void Raycast_WithCircleTangentBeyondSegment_ShouldRejectOutOfRangeRoot()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Half),
            new Vector2d(-Fixed64.FromFraction(49, 100), Fixed64.Half),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCircleCornerStartMovingAwayInsideBounds_ShouldRejectBeforeQuadratic()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(2, 5)),
            new Vector2d(Fixed64.FromFraction(1, 2), Fixed64.FromFraction(1, 2)),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCircleCornerStartAndNonIntersectingDirectionInsideBounds_ShouldRejectNegativeDiscriminant()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero);

        bool hit = context.Query2D.Raycast(
            new Vector2d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(2, 5)),
            new Vector2d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(6, 5)),
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Raycast_WithCircleRootBeyondShortSegmentInsideBounds_ShouldRejectOutOfRangeDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(2, 5)),
            new Vector2d(Fixed64.FromFraction(-7, 20), Fixed64.FromFraction(2, 5)),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
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
    public void SegmentQueries_WithExtremeEndpoints_ShouldRejectComponentAndMagnitudeOverflow()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);
        Vector2d componentStart = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d componentEnd = new(Fixed64.MaxValue, Fixed64.Zero);
        Vector2d magnitudeEnd = new(Fixed64.MaxValue, Fixed64.MaxValue);
        var rayHits = new SwiftList<Physics2DHit> { new() };
        var sweepHits = new SwiftList<Physics2DHit> { new() };

        context.Query2D.Raycast(componentStart, componentEnd, out _).Should().BeFalse();
        context.Query2D.Raycast(Vector2d.Zero, magnitudeEnd, out _).Should().BeFalse();
        context.Query2D.RaycastAll(componentStart, componentEnd, rayHits).Should().Be(0);
        context.Query2D.RaycastAll(Vector2d.Zero, magnitudeEnd, rayHits).Should().Be(0);
        rayHits.Should().BeEmpty();

        context.Query2D.SweepCircle(componentStart, componentEnd, Fixed64.Half, out _).Should().BeFalse();
        context.Query2D.SweepCircle(Vector2d.Zero, magnitudeEnd, Fixed64.Half, out _).Should().BeFalse();
        context.Query2D.SweepCircleAll(componentStart, componentEnd, Fixed64.Half, sweepHits).Should().Be(0);
        context.Query2D.SweepCircleAll(Vector2d.Zero, magnitudeEnd, Fixed64.Half, sweepHits).Should().Be(0);
        sweepHits.Should().BeEmpty();
        context.Query2D.LastQueryCandidateCount.Should().Be(0);

        QueryDetection2D.TryRaycast(componentStart, componentEnd, target.Collider, out _).Should().BeFalse();
        QueryDetection2D.TryRaycast(Vector2d.Zero, magnitudeEnd, target.Collider, out _).Should().BeFalse();
        QueryDetection2D.TryRaycast(Vector2d.Zero, Vector2d.Zero, target.Collider, out _).Should().BeFalse();
        QueryDetection2D.TrySweepCircle(componentStart, componentEnd, Fixed64.Half, target.Collider, out _).Should().BeFalse();
        QueryDetection2D.TrySweepCircle(Vector2d.Zero, magnitudeEnd, Fixed64.Half, target.Collider, out _).Should().BeFalse();
        QueryDetection2D.TrySweepCircle(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, target.Collider, out _).Should().BeFalse();
    }

    [Fact]
    public void SweepCircle_WithZeroLengthSegment_ShouldReturnNoHit()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, Vector2d.Zero);
        var hits = new SwiftList<Physics2DHit>();

        bool closestHit = context.Query2D.SweepCircle(
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out Physics2DHit closest);
        int allCount = context.Query2D.SweepCircleAll(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, hits);

        closestHit.Should().BeFalse();
        closest.Should().Be(default(Physics2DHit));
        allCount.Should().Be(0);
        hits.Count.Should().Be(0);
        context.Query2D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void CircleQueries_WithUnsupportedCustomCollider_ShouldRejectWithoutFabricatingAHit()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new UnsupportedTestCollider2D();
        collider.InitializeWithNoBody(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    Vector3d.Zero,
                    FixedQuaternion.Identity,
                    Vector3d.One)));

        QueryDetection2D
            .TryOverlapCircle(Vector2d.Zero, Fixed64.One, collider, out Physics2DHit overlapHit)
            .Should()
            .BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));

        QueryDetection2D
            .TrySweepCircle(
                -Vector2d.Right * Fixed64.Two,
                Vector2d.Right * Fixed64.Two,
                Fixed64.Half,
                collider,
                out Physics2DHit sweepHit)
            .Should()
            .BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithLongRepresentableSegment_ShouldReachTarget()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(context, Vector2d.Right * new Fixed64(100_000_000));

        bool hit = QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            Vector2d.Right * new Fixed64(100_000_010),
            Fixed64.Half,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().BeGreaterThan(new Fixed64(99_000_000));
    }

    [Fact]
    public void SweepCircle_WithExtremeRangeCircleCrossing_ShouldPreserveEntryDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);
        var targetCircle = (LSCircleCollider2D)target.Collider;
        Vector2d start = new((Fixed64)(-100_000), Fixed64.Half);
        Vector2d end = new((Fixed64)100_000, Fixed64.Half);
        Fixed64 sourceRadius = Fixed64.Half;
        Fixed64 combinedRadius = targetCircle.ScaledRadius + sourceRadius;
        Fixed64 expectedOffset = FixedMath.Sqrt(
            combinedRadius * combinedRadius - Fixed64.Half * Fixed64.Half);
        Fixed64 expectedDistance = (Fixed64)100_000 - expectedOffset;

        bool hit = QueryDetection2D.TrySweepCircle(
            start,
            end,
            sourceRadius,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Distance.Should().Be(expectedDistance);
        sweepHit.Normal.Should().Be(new Vector2d(-expectedOffset, Fixed64.Half).Normalized);
    }

    [Fact]
    public void RaycastCircle_WithContactExactlyAtSegmentEnd_ShouldPreserveAuthoredEndpoint()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(
            context,
            Vector2d.Right * Fixed64.FromFraction(3, 2));

        bool found = QueryDetection2D.TryRaycast(
            Vector2d.Zero,
            Vector2d.Right,
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point.Should().Be(Vector2d.Right);
        hit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void SweepCircleCapsule_WithContactExactlyAtSegmentEnd_ShouldPreserveAuthoredEndpoint()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);

        bool found = QueryDetection2D.TrySweepCircle(
            Vector2d.Right * (Fixed64)(-2),
            -Vector2d.Right,
            Fixed64.Half,
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point.Should().Be(-Vector2d.Right * Fixed64.Half);
        hit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void SweepCircle_WithUnrepresentableSegmentLength_ShouldReject()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(context, new Vector2d(100, 100));

        bool hit = QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            Fixed64.Half,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithUnrepresentableDisplacementLength_ShouldReject()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCircle(context, Vector2d.Zero);
        SolidBody2D target = CreateCircle(context, new Vector2d(100, 100));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithBoundsOutsideCollider_ShouldRejectBeforeShapeMath()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)2, (Fixed64)2),
            new Vector2d((Fixed64)4, (Fixed64)2),
            Fixed64.Half,
            body.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithSkippedCandidates_ShouldReturnClosestEligibleHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D excluded = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        LSCircleCollider2D trigger = CreateBodylessCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero));
        trigger.IsTrigger = true;
        _ = CreateCircle(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(2));
        SolidBody2D closest = CreateCircle(context, new Vector2d((Fixed64)3, Fixed64.Zero), new PhysicsLayer(1));
        _ = CreateCircle(context, new Vector2d((Fixed64)5, Fixed64.Zero), new PhysicsLayer(1));
        _ = CreateCircle(context, new Vector2d((Fixed64)3, (Fixed64)4), new PhysicsLayer(1));

        bool hit = context.Query2D.SweepCircle(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(1),
            out Physics2DHit sweepHit,
            excluded.Collider,
            includeTriggers: false);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(closest.Collider);
        context.Query2D.LastQueryCandidateCount.Should().BeGreaterThan(1);
    }

    [Fact]
    public void SweepCircleAgainstStaticAll_ShouldSkipDynamicTargetsAndSortHits()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D dynamicTarget = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero));
        dynamicTarget.SetMotionType(BodyMotionType.Dynamic);
        dynamicTarget.FreezeAxes = BodyFreezeAxes2D.None;
        SolidBody2D farStatic = CreateCircle(context, new Vector2d((Fixed64)5, Fixed64.Zero));
        SolidBody2D nearStatic = CreateCircle(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        farStatic.SetMotionType(BodyMotionType.Static);
        nearStatic.SetMotionType(BodyMotionType.Static);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(nearStatic.Collider);
        hits[1].Collider.Should().BeSameAs(farStatic.Collider);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, dynamicTarget.Collider));
    }

    [Fact]
    public void Raycast_WithStartInsideCollider_ShouldReturnZeroDistanceFallbackHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            Vector2d.Zero,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(body.Collider);
        rayHit.Distance.Should().Be(Fixed64.Zero);
        rayHit.Point.Should().Be(Vector2d.Zero);
        rayHit.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void Raycast_WithCircleStartInsideBoundsButMovingAway_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.Half, Fixed64.Half),
            new Vector2d(Fixed64.One, Fixed64.One),
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithStartOverlappingCollider_ShouldReturnZeroDistanceHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            body.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(body.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
        sweepHit.Point.Should().Be(Vector2d.Zero);
        sweepHit.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverOverlappingCircle_ShouldReportZeroDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, -Vector2d.Right * Fixed64.Half);
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d(Fixed64.FromFraction(9, 5), Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAgainstBoxFace_ShouldReportEdgeHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)4));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Point.X.Should().Be(-Fixed64.One);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverOverlappingBox_ShouldReportZeroDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d(-Fixed64.Half, Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverOutsideConvexEdgeSpan_ShouldMiss()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), (Fixed64)3));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAwayFromBoxFace_ShouldMiss()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)4));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverTooShortToReachBoxFace_ShouldMiss()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)4));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            Fixed64.Half * Vector2d.Right,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverStartingOverlapped_ShouldReportZeroDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d(-Fixed64.Half, Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverAgainstBoxFace_ShouldReportConvexSweepHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Point.X.Should().Be(-Fixed64.One);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverAwayFromBox_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithZeroDisplacement_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCircle(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverAgainstCapsule_ShouldReportReverseCapsuleHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Point.X.Should().Be(-Fixed64.Half);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverOverlappingCapsule_ShouldReportZeroDistance()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, -Vector2d.Right * Fixed64.Half);
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverMissingCapsule_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), (Fixed64)4));
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverParallelSeparatedFromBox_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), (Fixed64)3));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCircleMoverAgainstCompound_ShouldReturnOwnerAtNearestPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCircle(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)3, Fixed64.Zero))));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void TrySweepMoverShape_WithCircleMoverMissingCompound_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCircle(context, new Vector2d((Fixed64)(-3), (Fixed64)3));
        SolidBody2D target = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d((Fixed64)2, Fixed64.Zero))));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void OverlapCircle_WithCompoundPartsOutsideQuery_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d((Fixed64)6, Fixed64.Zero))));

        bool hit = QueryDetection2D.TryOverlapCircle(
            Vector2d.Zero,
            Fixed64.Half,
            compound.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void OverlapCircle_WithCompoundParts_ShouldReturnOwnerUsingClosestPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)3, Fixed64.Zero)),
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.One, Fixed64.Zero))));

        bool hit = QueryDetection2D.TryOverlapCircle(
            Vector2d.Zero,
            (Fixed64)4,
            compound.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeTrue();
        overlapHit.Collider.Should().BeSameAs(compound.Collider);
        overlapHit.Distance.Should().Be(Fixed64.Half);
        overlapHit.Point.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.Compound)]
    public void OverlapPolygon_WithSeparatedTarget_ShouldReturnFalse(ColliderType2D targetType)
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = Create2DShape(context, targetType, new Vector2d((Fixed64)5, Fixed64.Zero));

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            },
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, -3)]
    [InlineData(ColliderType2D.Circle, 3)]
    [InlineData(ColliderType2D.ConvexPolygon, -3)]
    [InlineData(ColliderType2D.ConvexPolygon, 3)]
    public void OverlapPolygon_WithTargetSeparatedOnEitherProjectionSide_ShouldReturnFalse(
        ColliderType2D targetType,
        int targetX)
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = Create2DShape(context, targetType, new Vector2d((Fixed64)targetX, Fixed64.Zero));

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            },
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void OverlapPolygon_WithCircleCenterInsideArea_ShouldReturnZeroDistanceFallbackNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            },
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeTrue();
        overlapHit.Collider.Should().BeSameAs(target.Collider);
        overlapHit.Distance.Should().Be(Fixed64.Zero);
        overlapHit.Normal.Should().Be(Vector2d.Right);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void OverlapPolygon_WithDegenerateQueryEdge_ShouldIgnoreZeroAxis(ColliderType2D targetType)
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = Create2DShape(context, targetType, Vector2d.Zero);
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };

        bool hit = QueryDetection2D.TryOverlapPolygon(
            vertices,
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeTrue();
        overlapHit.Collider.Should().BeSameAs(target.Collider);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void OverlapPolygon_WithDegenerateQueryEdgeAndSeparatedTarget_ShouldStillRejectOnRealAxis(ColliderType2D targetType)
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = Create2DShape(context, targetType, new Vector2d((Fixed64)5, Fixed64.Zero));
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };

        bool hit = QueryDetection2D.TryOverlapPolygon(
            vertices,
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCompoundPartsOutsideSegment_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)4)),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Zero, (Fixed64)6))));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            compound.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCompoundBoundsOverlapButPartsOutsideSegment_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, Fixed64.One)),
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, -Fixed64.One))));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            compound.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithCompoundPartsOutsidePath_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)4)),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Zero, (Fixed64)6))));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            compound.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithCompoundBoundsOverlapButPartsOutsidePath_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, Fixed64.One)),
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, -Fixed64.One))));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.FromFraction(1, 4),
            compound.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCompoundMover_ShouldUseEarliestMovingPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCompound(
            context,
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)3)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero)));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), Vector2d.One);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)6, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)3);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void TrySweepMoverShape_WithCompoundMoverPartsOutsideTarget_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCompound(
            context,
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)3)),
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)5))));
        SolidBody2D target = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), Vector2d.One);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)6, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithZeroLengthSegmentInsideBounds_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            Vector2d.Zero,
            Vector2d.Zero,
            body.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithNearZeroSegmentInsideBounds_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            new Vector2d(Fixed64.Epsilon, Fixed64.Zero),
            Fixed64.Half,
            body.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithBroadOverlapButTooShortToReachBoxEdge_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector2d(Fixed64.FromFraction(-13, 10), Fixed64.Zero),
            Fixed64.FromFraction(3, 4),
            box.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithEdgeProjectionOutsideBox_ShouldFallBackToTangentVertexHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One),
            Fixed64.Half,
            box.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(box.Collider);
        sweepHit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        sweepHit.Point.Should().Be(new Vector2d(-Fixed64.Half, Fixed64.Half));
        sweepHit.Normal.Should().Be(Vector2d.Forward);
    }

    [Fact]
    public void SweepCircle_WithBoxBoundsOverlapButNoEdgeOrVertexHit_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d(Fixed64.FromFraction(-7, 10), Fixed64.FromFraction(7, 10)),
            new Vector2d(Fixed64.FromFraction(7, 10), Fixed64.FromFraction(7, 10)),
            Fixed64.FromFraction(1, 10),
            box.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithConvexBoundsOverlapButNoEdgeOrVertexHit_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D diamond = CreatePolygon(
            context,
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), Fixed64.FromFraction(11, 10)),
            new Vector2d((Fixed64)2, Fixed64.FromFraction(11, 10)),
            Fixed64.FromFraction(1, 20),
            diamond.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithConvexEdgeImpactBeyondSegmentLength_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D diamond = CreatePolygon(
            context,
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Fixed64.Half,
            diamond.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithPointSweepAtBoxVertex_ShouldUseOpposingMotionNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), -Fixed64.Half),
            new Vector2d(Fixed64.One, -Fixed64.Half),
            Fixed64.Zero,
            box.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(box.Collider);
        sweepHit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        sweepHit.Point.Should().Be(new Vector2d(-Fixed64.Half, -Fixed64.Half));
        sweepHit.Normal.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void SweepPoint_WithVertexContactExactlyAtSegmentEnd_ShouldPreserveAuthoredEndpoint()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero);
        Vector2d endpoint = new(-Fixed64.Half, -Fixed64.Half);

        bool found = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), -Fixed64.Half),
            endpoint,
            Fixed64.Zero,
            box.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        hit.Point.Should().Be(endpoint);
    }

    [Fact]
    public void Raycast_WithCapsuleBoundsOverlapButSegmentOutsideRoundedBody_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.FromFraction(49, 100), Fixed64.FromFraction(99, 100)),
            new Vector2d(Fixed64.Half, Fixed64.FromFraction(99, 100)),
            capsule.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithCapsuleBoundsOverlapButPathOutsideCombinedRadius_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d(Fixed64.FromFraction(49, 100), Fixed64.FromFraction(99, 100)),
            new Vector2d(Fixed64.Half, Fixed64.FromFraction(99, 100)),
            Fixed64.FromFraction(1, 100),
            capsule.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void Raycast_WithCapsuleAxisHit_ShouldReportCapNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            new Vector2d(Fixed64.Zero, (Fixed64)(-3)),
            capsule.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(capsule.Collider);
        rayHit.Normal.Should().Be(Vector2d.Forward);
        rayHit.Distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void SweepCircle_WithStartInsideCapsule_ShouldReturnZeroDistanceHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.FromFraction(1, 4),
            capsule.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(capsule.Collider);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
        sweepHit.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Raycast_WithOneRawCapsuleRadius_ShouldKeepExactEntryAndRadialNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        Fixed64 radius = Fixed64.FromRaw(1);
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero, radius, (Fixed64)2);

        bool found = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            capsule.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Collider.Should().BeSameAs(capsule.Collider);
        hit.Distance.Should().Be((Fixed64)3 - radius);
        hit.Normal.Should().Be(-Vector2d.Right);
        hit.Point.Should().Be(-Vector2d.Right * radius);
    }

    [Fact]
    public void TrySweepMoverShape_WithCrossedCapsuleSegments_ShouldUseSegmentIntersectionContact()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);
        mover.SetRotation(FixedMath.DegToRad((Fixed64)90));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        sweepHit.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void SweepCircle_WithDegenerateCapsuleTarget_ShouldUseCircleFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero, Fixed64.Half, Fixed64.One);

        QueryDetection2D.TrySweepCircle(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            capsule.Collider,
            out Physics2DHit overlapHit).Should().BeTrue();

        overlapHit.Collider.Should().BeSameAs(capsule.Collider);
        overlapHit.Distance.Should().Be(Fixed64.Zero);
        overlapHit.Point.Should().Be(Vector2d.Zero);

        QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            capsule.Collider,
            out Physics2DHit sweepHit).Should().BeTrue();

        sweepHit.Collider.Should().BeSameAs(capsule.Collider);
        // The zero-length centered axis is an exact circle. Contact reporting
        // therefore uses its canonical radial normal at the exact spatial entry.
        Vector2d expectedNormal = -Vector2d.Right;
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Point.Should().Be(expectedNormal * ((LSCapsuleCollider2D)capsule.Collider).ScaledRadius);
        sweepHit.Normal.Should().Be(expectedNormal);
    }

    [Fact]
    public void Raycast_WithRoundedEndpointSlightlyOutsideDegenerateCapsule_ShouldRejectExactMiss()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero, Fixed64.Half, Fixed64.One);
        Vector2d direction = new Vector2d(
            Fixed64.One,
            Fixed64.FromFraction(1, 65536)).Normalized;
        Vector2d start = -direction * Fixed64.FromFraction(3, 2);
        Vector2d end = -direction * Fixed64.Half;

        bool hit = QueryDetection2D.TryRaycast(start, end, capsule.Collider, out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithLargeCapsuleProjectionCoefficients_ShouldKeepFiniteEntry()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero);
        Vector2d start = new((Fixed64)(-200_000), Fixed64.Zero);
        Vector2d end = new((Fixed64)200_000, Fixed64.Zero);

        bool found = QueryDetection2D.TrySweepCircle(
            start,
            end,
            Fixed64.One,
            capsule.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Collider.Should().BeSameAs(capsule.Collider);
        hit.Distance.Should().Be((Fixed64)199_998 + Fixed64.Half);
        hit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void Raycast_WithTwoRawTransverseChord_ShouldRetainTheCapsuleTangent()
    {
        using GravitasWorldContext context = Create2DContext();
        Fixed64 radius = Fixed64.FromRaw(1);
        SolidBody2D capsule = CreateCapsule(
            context,
            new Vector2d(Fixed64.Zero, Fixed64.FromRaw(2)),
            radius,
            radius * Fixed64.Two);
        Vector2d start = new((Fixed64)(-100_000), Fixed64.Zero);
        Vector2d end = new((Fixed64)100_000, Fixed64.FromRaw(2));

        bool found = QueryDetection2D.TryRaycast(
            start,
            end,
            capsule.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Collider.Should().BeSameAs(capsule.Collider);
        hit.Distance.Should().Be((Fixed64)100_000);
        hit.Point.Should().Be(new Vector2d(Fixed64.Zero, radius));
        hit.Normal.Should().Be(-Vector2d.Forward);
    }

    [Fact]
    public void Raycast_WithCompoundParts_ShouldReturnOwnerUsingNearestPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d((Fixed64)3, Fixed64.Zero))));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            compound.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(compound.Collider);
        rayHit.Distance.Should().Be(Fixed64.FromFraction(7, 2));
        rayHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void Raycast_WithCompoundPartMissBeforeHit_ShouldReturnOwnerAtHitPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)3)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero)));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            compound.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(compound.Collider);
        rayHit.Distance.Should().Be(Fixed64.FromFraction(7, 2));
        rayHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void SweepCircle_WithCompoundParts_ShouldReturnOwnerUsingNearestPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d((Fixed64)3, Fixed64.Zero))));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            compound.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(compound.Collider);
        sweepHit.Distance.Should().Be((Fixed64)3);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void SweepCircle_WithCompoundPartMissBeforeHit_ShouldReturnOwnerAtHitPart()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)3)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero)));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            compound.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(compound.Collider);
        sweepHit.Distance.Should().Be((Fixed64)3);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void Raycast_WithBoxCrossingTwoEdges_ShouldUseEntryEdgeOnly()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            box.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(box.Collider);
        Vector2d.DistanceSquared(rayHit.Point, new Vector2d(-Fixed64.One, Fixed64.Zero)).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (rayHit.Distance - (Fixed64)2).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        rayHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void Raycast_WithBoxFromRight_ShouldFlipEntryNormalAgainstRayDirection()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D box = CreateBox(context, Vector2d.Zero, new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            box.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(box.Collider);
        Vector2d.DistanceSquared(rayHit.Point, new Vector2d(Fixed64.One, Fixed64.Zero)).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (rayHit.Distance - (Fixed64)2).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        rayHit.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void OverlapPolygon_WithDiamondAreaSeparatedOnlyOnTargetAxis_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateBox(
            context,
            new Vector2d(Fixed64.FromFraction(8, 5), Fixed64.Zero),
            new PhysicsLayer(),
            Vector2d.One);

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d(Fixed64.Zero, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.Zero),
                new Vector2d(Fixed64.Zero, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.Zero)
            },
            Vector2d.Zero,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeFalse();
        overlapHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void OverlapPolygon_WithSeparatedCircleCapsuleAndConvexTargets_ShouldRejectOnShapeAxes()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D[] targets =
        {
            CreateCircle(context, new Vector2d(Fixed64.FromFraction(9, 5), Fixed64.Zero)).Collider,
            CreateCapsule(context, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(11, 5))).Collider,
            CreatePolygon(
                context,
                new Vector2d(Fixed64.FromFraction(8, 5), Fixed64.Zero),
                new Vector2d(Fixed64.Zero, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Zero),
                new Vector2d(Fixed64.Zero, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Zero)).Collider
        };

        foreach (LSCollider2D target in targets)
        {
            bool hit = QueryDetection2D.TryOverlapPolygon(
                stackalloc[]
                {
                    new Vector2d(-Fixed64.Half, -Fixed64.Half),
                    new Vector2d(Fixed64.Half, -Fixed64.Half),
                    new Vector2d(Fixed64.Half, Fixed64.Half),
                    new Vector2d(-Fixed64.Half, Fixed64.Half)
                },
                Vector2d.Zero,
                target,
                out Physics2DHit overlapHit);

            hit.Should().BeFalse();
            overlapHit.Should().Be(default(Physics2DHit));
        }
    }

    [Fact]
    public void Raycast_WithConvexBoundsOverlapButSegmentOutsideShape_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D diamond = CreatePolygon(
            context,
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));

        bool hit = QueryDetection2D.TryRaycast(
            new Vector2d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(4, 5)),
            new Vector2d(Fixed64.One, Fixed64.FromFraction(4, 5)),
            diamond.Collider,
            out Physics2DHit rayHit);

        hit.Should().BeFalse();
        rayHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithPolygonMoverMissingCircle_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), (Fixed64)3));
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithConvexMoverParallelAxisSeparation_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateBox(context, new Vector2d((Fixed64)(-3), (Fixed64)2));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithConvexCornerInsideSweptBoundsButOutsideRadius_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreatePolygon(context, Vector2d.Zero);

        bool hit = context.Query2D.SweepCircle(
            new Vector2d(Fixed64.FromFraction(-7, 10), Fixed64.FromFraction(7, 10)),
            new Vector2d(Fixed64.FromFraction(-29, 50), Fixed64.FromFraction(29, 50)),
            Fixed64.FromFraction(1, 10),
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void TrySweepMoverShape_WithSeparationOnlyOnRotatedTargetAxis_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, Vector2d.Zero);
        SolidBody2D target = CreatePolygon(
            context,
            new Vector2d(Fixed64.FromFraction(6, 5), Fixed64.FromFraction(6, 5)));
        target.SetRotation(FixedMath.DegToRad((Fixed64)45));

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d(Fixed64.One, -Fixed64.One),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithConvexEntryBeyondUnitInterval_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreatePolygon(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            Vector2d.Right,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleBelowConvexEdgeSpan_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), (Fixed64)(-3)));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void SweepCircle_WithSlantedEdgeImpactBeyondSegmentLength_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreatePolygon(
            context,
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), Fixed64.FromFraction(7, 5)),
            new Vector2d(Fixed64.FromFraction(-7, 5), Fixed64.FromFraction(7, 5)),
            Fixed64.Half,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithConvexMoverBelowParallelTargetAxis_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateBox(context, new Vector2d((Fixed64)(-3), (Fixed64)(-2)));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void OverlapPolygon_WithVertexOnCapsuleCore_ShouldUseClosestAxisFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);
        Vector2d center = new(Fixed64.FromFraction(1, 6), Fixed64.Zero);

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                Vector2d.Zero,
                new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(-1, 4)),
                new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4))
            },
            center,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeTrue();
        overlapHit.Collider.Should().BeSameAs(target.Collider);
        overlapHit.Point.Should().Be(center);
        Vector2d.DistanceSquared(overlapHit.Normal, Vector2d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        overlapHit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void OverlapPolygon_WithClosestAxisOutsideCapsuleCore_ShouldReportSideHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);
        Vector2d center = new(Fixed64.FromFraction(11, 20), Fixed64.Zero);

        bool hit = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d(Fixed64.FromFraction(9, 20), Fixed64.FromFraction(-1, 10)),
                new Vector2d(Fixed64.FromFraction(13, 20), Fixed64.FromFraction(-1, 10)),
                new Vector2d(Fixed64.FromFraction(13, 20), Fixed64.FromFraction(1, 10)),
                new Vector2d(Fixed64.FromFraction(9, 20), Fixed64.FromFraction(1, 10))
            },
            center,
            target.Collider,
            out Physics2DHit overlapHit);

        hit.Should().BeTrue();
        overlapHit.Collider.Should().BeSameAs(target.Collider);
        overlapHit.Point.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        Vector2d.DistanceSquared(overlapHit.Normal, Vector2d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (overlapHit.Distance - Fixed64.FromFraction(1, 20)).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)4);
    }

    [Fact]
    public void SweepCircle_WithReversedWindingConvexEdge_ShouldFlipOutwardNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D target = CreatePolygon(
            context,
            Vector2d.Zero,
            new Vector2d(-Fixed64.Half, -Fixed64.Half),
            new Vector2d(-Fixed64.Half, Fixed64.Half),
            new Vector2d(Fixed64.Half, Fixed64.Half),
            new Vector2d(Fixed64.Half, -Fixed64.Half));

        bool hit = QueryDetection2D.TrySweepCircle(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Point.Should().Be(new Vector2d(-Fixed64.Half, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Distance.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAgainstCircle_ShouldUseReversePointCapsuleHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAgainstBoxEdge_ShouldUseSegmentEdgeHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Point.X.Should().Be(-Fixed64.Half);
    }

    [Fact]
    public void TrySweepMoverShape_WithOffsetCapsuleMoverAgainstBoxFace_ShouldUseCenteredSideSurfacePoint()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 4)));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
        sweepHit.Point.X.Should().Be(-Fixed64.Half);
        sweepHit.Point.Y.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAgainstBoxCorner_ShouldUseReverseVertexHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), (Fixed64)(-3)));
        SolidBody2D target = CreateBox(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, (Fixed64)4),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        sweepHit.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TrySweepMoverShape_WithExtremeReverseEndpointOverflow_ShouldRejectCandidate()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(
            context,
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero));
        SolidBody2D target = CreateCircle(
            context,
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero));

        bool found = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            -Vector2d.Right,
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeFalse();
        hit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCircleEndpointOutsideScalarDomain_ShouldReject()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCircle(
            context,
            new Vector2d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero));
        SolidBody2D target = CreateCircle(context, Vector2d.Zero);

        bool found = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            Vector2d.Right * Fixed64.Two,
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeFalse();
        hit.Should().Be(default(Physics2DHit));
    }

    [Fact]
    public void TrySweepMoverShape_WithCapsuleMoverAgainstCapsule_ShouldUseNearestCapHit()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D mover = CreateCapsule(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCapsule(context, Vector2d.Zero);

        bool hit = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            target.Collider,
            out Physics2DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target.Collider);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void QueryDetection2D_WithBroadShapeMatrix_ShouldRemainDeterministicAndReturnValidHits()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D[] targets =
        {
            CreateCircle(context, Vector2d.Zero).Collider,
            CreateBox(context, new Vector2d((Fixed64)3, Fixed64.Zero), new PhysicsLayer(), new Vector2d((Fixed64)2, (Fixed64)2)).Collider,
            CreateCapsule(context, new Vector2d(Fixed64.Zero, (Fixed64)3)).Collider,
            CreatePolygon(
                context,
                new Vector2d((Fixed64)3, (Fixed64)3),
                new Vector2d(Fixed64.Zero, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.Zero),
                new Vector2d(Fixed64.Zero, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.Zero)).Collider
        };
        (Vector2d Start, Vector2d End)[] rays =
        {
            (new Vector2d((Fixed64)(-4), Fixed64.Zero), new Vector2d((Fixed64)5, Fixed64.Zero)),
            (new Vector2d((Fixed64)5, Fixed64.Zero), new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            (Vector2d.Zero, new Vector2d((Fixed64)4, Fixed64.Zero)),
            (new Vector2d((Fixed64)(-4), (Fixed64)2), new Vector2d((Fixed64)5, (Fixed64)2)),
            (new Vector2d((Fixed64)(-2), (Fixed64)4), new Vector2d((Fixed64)5, (Fixed64)4)),
            (new Vector2d((Fixed64)3, (Fixed64)(-2)), new Vector2d((Fixed64)3, (Fixed64)5))
        };
        (Vector2d Start, Vector2d End, Fixed64 Radius)[] circleSweeps =
        {
            (new Vector2d((Fixed64)(-4), Fixed64.Zero), new Vector2d((Fixed64)5, Fixed64.Zero), Fixed64.Half),
            (new Vector2d((Fixed64)5, Fixed64.Zero), new Vector2d((Fixed64)6, Fixed64.Zero), Fixed64.Half),
            (new Vector2d((Fixed64)(-1), (Fixed64)3), new Vector2d((Fixed64)4, (Fixed64)3), Fixed64.FromFraction(1, 4)),
            (new Vector2d((Fixed64)3, (Fixed64)(-3)), new Vector2d((Fixed64)3, (Fixed64)5), Fixed64.FromFraction(1, 3)),
            (new Vector2d((Fixed64)(-4), (Fixed64)4), new Vector2d((Fixed64)5, (Fixed64)4), Fixed64.FromFraction(1, 5))
        };
        (LSCollider2D Mover, Vector2d Displacement, LSCollider2D Target)[] moverSweeps =
        {
            (CreatePolygon(context, new Vector2d((Fixed64)(-4), Fixed64.Zero)).Collider, new Vector2d((Fixed64)5, Fixed64.Zero), targets[0]),
            (CreatePolygon(context, new Vector2d((Fixed64)(-4), (Fixed64)4)).Collider, new Vector2d((Fixed64)5, Fixed64.Zero), targets[0]),
            (CreateCapsule(context, new Vector2d((Fixed64)(-4), (Fixed64)3)).Collider, new Vector2d((Fixed64)5, Fixed64.Zero), targets[2]),
            (CreateCapsule(context, new Vector2d((Fixed64)3, (Fixed64)(-4))).Collider, new Vector2d(Fixed64.Zero, (Fixed64)5), targets[1]),
            (CreateBox(context, new Vector2d(Fixed64.Zero, (Fixed64)(-4))).Collider, new Vector2d((Fixed64)3, (Fixed64)5), targets[3])
        };

        int rayHits = 0;
        int rayMisses = 0;
        foreach (LSCollider2D target in targets)
        {
            foreach ((Vector2d start, Vector2d end) in rays)
            {
                bool first = QueryDetection2D.TryRaycast(start, end, target, out Physics2DHit firstHit);
                bool second = QueryDetection2D.TryRaycast(start, end, target, out Physics2DHit secondHit);

                second.Should().Be(first);
                if (first)
                {
                    rayHits++;
                    AssertValidHit(firstHit, target, Fixed64.Zero, (end - start).Magnitude);
                    firstHit.Distance.Should().Be(secondHit.Distance);
                    firstHit.Point.Should().Be(secondHit.Point);
                    firstHit.Normal.Should().Be(secondHit.Normal);
                }
                else
                {
                    rayMisses++;
                    firstHit.Should().Be(default(Physics2DHit));
                    secondHit.Should().Be(default(Physics2DHit));
                }
            }
        }

        int sweepHits = 0;
        int sweepMisses = 0;
        foreach (LSCollider2D target in targets)
        {
            foreach ((Vector2d start, Vector2d end, Fixed64 radius) in circleSweeps)
            {
                bool first = QueryDetection2D.TrySweepCircle(start, end, radius, target, out Physics2DHit firstHit);
                bool second = QueryDetection2D.TrySweepCircle(start, end, radius, target, out Physics2DHit secondHit);

                second.Should().Be(first);
                if (first)
                {
                    sweepHits++;
                    AssertValidHit(firstHit, target, Fixed64.Zero, (end - start).Magnitude);
                    firstHit.Distance.Should().Be(secondHit.Distance);
                    firstHit.Point.Should().Be(secondHit.Point);
                    firstHit.Normal.Should().Be(secondHit.Normal);
                }
                else
                {
                    sweepMisses++;
                    firstHit.Should().Be(default(Physics2DHit));
                    secondHit.Should().Be(default(Physics2DHit));
                }
            }
        }

        int moverHits = 0;
        int moverMisses = 0;
        foreach ((LSCollider2D mover, Vector2d displacement, LSCollider2D target) in moverSweeps)
        {
            bool first = QueryDetection2D.TrySweepMoverShape(mover, displacement, target, out Physics2DHit firstHit);
            bool second = QueryDetection2D.TrySweepMoverShape(mover, displacement, target, out Physics2DHit secondHit);

            second.Should().Be(first);
            if (first)
            {
                moverHits++;
                AssertValidHit(firstHit, target, Fixed64.Zero, displacement.Magnitude);
                firstHit.Distance.Should().Be(secondHit.Distance);
                firstHit.Point.Should().Be(secondHit.Point);
                firstHit.Normal.Should().Be(secondHit.Normal);
            }
            else
            {
                moverMisses++;
                firstHit.Should().Be(default(Physics2DHit));
                secondHit.Should().Be(default(Physics2DHit));
            }
        }

        bool polygonOverlap = QueryDetection2D.TryOverlapPolygon(
            stackalloc[]
            {
                new Vector2d((Fixed64)(-1), (Fixed64)(-1)),
                new Vector2d((Fixed64)4, (Fixed64)(-1)),
                new Vector2d((Fixed64)4, (Fixed64)4),
                new Vector2d((Fixed64)(-1), (Fixed64)4)
            },
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2)),
            targets[3],
            out Physics2DHit polygonHit);

        polygonOverlap.Should().BeTrue();
        AssertValidHit(polygonHit, targets[3], Fixed64.Zero, (Fixed64)5);
        rayHits.Should().BeGreaterThan(0);
        rayMisses.Should().BeGreaterThan(0);
        sweepHits.Should().BeGreaterThan(0);
        sweepMisses.Should().BeGreaterThan(0);
        moverHits.Should().BeGreaterThan(0);
        moverMisses.Should().BeGreaterThan(0);
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
            Mass = Fixed64.One
        };
        body.Collider.Layer = layer;
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
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
            Mass = Fixed64.One
        };
        body.Collider.Layer = layer;
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static SolidBody2D CreatePolygon(GravitasWorldContext context, Vector2d position)
    {
        return CreatePolygon(
            context,
            position,
            new Vector2d(-Fixed64.Half, -Fixed64.Half),
            new Vector2d(Fixed64.Half, -Fixed64.Half),
            new Vector2d(Fixed64.Half, Fixed64.Half),
            new Vector2d(-Fixed64.Half, Fixed64.Half));
    }

    private static SolidBody2D CreatePolygon(GravitasWorldContext context, Vector2d position, params Vector2d[] vertices)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(
            agent,
            new LSPolygonCollider2D(vertices))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static SolidBody2D CreateCapsule(GravitasWorldContext context, Vector2d position) =>
        CreateCapsule(context, position, Fixed64.Half, (Fixed64)2);

    private static SolidBody2D CreateCapsule(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 radius,
        Fixed64 height)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCapsuleCollider2D(radius, height))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static SolidBody2D CreateCompound(GravitasWorldContext context, Vector2d position)
    {
        return CreateCompound(
            context,
            position,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.One, Fixed64.Zero))));
    }

    private static SolidBody2D CreateCompound(
        GravitasWorldContext context,
        Vector2d position,
        LSCompoundCollider2D collider)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static SolidBody2D Create2DShape(
        GravitasWorldContext context,
        ColliderType2D type,
        Vector2d position)
    {
        return type switch
        {
            ColliderType2D.Circle => CreateCircle(context, position),
            ColliderType2D.AABox => CreateBox(context, position),
            ColliderType2D.ConvexPolygon => CreatePolygon(context, position),
            ColliderType2D.Capsule => CreateCapsule(context, position),
            ColliderType2D.Compound => CreateCompound(context, position),
            _ => throw new ArgumentOutOfRangeException(nameof(type), type, null)
        };
    }

    private static GravitasWorldContext Create2DContext()
    {
        return Physics2DTestWorld.CreateContext();
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);

    private static void AssertValidHit(Physics2DHit hit, LSCollider2D expectedCollider, Fixed64 minDistance, Fixed64 maxDistance)
    {
        hit.Collider.Should().BeSameAs(expectedCollider);
        hit.Distance.Should().BeGreaterThanOrEqualTo(minDistance);
        hit.Distance.Should().BeLessThanOrEqualTo(maxDistance);
        hit.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }
}
