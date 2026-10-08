using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class PreparedSegmentQuery2DTests
{
    [Theory]
    [InlineData(ColliderType2D.AABox, 4, 2)]
    [InlineData(ColliderType2D.AABox, 2, 1)]
    [InlineData(ColliderType2D.ConvexPolygon, 4, 2)]
    [InlineData(ColliderType2D.ConvexPolygon, 2, 1)]
    [InlineData(ColliderType2D.Capsule, 4, 2)]
    [InlineData(ColliderType2D.Capsule, 2, 1)]
    [InlineData(ColliderType2D.Capsule, 0, 2)]
    [InlineData(ColliderType2D.Circle, 4, 0)]
    [InlineData(ColliderType2D.Circle, 2, 0)]
    public void ContainedRay_ShouldKeepStartPointAndReportGeometricOutwardNormal(
        ColliderType2D shape, int xEighths, int yEighths)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D collider = CreateCollider(context, shape);
        Vector2d start = new(Fixed64.FromFraction(xEighths, 8), Fixed64.FromFraction(yEighths, 8));
        var hits = new SwiftList<Physics2DHit>();

        // The unique nearest surface is the right face/radial side, even when
        // travel is parallel to it or points away. Containment is still a hit.
        foreach (Vector2d travel in new[] { -Vector2d.Forward, Vector2d.Right })
        {
            Vector2d end = start + travel;
            context.Query2D.Raycast(start, end, out Physics2DHit closest).Should().BeTrue();
            closest.Collider.Should().BeSameAs(collider);
            closest.Point.Should().Be(start);
            closest.Distance.Should().Be(Fixed64.Zero);
            closest.Normal.Should().Be(Vector2d.Right);
            context.Query2D.RaycastAll(start, end, hits).Should().Be(1);
            hits[0].Should().Be(closest);
            QueryDetection2D.TryRaycast(start, end, collider, out Physics2DHit raw).Should().BeTrue();
            raw.Should().Be(closest);
        }
    }

    [Theory]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void ContainedRay_AtEquidistantFeatures_ShouldUseStableGeometricTie(ColliderType2D shape)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D collider = CreateCollider(context, shape);
        foreach (Vector2d start in new[] { Vector2d.Zero, new Vector2d(Fixed64.Half, Fixed64.Half) })
        {
            // Both shapes author their bottom edge first. At the center its
            // two opposing faces tie; at the upper-right corner its top face
            // ties the right face. Existing geometric feature order resolves both.
            Vector2d expected = start == Vector2d.Zero ? -Vector2d.Forward : Vector2d.Forward;
            for (int repeat = 0; repeat < 3; repeat++)
            {
                QueryDetection2D.TryRaycast(start, start + Vector2d.Right, collider, out Physics2DHit hit)
                    .Should().BeTrue();
                hit.Normal.Should().Be(expected);
                hit.Point.Should().Be(start);
                hit.Distance.Should().Be(Fixed64.Zero);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedRay_RotatedPolygon_ShouldReportSameSurfaceForEitherWinding(bool clockwise)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Vector2d[] vertices =
        {
            new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half)
        };
        if (clockwise)
            System.Array.Reverse(vertices);
        var collider = new LSPolygonCollider2D(vertices);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero, Fixed64.HalfPi, BodyMotionType.Static);
        Vector2d start = new(-Fixed64.FromFraction(1, 8), Fixed64.Quarter);

        context.Query2D.Raycast(start, start + Vector2d.Right, out Physics2DHit hit).Should().BeTrue();

        hit.Normal.X.Should().BeInRange(-Fixed64.Epsilon, Fixed64.Epsilon);
        hit.Normal.Y.Should().BeInRange(Fixed64.One - Fixed64.Epsilon, Fixed64.One + Fixed64.Epsilon);
        hit.Point.Should().Be(start);
        hit.Distance.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedRay_NearScalarBoundary_ShouldPreserveGeometricNormalAndStart(bool nearMaximum)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Vector2d origin = new(nearMaximum ? Fixed64.MaxValue - (Fixed64)4 : Fixed64.MinValue + (Fixed64)4, Fixed64.Zero);
        LSCollider2D collider = CreateCollider(context, ColliderType2D.ConvexPolygon, origin);
        Vector2d start = origin + new Vector2d(Fixed64.Quarter, Fixed64.FromFraction(1, 8));

        QueryDetection2D.TryRaycast(start, start - Vector2d.Forward, collider, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Normal.Should().Be(Vector2d.Right);
        hit.Point.Should().Be(start);
        hit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContainedRay_OnRotatedCapsuleAxis_ShouldUseLocalRadialFallback()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero, Fixed64.HalfPi, BodyMotionType.Static);

        context.Query2D.Raycast(Vector2d.Zero, Vector2d.Right, out Physics2DHit hit).Should().BeTrue();

        hit.Normal.X.Should().BeInRange(-Fixed64.Epsilon, Fixed64.Epsilon);
        hit.Normal.Y.Should().BeInRange(Fixed64.One - Fixed64.Epsilon, Fixed64.One + Fixed64.Epsilon);
        hit.Point.Should().Be(Vector2d.Zero);
        hit.Distance.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedRay_InMultipleCompoundParts_ShouldRetainFirstAuthoredWitness(bool floorFirst)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        CompoundColliderPart2D wall = CompoundColliderPart2D.AABBox(new Vector2d(1, 4), Vector2d.Zero);
        CompoundColliderPart2D floor = CompoundColliderPart2D.AABBox(new Vector2d(4, 1), Vector2d.Zero);
        var compound = new LSCompoundCollider2D(floorFirst ? floor : wall, floorFirst ? wall : floor);
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        Vector2d start = new(Fixed64.Quarter, Fixed64.FromFraction(1, 8));
        var hits = new SwiftList<Physics2DHit>();

        context.Query2D.Raycast(start, start + Vector2d.Right, out Physics2DHit closest).Should().BeTrue();
        context.Query2D.RaycastAll(start, start + Vector2d.Right, hits).Should().Be(1);

        closest.Collider.Should().BeSameAs(compound);
        closest.Normal.Should().Be(floorFirst ? Vector2d.Forward : Vector2d.Right);
        closest.Point.Should().Be(start);
        closest.Distance.Should().Be(Fixed64.Zero);
        hits[0].Should().Be(closest);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Compound)]
    public void ContainedRay_ShouldNotAllocateAfterWarmup(ColliderType2D shape)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D collider = CreateCollider(context, shape);
        Vector2d start = new(Fixed64.Quarter, Fixed64.FromFraction(1, 8));
        var hits = new SwiftList<Physics2DHit>();
        Physics2DHit closest = default;
        System.Action cast = () =>
        {
            context.Query2D.Raycast(start, start + Vector2d.Right, out closest);
            context.Query2D.RaycastAll(start, start + Vector2d.Right, hits);
        };

        AllocationTestHelper.MeasureSteadyState(cast).Should().Be(0);

        closest.Collider.Should().BeSameAs(collider);
        closest.Distance.Should().Be(Fixed64.Zero);
        hits.Count.Should().Be(1);
        hits[0].Should().Be(closest);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Compound)]
    public void PreparedQueries_PreserveRawAndServiceHits(ColliderType2D shape)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D collider = CreateCollider(context, shape);
        var results = new SwiftList<Physics2DHit>();
        // Oblique travel exercises unchanged normalization for convex leaves;
        // outside and contained starts exercise rejection and overlap branches.
        foreach (Vector2d start in new[]
        {
            new Vector2d((Fixed64)(-3), -Fixed64.Quarter),
            new Vector2d((Fixed64)(-3), (Fixed64)4),
            Vector2d.Zero
        })
        {
            Vector2d end = start + new Vector2d((Fixed64)6, Fixed64.Half);
            Vector2d.TrySubtract(end, start, out Vector2d segment).Should().BeTrue();
            Vector2d.TryGetMagnitude(segment, out Fixed64 length).Should().BeTrue();

            bool rawRay = QueryDetection2D.TryRaycast(start, end, collider, out Physics2DHit rawRayHit);
            QueryDetection2D.TryRaycast(start, end, segment, length, collider, out Physics2DHit preparedRayHit)
                .Should().Be(rawRay);
            preparedRayHit.Should().Be(rawRayHit);
            context.Query2D.Raycast(start, end, out Physics2DHit serviceRayHit).Should().Be(rawRay);
            serviceRayHit.Should().Be(rawRayHit);
            context.Query2D.RaycastAll(start, end, results).Should().Be(rawRay ? 1 : 0);
            if (rawRay)
                results[0].Should().Be(rawRayHit);

            bool rawSweep = QueryDetection2D.TrySweepCircle(start, end, Fixed64.Quarter, collider,
                out Physics2DHit rawSweepHit);
            QueryDetection2D.TrySweepCircle(start, end, segment, length, Fixed64.Quarter, collider,
                out Physics2DHit preparedSweepHit).Should().Be(rawSweep);
            preparedSweepHit.Should().Be(rawSweepHit);
            context.Query2D.SweepCircle(start, end, Fixed64.Quarter, out Physics2DHit serviceSweepHit)
                .Should().Be(rawSweep);
            serviceSweepHit.Should().Be(rawSweepHit);
            context.Query2D.SweepCircleAll(start, end, Fixed64.Quarter, results).Should().Be(rawSweep ? 1 : 0);
            if (rawSweep)
                results[0].Should().Be(rawSweepHit);
        }
    }

    [Fact]
    public void RaycastAll_WhenBoundsOverlapButCircleIsMissed_ShouldClearPreviousHits()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        CreateCollider(context, ColliderType2D.Circle);
        context.Query2D.Raycast(Vector2d.Left, Vector2d.Right, out Physics2DHit previous).Should().BeTrue();
        var results = new SwiftList<Physics2DHit> { previous };

        // The chord's bounds touch the circle's top edge, but the chord itself
        // stays outside the half-unit circle. Broad-phase admission is not a hit.
        Vector2d start = new(-Fixed64.One, Fixed64.Half);
        Vector2d end = new(Fixed64.Half, Fixed64.One);
        context.Query2D.RaycastAll(start, end, results).Should().Be(0);

        context.Query2D.LastQueryCandidateCount.Should().Be(1);
        results.Should().BeEmpty();
    }

    [Fact]
    public void OneRawTravel_RemainsAdmittedForRaycastAndRejectedForGenericSweep()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D collider = CreateCollider(context, ColliderType2D.Circle);
        Vector2d end = new(Fixed64.FromRaw(1), Fixed64.Zero);
        Vector2d.TryGetMagnitude(end, out Fixed64 length).Should().BeTrue();
        QueryDetection2D.TryRaycast(Vector2d.Zero, end, end, length, collider, out Physics2DHit prepared)
            .Should().BeTrue();
        prepared.Collider.Should().BeSameAs(collider);
        prepared.Distance.Should().Be(Fixed64.Zero);
        context.Query2D.Raycast(Vector2d.Zero, end, out Physics2DHit actual).Should().BeTrue();
        actual.Should().Be(prepared);
        context.Query2D.SweepCircle(Vector2d.Zero, end, Fixed64.Quarter, out _).Should().BeFalse();
        QueryDetection2D.TrySweepCircle(Vector2d.Zero, end, Fixed64.Quarter, collider, out _)
            .Should().BeFalse();
    }

    private static LSCollider2D CreateCollider(GravitasWorldContext context, ColliderType2D shape, Vector2d position = default)
    {
        LSCollider2D collider = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(new[]
            {
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)
            }),
            _ => new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, Fixed64.Two)),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.One, Fixed64.Zero)))
        };
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }
}
