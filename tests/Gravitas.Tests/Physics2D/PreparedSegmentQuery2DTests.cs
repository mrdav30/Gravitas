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

    private static LSCollider2D CreateCollider(GravitasWorldContext context, ColliderType2D shape)
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
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }
}
