using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class CollisionDetection2DTests
{
    [Fact]
    public void TryCollide_WithSeparatedLargeCircles_ShouldRejectDespiteOverlappingBounds()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D((Fixed64)25000);
        var second = new LSCircleCollider2D((Fixed64)25000);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(40000, 40000));

        // Exact squared distance 3,200,000,000 exceeds squared radius sum
        // 2,500,000,000. Saturating both to Fixed64.MaxValue invents contact.
        CollisionDetection2D.BoundsOverlap(first, second).Should().BeTrue();
        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);
        collided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(1, -1)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(10000, -1)]
    [InlineData(10000, 0)]
    [InlineData(10000, 1)]
    public void CircleContact_AtDiagonalTangency_ShouldDistinguishRawRadiusNeighbors(int scale, long step)
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D((Fixed64)(2 * scale));
        var second = new LSCircleCollider2D(Fixed64.FromRaw(((Fixed64)(3 * scale)).m_rawValue + step));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(3 * scale, 4 * scale));

        // The 3-4-5 distance is exact, including beyond scalar squared range.
        CollisionDetection2D.BoundsOverlap(first, second).Should().BeTrue();
        bool hit = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);
        hit.Should().Be(step >= 0);
        contact.Depth.Should().Be(Fixed64.FromRaw(Math.Max(step, 0)));
        contact.DepthIsClamped.Should().BeFalse();
        CollisionDetection2D.TryCollide(second, first, out Contact2D reversed).Should().Be(hit);
        reversed.Depth.Should().Be(contact.Depth);
        if (hit)
        {
            contact.Normal.Should().Be(new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)));
            reversed.Normal.Should().Be(-contact.Normal);
            reversed.PointA.Should().Be(contact.PointB);
            reversed.PointB.Should().Be(contact.PointA);
        }
        else
        {
            contact.Should().Be(default(Contact2D));
            reversed.Should().Be(default(Contact2D));
        }
    }

    [Fact]
    public void CircleContact_WithTinyNonzeroDistance_ShouldNotUseCoincidentFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.FromRaw(4));
        var second = new LSCircleCollider2D(Fixed64.FromRaw(4));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromRaw(3), Fixed64.FromRaw(4)));

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Normal.Should().Be(new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)));
        contact.Depth.Should().Be(Fixed64.FromRaw(3));
        contact.PointA.Should().Be(new Vector2d(Fixed64.FromRaw(2), Fixed64.FromRaw(3)));
        contact.PointB.Should().Be(new Vector2d(Fixed64.FromRaw(1), Fixed64.FromRaw(1)));
    }

    [Theory]
    [InlineData(1, 3)]
    [InlineData(2, 1)]
    public void CircleContact_WithIrrationalRawDistance_ShouldRoundFinalDepth(long coordinate, long expectedDepth)
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.FromRaw(2));
        var second = new LSCircleCollider2D(Fixed64.FromRaw(2));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromRaw(coordinate), Fixed64.FromRaw(coordinate)));

        // In raw units: 4-sqrt(2) rounds to 3; 4-sqrt(8) rounds to 1.
        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromRaw(expectedDepth));
        contact.Normal.Should().Be(new Vector2d(Fixed64.FromRaw(3037000500), Fixed64.FromRaw(3037000500)));
        contact.DepthIsClamped.Should().BeFalse();
    }

    [Fact]
    public void CircleContact_WithWideCenterDifferenceAndRadiusSum_ShouldKeepRepresentableDepth()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D((Fixed64)1600000000);
        var second = new LSCircleCollider2D((Fixed64)1600000000);
        _ = CreateBody(context, first, new Vector2d(-1500000000, 0));
        _ = CreateBody(context, second, new Vector2d(1500000000, 0));

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Normal.Should().Be(Vector2d.Right);
        contact.Depth.Should().Be((Fixed64)200000000);
        contact.DepthIsClamped.Should().BeFalse();
        contact.PointA.Should().Be(new Vector2d(100000000, 0));
        contact.PointB.Should().Be(new Vector2d(-100000000, 0));
    }

    [Theory]
    [InlineData(1, true)]
    [InlineData(2, false)]
    public void CircleContact_BelowScalarSquaredResolution_ShouldPreserveClassificationAndAnchorTerms(long coordinate, bool expectedHit)
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.MinIncrement);
        var second = new LSCircleCollider2D(Fixed64.MinIncrement);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromRaw(coordinate), Fixed64.FromRaw(coordinate)));

        CollisionDetection2D.BoundsOverlap(first, second).Should().BeTrue();
        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().Be(expectedHit);
        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);
        collided.Should().Be(expectedHit);
        if (expectedHit)
        {
            // 2-sqrt(2) rounds to one raw. Independently rounded world witnesses
            // differ by (-1,-1) raw, but their exact difference rounds to (0,0).
            contact.Depth.Should().Be(Fixed64.MinIncrement);
            contact.PointA.Should().Be(new Vector2d(Fixed64.MinIncrement, Fixed64.MinIncrement));
            contact.PointB.Should().Be(Vector2d.Zero);
            contact.AnchorB.TryGetOffsetFrom(contact.AnchorA, out Vector2d difference).Should().BeTrue();
            difference.Should().Be(Vector2d.Zero);
            manifold[0].AnchorB.TryGetOffsetFrom(manifold[0].AnchorA, out Vector2d manifoldDifference).Should().BeTrue();
            manifoldDifference.Should().Be(Vector2d.Zero);
        }
        else
        {
            contact.Should().Be(default(Contact2D));
            manifold.HasContact.Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(1, 1, false)]
    [InlineData(2, 3, true)]
    public void CircleContact_WithSubRawOverflow_ShouldClassifyClampingBeforeRounding(long coordinate, long radius, bool clamped)
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.MaxValue);
        var second = new LSCircleCollider2D(Fixed64.FromRaw(radius));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromRaw(coordinate), Fixed64.FromRaw(coordinate)));

        // M+1-sqrt(2) is below M, while M+3-sqrt(8) is above it. Both round to M.
        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.MaxValue);
        contact.DepthIsClamped.Should().Be(clamped);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    public void CircleContact_AtMaximumDepth_ShouldPropagateExactClampingToManifold(long step, bool clamped)
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.MaxValue);
        var second = new LSCircleCollider2D(Fixed64.One);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromRaw(Fixed64.One.m_rawValue + step), Fixed64.Zero));

        Fixed64 expectedDepth = Fixed64.FromRaw(long.MaxValue - Math.Max(step, 0));
        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Normal.Should().Be(Vector2d.Right);
        contact.Depth.Should().Be(expectedDepth);
        contact.DepthIsClamped.Should().Be(clamped);
        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);
        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Depth.Should().Be(expectedDepth);
        manifold[0].DepthIsClamped.Should().Be(clamped);
        manifold[0].Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void CircleContact_WithRotatedOwners_ShouldPreserveAnchorFramesAndWorldWitnesses()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D((Fixed64)5);
        var second = new LSCircleCollider2D((Fixed64)5);
        _ = CreateBody(context, first, Vector2d.Zero, Fixed64.HalfPi);
        _ = CreateBody(context, second, new Vector2d(5, 0), -Fixed64.HalfPi);

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.AnchorA.Origin.Should().Be(Vector2d.Zero);
        contact.AnchorB.Origin.Should().Be(new Vector2d(5, 0));
        contact.AnchorA.Rotation.Should().Be(Fixed64.HalfPi);
        contact.AnchorB.Rotation.Should().Be(-Fixed64.HalfPi);
        // A cardinal normal and quarter turns are exactly representable, so
        // these witness expectations do not assume an unrounded 3/5 normal.
        contact.Normal.Should().Be(Vector2d.Right);
        contact.PointA.Should().Be(new Vector2d(5, 0));
        contact.PointB.Should().Be(Vector2d.Zero);
        contact.Depth.Should().Be((Fixed64)5);
    }

    [Fact]
    public void CircleContact_WithUnrepresentableWorldWitness_ShouldRetainUsableAnchors()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.One);
        var second = new LSCircleCollider2D(Fixed64.One);
        Vector2d center = new(Fixed64.MaxValue, Fixed64.Zero);
        _ = CreateBody(context, first, center);
        _ = CreateBody(context, second, center);

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
        contact.Normal.Should().Be(Vector2d.Right);
        contact.Depth.Should().Be(Fixed64.Two);
        contact.AnchorA.TryGetWorldPoint(out _).Should().BeFalse();
        contact.AnchorA.TryGetOffsetFrom(center, out Vector2d firstOffset).Should().BeTrue();
        firstOffset.Should().Be(Vector2d.Right);
        contact.AnchorB.TryGetOffsetFrom(center, out Vector2d secondOffset).Should().BeTrue();
        secondOffset.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void CircleContact_AfterWarmup_ShouldKeepDirectAndManifoldQueriesAllocationFree()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D((Fixed64)25000);
        var second = new LSCircleCollider2D((Fixed64)25000);
        _ = CreateBody(context, first, Vector2d.Zero, Fixed64.One);
        _ = CreateBody(context, second, new Vector2d(18000, 24000), -Fixed64.One);
        var manifold = new ContactManifold2D();
        var item = new CollisionWorkItem2D(first, second, CollisionType2D.Circle_Circle);

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection2D.TryCollide(first, second, out Contact2D contact) ||
                contact.Depth != (Fixed64)20000 ||
                !CollisionDetection2D.TryCollide(item, manifold, 42) || manifold[0].Depth != contact.Depth)
                throw new InvalidOperationException("Circle contact changed during allocation measurement.");
        });

        allocated.Should().Be(0);
        manifold.Count.Should().Be(1);
        manifold[0].Normal.Should().Be(new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)));
    }
}
