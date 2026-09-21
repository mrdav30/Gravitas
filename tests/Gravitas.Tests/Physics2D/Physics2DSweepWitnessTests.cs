using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DSweepWitnessTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle, 6, 0)]
    [InlineData(ColliderType2D.Circle, 5, 0)]
    [InlineData(ColliderType2D.Circle, 3, 0)]
    [InlineData(ColliderType2D.Circle, 12, 6)]
    [InlineData(ColliderType2D.Capsule, 6, 0)]
    [InlineData(ColliderType2D.Capsule, 5, 0)]
    [InlineData(ColliderType2D.Capsule, 3, 0)]
    [InlineData(ColliderType2D.Capsule, 12, 6)]
    [InlineData(ColliderType2D.AABox, 6, 0)]
    [InlineData(ColliderType2D.AABox, 5, 0)]
    [InlineData(ColliderType2D.AABox, 3, 0)]
    [InlineData(ColliderType2D.AABox, 12, 6)]
    [InlineData(ColliderType2D.ConvexPolygon, 6, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 5, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 3, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 12, 6)]
    [InlineData(ColliderType2D.Compound, 6, 0)]
    [InlineData(ColliderType2D.Compound, 5, 0)]
    [InlineData(ColliderType2D.Compound, 3, 0)]
    [InlineData(ColliderType2D.Compound, 12, 6)]
    public void SweepCircle_PreservesTargetSurfaceAcrossInitialContactAndOrdinaryEntry(
        ColliderType2D shape, int startQuarters, int distanceQuarters)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D target = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.One),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One * Fixed64.Two),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(new[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            }),
            ColliderType2D.Compound => new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero),
                CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Right * (Fixed64)6)),
            _ => throw new ArgumentOutOfRangeException(nameof(shape))
        };
        Initialize(context, target);
        Vector2d start = new(Fixed64.FromFraction(startQuarters, 4), Fixed64.Zero);
        var hits = new SwiftList<Physics2DHit>();

        context.Query2D.SweepCircle(start, Vector2d.Zero, Fixed64.Half, out Physics2DHit closest)
            .Should().BeTrue();
        context.Query2D.SweepCircleAll(start, Vector2d.Zero, Fixed64.Half, hits).Should().Be(1);

        // Every target's right boundary is x=1. Touch/overlap travel is zero;
        // an initially separated center at x=3 travels 3/2 before contact.
        foreach (Physics2DHit hit in new[] { closest, hits[0] })
        {
            hit.Collider.Should().BeSameAs(target);
            hit.Body.Should().BeNull();
            hit.Distance.Should().Be(Fixed64.FromFraction(distanceQuarters, 4));
            hit.Point.Should().Be(Vector2d.Right);
            hit.Normal.Should().Be(Vector2d.Right);
        }
    }

    [Fact]
    public void SweepCircle_StartingOnFloor_ReturnsTheFloorPointNotTheProbeCenter()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));

        context.Query2D.SweepCircle(
            new Vector2d(Fixed64.Zero, Fixed64.Half), Vector2d.Zero,
            Fixed64.Half, out Physics2DHit hit).Should().BeTrue();

        hit.Collider.Should().BeSameAs(floor);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point.Should().Be(Vector2d.Zero);
        hit.Normal.Should().Be(Vector2d.Forward);
    }

    [Fact]
    public void SweepCircle_InitiallyTouchingTwoCompoundParts_KeepsTheFirstPartWitness()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var target = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d(4, 1), new Vector2d(Fixed64.Zero, -Fixed64.Half)));
        Initialize(context, target);
        Vector2d start = new(Fixed64.Half, Fixed64.Half);

        context.Query2D.SweepCircle(start, start + Vector2d.Right, Fixed64.Half, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Collider.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Half));
        hit.Normal.Should().Be(Vector2d.Right);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SweepCircle_InitialWitnessBeyondScalarFace_RetainsTheRelativeAnchor(bool positive)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var target = new LSCircleCollider2D(Fixed64.One);
        Initialize(context, target);
        Fixed64 face = positive ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 centerX = positive ? face - Fixed64.FromFraction(1, 4) : face + Fixed64.FromFraction(1, 4);
        target.Agent.Transform.TrySetWorldPosition(new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero))
            .Should().BeTrue();
        target.RebuildRuntimeShapeOnly().Should().BeTrue();
        Vector2d start = new(face, Fixed64.Zero);

        // This isolates the narrow phase: the extreme target is outside the fixture grid.
        QueryDetection2D.TrySweepCircle(start, start + Vector2d.Forward, Fixed64.Half, target, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Collider.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.Should().Be(positive ? Vector2d.Right : -Vector2d.Right);
        hit.TryGetPoint(out _).Should().BeFalse();
        hit.Anchor.TryGetOffsetFrom(new Vector2d(centerX, Fixed64.Zero), out Vector2d offset)
            .Should().BeTrue();
        offset.Should().Be(positive ? Vector2d.Right : -Vector2d.Right);
    }

    [Theory]
    [InlineData(0, 0, false)]
    [InlineData(2, 0, false)]
    [InlineData(4, 0, false)]
    [InlineData(4, 1, false)]
    [InlineData(4, -1, false)]
    [InlineData(4, 0, true)]
    [InlineData(4, 1, true)]
    [InlineData(4, -1, true)]
    public void CircleQueries_WithProbeCenterOnTargetAxis_KeepSurfaceWitnessAndNormalConsistent(
        int capsuleHeight, int axisOffsetQuarters, bool rotated)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D target = capsuleHeight == 0
            ? new LSCircleCollider2D(Fixed64.One)
            : new LSCapsuleCollider2D(Fixed64.One, (Fixed64)capsuleHeight);
        Initialize(context, target, rotation: rotated ? Fixed64.HalfPi : Fixed64.Zero);
        Fixed64 axialOffset = Fixed64.FromFraction(axisOffsetQuarters, 4);
        Vector2d start = rotated ? new Vector2d(-axialOffset, Fixed64.Zero) : new Vector2d(Fixed64.Zero, axialOffset);
        Vector2d expectedNormal = rotated ? Vector2d.Forward : Vector2d.Right;

        context.Query2D.OverlapCircle(start, Fixed64.Half, out Physics2DHit overlap).Should().BeTrue();
        context.Query2D.SweepCircle(start, start + expectedNormal, Fixed64.Half, out Physics2DHit sweep)
            .Should().BeTrue();

        // A center on the capsule axis has no unique radial direction. Select
        // the local +X surface, never an axial offset that remains inside it.
        foreach (Physics2DHit hit in new[] { overlap, sweep })
        {
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal.Should().Be(expectedNormal);
            hit.Point.Should().Be(start + expectedNormal);
        }
    }

    private static void Initialize(
        GravitasWorldContext context, LSCollider2D collider,
        Vector2d position = default, Fixed64 rotation = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        transform.LocalRotationXZRadians = rotation;
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
    }
}
