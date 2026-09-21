using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(ColliderType2D.AABox, 0)]
    [InlineData(ColliderType2D.AABox, 1)]
    [InlineData(ColliderType2D.AABox, -1)]
    [InlineData(ColliderType2D.Circle, 0)]
    [InlineData(ColliderType2D.Circle, 1)]
    [InlineData(ColliderType2D.Circle, -1)]
    [InlineData(ColliderType2D.Capsule, 0)]
    [InlineData(ColliderType2D.Capsule, 1)]
    [InlineData(ColliderType2D.Capsule, -1)]
    [InlineData(ColliderType2D.ConvexPolygon, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 1)]
    [InlineData(ColliderType2D.ConvexPolygon, -1)]
    public void RotationalArbiter_CenteredCircleOnPrimitive_ShouldRejectOnlyClosingMotion(
        ColliderType2D supportType,
        int normalDirection)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        LSCollider2D support = CreateInvariantSupportPrimitive2D(supportType);
        Fixed64 supportCenter = supportType == ColliderType2D.Capsule ? (Fixed64)(-2) : -Fixed64.One;
        CreateBody(context, support, new Vector2d(Fixed64.Zero, supportCenter), immovable: true);
        AddSeparatedInvariantSupportBlade2D(context);
        var start = new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8));
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            start, immovable: false, isKinematic: true);
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d displacement = normalDirection == 0
            ? Vector2d.Right * Fixed64.FromFraction(1, 1024)
            : Vector2d.Forward * Fixed64.FromFraction(normalDirection, 64);
        Vector2d requested = start + displacement;
        source.Agent.Transform.LocalPosition = new Vector3d(requested.X, Fixed64.Zero, requested.Y);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);
        Fixed64 requestedRotation = source.Agent.Transform.WorldRotationXZRadians;

        context.LateSimulate();

        if (normalDirection < 0)
        {
            // Contact exists from t = 0. The depth-limited search accepts the
            // first leaf's midpoint witness: 1 / 2^(MaxDepth + 1) of the frame.
            // For this 1/64-unit request that permits at most 1/524288 unit.
            Fixed64 contactTimeResolution = Fixed64.FromFraction(
                1, 1 << (ContinuousCollisionMath.RotationalIntervalMaxDepth + 1));
            Fixed64 maximumPenetration = displacement.Y.Abs() * contactTimeResolution;
            source.Position.Y.Should().BeInRange(start.Y - maximumPenetration, start.Y,
                "closing against {0} may consume only the initial contact interval", supportType);
            source.Position.Y.Should().BeGreaterThan(requested.Y,
                "the support must reject the requested penetration, not discard the contact");
            source.Position.X.Should().Be(start.X);
            source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        }
        else
        {
            source.Position.Should().Be(requested,
                "tangent or separating movement must not stick to {0}", supportType);
            source.Rotation.Should().Be(requestedRotation);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    [Fact]
    public void RotationalArbiter_CompoundSupport2D_ShouldNotHideItsLaterBlockingChild()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(4, 2), new Vector2d(Fixed64.Zero, -Fixed64.One)),
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.Quarter, (Fixed64)2),
                Vector2d.Right * Fixed64.Half));
        CreateBody(context, compound, Vector2d.Zero, immovable: true);
        AddSeparatedInvariantSupportBlade2D(context);
        var start = new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8));
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            start, immovable: false, isKinematic: true);
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        context.Query2D.SweepCircle(start, start + Vector2d.Right,
            Fixed64.FromFraction(1, 8), PhysicsLayerMask.All,
            out Physics2DHit firstHit, source.Collider).Should().BeTrue();
        firstHit.Collider.Should().BeSameAs(compound);
        Vector2d.Dot(firstHit.Normal, Vector2d.Right).Should().Be(Fixed64.Zero,
            "the first child is tangent support, not the later blocking wall");
        source.Agent.Transform.LocalPosition = new Vector3d(Fixed64.One, Fixed64.Zero, start.Y);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);

        context.LateSimulate();

        // The second child's near edge is 3/8, so the center may not pass 1/4.
        // The general compound path may conservatively stop on the first child.
        source.Position.X.Should().BeInRange(Fixed64.Zero, Fixed64.Quarter);
        source.Position.Y.Should().Be(start.Y);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    private static LSCollider2D CreateInvariantSupportPrimitive2D(ColliderType2D shape) => shape switch
    {
        ColliderType2D.AABox => new LSAABBoxCollider2D(new Vector2d(4, 2)),
        ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.One),
        ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4),
        ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
            new Vector2d(-2, -1), new Vector2d(2, -1), new Vector2d(2, 1), new Vector2d(-2, 1)),
        _ => throw new ArgumentOutOfRangeException(nameof(shape))
    };

    private static void AddSeparatedInvariantSupportBlade2D(GravitasWorldContext context)
    {
        SolidBody2D blade = CreateRotationalBlade(context);
        blade.ResetPosition(new Vector2d(Fixed64.Zero, (Fixed64)2), -FixedMath.DegToRad((Fixed64)5));
        blade.Agent.Transform.LocalPosition = new Vector3d(0, 0, 2);
        blade.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        // The rotating blade's broad circle admits the arbiter; its actual
        // polygon stays above 1.6, clear of every source pose in this fixture.
    }
}
