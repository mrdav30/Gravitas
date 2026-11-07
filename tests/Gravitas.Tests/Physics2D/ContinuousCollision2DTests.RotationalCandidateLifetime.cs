using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalCandidates_WithOneRegisteredSource_ShouldTrackTargetLifetimes(bool movingTarget)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateRotationalBlade(context);
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().Be(0);

        SolidBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * (Fixed64)3, immovable: !movingTarget);
        context.AdvanceLateSimulateToken();
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().BeGreaterThan(0);

        target.Deactivate();
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().Be(0);

        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)90);
        context.Simulate();
        context.LateSimulate();
        source.Rotation.Should().Be(FixedMath.DegToRad((Fixed64)90));
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);

        target.Initialize(Vector2d.Right * (Fixed64)3,
            motionType: movingTarget ? BodyMotionType.Dynamic : BodyMotionType.Static);
        context.AdvanceLateSimulateToken();
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().BeGreaterThan(0);

        // A lone target is still eligible when the source itself has been unregistered.
        source.Deactivate();
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().BeGreaterThan(0);
        target.Deactivate();
        source.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
            Vector2d.Zero, (Fixed64)4).Should().Be(0);
    }

    [Fact]
    public void RotationalCandidateSnapshot_WhenContactCallbackReuses2DDynamicId_ShouldRejectNewLifetime()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateRotationalMovingPairBlade2D(context, isKinematic: true);
        SolidBody2D target = CreateRotationalMovingPairTarget2D(context);
        int dynamicId = target.DynamicId;
        long lifetimeVersion = target.Collider.LifetimeVersion;
        context.Physics2D.PrepareContinuousCollisionFrame();

        source.Collider.OnContactEnter += _ =>
        {
            target.Deactivate();
            target.Initialize(Vector2d.Right * (Fixed64)16);
        };

        source.Collider.NotifyContact(target.Collider, isColliding: true, isChanged: true);

        target.DynamicId.Should().Be(dynamicId);
        target.Collider.LifetimeVersion.Should().BeGreaterThan(lifetimeVersion);
        SwiftList<int> candidates = context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                Vector2d.Zero,
                Vector2d.Zero,
                (Fixed64)4));
        candidates.Contains(dynamicId).Should().BeFalse();
    }
}
