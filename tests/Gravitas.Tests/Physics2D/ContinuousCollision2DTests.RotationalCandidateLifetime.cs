using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
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
