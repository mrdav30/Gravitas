using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void RotationalMixedCandidateSnapshot_When3DCallbackReuses2DDynamicId_ShouldRejectNewLifetime()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> source = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        int dynamicId = target.DynamicId;
        long lifetimeVersion = target.Collider.LifetimeVersion;
        Vector3d oldPosition = target.Position.ToVector3d(Fixed64.Zero);
        context.Physics2D.PrepareContinuousCollisionFrame();

        source.Collider.OnMixedContactEnter += _ =>
        {
            target.Deactivate();
            target.Initialize(Vector2d.Right * (Fixed64)16);
        };

        source.Collider.NotifyMixedContact(
            target.Collider,
            isColliding: true,
            isChanged: true,
            isTriggerPair: false);

        target.DynamicId.Should().Be(dynamicId);
        target.Collider.LifetimeVersion.Should().BeGreaterThan(lifetimeVersion);
        SwiftList<int> candidates = context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                oldPosition,
                Vector3d.Zero,
                Fixed64.One));
        candidates.Contains(dynamicId).Should().BeFalse();
    }

    [Fact]
    public void RotationalMixedCandidateSnapshot_When2DCallbackReuses3DDynamicId_ShouldRejectNewLifetime()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        int dynamicId = target.Body.DynamicId;
        long lifetimeVersion = target.Collider.LifetimeVersion;
        Vector3d oldPosition = target.Body.Position3d;
        context.Physics.PrepareContinuousCollisionFrame();

        source.Collider.OnMixedContactEnter += _ =>
        {
            target.Body.Deactivate();
            target.Body.Initialize(Vector3d.Right * (Fixed64)16, FixedQuaternion.Identity);
        };

        source.Collider.NotifyMixedContact(
            target.Collider,
            isColliding: true,
            isChanged: true,
            isTriggerPair: false);

        target.Body.DynamicId.Should().Be(dynamicId);
        target.Collider.LifetimeVersion.Should().BeGreaterThan(lifetimeVersion);
        SwiftList<int> candidates = context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                oldPosition,
                Vector3d.Zero,
                Fixed64.One));
        candidates.Contains(dynamicId).Should().BeFalse();
    }
}
