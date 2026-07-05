using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionCandidateOrderingTests
{
    [Fact]
    public void ShouldReplaceMixedHit_ShouldUseDistanceClosingReducerKindAndStableColliderIds()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider lower3D = CreateSphere3D(context, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider higher3D = CreateSphere3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        LSCircleCollider2D lower2D = CreateCircle2D(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        LSCircleCollider2D higher2D = CreateCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        lower3D.Id.Should().BeLessThan(higher3D.Id);
        lower2D.Id.Should().BeLessThan(higher2D.Id);

        PhysicsMixedHit current = CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: (Fixed64)2);

        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(lower3D, lower2D, PhysicsQueryReducerKind.Exact, distance: Fixed64.One),
                Fixed64.One,
                hasCandidate: false,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(lower3D, lower2D, PhysicsQueryReducerKind.Exact, distance: Fixed64.One),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: false,
                default,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: Fixed64.One),
                Fixed64.Zero,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(lower3D, lower2D, PhysicsQueryReducerKind.Exact, distance: (Fixed64)3),
                (Fixed64)4,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                (Fixed64)2,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.Exact, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(lower3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, lower2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeFalse();
    }

    private static PhysicsMixedHit CreateHit(
        LSCollider collider3D,
        LSCollider2D collider2D,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance) =>
        new(
            collider3D,
            collider2D,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            reducerKind,
            distance,
            Vector3d.Right);

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-4), (Fixed64)(-16)),
                new Vector3d((Fixed64)16, (Fixed64)4, (Fixed64)16)),
            out _).Should().BeTrue();
        return context;
    }

    private static LSSphereCollider CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static LSCircleCollider2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }
}
