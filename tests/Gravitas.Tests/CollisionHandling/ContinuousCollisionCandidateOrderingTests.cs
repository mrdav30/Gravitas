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
    public void ShouldReplace3DHit_ShouldUseDistanceClosingSpeedAndStableColliderIds()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider lower = CreateSphere3D(context, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider higher = CreateSphere3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        lower.Id.Should().BeLessThan(higher.Id);
        Physics3DHit current = CreateHit(higher, distance: (Fixed64)2);

        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, Fixed64.One),
                Fixed64.One,
                hasCandidate: false,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, Fixed64.One),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: false,
                default,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, Fixed64.One),
                Fixed64.Zero,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, (Fixed64)3),
                (Fixed64)4,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, current.Distance),
                (Fixed64)2,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit((LSCollider?)null, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                CreateHit((LSCollider?)null, current.Distance),
                Fixed64.One)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ShouldReplace2DHit_ShouldUseDistanceClosingSpeedAndStableColliderIds()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D lower = CreateCircle2D(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        LSCircleCollider2D higher = CreateCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        lower.Id.Should().BeLessThan(higher.Id);
        Physics2DHit current = CreateHit(higher, distance: (Fixed64)2);

        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, Fixed64.One),
                Fixed64.One,
                hasCandidate: false,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, Fixed64.One),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: false,
                default,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, Fixed64.One),
                Fixed64.Zero,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, (Fixed64)3),
                (Fixed64)4,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.Zero)
            .Should()
            .BeFalse();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, current.Distance),
                (Fixed64)2,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(lower, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                CreateHit(higher, current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeFalse();
    }

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
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(null, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                current,
                Fixed64.One)
            .Should()
            .BeTrue();
        ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                CreateHit(higher3D, null, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One,
                hasCandidate: true,
                hasCurrent: true,
                CreateHit(higher3D, higher2D, PhysicsQueryReducerKind.ConservativeFallback, distance: current.Distance),
                Fixed64.One)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void IsIgnoredTarget3D_ShouldIgnoreSelfSameBodyAndHierarchyRelatives()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> top = scenario.CreateSphere(new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(new Vector3d((Fixed64)(-6), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sibling = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> unrelated = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        var sameBody = new LSSphereCollider();
        sameBody.Initialize(top.Body);
        middle.Collider.SetParent(top.Collider);
        child.Collider.SetParent(middle.Collider);
        sibling.Collider.SetParent(top.Collider);

        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, ignored: null).Should().BeFalse();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, child.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(sameBody, top.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, top.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(top.Collider, child.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, sibling.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, unrelated.Collider).Should().BeFalse();
    }

    [Fact]
    public void IsIgnoredTarget2D_ShouldIgnoreSelfSameBodyAndHierarchyRelatives()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D top = CreateBody2D(context, new Vector2d((Fixed64)(-8), Fixed64.Zero));
        SolidBody2D middle = CreateBody2D(context, new Vector2d((Fixed64)(-6), Fixed64.Zero));
        SolidBody2D child = CreateBody2D(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        SolidBody2D sibling = CreateBody2D(context, new Vector2d((Fixed64)(-2), Fixed64.Zero));
        SolidBody2D unrelated = CreateBody2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        var sameBody = new LSCircleCollider2D(Fixed64.Half);
        sameBody.Initialize(top);
        middle.Collider.SetParent(top.Collider);
        child.Collider.SetParent(middle.Collider);
        sibling.Collider.SetParent(top.Collider);

        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, ignored: null).Should().BeFalse();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, child.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(sameBody, top.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, top.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(top.Collider, child.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, sibling.Collider).Should().BeTrue();
        ContinuousCollisionCandidateOrdering.IsIgnoredTarget(child.Collider, unrelated.Collider).Should().BeFalse();
    }

    private static Physics3DHit CreateHit(LSCollider? collider, Fixed64 distance) =>
        new(collider, Vector3d.Zero, Vector3d.Right, distance, Vector3d.Right);

    private static Physics2DHit CreateHit(LSCollider2D collider, Fixed64 distance) =>
        new(collider, Vector2d.Zero, Vector2d.Right, distance);

    private static PhysicsMixedHit CreateHit(
        LSCollider? collider3D,
        LSCollider2D? collider2D,
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

    private static SolidBody2D CreateBody2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }
}
