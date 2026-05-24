using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Raycasting;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Raycasting;

public sealed class GravitasCircleQueryServiceTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void OverlapCircleAll_ShouldSuppressDuplicateColliderHitsWithinContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.CircleQueries
            .OverlapCircleAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void OverlapCircleAll_ShouldReturnHitsOrderedBySurfaceDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicSphere(context, new Vector3d(1, 0, 0));
        LSSphereCollider far = CreateDynamicSphere(context, new Vector3d(3, 0, 0));
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.CircleQueries
            .OverlapCircleAll(Vector3d.Zero, (Fixed64)4, IncludeLayerZero, hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        hits[0].Distance.Should().Be(Fixed64.Half);
        hits[0].Point.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        hits[0].Normal.Should().Be(-Vector3d.Right);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
    }

    [Fact]
    public void OverlapCircleAll_ShouldResolveColliderIdsThroughOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSSphereCollider colliderA = CreateDynamicSphere(contextA, Vector3d.Zero);
        LSSphereCollider colliderB = CreateDynamicSphere(contextB, Vector3d.Zero);
        var hitsA = new SwiftList<LSRaycastHit>();
        var hitsB = new SwiftList<LSRaycastHit>();
        colliderA.Id.Should().Be(colliderB.Id);

        int countA = contextA.CircleQueries
            .OverlapCircleAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsA);
        int countB = contextB.CircleQueries
            .OverlapCircleAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsB);

        countA.Should().Be(1);
        countB.Should().Be(1);
        hitsA[0].Collider.Should().BeSameAs(colliderA);
        hitsB[0].Collider.Should().BeSameAs(colliderB);
        contextA.CircleQueries.Version.Should().Be(1);
        contextB.CircleQueries.Version.Should().Be(1);
    }

    [Fact]
    public void OverlapCircleInDirection_ShouldFilterByDirectionAndDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider right = CreateDynamicSphere(context, new Vector3d(2, 0, 0));
        CreateDynamicSphere(context, new Vector3d(-1, 0, 0));

        bool hit = context.CircleQueries.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)3,
            Vector3d.Right,
            out LSRaycastHit hitInfo,
            (Fixed64)2,
            IncludeLayerZero);

        hit.Should().BeTrue();
        hitInfo.Collider.Should().BeSameAs(right);
        hitInfo.Distance.Should().Be((Fixed64)1.5f);
    }

    private static LSSphereCollider CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return collider;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d(-4, -4, -4),
            new Vector3d(6, 6, 6));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
