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

public sealed class GravitasCirclecastServiceTests
{
    private static readonly SingleLayer IncludeLayerZero = new(0);

    [Fact]
    public void CircleCastAll_ShouldSuppressDuplicateColliderHitsWithinContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.Circlecasts
            .CircleCastAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void CircleCastAll_ShouldResolveColliderIdsThroughOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSSphereCollider colliderA = CreateDynamicSphere(contextA, Vector3d.Zero);
        LSSphereCollider colliderB = CreateDynamicSphere(contextB, Vector3d.Zero);
        var hitsA = new SwiftList<LSRaycastHit>();
        var hitsB = new SwiftList<LSRaycastHit>();
        colliderA.Id.Should().Be(colliderB.Id);

        int countA = contextA.Circlecasts
            .CircleCastAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsA);
        int countB = contextB.Circlecasts
            .CircleCastAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsB);

        countA.Should().Be(1);
        countB.Should().Be(1);
        hitsA[0].Collider.Should().BeSameAs(colliderA);
        hitsB[0].Collider.Should().BeSameAs(colliderB);
        contextA.Circlecasts.Version.Should().Be(1);
        contextB.Circlecasts.Version.Should().Be(1);
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
