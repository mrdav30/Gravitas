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

public sealed class GravitasRaycastServiceTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void RaycastAll_ShouldReturnHitsOrderedByDistanceWithinContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicSphere(context, new Vector3d(0, 0, 0));
        LSSphereCollider far = CreateDynamicSphere(context, Vector3d.Right * 2);
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.Raycasts
            .RaycastAll(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero, hits);

        count.Should().Be(2);
        hits[0].Collider?.Id.Should().Be(near.Id);
        hits[1].Collider?.Id.Should().Be(far.Id);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
    }

    [Fact]
    public void Raycast_ShouldHitHorizontalVerticalAndDiagonalSegments()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);

        bool horizontalHit = context.Raycasts.Raycast(
            Vector(-2, 0, 0),
            Vector3d.Right,
            (Fixed64)4,
            out LSRaycastHit horizontal,
            IncludeLayerZero);
        bool verticalHit = context.Raycasts.Raycast(
            Vector(0, -2, 0),
            Vector3d.Up,
            (Fixed64)4,
            out LSRaycastHit vertical,
            IncludeLayerZero);
        bool diagonalHit = context.Raycasts.Raycast(
            Vector(-2, -2, -2),
            new Vector3d(1, 1, 1).Normal,
            (Fixed64)4,
            out LSRaycastHit diagonal,
            IncludeLayerZero);

        horizontalHit.Should().BeTrue();
        verticalHit.Should().BeTrue();
        diagonalHit.Should().BeTrue();
        horizontal.Collider.Should().BeSameAs(collider);
        vertical.Collider.Should().BeSameAs(collider);
        diagonal.Collider.Should().BeSameAs(collider);
        horizontal.Distance.Should().BeLessThan(vertical.Distance + Fixed64.Epsilon);
    }

    [Fact]
    public void Raycast_ShouldReturnZeroDistanceWhenSegmentStartsInsideCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);

        bool hit = context.Raycasts.Raycast(
            new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)2,
            out LSRaycastHit rayHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(collider);
        rayHit.Distance.Should().Be(Fixed64.Zero);
        rayHit.Point.Should().Be(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void RaycastAll_ShouldReturnNoHitsWhenSegmentMissesCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CreateDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.Raycasts
            .RaycastAll(Vector(-2, 2, 0), Vector(2, 2, 0), IncludeLayerZero, hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void RaycastAll_ShouldResolveColliderIdsThroughOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSSphereCollider colliderA = CreateDynamicSphere(contextA, new Vector3d(0, 0, 0));
        LSSphereCollider colliderB = CreateDynamicSphere(contextB, new Vector3d(0, 0, 0));
        var hitsA = new SwiftList<LSRaycastHit>();
        var hitsB = new SwiftList<LSRaycastHit>();
        colliderA.Id.Should().Be(colliderB.Id);

        int countA = contextA.Raycasts
            .RaycastAll(Vector(-2, 0, 0), Vector(2, 0, 0), IncludeLayerZero, hitsA);
        int countB = contextB.Raycasts
            .RaycastAll(Vector(-2, 0, 0), Vector(2, 0, 0), IncludeLayerZero, hitsB);

        countA.Should().Be(1);
        countB.Should().Be(1);
        hitsA[0].Collider.Should().BeSameAs(colliderA);
        hitsB[0].Collider.Should().BeSameAs(colliderB);
        contextA.Raycasts.Version.Should().Be(1);
        contextB.Raycasts.Version.Should().Be(1);
    }

    private static Vector3d Vector(int x, int y, int z) => new((Fixed64)x, (Fixed64)y, (Fixed64)z);

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
