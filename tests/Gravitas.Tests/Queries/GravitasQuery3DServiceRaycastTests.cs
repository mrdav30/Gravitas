using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GravitasQuery3DServiceRaycastTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void RaycastAll_ShouldReturnHitsOrderedByDistanceWithinContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicSphere(context, new Vector3d(0, 0, 0));
        LSSphereCollider far = CreateDynamicSphere(context, Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
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

        bool horizontalHit = context.Query3D.Raycast(
            Vector(-2, 0, 0),
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit horizontal,
            IncludeLayerZero);
        bool verticalHit = context.Query3D.Raycast(
            Vector(0, -2, 0),
            Vector3d.Up,
            (Fixed64)4,
            out Physics3DHit vertical,
            IncludeLayerZero);
        bool diagonalHit = context.Query3D.Raycast(
            Vector(-2, -2, -2),
            new Vector3d(1, 1, 1).Normalized,
            (Fixed64)4,
            out Physics3DHit diagonal,
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

        bool hit = context.Query3D.Raycast(
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)2,
            out Physics3DHit rayHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(collider);
        rayHit.Distance.Should().Be(Fixed64.Zero);
        rayHit.Point.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void Raycast_WithEqualDistanceHits_ShouldUseColliderIdTieBreaker()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider first = CreateDynamicSphere(context, Vector3d.Zero);
        _ = CreateDynamicSphere(context, Vector3d.Zero);

        bool hit = context.Query3D.Raycast(
            Vector(-2, 0, 0),
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit rayHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        rayHit.Collider.Should().BeSameAs(first);
    }

    [Fact]
    public void Raycast_ShouldHitCylinderSideAndCaps()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider collider = CreateDynamicCylinder(context, Vector3d.Zero);

        bool sideHit = context.Query3D.Raycast(
            Vector(-2, 0, 0),
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit side,
            IncludeLayerZero);
        bool capHit = context.Query3D.Raycast(
            Vector(0, 2, 0),
            -Vector3d.Up,
            (Fixed64)4,
            out Physics3DHit cap,
            IncludeLayerZero);

        sideHit.Should().BeTrue();
        capHit.Should().BeTrue();
        side.Collider.Should().BeSameAs(collider);
        cap.Collider.Should().BeSameAs(collider);
        side.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        cap.Point.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        side.Normal.Should().Be(-Vector3d.Right);
        cap.Normal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void RaycastAll_ShouldReturnNoHitsWhenSegmentMissesCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CreateDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
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
        var hitsA = new SwiftList<Physics3DHit>();
        var hitsB = new SwiftList<Physics3DHit>();
        colliderA.Id.Should().Be(colliderB.Id);
        uint versionABefore = contextA.Query3D.RaycastVersion;
        uint versionBBefore = contextB.Query3D.RaycastVersion;

        int countA = contextA.Query3D
            .RaycastAll(Vector(-2, 0, 0), Vector(2, 0, 0), IncludeLayerZero, hitsA);
        int countB = contextB.Query3D
            .RaycastAll(Vector(-2, 0, 0), Vector(2, 0, 0), IncludeLayerZero, hitsB);

        countA.Should().Be(1);
        countB.Should().Be(1);
        hitsA[0].Collider.Should().BeSameAs(colliderA);
        hitsB[0].Collider.Should().BeSameAs(colliderB);
        contextA.Query3D.RaycastVersion.Should().Be(versionABefore + 1);
        contextB.Query3D.RaycastVersion.Should().Be(versionBBefore + 1);
    }

    [Fact]
    public void RaycastAll_WithColliderSpanningManyVoxels_ShouldReturnSingleColliderHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateLargeDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
            .RaycastAll(Vector(-4, 0, 0), Vector(4, 0, 0), IncludeLayerZero, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void RaycastAll_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicSphere(context, Vector3d.Zero);
        LSSphereCollider far = CreateDynamicSphere(context, Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
            .RaycastAll(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero, hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () => context.Query3D.RaycastAll(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero, hits));

        allocatedBytes.Should().Be(0);
    }

    private static Vector3d Vector(int x, int y, int z) => new((Fixed64)x, (Fixed64)y, (Fixed64)z);

    private static LSSphereCollider CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        return CreateDynamicCollider(context, new LSSphereCollider(), position);
    }

    private static LSCylinderCollider CreateDynamicCylinder(GravitasWorldContext context, Vector3d position)
    {
        return CreateDynamicCollider(context, new LSCylinderCollider(), position);
    }

    private static LSSphereCollider CreateLargeDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider { Radius = (Fixed64)3 };
        return CreateDynamicCollider(context, collider, position);
    }

    private static TCollider CreateDynamicCollider<TCollider>(GravitasWorldContext context, TCollider collider, Vector3d position)
        where TCollider : LSCollider
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
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
