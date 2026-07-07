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

public sealed class GravitasQuery3DServiceCircleTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void OverlapCircleAll_ShouldSuppressDuplicateColliderHitsWithinContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
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
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
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
    public void OverlapCircle_ShouldReturnClosestLayerFilteredHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateDynamicSphere(context, new Vector3d(1, 0, 0), new PhysicsLayer(2));
        LSSphereCollider included = CreateDynamicSphere(context, new Vector3d(3, 0, 0));

        bool found = context.Query3D.OverlapCircle(
            Vector3d.Zero,
            (Fixed64)4,
            out Physics3DHit hit,
            IncludeLayerZero);

        found.Should().BeTrue();
        hit.Collider.Should().BeSameAs(included);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCircleAll_ShouldResolveColliderIdsThroughOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSSphereCollider colliderA = CreateDynamicSphere(contextA, Vector3d.Zero);
        LSSphereCollider colliderB = CreateDynamicSphere(contextB, Vector3d.Zero);
        var hitsA = new SwiftList<Physics3DHit>();
        var hitsB = new SwiftList<Physics3DHit>();
        colliderA.Id.Should().Be(colliderB.Id);

        int countA = contextA.Query3D
            .OverlapCircleAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsA);
        int countB = contextB.Query3D
            .OverlapCircleAll(Vector3d.Zero, Fixed64.One * 2, IncludeLayerZero, hitsB);

        countA.Should().Be(1);
        countB.Should().Be(1);
        hitsA[0].Collider.Should().BeSameAs(colliderA);
        hitsB[0].Collider.Should().BeSameAs(colliderB);
        contextA.Query3D.CircleVersion.Should().Be(1);
        contextB.Query3D.CircleVersion.Should().Be(1);
    }

    [Fact]
    public void OverlapCircleInDirection_ShouldFilterByDirectionAndDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider right = CreateDynamicSphere(context, new Vector3d(2, 0, 0));
        CreateDynamicSphere(context, new Vector3d(-1, 0, 0));

        bool hit = context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)3,
            Vector3d.Right,
            out Physics3DHit hitInfo,
            (Fixed64)2,
            IncludeLayerZero);

        hit.Should().BeTrue();
        hitInfo.Collider.Should().BeSameAs(right);
        hitInfo.Distance.Should().Be((Fixed64)1.5f);
    }

    [Fact]
    public void OverlapCircleInDirection_WithZeroDirection_ShouldReturnNoHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateDynamicSphere(context, new Vector3d(1, 0, 0));

        bool hit = context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)3,
            Vector3d.Zero,
            out Physics3DHit hitInfo,
            (Fixed64)3,
            IncludeLayerZero);

        hit.Should().BeFalse();
        hitInfo.Collider.Should().BeNull();
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void OverlapCircleInDirection_WithHitBeyondMaxDistance_ShouldReturnNoHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateDynamicSphere(context, new Vector3d(3, 0, 0));

        bool hit = context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)4,
            Vector3d.Right,
            out Physics3DHit hitInfo,
            Fixed64.One,
            IncludeLayerZero);

        hit.Should().BeFalse();
        hitInfo.Collider.Should().BeNull();
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCircleAll_WithColliderSpanningManyVoxels_ShouldReturnSingleColliderHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateLargeDynamicSphere(context, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D
            .OverlapCircleAll(Vector3d.Zero, (Fixed64)4, IncludeLayerZero, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void OverlapSphereAgainstStaticAll_ShouldFilterExcludedLayerTriggerAndMovableDynamicTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider excluded = CreateBodylessSphere(context, Vector3d.Right);
        LSSphereCollider included = CreateBodylessSphere(context, Vector3d.Right * 2);
        _ = CreateDynamicSphere(context, Vector3d.Right * 3);
        _ = CreateBodylessSphere(context, Vector3d.Right * 4, new PhysicsLayer(1));
        _ = CreateBodylessSphere(context, Vector3d.Right * 5, isTrigger: true);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            (Fixed64)6,
            IncludeLayerZero,
            hits,
            excluded,
            includeTriggers: false);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, excluded));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }


    private static LSSphereCollider CreateDynamicSphere(
        GravitasWorldContext context,
        Vector3d position,
        PhysicsLayer? layer = null)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return collider;
    }

    private static LSSphereCollider CreateBodylessSphere(
        GravitasWorldContext context,
        Vector3d position,
        PhysicsLayer? layer = null,
        bool isTrigger = false)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var collider = new LSSphereCollider
        {
            IsTrigger = isTrigger
        };
        if (layer.HasValue)
            collider.Layer = layer.Value;

        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSSphereCollider CreateLargeDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider { Radius = (Fixed64)3 };
        var body = new SolidBody(agent, collider)
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
