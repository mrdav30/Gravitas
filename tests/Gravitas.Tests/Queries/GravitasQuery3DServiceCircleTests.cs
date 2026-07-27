using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
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
    public void CircleVersionWrap_ShouldNotSuppressColliderFromPreviousVersionOneQuery()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context, Vector3d.Zero);
        collider.CircleQueryVersion = 1;
        context.Query3D.CircleVersion = uint.MaxValue;
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.OverlapCircleAll(
            Vector3d.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        context.Query3D.CircleVersion.Should().Be(1);
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
    public void OverlapCircleAll_AtOpenMeshBoundsCenter_ShouldUseAuthoredSurface()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor();
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.OverlapCircleAll(
            Vector3d.Zero,
            Fixed64.FromFraction(1, 16),
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(mesh);
        hits[0].Point.Should().Be(Vector3d.Zero);
        hits[0].Distance.Should().Be(Fixed64.Zero);
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
    public void OverlapCircle_AtCuboidCenter_ShouldUseNearestFaceDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider cuboid =
            CreateBodylessCuboid(context, Vector3d.Zero);

        context.Query3D.OverlapCircle(
                Vector3d.Zero,
                Fixed64.FromFraction(3, 5),
                out Physics3DHit hit,
                IncludeLayerZero)
            .Should().BeTrue();

        hit.Collider.Should().BeSameAs(cuboid);
        hit.Distance.Should().Be(Fixed64.Half);
        hit.Anchor.TryGetOffsetFrom(
                Vector3d.Zero,
                out Vector3d surfaceOffset)
            .Should().BeTrue();
        surfaceOffset.Should().Be(Vector3d.Right * Fixed64.Half);
    }

    [Fact]
    public void OverlapCircle_WithNoCandidate_ShouldEmitDeterministicMissDiagnostic()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Diagnostics.Enable();

        bool found = context.Query3D.OverlapCircle(
            Vector3d.Zero,
            Fixed64.One,
            out Physics3DHit hit,
            IncludeLayerZero);

        found.Should().BeFalse();
        hit.Collider.Should().BeNull();
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
        context.Diagnostics.EventCount.Should().Be(1);
        GravitasDiagnosticEvent diagnostic = context.Diagnostics.Events[0];
        diagnostic.Kind.Should().Be(GravitasDiagnosticEventKind.CircleQuery);
        diagnostic.Hit.Should().BeFalse();
        diagnostic.DataB.Should().Be(0);
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
    public void OverlapCircleInDirection_WithSmallRepresentableDirection_ShouldUseRobustNormalization()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicSphere(context, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)3,
            new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            out Physics3DHit hitInfo,
            (Fixed64)3,
            IncludeLayerZero);

        hit.Should().BeTrue();
        hitInfo.Collider.Should().BeSameAs(target);
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
    public void OverlapQueries_ShouldRejectBroadAndNarrowPhaseMissesFromTracedPartition()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider broadMiss = CreateBodylessSphere(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        LSCuboidCollider narrowMiss = CreateBodylessCuboid(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Fixed64 radius = Fixed64.FromFraction(1, 8);
        var circleHits = new SwiftList<Physics3DHit>();
        var sphereHits = new SwiftList<Physics3DHit>();
        context.Diagnostics.Enable();

        broadMiss.Center.Magnitude.Should().BeGreaterThan(broadMiss.ScaledRadius + radius);
        narrowMiss.Center.Magnitude.Should().BeLessThanOrEqualTo(narrowMiss.ScaledRadius + radius);
        narrowMiss.ClosestPointOnSurface(Vector3d.Zero).Magnitude.Should().BeGreaterThan(radius);

        int circleCount = context.Query3D.OverlapCircleAll(Vector3d.Zero, radius, IncludeLayerZero, circleHits);
        int circleCandidates = context.Query3D.LastQueryCandidateCount;
        int sphereCount = context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            radius,
            IncludeLayerZero,
            sphereHits);

        circleCount.Should().Be(0);
        sphereCount.Should().Be(0);
        circleHits.Count.Should().Be(0);
        sphereHits.Count.Should().Be(0);
        circleCandidates.Should().Be(2);
        context.Query3D.LastQueryCandidateCount.Should().Be(2);
        context.Diagnostics.EventCount.Should().Be(1);
        GravitasDiagnosticEvent diagnostic = context.Diagnostics.Events[0];
        diagnostic.Kind.Should().Be(GravitasDiagnosticEventKind.CircleQuery);
        diagnostic.Hit.Should().BeFalse();
        diagnostic.DataB.Should().Be(0);
        diagnostic.ColliderAId.Should().Be(-1);
        diagnostic.ColliderAType.Should().Be(ColliderType.None);
    }

    [Fact]
    public void OverlapQueries_WithStalePartitionColliderId_ShouldIgnoreStaleEntry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateBodylessSphere(context, Vector3d.Zero);
        for (int i = 0; i < collider.PartitionCoordinates!.Count; i++)
        {
            context.World.TryGetVoxel(collider.PartitionCoordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.ContainedStaticObjects!.Add(collider.Id + 1).Should().BeTrue();
        }

        context.Physics.TryGetColliderById(collider.Id + 1, out _).Should().BeFalse();
        var circleHits = new SwiftList<Physics3DHit>();
        var sphereHits = new SwiftList<Physics3DHit>();

        int circleCount = context.Query3D.OverlapCircleAll(
            Vector3d.Zero,
            Fixed64.One,
            IncludeLayerZero,
            circleHits);
        int sphereCount = context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            Fixed64.One,
            IncludeLayerZero,
            sphereHits);

        circleCount.Should().Be(1);
        sphereCount.Should().Be(1);
        circleHits[0].Collider.Should().BeSameAs(collider);
        sphereHits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void ClosestOverlapQueries_ShouldKeepNearerHitWhenFartherCandidateIsProcessedLater()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateDynamicSphere(context, Vector3d.Right * 2);
        LSSphereCollider nearer = CreateDynamicSphere(context, Vector3d.Right);

        bool closestFound = context.Query3D.OverlapCircle(
            Vector3d.Zero,
            (Fixed64)3,
            out Physics3DHit closest,
            IncludeLayerZero);
        bool directionalFound = context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)3,
            Vector3d.Right,
            out Physics3DHit directional,
            (Fixed64)3,
            IncludeLayerZero);

        closestFound.Should().BeTrue();
        directionalFound.Should().BeTrue();
        closest.Collider.Should().BeSameAs(nearer);
        directional.Collider.Should().BeSameAs(nearer);
        closest.Distance.Should().Be(Fixed64.Half);
        directional.Distance.Should().Be(Fixed64.Half);
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

    [Fact]
    public void OverlapSphereAgainstStaticAll_ShouldSuppressLinkedTargetsForExcludedCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> linked = scenario.CreateSphere(Vector3d.Right, immovable: true);
        ScenarioBody<LSSphereCollider> included = scenario.CreateSphere(Vector3d.Right * 2, immovable: true);
        _ = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(source.Body, linked.Body));
        var hits = new SwiftList<Physics3DHit>();

        int count = scenario.Context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            (Fixed64)3,
            IncludeLayerZero,
            hits,
            source.Collider);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapSphereAgainstStaticAll_ShouldIncludeTriggerWhenRequested()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider trigger = CreateBodylessSphere(context, Vector3d.Right, isTrigger: true);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits,
            excludedCollider: null,
            includeTriggers: true);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(trigger);
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapSphereAgainstStaticAll_WithNonPositiveRadius_ShouldReturnNoHits()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateBodylessSphere(context, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            Fixed64.Zero,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
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

    private static LSCuboidCollider CreateBodylessCuboid(GravitasWorldContext context, Vector3d position)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var collider = new LSCuboidCollider();
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

    private static JointDefinition3D CreateBallSocket(SolidBody first, SolidBody second)
    {
        return new JointDefinition3D(
            first,
            second,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked);
    }

    private static FixedTransform LocalFrame(Vector3d position) =>
        new(position, FixedQuaternion.Identity, Vector3d.One);
}
