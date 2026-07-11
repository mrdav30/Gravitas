using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GravitasQuery3DServiceSweepTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);
    private static readonly Fixed64 QueryTolerance = Fixed64.FromFraction(1, 1_000_000);

    public static TheoryData<Vector3d, Vector3d, Vector3d, Vector3d, Vector3d> ConvexSweepHitNormalCases => new()
    {
        { Vector3d.Right, Vector3d.Zero, Vector3d.Zero, Vector3d.Right, -Vector3d.Right },
        { -Vector3d.Right, Vector3d.Zero, Vector3d.Zero, Vector3d.Right, -Vector3d.Right },
        { Vector3d.Zero, Vector3d.Right, Vector3d.Zero, Vector3d.Right, -Vector3d.Right },
        { Vector3d.Zero, Vector3d.Zero, Vector3d.Up, Vector3d.Right, Vector3d.Up },
        { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Right, -Vector3d.Right },
        { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero }
    };

    [Fact]
    public void SweepSphere_ShouldReportTimeOfImpactAndTargetSurfacePoint()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.One);
        sweepHit.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector3d.Right);
        sweepHit.Direction.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweepSphere_ShouldReturnStartingOverlapAtZeroDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepSphere(
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)2,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
        sweepHit.Point.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 0)]
    [InlineData(1, -1)]
    public void SweepSphere_WithInvalidDirectionOrDistance_ShouldReturnFalse(int directionX, int maxDistance)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right * (Fixed64)directionX,
            (Fixed64)maxDistance,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepSphere_WithZeroRadius_ShouldReturnFalseWithoutTraversingCandidates()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 0)]
    [InlineData(1, -1)]
    public void SweepSphereAll_WithInvalidSegmentOrRadius_ShouldReturnZeroAndClearResults(int segmentLength, int radius)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit> { new() };

        int count = context.Query3D.SweepSphereAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)(-2 + segmentLength), Fixed64.Zero, Fixed64.Zero),
            (Fixed64)radius,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAll_ShouldSuppressDuplicatesAndOrderByImpactDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
    }

    [Fact]
    public void SweepSphere_ShouldKeepCloserHitWhenFartherColliderIsVisitedLater()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)8,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(near);
        sweepHit.Collider.Should().NotBeSameAs(far);
        sweepHit.Distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void SweepSphereAll_ShouldIncludeMovableKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider movable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider kinematic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2, isKinematic: true);
        LSSphereCollider immovable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 4, immovable: true);
        LSSphereCollider bodyless = CreateBodylessCollider(context, Vector3d.Right * 6);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, movable));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void SweepSphereAgainstStaticAll_ShouldSkipMovableDynamicTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        _ = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAgainstStaticAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source,
            includeTriggers: false);

        count.Should().Be(0);
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAgainstStatic_ShouldSkipNearMovableDynamicAndHitStaticStyleTarget()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider movable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider kinematic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2, isKinematic: true);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAgainstStaticAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source,
            includeTriggers: true);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(kinematic);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, movable));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainstStaticAll_ShouldIncludeKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider kinematic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero, isKinematic: true);
        LSSphereCollider immovable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2, immovable: true);
        LSSphereCollider nonDynamic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 4, isDynamic: false);
        LSSphereCollider bodyless = CreateBodylessCollider(context, Vector3d.Right * 6);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAgainstStaticAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source,
            includeTriggers: false);

        count.Should().Be(4);
        context.Query3D.LastQueryCandidateCount.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, nonDynamic));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void SweepExactSourceAgainstStaticAll_ShouldUseStaticMobilityAndTriggerFilters()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        _ = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider kinematic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2, isKinematic: true);
        LSSphereCollider immovable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 4, immovable: true);
        LSSphereCollider bodyless = CreateBodylessCollider(context, Vector3d.Right * 6);
        LSSphereCollider trigger = CreateBodylessCollider(context, Vector3d.Right * 8);
        trigger.IsTrigger = true;
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepExactSourceAgainstStaticAll(
            source,
            Vector3d.Right * (Fixed64)12,
            IncludeLayerZero,
            hits,
            excludedCollider: source,
            includeTriggers: false);

        count.Should().Be(3);
        context.Query3D.LastQueryCandidateCount.Should().Be(3);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, trigger));
    }

    [Fact]
    public void SweepExactSourceAgainstStaticAll_ShouldSkipNearMovableDynamicAndHitStaticStyleTarget()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider movable = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider kinematic = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2, isKinematic: true);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepExactSourceAgainstStaticAll(
            source,
            Vector3d.Right * (Fixed64)8,
            IncludeLayerZero,
            hits,
            excludedCollider: source,
            includeTriggers: true);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(kinematic);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, movable));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphere_ShouldBreakClosestHitTiesByColliderId()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider first = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4)));
        CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.FromFraction(1, 4)));

        bool hit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.FromFraction(1, 4),
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(first);
    }

    [Fact]
    public void SweepSphere_ShouldHonorLayerMaskAndExcludedCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider self = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider ignoredByMask = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right);
        ignoredByMask.Layer = new PhysicsLayer(1);
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);

        bool hit = context.Query3D.SweepSphere(
            Vector3d.Zero,
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit sweepHit,
            IncludeLayerZero,
            self);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Collider.Should().NotBeSameAs(self);
        sweepHit.Collider.Should().NotBeSameAs(ignoredByMask);
    }

    [Fact]
    public void SweepSphere_ShouldSupportCapsuleCuboidCylinderAndRotatedTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateDynamicCollider(context, new LSCapsuleCollider(), Vector3d.Zero);
        LSCuboidCollider cuboid = CreateDynamicCollider(context, new LSCuboidCollider(), new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        LSCuboidCollider rotatedCuboid = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));

        AssertSweepHits(context, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), capsule);
        AssertSweepHits(context, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero), cuboid);
        AssertSweepHits(context, new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero), cylinder);
        AssertSweepHits(context, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero), rotatedCuboid);
    }

    [Fact]
    public void SweepSphere_ShouldSupportMeshAndCompoundTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        LSCompoundCollider compound = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero);

        bool meshHit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)6,
            out Physics3DHit meshSweepHit,
            IncludeLayerZero);
        bool compoundHit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)10,
            out Physics3DHit compoundSweepHit,
            IncludeLayerZero);

        meshHit.Should().BeTrue();
        meshSweepHit.Collider.Should().BeSameAs(mesh);
        meshSweepHit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        meshSweepHit.Point.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        compoundHit.Should().BeTrue();
        compoundSweepHit.Collider.Should().BeSameAs(compound);
        compoundSweepHit.Distance.Should().Be(Fixed64.One);
    }

    [Fact]
    public void SweepSphereAll_ShouldCollapseMeshTrianglesToOwnerAndOrderMeshTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider near = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        LSMeshCollider far = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAll(
            new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)6, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[0].Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hits[1].Collider.Should().BeSameAs(far);
        hits[1].Distance.Should().Be(Fixed64.FromFraction(9, 2));
    }

    [Fact]
    public void SweepSphereAll_WithMeshTarget_ShouldReportTriangleCandidateCount()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        _ = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepSphereAll(
            new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(2);

        _ = context.Query3D.OverlapCircleAll(Vector3d.Zero, (Fixed64)2, IncludeLayerZero, hits);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepConvexMesh_ShouldHitSphereTargetAsSource()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be((Fixed64)2);
        sweepHit.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCapsule_ShouldUseRotatedCapsuleSourceGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider source = CreateDynamicCollider(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCapsule(
            source,
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        AssertDistanceNear(sweepHit.Distance, (Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCuboidAll_ShouldSuppressSourceAndOrderTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepCuboidAll(
            source,
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[0].Distance.Should().Be((Fixed64)2);
        hits[1].Collider.Should().BeSameAs(far);
        hits[1].Distance.Should().Be((Fixed64)4);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, source));
    }

    [Theory]
    [InlineData(ColliderType.Capsule)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    [InlineData(ColliderType.Compound)]
    public void RegisteredSourceAllHitSweeps_ShouldSuppressSourceAndOrderTargets(ColliderType sourceType)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider source = CreateDynamicCollider(
            context,
            CreateAllHitSweepSource(sourceType),
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = source switch
        {
            LSCapsuleCollider capsule => context.Query3D.SweepCapsuleAll(
                capsule, Vector3d.Right * (Fixed64)6, IncludeLayerZero, hits),
            LSCylinderCollider cylinder => context.Query3D.SweepCylinderAll(
                cylinder, Vector3d.Right * (Fixed64)6, IncludeLayerZero, hits),
            LSConeCollider cone => context.Query3D.SweepConeAll(
                cone, Vector3d.Right * (Fixed64)6, IncludeLayerZero, hits),
            LSCompoundCollider compound => context.Query3D.SweepCompoundAll(
                compound, Vector3d.Right * (Fixed64)6, IncludeLayerZero, hits),
            _ => throw new InvalidOperationException($"Unsupported source type {sourceType}.")
        };

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, source));
    }

    [Fact]
    public void SweepCuboid_WithZeroDisplacement_ShouldReturnFalseAndResetCounters()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Zero);
        _ = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right);

        bool hit = context.Query3D.SweepCuboid(
            source,
            Vector3d.Zero,
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCuboidAll_WithZeroDisplacement_ShouldClearResultsAndResetCounters()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Zero);
        _ = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right);
        var hits = new SwiftList<Physics3DHit> { new() };

        int count = context.Query3D.SweepCuboidAll(
            source,
            Vector3d.Zero,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCuboidAll_WithStalePartitionColliderId_ShouldIgnoreStaleEntry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Left * 3);
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        int staleId = target.Id + 1_000;
        for (int i = 0; i < target.PartitionCoordinates!.Count; i++)
        {
            context.World.TryGetVoxel(target.PartitionCoordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.ContainedDynamicObjects!.Add(staleId).Should().BeTrue();
        }

        context.Physics.TryGetColliderById(staleId, out _).Should().BeFalse();
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepCuboidAll(
            source,
            Vector3d.Right * (Fixed64)6,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(target);
    }

    [Fact]
    public void StaticOnlyExactSweep_WithDynamicColliderRetainedInStaticSet_ShouldRejectStaleMobility()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Left * 3);
        LSSphereCollider dynamicTarget = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        for (int i = 0; i < dynamicTarget.PartitionCoordinates!.Count; i++)
        {
            context.World.TryGetVoxel(dynamicTarget.PartitionCoordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.ContainedStaticObjects ??= new SwiftSparseSet();
            partition.ContainedStaticObjects.Add(dynamicTarget.Id).Should().BeTrue();
        }

        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepExactSourceAgainstStaticAll(
            source,
            Vector3d.Right * (Fixed64)6,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
        context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCuboid_ShouldKeepCloserHitWhenFartherColliderIsVisitedLater()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCuboid(
            source,
            Vector3d.Right * (Fixed64)8,
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(near);
        sweepHit.Collider.Should().NotBeSameAs(far);
        sweepHit.Distance.Should().Be((Fixed64)3);
    }

    [Fact]
    public void SweepCuboid_ShouldSuppressTargetsLinkedToSource()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider linked = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider included = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        _ = context.Constraints3D.RegisterJoint(CreateBallSocket(source.Body!, linked.Body!));

        bool hit = context.Query3D.SweepCuboid(
            source,
            Vector3d.Right * (Fixed64)8,
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(included);
        sweepHit.Collider.Should().NotBeSameAs(linked);
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepCylinder_ShouldUseRotatedCylinderSourceGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider source = CreateDynamicCollider(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCylinder(
            source,
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        AssertDistanceNear(sweepHit.Distance, (Fixed64)2);
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCone_ShouldUseAnalyticConeSourceGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider source = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCone(
            source,
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().BeLessThan((Fixed64)3);
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCuboid_WithInitialCenterOverlap_ShouldReturnStableFallbackSurfacePointAndNormal()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCuboidCollider source = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            Vector3d.Zero);
        LSCuboidCollider target = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCuboid(
            source,
            Vector3d.Right * (Fixed64)2,
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
        sweepHit.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepConvexMesh_ShouldHitCapsuleAndCylinderTargetsAsSource()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        LSCapsuleCollider capsule = CreateDynamicCollider(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero);
        LSCylinderCollider cylinder = CreateDynamicCollider(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector3d.Right * 3);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepConvexMeshAll(
            source,
            new Vector3d((Fixed64)9, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(capsule);
        hits[0].Distance.Should().Be((Fixed64)2);
        hits[0].Normal.Should().Be(-Vector3d.Right);
        hits[1].Collider.Should().BeSameAs(cylinder);
        hits[1].Distance.Should().Be((Fixed64)5);
        hits[1].Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepConvexMesh_ShouldReturnCompoundTargetOwnerThroughNearestPartGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSCompoundCollider target = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be((Fixed64)3);
        sweepHit.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepConvexMesh_WithEqualDistanceCompoundParts_ShouldUseStablePartOrdering()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSCompoundCollider target = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Up),
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Up)),
            Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().BeGreaterThan((Fixed64)3);
        sweepHit.Distance.Should().BeLessThan(Fixed64.FromFraction(31, 10));
        sweepHit.Point.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void SweepConvexMeshAll_ShouldSupportConcaveMeshTargetsAsOrderedTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero));
        LSMeshCollider near = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                mode: MeshColliderMode.Concave,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        LSMeshCollider far = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                mode: MeshColliderMode.Concave,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Right * 2);
        var hits = new SwiftList<Physics3DHit>();

        int count = context.Query3D.SweepConvexMeshAll(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[0].Distance.Should().Be(Fixed64.FromFraction(7, 2));
        hits[1].Collider.Should().BeSameAs(far);
        hits[1].Distance.Should().Be(Fixed64.FromFraction(11, 2));
        context.Query3D.LastQueryCandidateCount.Should().Be(2);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(4);
    }

    [Fact]
    public void SweepConvexMesh_WithEqualDistanceConcaveTriangles_ShouldUseTriangleOrdinal()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero));
        LSMeshCollider target = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, Fixed64.Half, -Fixed64.Half),
                    new Vector3d(Fixed64.Zero, (Fixed64)1.5f, -Fixed64.Half),
                    new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.FromFraction(1, 4)),
                    new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half),
                    new Vector3d(Fixed64.Zero, (Fixed64)1.5f, Fixed64.Half),
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(1, 4)),
                    new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, (Fixed64)1.5f, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(1, 8))
                },
                new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool firstHit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit first);
        int firstCandidateCount = context.Query3D.LastMeshTriangleCandidateCount;
        bool secondHit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit second);

        firstHit.Should().BeTrue();
        secondHit.Should().BeTrue();
        first.Collider.Should().BeSameAs(target);
        AssertDistanceNear(first.Distance, Fixed64.FromFraction(7, 2));
        first.Point.Z.Should().BeLessThan(Fixed64.Zero);
        second.Collider.Should().BeSameAs(target);
        second.Point.Should().Be(first.Point);
        second.Normal.Should().Be(first.Normal);
        second.Distance.Should().Be(first.Distance);
        firstCandidateCount.Should().Be(3);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(3);
    }

    [Fact]
    public void SweepConvexMesh_WithConcaveBroadCandidateExactMiss_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero));
        Fixed64 gap = Fixed64.FromFraction(1, 5000);
        LSMeshCollider target = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, (Fixed64)1.5f + gap, Fixed64.Half + gap),
                    new Vector3d(Fixed64.Zero, (Fixed64)2 + gap, Fixed64.Half + gap),
                    new Vector3d(Fixed64.Zero, (Fixed64)1.5f + gap, Fixed64.One + gap)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        target.BoundsMin.Y.Should().BeLessThan(source.BoundsMax.Y + Fixed64.FromFraction(1, 4096));
        target.BoundsMin.Z.Should().BeLessThan(source.BoundsMax.Z + Fixed64.FromFraction(1, 4096));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepConvexMesh_WithLaterFartherSuccessfulTriangle_ShouldRetainNearerHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero));
        Fixed64 gap = Fixed64.FromFraction(1, 8192);
        LSMeshCollider target = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(gap, Fixed64.Zero, -Fixed64.One),
                    new Vector3d(gap, (Fixed64)2, -Fixed64.One),
                    new Vector3d(gap, Fixed64.One, Fixed64.One),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, (Fixed64)2, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One)
                },
                new[] { 0, 1, 2, 3, 4, 5 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool firstHit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit first);
        int firstCandidateCount = context.Query3D.LastMeshTriangleCandidateCount;
        bool secondHit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit second);

        firstHit.Should().BeTrue();
        secondHit.Should().BeTrue();
        first.Collider.Should().BeSameAs(target);
        first.Point.X.Should().Be(Fixed64.Zero);
        AssertDistanceNear(first.Distance, Fixed64.FromFraction(7, 2));
        second.Point.Should().Be(first.Point);
        second.Normal.Should().Be(first.Normal);
        second.Distance.Should().Be(first.Distance);
        firstCandidateCount.Should().Be(2);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(2);
    }

    [Fact]
    public void SweepConvexMesh_WithConcaveLowerBoundBeyondSweep_ShouldExcludeTriangle()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), (Fixed64)(-4), Fixed64.Zero));
        LSMeshCollider target = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.FromFraction(9, 2), (Fixed64)5, -Fixed64.FromFraction(1, 4)),
                    new Vector3d((Fixed64)5, Fixed64.FromFraction(9, 2), -Fixed64.FromFraction(1, 4)),
                    new Vector3d((Fixed64)5, (Fixed64)5, Fixed64.FromFraction(1, 4))
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, (Fixed64)8, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        context.Query3D.LastQueryCandidateCount.Should().Be(1);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepConvexMesh_WithNextLowerBoundBeyondClosestHit_ShouldStopAtNearTriangle()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero));
        LSMeshCollider target = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, (Fixed64)2, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One),
                    new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.One),
                    new Vector3d(Fixed64.One, (Fixed64)2, -Fixed64.One),
                    new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
                },
                new[] { 0, 1, 2, 3, 4, 5 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool hit = context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Point.X.Should().Be(Fixed64.Zero);
        AssertDistanceNear(sweepHit.Distance, Fixed64.FromFraction(7, 2));
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(2);
    }

    [Fact]
    public void SweepConvexMesh_WithConcaveSource_ShouldThrow()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateUChannel(
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));

        Action query = () => context.Query3D.SweepConvexMesh(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out _);

        query.Should().Throw<ArgumentException>().WithMessage("*Concave mesh sources*");
    }

    [Fact]
    public void SweepCompound_ShouldSweepAuthoredConvexPartsAsSource()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var source = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.ConvexMesh(
                    MeshTestFixtures.CreateConvexCube().Mesh.LocalVertices,
                    MeshTestFixtures.CreateConvexCube().Mesh.Triangles,
                    new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                    MeshInertiaPolicy.SurfaceApproximation),
                CompoundColliderPart.ConvexMesh(
                    MeshTestFixtures.CreateConvexCube().Mesh.LocalVertices,
                    MeshTestFixtures.CreateConvexCube().Mesh.Triangles,
                    new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                    MeshInertiaPolicy.SurfaceApproximation)),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Query3D.SweepCompound(
            source,
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
    }

    [Fact]
    public void ConvexSweepWorker_WithCompoundSourcePartsMissingTarget_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCompoundCollider source = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-2)))),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCompoundSource(source, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WithCompoundSourceMissBeforeHit_ShouldReturnLaterHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCompoundCollider source = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2)),
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero)),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCompoundSource(source, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be((Fixed64)3);
    }

    [Fact]
    public void ConvexSweepWorker_WithCompoundSourceLaterFartherHit_ShouldKeepEarlierHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCompoundCollider source = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right),
                CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Right)),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCompoundSource(source, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ConvexSweepWorker_WithCompoundSourceEqualDistanceParts_ShouldKeepLowerPartOrdinal()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCompoundCollider source = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4))),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.FromFraction(1, 4)))),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCompoundSource(source, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Point.Z.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConvexSweepWorker_WithCompoundTargetPartsMissingSource_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSCompoundCollider target = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-2)))),
            Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PreparePrimitiveSource(source, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WithVerticalCircleSlabSource_ShouldHitSphere()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCircleSlabSource(
            new Vector3d(Fixed64.Zero, (Fixed64)(-4), Fixed64.Zero),
            Fixed64.Half,
            Fixed64.FromFraction(1, 4),
            new Vector3d(Fixed64.Zero, (Fixed64)8, Fixed64.Zero));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        sweepHit.Direction.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void ConvexSweepWorker_WithCircleSlabSourceAndTargetBoundsMiss_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        var worker = new ConvexSweepQueryWorker();

        worker.PrepareCircleSlabSource(
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.FromFraction(1, 4),
            Vector3d.Right);
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WithoutPreparedSource_ShouldRejectTarget()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
        worker.LastMeshTriangleCandidateCount.Should().Be(0);
    }

    [Fact]
    public void ConvexSweepWorker_WithZeroDisplacement_ShouldRejectTarget()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), -Vector3d.Right);
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PreparePrimitiveSource(source, Vector3d.Zero);
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WhenSweptBoundsMissTarget_ShouldRejectBeforeNarrowPhase()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)(-6), Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        var worker = new ConvexSweepQueryWorker();

        worker.PreparePrimitiveSource(source, Vector3d.Right);
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WithOverlappingBoundsButMovingAway_ShouldReject()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5)));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PreparePrimitiveSource(source, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Fact]
    public void ConvexSweepWorker_WithOverlappingBoundsButTooShortApproach_ShouldReject()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider source = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5)));
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        worker.PreparePrimitiveSource(
            source,
            new Vector3d(-Fixed64.FromFraction(1, 20), Fixed64.Zero, -Fixed64.FromFraction(1, 20)));
        bool hit = worker.TrySweepPreparedSource(target, out Physics3DHit sweepHit);

        hit.Should().BeFalse();
        sweepHit.Should().Be(default(Physics3DHit));
    }

    [Theory]
    [MemberData(nameof(ConvexSweepHitNormalCases))]
    public void ConvexSweepHitPolicy_ShouldResolveStableNormals(
        Vector3d point,
        Vector3d resultNormal,
        Vector3d fallbackNormal,
        Vector3d sweepDirection,
        Vector3d expected)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        Vector3d normal = ConvexSweepHitPolicy.ResolveHitNormal(
            target,
            point,
            resultNormal,
            fallbackNormal,
            sweepDirection);

        normal.Should().Be(expected);
    }

    [Fact]
    public void ConvexSweepWorker_WithUnsupportedPrimitiveSource_ShouldThrow()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider source = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateUChannel(
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();

        Action query = () => worker.PreparePrimitiveSource(source, Vector3d.Right);

        query.Should().Throw<NotSupportedException>().WithMessage("*LSMeshCollider sources*");
    }

    [Theory]
    [InlineData(0, 1, "radius")]
    [InlineData(1, 0, "halfHeight")]
    public void ConvexSweepWorker_WithInvalidCircleSlabSource_ShouldThrow(
        int radius,
        int halfHeight,
        string parameterName)
    {
        var worker = new ConvexSweepQueryWorker();

        Action query = () => worker.PrepareCircleSlabSource(
            Vector3d.Zero,
            (Fixed64)radius,
            (Fixed64)halfHeight,
            Vector3d.Right);

        query.Should().Throw<ArgumentException>().Where(exception => exception.ParamName == parameterName);
    }

    [Fact]
    public void SweepSphere_ShouldOrientMeshNormalsAgainstSweepDirection()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);

        bool leftHit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit leftSweepHit,
            IncludeLayerZero);
        bool rightHit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            -Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit rightSweepHit,
            IncludeLayerZero);

        leftHit.Should().BeTrue();
        leftSweepHit.Collider.Should().BeSameAs(mesh);
        leftSweepHit.Normal.Should().Be(-Vector3d.Right);
        rightHit.Should().BeTrue();
        rightSweepHit.Collider.Should().BeSameAs(mesh);
        rightSweepHit.Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweptSphereWorker_WithQuantizedZeroMeshEdgeLength_ShouldUseOtherTriangleFeatures()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 shortEdge = Fixed64.FromRaw(1 << 14);
        LSMeshCollider mesh = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    Vector3d.Zero,
                    new Vector3d(shortEdge, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, (Fixed64)8, Fixed64.Zero)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        Vector3d start = new(shortEdge / (Fixed64)4, Fixed64.One, -Fixed64.One);
        Vector3d end = new(start.X, start.Y, Fixed64.One);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        (shortEdge * shortEdge).Should().Be(Fixed64.Zero);
        AssertSweptSphereBroadOverlap(start, end, radius, mesh);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(mesh, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        centerAtImpact.Should().Be(new Vector3d(start.X, start.Y, -radius));
        AssertDistanceNear(distance, Fixed64.One - radius);
        worker.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweptSphereWorker_WithMutatedZeroMeshFaceNormal_ShouldSkipTriangleAndReturnDefault()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = CreateDynamicCollider(
            context,
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        mesh.Mesh.FaceNormals[0] = Vector3d.Zero;
        var worker = new SweptSphereQueryWorker();
        Vector3d start = -Vector3d.Right;
        Vector3d end = Vector3d.Right;
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        mesh.Mesh.GetFaceNormalWorld(0).Should().Be(Vector3d.Zero);
        AssertSweptSphereBroadOverlap(start, end, radius, mesh);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(mesh, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
        worker.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweptSphereWorker_WithBroadOverlapExactMisses_ShouldDefaultCapsuleMeshAndCompoundOutputs()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateDynamicCollider(context, new LSCapsuleCollider(), Vector3d.Zero);
        LSMeshCollider mesh = CreateDynamicCollider(
            context,
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.Half,
                Fixed64.Half,
                mode: MeshColliderMode.Concave,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        LSCompoundCollider compound = CreateDynamicCollider(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Forward),
                CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Forward)),
            Vector3d.Zero);

        AssertSweptSphereExactMissReturnsDefault(
            capsule,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.FromFraction(7, 10)),
            Fixed64.FromFraction(1, 10));
        AssertSweptSphereExactMissReturnsDefault(
            mesh,
            new Vector3d(-Fixed64.FromFraction(1, 10), Fixed64.FromFraction(41, 20), Fixed64.FromFraction(11, 20)),
            new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.FromFraction(43, 20), Fixed64.FromFraction(13, 20)),
            Fixed64.FromFraction(1, 20));
        AssertSweptSphereExactMissReturnsDefault(
            compound,
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Fixed64.FromFraction(1, 10));
    }

    [Fact]
    public void SweptSphereWorker_ShouldDetectCylinderSideImpact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        var worker = new SweptSphereQueryWorker();
        Vector3d origin = new((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
        worker.Prepare(origin, origin + Vector3d.Right * (Fixed64)4, Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be(Fixed64.FromFraction(5, 4));
        centerAtImpact.Should().Be(new Vector3d((Fixed64)7 + Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SweptSphereWorker_ShouldDetectCylinderCapImpact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        Vector3d start = new(Fixed64.Zero, (Fixed64)2, Fixed64.Zero);
        Vector3d end = new(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        centerAtImpact.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 5), Fixed64.Zero));
        AssertDistanceNear(distance, Fixed64.FromFraction(7, 5));
    }

    [Fact]
    public void SweptSphereWorker_WithCylinderSideRootBeyondTravel_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Vector3d shortStart = new(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half);
        Vector3d shortEnd = new(-Fixed64.FromFraction(9, 20), Fixed64.Zero, -Fixed64.FromFraction(9, 20));
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(shortStart, shortEnd, radius, cylinder);
        worker.Prepare(shortStart, shortEnd, radius);

        worker.TrySweep(cylinder, out Vector3d shortCenter, out Fixed64 shortDistance).Should().BeFalse();
        shortCenter.Should().Be(Vector3d.Zero);
        shortDistance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithCylinderSideRootsBehindSweep_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Vector3d start = new(Fixed64.Half, Fixed64.Zero, Fixed64.Half);
        Vector3d end = new(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.FromFraction(7, 10));
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithCylinderSideMissInsideCandidateBounds_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Vector3d start = new(-Fixed64.One, Fixed64.Zero, Fixed64.FromFraction(1, 10));
        Vector3d end = new(-Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.One);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SweptSphereWorker_WithCylinderSideRootOutsideHeight_ShouldReturnFalse(int verticalSign)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Fixed64 sign = (Fixed64)verticalSign;
        Vector3d start = new(-Fixed64.Half, sign * Fixed64.FromFraction(11, 20), -Fixed64.Half);
        Vector3d end = new(Fixed64.Half, sign * Fixed64.FromFraction(31, 20), Fixed64.Half);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithCylinderCapBehindSweep_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Vector3d start = new(Fixed64.Half, Fixed64.Zero, Fixed64.Half);
        Vector3d end = new(Fixed64.Half, Fixed64.One, Fixed64.Half);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithCylinderCapIntersectionOutsideRadius_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();

        Vector3d start = new(Fixed64.FromFraction(9, 20), (Fixed64)2, Fixed64.FromFraction(9, 20));
        Vector3d end = new(Fixed64.FromFraction(9, 20), (Fixed64)(-2), Fixed64.FromFraction(9, 20));
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cylinder);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_ShouldDetectConeSideImpactWithStableSurfaceDelta()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        var worker = new SweptSphereQueryWorker();
        Vector3d origin = new((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
        worker.Prepare(origin, origin + Vector3d.Right * (Fixed64)4, Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().BeInRange(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(7, 4));
        centerAtImpact.X.Should().BeInRange(Fixed64.FromFraction(29, 4), Fixed64.FromFraction(31, 4));
        centerAtImpact.Y.Should().Be(Fixed64.Zero);
        centerAtImpact.Z.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithConeStartingOverlap_ShouldReturnZeroDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(
            Vector3d.Zero,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be(Fixed64.Zero);
        centerAtImpact.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithConeSeparatedAndMovingAway_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithConeNearSurfaceMovingAwayInsideSweptBounds_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        Vector3d start = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(9, 10), Fixed64.Zero);
        Vector3d end = start + Vector3d.Right * Fixed64.Half;
        Fixed64 radius = Fixed64.FromFraction(1, 4);
        AssertSweptSphereBroadOverlap(start, end, radius, cone);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithConeNearSurfaceShortSweep_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSConeCollider cone = CreateDynamicCollider(
            context,
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        Vector3d start = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(9, 10), Fixed64.Zero);
        Vector3d end = start + Vector3d.Right * Fixed64.FromFraction(1, 2048);
        Fixed64 radius = Fixed64.FromFraction(1, 4);
        AssertSweptSphereBroadOverlap(start, end, radius, cone);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithRotatedCuboidBroadOverlapButLocalYSeparation_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);
        LSCuboidCollider cuboid = CreateDynamicCollider(context, new LSCuboidCollider(), Vector3d.Zero, rotation);
        var worker = new SweptSphereQueryWorker();
        Vector3d localStart = new(Fixed64.Zero, Fixed64.FromFraction(7, 10), Fixed64.Zero);
        Vector3d localEnd = new(Fixed64.One, Fixed64.FromFraction(7, 10), Fixed64.Zero);
        Vector3d start = rotation * localStart;
        Vector3d end = rotation * localEnd;
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        AssertSweptSphereBroadOverlap(start, end, radius, cuboid);
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(cuboid, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithZeroLengthSegment_ShouldReturnFalse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider sphere = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(Vector3d.Zero, Vector3d.Zero, Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(sphere, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweptSphereWorker_WithUnsupportedCollider_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(-Vector3d.Right * (Fixed64)2, Vector3d.Right * (Fixed64)2, Fixed64.Half);

        bool hit = worker.TrySweep(unsupported, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SweepSphere_ShouldSupportVerticalAndDiagonalSweeps()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider vertical = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider diagonal = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4));

        bool verticalHit = context.Query3D.SweepSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Down,
            (Fixed64)4,
            out Physics3DHit verticalHitInfo,
            IncludeLayerZero);
        bool diagonalHit = context.Query3D.SweepSphere(
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2),
            Fixed64.Half,
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
            (Fixed64)4,
            out Physics3DHit diagonalHitInfo,
            IncludeLayerZero);

        verticalHit.Should().BeTrue();
        verticalHitInfo.Collider.Should().BeSameAs(vertical);
        diagonalHit.Should().BeTrue();
        diagonalHitInfo.Collider.Should().BeSameAs(diagonal);
    }

    private static void AssertSweepHits(GravitasWorldContext context, Vector3d origin, LSCollider expected)
    {
        bool hit = context.Query3D.SweepSphere(
            origin,
            Fixed64.FromFraction(1, 4),
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(expected);
    }

    private static LSCollider CreateAllHitSweepSource(ColliderType sourceType) => sourceType switch
    {
        ColliderType.Capsule => new LSCapsuleCollider(),
        ColliderType.Cylinder => new LSCylinderCollider(),
        ColliderType.Cone => new LSConeCollider(),
        ColliderType.Compound => new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero)),
        _ => throw new ArgumentOutOfRangeException(nameof(sourceType), sourceType, null)
    };

    private static void AssertDistanceNear(Fixed64 actual, Fixed64 expected)
    {
        (actual - expected).Abs().Should().BeLessThan(QueryTolerance);
    }

    private static void AssertSweptSphereBroadOverlap(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        LSCollider collider)
    {
        SweepBoundsUtility.CreateSweptSphereBounds(
            start,
            end,
            radius,
            Fixed64.FromFraction(1, 4096),
            out Vector3d min,
            out Vector3d max);
        SweepBoundsUtility.OverlapsInclusive(min, max, collider.BoundsMin, collider.BoundsMax).Should().BeTrue();
    }

    private static void AssertSweptSphereExactMissReturnsDefault(
        LSCollider collider,
        Vector3d start,
        Vector3d end,
        Fixed64 radius)
    {
        AssertSweptSphereBroadOverlap(start, end, radius, collider);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(start, end, radius);

        bool hit = worker.TrySweep(collider, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeFalse();
        centerAtImpact.Should().Be(Vector3d.Zero);
        distance.Should().Be(Fixed64.Zero);
    }

    private static JointDefinition3D CreateBallSocket(SolidBody first, SolidBody second) =>
        new(
            first,
            second,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked);

    private static FixedTransform LocalFrame(Vector3d position) =>
        new(position, FixedQuaternion.Identity, Vector3d.One);

    private static TCollider CreateDynamicCollider<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null,
        bool immovable = false,
        bool isKinematic = false,
        bool isDynamic = true)
        where TCollider : LSCollider
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes3D.Position : BodyFreezeAxes3D.None,
            IsKinematic = isKinematic
        };

        body.Initialize(position, rotation ?? FixedQuaternion.Identity, isDynamic);
        return collider;
    }

    private static LSSphereCollider CreateBodylessCollider(GravitasWorldContext context, Vector3d position)
    {
        EnsureGrid(context);
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d((Fixed64)(-8), (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d((Fixed64)16, (Fixed64)8, (Fixed64)8));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
