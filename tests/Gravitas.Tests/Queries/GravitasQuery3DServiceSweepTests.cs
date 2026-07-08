using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GravitasQuery3DServiceSweepTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);
    private static readonly Fixed64 QueryTolerance = Fixed64.FromFraction(1, 1_000_000);

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
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(1, 4))
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
        AssertDistanceNear(sweepHit.Distance, Fixed64.FromFraction(7, 2));
        sweepHit.Point.Z.Should().BeLessThan(Fixed64.Zero);
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
        worker.Prepare(
            start,
            start + Vector3d.Right * Fixed64.Half,
            Fixed64.FromFraction(1, 4));

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
        worker.Prepare(
            start,
            start - Vector3d.Right * Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 4));

        bool hit = worker.TrySweep(cone, out Vector3d centerAtImpact, out Fixed64 distance);

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

    private static void AssertDistanceNear(Fixed64 actual, Fixed64 expected)
    {
        (actual - expected).Abs().Should().BeLessThan(QueryTolerance);
    }

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
