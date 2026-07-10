using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GravitasQuery3DServiceConeTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void OverlapCone_ShouldHitPrimitiveTargetsAndOrderAllHitsByAxialDistance()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)).Collider;
        LSCuboidCollider cuboid = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Half, Fixed64.Zero)).Collider;
        LSCapsuleCollider capsule = scenario.CreateCapsule(new Vector3d((Fixed64)6, Fixed64.One, Fixed64.Zero)).Collider;
        LSCylinderCollider cylinder = scenario.CreateCylinder(new Vector3d((Fixed64)8, Fixed64.One, Fixed64.Zero)).Collider;
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d((Fixed64)10, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        var hits = new SwiftList<Physics3DHit>();

        bool closest = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)12,
            (Fixed64)3,
            out Physics3DHit closestHit,
            IncludeLayerZero);
        int count = scenario.Context.Query3D.OverlapConeAll(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)12,
            (Fixed64)3,
            IncludeLayerZero,
            hits);

        closest.Should().BeTrue();
        closestHit.Collider.Should().BeSameAs(sphere);
        count.Should().Be(5);
        hits[0].Collider.Should().BeSameAs(sphere);
        hits[1].Collider.Should().BeSameAs(cuboid);
        hits[2].Collider.Should().BeSameAs(capsule);
        hits[3].Collider.Should().BeSameAs(cylinder);
        hits[4].Collider.Should().BeSameAs(cone);
        hits.Should().OnlyContain(hit => hit.Distance >= Fixed64.Zero);
    }

    [Fact]
    public void OverlapCone_WithConvexTargetInsideBoundsButOutsideVolume_ShouldRejectExactMiss()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateSphere(new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)3,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        coneHit.Collider.Should().BeNull();
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithUnsupportedTargetInsideBoundsButOutsideVolume_ShouldRejectConservativeMiss()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.InitializeStaticCollider(
            new UnsupportedTestCollider3D(),
            new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)3,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        coneHit.Collider.Should().BeNull();
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithCenteredUnsupportedTargetNearBase_ShouldUseOriginDirectedFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var target = new UnsupportedTestCollider3D();
        Vector3d center = new(Fixed64.FromFraction(15, 4), Fixed64.Zero, Fixed64.Zero);
        scenario.InitializeStaticCollider(target, center);

        bool firstHit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.FromFraction(1, 4),
            out Physics3DHit first,
            IncludeLayerZero);
        int firstCandidateCount = scenario.Context.Query3D.LastQueryCandidateCount;
        bool secondHit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.FromFraction(1, 4),
            out Physics3DHit second,
            IncludeLayerZero);

        firstHit.Should().BeTrue();
        secondHit.Should().BeTrue();
        first.Collider.Should().BeSameAs(target);
        first.Point.Should().Be(center);
        first.Normal.Should().Be(Vector3d.Up);
        first.Distance.Should().Be(Fixed64.FromFraction(15, 4));
        second.Collider.Should().BeSameAs(target);
        second.Point.Should().Be(first.Point);
        second.Normal.Should().Be(first.Normal);
        second.Distance.Should().Be(first.Distance);
        firstCandidateCount.Should().Be(1);
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithSupportedConvexIntersectionAndOutsideSurfaceProbes_ShouldUseClampedFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider target = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(43, 50) },
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;

        bool firstHit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit first,
            IncludeLayerZero);
        int firstCandidateCount = scenario.Context.Query3D.LastQueryCandidateCount;
        bool secondHit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit second,
            IncludeLayerZero);

        firstHit.Should().BeTrue();
        secondHit.Should().BeTrue();
        first.Collider.Should().BeSameAs(target);
        first.Distance.Should().BeInRange(Fixed64.Zero, (Fixed64)4);
        first.Point.Y.Should().BeGreaterThan(first.Distance / (Fixed64)4);
        first.Normal.X.Should().BeLessThan(Fixed64.Zero);
        first.Normal.Y.Should().BeLessThan(Fixed64.Zero);
        first.Normal.Z.Should().Be(Fixed64.Zero);
        second.Collider.Should().BeSameAs(target);
        second.Point.Should().Be(first.Point);
        second.Normal.Should().Be(first.Normal);
        second.Distance.Should().Be(first.Distance);
        firstCandidateCount.Should().Be(1);
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_ShouldSupportMeshCompoundFilteringAndValidation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        LSCompoundCollider compound = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Cone(Fixed64.Half, (Fixed64)2, Vector3d.Zero)),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
                    new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.One),
                    new Vector3d((Fixed64)6, (Fixed64)2, Fixed64.One)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        LSSphereCollider trigger = scenario.CreateStaticSphere(new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        trigger.IsTrigger = true;
        LSSphereCollider masked = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero)).Collider;
        masked.Layer = new PhysicsLayer(2);
        var hits = new SwiftList<Physics3DHit>();

        int count = scenario.Context.Query3D.OverlapConeAll(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)10,
            (Fixed64)2,
            IncludeLayerZero,
            hits);
        Action zeroDirection = () => scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.One,
            Fixed64.Half,
            out _,
            IncludeLayerZero);
        Action invalidLength = () => scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Half,
            out _,
            IncludeLayerZero);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, mesh));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, compound));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, concaveMesh));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, trigger));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, masked));
        zeroDirection.Should().Throw<ArgumentException>().WithParameterName("direction");
        invalidLength.Should().Throw<ArgumentException>().WithParameterName("length");
    }

    [Fact]
    public void OverlapCone_WithConcaveTriangleAcrossAxis_ShouldReportAxisIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
                    new Vector3d((Fixed64)2, -Fixed64.Half, Fixed64.FromFraction(866, 1000)),
                    new Vector3d((Fixed64)2, -Fixed64.Half, -Fixed64.FromFraction(866, 1000))
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Half,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().Be((Fixed64)2);
        coneHit.Point.X.Should().Be((Fixed64)2);
        coneHit.Point.Y.Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
        coneHit.Point.Z.Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithConcaveEqualDistanceTriangles_ShouldUseStableTriangleHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(1, 4), Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4)),
                    new Vector3d((Fixed64)2, Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)2, -Fixed64.FromFraction(1, 4), Fixed64.Zero),
                    new Vector3d((Fixed64)2, -Fixed64.FromFraction(1, 4), -Fixed64.FromFraction(1, 4)),
                    new Vector3d((Fixed64)2, -Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(3, 8), Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(3, 8), Fixed64.FromFraction(1, 8)),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(7, 16), Fixed64.Zero)
                },
                new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().Be((Fixed64)2);
        coneHit.Point.Should().Be(new Vector3d((Fixed64)2, Fixed64.FromFraction(1, 4), Fixed64.Zero));
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(3);
    }

    [Fact]
    public void OverlapCone_WithConcaveTriangleInsideBoundsButOutsideVolume_ShouldRejectWithoutHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.FromFraction(3, 2)),
                    new Vector3d((Fixed64)2, Fixed64.Half, Fixed64.FromFraction(3, 2)),
                    new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.FromFraction(7, 5))
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)2,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        coneHit.Collider.Should().BeNull();
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
        concaveMesh.Mode.Should().Be(MeshColliderMode.Concave);
    }

    [Fact]
    public void OverlapCone_WithConcaveGeneratorParallelTriangleOutsideVolume_ShouldRejectWithoutHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
                    new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(3, 2), Fixed64.FromFraction(1, 10))
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)2,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        coneHit.Collider.Should().BeNull();
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
        concaveMesh.Mode.Should().Be(MeshColliderMode.Concave);
    }

    [Fact]
    public void OverlapCone_WithConcaveLaterCloserTriangle_ShouldReplaceFartherHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)3, Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)3, -Fixed64.Half, Fixed64.Half),
                    new Vector3d((Fixed64)3, -Fixed64.Half, -Fixed64.Half),
                    new Vector3d((Fixed64)2, Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)2, -Fixed64.Half, Fixed64.Half),
                    new Vector3d((Fixed64)2, -Fixed64.Half, -Fixed64.Half)
                },
                new[] { 0, 1, 2, 3, 4, 5 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().Be((Fixed64)2);
        coneHit.Point.X.Should().Be((Fixed64)2);
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(2);
    }

    [Fact]
    public void OverlapCone_WithConcaveLaterFartherTriangle_ShouldRetainNearerHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)2, Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)2, -Fixed64.Half, Fixed64.Half),
                    new Vector3d((Fixed64)2, -Fixed64.Half, -Fixed64.Half),
                    new Vector3d((Fixed64)3, Fixed64.Half, Fixed64.Zero),
                    new Vector3d((Fixed64)3, -Fixed64.Half, Fixed64.Half),
                    new Vector3d((Fixed64)3, -Fixed64.Half, -Fixed64.Half)
                },
                new[] { 0, 1, 2, 3, 4, 5 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().Be((Fixed64)2);
        coneHit.Point.X.Should().Be((Fixed64)2);
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(2);
    }

    [Fact]
    public void OverlapCone_WithConcaveHomogeneousGeneratorEdge_ShouldReportApexContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    Vector3d.Zero,
                    new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.One, Fixed64.FromFraction(1, 10))
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)2,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().Be(Fixed64.Zero);
        coneHit.Point.Should().Be(Vector3d.Zero);
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithCompoundMissThenNearAndFarHits_ShouldKeepNearHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCompoundCollider compound = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)2,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(compound);
        coneHit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        coneHit.Point.X.Should().Be(Fixed64.FromFraction(3, 2));
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithCompoundOnlyExactMissParts_ShouldReturnNoHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)3, (Fixed64)3, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            (Fixed64)2,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeFalse();
        coneHit.Should().Be(default(Physics3DHit));
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCone_WithConcaveTriangleSpanningAxialLimits_ShouldClipOutOfRangeVertices()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d((Fixed64)2, Fixed64.FromFraction(1, 4), Fixed64.Zero)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;

        bool hit = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out Physics3DHit coneHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        coneHit.Collider.Should().BeSameAs(concaveMesh);
        coneHit.Distance.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        coneHit.Distance.Should().BeLessThan(Fixed64.FromFraction(1, 1000));
        scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().Be(1);
    }

    [Fact]
    public void OverlapConeAll_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        _ = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Half, Fixed64.Zero));
        var hits = new SwiftList<Physics3DHit>(8);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
            scenario.Context.Query3D.OverlapConeAll(
                Vector3d.Zero,
                Vector3d.Right,
                (Fixed64)6,
                (Fixed64)2,
                IncludeLayerZero,
                hits));

        allocatedBytes.Should().Be(0);
    }
}
