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

public sealed class GravitasQuery3DBatchTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void RaycastBatch_ShouldRequireOneOutputSlotPerRequest()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(-2, 0, 0), Vector(2, 0, 0), IncludeLayerZero)
        };
        Physics3DHit[] closestHits = Array.Empty<Physics3DHit>();

        Action act = () => scenario.Context.Query3D.RaycastBatch(requests, closestHits);

        act.Should().Throw<ArgumentException>()
            .WithParameterName("closestHits");
    }

    [Fact]
    public void RaycastAllBatch_ShouldClearAndFillCallerOwnedBuffers()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider near = scenario.CreateSphere(Vector3d.Zero).Collider;
        LSSphereCollider far = scenario.CreateSphere(Vector3d.Right * 2).Collider;
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero),
            new(Vector3d.Zero, Vector3d.Zero, IncludeLayerZero),
            new(Vector(-2, 2, 0), Vector(4, 2, 0), IncludeLayerZero)
        };
        var hits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[requests.Length];
        ranges[0] = new PhysicsQueryHitRange(12, 34);
        hits.Add(default);

        int count = scenario.Context.Query3D.RaycastAllBatch(requests, hits, ranges);

        count.Should().Be(2);
        hits.Count.Should().Be(2);
        ranges[0].Start.Should().Be(0);
        ranges[0].Count.Should().Be(2);
        ranges[1].Start.Should().Be(2);
        ranges[1].Count.Should().Be(0);
        ranges[2].Start.Should().Be(2);
        ranges[2].Count.Should().Be(0);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
    }

    [Fact]
    public void RaycastBatch_ShouldPreserveRequestOrderAndReportZeroLengthAsMiss()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider first = scenario.CreateSphere(Vector3d.Zero).Collider;
        LSSphereCollider second = scenario.CreateSphere(Vector3d.Right * 4).Collider;
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(4, -2, 0), Vector(4, 2, 0), IncludeLayerZero),
            new(Vector3d.Zero, Vector3d.Zero, IncludeLayerZero),
            new(Vector(0, -2, 0), Vector(0, 2, 0), IncludeLayerZero)
        };
        Physics3DHit[] closestHits = new Physics3DHit[requests.Length];

        int count = scenario.Context.Query3D.RaycastBatch(requests, closestHits);

        count.Should().Be(2);
        closestHits[0].Collider.Should().BeSameAs(second);
        closestHits[1].Collider.Should().BeNull();
        closestHits[2].Collider.Should().BeSameAs(first);
    }

    [Fact]
    public void RaycastAllBatch_ShouldPreservePerRequestHitOrdering()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider near = scenario.CreateSphere(Vector3d.Zero).Collider;
        LSSphereCollider far = scenario.CreateSphere(Vector3d.Right * 2).Collider;
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero),
            new(Vector(4, 0, 0), Vector(-2, 0, 0), IncludeLayerZero)
        };
        var hits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[requests.Length];

        int count = scenario.Context.Query3D.RaycastAllBatch(requests, hits, ranges);

        count.Should().Be(4);
        ranges[0].Start.Should().Be(0);
        ranges[0].Count.Should().Be(2);
        ranges[1].Start.Should().Be(2);
        ranges[1].Count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        hits[2].Collider.Should().BeSameAs(far);
        hits[3].Collider.Should().BeSameAs(near);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
        hits[2].Distance.Should().BeLessThan(hits[3].Distance);
    }

    [Fact]
    public void RaycastBatch_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateSphere(Vector3d.Zero);
        _ = scenario.CreateSphere(Vector3d.Right * 2);
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero),
            new(Vector(-2, 2, 0), Vector(4, 2, 0), IncludeLayerZero)
        };
        Physics3DHit[] closestHits = new Physics3DHit[requests.Length];
        var hits = new SwiftList<Physics3DHit>(8);
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[requests.Length];

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            scenario.Context.Query3D.RaycastBatch(requests, closestHits);
            scenario.Context.Query3D.RaycastAllBatch(requests, hits, ranges);
        });

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void RaycastAllBatch_WithDiagnosticsEnabled_ShouldKeepHitOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider near = scenario.CreateSphere(Vector3d.Zero).Collider;
        LSSphereCollider far = scenario.CreateSphere(Vector3d.Right * 2).Collider;
        scenario.Context.Diagnostics.Enable(eventCapacity: 8, drawCommandCapacity: 0);
        PhysicsRaycast3DRequest[] requests =
        {
            new(Vector(-2, 0, 0), Vector(4, 0, 0), IncludeLayerZero)
        };
        var hits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[requests.Length];

        int count = scenario.Context.Query3D.RaycastAllBatch(requests, hits, ranges);

        count.Should().Be(2);
        ranges[0].Count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        scenario.Context.Query3D.LastBatchRequestCount.Should().Be(1);
        scenario.Context.Query3D.LastBatchHitCount.Should().Be(2);
    }

    [Fact]
    public void SweepSphereAndOverlapCircleBatches_ShouldMatchSingleQueryResults()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider first = scenario.CreateSphere(Vector3d.Zero).Collider;
        LSSphereCollider second = scenario.CreateSphere(Vector3d.Right * 4).Collider;
        PhysicsSweepSphere3DRequest[] sweepRequests =
        {
            new(Vector(-4, 0, 0), Vector(2, 0, 0), Fixed64.Half, IncludeLayerZero),
            new(Vector(4, -2, 0), Vector(4, 2, 0), Fixed64.Half, IncludeLayerZero),
            new(Vector(-4, -4, 0), Vector(4, -4, 0), Fixed64.Zero, IncludeLayerZero),
            new(Vector3d.Zero, Vector3d.Zero, Fixed64.Half, IncludeLayerZero)
        };
        PhysicsOverlapCircle3DRequest[] overlapRequests =
        {
            new(Vector3d.Zero, (Fixed64)2, IncludeLayerZero),
            new(Vector3d.Right * 4, (Fixed64)2, IncludeLayerZero),
            new(Vector(0, 0, 8), Fixed64.Half, IncludeLayerZero)
        };
        Physics3DHit[] closestSweeps = new Physics3DHit[sweepRequests.Length];
        Physics3DHit[] closestOverlaps = new Physics3DHit[overlapRequests.Length];
        var sweepHits = new SwiftList<Physics3DHit>();
        var overlapHits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] sweepRanges = new PhysicsQueryHitRange[sweepRequests.Length];
        PhysicsQueryHitRange[] overlapRanges = new PhysicsQueryHitRange[overlapRequests.Length];

        int sweepClosestCount = scenario.Context.Query3D.SweepSphereBatch(sweepRequests, closestSweeps);
        int sweepAllCount = scenario.Context.Query3D.SweepSphereAllBatch(sweepRequests, sweepHits, sweepRanges);
        int overlapClosestCount = scenario.Context.Query3D.OverlapCircleBatch(overlapRequests, closestOverlaps);
        int overlapAllCount = scenario.Context.Query3D.OverlapCircleAllBatch(overlapRequests, overlapHits, overlapRanges);

        sweepClosestCount.Should().Be(2);
        sweepAllCount.Should().Be(2);
        closestSweeps[0].Collider.Should().BeSameAs(first);
        closestSweeps[1].Collider.Should().BeSameAs(second);
        closestSweeps[2].Collider.Should().BeNull();
        closestSweeps[3].Collider.Should().BeNull();
        sweepHits[sweepRanges[0].Start].Collider.Should().BeSameAs(first);
        sweepHits[sweepRanges[1].Start].Collider.Should().BeSameAs(second);
        sweepRanges[2].Count.Should().Be(0);
        sweepRanges[3].Count.Should().Be(0);
        overlapClosestCount.Should().Be(2);
        overlapAllCount.Should().Be(2);
        closestOverlaps[0].Collider.Should().BeSameAs(first);
        closestOverlaps[1].Collider.Should().BeSameAs(second);
        closestOverlaps[2].Should().Be(default(Physics3DHit));
        overlapHits[overlapRanges[0].Start].Collider.Should().BeSameAs(first);
        overlapHits[overlapRanges[1].Start].Collider.Should().BeSameAs(second);
        overlapRanges[2].Start.Should().Be(2);
        overlapRanges[2].Count.Should().Be(0);
    }

    [Fact]
    public void RegisteredSourceSweepBatch_ShouldReuseExactSourceSweepBehavior()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(Vector(-4, 0, 0));
        LSSphereCollider target = scenario.CreateSphere(Vector3d.Zero, immovable: true).Collider;
        PhysicsSweepCuboid3DRequest[] requests =
        {
            new(source.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(source.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(source.Collider, Vector(2, 0, 0), IncludeLayerZero)
        };
        Physics3DHit[] closestHits = new Physics3DHit[requests.Length];
        var hits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[requests.Length];

        int closestCount = scenario.Context.Query3D.SweepCuboidBatch(requests, closestHits);
        int allCount = scenario.Context.Query3D.SweepCuboidAllBatch(requests, hits, ranges);

        closestCount.Should().Be(2);
        closestHits[0].Collider.Should().BeSameAs(target);
        closestHits[1].Should().Be(closestHits[0]);
        closestHits[2].Should().Be(default(Physics3DHit));
        allCount.Should().Be(2);
        ranges[0].Count.Should().Be(1);
        ranges[1].Start.Should().Be(1);
        ranges[1].Count.Should().Be(1);
        ranges[2].Start.Should().Be(2);
        ranges[2].Count.Should().Be(0);
        hits[ranges[0].Start].Collider.Should().BeSameAs(target);
        hits[ranges[1].Start].Should().Be(hits[ranges[0].Start]);
    }

    [Fact]
    public void RegisteredSourceSweepBatches_ShouldRouteCapsuleCylinderMeshAndCompoundSources()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsuleSource = scenario.CreateCapsule(Vector(-4, -9, 0));
        LSSphereCollider capsuleTarget = scenario.CreateSphere(Vector(0, -9, 0), immovable: true).Collider;
        ScenarioBody<LSCylinderCollider> cylinderSource = scenario.CreateCylinder(Vector(-4, -3, 0));
        LSSphereCollider cylinderTarget = scenario.CreateSphere(Vector(0, -3, 0), immovable: true).Collider;
        ScenarioBody<LSMeshCollider> meshSource = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector(-4, 3, 0),
            FixedQuaternion.Identity);
        LSSphereCollider meshTarget = scenario.CreateSphere(Vector(0, 3, 0), immovable: true).Collider;
        ScenarioBody<LSCompoundCollider> compoundSource = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero)),
            Vector(-4, 9, 0),
            FixedQuaternion.Identity);
        LSSphereCollider compoundTarget = scenario.CreateSphere(Vector(0, 9, 0), immovable: true).Collider;
        var hits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[2];

        PhysicsSweepCapsule3DRequest[] capsuleRequests =
        {
            new(capsuleSource.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(capsuleSource.Collider, Vector3d.Zero, IncludeLayerZero)
        };
        Physics3DHit[] capsuleClosest = new Physics3DHit[capsuleRequests.Length];
        int capsuleClosestCount = scenario.Context.Query3D.SweepCapsuleBatch(capsuleRequests, capsuleClosest);
        int capsuleAllCount = scenario.Context.Query3D.SweepCapsuleAllBatch(capsuleRequests, hits, ranges);
        AssertSingleHitBatch(capsuleClosestCount, capsuleClosest, capsuleAllCount, hits, ranges, capsuleTarget);

        PhysicsSweepCylinder3DRequest[] cylinderRequests =
        {
            new(cylinderSource.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(cylinderSource.Collider, Vector3d.Zero, IncludeLayerZero)
        };
        Physics3DHit[] cylinderClosest = new Physics3DHit[cylinderRequests.Length];
        int cylinderClosestCount = scenario.Context.Query3D.SweepCylinderBatch(cylinderRequests, cylinderClosest);
        int cylinderAllCount = scenario.Context.Query3D.SweepCylinderAllBatch(cylinderRequests, hits, ranges);
        AssertSingleHitBatch(cylinderClosestCount, cylinderClosest, cylinderAllCount, hits, ranges, cylinderTarget);

        PhysicsSweepConvexMesh3DRequest[] meshRequests =
        {
            new(meshSource.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(meshSource.Collider, Vector3d.Zero, IncludeLayerZero)
        };
        Physics3DHit[] meshClosest = new Physics3DHit[meshRequests.Length];
        int meshClosestCount = scenario.Context.Query3D.SweepConvexMeshBatch(meshRequests, meshClosest);
        int meshAllCount = scenario.Context.Query3D.SweepConvexMeshAllBatch(meshRequests, hits, ranges);
        AssertSingleHitBatch(meshClosestCount, meshClosest, meshAllCount, hits, ranges, meshTarget);

        PhysicsSweepCompound3DRequest[] compoundRequests =
        {
            new(compoundSource.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(compoundSource.Collider, Vector3d.Zero, IncludeLayerZero)
        };
        Physics3DHit[] compoundClosest = new Physics3DHit[compoundRequests.Length];
        int compoundClosestCount = scenario.Context.Query3D.SweepCompoundBatch(compoundRequests, compoundClosest);
        int compoundAllCount = scenario.Context.Query3D.SweepCompoundAllBatch(compoundRequests, hits, ranges);
        AssertSingleHitBatch(
            compoundClosestCount,
            compoundClosest,
            compoundAllCount,
            hits,
            ranges,
            compoundTarget);
    }

    [Fact]
    public void ConeVolumeAndConeSourceBatches_ShouldMatchSingleQueryResults()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider target = scenario.CreateSphere(Vector(2, 0, 0)).Collider;
        ScenarioBody<LSConeCollider> source = scenario.CreateBody(
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            Vector(-4, 0, 0),
            FixedQuaternion.Identity);
        PhysicsOverlapCone3DRequest[] coneRequests =
        {
            new(Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.One, IncludeLayerZero),
            new(Vector3d.Zero, Vector3d.Up, (Fixed64)4, Fixed64.One, IncludeLayerZero)
        };
        PhysicsSweepCone3DRequest[] sweepRequests =
        {
            new(source.Collider, Vector(6, 0, 0), IncludeLayerZero),
            new(source.Collider, Vector3d.Zero, IncludeLayerZero)
        };
        Physics3DHit[] closestConeHits = new Physics3DHit[coneRequests.Length];
        Physics3DHit[] closestSweepHits = new Physics3DHit[sweepRequests.Length];
        var coneHits = new SwiftList<Physics3DHit>();
        var sweepHits = new SwiftList<Physics3DHit>();
        PhysicsQueryHitRange[] coneRanges = new PhysicsQueryHitRange[coneRequests.Length];
        PhysicsQueryHitRange[] sweepRanges = new PhysicsQueryHitRange[sweepRequests.Length];

        int coneClosestCount = scenario.Context.Query3D.OverlapConeBatch(coneRequests, closestConeHits);
        int coneAllCount = scenario.Context.Query3D.OverlapConeAllBatch(coneRequests, coneHits, coneRanges);
        int sweepClosestCount = scenario.Context.Query3D.SweepConeBatch(sweepRequests, closestSweepHits);
        int sweepAllCount = scenario.Context.Query3D.SweepConeAllBatch(sweepRequests, sweepHits, sweepRanges);

        coneClosestCount.Should().Be(1);
        coneAllCount.Should().Be(1);
        closestConeHits[0].Collider.Should().BeSameAs(target);
        closestConeHits[1].Collider.Should().BeNull();
        coneHits[coneRanges[0].Start].Collider.Should().BeSameAs(target);
        coneRanges[1].Count.Should().Be(0);
        sweepClosestCount.Should().Be(1);
        sweepAllCount.Should().Be(1);
        closestSweepHits[0].Collider.Should().BeSameAs(target);
        closestSweepHits[1].Collider.Should().BeNull();
        sweepHits[sweepRanges[0].Start].Collider.Should().BeSameAs(target);
        sweepRanges[1].Count.Should().Be(0);
    }

    [Fact]
    public void OverlapCircleInDirectionBatch_ShouldPreserveRequestOrderAndRejectOppositeDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider right = scenario.CreateSphere(Vector(2, 0, 0)).Collider;
        LSSphereCollider left = scenario.CreateSphere(Vector(-2, 0, 0)).Collider;
        PhysicsOverlapCircleInDirection3DRequest[] requests =
        {
            new(Vector3d.Zero, (Fixed64)3, Vector3d.Right, (Fixed64)3, IncludeLayerZero),
            new(Vector3d.Zero, (Fixed64)3, Vector3d.Up, (Fixed64)3, IncludeLayerZero),
            new(Vector3d.Zero, (Fixed64)3, -Vector3d.Right, (Fixed64)3, IncludeLayerZero),
            new(Vector3d.Zero, (Fixed64)3, Vector3d.Zero, (Fixed64)3, IncludeLayerZero)
        };
        Physics3DHit[] closestHits = new Physics3DHit[requests.Length];

        int count = scenario.Context.Query3D.OverlapCircleInDirectionBatch(requests, closestHits);

        count.Should().Be(2);
        closestHits[0].Collider.Should().BeSameAs(right);
        closestHits[1].Collider.Should().BeNull();
        closestHits[2].Collider.Should().BeSameAs(left);
        closestHits[3].Collider.Should().BeNull();
        scenario.Context.Query3D.LastBatchRequestCount.Should().Be(requests.Length);
        scenario.Context.Query3D.LastBatchHitCount.Should().Be(2);
    }

    private static void AssertSingleHitBatch(
        int closestCount,
        Physics3DHit[] closestHits,
        int allCount,
        SwiftList<Physics3DHit> hits,
        PhysicsQueryHitRange[] ranges,
        LSCollider target)
    {
        closestCount.Should().Be(1);
        closestHits[0].Collider.Should().BeSameAs(target);
        closestHits[1].Collider.Should().BeNull();
        allCount.Should().Be(1);
        hits.Count.Should().Be(1);
        ranges[0].Start.Should().Be(0);
        ranges[0].Count.Should().Be(1);
        ranges[1].Start.Should().Be(1);
        ranges[1].Count.Should().Be(0);
        hits[0].Collider.Should().BeSameAs(target);
    }

    private static Vector3d Vector(int x, int y, int z) => new((Fixed64)x, (Fixed64)y, (Fixed64)z);
}
