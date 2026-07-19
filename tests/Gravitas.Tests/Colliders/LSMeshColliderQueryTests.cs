using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSMeshColliderQueryTests
{
    [Fact]
    public void ColliderOverlapsRay_ShouldReturnTriangleIntersectionPoint()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), (Fixed64)(-2)),
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), (Fixed64)2));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero));
    }

    [Fact]
    public void ColliderOverlapsRay_WithNonUniformScale_ShouldUseScaledTriangleAndCachedBvh()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = CreateTriangleMesh();
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4))));
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d expected = new(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero);
        worker.PrepareSegmentCheck(expected - Vector3d.Forward, expected + Vector3d.Forward);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(expected);
        mesh.Mesh.TriangleBvhBuildCount.Should().Be(1);
    }

    [Fact]
    public void ColliderOverlapsRay_ShouldRejectBoundingBoxHitOutsideTriangle()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4), (Fixed64)(-2)),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4), (Fixed64)2));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithPointOnTriangle_ShouldReturnPoint()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero);

        worker.PrepareSegmentCheck(point, point);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(point);
    }

    [Fact]
    public void ColliderOverlapsRay_WithPointOffTrianglePlane_ShouldReturnFalse()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.One);

        worker.PrepareSegmentCheck(point, point);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithPointOffSlantedTrianglePlaneInsideBounds_ShouldReturnFalse()
    {
        LSMeshCollider mesh = CreateSlantedTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter);

        worker.PrepareSegmentCheck(point, point);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithPointInTrianglePlaneOutsideTriangle_ShouldReturnFalse()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4), Fixed64.Zero);

        worker.PrepareSegmentCheck(point, point);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithCoplanarSegmentOnTriangleEdge_ShouldReturnUniqueEndpoints()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Right);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[0].Should().Be(Vector3d.Zero);
        hits[1].Should().Be(Vector3d.Right);
    }

    [Fact]
    public void ColliderOverlapsRay_WithIntersectionsDisabled_ShouldReturnTrueWithoutPoints()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), (Fixed64)(-2)),
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), (Fixed64)2),
            calculateIntersectionPoints: false);

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithSegmentOnSharedTriangleEdge_ShouldSuppressDuplicatePoint()
    {
        LSMeshCollider mesh = CreateQuadMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[0].Should().Be(Vector3d.Zero);
        hits[1].Should().Be(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
    }

    [Fact]
    public void ColliderOverlapsRay_WithCoplanarSegmentOutsideTriangle_ShouldReturnFalse()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Half, Fixed64.Zero));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void ColliderOverlapsRay_WithParallelSegmentOffSlantedTrianglePlane_ShouldReturnFalse()
    {
        LSMeshCollider mesh = CreateSlantedTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.FromFraction(3, 20), Fixed64.FromFraction(7, 20), Fixed64.FromFraction(13, 20)),
            new Vector3d(Fixed64.FromFraction(7, 20), Fixed64.FromFraction(3, 20), Fixed64.FromFraction(13, 20)));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void GetNormalAtPoint_ShouldUseClosestTriangleNormal()
    {
        LSMeshCollider mesh = CreateTriangleMesh();

        Vector3d normal = mesh.GetNormalAtPoint(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.One));

        normal.Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void SurfaceQueries_WhenBvhNeighborhoodIsEmpty_ShouldScanAuthoredTriangles()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = CreateDisconnectedCornerMesh();
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Vector3d queryPoint = new((Fixed64)20, (Fixed64)(-20), (Fixed64)20);
        Vector3d expectedClosest = new((Fixed64)10, (Fixed64)9, (Fixed64)9);

        mesh.ClosestPointOnSurface(queryPoint).Should().Be(expectedClosest);
        mesh.GetNormalAtPoint(queryPoint).Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void SurfaceQueries_WhenAllRoundedDistancesSaturate_ShouldKeepTheExactlyClosestTriangle()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-9), (Fixed64)(-9), (Fixed64)(-9)),
                new Vector3d((Fixed64)(-10), (Fixed64)(-9), (Fixed64)(-9)),
                new Vector3d((Fixed64)(-9), (Fixed64)(-10), (Fixed64)(-9)),
                new Vector3d((Fixed64)9, (Fixed64)9, (Fixed64)9),
                new Vector3d((Fixed64)10, (Fixed64)9, (Fixed64)9),
                new Vector3d((Fixed64)9, (Fixed64)10, (Fixed64)9)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Vector3d queryPoint = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);

        Vector3d closest = mesh.ClosestPointOnSurface(queryPoint);

        closest.X.Should().BeGreaterThan(Fixed64.Zero);
        closest.Z.Should().Be((Fixed64)9);
    }

    [Fact]
    public void ClosestPointOnSurface_WhenInitialBvhNeighborhoodOmitsCloserTriangle_ShouldExpandFromExactUpperBound()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)4, (Fixed64)(-5), Fixed64.Zero),
                new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)(-5)),
                new Vector3d((Fixed64)10, (Fixed64)5, (Fixed64)5),
                new Vector3d(Fixed64.FromFraction(99, 10), (Fixed64)5, (Fixed64)5),
                new Vector3d((Fixed64)10, Fixed64.FromFraction(49, 10), (Fixed64)5)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));

        Vector3d closest = mesh.ClosestPointOnSurface(
            new Vector3d((Fixed64)11, Fixed64.Zero, Fixed64.Zero));

        closest.Should().Be(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void ClosestPointOnSurface_WhenUpperBoundSaturates_ShouldUseFullAuthoredScan()
    {
        Fixed64 offset = new(1_500_000_000);
        Vector3d nearby = new(offset, offset, Fixed64.Zero);
        var mesh = new LSMeshCollider(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                nearby,
                nearby + Vector3d.Right,
                nearby + Vector3d.Up
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.RequireClosedVolume);

        Vector3d closest = mesh.ClosestPointOnSurface(nearby + Vector3d.Forward);

        closest.Should().Be(nearby);
    }

    [Fact]
    public void ClosestPointOnSurface_WhenSearchDeltaContainsMinValue_ShouldUseFullAuthoredScan()
    {
        var mesh = new LSMeshCollider(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

        Vector3d closest = mesh.ClosestPointOnSurface(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));
        Vector3d closestFromMinY = mesh.ClosestPointOnSurface(
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero));

        closest.Should().Be(Vector3d.Zero);
        closestFromMinY.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SurfaceQueries_WhenFirstTriangleIsWithinEpsilon_ShouldContinueToLaterExactHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 nearOffset = -Fixed64.Epsilon;
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d(-Fixed64.One, -Fixed64.One, nearOffset),
                new Vector3d(Fixed64.One, -Fixed64.One, nearOffset),
                new Vector3d(Fixed64.Zero, Fixed64.One, nearOffset),
                new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));

        Vector3d closest = mesh.ClosestPointOnSurface(Vector3d.Zero);

        closest.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void GetNormalAtPoint_WithMultipleExactTriangles_ShouldUseLowestAuthoredTriangleIndex()
    {
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d(-Fixed64.One, -Fixed64.One, (Fixed64)10),
                new Vector3d(Fixed64.One, -Fixed64.One, (Fixed64)10),
                new Vector3d(Fixed64.Zero, Fixed64.One, (Fixed64)10),
                new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            },
            new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.RequireClosedVolume);

        Vector3d normal = mesh.GetNormalAtPoint(Vector3d.Zero);

        normal.Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void ClosestPointOnSurface_WithPointOnTriangle_ShouldReturnExactPoint()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        Vector3d point = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);

        Vector3d closest = mesh.ClosestPointOnSurface(point);

        closest.Should().Be(point);
    }

    [Fact]
    public void TryGetPlanarSurfaceNormal_ShouldReturnAuthoredTriangleNormal()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        Vector3d point = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.One);

        bool found = mesh.TryGetPlanarSurfaceNormal(point, out Vector3d normal);

        found.Should().BeTrue();
        normal.Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void ClosestPointOnSurface_ShouldQueryMovedMeshInLocalBVHSpace()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var body = scenario.CreateBody(
            CreateTriangleMesh(),
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        Vector3d closest = body.Collider.ClosestPointOnSurface(
            new Vector3d((Fixed64)6 + Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4), (Fixed64)2));

        closest.Should().Be(new Vector3d((Fixed64)6 + Fixed64.Half, Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void GetNearbyTriangles_ShouldUseDominantBoundsAxis()
    {
        LSMeshCollider mesh = CreateZDominantMesh();
        var indices = new SwiftList<int>();

        mesh.GetNearbyTriangles(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero), indices);

        indices.Count.Should().Be(2);
        ContainsTriangleIndex(indices, 0).Should().BeTrue();
        ContainsTriangleIndex(indices, 1).Should().BeTrue();
    }

    [Fact]
    public void GetTrianglesInBounds_ShouldQueryMovedMeshWithoutRebuildingTriangleBVH()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var body = scenario.CreateBody(
            CreateTriangleMesh(),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        var indices = new SwiftList<int>();

        body.Collider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)6, Fixed64.One, Fixed64.Zero)),
            indices);
        int buildCount = body.Collider.Mesh.TriangleBvhBuildCount;

        body.Body.SetPosition(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        body.Collider.Simulate();
        body.Collider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)9, Fixed64.One, Fixed64.Zero)),
            indices);

        indices.Count.Should().Be(1);
        indices[0].Should().Be(0);
        body.Collider.Mesh.TriangleBvhBuildCount.Should().Be(buildCount);
    }

    private static LSMeshCollider CreateTriangleMesh() =>
        new(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider CreateQuadMesh() =>
        new(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
                Vector3d.Up
            },
            new[] { 0, 1, 2, 0, 2, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider CreateSlantedTriangleMesh() =>
        new(
            new[]
            {
                Vector3d.Zero,
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One)
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider CreateZDominantMesh() =>
        new(
            new[]
            {
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-4)),
                new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)(-4)),
                new Vector3d(Fixed64.Zero, Fixed64.One, (Fixed64)(-4)),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4),
                new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)4),
                new Vector3d(Fixed64.Zero, Fixed64.One, (Fixed64)4)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider CreateDisconnectedCornerMesh() =>
        new(
            new[]
            {
                new Vector3d((Fixed64)9, (Fixed64)9, (Fixed64)9),
                new Vector3d((Fixed64)10, (Fixed64)9, (Fixed64)9),
                new Vector3d((Fixed64)9, (Fixed64)10, (Fixed64)9),
                new Vector3d((Fixed64)(-9), (Fixed64)(-9), (Fixed64)(-9)),
                new Vector3d((Fixed64)(-10), (Fixed64)(-9), (Fixed64)(-9)),
                new Vector3d((Fixed64)(-9), (Fixed64)(-10), (Fixed64)(-9))
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

    private static bool ContainsTriangleIndex(SwiftList<int> indices, int triangleIndex)
    {
        for (int i = 0; i < indices.Count; i++)
        {
            if (indices[i] == triangleIndex)
                return true;
        }

        return false;
    }
}
