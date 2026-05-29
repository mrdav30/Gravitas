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
            new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Fraction(1, 4), (Fixed64)(-2)),
            new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Fraction(1, 4), (Fixed64)2));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Fraction(1, 4), Fixed64.Zero));
    }

    [Fact]
    public void ColliderOverlapsRay_ShouldRejectBoundingBoxHitOutsideTriangle()
    {
        LSMeshCollider mesh = CreateTriangleMesh();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Fraction(3, 4), (Fixed64)(-2)),
            new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Fraction(3, 4), (Fixed64)2));

        bool hit = mesh.ColliderOverlapsRay(worker, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void GetNormalAtPoint_ShouldUseClosestTriangleNormal()
    {
        LSMeshCollider mesh = CreateTriangleMesh();

        Vector3d normal = mesh.GetNormalAtPoint(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Fraction(1, 4), Fixed64.One));

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
            new Vector3d((Fixed64)6 + Fixed64.Fraction(3, 4), Fixed64.Fraction(3, 4), (Fixed64)2));

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
            new[] { 0, 1, 2 });

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
            new[] { 0, 1, 2, 3, 4, 5 });

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
