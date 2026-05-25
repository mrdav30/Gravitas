using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Raycasting;
using SwiftCollections;
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
    public void GetNearbyTriangles_ShouldUseDominantBoundsAxis()
    {
        LSMeshCollider mesh = CreateZDominantMesh();
        var indices = new SwiftList<int>();

        mesh.GetNearbyTriangles(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero), indices);

        indices.Count.Should().Be(2);
        ContainsTriangleIndex(indices, 0).Should().BeTrue();
        ContainsTriangleIndex(indices, 1).Should().BeTrue();
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
