using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeSurfaceConnectivityTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleClips_KeepSeparateRegionsWithGeometricOrdering(bool reverseDiscovery)
    {
        PhysicsMesh mesh = Ring();
        var tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        var connectivity = new MeshConeSurfaceConnectivity();
        // The thin horizontal cone section crosses the ring's left/right
        // bars. Its transverse radius <1/2 excludes the top/bottom bars.
        int[] admitted = reverseDiscovery ? new[] { 7, 2, 6, 3 } : new[] { 2, 3, 6, 7 };
        connectivity.Build(mesh, 0, Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.Half, admitted);
        Assert.Equal(2, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(6));
        Assert.Equal(0, connectivity.GetRegionOrdinal(7));
        Assert.Equal(1, connectivity.GetRegionOrdinal(2));
        Assert.Equal(1, connectivity.GetRegionOrdinal(3));
        Assert.Equal(-1, connectivity.GetRegionOrdinal(0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ApexTangency_ConnectsSharedEdgeOrVertexSections(bool vertexTouch)
    {
        Vector3d[] vertices = vertexTouch
            ? new[] { Vector3d.Zero, new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) }
            : new[] { new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) };
        int[] triangles = vertexTouch ? new[] { 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1 } : new[] { 0, 1, 2, 0, 2, 3 };
        PhysicsMesh mesh = Create(vertices, triangles);
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh, 0, new Vector3d(0, -2, 0), FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2,
            vertexTouch ? new[] { 3, 1, 2, 0 } : new[] { 1, 0 });
        Assert.Equal(1, connectivity.RegionCount);
        for (int i = 0; i < mesh.TriangleCount; i++) Assert.Equal(0, connectivity.GetRegionOrdinal(i));
        connectivity.Build(mesh, 0, new Vector3d(0, -2, 0), FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, ReadOnlySpan<int>.Empty);
        Assert.Equal(0, connectivity.RegionCount);
        Assert.Equal(-1, connectivity.GetRegionOrdinal(0));
    }

    [Fact]
    public void DeclinedSurface_ConsumesOnlyItsOwnAdmittedTriangle()
    {
        Vector3d[] vertices = { new(-1, 0, -1), new(1, 0, -1), new(0, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 1, 2 });
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, new[] { 1, 0 });
        Assert.Equal(1, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(0));
        Assert.Equal(-1, connectivity.GetRegionOrdinal(1));
        connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, new[] { 1 });
        Assert.Equal(0, connectivity.RegionCount);
        Assert.Equal(-1, connectivity.GetRegionOrdinal(0));
    }

    [Fact]
    public void PreparedSurface_IgnoresAdmittedTrianglesFromAnotherPlane()
    {
        PhysicsMesh mesh = Create(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1),
            new Vector3d(-1, 1, -1), new Vector3d(1, 1, -1), new Vector3d(0, 1, 1)
        }, new[] { 0, 1, 2, 0, 2, 3, 4, 5, 6 });
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, new[] { 2, 1, 0 });
        Assert.Equal(1, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(0));
        Assert.Equal(0, connectivity.GetRegionOrdinal(1));
        Assert.Equal(-1, connectivity.GetRegionOrdinal(2));
        connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, new[] { 2 });
        Assert.Equal(0, connectivity.RegionCount);
    }

    [Fact]
    public void RepeatedRegionBuilds_ReuseRetainedCapacityWithoutAllocations()
    {
        PhysicsMesh mesh = Ring();
        var tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        var connectivity = new MeshConeSurfaceConnectivity();
        int[] admitted = { 7, 2, 6, 3 };
        int[] wholeRing = { 7, 6, 5, 4, 3, 2, 1, 0 };
        for (int i = 0; i < 8; i++)
        {
            connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)10, wholeRing);
            Assert.Equal(1, connectivity.RegionCount);
            connectivity.Build(mesh, 0, Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.Half, admitted);
            Assert.Equal(2, connectivity.RegionCount);
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 32; i++)
        {
            connectivity.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)10, wholeRing);
            connectivity.Build(mesh, 0, Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.Half, admitted);
        }
        long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, bytes);
        Assert.Equal(2, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(6));
        Assert.Equal(1, connectivity.GetRegionOrdinal(2));
    }

    [Fact]
    public void IndependentOwners_RetainSeparateConnectivity()
    {
        var connectivityA = new MeshConeSurfaceConnectivity();
        var connectivityB = new MeshConeSurfaceConnectivity();
        PhysicsMesh mesh = Ring();
        var tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        connectivityA.Build(mesh, 0, Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.Half, new[] { 2, 3, 6, 7 });
        Assert.Equal(2, connectivityA.RegionCount);
        Assert.Equal(0, connectivityB.RegionCount);
        connectivityB.Build(mesh, 0, Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.Half, new[] { 2, 3 });
        Assert.Equal(1, connectivityB.RegionCount);
        Assert.Equal(2, connectivityA.RegionCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedEdgeOutsideCone_DoesNotJoinTwoAdmittedNotchLobes(bool reverseDiscovery)
    {
        // Mesh preparation centers the authored bounds at (1/2,0,2).
        // The committed triangles share X=[1/2,5/2], Z=0. The section is a
        // radius-5/4 disk centered at (-5/2,0), admitting both left tips but
        // missing their shared edge. Prepared adjacency is not connectivity.
        PhysicsMesh mesh = Create(new[]
        {
            new Vector3d(1,0,2), new Vector3d(3,0,2),
            new Vector3d(-2,0,1), new Vector3d(-2,0,3)
        }, new[] { 0, 1, 2, 1, 0, 3 });
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh, 0, new Vector3d(-(Fixed64)5 / 2, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            (Fixed64)4, (Fixed64)5 / 2, reverseDiscovery ? new[] { 1, 0 } : new[] { 0, 1 });
        Assert.Equal(2, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(0));
        Assert.Equal(1, connectivity.GetRegionOrdinal(1));
    }

    [Fact]
    public void TwiceWoundStar_DeclinesSharedManifoldOwnership()
    {
        // The radius-1/8 section is strictly inside overlapping triangles 2
        // and 4, away from their common vertex and every retained perimeter.
        // Seam cancellation alone is not a planar embedding certificate.
        PhysicsMesh mesh = Create(new[]
        {
            new Vector3d(0,0,3), new Vector3d(-3,0,1), new Vector3d(-2,0,-3),
            new Vector3d(2,0,-3), new Vector3d(3,0,1), Vector3d.Zero
        }, new[] { 5,0,2, 5,2,4, 5,4,1, 5,1,3, 5,3,0 });
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh, 2, new Vector3d(Fixed64.Quarter, Fixed64.Zero, (Fixed64)3 / 4),
            FixedQuaternion.Identity, (Fixed64)4, Fixed64.Quarter, new[] { 2, 4 });
        Assert.Equal(1, connectivity.RegionCount);
        Assert.Equal(0, connectivity.GetRegionOrdinal(2));
        Assert.Equal(-1, connectivity.GetRegionOrdinal(4));
        Assert.True(mesh.GetCoplanarTriangleNeighbors(2).IsEmpty);
        Assert.NotEqual(mesh.GetCanonicalSurfaceOrdinal(2), mesh.GetCanonicalSurfaceOrdinal(4));
        Assert.Equal(2, mesh.GetManifoldSurfaceOwner(2));
        Assert.Equal(4, mesh.GetManifoldSurfaceOwner(4));
        Assert.True(mesh.GetManifoldSurfaceBoundaryVertexPairs(2).IsEmpty);
        Assert.True(mesh.GetManifoldSurfaceBoundaryVertexPairs(4).IsEmpty);
    }

    [Theory]
    [InlineData(16)]
    [InlineData(32)]
    public void LargeVertexFan_ConnectsApexTouchWithRetainedCapacity(int sideSegments)
    {
        int count = sideSegments * 4;
        var vertices = new Vector3d[count + 1];
        var triangles = new int[count * 3];
        var admitted = new int[count];
        // Integer perimeter coordinates retain a collinear-subdivided square;
        // every triangle's cone section consists only of the shared center.
        for (int i = 0; i < sideSegments; i++)
        {
            int coordinate = -sideSegments + i * 2;
            vertices[1 + i] = new Vector3d(coordinate, 0, -sideSegments);
            vertices[1 + sideSegments + i] = new Vector3d(sideSegments, 0, coordinate);
            vertices[1 + 2 * sideSegments + i] = new Vector3d(-coordinate, 0, sideSegments);
            vertices[1 + 3 * sideSegments + i] = new Vector3d(-sideSegments, 0, -coordinate);
        }
        for (int i = 0; i < count; i++)
        {
            triangles[i * 3 + 1] = i + 1;
            triangles[i * 3 + 2] = (i + 1) % count + 1;
            admitted[i] = count - i - 1;
        }
        PhysicsMesh mesh = Create(vertices, triangles);
        var connectivity = new MeshConeSurfaceConnectivity();
        var center = new Vector3d(0, -2, 0);
        for (int i = 0; i < 8; i++)
            connectivity.Build(mesh, 0, center, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, admitted);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
            connectivity.Build(mesh, 0, center, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, admitted);
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(1, connectivity.RegionCount);
        for (int i = 0; i < count; i++) Assert.Equal(0, connectivity.GetRegionOrdinal(i));
    }

    private static PhysicsMesh Ring() => Create(new[]
    {
        new Vector3d(-2,0,-2), new Vector3d(2,0,-2), new Vector3d(2,0,2), new Vector3d(-2,0,2),
        new Vector3d(-Fixed64.Half,Fixed64.Zero,-Fixed64.Half), new Vector3d(Fixed64.Half,Fixed64.Zero,-Fixed64.Half),
        new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half), new Vector3d(-Fixed64.Half,Fixed64.Zero,Fixed64.Half)
    }, new[] { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 });

    private static PhysicsMesh Create(Vector3d[] vertices, int[] triangles) =>
        new(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
}
