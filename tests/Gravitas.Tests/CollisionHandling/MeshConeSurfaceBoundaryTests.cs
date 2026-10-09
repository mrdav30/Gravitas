using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeSurfaceBoundaryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteBoundaryNormals_MustSupportTheirActualTriangleFan(bool reverse)
    {
        int[] triangles = reverse ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 };
        var mesh = new PhysicsMesh(new[] { new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1),
            new Vector3d(-1, 0, 1) }, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        Assert.True(boundary.IsolatedCount > 0);
        int corners = 0;
        for (int i = 0; i < boundary.IsolatedCount; i++)
        {
            int vertex = boundary.GetIsolatedVertex(i);
            FixedContactAnchors contact = boundary.GetIsolatedContact(i);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
            if (vertex >= 0)
            {
                corners++;
                Assert.Equal(mesh.ScaledLocalVertices[vertex], point);
            }
            foreach (int neighbor in mesh.GetWeldedTriangleVertexIndices(0))
            {
                Vector3d ray = mesh.ScaledLocalVertices[neighbor] - point;
                // The fixture has generous nonzero margins. One raw normal
                // rounding unit only admits a representational zero here.
                Assert.True(Vector3d.Dot(contact.Normal, ray).m_rawValue <= 4,
                    "A finite boundary normal points into its own authored face.");
            }
        }
        Assert.True(corners > 0);
    }

    [Fact]
    public void CoveredDiagonal_IsNotAnExposedContactFeature()
    {
        var mesh = Create(new[] { new Vector3d(-4,0,-4), new Vector3d(4,0,-4),
            new Vector3d(4,0,4), new Vector3d(-4,0,4) }, new[] { 0,1,2, 0,2,3 });
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2);
        Assert.Equal(0, boundary.IsolatedCount);
        Assert.Equal(0, boundary.FamilyCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleCorners_UseTheWholeIncidentFan(bool reverse)
    {
        Vector3d[] vertices = { new(-2,0,-2), new(2,0,-2), new(2,0,2), new(-2,0,2),
            new(-1,0,-1), new(1,0,-1), new(1,0,1), new(-1,0,1) };
        int[] triangles = { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 };
        if (reverse)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        var mesh = Create(vertices, triangles);
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)10);
        Assert.True(boundary.IsolatedCount > 0);
        int outerCorners = 0;
        for (int i = 0; i < boundary.IsolatedCount; i++)
        {
            int vertex = boundary.GetIsolatedVertex(i);
            if (vertex < 0) continue;
            if (vertex < 4) outerCorners++;
            FixedContactAnchors contact = boundary.GetIsolatedContact(i);
            foreach (int incident in mesh.GetWeldedVertexTriangleIndices(vertex))
                foreach (int neighbor in mesh.GetWeldedTriangleVertexIndices(incident))
                    Assert.True(Vector3d.Dot(contact.Normal, vertices[neighbor] - vertices[vertex]).m_rawValue <= 16);
            // A reflex fan's polar contains only plane-perpendicular normals.
            if (vertex >= 4)
            {
                Assert.InRange(contact.Normal.X.m_rawValue, -1L, 1L);
                Assert.InRange(contact.Normal.Z.m_rawValue, -1L, 1L);
            }
        }
        Assert.True(outerCorners > 0);
    }

    [Fact]
    public void DeclinedOwners_DoNotBorrowAnotherTrianglesFan()
    {
        Vector3d[] vertices = { new(-1,0,-1), new(1,0,-1), new(-1,0,1), new(0,1,0) };
        var single = Create(vertices[..3], new[] { 0,1,2 });
        // Overlapping duplicates are declined; the vertical triangle sharing
        // a boundary vertex must likewise remain a separate physical owner.
        var declined = Create(vertices, new[] { 0,1,2, 0,1,2, 0,2,3 });
        // PhysicsMesh positions its bounds center. Keep the tested triangle
        // at the same world plane despite the added vertical owner's bounds.
        declined.UpdatePosition(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity);
        Assert.Empty(declined.GetManifoldSurfaceBoundaryVertexPairs(0).ToArray());
        var baseline = new MeshConeSurfaceBoundary(); var actual = new MeshConeSurfaceBoundary();
        baseline.Build(single, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        actual.Build(declined, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        Assert.Equal(baseline.IsolatedCount, actual.IsolatedCount);
        Assert.Equal(baseline.FamilyCount, actual.FamilyCount);
        for (int i = 0; i < actual.IsolatedCount; i++)
        {
            Assert.Equal(baseline.GetIsolatedContact(i).Normal, actual.GetIsolatedContact(i).Normal);
            Assert.Equal(baseline.GetIsolatedContact(i).Depth, actual.GetIsolatedContact(i).Depth);
        }
    }

    [Fact]
    public void ReusedBoundary_ClearsOldContactsAndAllocatesZeroAfterWarmup()
    {
        var mesh = Create(new[] { new Vector3d(-1,0,-1), new Vector3d(1,0,-1), new Vector3d(-1,0,1) }, new[] { 0,1,2 });
        var boundary = new MeshConeSurfaceBoundary();
        for (int i = 0; i < 8; i++)
            boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        Assert.True(boundary.IsolatedCount > 0);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
        {
            boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
            for (int j = 0; j < boundary.IsolatedCount; j++) _ = boundary.GetIsolatedContact(j);
        }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        boundary.Build(mesh, 0, new Vector3d(0,10,0), FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        Assert.Equal(0, boundary.IsolatedCount);
        Assert.Equal(0, boundary.FamilyCount);
        Assert.Throws<IndexOutOfRangeException>(() => boundary.GetIsolatedContact(0));
    }

    private static PhysicsMesh Create(Vector3d[] vertices, int[] triangles) =>
        new(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
}
