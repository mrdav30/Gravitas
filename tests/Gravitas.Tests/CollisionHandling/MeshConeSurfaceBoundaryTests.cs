using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeSurfaceBoundaryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearBaseAxisCorner_ShouldChooseItsCapExitOverTheLongerSideCircle(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d corner = Vector3d.Down * Fixed64.Half;
        var mesh = new LSMeshCollider(new[] { corner, corner + Vector3d.Forward * Fixed64.Two,
            corner + Vector3d.Down + Vector3d.Forward * Fixed64.Two },
            reverse ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 2, 2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        // At this axis corner the base exit is 1/2. The admitted side-normal
        // circle has depth 3/(2*sqrt(5)) > 1/2 and must not add a competing
        // pressure row. The corner's finite cap witness is unique.
        bool found = false;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup contacts = ref pair.Manifold.GetGroup(group);
            if (contacts.Key.Region >= 0) continue;
            for (int sample = 0; sample < contacts.Count; sample++)
            {
                ManifoldContact contact = contacts[sample];
                if (contact.PointA != corner) continue;
                found = true;
                Assert.Equal(Fixed64.Half, contact.Depth);
                Assert.Equal(Vector3d.Up, contact.Normal);
                Assert.Equal(Vector3d.Down, contact.PointB);
            }
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void SurfaceGeneratorTouch_RetainsOnlyItsZeroOrRawNeighborExit(int offsetRaw)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateGeneratorPlane();
        scenario.InitializeStaticCollider(mesh, new(Fixed64.Half + Fixed64.FromRaw(offsetRaw), Fixed64.Zero, Fixed64.Zero));
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 2, 2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(mesh.Mesh, 0, cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, new[] { 0, 1 });
        var faces = new MeshConeFaceRegions();
        faces.Build(mesh.Mesh, 0, cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, connectivity, new[] { 0, 1 });
        var manifold = new ContactManifold();
        new MeshConeSurfaceBoundary().BuildSurfaceContacts(mesh, cone, 0, connectivity, faces, manifold);
        Assert.True(manifold.HasContact);
        foreach (ManifoldContact contact in manifold)
            Assert.InRange(contact.Depth.m_rawValue, 0, offsetRaw == 0 ? 0 : 2);
    }

    [Fact]
    public void FullLoop_StationaryGeneratorTouchDoesNotDepenetrate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateGeneratorPlane();
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(mesh, new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, immovable: true);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 2, 2), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scenario.CreateBody(cone, Vector3d.Zero, FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        Assert.Equal(Vector3d.Zero, body.Position3d);
        Assert.Equal(Vector3d.Zero, body.LinearVelocity);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SurfaceCorner_KeepsSelectedHemisphereAndSeveralTouchNormalsOnce(bool reverse, bool narrowFan)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d firstRay = narrowFan ? Vector3d.Right * (Fixed64)3 + Vector3d.Forward : Vector3d.Right;
        Vector3d secondRay = narrowFan ? Vector3d.Right * (Fixed64)3 + Vector3d.Forward * Fixed64.Two : Vector3d.Forward;
        var collider = new LSMeshCollider(new[] { Vector3d.Zero, firstRay, secondRay },
            reverse ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        // Runtime mesh initialization adds the authored bounds-center offset;
        // the host transform moves the authored origin, unlike PhysicsMesh.UpdatePosition.
        scenario.InitializeStaticCollider(collider, new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        collider.Mesh.GetLocalTriangleVertices(0, out Vector3d corner, out _, out _);
        Assert.True(new FixedPointAnchor(collider.Mesh.Origin, collider.Mesh.Rotation, corner).TryGetPoint(out Vector3d worldCorner));
        Assert.Equal(cone.Center + cone.WorldAxis * (cone.Height / Fixed64.Two), worldCorner);
        var connectivity = new MeshConeSurfaceConnectivity();
        connectivity.Build(collider.Mesh, 0, cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, new[] { 0 });
        var faces = new MeshConeFaceRegions();
        faces.Build(collider.Mesh, 0, cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, connectivity, new[] { 0 });
        var boundary = new MeshConeSurfaceBoundary();
        var manifold = new ContactManifold();
        boundary.BuildSurfaceContacts(collider, cone, 0, connectivity, faces, manifold);
        Assert.Equal(1, manifold.GroupCount);
        Assert.InRange(manifold.Count, 2, ContactManifold.MaxContactsPerGroup);
        // A narrow authored wedge admits a wider apex-normal sector, whose
        // distinct boundary and interior directions need the full sample budget.
        if (narrowFan) Assert.Equal(ContactManifold.MaxContactsPerGroup, manifold.Count);
        foreach (ManifoldContact contact in manifold)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.Normal.Y <= Fixed64.Zero);
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero), contact.PointA);
            Assert.Equal(contact.PointA, contact.PointB);
            Assert.True(Vector3d.Dot(contact.Normal, firstRay).m_rawValue <= 4);
            Assert.True(Vector3d.Dot(contact.Normal, secondRay).m_rawValue <= 4);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ApexFamily_UsesActualCornerFanAndWorldAnchors(bool reverse)
    {
        var mesh = Create(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward },
            reverse ? new[] { 0,2,1 } : new[] { 0,1,2 });
        // Standalone geometry is recentered: keep the authored first corner
        // at the cone apex rather than confusing mesh origin with this vertex.
        mesh.UpdatePosition(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Half), FixedQuaternion.Identity);
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        Assert.True(boundary.FamilyCount > 0);
        Assert.True(boundary.FamilySampleCount > 0);
        int touches = 0;
        for (int i = 0; i < boundary.FamilySampleCount; i++)
        {
            var contact = boundary.GetFamilySampleContact(i);
            Assert.True(boundary.TryGetFamilySampleWorldAnchors(i, out Vector3d p, out Vector3d q));
            Assert.Equal(Vector3d.Up, p);
            // The paired surface normal is opposite the cone's outward
            // apex support cone; exact zero-depth ownership retains that sign.
            if (contact.Depth == Fixed64.Zero)
            {
                touches++;
                Assert.Equal(p, q);
                Assert.True(contact.Normal.Y <= Fixed64.Zero);
            }
            else
            {
                // The same corner's horizontal fan also admits the positive
                // base and rim exits. They remain separate finite families.
                Assert.Equal(-Fixed64.One, q.Y);
                Assert.True(contact.Normal.Y > Fixed64.Zero);
                if (q.X == Fixed64.Zero && q.Z == Fixed64.Zero)
                {
                    Assert.Equal(Fixed64.Two, contact.Depth);
                    Assert.Equal(Vector3d.Up, contact.Normal);
                }
                else
                {
                    Assert.InRange((q.X * q.X + q.Z * q.Z).m_rawValue,
                        Fixed64.One.m_rawValue - 2, Fixed64.One.m_rawValue + 2);
                    Assert.Equal(FixedMath.Sqrt((Fixed64)5), contact.Depth);
                }
            }
            Assert.True(contact.Normal.X.m_rawValue <= 1);
            Assert.True(contact.Normal.Z.m_rawValue <= 1);
        }
        Assert.True(touches > 0);
        Assert.Throws<IndexOutOfRangeException>(() => boundary.GetFamilySampleContact(-1));
        Assert.False(boundary.TryGetFamilySampleWorldAnchors(-1, out _, out _));
        Assert.False(boundary.TryGetFamilySampleWorldAnchors(boundary.FamilySampleCount, out _, out _));
        boundary.Build(mesh, 0, new Vector3d(0,10,0), FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        Assert.Equal(0, boundary.FamilySampleCount);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void BoundaryFans_UseAnInjectiveProjectionForEachAuthoredPlane(int axis)
    {
        Vector3d[] vertices = axis == 0 ? new[] { Vector3d.Zero, Vector3d.Up, Vector3d.Forward }
            : axis == 1 ? new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward }
            : new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up };
        var mesh = Create(vertices, new[] { 0,1,2 });
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        Assert.True(boundary.IsolatedCount + boundary.FamilySampleCount > 0);
        for (int i = 0; i < boundary.IsolatedCount + boundary.FamilySampleCount; i++)
        {
            FixedContactAnchors contact;
            Vector3d p;
            if (i < boundary.IsolatedCount)
            {
                contact = boundary.GetIsolatedContact(i);
                Assert.True(contact.FirstAnchor.TryGetPoint(out p));
            }
            else
            {
                int index = i - boundary.IsolatedCount;
                contact = boundary.GetFamilySampleContact(index);
                Assert.True(boundary.TryGetFamilySampleWorldAnchors(index, out p, out _));
            }
            foreach (int neighbor in mesh.GetWeldedTriangleVertexIndices(0))
                Assert.True(Vector3d.Dot(contact.Normal, mesh.ScaledLocalVertices[neighbor] - p).m_rawValue <= 16);
        }
    }
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

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReflexCorner_WithOppositeCollinearIncidentRayKeepsOnlyPlaneSupport(bool reverseDiscovery)
    {
        int[] triangles = reverseDiscovery ? new[] { 3,6,0, 3,5,6, 3,4,5, 3,0,1, 3,1,2 }
            : new[] { 3,0,1, 3,1,2, 3,4,5, 3,5,6, 3,6,0 };
        var mesh = Create(new[] { new Vector3d(-2,0,-2), new Vector3d(2,0,-2),
            new Vector3d(2,0,0), Vector3d.Zero, new Vector3d(0,0,2),
            new Vector3d(-2,0,2), new Vector3d(-2,0,0) }, triangles);
        var boundary = new MeshConeSurfaceBoundary();
        boundary.Build(mesh, 0, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4);
        int cornerContacts = 0;
        for (int i = 0; i < boundary.IsolatedCount; i++)
        {
            var contact = boundary.GetIsolatedContact(i);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
            if (p != Vector3d.Zero) continue;
            cornerContacts++;
            Assert.Equal(Fixed64.Zero, contact.Normal.X);
            Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        }
        for (int i = 0; i < boundary.FamilySampleCount; i++)
        {
            Assert.True(boundary.TryGetFamilySampleWorldAnchors(i, out Vector3d p, out _));
            if (p != Vector3d.Zero) continue;
            cornerContacts++;
            var contact = boundary.GetFamilySampleContact(i);
            Assert.Equal(Fixed64.Zero, contact.Normal.X);
            Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        }
        Assert.True(cornerContacts > 0);
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

    private static LSMeshCollider CreateGeneratorPlane() => new(new Vector3d[]
        {
            new(Fixed64.Quarter, -Fixed64.Half, -Fixed64.One), new(-Fixed64.Quarter, Fixed64.Half, -Fixed64.One),
            new(-Fixed64.Quarter, Fixed64.Half, Fixed64.One), new(Fixed64.Quarter, -Fixed64.Half, Fixed64.One)
        }, new[] { 0, 1, 2, 0, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
}
