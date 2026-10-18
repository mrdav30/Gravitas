using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceMeshLifecycleTests
{
    [Fact]
    public void NonconvexFloorAndSeparateWallSharingACollider_ShouldRetainEachOwnersFiniteConstraint()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = new LSMeshCollider(new[] { new Vector3d(-2, 0, -2), new(2, 0, -2), new(2, 0, -1),
            new(0, 0, -1), new(0, 0, 2), new(-2, 0, 2), new(-3, 0, -2), new(-3, 0, 2),
            new(-3, 2, -2), new(-3, 2, 2) },
            new[] { 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 6, 7, 8, 7, 9, 8 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(source, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
        var mover = Cube(new Vector3d(3, 1, 1));
        scene.CreateBody(mover, new(-7 * Fixed64.Quarter, 3 * Fixed64.One / 8, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source, floor = false, wall = false;
        foreach (var row in pair.Manifold)
        {
            Vector3d normal = sourceA ? row.Normal : -row.Normal;
            Assert.False(row.DepthIsClamped);
            Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
            Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
            Assert.True(normal == Vector3d.Up || normal == Vector3d.Right,
                $"normal={normal}; depth={row.Depth}; source={point}; moving={moving}");
            if (normal == Vector3d.Up)
            {
                floor = true; Assert.Equal(Fixed64.One / 8, row.Depth);
                Assert.Equal(Fixed64.Zero, point.Y);
                Assert.InRange(point.X.m_rawValue, ((Fixed64)(-2)).m_rawValue, 0L);
            }
            else
            {
                wall = true; Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.Equal((Fixed64)(-3), point.X);
                Assert.InRange(point.Y.m_rawValue, 0L, Fixed64.Two.m_rawValue);
            }
            Assert.InRange(point.Z.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Two.m_rawValue);
        }
        Assert.True(floor && wall, "a nonconvex floor's triangle search cannot consume its collider's separate wall owner");
    }

    [Fact]
    public void ConcaveMovingSheetOverAHole_ShouldRejectTheEmptyPlaneProjectionAndKeepRealRimContacts()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 hole = Fixed64.One / 40;
        Fixed64[] x = { (Fixed64)(-2), -hole, hole, (Fixed64)2 };
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, (Fixed64)2 };
        var ring = Ring(scene, x, z);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { ring });
        // The first triangle has its lowest point over the opening. Its
        // sloping sides meet the actual rim; the attached wing makes this a
        // genuinely nonconvex owner that retains native triangle semantics.
        var mover = new LSMeshCollider(new[] { new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new(Fixed64.Half, Fixed64.Half, Fixed64.Zero), new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero),
            new(-Fixed64.Half, Fixed64.Half, Fixed64.Zero), new(-Fixed64.One, Fixed64.Half, Fixed64.Zero),
            new(-Fixed64.One, Fixed64.Zero, Fixed64.Zero) }, new[] { 0, 1, 2, 1, 3, 2, 3, 4, 5, 3, 5, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        var body = scene.CreateBody(mover, Vector3d.Up * Fixed64.FromFraction(2, 5), FixedQuaternion.Identity).Body;
        var pair = scene.CreatePair(ring, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == ring;
        foreach (var row in pair.Manifold)
        {
            Assert.True(row.Depth > Fixed64.Zero); Assert.False(row.DepthIsClamped);
            Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var source));
            Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out _));
            Assert.Equal(Fixed64.Zero, source.Y);
            Assert.True(source.X.Abs() >= hole || source.Z.Abs() >= Fixed64.Half,
                "a proposed plane projection inside the empty opening cannot become a physical witness");
        }
        // A shallower, still geometrically straddling pose fits entirely
        // through the opening. The same native path must now report no hit.
        body.SetPosition(Vector3d.Up * (Fixed64.Half - Fixed64.FromRaw(1))); mover.RebuildRuntimeShapeOnly();
        pair = scene.CreatePair(ring, mover);
        Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
    }

    [Fact]
    public void InteriorTrianglesWithoutAnyRetainedLocalWitness_ShouldPreserveOverlapAndTheActualRingResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var ring = Ring(scene,
            new Fixed64[] { (Fixed64)(-4), -Fixed64.Half, Fixed64.Half, Fixed64.One, Fixed64.Two, (Fixed64)4 },
            new Fixed64[] { (Fixed64)(-4), -Fixed64.Half, -Fixed64.Quarter, Fixed64.Quarter, Fixed64.Half, (Fixed64)4 });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { ring });
        var mover = new LSMeshCollider(new[] { new Vector3d(0, -1, 0), new(0, 1, 0), new(2, 0, 0) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        var pair = scene.CreatePair(ring, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == ring;
        foreach (var row in pair.Manifold)
        {
            Vector3d normal = sourceA ? row.Normal : -row.Normal;
            Assert.True(normal.X != Fixed64.Zero || normal.Z != Fixed64.Zero,
                "both vertical extremes project into the hole, so an unsupported plane row is not a finite witness");
            Assert.True(row.Depth > Fixed64.Zero); Assert.False(row.DepthIsClamped);
            Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
            Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out _));
            Assert.Equal(Fixed64.Zero, point.Y);
            Assert.True(point.X.Abs() >= Fixed64.Half || point.Z.Abs() >= Fixed64.Half);
        }
        // Interior triangles at x=[1,2], z=[-.25,.25] overlap the sheet,
        // but supply no physical perimeter or admissible plane projection.
        // Rejecting all of their local candidates cannot erase the real
        // overlap or replace the ring's genuine rows with authored diagonals.
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 1)]
    public void TangentSupplier_ShouldKeepItsFiniteAnchorsWhileTheFartherJoinedPieceSuppliesNone(int normalAxis, int sign)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        int tangentAxis = (normalAxis + 1) % 3, otherAxis = (normalAxis + 2) % 3;
        Fixed64 join = sign * 3 * Fixed64.Quarter, outer = sign * 5 * Fixed64.Quarter;
        // Register the tangent supplier first so coincident physical witnesses
        // retain its stable supplier identity at the shared line.
        var tangent = Patch(FixedMath.Min(join, outer), FixedMath.Max(join, outer));
        var carrier = sign > 0 ? Patch((Fixed64)(-4), join) : Patch(join, (Fixed64)4);
        var remote = sign > 0 ? Patch(outer, (Fixed64)4) : Patch((Fixed64)(-4), outer);
        tangent.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { remote, carrier, tangent });
        Vector3d start = Vector3d.Zero, normal = Vector3d.Zero;
        start[normalAxis] = Fixed64.Quarter; start[tangentAxis] = sign * Fixed64.Quarter;
        normal[normalAxis] = Fixed64.One;
        var mover = Cube(Vector3d.One);
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        Assert.Equal(join, sign > 0 ? mover.BoundsMax[tangentAxis] : mover.BoundsMin[tangentAxis]);
        Assert.True(sign > 0 ? remote.BoundsMin[tangentAxis] > mover.BoundsMax[tangentAxis]
            : remote.BoundsMax[tangentAxis] < mover.BoundsMin[tangentAxis]);
        bool retainedTangent = false;
        foreach (var member in new[] { tangent, carrier, remote })
        {
            var pair = scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) { Assert.Empty(pair.Manifold); continue; }
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                int supplier = sourceA ? group.Key.SourceIdA : group.Key.SourceIdB;
                for (int pointIndex = 0; pointIndex < group.Count; pointIndex++)
                {
                    var row = group[pointIndex];
                    Assert.Equal(normal, sourceA ? row.Normal : -row.Normal);
                    Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                    Assert.NotEqual(remote.Id + 1, supplier);
                    Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
                    Assert.Equal(Fixed64.Zero, point[normalAxis]);
                    if (supplier != tangent.Id + 1) continue;
                    retainedTangent = true;
                    Assert.Equal(join, point[tangentAxis]);
                    Assert.Equal(PhysicsMaterial.Bouncy, sourceA ? row.MaterialA : row.MaterialB);
                }
            }
        }
        Assert.True(retainedTangent, "an exact boundary touch remains a finite supplier, with its own material");
        surface.Dispose();
        body.SetPosition(start + normal * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { tangent, carrier, remote })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));

        LSMeshCollider Patch(Fixed64 min, Fixed64 max)
        {
            Vector3d Vertex(Fixed64 along, Fixed64 across)
            { Vector3d point = Vector3d.Zero; point[tangentAxis] = along; point[otherAxis] = across; return point; }
            var mesh = new LSMeshCollider(new[] { Vertex(min, (Fixed64)(-4)), Vertex(max, (Fixed64)(-4)),
                Vertex(min, (Fixed64)4), Vertex(max, (Fixed64)4) },
                new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
        }
    }

    [Fact]
    public void FullyObliquePlane_ShouldRetainTheFiniteSupplierBeyondTheMoversWorldBounds()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 cutX = Fixed64.FromFraction(4, 5), cutZ = -Fixed64.One / 5;
        var supplier = Patch(cutX, (Fixed64)4, cutZ, (Fixed64)4);
        var carrier = Patch((Fixed64)(-4), cutX, (Fixed64)(-4), (Fixed64)4);
        var lower = Patch(cutX, (Fixed64)4, (Fixed64)(-4), cutZ);
        supplier.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { supplier, carrier, lower });
        Vector3d normal = new Vector3d(-1, 1, -1).Normalized;
        var mover = Cube(new Vector3d(4, 1, 2));
        scene.CreateBody(mover, normal * Fixed64.FromFraction(1, 10), FixedQuaternion.Identity);
        Assert.True(supplier.BoundsMin.Y > mover.BoundsMax.Y);
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(supplier, mover)));
        var pair = scene.CreatePair(carrier, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == carrier;
        foreach (var row in pair.Manifold)
        {
            Vector3d direction = sourceA ? row.Normal : -row.Normal;
            Assert.True(direction.X < Fixed64.Zero && direction.Y > Fixed64.Zero && direction.Z < Fixed64.Zero);
            Assert.Equal(-direction.X, direction.Y); Assert.Equal(-direction.Z, direction.Y);
            Assert.InRange(row.Depth.m_rawValue, Fixed64.One.m_rawValue, Fixed64.Two.m_rawValue);
            Assert.False(row.DepthIsClamped);
            Assert.Equal(PhysicsMaterial.Bouncy, sourceA ? row.MaterialA : row.MaterialB);
            ContactAnchor anchor = sourceA ? row.AnchorA : row.AnchorB;
            Assert.Equal(supplier.Mesh.Origin, anchor.Origin); Assert.Equal(supplier.Mesh.Rotation, anchor.Rotation);
            Assert.True(anchor.TryGetWorldPoint(out var point));
            Assert.True(point.Y > mover.BoundsMax.Y);
            Assert.InRange(point.X.m_rawValue, Fixed64.FromFraction(4, 5).m_rawValue, ((Fixed64)4).m_rawValue);
            Assert.InRange(point.Z.m_rawValue, cutZ.m_rawValue, ((Fixed64)4).m_rawValue);
            Assert.InRange((point.Y - point.X - point.Z).Abs().m_rawValue, 0L, 2L);
        }

        LSMeshCollider Patch(Fixed64 min, Fixed64 max, Fixed64 near, Fixed64 far)
        {
            // Author y=x+z directly: all normal components are nonzero,
            // while the pieces retain exactly the same shared geometry.
            var mesh = new LSMeshCollider(new[] { new Vector3d(min, min + near, near), new(max, max + near, near),
                new(min, min + far, far), new(max, max + far, far) },
                new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void RemoteSupplierMutation_ShouldRetireCarrierLoadsAndRefreshTheSamePose(int mutation)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 join = -Fixed64.FromFraction(36, 25);
        var supplier = Floor(scene, -Fixed64.FromFraction(3, 2), join);
        var carrier = Floor(scene, join, (Fixed64)2);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { supplier, carrier });
        var mover = Cube(new Vector3d(4, 1, 2));
        var body = scene.CreateBody(mover, Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30)).Body;
        Vector3d position = body.Position3d;
        var pair = scene.Context.Physics.GetCollisionPair(carrier.Id, mover.Id)!;
        Assert.NotNull(pair);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        ref var initial = ref pair.Manifold.GetGroup(0);
        var key = initial.Key;
        var row = initial[0];
        int supplierId = supplier.Id + 1;
        Assert.True(key.SourceIdA == supplierId || key.SourceIdB == supplierId);
        pair.StoreWarmStartImpulse(key, row.ContactId, row.Normal, Fixed64.One, Fixed64.Half, Fixed64.Quarter);
        Assert.True(pair.TryGetWarmStartImpulse(key, row.ContactId, out _));

        LSMeshCollider currentSupplier = supplier;
        switch (mutation)
        {
            case 0: supplier.Deactivate(); break;
            case 1:
                currentSupplier = Floor(scene, -Fixed64.FromFraction(3, 2), join);
                currentSupplier.Material = PhysicsMaterial.Bouncy;
                surface.ReplaceMembers(new[] { carrier, currentSupplier });
                break;
            case 2: supplier.Material = PhysicsMaterial.Bouncy; break;
            case 3: supplier.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(mover.Layer.Index); break;
            default: supplier.IsTrigger = true; break;
        }

        Assert.True(pair.Active);
        Assert.False(pair.TryGetWarmStartImpulse(key, row.ContactId, out _));
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.NotEmpty(pair.Manifold);
        bool carrierA = pair.ColliderA == carrier;
        for (int index = 0; index < pair.Manifold.GroupCount; index++)
        {
            ref var group = ref pair.Manifold.GetGroup(index);
            int source = carrierA ? group.Key.SourceIdA : group.Key.SourceIdB;
            if (mutation == 1 || mutation == 2)
            {
                Assert.Equal(currentSupplier.Id + 1, source);
                for (int point = 0; point < group.Count; point++)
                    Assert.Equal(PhysicsMaterial.Bouncy, carrierA ? group[point].MaterialA : group[point].MaterialB);
            }
            else Assert.NotEqual(supplierId, source);
        }
        Assert.Equal(position, body.Position3d);
    }

    [Fact]
    public void MovingMeshScaleChange_ShouldRefreshSupportDepthWithoutChangingTheBodyPose()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, (Fixed64)(-4), Fixed64.Zero);
        var right = Floor(scene, Fixed64.Zero, (Fixed64)4);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var mover = Cube(Vector3d.One);
        var body = scene.CreateBody(mover, new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity).Body;
        Vector3d position = body.Position3d;
        CheckDepth(Fixed64.Quarter);

        mover.Transform.LocalScale = new Vector3d(1, 2, 1);
        mover.RebuildRuntimeShapeOnly();

        Assert.Equal(Fixed64.Two, mover.BoundsMax.Y - mover.BoundsMin.Y);
        CheckDepth(3 * Fixed64.Quarter);
        Assert.Equal(position, body.Position3d);

        void CheckDepth(Fixed64 depth)
        {
            int count = 0;
            foreach (var member in new[] { left, right })
            {
                var pair = scene.CreatePair(member, mover);
                Assert.True(CollisionDetection.DoCollisionCheck(pair));
                foreach (var row in pair.Manifold)
                { Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(depth, row.Depth); count++; }
            }
            Assert.Equal(4, count);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExcludedOrAmbiguousNeighbor_ShouldPreserveTheOriginalFiniteRim(bool ambiguous)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, (Fixed64)(-2), Fixed64.Zero);
        var right = Floor(scene, Fixed64.Zero, (Fixed64)2);
        if (!ambiguous) right.IsTrigger = true;
        using var surface = scene.Context.Physics.CreateStaticSurface(ambiguous
            ? new[] { left, right, Floor(scene, Fixed64.Zero, (Fixed64)2) } : new[] { left, right });
        var mover = Cube(Vector3d.One);
        scene.CreateBody(mover, new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(3, 10), Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(left, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.NotEmpty(pair.Manifold);
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(Vector3d.Right, row.Normal);
            Assert.Equal(Fixed64.Half - Fixed64.FromFraction(2, 5), row.Depth);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
            Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Y);
        }
    }

    [Fact]
    public void FoldedMember_ShouldPreserveBothFloorAndWallConstraints()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, (Fixed64)(-4), Fixed64.Zero);
        var folded = new LSMeshCollider(new[] { new Vector3d(0, 0, -4), new(4, 0, -4), new(0, 0, 4), new(4, 0, 4),
            new(0, 4, -4), new(0, 4, 4) }, new[] { 0, 2, 1, 1, 2, 3, 0, 4, 2, 2, 4, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(folded, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { folded, left });
        var mover = Cube(Vector3d.One);
        scene.CreateBody(mover, new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        bool floor = false, wall = false;
        foreach (var member in new[] { left, folded })
        {
            var pair = scene.CreatePair(member, mover);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.True(row.Normal == Vector3d.Up || row.Normal == Vector3d.Right);
                floor |= row.Normal == Vector3d.Up; wall |= row.Normal == Vector3d.Right;
                Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
                Assert.True(row.Normal == Vector3d.Up ? point.Y == Fixed64.Zero : point.X == Fixed64.Zero);
            }
        }
        Assert.True(floor && wall);
    }

    private static LSMeshCollider Ring(PhysicsScenarioBuilder scene, Fixed64[] x, Fixed64[] z)
    {
        var vertices = new Vector3d[x.Length * z.Length]; var triangles = new System.Collections.Generic.List<int>();
        for (int row = 0; row < z.Length; row++)
            for (int column = 0; column < x.Length; column++)
            {
                int vertex = row * x.Length + column; vertices[vertex] = new(x[column], Fixed64.Zero, z[row]);
                if (row == z.Length - 1 || column == x.Length - 1 || column == 1 && row > 0 && row < z.Length - 2) continue;
                triangles.AddRange(new[] { vertex, vertex + x.Length, vertex + 1, vertex + 1, vertex + x.Length, vertex + x.Length + 1 });
            }
        var mesh = new LSMeshCollider(vertices, triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
    }

    private static LSMeshCollider Floor(PhysicsScenarioBuilder scene, Fixed64 min, Fixed64 max)
    {
        var mesh = new LSMeshCollider(new[] { new Vector3d(min, Fixed64.Zero, (Fixed64)(-4)), new(max, Fixed64.Zero, (Fixed64)(-4)),
            new(min, Fixed64.Zero, (Fixed64)4), new(max, Fixed64.Zero, (Fixed64)4) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
    }

    private static LSMeshCollider Cube(Vector3d scale)
    {
        var unit = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = unit.Mesh.LocalVertices.ToArray();
        for (int index = 0; index < vertices.Length; index++) vertices[index] *= scale;
        return new(vertices, unit.Mesh.Triangles.ToArray(), MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
    }
}
