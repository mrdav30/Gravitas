using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceMeshAdmissionTests
{
    [Fact]
    public void PartialJoin_ShouldRetainItsGenuineRimBesideAnInitiallySeparatedMember()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Rectangle(scene, -4, 0, -4, 4);
        var right = Rectangle(scene, 0, 4, -2, 2);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var unit = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = unit.Mesh.LocalVertices.ToArray();
        for (int index = 0; index < vertices.Length; index++) vertices[index].Y *= (Fixed64)4;
        var mover = new LSMeshCollider(vertices, unit.Mesh.Triangles.ToArray(),
            MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        Vector3d start = new(-Fixed64.FromFraction(3, 5), Fixed64.FromFraction(3, 10), (Fixed64)2);
        scene.CreateBody(mover, start, FixedQuaternion.Identity);
        Assert.Equal(new Vector3d(1, 4, 1), mover.BoundsMax - mover.BoundsMin);
        Assert.True(mover.BoundsMax.X < right.BoundsMin.X);
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(right, mover)));
        var pair = scene.CreatePair(left, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        Fixed64 depth = Fixed64.Half - start.X;
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(Vector3d.Right, row.Normal);
            Assert.Equal(depth, row.Depth); Assert.False(row.DepthIsClamped);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Y);
            Assert.InRange(source.Z.m_rawValue, ((Fixed64)2).m_rawValue, (start.Z + Fixed64.Half).m_rawValue);
            Assert.True(row.AnchorB.TryGetWorldPoint(out var moving));
            Assert.Equal(start.X - Fixed64.Half, moving.X);
            Assert.Equal(source.Y, moving.Y); Assert.Equal(source.Z, moving.Z);
        }
        // This genuine finite constraint is local stabilization, not a
        // certificate that its full-depth translation clears the surface union.
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubdividedConcaveRim_ShouldMatchItsCanonicalUnsplitExit(bool reverse)
    {
        var control = Check(false);
        Assert.True(control.SetEquals(Check(true)));

        HashSet<Vector3d> Check(bool subdivided)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            // The same L-shaped domain uses either one x=0 rim edge, or
            // three authored subedges whose monotone corners reduce away.
            var vertices = new[] { new Vector3d(-2, 0, -2), new(2, 0, -2), new(2, 0, -1),
                new(0, 0, -1), new(0, 0, 2), new(-2, 0, 2), new(0, 0, 0), new(0, 0, 1) };
            if (!subdivided) vertices = vertices[..6];
            var source = new LSMeshCollider(vertices, subdivided
                ? new[] { 0, 1, 2, 0, 2, 3, 0, 3, 6, 0, 6, 7, 0, 7, 4, 0, 4, 5 }
                : new[] { 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(source, Vector3d.Zero);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
            var unit = MeshTestFixtures.CreateConvexCube();
            Vector3d[] moving = unit.Mesh.LocalVertices.ToArray();
            for (int index = 0; index < moving.Length; index++) moving[index].Z *= Fixed64.Half;
            var mover = new LSMeshCollider(moving, unit.Mesh.Triangles.ToArray(),
                MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
            Vector3d start = new(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(3, 10), Fixed64.Half);
            var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
            Assert.Equal(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Half), mover.BoundsMax - mover.BoundsMin);
            var pair = reverse ? scene.CreatePair(mover, source) : scene.CreatePair(source, mover);
            Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
            bool sourceA = pair.ColliderA == source;
            var points = new HashSet<Vector3d>();
            Fixed64 depth = Fixed64.Half - start.X;
            foreach (var row in pair.Manifold)
            {
                Assert.Equal(Vector3d.Right, sourceA ? row.Normal : -row.Normal);
                Assert.Equal(depth, row.Depth); Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
                Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Y);
                Assert.InRange(point.Z.m_rawValue, Fixed64.Quarter.m_rawValue, (3 * Fixed64.Quarter).m_rawValue);
                points.Add(point);
            }
            Assert.Equal(2, points.Count);
            surface.Dispose();
            body.SetPosition(start + Vector3d.Right * (depth + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(source, mover)));
            return points;
        }
    }

    [Fact]
    public void HullCoveringBothUnclippedEdgeEnds_ShouldJoinOnlyThroughTheActualFiniteWindow()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Rectangle(scene, -4, 0, -4, 4);
        var right = Rectangle(scene, 0, 4, -2, 2);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var unit = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = unit.Mesh.LocalVertices.ToArray();
        for (int index = 0; index < vertices.Length; index++)
            vertices[index] = new(vertices[index].X * 9, vertices[index].Y, vertices[index].Z * 9);
        var mover = new LSMeshCollider(vertices, unit.Mesh.Triangles.ToArray(),
            MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        Assert.Equal(new Vector3d(9, 1, 9), mover.BoundsMax - mover.BoundsMin);
        var regions = new HashSet<int>();
        int contacts = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) continue;
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                if (group.Count > 0) regions.Add(group.Key.Region);
                for (int pointIndex = 0; pointIndex < group.Count; pointIndex++)
                {
                    var row = group[pointIndex]; contacts++;
                    Assert.Equal(Vector3d.Up, sourceA ? row.Normal : -row.Normal);
                    Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                    Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
                    Assert.Equal(Fixed64.Zero, point.Y);
                    Assert.InRange(point.X.m_rawValue, ((Fixed64)(-4)).m_rawValue, ((Fixed64)4).m_rawValue);
                    Assert.True(point.Z.Abs() <= (point.X > Fixed64.Zero ? (Fixed64)2 : (Fixed64)4),
                        "the narrow neighbor cannot fill the exposed ends of the longer source edge");
                }
            }
        }
        Assert.True(contacts >= 2);
        Assert.Single(regions);
        // Both original z=±4 edge vertices lie in the hull's support face,
        // outside the certified z=[-2,2] join. Only its clipped interior may
        // connect the two finite supports into the one physical region.
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveMovingOwner_ShouldPreserveLocalTriangleConstraintsAndItsActualMissingArea(bool reversed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Rectangle(scene, -4, 0, -4, 4);
        var right = Rectangle(scene, 0, 4, -4, 4);
        using var surface = scene.Context.Physics.CreateStaticSurface(reversed ? new[] { right, left } : new[] { left, right });
        var vertices = new Vector3d[8];
        for (int index = 0; index < vertices.Length; index++)
            vertices[index] = new((index % 3 - 1) * Fixed64.Half, (index / 3 - 1) * Fixed64.Half, Fixed64.Zero);
        // Three coplanar squares form one connected L-shaped owner. Its
        // missing upper-right square must not become a convex moving patch.
        var mover = new LSMeshCollider(vertices, new[] { 0, 1, 3, 1, 4, 3, 1, 2, 4, 2, 5, 4, 3, 4, 6, 4, 7, 6 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d start = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        int contacts = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = reversed ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) continue;
            bool sourceA = pair.ColliderA == member;
            foreach (var row in pair.Manifold)
            {
                contacts++;
                Vector3d normal = sourceA ? row.Normal : -row.Normal;
                Assert.True(normal == Vector3d.Up || normal == Vector3d.Down);
                Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var source));
                Assert.Equal(Fixed64.Zero, source.Y); Assert.Equal(Fixed64.Zero, source.Z);
                Assert.InRange(source.X.m_rawValue, (-Fixed64.Quarter).m_rawValue, (3 * Fixed64.Quarter).m_rawValue);
                Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
                Assert.Equal(source.X, moving.X); Assert.Equal(Fixed64.Zero, moving.Z);
                Assert.Equal(-normal.Y * Fixed64.Quarter, moving.Y);
            }
        }
        Assert.True(contacts > 0);
        // Native open triangles supply local constraints, not one owner-wide
        // recovery direction. Separately verify this explicitly chosen clear pose.
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));

        // These joined finite pieces pass through the empty upper-right
        // square, inside the L's convex hull and overall bounding box.
        var near = Rectangle(scene, 5 * Fixed64.One / 16, 7 * Fixed64.One / 16, -Fixed64.Quarter, Fixed64.Zero);
        var far = Rectangle(scene, 5 * Fixed64.One / 16, 7 * Fixed64.One / 16, Fixed64.Zero, Fixed64.Quarter);
        using var gapSurface = scene.Context.Physics.CreateStaticSurface(new[] { near, far });
        body.SetPosition(new(Fixed64.Quarter, -Fixed64.One / 8, Fixed64.Zero)); mover.RebuildRuntimeShapeOnly();
        Assert.True(mover.BoundsMin.Y < Fixed64.Zero && mover.BoundsMax.Y > Fixed64.Zero);
        Assert.True(mover.BoundsMin.X < near.BoundsMin.X && mover.BoundsMax.X > near.BoundsMax.X);
        foreach (var member in new[] { near, far })
        {
            var pair = reversed ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
        }
    }

    [Theory]
    [InlineData(-1, false, 0)]
    [InlineData(0, false, 0)]
    [InlineData(1, false, 0)]
    [InlineData(-1, true, 0)]
    [InlineData(0, true, 0)]
    [InlineData(1, true, 0)]
    [InlineData(-1, false, 1)]
    [InlineData(0, false, 1)]
    [InlineData(1, false, 1)]
    [InlineData(-1, true, 1)]
    [InlineData(0, true, 1)]
    [InlineData(1, true, 1)]
    [InlineData(-1, false, 2)]
    [InlineData(0, false, 2)]
    [InlineData(1, false, 2)]
    [InlineData(-1, true, 2)]
    [InlineData(0, true, 2)]
    [InlineData(1, true, 2)]
    public void PartialJoinCut_ShouldRetainItsExposedRimAtRawLatticeOffsets(int rawOffset, bool reversed, int rimAxis)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        // Cyclic coordinate permutations describe the same physical cut with
        // authored rims along every axis, including vertical finite surfaces.
        var left = Rectangle(scene, -4, 0, -4, 4, reversed, rimAxis: rimAxis);
        var right = Rectangle(scene, 0, 4, -2, 2, reversed, rimAxis: rimAxis);
        using var surface = scene.Context.Physics.CreateStaticSurface(reversed ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = new(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(3, 10),
            (Fixed64)2 + Fixed64.Half + Fixed64.FromRaw(rawOffset));
        var body = scene.CreateBody(mover, Permute(start, rimAxis), FixedQuaternion.Identity).Body;
        Vector3d exitNormal = Permute(Vector3d.Right, rimAxis);
        bool rim = false;
        Fixed64 exit = Fixed64.Zero;
        foreach (var member in new[] { left, right })
        {
            var pair = reversed ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) continue;
            bool sourceA = pair.ColliderA == member;
            foreach (var row in pair.Manifold)
            {
                Vector3d normal = sourceA ? row.Normal : -row.Normal;
                if (normal != exitNormal) continue;
                rim = true;
                Assert.Equal(Fixed64.Half - start.X, row.Depth);
                exit = row.Depth;
                ContactAnchor source = sourceA ? row.AnchorA : row.AnchorB;
                Assert.True(source.TryGetWorldPoint(out var point));
                point = Permute(point, rimAxis == 2 ? 2 : 1 - rimAxis);
                Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Y);
                Assert.True(point.Z >= (Fixed64)2, "the covered side of the exact cut cannot supply the exposed rim");
                Assert.True(point.Z <= start.Z + Fixed64.Half);
                Assert.False(row.DepthIsClamped);
            }
        }
        Assert.True(rim);
        // Check complete-shape clearance against this exposed source after
        // disabling continuity, independently of its retained-witness filter.
        surface.Dispose();
        body.SetPosition(Permute(start, rimAxis) + exitNormal * (exit + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(left, mover)));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void JoinedHullSupport_ShouldPreserveDepthAndFiniteAnchorsNearBothWorldLimits(bool maximum, bool reversed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 coordinate = maximum ? Fixed64.MaxValue - (Fixed64)8 : Fixed64.MinValue + (Fixed64)8;
        Vector3d origin = new(coordinate, coordinate, coordinate);
        Assert.True(scene.Context.World.TryAddGrid(new GridConfiguration(origin - Vector3d.One * (Fixed64)6,
            origin + Vector3d.One * (Fixed64)6), out _));
        var left = Rectangle(scene, -4, 0, -4, 4, reversed, origin);
        var right = Rectangle(scene, 0, 4, -4, 4, reversed, origin);
        using var surface = scene.Context.Physics.CreateStaticSurface(reversed ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = origin + new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        int count = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = reversed ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) continue;
            bool sourceA = pair.ColliderA == member;
            foreach (var row in pair.Manifold)
            {
                count++;
                Assert.Equal(Vector3d.Up, sourceA ? row.Normal : -row.Normal);
                Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var point));
                Vector3d local = point - origin;
                Assert.Equal(Fixed64.Zero, local.Y);
                Assert.InRange(local.X.m_rawValue, (-Fixed64.Quarter).m_rawValue, (Fixed64.Half + Fixed64.Quarter).m_rawValue);
                Assert.InRange(local.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            }
        }
        Assert.True(count >= 2);
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UntrustedIncidentOwner_ShouldPreserveTheNativeHullCorner(bool ambiguityAtCorner)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Rectangle(scene, 0, 4, 0, 4);
        var neighbor = Rectangle(scene, -4, 0, 0, 4);
        var overlapping = ambiguityAtCorner
            ? new LSMeshCollider(new[] { new Vector3d(-4, 0, 0), Vector3d.Zero, new(-4, 0, 4) },
                new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            : Rectangle(scene, -4, -1, 2, 4);
        if (ambiguityAtCorner) scene.InitializeStaticCollider(overlapping, Vector3d.Zero);
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, new(-Fixed64.FromFraction(2, 5), Fixed64.Quarter, -Fixed64.FromFraction(2, 5)), FixedQuaternion.Identity);
        var native = NativeDirections(scene, source, mover);
        Assert.Contains(native, row => row.Normal.Y == Fixed64.Zero && (row.Normal.X < Fixed64.Zero || row.Normal.Z < Fixed64.Zero));
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source, neighbor, overlapping });
        var actual = NativeDirections(scene, source, mover);
        foreach (var row in native) Assert.Contains(row, actual);
    }

    [Fact]
    public void FilteredIncidentNeighbor_ShouldPreserveTheNativeHullEndConstraint()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Rectangle(scene, -4, 0, -4, 4);
        var neighbor = Rectangle(scene, 0, 4, -4, 4);
        neighbor.IsTrigger = true;
        var parallel = Rectangle(scene, -4, 0, -4, 4, origin: Vector3d.Up * Fixed64.Quarter);
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, new(Fixed64.Zero, Fixed64.FromFraction(3, 10), (Fixed64)4 + Fixed64.FromFraction(2, 5)), FixedQuaternion.Identity);
        var native = NativeDirections(scene, source, mover);
        Assert.Contains(native, row => row.Normal.Z > Fixed64.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source, neighbor, parallel });
        var actual = NativeDirections(scene, source, mover);
        foreach (var row in native) Assert.Contains(row, actual);
    }

    private static HashSet<(Vector3d Normal, Fixed64 Depth)> NativeDirections(PhysicsScenarioBuilder scene,
        LSMeshCollider source, LSMeshCollider mover)
    {
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source;
        var result = new HashSet<(Vector3d, Fixed64)>();
        foreach (var row in pair.Manifold)
        {
            Assert.True(row.AnchorA.TryGetWorldPoint(out _)); Assert.True(row.AnchorB.TryGetWorldPoint(out _));
            Assert.False(row.DepthIsClamped);
            Assert.True(row.Depth >= Fixed64.Zero);
            result.Add((sourceA ? row.Normal : -row.Normal, row.Depth));
        }
        return result;
    }

    private static LSMeshCollider Rectangle(PhysicsScenarioBuilder scene, int left, int right, int near, int far,
        bool reversed = false, Vector3d origin = default, int rimAxis = 2) =>
        Rectangle(scene, (Fixed64)left, (Fixed64)right, (Fixed64)near, (Fixed64)far, reversed, origin, rimAxis);

    private static LSMeshCollider Rectangle(PhysicsScenarioBuilder scene, Fixed64 left, Fixed64 right, Fixed64 near, Fixed64 far,
        bool reversed = false, Vector3d origin = default, int rimAxis = 2)
    {
        var points = new[] { new Vector3d(left, Fixed64.Zero, near), new(left, Fixed64.Zero, far),
            new(right, Fixed64.Zero, near), new(right, Fixed64.Zero, far) };
        for (int index = 0; index < points.Length; index++) points[index] = Permute(points[index], rimAxis);
        var mesh = new LSMeshCollider(points,
            reversed ? new[] { 0, 2, 1, 2, 3, 1 } : new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, origin); return mesh;
    }

    private static Vector3d Permute(Vector3d point, int rimAxis) => rimAxis switch
    {
        0 => new(point.Z, point.X, point.Y),
        1 => new(point.Y, point.Z, point.X),
        _ => point
    };
}
