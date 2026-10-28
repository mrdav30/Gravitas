using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks native moving mesh fill, independent parts and authored provenance.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveMovingMeshMissingArea_ShouldRemainEmptyAcrossPoseChanges(bool ring)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = ring
            ? new[] { Rectangle(scene, -1, 0, -1, 1, Fixed64.One), Rectangle(scene, 0, 1, -1, 1, Fixed64.One) }
            : new[] { Rectangle(scene, 1, 2, 1, 3, Fixed64.One), Rectangle(scene, 2, 3, 1, 3, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSMeshCollider mover = MissingCellMesh(ring);
        scene.CreateBody(mover, Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity);
        var ordinary = new ContactManifold();
        for (int iteration = 0; iteration < 4; iteration++)
        {
            bool filled = (iteration & 1) != 0;
            SetPosition(mover, new(filled ? (Fixed64)4 : Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
            foreach (LSCollider2D member in members)
            {
                Assert.True(mover.Bounds.Intersects(member.MixedBounds3D));
                Assert.Equal(filled, CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary));
                Assert.Equal(filled, CollisionDetectionMixed.TryBuildDiscreteManifold(mover, member, ordinary));
            }
            if (filled) Assert.NotEmpty(Rows(mover, members));
            else Assert.Empty(Rows(mover, members));
        }
        // This pose clears every finite triangle; no local row is assumed to
        // encode a global escape from a genuinely nonconvex sheet.
        AssertWholeShapeExit(mover, members, new(4, 1, 0), Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DisconnectedMovingMeshIslands_ShouldKeepSeparateRegionsAndFiniteSuppliers(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -4, 4, Fixed64.Two);
        var right = Rectangle(scene, 0, 2, -4, 4, Fixed64.Two);
        left.Material = PhysicsMaterial.Default; right.Material = PhysicsMaterial.Bouncy;
        var members = reverse ? new[] { right, left } : new[] { left, right };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 h = Fixed64.Half;
        var mover = new LSMeshCollider(new[]
        {
            new Vector3d(-h, Fixed64.Zero, -5 * h), new(h, Fixed64.Zero, -5 * h),
            new(-h, Fixed64.Zero, -3 * h), new(h, Fixed64.Zero, -3 * h),
            new(-h, Fixed64.Zero, 3 * h), new(h, Fixed64.Zero, 3 * h),
            new(-h, Fixed64.Zero, 5 * h), new(h, Fixed64.Zero, 5 * h)
        }, new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.CreateBody(mover, new(Fixed64.Quarter, 3 * h, Fixed64.Zero), FixedQuaternion.Identity);
        var surfaces = new HashSet<int>(); var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(h, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(3 * h, moving.Y); Assert.Equal(Fixed64.Two, point.Y);
            Assert.True(point.Z.Abs() >= 3 * h && point.Z.Abs() <= 5 * h);
            LSCollider2D supplier = sample.Supplier == left.Id + 1 ? left : right;
            Assert.Equal(supplier.Id + 1, sample.Supplier);
            Assert.Equal(supplier.Material, sample.Row.MaterialB);
            Assert.True(supplier.ContainsPoint(new(point.X, point.Z)));
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        var manifold = new ContactManifold();
        foreach (LSCollider2D member in members)
        {
            CollisionDetectionMixed.TryBuildDiscreteManifold(mover, member, manifold);
            for (int group = 0; group < manifold.GroupCount; group++) surfaces.Add(manifold.GetGroup(group).Key.SurfaceA);
        }
        Assert.Equal(2, surfaces.Count); Assert.Equal(2, regions.Count);
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
        AssertWholeShapeExit(mover, members, new(Fixed64.Quarter, Fixed64.Two, Fixed64.Zero), Vector3d.Up);
    }

    [Fact]
    public void AuthoredOffsetOpenMesh_ShouldRetainFiniteExitsAcrossGeometryChartChanges()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 4, -2, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = (Fixed64)100 + Fixed64.MinIncrement, z = (Fixed64)(-200);
        var mover = new LSMeshCollider(new[] { new Vector3d(x - Fixed64.One, Fixed64.Zero, z - Fixed64.Quarter),
            new(x + Fixed64.One, Fixed64.Zero, z - Fixed64.Quarter), new(x - Fixed64.One, Fixed64.Zero, z + Fixed64.Quarter),
            new(x + Fixed64.One, Fixed64.Zero, z + Fixed64.Quarter) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d host = new((Fixed64)(-100) + Fixed64.Quarter, 3 * Fixed64.Half, (Fixed64)200);
        var agent = new TestMatterAgent(scene.Context, new FixedTransform(host, FixedQuaternion.Identity, Vector3d.One));
        new SolidBody(agent, mover) { Mass = Fixed64.One }.Initialize(host, FixedQuaternion.Identity);
        // The odd raw authored offset times 3/2 cannot be represented by one
        // rounded centered chart. Switch back as well to exercise cache refresh.
        foreach (Vector3d scale in new[] { Vector3d.One, new(3 * Fixed64.Half, Fixed64.One, Fixed64.Two), Vector3d.One })
        {
            agent.Transform.LocalScale = scale;
            host = new(-100 * scale.X + Fixed64.Quarter, 3 * Fixed64.Half, 200 * scale.Z);
            SetPosition(mover, host);
            var rows = Rows(mover, members); Assert.NotEmpty(rows);
            foreach (var sample in rows)
            {
                Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Half, sample.Row.Depth);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                Assert.Equal(3 * Fixed64.Half, moving.Y); Assert.Equal(Fixed64.Two, source.Y);
                Assert.InRange(source.X.m_rawValue, (Fixed64.Quarter - scale.X).m_rawValue,
                    (Fixed64.Quarter + scale.X).m_rawValue + 2);
                LSCollider2D supplier = sample.Supplier == members[0].Id + 1 ? members[0] : members[1];
                Assert.Equal(supplier.Id + 1, sample.Supplier);
                Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
            }
            AssertWholeShapeExit(mover, members, new(host.X, Fixed64.Two, host.Z), Vector3d.Up);
        }
    }

    [Fact]
    public void DeepWideJoinedUnion_ShouldGrowTheOpenMeshSupportSearchBeforeSelectingItsExit()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 2, -10, 10, (Fixed64)10),
            Rectangle(scene, 2, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 h = Fixed64.Half;
        var mover = new LSMeshCollider(new[] { new Vector3d(-h, Fixed64.Zero, -h), new(h, Fixed64.Zero, -h),
            new(-h, Fixed64.Zero, h), new(h, Fixed64.Zero, h) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal((Fixed64)10, sample.Row.Depth);
            Assert.Equal(Fixed64.One, sample.Row.Normal.Y.Abs());
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.X); Assert.Equal(Fixed64.Zero, sample.Row.Normal.Z);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, moving.Y); Assert.Equal(-10 * sample.Row.Normal.Y, source.Y);
            LSCollider2D supplier = sample.Supplier == members[0].Id + 1 ? members[0] : members[1];
            Assert.Equal(supplier.Id + 1, sample.Supplier);
            Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
        }
        AssertWholeShapeExit(mover, members, Vector3d.Up * (Fixed64)10, Vector3d.Up);
        AssertWholeShapeExit(mover, members, Vector3d.Down * (Fixed64)10, Vector3d.Down);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpenMeshCompoundLeaf_ShouldKeepNamespaceMaterialAndActualSupplier(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        var right = Rectangle(scene, 0, 2, -2, 2, Fixed64.Two);
        var members = reverse ? new[] { right, left } : new[] { left, right };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 h = Fixed64.Half;
        var shape = ColliderShapeDefinition.ConvexMesh(new[] { new Vector3d(-h, Fixed64.Zero, -h),
            new(h, Fixed64.Zero, -h), new(-h, Fixed64.Zero, h), new(h, Fixed64.Zero, h) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshInertiaPolicy.SurfaceApproximation, PhysicsMaterial.Bouncy);
        var mover = new LSCompoundCollider(new CompoundColliderPart(shape, Vector3d.Right, FixedQuaternion.Identity, Vector3d.One),
            CompoundColliderPart.Sphere(h, new(-1, 4, 4)));
        SolidBody body = scene.CreateBody(mover, new(-3 * Fixed64.Quarter, 3 * h, Fixed64.Zero), FixedQuaternion.Identity).Body;
        Assert.NotEqual(mover.GetPartCollider(0).Center, body.WorldCenterOfMass);
        for (int revision = 0; revision < 2; revision++)
        {
            if (revision == 1) left.Material = PhysicsMaterial.Bouncy;
            var rows = Rows(mover, members); Assert.NotEmpty(rows);
            foreach (var sample in rows)
            {
                Assert.Equal(1, sample.Row.FeatureNamespaceA);
                Assert.Equal(PhysicsMaterial.Bouncy, sample.Row.MaterialA);
                Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(h, sample.Row.Depth);
                LSCollider2D supplier = sample.Supplier == left.Id + 1 ? left : right;
                Assert.Equal(supplier.Id + 1, sample.Supplier); Assert.Equal(supplier.Material, sample.Row.MaterialB);
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
                Assert.True(supplier.ContainsPoint(new(point.X, point.Z))); Assert.Equal(Fixed64.Two, point.Y);
            }
        }
        AssertWholeShapeExit(mover, members, new(-3 * Fixed64.Quarter, Fixed64.Two, Fixed64.Zero), Vector3d.Up);
    }

    private static LSMeshCollider MissingCellMesh(bool ring)
    {
        int[] coordinates = ring ? new[] { -4, -2, 2, 4 } : new[] { -4, 0, 4 };
        int count = coordinates.Length;
        var vertices = new Vector3d[count * count]; var triangles = new List<int>();
        for (int row = 0; row < count; row++)
            for (int column = 0; column < count; column++) vertices[row * count + column] = new(coordinates[column], 0, coordinates[row]);
        for (int row = 0; row < count - 1; row++)
            for (int column = 0; column < count - 1; column++)
            {
                if (row == 1 && column == 1) continue;
                int a = row * count + column, b = a + 1, c = a + count, d = c + 1;
                triangles.Add(a); triangles.Add(c); triangles.Add(b); triangles.Add(b); triangles.Add(c); triangles.Add(d);
            }
        // Omitting the L's corner cell leaves its outer corner unused; submit
        // only referenced vertices while preserving the authored grid order.
        var referenced = new bool[vertices.Length];
        foreach (int index in triangles) referenced[index] = true;
        var remap = new int[vertices.Length]; var compact = new List<Vector3d>();
        for (int index = 0; index < vertices.Length; index++)
            if (referenced[index]) { remap[index] = compact.Count; compact.Add(vertices[index]); }
        for (int index = 0; index < triangles.Count; index++) triangles[index] = remap[triangles[index]];
        return new LSMeshCollider(compact.ToArray(), triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
