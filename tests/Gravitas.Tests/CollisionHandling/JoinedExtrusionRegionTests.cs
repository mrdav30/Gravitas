using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedExtrusionRegionTests
{
    [Fact]
    public void DuplicateFilledOwners_ShouldRetainTheirIndependentFiniteBoxRegions()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var members = new[] { Patch(false, false), Patch(false, false) };
        foreach (var member in members) scene.InitializeStaticCollider(member, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var box = new LSCuboidCollider { Size = new(2, 1, 2) };
        scene.CreateBody(box, new Vector3d(Fixed64.Half, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);

        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, box);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            var regions = new HashSet<int>();
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                var group = pair.Manifold.GetGroup(index);
                if (group.Key.Region >= 0) regions.Add(group.Key.Region);
                for (int rowIndex = 0; rowIndex < group.Count; rowIndex++)
                {
                    var row = group[rowIndex];
                    Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
                    Assert.Equal(Fixed64.Zero, point.Y);
                    Assert.True(point.X <= -Fixed64.One || point.Z.Abs() >= Fixed64.Half);
                    Assert.True(row.AnchorB.TryGetWorldPoint(out _));
                }
            }
            Assert.Equal(2, regions.Count);
        }
        Assert.Equal(4, surface.SlabFaces.RegionCount);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void RemoteSpines_ShouldKeepSeparateSupportsUntilAnAdmittedNeighborBridgesThem(
        bool cylinder, bool bridge, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        // Both U spines are outside the mover footprint. The common edges
        // join its lower and upper arms separately; only a filled right side
        // provides a connection through the actually admitted support domain.
        var left = Patch(false, false);
        var right = Patch(true, bridge);
        left.Material = new(Fixed64.One, Fixed64.Half, Fixed64.Quarter);
        right.Material = new(Fixed64.Half, Fixed64.Quarter, Fixed64.Half);
        var members = reverse ? new[] { right, left } : new[] { left, right };
        foreach (var member in members) scene.InitializeStaticCollider(member, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        LSCollider shape = cylinder
            ? new LSCylinderCollider { Radius = Fixed64.One, Size = new(2, 1, 2), Material = PhysicsMaterial.Frictionless }
            : new LSCuboidCollider { Size = new(2, 1, 2), Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(shape, new Vector3d(Fixed64.Half, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);

        var initial = Inspect();
        surface.ReplaceMembers(reverse ? new[] { left, right } : new[] { right, left });
        Assert.Equal(initial, Inspect());

        List<(int Region, int Supplier, Vector3d Source, Vector3d Exit)> Inspect()
        {
            var retained = new List<(int Region, int Supplier, Vector3d Source, Vector3d Exit)>();
            var regions = new HashSet<int>();
            foreach (var member in members)
            {
                var pair = scene.CreatePair(member, shape);
                Assert.True(CollisionDetection.DoCollisionCheck(pair));
                for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
                {
                    var group = pair.Manifold.GetGroup(groupIndex);
                    if (group.Key.Region < 0) continue;
                    bool fromLeft = group.Key.SourceIdA == left.Id + 1;
                    Assert.True(fromLeft || group.Key.SourceIdA == right.Id + 1);
                    regions.Add(group.Key.Region);
                    for (int index = 0; index < group.Count; index++)
                    {
                        var row = group[index];
                        Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
                        Assert.False(row.DepthIsClamped);
                        Assert.Equal(fromLeft ? left.Material : right.Material, row.MaterialA);
                        Assert.Equal(shape.Material, row.MaterialB);
                        Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
                        Assert.True(row.AnchorB.TryGetWorldPoint(out var exit));
                        Assert.Equal(Fixed64.Zero, source.Y);
                        Assert.InRange(source.X.m_rawValue, (-Fixed64.Half).m_rawValue, (Fixed64.One + Fixed64.Half).m_rawValue);
                        Assert.InRange(source.Z.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
                        Assert.True(fromLeft ? source.X <= Fixed64.One : source.X >= Fixed64.One);
                        if (!bridge || fromLeft) Assert.True(source.Z.Abs() >= Fixed64.Half);
                        Assert.Equal(source.X, exit.X); Assert.Equal(source.Z, exit.Z);
                        Assert.Equal(-Fixed64.Quarter, exit.Y);
                        retained.Add((group.Key.Region, group.Key.SourceIdA, source, exit));
                    }
                }
            }
            Assert.Equal(bridge ? 1 : 2, surface.SlabFaces.RegionCount);
            Assert.Equal(bridge ? 1 : 2, regions.Count);
            Assert.NotEmpty(retained);
            return retained;
        }
    }

    private static LSMeshCollider Patch(bool right, bool bridge)
    {
        Fixed64[] x = right ? new Fixed64[] { Fixed64.One, Fixed64.Two, (Fixed64)3 } : new Fixed64[] { (Fixed64)(-2), -Fixed64.One, Fixed64.One };
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, Fixed64.Two };
        var vertices = new Vector3d[x.Length * z.Length];
        var triangles = new List<int>();
        for (int row = 0; row < z.Length; row++)
            for (int column = 0; column < x.Length; column++)
            {
                int vertex = row * x.Length + column;
                vertices[vertex] = new(x[column], Fixed64.Zero, z[row]);
                if (row == z.Length - 1 || column == x.Length - 1) continue;
                if (row == 1 && !bridge && column == (right ? 0 : 1)) continue;
                triangles.AddRange(new[] { vertex, vertex + x.Length, vertex + 1,
                    vertex + 1, vertex + x.Length, vertex + x.Length + 1 });
            }
        return new(vertices, triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
