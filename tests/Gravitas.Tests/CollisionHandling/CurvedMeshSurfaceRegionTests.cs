using System;
using System.Collections.Generic;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CurvedMeshSurfaceRegionTests
{
    [Fact]
    public void WarmedSectionChurn_ShouldReuseTheSameContextScratchWithoutAllocations()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider[] meshes = { CreateComb(false, false), CreateComb(true, true),
            new(new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
                new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) },
                new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave) };
        LSCollider[] shapes = { new LSSphereCollider { Radius = Fixed64.One },
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new(2, 3, 2) } };
        foreach (LSMeshCollider mesh in meshes) scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        foreach (LSCollider shape in shapes) scenario.InitializeStaticCollider(shape, Vector3d.Up * Fixed64.Quarter);
        var pairs = new CollisionPair[meshes.Length * shapes.Length];
        for (int mesh = 0; mesh < meshes.Length; mesh++)
        for (int shape = 0; shape < shapes.Length; shape++)
            pairs[mesh * shapes.Length + shape] = scenario.CreatePair(meshes[mesh], shapes[shape]);
        for (int warmup = 0; warmup < 8; warmup++)
            foreach (CollisionPair pair in pairs) Assert.True(CollisionDetection.DoCollisionCheck(pair));

        bool allHit = true;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 64; iteration++)
            foreach (CollisionPair pair in pairs) allHit &= CollisionDetection.DoCollisionCheck(pair);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.True(allHit);
        Assert.Equal(0L, allocated);
        for (int index = 0; index < pairs.Length; index++)
        {
            int groups = 0;
            for (int group = 0; group < pairs[index].Manifold.GroupCount; group++)
                if (pairs[index].Manifold.GetGroup(group).Key.Region >= 0) groups++;
            Assert.Equal(index < 4 ? 5 : 1, groups);
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, false, false)]
    [InlineData(true, true, true)]
    public void ConnectedComb_WithFiveDisconnectedSections_ShouldRetainEveryTab(
        bool capsule, bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateComb(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        int owner = mesh.Mesh.GetManifoldSurfaceOwner(0);
        for (int triangle = 1; triangle < mesh.Mesh.TriangleCount; triangle++)
            mesh.Mesh.GetManifoldSurfaceOwner(triangle).Should().Be(owner,
                "the entire comb is one trusted connected authored surface");
        LSCollider shape = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.One, Size = new(2, 3, 2) }
            : new LSSphereCollider { Radius = Fixed64.One };
        scenario.InitializeStaticCollider(shape, Vector3d.Up * Fixed64.Quarter);
        CollisionPair pair = scenario.CreatePair(mesh, shape);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        int faceGroups = 0;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup contacts = ref pair.Manifold.GetGroup(group);
            if (contacts.Key.Region < 0) continue;
            contacts.Key.Region.Should().Be(faceGroups);
            contacts.Count.Should().BeInRange(1, ContactManifold.MaxContactsPerGroup);
            Fixed64 minimumX = Fixed64.FromFraction(6 * faceGroups - 13, 16);
            Fixed64 maximumX = minimumX + Fixed64.FromFraction(1, 8);
            Vector3d closest = new(FixedMath.Clamp(Fixed64.Zero, minimumX, maximumX),
                Fixed64.Zero, Fixed64.FromFraction(3, 8));
            // These dyadic coordinates make the squared radicand exact in
            // Q32.32. Only its square root rounds; the plane offset and core
            // half-length are exact translations of that rounded result.
            Fixed64 expectedDepth = FixedMath.Sqrt(Fixed64.One - closest.X * closest.X
                - closest.Z * closest.Z) - Fixed64.Quarter + (capsule ? Fixed64.Half : Fixed64.Zero);
            int start = pair.Manifold.GetGroupStartIndex(group);
            int maxima = 0;
            for (int sample = 0; sample < contacts.Count; sample++)
            {
                ManifoldContact contact = pair.Manifold[start + sample];
                contact.Normal.Should().Be(Vector3d.Up);
                contact.DepthIsClamped.Should().BeFalse();
                contact.Depth.Should().BeLessThanOrEqualTo(expectedDepth);
                if (contact.Depth != expectedDepth) continue;
                contact.PointA.Should().Be(closest);
                contact.PointB.Should().Be(closest - Vector3d.Up * expectedDepth);
                maxima++;
            }
            maxima.Should().Be(1, "each tab has one nearest finite point and therefore one largest face ray");
            faceGroups++;
        }
        faceGroups.Should().Be(5, "each disconnected admitted tab owns an independent finite face sample cap");
        for (int tab = 0; tab < 5; tab++)
        {
            Fixed64 minimumX = Fixed64.FromFraction(6 * tab - 13, 16);
            Fixed64 maximumX = minimumX + Fixed64.FromFraction(1, 8);
            bool found = false;
            for (int contact = 0; contact < pair.Manifold.Count; contact++)
            {
                Vector3d point = pair.Manifold[contact].PointA;
                found |= point.X >= minimumX && point.X <= maximumX
                    && point.Z >= Fixed64.FromFraction(3, 8) && point.Z < Fixed64.One;
            }
            found.Should().BeTrue($"tab {tab} is an independent admitted region, despite its distant shared backbone");
        }
    }

    [Fact]
    public void JoiningOneCombTab_ShouldRetainAllFiveDisconnectedFiniteSections()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var comb = CreateComb(false, true);
        Fixed64 low = Fixed64.FromFraction(-1, 16), high = Fixed64.FromFraction(1, 16), far = Fixed64.FromFraction(3, 8);
        var neighbor = new LSMeshCollider(new[] { new Vector3d(low, Fixed64.Zero, Fixed64.Zero), new(high, Fixed64.Zero, Fixed64.Zero),
            new(low, Fixed64.Zero, far), new(high, Fixed64.Zero, far) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(comb, Vector3d.Zero);
        scenario.InitializeStaticCollider(neighbor, Vector3d.Zero);
        using var surface = scenario.Context.Physics.CreateStaticSurface(new[] { neighbor, comb });
        var sphere = new LSSphereCollider { Radius = Fixed64.One };
        scenario.CreateBody(sphere, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        var regions = new HashSet<int>();
        foreach (var member in new[] { comb, neighbor })
        {
            var pair = scenario.CreatePair(member, sphere);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
                if (pair.Manifold.GetGroup(group).Key.Region >= 0) regions.Add(pair.Manifold.GetGroup(group).Key.Region);
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var a));
                Assert.Equal(Fixed64.Zero, a.Y);
                Assert.True(row.Depth >= Fixed64.Zero);
            }
        }
        Assert.Equal(5, regions.Count);
    }

    private static LSMeshCollider CreateComb(bool reverse, bool subdivide)
    {
        int[] numerators = { -13, -11, -7, -5, -1, 1, 5, 7, 11, 13 };
        var vertices = new List<Vector3d>();
        var triangles = new List<int>();
        for (int row = 0; row < 2; row++)
            foreach (int numerator in numerators)
                vertices.Add(new(Fixed64.FromFraction(numerator, 16), Fixed64.Zero,
                    (Fixed64)2 + row * Fixed64.Quarter));
        for (int column = 0; column < 9; column++)
            AddQuad(column, column + 1, column + 10, column + 11);
        for (int tab = 0; tab < 5; tab++)
        {
            int first = vertices.Count;
            vertices.Add(new(Fixed64.FromFraction(numerators[2 * tab], 16), Fixed64.Zero,
                Fixed64.FromFraction(3, 8)));
            vertices.Add(new(Fixed64.FromFraction(numerators[2 * tab + 1], 16), Fixed64.Zero,
                Fixed64.FromFraction(3, 8)));
            AddQuad(first, first + 1, 2 * tab, 2 * tab + 1);
        }
        if (reverse)
        {
            for (int index = 0; index < triangles.Count; index += 3)
                (triangles[index], triangles[index + 2]) = (triangles[index + 2], triangles[index]);
            // Reverse discovery order independently of the coherent winding.
            for (int first = 0, last = triangles.Count - 3; first < last; first += 3, last -= 3)
                for (int corner = 0; corner < 3; corner++)
                    (triangles[first + corner], triangles[last + corner]) = (triangles[last + corner], triangles[first + corner]);
        }
        return new(vertices.ToArray(), triangles.ToArray(), MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

        void AddQuad(int a, int b, int c, int d)
        {
            AddTriangle(a, c, b);
            AddTriangle(b, c, d);
        }

        void AddTriangle(int a, int b, int c)
        {
            if (!subdivide)
            {
                triangles.AddRange(new[] { a, b, c });
                return;
            }
            int center = vertices.Count;
            vertices.Add((vertices[a] + vertices[b] + vertices[c]) / 3);
            triangles.AddRange(new[] { a, b, center, b, c, center, c, a, center });
        }
    }
}
