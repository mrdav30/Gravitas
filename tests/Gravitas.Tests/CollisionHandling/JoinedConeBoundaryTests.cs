using System.Collections.Generic;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedConeBoundaryTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SharedOuterEndpoint_ShouldHaveOnePhysicalRadialSupplier(bool reverse, bool oppositeAuthoredNormal)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Rectangle(scene, -8, 0, -4, 4);
        var right = oppositeAuthoredNormal
            ? Mesh(scene, new[] { new Vector3d(-8, 0, -4), new(-8, 0, 4), new(0, 0, -4), new(0, 0, 4) },
                new[] { 0, 1, 2, 2, 1, 3 }, rotation: new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero))
            : Rectangle(scene, 0, 8, -4, 4);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)4 + Fixed64.Quarter), FixedQuaternion.Identity);
        int radial = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                Assert.True(row.AnchorB.TryGetWorldPoint(out _));
                if (row.Normal.Z == Fixed64.Zero) continue;
                radial++;
                Assert.Same(left, member);
                Assert.Equal(new Vector3d(0, 0, 4), p);
                Assert.True(row.Normal.Z > Fixed64.Zero);
                Assert.True(row.Depth > Fixed64.Zero);
            }
        }
        Assert.Equal(1, radial);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedApexCorner_ShouldUseTheUnionFanAndOneSupplier(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = Mesh(scene, new[] { Vector3d.Zero, new Vector3d(4, 0, 0), new(4, 0, 4) },
            new[] { 0, 1, 2 }, Vector3d.Up * Fixed64.Half);
        var second = Mesh(scene, new[] { Vector3d.Zero, new Vector3d(4, 0, 4), new(0, 0, 4) },
            new[] { 0, 1, 2 }, Vector3d.Up * Fixed64.Half);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { second, first } : new[] { first, second });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, Vector3d.Zero, FixedQuaternion.Identity);
        int boundary = 0;
        bool radial = false;
        foreach (var member in new[] { first, second })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
            {
                ContactGroup group = pair.Manifold.GetGroup(groupIndex);
                if (group.Key.Region >= 0) continue;
                Assert.Same(first, member);
                for (int index = 0; index < group.Count; index++)
                {
                    ManifoldContact row = group[index];
                    boundary++;
                    Assert.Equal(Fixed64.Zero, row.Depth);
                    Assert.Equal(Vector3d.Up * Fixed64.Half, row.PointA);
                    Assert.Equal(row.PointA, row.PointB);
                    Assert.True(row.Normal.X <= Fixed64.FromRaw(4));
                    Assert.True(row.Normal.Z <= Fixed64.FromRaw(4));
                    Assert.True(row.Normal.Y <= Fixed64.Zero);
                    radial |= row.Normal.X != Fixed64.Zero || row.Normal.Z != Fixed64.Zero;
                }
            }
        }
        Assert.InRange(boundary, 2, ContactManifold.MaxContactsPerGroup);
        Assert.True(radial);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RemoteBridge_ShouldKeepTheTwoJoinedFiniteSectionsIndependent(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var lower = Mesh(scene, new[] { new Vector3d(-2, 0, -2), new(-1, 0, -2), new(-1, 0, 0), new(-2, 0, 0),
            new(1, 0, -2), new(2, 0, -2), new(2, 0, 0), new(1, 0, 0) },
            new[] { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 });
        var upper = Mesh(scene, new[] { new Vector3d(-2, 0, 0), new(-1, 0, 0), new(-1, 0, 1), new(-1, 0, 2), new(-2, 0, 2),
            new(1, 0, 0), new(2, 0, 0), new(2, 0, 2), new(1, 0, 2), new(1, 0, 1) },
            new[] { 0, 1, 2, 0, 2, 4, 2, 3, 4, 5, 6, 9, 6, 7, 9, 7, 8, 9, 2, 9, 8, 2, 8, 3 });
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { upper, lower } : new[] { lower, upper });
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 4, 2) };
        scene.CreateBody(cone, Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized);
        AssertIndependentArmFaces(scene, new[] { lower, upper }, cone);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoiningOutsideTheCone_ShouldNotMergeItsIndependentContactRegions(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        // Each L-shaped member has one admitted arm. Their common bridge edge
        // at X=0, Z>=1 lies beyond the cone section, so it cannot join contacts.
        Vector3d[] vertices = { new(-2, 0, -2), new(-1, 0, -2), new(-1, 0, 1), new(-2, 0, 1),
            new(-1, 0, 2), new(-2, 0, 2), new(0, 0, 1), new(0, 0, 2) };
        int[] triangles = { 0, 1, 2, 0, 2, 3, 3, 2, 4, 3, 4, 5, 2, 6, 7, 2, 7, 4 };
        var first = Mesh(scene, vertices, triangles);
        var second = Mesh(scene, vertices, triangles,
            rotation: new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { second, first } : new[] { first, second });
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 4, 2) };
        scene.CreateBody(cone, Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized);
        AssertIndependentArmFaces(scene, new[] { first, second }, cone);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OppositeAuthoredPlaneNormals_ShouldRetainOneCompleteObliqueConeExit(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = Rectangle(scene, -8, 0, -4, 4);
        var second = Mesh(scene, new[] { new Vector3d(-8, 0, -4), new(-8, 0, 4), new(0, 0, -4), new(0, 0, 4) },
            new[] { 0, 1, 2, 2, 1, 3 }, rotation: new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        first.Material = PhysicsMaterial.Frictionless; second.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { second, first } : new[] { first, second });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(-Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)));
        int rows = 0;
        Fixed64 maximum = Fixed64.Zero;
        foreach (var member in new[] { first, second })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                rows++;
                Assert.Equal(Vector3d.Up, row.Normal);
                Assert.Equal(member.Material, row.MaterialA);
                // The final surface witness uses the translated physical chart;
                // supplier identity and material remain independent of that chart.
                Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                Assert.InRange(p.Y.m_rawValue, -1, 1);
                maximum = FixedMath.Max(maximum, row.Depth);
            }
        }
        Assert.InRange(rows, 1, ContactManifold.MaxContactsPerGroup);
        Assert.InRange(maximum.m_rawValue, Fixed64.FromFraction(37, 100).m_rawValue - 16, Fixed64.FromFraction(37, 100).m_rawValue + 16);
        AssertClear(new[] { first, second }, cone, cone.Center + Vector3d.Up * (maximum + Fixed64.FromRaw(32)));
    }

    [Fact]
    public void MultiPlaneMember_ShouldRetainTheFloorAndWallConstraints()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var corner = Mesh(scene, new[] { new Vector3d(-4, 0, -4), new(-4, 0, 4), new(0, 0, -4), new(0, 0, 4),
            new(0, 4, -4), new(0, 4, 4) }, new[] { 0, 1, 2, 2, 1, 3, 2, 3, 4, 4, 3, 5 });
        var neighbor = Rectangle(scene, 0, 4, -4, 4);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { neighbor, corner });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(-Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        Fixed64 up = Fixed64.Zero, left = Fixed64.Zero;
        foreach (var member in new[] { corner, neighbor })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out _));
                Assert.True(row.AnchorB.TryGetWorldPoint(out _));
                if (row.Normal == Vector3d.Up) up = FixedMath.Max(up, row.Depth);
                if (row.Normal == -Vector3d.Right) left = FixedMath.Max(left, row.Depth);
            }
        }
        Assert.True(up > Fixed64.Zero && left > Fixed64.Zero);
        // These are independent finite-surface exits from the original pose.
        // Adding the upward exit exposes a wider cone section to the wall,
        // so their vector sum is not a simultaneous minimum translation.
        AssertClear(new[] { neighbor }, cone, cone.Center + Vector3d.Up * (up + Fixed64.FromRaw(32)));
        for (int triangle = 0; triangle < corner.Mesh.TriangleCount; triangle++)
        {
            Vector3d offset = triangle < 2 ? Vector3d.Up * (up + Fixed64.FromRaw(32))
                : -Vector3d.Right * (left + Fixed64.FromRaw(32));
            Assert.True(corner.Mesh.TryGetTriangleVertices(triangle, out var a, out var b, out var c));
            Assert.False(new FixedTriangle(a, b, c).TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                cone.Center + offset, cone.Rotation, cone.Height, cone.ScaledRadius, out _));
        }
    }

    private static void AssertIndependentArmFaces(PhysicsScenarioBuilder scene, LSMeshCollider[] members, LSConeCollider cone)
    {
        var regions = new HashSet<int>();
        bool left = false, right = false;
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
            {
                ContactGroup group = pair.Manifold.GetGroup(groupIndex);
                if (group.Key.Region < 0) continue;
                regions.Add(group.Key.Region);
                for (int index = 0; index < group.Count; index++)
                {
                    ManifoldContact row = group[index];
                    Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                    Assert.True(row.AnchorB.TryGetWorldPoint(out _));
                    Assert.Equal(Fixed64.Zero, p.Y);
                    Assert.True(FixedMath.Abs(p.X) >= Fixed64.One);
                    Assert.True(FixedMath.Abs(p.Z) <= Fixed64.One);
                    Assert.Equal(Fixed64.One, FixedMath.Abs(row.Normal.Y));
                    left |= p.X < Fixed64.Zero;
                    right |= p.X > Fixed64.Zero;
                }
            }
        }
        Assert.Equal(2, regions.Count);
        Assert.True(left && right);
    }

    [Fact]
    public void FractionallyScaledJoinedFaces_ShouldRetainOneCompletePhysicalConeExit()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        var first = Mesh(scene, new[] { new Vector3d(-4,0,-4), new(-4,0,4), new(0,0,-4), new(0,0,4) },
            new[] { 0,1,2,2,1,3 }, scale: scale);
        var second = Mesh(scene, new[] { new Vector3d(0,0,-4), new(0,0,4), new(5,0,-4), new(5,0,4) },
            new[] { 0,1,2,2,1,3 }, scale: scale);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { second, first });
        var cone = new LSConeCollider();
        Vector3d center = Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(cone, center, FixedQuaternion.Identity).Body;
        int count = 0; Fixed64 maximum = Fixed64.Zero;
        foreach (var member in new[] { first, second })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                count++;
                Assert.Equal(Vector3d.Up, row.Normal);
                Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                Assert.InRange(point.Y.m_rawValue, -1L, 1L);
                maximum = FixedMath.Max(maximum, row.Depth);
            }
        }
        Assert.InRange(count, 1, ContactManifold.MaxContactsPerGroup);
        Assert.Equal(Fixed64.Quarter, maximum);
        body.SetPosition(center + Vector3d.Up * (maximum + Fixed64.FromRaw(32)));
        cone.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { first, second })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, cone)));
    }

    private static void AssertClear(LSMeshCollider[] members, LSConeCollider cone, Vector3d outside)
    {
        foreach (var member in members)
            for (int triangle = 0; triangle < member.Mesh.TriangleCount; triangle++)
            {
                Assert.True(member.Mesh.TryGetTriangleVertices(triangle, out var a, out var b, out var c));
                Assert.False(new FixedTriangle(a, b, c).TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                    outside, cone.Rotation, cone.Height, cone.ScaledRadius, out _));
            }
    }

    private static LSMeshCollider Rectangle(PhysicsScenarioBuilder scene, int left, int right, int near, int far) =>
        Mesh(scene, new[] { new Vector3d(left, 0, near), new(left, 0, far), new(right, 0, near), new(right, 0, far) },
            new[] { 0, 1, 2, 2, 1, 3 });

    private static LSMeshCollider Mesh(PhysicsScenarioBuilder scene, Vector3d[] vertices, int[] triangles,
        Vector3d position = default, FixedQuaternion? rotation = null, Vector3d? scale = null)
    {
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(position, rotation ?? FixedQuaternion.Identity, scale ?? Vector3d.One)));
        return mesh;
    }
}
