using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class MeshCapsuleSurfaceBoundaryTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void MixedDirectionCoreAtConvexCorner_ShouldUseAnAdmittedActiveFanWall(
        bool reverseCore, bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(1, 1, -1).Normalized);
        if (reverseCore)
            rotation *= new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        Vector3d corner = new(-2, 0, -2);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.CreateBody(capsule, corner, rotation, immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundCorner = false;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup contacts = ref pair.Manifold.GetGroup(group);
            if (contacts.Key.Region >= 0) continue;
            for (int index = 0; index < contacts.Count; index++)
            {
                ManifoldContact contact = contacts[index];
                if (contact.PointA != corner) continue;
                foundCorner = true;
                // The projected face direction enters +X or +Z. An active fan
                // wall supplies the admissible perpendicular corner direction.
                // Reversing the core leaves the shape unchanged and swaps the
                // signs of its wall-cross candidates.
                contact.Normal.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
                contact.Normal.Z.Should().BeLessThanOrEqualTo(Fixed64.Zero);
                (contact.Normal.X == Fixed64.Zero || contact.Normal.Z == Fixed64.Zero).Should().BeTrue();
                Vector3d localNormal = rotation.Inverse().Rotate(contact.Normal);
                localNormal.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
                (contact.Depth - Fixed64.Half).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
                Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Magnitude
                    .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
                Vector3d localPoint = rotation.Inverse().Rotate(contact.PointB - corner);
                (localPoint.X * localPoint.X + localPoint.Z * localPoint.Z - Fixed64.Quarter).Abs()
                    .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64), "the paired ray ends on the cylindrical boundary");
            }
        }
        foundCorner.Should().BeTrue("a convex corner retains its independent admitted radial constraint");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ReflexCornerCoreCrossing_ShouldRetainTheSelectedFaceRay(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new System.Collections.Generic.List<Vector3d>
        {
            new(-2,0,-2), new(0,0,-2), new(2,0,-2), new(-2,0,0),
            Vector3d.Zero, new(2,0,0), new(-2,0,2), new(0,0,2)
        };
        int[] authored = { 0,3,1, 1,3,4, 1,4,2, 2,4,5, 3,6,4, 4,6,7 };
        var triangles = new System.Collections.Generic.List<int>();
        for (int index = 0; index < authored.Length; index += 3)
        {
            int a = authored[index], b = authored[index + 1], c = authored[index + 2];
            if (reverse) (a, c) = (c, a);
            if (!subdivide) triangles.AddRange(new[] { a, b, c });
            else
            {
                int middle = vertices.Count;
                vertices.Add((vertices[a] + vertices[b] + vertices[c]) / 3);
                triangles.AddRange(new[] { a,b,middle, b,c,middle, c,a,middle });
            }
        }
        var mesh = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(),
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1,2,1) };
        scenario.InitializeStaticCollider(capsule, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundCorner = false;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup contacts = ref pair.Manifold.GetGroup(group);
            if (contacts.Key.Region >= 0) continue;
            for (int index = 0; index < contacts.Count; index++)
            {
                ManifoldContact contact = contacts[index];
                if (contact.PointA != Vector3d.Zero) continue;
                foundCorner = true;
                contact.Normal.Should().Be(Vector3d.Down,
                    "a reflex filled-sector fan admits only plane-normal support, with the region's selected hemisphere");
                contact.Depth.Should().Be(Fixed64.One, "the axial ray exits beyond the complete core and cap");
                contact.PointB.Should().Be(Vector3d.Up);
                Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Should().Be(Vector3d.Zero);
            }
        }
        foundCorner.Should().BeTrue("the genuine reflex corner retains its own constraint even when no perpendicular core ray is admitted");
        var separated = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1,2,1) };
        scenario.InitializeStaticCollider(separated, Vector3d.Down * (Fixed64.One + Fixed64.FromRaw(1)));
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, separated)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AngledCoreCrossingTrueEdge_ShouldPairItsAdmittedPerpendicularRay(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(-1, 1, 1).Normalized);
        Vector3d center = new(-2, 0, 0);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.CreateBody(capsule, center, rotation, immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundEdge = false;
        Vector3d exit = default;
        for (int index = 0; index < pair.Manifold.Count; index++)
        {
            ManifoldContact contact = pair.Manifold[index];
            if (contact.Normal.X >= Fixed64.Zero) continue;
            foundEdge = true; exit = contact.Normal;
            contact.Normal.Y.Should().BeLessThan(Fixed64.Zero, "the perimeter ray must keep the region's selected hemisphere");
            contact.Normal.Z.Should().Be(Fixed64.Zero, "an interior edge contact is perpendicular to its tangent");
            (contact.Depth - Fixed64.Half).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Magnitude
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            Vector3d offset = rotation.Inverse().Rotate(contact.PointB - center);
            (offset.X * offset.X + offset.Z * offset.Z - Fixed64.Quarter).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64), "the paired point must be on the cylindrical boundary");
        }
        foundEdge.Should().BeTrue();
        var separated = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.CreateBody(separated, center + exit * (Fixed64.Half + Fixed64.FromRaw(256)), rotation, immovable: true);
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, separated)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void OutwardTiltedCoreAtTrueEdge_ShouldPreserveTheProjectedFaceExit(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Two).Normalized;
        Vector3d center = new(-2, 0, 0);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.CreateBody(capsule, center, rotation, immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundEdge = false;
        foreach (ManifoldContact contact in pair.Manifold)
        {
            if (contact.Normal.X >= Fixed64.Zero) continue;
            foundEdge = true;
            // The core is directed near (-4/5,3/5,0). Projecting the selected
            // Down face normal perpendicular to it gives an outward direction
            // near (-3/5,-4/5,0), within the genuine edge's incident fan.
            contact.PointA.Should().Be(center);
            contact.Normal.Y.Should().BeLessThan(Fixed64.Zero);
            contact.Normal.Z.Should().Be(Fixed64.Zero);
            Vector3d.Dot(contact.Normal, capsule.WorldAxis).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            (contact.Depth - Fixed64.Half).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Magnitude
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            Vector3d local = rotation.Inverse().Rotate(contact.PointB - center);
            (local.X * local.X + local.Z * local.Z - Fixed64.Quarter).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
            local.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64));
        }
        foundEdge.Should().BeTrue("an admitted outward projection retains the true edge's independent boundary constraint");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ParallelCoreAlongTrueEdge_ShouldKeepAnInteriorWitness(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 7, 1) };
        scenario.CreateBody(capsule, new(-Fixed64.Two - Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.LookRotation(Vector3d.Up, Vector3d.Forward), immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundInterior = false;
        for (int index = 0; index < pair.Manifold.Count; index++)
        {
            ManifoldContact contact = pair.Manifold[index];
            if (contact.Normal.X >= Fixed64.Zero || contact.PointA.Z.Abs() >= Fixed64.Two) continue;
            foundInterior = true;
            contact.PointA.Should().Be(new Vector3d(-2, 0, 0));
            contact.Normal.Y.Should().BeLessThan(Fixed64.Zero);
            Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Magnitude
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(8));
        }
        foundInterior.Should().BeTrue("a positive closest interval on the genuine edge must not collapse into corner constraints");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CoreCrossingTrueEdge_ShouldRetainItsPerpendicularRadiusExit(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.InitializeStaticCollider(capsule, new(-2, 0, 0));
        CollisionPair pair = scenario.CreatePair(mesh, capsule);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool foundEdge = false;
        for (int index = 0; index < pair.Manifold.Count; index++)
        {
            ManifoldContact contact = pair.Manifold[index];
            Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal)
                .Should().Be(Vector3d.Zero, "every retained row must pair its actual boundary ray");
            if (contact.Normal.X >= Fixed64.Zero) continue;
            foundEdge = true;
            contact.Normal.Should().Be(Vector3d.Left);
            contact.Depth.Should().Be(Fixed64.Half,
                "the core crosses the exposed edge, whose perpendicular exit is shorter than either axial cap ray");
            contact.PointA.Should().Be(new Vector3d(-2, 0, 0));
            contact.PointB.Should().Be(Vector3d.Right * (-Fixed64.Two + Fixed64.Half));
        }
        foundEdge.Should().BeTrue("a true edge retains its independent finite radial constraint");

        var separated = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scenario.InitializeStaticCollider(separated, Vector3d.Left * (Fixed64.Two + Fixed64.Half + Fixed64.FromRaw(1)));
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, separated)).Should().BeFalse();
    }

    private static LSMeshCollider CreateQuad(bool reverse, bool subdivide)
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(-2, 0, 2), new(2, 0, 2), Vector3d.Zero };
        if (!subdivide) System.Array.Resize(ref vertices, 4);
        int[] triangles = subdivide ? new[] { 0, 2, 4, 2, 3, 4, 3, 1, 4, 1, 0, 4 } : new[] { 0, 2, 1, 1, 2, 3 };
        if (reverse)
            for (int index = 0; index < triangles.Length; index += 3)
                (triangles[index], triangles[index + 2]) = (triangles[index + 2], triangles[index]);
        return new(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
