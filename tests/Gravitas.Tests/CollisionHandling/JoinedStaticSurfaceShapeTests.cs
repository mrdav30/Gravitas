using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;
using System;
using System.Collections.Generic;
using Chronicler.Hashing;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceShapeTests
{
    [Theory]
    [InlineData("Cone")]
    [InlineData("ConvexMesh")]
    [InlineData("ConcaveMesh")]
    public void ShapeCrossing_ShouldReplayEveryFrameAcrossDeclarationOrder(string family)
    {
        Assert.Equal(Trace(false), Trace(true));

        List<ChronicleHash> Trace(bool reverse)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.SetFrameRate(16);
            scene.Context.Environment.Gravity = Fixed64.One;
            scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var members = new[] { Floor(scene, -8, 0), Floor(scene, 0, 8, 2) };
            using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { members[1], members[0] } : members);
            LSCollider shape = family switch
            {
                "ConvexMesh" => MeshTestFixtures.CreateConvexCube(),
                "ConcaveMesh" => MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation),
                _ => new LSConeCollider()
            };
            shape.Material = PhysicsMaterial.Frictionless;
            var body = scene.CreateBody(shape, new((Fixed64)(-2), Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity,
                preventAngularForces: true).Body;
            body.UseManualGrounding(); body.SleepEnabled = false;
            var hashes = new List<ChronicleHash>();
            for (int frame = 0; frame < 96; frame++)
            {
                if (frame == 32) body.AddLinearImpulse(Vector3d.Right);
                scene.Context.Simulate(); scene.Context.LateSimulate();
                hashes.Add(scene.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
            }
            return hashes;
        }
    }

    [Theory]
    [InlineData("Cone")]
    [InlineData("Cylinder")]
    [InlineData("Cuboid")]
    [InlineData("ConvexMesh")]
    [InlineData("ConcaveMesh")]
    public void JoinedInterior_ShouldRetainOnlyTheCompleteFloorExit(string family)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0);
        var right = Floor(scene, 0, 8);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        LSCollider shape = family switch
        {
            "Cone" => new LSConeCollider(),
            "Cylinder" => new LSCylinderCollider(),
            "Cuboid" => new LSCuboidCollider(),
            "ConvexMesh" => MeshTestFixtures.CreateConvexCube(),
            _ => MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        };
        scene.CreateBody(shape, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        int rows = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, shape);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                rows++;
                Assert.Equal(Vector3d.Up, row.Normal);
                Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                Assert.Equal(Fixed64.Zero, p.Y);
            }
        }
        Assert.True(rows > 0);
    }

    [Theory]
    [InlineData("Cone", false, false, false, 1, 2)]
    [InlineData("Cone", true, false, false, 1, 2)]
    [InlineData("Cone", true, true, false, 1, 2)]
    [InlineData("Cone", true, false, true, 1, 2)]
    [InlineData("Cone", false, true, false, 32, 2)]
    [InlineData("Cone", false, false, false, 1, 16)]
    [InlineData("Cuboid", false, false, false, 1, 2)]
    [InlineData("Cylinder", false, false, false, 1, 2)]
    [InlineData("Cuboid", false, false, true, 1, 2)]
    [InlineData("Cylinder", false, false, true, 1, 2)]
    [InlineData("ConvexMesh", false, false, false, 1, 2)]
    [InlineData("ConvexMesh", true, true, true, 1, 2)]
    [InlineData("ConcaveMesh", false, false, false, 1, 2)]
    public void JoinedShapeCrossing_ShouldPreserveFrictionlessMotion(string family, bool partial, bool reverse, bool compound, int divisions, int count)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var members = new LSMeshCollider[count];
        for (int index = 0; index < count; index++)
            members[index] = Floor(scene, -8 + 16 * index / count, -8 + 16 * (index + 1) / count,
                partial && index == 1 ? 2 : 4, divisions);
        if (reverse) Array.Reverse(members);
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        LSCollider shape = family switch
        {
            "Cuboid" => compound ? new LSCompoundCollider(CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Zero,
                PhysicsMaterial.Frictionless)) : new LSCuboidCollider(),
            "Cylinder" => compound ? new LSCompoundCollider(CompoundColliderPart.Cylinder(Fixed64.Half, Fixed64.One, Vector3d.Zero,
                PhysicsMaterial.Frictionless)) : new LSCylinderCollider(),
            "ConvexMesh" => compound ? new LSCompoundCollider(new CompoundColliderPart(
                MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.One, PhysicsMaterial.Frictionless))
                : MeshTestFixtures.CreateConvexCube(),
            "ConcaveMesh" => MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation),
            _ => compound ? new LSCompoundCollider(CompoundColliderPart.Cone(Fixed64.Half, Fixed64.One, Vector3d.Zero,
                PhysicsMaterial.Frictionless)) : new LSConeCollider()
        };
        shape.Material = PhysicsMaterial.Frictionless;
        var body = scene.CreateBody(shape, new((Fixed64)(-2), Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding(); body.SleepEnabled = false;
        for (int frame = 0; frame < 32; frame++) Step();
        body.AddLinearImpulse(Vector3d.Right);
        bool valid = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            for (int frame = 0; frame < 128; frame++)
            {
                if (body.Position3d.X >= Fixed64.Two && body.LinearVelocity.X > Fixed64.Zero)
                    body.AddLinearImpulse(-Vector3d.Right * Fixed64.Two);
                else if (body.Position3d.X <= -Fixed64.Two && body.LinearVelocity.X < Fixed64.Zero)
                    body.AddLinearImpulse(Vector3d.Right * Fixed64.Two);
                Step();
                valid &= FixedMath.Abs(body.LinearVelocity.X) == Fixed64.One && body.LinearVelocity.Z == Fixed64.Zero;
            }
        }, warmupIterations: 4, stabilizationIterations: 2, measurementIterations: 4);
        Assert.True(valid);
        Assert.Equal(0, bytes);

        void Step() { scene.Context.Simulate(); scene.Context.LateSimulate(); }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ConeFaceExit_ShouldClearTheCompleteShapeAndRetainSupplierFrames(bool tilted, bool independent)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0);
        var right = independent ? new LSMeshCollider(new[] { new Vector3d(-4, 0, 0), new(4, 0, 0), new(-4, 0, 8), new(4, 0, 8) },
            new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) : Floor(scene, 0, 8);
        if (independent)
        {
            var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
            right.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        }
        left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(tilted ? -Fixed64.Quarter : Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), tilted
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized : FixedQuaternion.Identity);
        Fixed64 maximum = Fixed64.Zero;
        int count = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                count++;
                Assert.Equal(Vector3d.Up, row.Normal);
                Assert.Equal(member.Material, row.MaterialA);
                Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
                Assert.InRange(point.Y.m_rawValue, -1, 1);
                Assert.True(row.AnchorA.TryGetOffsetFrom(cone.Center, out Vector3d localSurfacePoint));
                Assert.Equal(point - cone.Center, localSurfacePoint);
                maximum = FixedMath.Max(maximum, row.Depth);
            }
        }
        Assert.InRange(count, 1, ContactManifold.MaxContactsPerGroup);
        Vector3d outside = cone.Center + Vector3d.Up * (maximum + Fixed64.FromRaw(16));
        foreach (var member in new[] { left, right })
            for (int triangle = 0; triangle < member.Mesh.TriangleCount; triangle++)
            {
                Assert.True(member.Mesh.TryGetTriangleVertices(triangle, out var a, out var b, out var c));
                Assert.False(new FixedTriangle(a, b, c).TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                    outside, cone.Rotation, cone.Height, cone.ScaledRadius, out _));
            }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeAtPartialJoinEnd_ShouldRetainRealCornerAndFiniteBoundaryWitnesses(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0);
        var right = Floor(scene, 0, 8, 2);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(Fixed64.Quarter, Fixed64.Quarter, (Fixed64)2 + Fixed64.Quarter), FixedQuaternion.Identity);
        bool boundary = false;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                Assert.True(row.AnchorB.TryGetWorldPoint(out _));
                Assert.InRange(p.Y.m_rawValue, -1, 1);
                Assert.True(p.X <= Fixed64.Zero || p.Z <= (Fixed64)2);
                boundary |= row.Normal.X != Fixed64.Zero || row.Normal.Z != Fixed64.Zero;
            }
        }
        Assert.True(boundary);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatingCone_ShouldKeepTangentialVelocityAcrossTheJoin(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(32);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var left = Floor(scene, -8, 0); var right = Floor(scene, 0, 8, 2);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var cone = new LSConeCollider { Material = PhysicsMaterial.Frictionless };
        var body = scene.CreateBody(cone, new((Fixed64)(-1), Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.UseManualGrounding(); body.SleepEnabled = false;
        body.AddLinearImpulse(Vector3d.Right);
        body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        for (int frame = 0; frame < 128; frame++)
        {
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal(Fixed64.One, body.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z);
            Assert.True(body.Position3d.Y >= Fixed64.Zero);
        }
        Assert.NotEqual(FixedQuaternion.Identity, body.Rotation);
        Assert.True(body.Position3d.X > Fixed64.Two);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Hole_ShouldRemainEmptyAndRetainItsActualRim(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var members = new[] { Rectangle(-4, -1, -4, 4), Rectangle(1, 4, -4, 4), Rectangle(-1, 1, -4, -1), Rectangle(-1, 1, 1, 4) };
        if (reverse) Array.Reverse(members);
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var cone = new LSConeCollider();
        scene.CreateBody(cone, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        foreach (var member in members) Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, cone)));
        cone.Body!.SetPosition(new(Fixed64.One + Fixed64.Quarter, Fixed64.Quarter, Fixed64.One + Fixed64.Quarter));
        bool hit = false;
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, cone);
            if (!CollisionDetection.DoCollisionCheck(pair)) continue;
            foreach (var row in pair.Manifold)
            {
                hit = true;
                Assert.True(row.AnchorA.TryGetWorldPoint(out var p));
                Assert.True(FixedMath.Abs(p.X) >= Fixed64.One || FixedMath.Abs(p.Z) >= Fixed64.One);
            }
        }
        Assert.True(hit);

        LSMeshCollider Rectangle(int left, int right, int near, int far)
        {
            var mesh = new LSMeshCollider(new[] { new Vector3d(left, 0, near), new(left, 0, far), new(right, 0, near), new(right, 0, far) },
                new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FoldedAndAmbiguousMembers_ShouldRetainIndependentPhysicalConstraints(bool duplicate)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0);
        var right = duplicate ? Floor(scene, -8, 0) : new LSMeshCollider(new[] { new Vector3d(0, 0, -4), new(0, 4, -4), new(0, 0, 4), new(0, 4, 4) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        if (!duplicate) scene.InitializeStaticCollider(right, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(-Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            Assert.True(pair.Manifold.HasContact);
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out _));
                Assert.True(row.AnchorB.TryGetWorldPoint(out _));
            }
        }
    }

    [Fact]
    public void AlternatingConeAndCapsule_ShouldNotReuseTheOtherFamilyGeometry()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var members = new[] { Floor(scene, -8, 0), Floor(scene, 0, 8) };
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var cone = new LSConeCollider();
        var capsule = new LSCapsuleCollider { Size = new(1, 2, 1), Radius = Fixed64.Half };
        scene.CreateBody(cone, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        scene.CreateBody(capsule, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        for (int iteration = 0; iteration < 4; iteration++)
            foreach (LSCollider shape in new LSCollider[] { cone, capsule })
            {
                Fixed64 maximum = Fixed64.Zero;
                foreach (var member in members)
                {
                    var pair = scene.CreatePair(member, shape);
                    Assert.True(CollisionDetection.DoCollisionCheck(pair));
                    foreach (var row in pair.Manifold)
                    {
                        Assert.Equal(Vector3d.Up, row.Normal);
                        maximum = FixedMath.Max(maximum, row.Depth);
                    }
                }
                Assert.Equal(shape == cone ? Fixed64.Quarter : Fixed64.One - Fixed64.Quarter, maximum);
            }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AmbiguousClassificationCut_ShouldKeepTheCompleteNativeOwner(bool addCertifiedNeighbor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Floor(scene, -8, 0);
        var crossing = new LSMeshCollider(new[] { new Vector3d(-1, 0, -1), new(1, 0, 1), new(1, 0, -1) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(crossing, Vector3d.Zero);
        var neighbor = Floor(scene, 0, 8);
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var expected = Rows();
        Assert.Contains(expected, row => row.Normal.X != Fixed64.Zero && row.A.Z == Fixed64.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(addCertifiedNeighbor
            ? new[] { neighbor, crossing, source } : new[] { crossing, source });
        Assert.Equal(expected, Rows());

        List<(Vector3d Normal, Fixed64 Depth, Vector3d A, Vector3d B)> Rows()
        {
            var result = new List<(Vector3d, Fixed64, Vector3d, Vector3d)>();
            var pair = scene.CreatePair(source, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var a)); Assert.True(row.AnchorB.TryGetWorldPoint(out var b));
                result.Add((row.Normal, row.Depth, a, b));
            }
            return result;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UntrustedIncidentOwner_ShouldNotSuppressItsTrustedNeighborsNativeCorner(bool ambiguityAtCorner)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Rectangle(0, 4, 0, 4);
        var neighbor = Rectangle(-4, 0, 0, 4);
        var overlapping = ambiguityAtCorner
            ? new LSMeshCollider(new[] { new Vector3d(-4, 0, 0), Vector3d.Zero, new(-4, 0, 4) },
                new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            : Rectangle(-4, -1, 2, 4);
        if (ambiguityAtCorner) scene.InitializeStaticCollider(overlapping, Vector3d.Zero);
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(-Fixed64.Quarter, Fixed64.Quarter, -Fixed64.Quarter), FixedQuaternion.Identity);
        var native = Rows();
        Assert.Contains(native, row => row.Normal.X < Fixed64.Zero && row.Normal.Z < Fixed64.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source, neighbor, overlapping });
        Assert.Equal(native, Rows());

        List<(Vector3d Normal, Fixed64 Depth, Vector3d A, Vector3d B)> Rows()
        {
            var result = new List<(Vector3d, Fixed64, Vector3d, Vector3d)>();
            var pair = scene.CreatePair(source, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var a)); Assert.True(row.AnchorB.TryGetWorldPoint(out var b));
                result.Add((row.Normal, row.Depth, a, b));
            }
            return result;
        }

        LSMeshCollider Rectangle(int left, int right, int near, int far)
        {
            var mesh = new LSMeshCollider(new[] { new Vector3d(left, 0, near), new(left, 0, far), new(right, 0, near), new(right, 0, far) },
                new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
        }
    }

    [Fact]
    public void FilteredIncidentMember_ShouldKeepTheNativeExposedCorner()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0);
        var right = Floor(scene, 0, 8);
        right.IsTrigger = true;
        var parallel = new LSMeshCollider(new[] { new Vector3d((Fixed64)(-8), Fixed64.Quarter, (Fixed64)(-4)),
            new((Fixed64)(-8), Fixed64.Quarter, (Fixed64)4), new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)(-4)),
            new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)4) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(parallel, Vector3d.Zero);
        var cone = new LSConeCollider();
        scene.CreateBody(cone, new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)4 + Fixed64.Quarter), FixedQuaternion.Identity);
        var expected = Rows();
        Assert.Contains(expected, row => row.Normal.Z > Fixed64.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, parallel, left });
        Assert.Equal(expected, Rows());

        List<(Vector3d Normal, Fixed64 Depth, Vector3d A, Vector3d B)> Rows()
        {
            var rows = new List<(Vector3d, Fixed64, Vector3d, Vector3d)>();
            var pair = scene.CreatePair(left, cone);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var a)); Assert.True(row.AnchorB.TryGetWorldPoint(out var b));
                rows.Add((row.Normal, row.Depth, a, b));
            }
            return rows;
        }
    }

    private static LSMeshCollider Floor(PhysicsScenarioBuilder scene, int left, int right, int halfWidth = 4, int divisions = 1)
    {
        var vertices = new Vector3d[2 * (divisions + 1)];
        var indices = new int[6 * divisions];
        for (int i = 0; i <= divisions; i++)
        {
            Fixed64 x = left + (Fixed64)(right - left) * i / divisions;
            vertices[2 * i] = new(x, Fixed64.Zero, (Fixed64)(-halfWidth));
            vertices[2 * i + 1] = new(x, Fixed64.Zero, (Fixed64)halfWidth);
            if (i == divisions) continue;
            int v = 2 * i, t = 6 * i;
            indices[t] = v; indices[t + 1] = v + 1; indices[t + 2] = v + 2;
            indices[t + 3] = v + 2; indices[t + 4] = v + 1; indices[t + 5] = v + 3;
        }
        var mesh = new LSMeshCollider(vertices, indices, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }
}
