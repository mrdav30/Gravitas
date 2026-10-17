using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Verifies finite mixed surface regions, real perimeters and their public/runtime consumers.</content>
public sealed partial class MixedNarrowPhaseTests
{
    [Fact]
    public void SubdividedNativeSupportLine_ShouldRetainOnlyItsTwoExtremeLevers()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider template = MeshTestFixtures.CreateConvexCube();
        var points = template.Mesh.LocalVertices.ToArray().ToList();
        for (int index = 1; index <= 5; index++)
            points.Add(new Vector3d(Fixed64.Half, Fixed64.Half, -Fixed64.Half + Fixed64.FromFraction(index, 6)));
        var triangles = new System.Collections.Generic.List<int>();
        int[] original = template.Mesh.Triangles.ToArray();
        for (int index = 0; index < original.Length; index += 3)
        {
            int[] face = original[index..(index + 3)];
            if (!face.Contains(3) || !face.Contains(7)) triangles.AddRange(face);
        }
        // Split both incident faces along the same physical cube edge.
        int previous = 3;
        for (int index = 8; index <= 13; index++)
        {
            int next = index == 13 ? 7 : index;
            triangles.AddRange(new[] { 5, previous, next, 6, next, previous });
            previous = next;
        }
        var hull = new LSMeshCollider(points.ToArray(), triangles.ToArray(), MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
        scene.CreateBody(hull, Vector3d.Zero, FixedQuaternion.Identity);
        var target = new LSAABBoxCollider2D(Vector2d.One) { MixedHalfThicknessOverride = Fixed64.Half };
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new Vector3d(1, 1, 0), FixedQuaternion.Identity, Vector3d.One)));
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(hull, target, manifold));
        Assert.Equal(2, manifold.Count);
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Fixed64.Zero, row.Depth);
            Assert.Equal(Vector3d.Right, row.Normal);
            Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Half, row.AnchorA.LocalPoint.Z), row.AnchorA.LocalPoint);
            Assert.Equal(Fixed64.Half, row.AnchorA.LocalPoint.Z.Abs());
            Assert.True(row.AnchorA.TryGetOffsetFrom(row.AnchorB, out Vector3d gap));
            Assert.Equal(Vector3d.Zero, gap);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedHullPrismEdgeWinner_ShouldKeepMatchedSupportEdges(bool polygon)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scene.Context;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCollider2D target = polygon
            ? new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half))
            : new LSAABBoxCollider2D(Vector2d.One);
        target.MixedHalfThicknessOverride = Fixed64.Half;
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(hull, new Vector3d(Fixed64.FromFraction(-12, 10), -Fixed64.Half, Fixed64.FromFraction(-6, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45));
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(hull, target, manifold));
        Assert.Equal(1, manifold.Count);
        ManifoldContact row = manifold.PrimaryContact;
        Assert.True(row.Depth > Fixed64.Zero);
        Assert.True(row.AnchorA.TryGetOffsetFrom(row.AnchorB, out Vector3d gap));
        Assert.InRange(Vector3d.Cross(gap, row.Normal).Magnitude.m_rawValue, 0L, 64L);
        Assert.InRange((Vector3d.Dot(gap, row.Normal) - row.Depth).Abs().m_rawValue, 0L, 64L);
        foreach (Vector3d point in new[] { row.AnchorA.LocalPoint, row.AnchorB.LocalPoint })
            Assert.Equal(2, Enumerable.Range(0, 3).Count(axis => point[axis].Abs() == Fixed64.Half));
    }

    [Fact]
    public void PrismWhollyInsideClosedHull_ShouldRetainItsFullSolidExit()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scene.Context;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 eighth = Fixed64.FromFraction(1, 8);
        var embedded = new LSPolygonCollider2D(new(-eighth, -eighth), new(eighth, -eighth),
            new(eighth, eighth), new(-eighth, eighth)) { MixedHalfThicknessOverride = eighth };
        embedded.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(hull, Vector3d.Zero, FixedQuaternion.Identity);
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(hull, embedded, manifold));
        Assert.Equal(1, manifold.GroupCount);
        Assert.Equal(4, manifold.Count);
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Fixed64.Half + eighth, row.Depth);
            Assert.True(row.AnchorA.TryGetOffsetFrom(Vector3d.Zero, out Vector3d first));
            Assert.True(row.AnchorB.TryGetOffsetFrom(Vector3d.Zero, out Vector3d second));
            Assert.Equal(-row.Normal * row.Depth, second - first);
        }
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void NativeCubeAgainstPrismWalls_ShouldRetainAWarmedPhysicalResponse(int pieces, bool hostMoves)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scene.Context;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(16);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        for (int index = 0; index < pieces; index++)
        {
            Fixed64 half = pieces == 1 ? (Fixed64)8 : (Fixed64)4;
            var wall = new LSPolygonCollider2D(new(-half, -Fixed64.One), new(half, -Fixed64.One),
                new(half, Fixed64.One), new(-half, Fixed64.One))
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
            wall.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
                new Vector3d(pieces == 1 ? Fixed64.Zero : index == 0 ? (Fixed64)(-4) : (Fixed64)4, Fixed64.Zero, -Fixed64.One),
                FixedQuaternion.Identity, Vector3d.One)));
        }
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        hull.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scene.CreateBody(hull, Vector3d.Forward * Fixed64.Half, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        for (int frame = 0; frame < 96; frame++) Step();
        Vector3d settled = body.Position3d;
        long allocated = System.GC.GetAllocatedBytesForCurrentThread();
        for (int frame = 0; frame < 128; frame++) Step();
        allocated = System.GC.GetAllocatedBytesForCurrentThread() - allocated;
        Assert.Equal(0, allocated);
        Assert.True(settled.Z > Fixed64.Zero && settled.Z <= Fixed64.Half);
        Assert.Equal(settled, body.Position3d);
        Assert.Equal(Vector3d.Zero, body.LinearVelocity);
        Assert.Equal(Vector3d.Zero, body.AngularVelocity);
        Assert.Equal(FixedQuaternion.Identity, body.Rotation);

        void Step()
        {
            if (hostMoves)
            {
                Vector3d position = body.Position3d;
                position.X = position.X == Fixed64.Zero ? Fixed64.Quarter : Fixed64.Zero;
                body.ResetPosition(position, FixedQuaternion.Identity);
            }
            body.AddForce(Vector3d.Backward); context.Simulate(); context.LateSimulate();
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void FreelyRotatingNativeCubeAgainstPrismWalls_ShouldMatchCuboidResponseQuality(int pieces)
    {
        var native = Run(native: true);
        var cuboid = Run(native: false);
        Assert.Equal(cuboid.InverseInertia, native.InverseInertia);
        // One forced step contributes dt squared; correction removes only its configured fraction.
        Fixed64 step = Fixed64.FromFraction(1, 16);
        Fixed64 tolerance = CollisionResponseMixed.PenetrationSlop
            + step * step / CollisionResponseMixed.PenetrationCorrectionPercent;
        foreach (var result in new[] { native, cuboid })
        {
            Assert.InRange(result.Penetration.m_rawValue, 0L, tolerance.m_rawValue);
            Assert.InRange(result.Position.Z.m_rawValue, (Fixed64.Half - tolerance).m_rawValue, (Fixed64.Half + tolerance).m_rawValue);
        }
        Assert.InRange(native.TangentialTravel.m_rawValue, 0L, CollisionResponseMixed.PenetrationSlop.m_rawValue);
        Assert.True(native.TangentialTravel <= cuboid.TangentialTravel + tolerance);
        Assert.InRange((native.Position.Z - cuboid.Position.Z).Abs().m_rawValue, 0L, tolerance.m_rawValue);

        (Vector3d Position, Fixed3x3 InverseInertia, Fixed64 Penetration, Fixed64 TangentialTravel) Run(bool native)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            GravitasWorldContext context = scene.Context;
            context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
            context.SetFrameRate(16);
            context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
            context.Environment.MinSpeed = Fixed64.Zero;
            for (int index = 0; index < pieces; index++)
            {
                Fixed64 half = pieces == 1 ? (Fixed64)8 : (Fixed64)4;
                var wall = new LSPolygonCollider2D(new(-half, -Fixed64.One), new(half, -Fixed64.One),
                    new(half, Fixed64.One), new(-half, Fixed64.One))
                { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
                wall.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
                    new Vector3d(pieces == 1 ? Fixed64.Zero : index == 0 ? (Fixed64)(-4) : (Fixed64)4, Fixed64.Zero, -Fixed64.One),
                    FixedQuaternion.Identity, Vector3d.One)));
            }
            LSCollider collider = native ? MeshTestFixtures.CreateConvexCube() : new LSCuboidCollider { Size = Vector3d.One };
            collider.Material = PhysicsMaterial.Frictionless;
            SolidBody body = scene.CreateBody(collider, Vector3d.Forward * Fixed64.Half, FixedQuaternion.Identity).Body;
            Fixed3x3 inertia = body.InverseInertiaTensor;
            body.SleepEnabled = false;
            body.UseManualGrounding();
            Fixed64 penetration = Fixed64.Zero, tangential = Fixed64.Zero;
            Vector3d[] corners = MeshTestFixtures.CreateConvexCube().Mesh.LocalVertices.ToArray();
            for (int frame = 0; frame < 224; frame++)
            {
                body.AddForce(Vector3d.Backward); context.Simulate(); context.LateSimulate();
                if (frame < 96) continue;
                tangential = FixedMath.Max(tangential, FixedMath.Max(body.Position3d.X.Abs(), body.Position3d.Y.Abs()));
                foreach (Vector3d corner in corners)
                    penetration = FixedMath.Max(penetration, -(body.Position3d + body.Rotation.Rotate(corner)).Z);
            }
            TestContext.Current.TestOutputHelper!.WriteLine($"native={native}, pieces={pieces}, position={body.Position3d}, rotation={body.Rotation}, velocity={body.LinearVelocity}, angular={body.AngularVelocity}, maxPenetration={penetration}, maxTangentialTravel={tangential}, inertia={inertia}");
            return (body.Position3d, inertia, penetration, tangential);
        }
    }

    [Fact]
    public void ClosedNativeHullAgainstPrism_ShouldUseOneSolidFaceAndMatchedCorners()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scene.Context;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var wall = new LSPolygonCollider2D(new(-4, -1), new(4, -1), new(4, 1), new(-4, 1))
        { MixedHalfThicknessOverride = Fixed64.Two };
        wall.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(0, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(hull, new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        var manifold = new ContactManifold();

        Assert.True(CollisionDetectionMixed.TryBuildManifold(hull, wall, manifold));
        Assert.Equal(1, manifold.GroupCount);
        Assert.Equal(4, manifold.Count);
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Vector3d.Backward, row.Normal);
            Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.True(row.AnchorA.TryGetOffsetFrom(Vector3d.Zero, out Vector3d first));
            Assert.True(row.AnchorB.TryGetOffsetFrom(Vector3d.Zero, out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), second - first);
            Assert.Equal(Fixed64.Half, first.X.Abs());
            Assert.Equal(Fixed64.Half, first.Y.Abs());
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedHullFaceClippedToEightCorners_ShouldRetainFourFiniteSupports(bool reverseWinding)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scene.Context;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 reach = Fixed64.FromFraction(3, 4), inset = Fixed64.FromFraction(1, 8);
        Vector2d[] vertices = { new(-reach, -inset), new(-inset, -reach), new(inset, -reach), new(reach, -inset),
            new(reach, inset), new(inset, reach), new(-inset, reach), new(-reach, inset) };
        if (reverseWinding) System.Array.Reverse(vertices);
        var target = new LSPolygonCollider2D(vertices) { MixedHalfThicknessOverride = Fixed64.Half };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(hull, Vector3d.Up * reach, FixedQuaternion.Identity);
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(hull, target, manifold));
        Assert.Equal(1, manifold.GroupCount);
        Assert.Equal(4, manifold.Count);
        Assert.Equal(4, manifold.Select(row => row.AnchorA.LocalPoint).Distinct().Count());
        Vector3d[] retained = manifold.Select(row => row.AnchorA.LocalPoint).ToArray();
        foreach (Vector3d point in retained)
            Assert.Contains(retained, other => other.X == -point.X && other.Z == -point.Z);
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Vector3d.Down, row.Normal);
            Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.True(row.AnchorA.TryGetOffsetFrom(Vector3d.Zero, out Vector3d first));
            Assert.True(row.AnchorB.TryGetOffsetFrom(Vector3d.Zero, out Vector3d second));
            Assert.Equal(Vector3d.Up * Fixed64.Quarter, second - first);
            Assert.Equal(Fixed64.Quarter, first.Y);
            Assert.Equal(Fixed64.Half, second.Y);
            // Clipping x,z <= 1/2 by |x|+|z| <= 7/8 creates eight exact finite vertices.
            Assert.Equal(Fixed64.Half, FixedMath.Max(first.X.Abs(), first.Z.Abs()));
            Assert.Equal(reach + inset, first.X.Abs() + first.Z.Abs());
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, 0)]
    [InlineData(ColliderType2D.Circle, 1)]
    [InlineData(ColliderType2D.Circle, 2)]
    [InlineData(ColliderType2D.Circle, 3)]
    [InlineData(ColliderType2D.Circle, 4)]
    [InlineData(ColliderType2D.Capsule, 0)]
    [InlineData(ColliderType2D.Capsule, 1)]
    [InlineData(ColliderType2D.Capsule, 2)]
    [InlineData(ColliderType2D.Capsule, 3)]
    [InlineData(ColliderType2D.Capsule, 4)]
    [InlineData(ColliderType2D.AABox, 0)]
    [InlineData(ColliderType2D.AABox, 1)]
    [InlineData(ColliderType2D.AABox, 4)]
    [InlineData(ColliderType2D.ConvexPolygon, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 1)]
    [InlineData(ColliderType2D.ConvexPolygon, 4)]
    public void FilledMeshPatch_ShouldRejectCoveredSeamForEveryEmbeddedShape(ColliderType2D shape, int topology)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionFloor(topology);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        // A square centered at 9/20 narrowly crosses the diagonal; curved shapes
        // at 3/10 reproduce a smaller false triangle exit than the real cap exit.
        Fixed64 coordinate = shape is ColliderType2D.AABox or ColliderType2D.ConvexPolygon
            ? Fixed64.FromFraction(9, 20) : Fixed64.FromFraction(3, 10);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, new Vector2d(coordinate, coordinate));
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();

        manifold.GroupCount.Should().Be(1, "the filled quad and intersecting footprint each form one connected region");
        foreach (ManifoldContact row in manifold)
        {
            row.Normal.Should().Be(Vector3d.Down);
            row.Depth.Should().Be(Fixed64.Quarter);
            row.DepthIsClamped.Should().BeFalse();
            row.PointA.Y.Should().Be(Fixed64.Zero);
            row.PointB.Y.Should().Be(Fixed64.Quarter);
            row.AnchorA.Origin.Should().Be(mesh.Mesh.Origin);
            row.AnchorB.Origin.Y.Should().Be(-Fixed64.Quarter);
        }
        CollisionDetectionMixed.TryCollide(mesh, slab.Collider, out MixedContact representative).Should().BeTrue();
        representative.Normal3DTo2D.Should().Be(manifold.PrimaryContact.Normal);
        representative.Depth.Should().Be(manifold.PrimaryContact.Depth);
        representative.Anchor3D.Should().Be(manifold.PrimaryContact.AnchorA);
        representative.Anchor2D.Should().Be(manifold.PrimaryContact.AnchorB);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, false, false)]
    [InlineData(ColliderType2D.Circle, false, true)]
    [InlineData(ColliderType2D.Circle, true, false)]
    [InlineData(ColliderType2D.Circle, true, true)]
    [InlineData(ColliderType2D.Capsule, false, false)]
    [InlineData(ColliderType2D.Capsule, false, true)]
    [InlineData(ColliderType2D.Capsule, true, false)]
    [InlineData(ColliderType2D.Capsule, true, true)]
    [InlineData(ColliderType2D.AABox, false, false)]
    [InlineData(ColliderType2D.AABox, false, true)]
    [InlineData(ColliderType2D.AABox, true, false)]
    [InlineData(ColliderType2D.AABox, true, true)]
    [InlineData(ColliderType2D.ConvexPolygon, false, false)]
    [InlineData(ColliderType2D.ConvexPolygon, false, true)]
    [InlineData(ColliderType2D.ConvexPolygon, true, false)]
    [InlineData(ColliderType2D.ConvexPolygon, true, true)]
    public void FiniteMeshPatch_ShouldRetainRealOuterAndHoleRims(ColliderType2D shape, bool hole, bool touching)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = hole ? CreateSurfaceReductionRing() : CreateSurfaceReductionFloor(0);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        // Right outer edge X=2, right hole edge X=1. The source crosses their
        // finite segments at Z=0; no authored diagonal is a physical boundary.
        Fixed64 centerX = hole
            ? (touching ? Fixed64.Half : Fixed64.FromFraction(3, 4))
            : (touching ? Fixed64.FromFraction(5, 2) : Fixed64.FromFraction(19, 8));
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, new Vector2d(centerX, Fixed64.Zero));
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();

        Vector3d normal = hole ? Vector3d.Left : Vector3d.Right;
        Fixed64 depth = touching ? Fixed64.Zero : hole ? Fixed64.Quarter : Fixed64.FromFraction(1, 8);
        ManifoldContact[] rimRows = manifold.Where(row => row.Normal == normal).ToArray();
        rimRows.Should().NotBeEmpty("a finite physical perimeter must survive covered seam suppression");
        foreach (ManifoldContact row in rimRows)
        {
            row.Depth.Should().Be(depth);
            row.PointA.X.Should().Be(hole ? Fixed64.One : (Fixed64)2);
            row.PointB.X.Should().Be(centerX + (hole ? Fixed64.Half : -Fixed64.Half));
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void FilledMeshPatch_Retriangulation_ShouldPreserveCanonicalSamplesAndRepresentative(ColliderType2D shape)
    {
        ManifoldContact[]? reference = null;
        for (int topology = 0; topology < 6; topology++)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSMeshCollider mesh = CreateSurfaceReductionFloor(topology);
            CreateMesh3D(context, mesh, Vector3d.Zero);
            SolidBody2D slab = CreateSurfaceReductionSlab(context, shape,
                new Vector2d(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10)));
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            ManifoldContact[] rows = manifold.OrderBy(row => row.ContactId).ToArray();
            reference ??= rows;
            rows.Length.Should().Be(reference.Length, $"topology {topology} is the same finite filled quad");
            for (int i = 0; i < rows.Length; i++)
            {
                rows[i].ContactId.Should().Be(reference[i].ContactId, $"sample {i}, topology {topology}");
                rows[i].AnchorA.Should().Be(reference[i].AnchorA);
                rows[i].AnchorB.Should().Be(reference[i].AnchorB);
                rows[i].Normal.Should().Be(reference[i].Normal);
                rows[i].Depth.Should().Be(reference[i].Depth);
            }
            CollisionDetectionMixed.TryCollide(mesh, slab.Collider, out MixedContact representative).Should().BeTrue();
            representative.Anchor3D.Should().Be(reference[0].AnchorA);
            representative.Anchor2D.Should().Be(reference[0].AnchorB);
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, 2)]
    [InlineData(ColliderType2D.Capsule, 4)]
    [InlineData(ColliderType2D.AABox, 4)]
    [InlineData(ColliderType2D.ConvexPolygon, 4)]
    public void VerticalFilledMeshPatch_ShouldRetainOnlyCanonicalPhysicalSideSupports(ColliderType2D shape, int count)
    {
        ManifoldContact[]? reference = null;
        for (int topology = 0; topology < 6; topology++)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSMeshCollider mesh = CreateSurfaceReductionFloor(topology, vertical: true);
            CreateMesh3D(context, mesh, Vector3d.Zero);
            SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, new Vector2d(Fixed64.Quarter, Fixed64.Zero), Fixed64.Zero);
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            manifold.GroupCount.Should().Be(1);
            manifold.Count.Should().Be(count, "a curved circle has a support line; a flat stadium/prism side has a support rectangle");
            ManifoldContact[] rows = manifold.OrderBy(row => row.ContactId).ToArray();
            reference ??= rows;
            for (int row = 0; row < rows.Length; row++)
            {
                rows[row].Normal.Should().Be(Vector3d.Right);
                rows[row].Depth.Should().Be(Fixed64.Quarter);
                rows[row].ContactId.Should().Be(reference[row].ContactId);
                rows[row].AnchorA.Should().Be(reference[row].AnchorA);
                rows[row].AnchorB.Should().Be(reference[row].AnchorB);
            }
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void VerticalRing_ShouldKeepDisconnectedPhysicalSideSupportRegions(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionRing(vertical: true);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, new Vector2d(Fixed64.Quarter, Fixed64.Zero), Fixed64.Zero, (Fixed64)2);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
        ManifoldContact[] faces = manifold.Where(row => row.Normal == Vector3d.Right).ToArray();
        faces.Should().NotBeEmpty();
        faces.Any(row => row.PointA.Y >= Fixed64.One).Should().BeTrue();
        faces.Any(row => row.PointA.Y <= -Fixed64.One).Should().BeTrue();
        foreach (ManifoldContact row in faces) row.Depth.Should().Be(Fixed64.Quarter);
        manifold.GroupCount.Should().BeGreaterThanOrEqualTo(2);
    }

    [Fact]
    public void MixedSlabCrossingIndependentMeshPlanes_ShouldRetainBothFaceDirections()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = new LSMeshCollider(
            new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2),
                new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, -2, 2), new Vector3d(0, 2, 2) },
            new[] { 0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, ColliderType2D.Circle,
            new Vector2d(Fixed64.Quarter, Fixed64.Zero));
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();

        manifold.GroupCount.Should().Be(2, "both physical support faces retain their own constraints");
        manifold.Should().Contain(row => row.Normal == Vector3d.Down && row.Depth == Fixed64.Quarter);
        manifold.Should().Contain(row => row.Normal == Vector3d.Right && row.Depth == Fixed64.Quarter);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Quarter);
        foreach (ManifoldContact row in manifold)
        {
            row.Depth.Should().BeInRange(Fixed64.Zero, Fixed64.Quarter);
            row.Normal.Should().BeOneOf(Vector3d.Down, Vector3d.Right);
            AssertFiniteSurfaceReductionRay(row, ColliderType2D.Circle,
                new Vector2d(Fixed64.Quarter, Fixed64.Zero));
            if (row.Normal == Vector3d.Down)
                row.PointA.Y.Should().Be(Fixed64.Zero);
            else
                row.PointA.X.Should().Be(Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void TiltedFilledMeshPatch_ShouldUseMaximumFiniteDirectionalRay(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = new LSMeshCollider(
            new[] { new Vector3d(-2, 2, -2), new Vector3d(2, -2, -2), new Vector3d(-2, 2, 2), new Vector3d(2, -2, 2) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, Vector2d.Zero);
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();

        // On X+Y=0, the smaller +X/+Y ray reaches both X=1/2 and
        // Y=1/4 from X=1/8,Y=-1/8: depth=3*sqrt(2)/8. The opposite
        // ray reaches 5*sqrt(2)/8. Every shape has this same X/cap section.
        Fixed64 component = FixedMath.Sqrt(Fixed64.Half);
        Fixed64 depth = FixedMath.Sqrt((Fixed64)2) * Fixed64.FromFraction(3, 8);
        manifold.GroupCount.Should().Be(1, "the filled owner contains one physical support stratum");
        (manifold.PrimaryContact.Depth - depth).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
        foreach (ManifoldContact row in manifold)
        {
            (row.Normal.X + component).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
            (row.Normal.Y + component).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
            row.Normal.Z.Should().Be(Fixed64.Zero);
            row.Depth.Should().BeInRange(Fixed64.Zero, depth + Fixed64.FromRaw(2));
            (row.PointA.X + row.PointA.Y).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
            AssertFiniteSurfaceReductionRay(row, shape, Vector2d.Zero);
        }
    }

    [Fact]
    public void TiltedMeshFace_ShouldPreserveFrictionlessSpinOfACircularSlab()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.SetFrameRate(64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        var mesh = new LSMeshCollider(
            new[] { new Vector3d(-4, 12, -4), new Vector3d(4, 4, -4),
                new Vector3d(-4, -4, 4), new Vector3d(4, -12, 4) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            { Material = PhysicsMaterial.Frictionless };
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D circle = CreateSurfaceReductionSlab(context, ColliderType2D.Circle, Vector2d.Zero);
        circle.Collider.Material = PhysicsMaterial.Frictionless;
        circle.FreezeAxes = BodyFreezeAxes2D.Position;
        circle.UseManualGrounding();
        circle.ApplyCollisionAngularVelocityDelta(Fixed64.One);
        circle.AngularVelocity.Should().Be(Fixed64.One);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, circle.Collider, manifold).Should().BeTrue();

        context.Simulate();
        circle.AngularVelocity.Should().Be(Fixed64.One, "free integration must preserve the incoming spin");
        context.LateSimulate();

        circle.AngularVelocity.Should().Be(Fixed64.One,
            "yaw leaves the circular slab geometry unchanged, and frictionless normals cannot oppose its spin");
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, false)]
    [InlineData(ColliderType2D.Circle, true)]
    [InlineData(ColliderType2D.Capsule, false)]
    [InlineData(ColliderType2D.Capsule, true)]
    [InlineData(ColliderType2D.AABox, false)]
    [InlineData(ColliderType2D.AABox, true)]
    [InlineData(ColliderType2D.ConvexPolygon, false)]
    [InlineData(ColliderType2D.ConvexPolygon, true)]
    public void TallSlabFaceClearance_ShouldUseTheWholeFiniteOwnerAndCanonicalCoverage(ColliderType2D shape, bool finite)
    {
        ManifoldContact[]? reference = null;
        foreach (int divisions in new[] { 1, 8 })
        {
            using GravitasWorldContext context = CreateMixedContext();
            int extent = finite ? 2 : 128;
            var vertices = new Vector3d[(divisions + 1) * (divisions + 1)];
            var triangles = new int[6 * divisions * divisions];
            for (int z = 0; z <= divisions; z++)
                for (int x = 0; x <= divisions; x++)
                {
                    Fixed64 px = (Fixed64)(-extent) + Fixed64.FromFraction(2 * extent * x, divisions);
                    Fixed64 pz = (Fixed64)(-extent) + Fixed64.FromFraction(2 * extent * z, divisions);
                    vertices[z * (divisions + 1) + x] = new Vector3d(px, -px, pz);
                }
            int index = 0;
            for (int z = 0; z < divisions; z++)
                for (int x = 0; x < divisions; x++)
                {
                    int first = z * (divisions + 1) + x, next = first + divisions + 1;
                    triangles[index++] = first; triangles[index++] = next; triangles[index++] = first + 1;
                    triangles[index++] = first + 1; triangles[index++] = next; triangles[index++] = next + 1;
                }
            var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            var source = CreateMesh3D(context, mesh, Vector3d.Zero);
            SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, Vector2d.Zero, Fixed64.Zero, (Fixed64)100);
            var manifold = new ContactManifold();

            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();

            // A finite owner clears the tall extrusion sideways at X=+/-2.5;
            // the large owner reaches the real oblique cap support instead.
            Fixed64 expected = finite ? Fixed64.FromFraction(5, 2)
                : Fixed64.FromFraction(201, 2) / FixedMath.Sqrt(Fixed64.Two);
            (manifold.PrimaryContact.Depth - expected).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(4));
            if (finite)
            {
                manifold.PrimaryContact.Normal.X.Abs().Should().Be(Fixed64.One);
                manifold.PrimaryContact.Normal.Y.Should().Be(Fixed64.Zero);
                manifold.PrimaryContact.Normal.Z.Should().Be(Fixed64.Zero);
            }
            ManifoldContact[] rows = manifold.OrderBy(row => row.ContactId).ToArray();
            reference ??= rows;
            rows.Length.Should().Be(reference.Length, "the filled surface and initial section are unchanged by subdivision");
            for (int row = 0; row < rows.Length; row++)
            {
                rows[row].ContactId.Should().Be(reference[row].ContactId);
                rows[row].AnchorA.Should().Be(reference[row].AnchorA);
                rows[row].AnchorB.Should().Be(reference[row].AnchorB);
                rows[row].Depth.Should().Be(reference[row].Depth);
            }
            // Move the whole finite owner along its selected direction. The
            // smaller source-in-section depth must not be accepted as clearance.
            Vector3d direction = -manifold.PrimaryContact.Normal;
            Fixed64 margin = Fixed64.FromFraction(1, 1024);
            source.Body.SetPosition(direction * (expected - margin));
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            source.Body.SetPosition(direction * (expected + margin));
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeFalse();
        }
    }

    [Fact]
    public void ObliqueCircleSupport_OnTheTrueFiniteEdge_ShouldKeepTheMatchedSupportPoint()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 edgeX = Fixed64.FromFraction(1, 8);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(edgeX, -edgeX, (Fixed64)(-2)),
            new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)2),
            new Vector3d(edgeX, -edgeX, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, ColliderType2D.Circle, Vector2d.Zero);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
        ManifoldContact row = manifold.Single(contact => contact.Normal.X < Fixed64.Zero && contact.Normal.Y < Fixed64.Zero);
        row.PointA.Should().Be(new Vector3d(edgeX, -edgeX, Fixed64.Zero));
        row.PointB.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Quarter, Fixed64.Zero));
        (row.Depth - FixedMath.Sqrt(Fixed64.Two) * Fixed64.FromFraction(3, 8)).Abs()
            .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
    }

    [Fact]
    public void DeepNonconvexSlabContact_ShouldCertifyTheWholeOwnerAcrossSubdivisionAndWinding()
    {
        ManifoldContact? reference = null;
        foreach (int divisions in new[] { 1, 2 })
        foreach (bool reversed in new[] { false, true })
        {
            using GravitasWorldContext context = CreateMixedContext();
            var vertices = new System.Collections.Generic.List<Vector3d>();
            var triangles = new System.Collections.Generic.List<int>();
            int[] xs = { -4, 1, 3, 4 }, zs = { -4, -1, 1, 4 };
            for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
            {
                if (x == 1 && z == 1) continue; // A distant hole preserves a nonconvex finite owner.
                for (int iz = 0; iz < divisions; iz++)
                for (int ix = 0; ix < divisions; ix++)
                {
                    int first = vertices.Count;
                    Fixed64 x0 = (Fixed64)xs[x] + Fixed64.FromFraction((xs[x + 1] - xs[x]) * ix, divisions);
                    Fixed64 x1 = (Fixed64)xs[x] + Fixed64.FromFraction((xs[x + 1] - xs[x]) * (ix + 1), divisions);
                    Fixed64 z0 = (Fixed64)zs[z] + Fixed64.FromFraction((zs[z + 1] - zs[z]) * iz, divisions);
                    Fixed64 z1 = (Fixed64)zs[z] + Fixed64.FromFraction((zs[z + 1] - zs[z]) * (iz + 1), divisions);
                    vertices.AddRange(new[] { new Vector3d(x0, -x0, z0), new Vector3d(x1, -x1, z0),
                        new Vector3d(x0, -x0, z1), new Vector3d(x1, -x1, z1) });
                    triangles.AddRange(reversed ? new[] { first + 1, first + 2, first, first + 3, first + 2, first + 1 }
                        : new[] { first, first + 2, first + 1, first + 1, first + 2, first + 3 });
                }
            }
            var mesh = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(), MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation);
            var source = CreateMesh3D(context, mesh, Vector3d.Zero);
            SolidBody2D slab = CreateSurfaceReductionSlab(context, ColliderType2D.Circle,
                Vector2d.Left * (Fixed64)2, Fixed64.Zero, (Fixed64)100);
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            manifold.Count.Should().Be(1, "neither the original section perimeter nor the remote cap projection provides a contact");
            ManifoldContact row = manifold.PrimaryContact;
            row.Normal.Should().Be(Vector3d.Left);
            row.Depth.Should().Be(Fixed64.FromFraction(5, 2));
            row.TryGetPointA(out _).Should().BeTrue();
            row.TryGetPointB(out _).Should().BeTrue();
            reference ??= row;
            row.ContactId.Should().Be(reference.Value.ContactId);
            row.AnchorA.Should().Be(reference.Value.AnchorA);
            row.AnchorB.Should().Be(reference.Value.AnchorB);
            context.Simulate();
            Vector2d queryStart = Vector2d.Left * (Fixed64)2;
            context.QueryMixed.SweepCircleAgainst3D(queryStart, queryStart + Vector2d.Left,
                Fixed64.Half, Fixed64.Zero, (Fixed64)100, PhysicsLayerMask.FromLayer(0), out PhysicsMixedHit hit)
                .Should().BeTrue("initial public queries use the same whole-owner fallback without a source collider");
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Collider3D.Should().BeSameAs(mesh);
            Fixed64 margin = Fixed64.FromFraction(1, 1024);
            source.Body.SetPosition(-row.Normal * (row.Depth - margin));
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            source.Body.SetPosition(-row.Normal * (row.Depth + margin));
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeFalse();
        }
    }

    private static void AssertFiniteSurfaceReductionRay(ManifoldContact row, ColliderType2D shape, Vector2d center)
    {
        // Retained coverage rays may be shallower than the regional maximum.
        // Each still pairs a finite face witness with its own slab boundary exit,
        // separated along the shared canonical direction by its actual depth.
        Fixed64 tolerance = Fixed64.FromRaw(4);
        Vector3d gapError = row.PointB - row.PointA + row.Normal * row.Depth;
        gapError.X.Abs().Should().BeLessThanOrEqualTo(tolerance);
        gapError.Y.Abs().Should().BeLessThanOrEqualTo(tolerance);
        gapError.Z.Abs().Should().BeLessThanOrEqualTo(tolerance);
        bool exitOnRim = false;
        foreach (Vector3d point in new[] { row.PointB })
        {
            point.Y.Should().BeInRange(-Fixed64.FromFraction(3, 4) - tolerance, Fixed64.Quarter + tolerance);
            Fixed64 x = point.X - center.X, z = point.Z - center.Y;
            bool onRim;
            if (shape is ColliderType2D.Circle or ColliderType2D.Capsule)
            {
                Fixed64 coreHalf = shape == ColliderType2D.Capsule ? Fixed64.FromFraction(1, 8) : Fixed64.Zero;
                Fixed64 radialZ = FixedMath.Max(z.Abs() - coreHalf, Fixed64.Zero);
                Fixed64 radialSquared = x * x + radialZ * radialZ;
                radialSquared.Should().BeLessThanOrEqualTo(Fixed64.Quarter + tolerance);
                onRim = (radialSquared - Fixed64.Quarter).Abs() <= tolerance;
            }
            else
            {
                x.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half + tolerance);
                z.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half + tolerance);
                onRim = (x.Abs() - Fixed64.Half).Abs() <= tolerance
                    || (z.Abs() - Fixed64.Half).Abs() <= tolerance;
            }
            exitOnRim = onRim;
        }
        bool exitOnCap = (row.PointB.Y + Fixed64.FromFraction(3, 4)).Abs() <= tolerance
            || (row.PointB.Y - Fixed64.Quarter).Abs() <= tolerance;
        (exitOnCap || exitOnRim).Should().BeTrue("each paired ray ends on the actual finite slab boundary");
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void FilledMeshPatch_FullRangeTranslation_ShouldPreserveFiniteLocalAnchors(ColliderType2D shape)
    {
        ManifoldContact[]? reference = null;
        foreach (Vector3d translation in new[] { Vector3d.Zero,
            new Vector3d(Fixed64.MaxValue - (Fixed64)4, Fixed64.MinValue + (Fixed64)4, Fixed64.MaxValue - (Fixed64)4) })
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSMeshCollider mesh = CreateSurfaceReductionFloor(0);
            CreateMesh3D(context, mesh, translation);
            Fixed64 coordinate = Fixed64.FromFraction(3, 10);
            SolidBody2D slab = CreateSurfaceReductionSlab(context, shape,
                new Vector2d(translation.X + coordinate, translation.Z + coordinate), translation.Y - Fixed64.Quarter);
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
            ManifoldContact[] rows = manifold.OrderBy(row => row.AnchorA.LocalPoint.X)
                .ThenBy(row => row.AnchorA.LocalPoint.Y).ThenBy(row => row.AnchorA.LocalPoint.Z).ToArray();
            reference ??= rows;
            rows.Length.Should().Be(reference.Length);
            for (int i = 0; i < rows.Length; i++)
            {
                rows[i].Normal.Should().Be(Vector3d.Down);
                rows[i].Depth.Should().Be(Fixed64.Quarter);
                rows[i].DepthIsClamped.Should().BeFalse();
                rows[i].AnchorA.LocalPoint.Should().Be(reference[i].AnchorA.LocalPoint);
                rows[i].AnchorB.LocalPoint.Should().Be(reference[i].AnchorB.LocalPoint);
                (rows[i].AnchorA.Origin - reference[i].AnchorA.Origin).Should().Be(translation);
                (rows[i].AnchorB.Origin - reference[i].AnchorB.Origin).Should().Be(translation);
                rows[i].TryGetPointA(out _).Should().BeTrue();
                rows[i].TryGetPointB(out _).Should().BeTrue();
            }
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void RingAndOuterEmptySpace_ShouldNotBecomeAnInfinitePlane(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionRing();
        CreateMesh3D(context, mesh, Vector3d.Zero);
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, Vector2d.Zero);
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
        slab.SetPosition(Vector2d.Right * (Fixed64)3);
        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(4)]
    public void CircleSweepInsideFilledMeshPatch_ShouldUseMotionIndependentFiniteFace(int topology)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionFloor(topology);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10));
        foreach (Vector2d travel in new[] { Vector2d.Left, Vector2d.Right })
        {
            context.QueryMixed.SweepCircleAgainst3D(start, start + travel, Fixed64.Half,
                -Fixed64.Quarter, Fixed64.Half, PhysicsLayerMask.FromLayer(0), out PhysicsMixedHit hit)
                .Should().BeTrue();

            hit.Collider3D.Should().BeSameAs(mesh);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal3DTo2D.Should().Be(Vector3d.Down);
            hit.NormalFor2DSource.Should().Be(Vector2d.Zero);
            hit.Point3D.Y.Should().Be(Fixed64.Zero);
            hit.Point2D.Y.Should().Be(Fixed64.Quarter);
            // Any finite face sample is valid; selecting the representative does
            // not promise the disk center. One rounding per coordinate and each
            // squared product permits four raw units in the radial certificate.
            Fixed64 dx = hit.Point3D.X - start.X, dz = hit.Point3D.Z - start.Y;
            (dx * dx + dz * dz).Should().BeLessThanOrEqualTo(Fixed64.Quarter + Fixed64.FromRaw(4));
            (hit.Point3D.X - hit.Point2D.X).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
            (hit.Point3D.Z - hit.Point2D.Z).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(2));
        }
    }

    [Fact]
    public void CircleSweepWithinRealHoleAndOutsideMesh_ShouldRemainMissed()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionRing();
        CreateMesh3D(context, mesh, Vector3d.Zero);
        foreach (Vector2d start in new[] { Vector2d.Zero, Vector2d.Right * (Fixed64)3 })
        {
            context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Forward * Fixed64.Quarter,
                Fixed64.Half, -Fixed64.Quarter, Fixed64.Half, PhysicsLayerMask.FromLayer(0), out _)
                .Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void FilledMeshPatchFullLoop_ShouldPreserveFrictionlessPlanarTravelAndContactLifetime(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        LSMeshCollider mesh = CreateSurfaceReductionFloor(0);
        mesh.Material = PhysicsMaterial.Frictionless;
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Vector2d start = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10));
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, start);
        slab.FreezeAxes = BodyFreezeAxes2D.Rotation;
        slab.Collider.Material = PhysicsMaterial.Frictionless;
        slab.UseManualGrounding();
        Vector2d displacement = Vector2d.Left * Fixed64.FromFraction(1, 16);
        slab.AddLinearImpulse(displacement * slab.Mass);
        int entered = 0, contacted = 0, exited = 0;
        slab.Collider.OnMixedContactEnter += other => { other.Should().BeSameAs(mesh); entered++; };
        slab.Collider.OnMixedContact += other => { other.Should().BeSameAs(mesh); contacted++; };
        slab.Collider.OnMixedContactExit += other => { other.Should().BeSameAs(mesh); exited++; };

        for (int frame = 1; frame <= 2; frame++)
        {
            context.Simulate();
            context.LateSimulate();

            // A vertical constraint cannot translate or spin the plane-constrained
            // 2D participant; frictionless travel must cross the authored seam.
            slab.Position.Should().Be(start + displacement * (Fixed64)frame);
            slab.LinearVelocity.Should().Be(displacement);
            slab.AngularVelocity.Should().Be(Fixed64.Zero);
            slab.Agent.Transform.LocalPosition.Y.Should().Be(-Fixed64.Quarter);
            context.MixedCollisions.ActivePairCount.Should().Be(1);
        }
        entered.Should().Be(1);
        contacted.Should().BeGreaterThan(0);
        exited.Should().Be(0);
        slab.SetPosition(new Vector2d(4, 4));
        context.Simulate();
        context.LateSimulate();
        exited.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void StadiumCrossingRingStrips_ShouldRetainBothFiniteRegionsAndRealPerimeters()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionRing();
        CreateMesh3D(context, mesh, Vector3d.Zero);
        var stadium = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4) { MixedHalfThicknessOverride = (Fixed64)2 };
        SolidBody2D slab = CreateBody2D(context, stadium, Vector2d.Zero);
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(mesh, stadium, manifold).Should().BeTrue();

        // The three-unit core meets both strips while the center is in a hole.
        // A manifold retains their independent constraints; no row claims a
        // global escape from the union merely by clearing one local inner rim.
        ManifoldContact[] faces = manifold.Where(row => row.Normal.Y.Abs() == Fixed64.One).ToArray();
        faces.Should().NotBeEmpty();
        faces.Any(row => row.PointA.Z >= Fixed64.One).Should().BeTrue();
        faces.Any(row => row.PointA.Z <= -Fixed64.One).Should().BeTrue();
        foreach (ManifoldContact face in faces)
            face.Depth.Should().Be((Fixed64)2);
        foreach (ManifoldContact row in manifold)
            row.Normal.X.Should().Be(Fixed64.Zero, "neither crossed strip has an X-facing physical rim or diagonal seam");
        manifold.GroupCount.Should().BeGreaterThanOrEqualTo(2);

        Fixed64 step = Fixed64.FromFraction(1, 4096);
        foreach (int phase in new[] { -1, 0, 1 })
        {
            slab.Agent.Transform.LocalPosition = new Vector3d(Fixed64.Zero, (Fixed64)2 + (Fixed64)phase * step, Fixed64.Zero);
            slab.SetPosition(Vector2d.Zero);
            CollisionDetectionMixed.TryBuildManifold(mesh, stadium, manifold).Should().Be(phase <= 0,
                "the literal vertical extent touches at two and separates beyond it");
        }
    }

    private static LSMeshCollider CreateSurfaceReductionFloor(int topology, bool vertical = false)
    {
        Vector3d[] vertices = topology == 5
            ? new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2), Vector3d.Zero, new Vector3d(0, 0, -2) }
            : topology == 4 ? new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2), Vector3d.Zero }
            : new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) };
        int[] triangles = topology switch
        {
            1 => new[] { 3, 2, 1, 1, 2, 0 }, // Reversed winding and authored triangle order.
            2 => new[] { 0, 2, 1, 3, 2, 1 }, // Mixed authored winding.
            3 => new[] { 0, 3, 1, 0, 2, 3 }, // Opposite diagonal.
            4 => new[] { 0, 4, 1, 1, 4, 3, 3, 4, 2, 2, 4, 0 }, // Referenced interior fan vertex.
            5 => new[] { 0, 4, 5, 5, 4, 1, 1, 4, 3, 3, 4, 2, 2, 4, 0 }, // Collinear perimeter subdivision.
            _ => new[] { 0, 2, 1, 1, 2, 3 }
        };
        return CreateSurfaceReductionMesh(vertices, triangles, vertical);
    }

    private static LSMeshCollider CreateSurfaceReductionRing(bool vertical = false) => CreateSurfaceReductionMesh(
        new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2),
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) },
        new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 },
        vertical);

    private static LSMeshCollider CreateSurfaceReductionMesh(Vector3d[] vertices, int[] triangles, bool vertical)
    {
        if (vertical)
            for (int index = 0; index < vertices.Length; index++)
                vertices[index] = new Vector3d(Fixed64.Zero, vertices[index].X, vertices[index].Z);
        return new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }

    private static SolidBody2D CreateSurfaceReductionSlab(GravitasWorldContext context, ColliderType2D shape, Vector2d position,
        Fixed64? hostY = null, Fixed64? halfThickness = null)
    {
        LSCollider2D collider = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.FromFraction(5, 4)),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            _ => new LSPolygonCollider2D(new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half), new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half))
        };
        collider.MixedHalfThicknessOverride = halfThickness ?? Fixed64.Half;
        // Bind the actual host Y before initialization so mixed cached bounds agree.
        var agent = new TestMatterAgent(context,
            new FixedTransform(new Vector3d(position.X, hostY ?? -Fixed64.Quarter, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        body.Initialize(position, Fixed64.Zero);
        return body;
    }
}
