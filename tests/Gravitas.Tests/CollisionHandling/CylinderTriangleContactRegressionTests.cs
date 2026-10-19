using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CylinderTriangleContactRegressionTests
{
    [Fact]
    public void CylinderOutsidePatchCorner_ShouldRejectDespiteOverlappingBounds()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1),
            new Vector3d(-1, 0, 1), new Vector3d(1, 0, 1)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter, Fixed64.One));
        Fixed64 coordinate = Fixed64.FromFraction(5, 4);
        scene.InitializeStaticCollider(cylinder, new(coordinate, Fixed64.Quarter, coordinate));
        mesh.Bounds.Intersects(cylinder.Bounds).Should().BeTrue();
        var pair = scene.CreatePair(mesh, cylinder);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Should().BeEmpty();
    }

    [Fact]
    public void FilledPatchScratch_ShouldNotRetainSamplesAcrossTiltAndDeclarationChanges()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var floor = new LSMeshCollider(new[]
        {
            new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
        scene.InitializeStaticCollider(floor, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter, Fixed64.One));
        SolidBody body = scene.CreateBody(cylinder, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        var pair = scene.CreatePair(floor, cylinder);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        var expected = pair.Manifold.Select(row => (row.PointA, row.PointB, row.Normal, row.Depth)).ToArray();

        body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30));
        cylinder.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Should().OnlyContain(row => row.Normal == Vector3d.Up && row.Depth > Fixed64.Quarter);
        using (scene.Context.Physics.CreateStaticSurface(new[] { floor }))
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        body.SetRotation(FixedQuaternion.Identity);
        cylinder.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Select(row => (row.PointA, row.PointB, row.Normal, row.Depth)).Should().Equal(expected);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TiltedCylinderOnAffinePatch_ShouldKeepFiniteSupportsAndClearTheWholePlane(bool tinyRadius)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, -Fixed64.One),
            new Vector3d(-1, 0, 1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.One)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity, new Vector3d(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5)))));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(
            tinyRadius ? Fixed64.MinIncrement : Fixed64.Quarter,
            Fixed64.One + (tinyRadius ? Fixed64.FromRaw(3) : Fixed64.Zero)));
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        FixedQuaternion rotation = tinyRadius
            ? FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi / 4)
            : FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi / 6);
        SolidBody body = scene.CreateBody(cylinder, start, rotation).Body;
        var pair = scene.CreatePair(mesh, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        Fixed64 maximum = Fixed64.Zero;
        foreach (ManifoldContact row in pair.Manifold)
        {
            row.Normal.Should().Be(Vector3d.Up);
            row.PointA.Y.Should().Be(Fixed64.Zero);
            row.PointA.X.Should().BeInRange(-Fixed64.FromFraction(2, 3), Fixed64.FromFraction(5, 6));
            row.PointA.Z.Should().BeInRange(-Fixed64.FromFraction(3, 5), Fixed64.FromFraction(3, 5));
            Vector3d local = rotation.Inverse() * (row.PointB - start);
            local.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half + Fixed64.FromRaw(4));
            local.X.Abs().Should().BeLessThanOrEqualTo(cylinder.ScaledRadius + Fixed64.FromRaw(4));
            local.Z.Abs().Should().BeLessThanOrEqualTo(cylinder.ScaledRadius + Fixed64.FromRaw(4));
            (local.X * local.X + local.Z * local.Z).Should().BeLessThanOrEqualTo(
                cylinder.ScaledRadius * cylinder.ScaledRadius + Fixed64.FromRaw(4));
            maximum = FixedMath.Max(maximum, row.Depth);
        }
        maximum.Should().BeGreaterThan(Fixed64.Zero);
        body.SetPosition(start + Vector3d.Up * (maximum + Fixed64.FromRaw(2)));
        cylinder.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(scene.CreatePair(mesh, cylinder)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderOnPatchAndIsolatedWall_ShouldKeepBothSurfaceConstraints(bool splitFloor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var vertices = new[]
        {
            new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2),
            new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2)
        };
        var mesh = new LSMeshCollider(splitFloor ? vertices : new[] { vertices[0], new Vector3d(4, 0, -2), new Vector3d(-2, 0, 4), vertices[4], vertices[5], vertices[6] }, splitFloor
            ? new[] { 0, 2, 1, 1, 2, 3, 4, 5, 6 } : new[] { 0, 2, 1, 3, 4, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter, Fixed64.One));
        scene.InitializeStaticCollider(cylinder, new(Fixed64.FromFraction(1, 16), Fixed64.Quarter, Fixed64.Zero));
        var pair = scene.CreatePair(mesh, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.GroupCount.Should().Be(2);
        bool floor = false, wall = false;
        foreach (ManifoldContact row in pair.Manifold)
        {
            floor |= row.Normal == Vector3d.Up;
            wall |= row.Normal == Vector3d.Right;
        }
        floor.Should().BeTrue();
        wall.Should().BeTrue();
    }

    [Theory]
    [InlineData(1L, -1L, 2L)]
    [InlineData(3L, 1L, 0L)]
    public void CylinderOnFilledPatch_ShouldKeepExactOddHeightAndMaterials(
        long heightOffset, long planeOffset, long depthRaw)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 y = (Fixed64)5 + Fixed64.FromRaw(planeOffset);
        var floor = new LSMeshCollider(new[]
        {
            new Vector3d(-(Fixed64)2, y, -(Fixed64)2), new Vector3d((Fixed64)2, y, -(Fixed64)2),
            new Vector3d(-(Fixed64)2, y, (Fixed64)2), new Vector3d((Fixed64)2, y, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
        floor.Material = PhysicsMaterial.Default;
        scene.InitializeStaticCollider(floor, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter,
            (Fixed64)10 + Fixed64.FromRaw(heightOffset))) { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(cylinder, Vector3d.Zero);
        var pair = scene.CreatePair(floor, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        foreach (ManifoldContact row in pair.Manifold)
        {
            row.Normal.Should().Be(Vector3d.Down);
            row.Depth.Should().Be(Fixed64.FromRaw(depthRaw));
            row.MaterialA.Should().Be(floor.Material);
            row.MaterialB.Should().Be(cylinder.Material);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderInsideMeshHole_ShouldStaySeparatedAndKeepItsTrueRim(bool touchRim)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var vertices = new System.Collections.Generic.List<Vector3d>();
        var triangles = new System.Collections.Generic.List<int>();
        Fixed64[] coordinates = { -Fixed64.Two, -Fixed64.Half, Fixed64.Half, Fixed64.Two };
        foreach (Fixed64 z in coordinates)
            foreach (Fixed64 x in coordinates) vertices.Add(new(x, Fixed64.Zero, z));
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
            {
                if (x == 1 && z == 1) continue;
                int a = z * 4 + x;
                triangles.AddRange(new[] { a, a + 4, a + 1, a + 1, a + 4, a + 5 });
            }
        var mesh = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(), MeshColliderMode.Concave);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter, Fixed64.One));
        Vector3d start = new(touchRim ? Fixed64.FromFraction(3, 8) : Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        SolidBody body = scene.CreateBody(cylinder, start, FixedQuaternion.Identity).Body;
        var pair = scene.CreatePair(mesh, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().Be(touchRim);
        if (!touchRim) return;
        bool rim = false;
        foreach (ManifoldContact row in pair.Manifold)
        {
            if (row.Normal != Vector3d.Left) continue;
            rim = true;
            row.Depth.Should().Be(Fixed64.FromFraction(1, 8));
            row.PointA.X.Should().Be(Fixed64.Half);
        }
        rim.Should().BeTrue();
        body.SetPosition(start + Vector3d.Left * (Fixed64.FromFraction(1, 8) + Fixed64.MinIncrement));
        cylinder.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(scene.CreatePair(mesh, cylinder)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CylinderOnFilledPatch_ShouldIgnoreTheCoveredDiagonalAndResolveVertically(bool reverseTriangles, bool reversePair)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = Fixed64.Zero;
        var floor = new LSMeshCollider(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, -(Fixed64)1),
            new Vector3d(-1, 0, 1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.One)
        }, reverseTriangles ? new[] { 1, 2, 3, 0, 2, 1 } : new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(floor, Vector3d.Zero);
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Quarter, Fixed64.One))
        { Material = PhysicsMaterial.Frictionless };
        Vector3d start = Vector3d.Up * Fixed64.FromFraction(1, 10);
        SolidBody body = scene.CreateBody(cylinder, start, FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        CollisionPair pair = reversePair ? scene.CreatePair(cylinder, floor) : scene.CreatePair(floor, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        Fixed64 depth = Fixed64.Half - start.Y;
        foreach (ManifoldContact row in pair.Manifold)
        {
            row.Normal.Should().Be(Vector3d.Up);
            row.Depth.Should().Be(depth);
            row.PointA.Y.Should().Be(Fixed64.Zero);
            row.PointB.Y.Should().Be(start.Y - Fixed64.Half);
            (row.PointB.X * row.PointB.X + row.PointB.Z * row.PointB.Z)
                .Should().BeLessThanOrEqualTo(Fixed64.Quarter * Fixed64.Quarter);
        }
        // The whole cylinder clears the filled plane after this exit, not
        // merely one triangle. Its analytic bottom is then one raw unit above it.
        body.SetPosition(start + Vector3d.Up * (depth + Fixed64.MinIncrement));
        cylinder.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(scene.CreatePair(floor, cylinder)).Should().BeFalse();
        body.SetPosition(start);
        cylinder.RebuildRuntimeShapeOnly();
        scene.Context.Simulate();
        scene.Context.LateSimulate();
        body.Position3d.X.Should().Be(Fixed64.Zero);
        body.Position3d.Z.Should().Be(Fixed64.Zero);
        body.Position3d.Y.Should().BeGreaterThan(start.Y);
    }

    [Fact]
    public void CylinderContact_ShouldFindTriangleIntrusionAwayFromCenterClosestPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, 7, -5), new Vector3d(10, 1, -5), new Vector3d(5, 4, 5)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        var surface = scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        var cylinder = scenario.CreateBody(
            new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10)),
            Vector3d.Zero, FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(surface.Collider, cylinder.Collider);

        // The triangle lies on 3x+5y=35. Its closest point to the cylinder
        // center has y=175/34>5 and is outside the upper cap. Nevertheless,
        // (4,23/5,0) lies in the triangle and strictly inside the cylinder.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        // Depth is the witness separation along the selected normal, not the
        // distance from an unrelated interior point to another cylinder face.
        Fixed64 projectedDepth = Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal);
        projectedDepth.m_rawValue.Should().BeInRange(
            contact.Depth.m_rawValue - 8, contact.Depth.m_rawValue + 8);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderTriangle_ObliqueRimContact_ShouldRetainMatchedWitnesses(bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CollisionPair pair = CreateRimPair(context, Vector3d.Zero, reverse);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(5, 16));
        // The edge midpoint is q=(77/16,19/4,0); p=(5,5,0) is on
        // the cap rim, and q-p=(5/16)*(-3/5,-4/5,0).
        AssertCoordinate(contact.Normal.X, Fixed64.FromFraction(-3, 5));
        AssertCoordinate(contact.Normal.Y, Fixed64.FromFraction(-4, 5));
        AssertCoordinate(contact.Normal.Z, Fixed64.Zero);
        AssertCoordinate(contact.PointA.X, Fixed64.FromFraction(77, 16));
        AssertCoordinate(contact.PointA.Y, Fixed64.FromFraction(19, 4));
        AssertCoordinate(contact.PointA.Z, Fixed64.Zero);
        AssertCoordinate(contact.PointB.X, (Fixed64)5);
        AssertCoordinate(contact.PointB.Y, (Fixed64)5);
        AssertCoordinate(contact.PointB.Z, Fixed64.Zero);

        ulong identity = contact.ContactId;
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.ContactId.Should().Be(identity);
    }

    [Fact]
    public void CylinderTriangle_WithUnsupportedRadius_ShouldRejectBeforeNarrowPhase()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cylinder = new LSCylinderCollider { Radius = Fixed64.MaxValue, Size = Vector3d.One };
        System.Action initialize = () => cylinder.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity, Vector3d.One)));
        initialize.Should().Throw<System.ArgumentException>().WithMessage("*geometry reach*");
        cylinder.Id.Should().Be(-1);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void CylinderTriangle_RimBoundary_ShouldDistinguishRawStepFromTouch(
        long verticalRawOffset, bool expected)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d offset = new(Fixed64.FromFraction(3, 16),
            Fixed64.FromFraction(1, 4) + Fixed64.FromRaw(verticalRawOffset), Fixed64.Zero);
        CollisionPair pair = CreateRimPair(context, offset);

        CollisionDetection.DoCollisionCheck(pair).Should().Be(expected);
        pair.Manifold.HasContact.Should().Be(expected);
        if (verticalRawOffset == 0)
            pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CylinderTriangle_ObliqueRimContact_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CollisionPair pair = CreateRimPair(context, Vector3d.Zero);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair))
                throw new InvalidOperationException("The rim contact disappeared during measurement.");
        }, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 4);
        allocated.Should().Be(0);
    }

    [Fact]
    public void CylinderTriangle_SmallCapFaceWithoutSampledSupports_ShouldKeepPrimaryContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 height = Fixed64.FromFraction(19, 4);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)2, height, (Fixed64)2),
            new Vector3d((Fixed64)3, height, (Fixed64)2),
            new Vector3d((Fixed64)2, height, (Fixed64)3)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(context));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context));
        var pair = new CollisionPair(mesh, cylinder);

        // None of the four rim cardinal points or cap center lies on this
        // triangle. Failure to enrich the manifold must not discard the hit.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Normal.Should().Be(Vector3d.Down);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.PointA.Y.Should().Be(height);
        contact.PointB.Y.Should().Be((Fixed64)5);
        AssertCoordinate(contact.PointA.X, contact.PointB.X);
        AssertCoordinate(contact.PointA.Z, contact.PointB.Z);
    }

    private static CollisionPair CreateRimPair(
        GravitasWorldContext context, Vector3d triangleOffset, bool reverse = false)
    {
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(triangleOffset, FixedQuaternion.Identity, Vector3d.One)));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context));
        return reverse ? new CollisionPair(cylinder, mesh) : new CollisionPair(mesh, cylinder);
    }

    [Fact]
    public void CylinderTriangle_NearlyCapParallelRim_ShouldNotBeEnrichedAsCapFace()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(Fixed64.FromFraction(-77, 256), Fixed64.FromFraction(351, 64), Fixed64.FromFraction(-85, 16)),
            new Vector3d(Fixed64.FromFraction(2611, 256), Fixed64.FromFraction(247, 64), Fixed64.FromFraction(85, 16)),
            new Vector3d(Fixed64.FromFraction(1475, 256), Fixed64.FromFraction(635, 64), Fixed64.Zero)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(context));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context));
        var pair = new CollisionPair(mesh, cylinder);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        // 84/85 exceeds the former 63/64 cap-alignment threshold, but
        // the contact belongs to an edge and rim, not two parallel faces.
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(85, 256));
        AssertCoordinate(contact.Normal.X, Fixed64.FromFraction(-13, 85));
        AssertCoordinate(contact.Normal.Y, Fixed64.FromFraction(-84, 85));
        AssertCoordinate(contact.Normal.Z, Fixed64.Zero);
        AssertCoordinate(contact.PointA.X, Fixed64.FromFraction(1267, 256));
        AssertCoordinate(contact.PointA.Y, Fixed64.FromFraction(299, 64));
        AssertCoordinate(contact.PointA.Z, Fixed64.Zero);
        AssertCoordinate(contact.PointB.X, (Fixed64)5);
        AssertCoordinate(contact.PointB.Y, (Fixed64)5);
        AssertCoordinate(contact.PointB.Z, Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderTriangle_DuplicateCapFaces_ShouldNotInsertFallbackIntoFullManifold(bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        CollisionPair single = CreateLargeCapPair(context, new[] { 0, 1, 2 });
        CollisionPair duplicate = CreateLargeCapPair(context, reverse
            ? new[] { 0, 2, 1, 0, 1, 2 } : new[] { 0, 1, 2, 0, 2, 1 });
        CollisionDetection.DoCollisionCheck(single).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(duplicate).Should().BeTrue();
        single.Manifold.Count.Should().Be(4);
        duplicate.Manifold.Count.Should().Be(4);
        for (int index = 0; index < 4; index++)
        {
            duplicate.Manifold[index].ContactId.Should().Be(single.Manifold[index].ContactId);
            duplicate.Manifold[index].PointA.Should().Be(single.Manifold[index].PointA);
            duplicate.Manifold[index].PointB.Should().Be(single.Manifold[index].PointB);
        }
    }

    private static CollisionPair CreateLargeCapPair(GravitasWorldContext context, int[] triangles)
    {
        Fixed64 y = Fixed64.FromFraction(19, 4);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-16), y, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, y, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, y, (Fixed64)16)
        }, triangles, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(context));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context));
        return new CollisionPair(mesh, cylinder);
    }

    [Theory]
    [InlineData(3L, 1L, 0L, false)]
    [InlineData(1L, -1L, 2L, false)]
    [InlineData(3L, 1L, 0L, true)]
    [InlineData(1L, -1L, 2L, true)]
    public void CylinderTriangle_EnrichedCap_ShouldRoundExactDepthOnlyOnce(
        long heightRawOffset, long triangleRawOffset, long expectedDepthRaw, bool affine)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 y = (Fixed64)5 + Fixed64.FromRaw(triangleRawOffset);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-16), y, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, y, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, y, (Fixed64)16),
            new Vector3d(affine ? Fixed64.FromFraction(17, 2) : (Fixed64)8, -y, (Fixed64)8),
            new Vector3d((Fixed64)12, -y, (Fixed64)8),
            new Vector3d((Fixed64)8, -y, (Fixed64)12)
        }, new[] { 0, 1, 2, 3, 4, 5 }, MeshColliderMode.Concave);
        // The remote half-unit X needs fractional scaling to preserve a
        // source frame, while the nearby horizontal face still fits the cap.
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            affine ? new Vector3d(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.One) : Vector3d.One);
        mesh.InitializeWithNoBody(new TestMatterAgent(context, transform));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(
            Fixed64.One, (Fixed64)10 + Fixed64.FromRaw(heightRawOffset)));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context));
        var pair = new CollisionPair(mesh, cylinder);

        // The remote lower triangle fixes the mesh's canonical origin at zero
        // without entering the cylinder's candidates. A lone horizontal face
        // would be recentered at y, masking intermediate support rounding.
        mesh.Mesh.Origin.Should().Be(Vector3d.Zero);
        cylinder.Height.Should().Be((Fixed64)10 + Fixed64.FromRaw(heightRawOffset));
        // The exact cap gap is 0.5 or 1.5 raw, rounding to even (0 or 2).
        // Rounding the cap anchor before measuring the gap instead gives 1.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(4);
        for (int index = 0; index < pair.Manifold.Count; index++)
        {
            ManifoldContact contact = pair.Manifold[index];
            contact.Depth.Should().Be(Fixed64.FromRaw(expectedDepthRaw));
            contact.DepthIsClamped.Should().BeFalse();
            contact.Normal.Should().Be(Vector3d.Down);
            contact.PointA.Y.Should().Be(y);
            contact.PointA.X.Should().Be(contact.PointB.X);
            contact.PointA.Z.Should().Be(contact.PointB.Z);
        }
    }

    // Final normal normalization and world-point materialization each round;
    // two raw units cover their composed Q32.32 rounding, not a geometric margin.
    private static void AssertCoordinate(Fixed64 actual, Fixed64 expected) =>
        actual.m_rawValue.Should().BeInRange(expected.m_rawValue - 2, expected.m_rawValue + 2);
}
