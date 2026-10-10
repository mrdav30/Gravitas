//=======================================================================
// MeshConeSurfaceManifoldTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeSurfaceManifoldTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TwoWalls_ShouldRetainBothIndependentNormals(bool reversed)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateWalls(reversed);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, new Vector3d(Fixed64.FromFraction(3, 10), Fixed64.Zero, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool right = false, forward = false;
        foreach (ManifoldContact contact in pair.Manifold)
        {
            right |= contact.Normal == Vector3d.Right;
            forward |= contact.Normal == Vector3d.Forward;
        }
        (right && forward).Should().BeTrue("both genuine wall constraints must survive reduction");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void FullLoop_ShouldBlockApproachIntoBothWalls(bool reversed, bool separateColliders)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        for (int wall = 0; wall < (separateColliders ? 2 : 1); wall++)
        {
            LSMeshCollider mesh = CreateWalls(reversed, separateColliders ? wall : -1);
            mesh.Material = PhysicsMaterial.Frictionless;
            scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        }
        SolidBody body = scenario.CreateCone(new Vector3d(Fixed64.FromFraction(3, 10), Fixed64.Zero,
            Fixed64.FromFraction(3, 10)), preventAngularForces: true).Body;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        body.UseManualGrounding();
        body.AddLinearImpulse(new Vector3d(-1, 0, -1));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        body.LinearVelocity.Z.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CommonRigidPose_ShouldPreserveSurfaceGroupsAndContactIdentities(bool rotate)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateWalls(false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        Vector3d conePosition = new(Fixed64.FromFraction(3, 10), Fixed64.Zero, Fixed64.FromFraction(3, 10));
        scenario.InitializeStaticCollider(cone, conePosition);
        CollisionPair original = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(original));
        LSMeshCollider translatedMesh = CreateWalls(true);
        Vector3d translation = new(13, -7, 5);
        // This exact unit quaternion permutes the axes, avoiding a rounding
        // change to the geometry while exercising both participants' frames.
        FixedQuaternion rotation = rotate
            ? new(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        translatedMesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(translation, rotation, Vector3d.One)));
        var translatedCone = new LSConeCollider();
        translatedCone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(rotation * conePosition + translation, rotation, Vector3d.One)));
        CollisionPair translated = scenario.CreatePair(translatedMesh, translatedCone);
        Assert.True(CollisionDetection.DoCollisionCheck(translated));
        Assert.Equal(original.Manifold.GroupCount, translated.Manifold.GroupCount);
        Assert.Equal(original.Manifold.Select(c => c.ContactId), translated.Manifold.Select(c => c.ContactId));
        for (int index = 0; index < original.Manifold.Count; index++)
        {
            Assert.Equal(rotation * original.Manifold[index].Normal, translated.Manifold[index].Normal);
            Assert.Equal(original.Manifold[index].Depth, translated.Manifold[index].Depth);
            Assert.Equal(rotation * original.Manifold[index].PointA + translation, translated.Manifold[index].PointA);
            Assert.Equal(rotation * original.Manifold[index].PointB + translation, translated.Manifold[index].PointB);
        }
    }

    [Fact]
    public void RemoteAuthoredTriangle_ShouldPreserveTheSameLocalWorldSurfaceConstraints()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider baseline = CreateWalls(false), shiftedOrigin = CreateWalls(false, remoteBoundsTriangle: true);
        scenario.InitializeStaticCollider(baseline, Vector3d.Zero);
        scenario.InitializeStaticCollider(shiftedOrigin, Vector3d.Zero);
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, new(Fixed64.FromFraction(3,10), Fixed64.Zero, Fixed64.FromFraction(3,10)));
        CollisionPair original = scenario.CreatePair(baseline, cone), changed = scenario.CreatePair(shiftedOrigin, cone);
        Assert.NotEqual(baseline.Mesh.LocalBounds.Center, shiftedOrigin.Mesh.LocalBounds.Center);
        Assert.True(CollisionDetection.DoCollisionCheck(original));
        Assert.True(CollisionDetection.DoCollisionCheck(changed));
        Assert.Equal(original.Manifold.GroupCount, changed.Manifold.GroupCount);
        // A remote nonintersecting triangle changes the authored bounds origin,
        // preserving both actual local walls. Compare physical rows independently
        // of cache identity because rebuilding authored geometry invalidates it.
        var expected = original.Manifold.OrderBy(c => c.PointA.X).ThenBy(c => c.PointA.Y).ThenBy(c => c.PointA.Z)
            .ThenBy(c => c.PointB.X).ThenBy(c => c.PointB.Y).ThenBy(c => c.PointB.Z)
            .Select(c => (c.PointA, c.PointB, c.Normal, c.Depth));
        var actual = changed.Manifold.OrderBy(c => c.PointA.X).ThenBy(c => c.PointA.Y).ThenBy(c => c.PointA.Z)
            .ThenBy(c => c.PointB.X).ThenBy(c => c.PointB.Y).ThenBy(c => c.PointB.Z)
            .Select(c => (c.PointA, c.PointB, c.Normal, c.Depth));
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void ConnectedUNotch_ShouldKeepSectionsDisconnectedWhenItsBridgeMissesTheCone()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        // The entire U is one trusted planar surface. Its remote bridge lies
        // beyond the narrow cone section, so the admitted arms need separate
        // regional exits, including opposite normals.
        Vector3d[] vertices = { new(-2,0,-2), new(-1,0,-2), new(-1,0,1), new(-1,0,2), new(-2,0,2),
            new(1,0,-2), new(2,0,-2), new(2,0,2), new(1,0,2), new(1,0,1) };
        int[] triangles = { 0,1,2, 0,2,4, 2,3,4, 5,6,9, 6,7,9, 7,8,9, 2,9,8, 2,8,3 };
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        for (int triangle = 0; triangle < mesh.Mesh.TriangleCount; triangle++)
            Assert.Equal(0, mesh.Mesh.GetManifoldSurfaceOwner(triangle));
        // R=1 gives a plane-section X extent near 1.79; R=1/2 would miss
        // both arms. The section still cannot reach the bridge at Z>=1.
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2,1000,2) };
        FixedQuaternion tilt = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3,5), Fixed64.FromFraction(4,5));
        cone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, tilt, Vector3d.One)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        int faces = 0;
        bool left = false, right = false;
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ContactGroup group = pair.Manifold.GetGroup(groupIndex);
            if (group.Key.Region < 0) continue;
            faces++;
            for (int point = 0; point < group.Count; point++)
            {
                ManifoldContact contact = group[point];
                Assert.Equal(Fixed64.Zero, contact.PointA.Y);
                Assert.True(FixedMath.Abs(contact.PointA.X) >= Fixed64.One);
                Assert.True(FixedMath.Abs(contact.PointA.Z) <= Fixed64.Half);
                Assert.Equal(contact.PointA.X < Fixed64.Zero ? Vector3d.Up : Vector3d.Down, contact.Normal);
                left |= contact.PointA.X < Fixed64.Zero;
                right |= contact.PointA.X > Fixed64.Zero;
            }
        }
        Assert.Equal(2, faces);
        Assert.True(left && right);
    }

    [Fact]
    public void LongTiltedInterior_ShouldKeepTheWholeRegionMaximumInItsFiniteFaceGroup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2,1000,2) };
        FixedQuaternion tilt = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3,5), Fixed64.FromFraction(4,5));
        cone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, tilt, Vector3d.One)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        ContactGroup face = default;
        int faceCount = 0;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            if (pair.Manifold.GetGroup(group).Key.Region >= 0)
            { face = pair.Manifold.GetGroup(group); faceCount++; }
        Assert.Equal(1, faceCount);
        Assert.Equal(ContactManifold.MaxContactsPerGroup, face.Count);
        ManifoldContact deepest = face[0];
        for (int index = 0; index < face.Count; index++)
        {
            ManifoldContact contact = face[index];
            Assert.Equal(Vector3d.Down, contact.Normal);
            Assert.Equal(Fixed64.Zero, contact.PointA.Y);
            Assert.True(FixedMath.Abs(contact.PointA.X) <= (Fixed64)2);
            Assert.True(FixedMath.Abs(contact.PointA.Z) <= (Fixed64)2);
            Assert.Equal(contact.PointA.X, contact.PointB.X);
            Assert.Equal(contact.PointA.Z, contact.PointB.Z);
            Assert.Equal(contact.Depth, contact.PointB.Y);
            if (contact.Depth > deepest.Depth) deepest = contact;
        }
        // Same bounds as the exact face-region regression: the midpoint ray
        // is roughly half the required maximum near the lateral boundary.
        Assert.InRange(deepest.Depth.m_rawValue, Fixed64.One.m_rawValue, Fixed64.FromFraction(11,10).m_rawValue);
        Assert.True(FixedMath.Abs(deepest.PointA.X) > Fixed64.One);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void FiniteGeneratorTouch_ShouldDistinguishItsRawNeighborGap(int offsetRaw)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices = { new(Fixed64.Quarter,-Fixed64.Half,-Fixed64.One),
            new(-Fixed64.Quarter,Fixed64.Half,-Fixed64.One), new(-Fixed64.Quarter,Fixed64.Half,Fixed64.One),
            new(Fixed64.Quarter,-Fixed64.Half,Fixed64.One) };
        var mesh = new LSMeshCollider(vertices, new[] { 0,1,2, 0,2,3 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.Half + Fixed64.FromRaw(offsetRaw), Fixed64.Zero, Fixed64.Zero));
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2,2,2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.Equal(offsetRaw <= 0, CollisionDetection.DoCollisionCheck(pair));
        if (offsetRaw != 0) return;
        Assert.True(pair.Manifold.Count >= 2);
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ContactGroup group = pair.Manifold.GetGroup(groupIndex);
            for (int point = 0; point < group.Count; point++)
            {
                ManifoldContact contact = group[point];
                Assert.True(contact.Depth == Fixed64.Zero,
                    $"A tangent generator has no positive-depth constraint: region {group.Key.Region}, "
                    + $"normal {contact.Normal}, depth {contact.Depth}, mesh {contact.PointA}, cone {contact.PointB}.");
                Assert.Equal(contact.PointA, contact.PointB);
                Assert.Equal(Fixed64.Zero, contact.PointA.Z);
                Assert.InRange(contact.PointA.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
                Assert.Equal((Fixed64.One - contact.PointA.Y) / 2, contact.PointA.X);
            }
        }
        Assert.Contains(pair.Manifold, c => c.PointA.Y == -Fixed64.Half);
        Assert.Contains(pair.Manifold, c => c.PointA.Y == Fixed64.Half);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void BaseCapAndRimTouch_ShouldDistinguishTheirRawNeighborGap(int offsetRaw)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.Zero, -Fixed64.One + Fixed64.FromRaw(offsetRaw), Fixed64.Zero));
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2,2,2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.Equal(offsetRaw >= 0, CollisionDetection.DoCollisionCheck(pair));
        if (offsetRaw != 0) return;
        Assert.Equal(1, pair.Manifold.GroupCount);
        Assert.Equal(ContactManifold.MaxContactsPerGroup, pair.Manifold.Count);
        foreach (ManifoldContact contact in pair.Manifold)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.Equal(contact.PointA, contact.PointB);
            Assert.Equal(-Fixed64.One, contact.PointA.Y);
            Assert.Equal(Vector3d.Up, contact.Normal);
            Fixed64 radialSquared = contact.PointA.X * contact.PointA.X + contact.PointA.Z * contact.PointA.Z;
            Assert.InRange(radialSquared.m_rawValue, Fixed64.Zero.m_rawValue, (Fixed64.One + Fixed64.FromRaw(4)).m_rawValue);
        }
        Assert.Contains(pair.Manifold, c => FixedMath.Abs(c.PointA.X * c.PointA.X
            + c.PointA.Z * c.PointA.Z - Fixed64.One) <= Fixed64.FromRaw(4));
    }

    [Fact]
    public void ScaleRefreshAndFailedPreparation_ShouldKeepOnlyCommittedGeometryAndContactIdentities()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        SolidBody wall = scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true).Body;
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, new(Fixed64.One, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        ulong[] identities = pair.Manifold.Select(c => c.ContactId).ToArray();
        Vector3d[] witnesses = pair.Manifold.Select(c => c.PointA).ToArray();
        var bounds = mesh.Bounds;
        wall.PositionTransform.LocalScale = new(Fixed64.Quarter, Fixed64.One, Fixed64.One);
        Assert.True(mesh.RebuildRuntimeShapeOnly());
        Assert.False(CollisionDetection.DoCollisionCheck(pair));
        wall.PositionTransform.LocalScale = Vector3d.One;
        Assert.True(mesh.RebuildRuntimeShapeOnly());
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(identities, pair.Manifold.Select(c => c.ContactId));
        wall.PositionTransform.LocalScale = new(Fixed64.MinIncrement, Fixed64.One, Fixed64.MinIncrement);
        Assert.Throws<ArgumentException>(() => mesh.RebuildRuntimeShapeOnly());
        Assert.Equal(bounds, mesh.Bounds);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(identities, pair.Manifold.Select(c => c.ContactId));
        Assert.Equal(witnesses, pair.Manifold.Select(c => c.PointA));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ThinTab_FullLoopShouldBlockTheFaceWithoutInventingACoveredSeamConstraint(bool originalReproduction)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        // Keep the original issue's exact dimensions alongside the longer-tab
        // control: both unions must suppress their covered square/tab seam.
        Fixed64 side = Fixed64.FromFraction(1, originalReproduction ? 5 : 10);
        Fixed64 tab = Fixed64.FromFraction(1, originalReproduction ? 20 : 32);
        Fixed64 extensionEnd = originalReproduction ? Fixed64.Quarter : Fixed64.Half;
        Vector3d[] vertices = { new(-side,Fixed64.Zero,-side), new(side,Fixed64.Zero,-side),
            new(side,Fixed64.Zero,-tab), new(extensionEnd,Fixed64.Zero,-tab),
            new(extensionEnd,Fixed64.Zero,tab), new(side,Fixed64.Zero,tab),
            new(side,Fixed64.Zero,side), new(-side,Fixed64.Zero,side) };
        var mesh = new LSMeshCollider(vertices, new[] { 0,1,2, 0,2,5, 0,5,6, 0,6,7, 2,3,4, 2,4,5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        SolidBody body = scenario.CreateCone(Vector3d.Down * Fixed64.Quarter, preventAngularForces: true).Body;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        body.UseManualGrounding();
        CollisionPair pair = scenario.CreatePair(mesh, body.Collider);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Contains(pair.Manifold, c => c.Normal == Vector3d.Down && c.Depth == Fixed64.Quarter);
        int boundaryRows = 0;
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ContactGroup group = pair.Manifold.GetGroup(groupIndex);
            if (group.Key.Region >= 0) continue;
            for (int index = 0; index < group.Count; index++)
            {
                Vector3d point = group[index].PointA;
                Assert.Equal(Fixed64.Zero, point.Y);
                // X=side, |Z|<tab is covered by the attached tab, never an
                // exposed edge. Only the actual square/tab perimeter supports
                // an independent boundary row.
                bool square = (point.X == -side && FixedMath.Abs(point.Z) <= side)
                    || (FixedMath.Abs(point.Z) == side && FixedMath.Abs(point.X) <= side)
                    || (point.X == side && FixedMath.Abs(point.Z) >= tab && FixedMath.Abs(point.Z) <= side);
                bool extension = (FixedMath.Abs(point.Z) == tab && point.X >= side && point.X <= extensionEnd)
                    || (point.X == extensionEnd && FixedMath.Abs(point.Z) <= tab);
                Assert.True(square || extension);
                boundaryRows++;
            }
        }
        // The original square covers the entire cone section; its covered
        // triangulation seam must yield only the face group. The longer-tab
        // control intersects its actual perimeter and requires boundary rows.
        if (originalReproduction) Assert.Equal(0, boundaryRows);
        else Assert.True(boundaryRows > 0);
        body.AddLinearImpulse(Vector3d.Up * body.Mass);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        Assert.True(body.LinearVelocity.Y <= Fixed64.Zero);
        Assert.True(body.Position3d.Y <= -Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoledSurface_ShouldRejectTheEmptyCenterAndRetainOnlyRealDomainWitnesses(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-2,0,-2), new Vector3d(2,0,-2),
            new Vector3d(2,0,2), new Vector3d(-2,0,2), new Vector3d(-1,0,-1),
            new Vector3d(1,0,-1), new Vector3d(1,0,1), new Vector3d(-1,0,1) };
        int[] triangles = { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 };
        if (reverse)
            for (int index = 0; index < triangles.Length; index += 3)
                (triangles[index + 1], triangles[index + 2]) = (triangles[index + 2], triangles[index + 1]);
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var small = new LSConeCollider();
        scenario.InitializeStaticCollider(small, Vector3d.Zero);
        Assert.False(CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, small)));
        var wide = new LSConeCollider { Radius = (Fixed64)4 };
        scenario.InitializeStaticCollider(wide, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, wide);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        foreach (ManifoldContact contact in pair.Manifold)
        {
            Assert.Equal(Fixed64.Zero, contact.PointA.Y);
            Assert.True(FixedMath.Abs(contact.PointA.X) >= Fixed64.One - Fixed64.FromRaw(1)
                || FixedMath.Abs(contact.PointA.Z) >= Fixed64.One - Fixed64.FromRaw(1));
            Assert.True(contact.Normal.Y <= Fixed64.Zero);
        }
    }

    [Fact]
    public void DisconnectedSupports_ShouldRetainSpatialCoverageOnBothIslands()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateDisconnectedSupports();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = (Fixed64)2 };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.True(pair.Manifold.GroupCount >= 2);
        Assert.Contains(pair.Manifold, c => c.PointA.X < Fixed64.Zero);
        Assert.Contains(pair.Manifold, c => c.PointA.X > Fixed64.Zero);
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.InRange(pair.Manifold.GetGroupContactCount(group), 1, ContactManifold.MaxContactsPerGroup);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void FullLoop_DisconnectedSupports_ShouldOpposeAngularApproachOnEitherIsland(int direction)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateDisconnectedSupports();
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        var cone = new LSConeCollider { Radius = (Fixed64)2, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scenario.CreateBody(cone, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        body.UseManualGrounding();
        // Pin the center so only the off-center normal lever can stop rotation.
        // Reversing the angular approach makes the other island responsible.
        body.FreezeAxes = BodyFreezeAxes3D.Position | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        Fixed64 initial = Fixed64.Quarter * (Fixed64)direction;
        body.ApplyCollisionAngularVelocityDelta(Vector3d.Forward * initial);
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Contains(pair.Manifold, c => c.PointA.X < Fixed64.Zero && c.Normal == Vector3d.Up);
        Assert.Contains(pair.Manifold, c => c.PointA.X > Fixed64.Zero && c.Normal == Vector3d.Up);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Assert.True(FixedMath.Abs(body.AngularVelocity.Z) < FixedMath.Abs(initial));
        Assert.Equal(Vector3d.Zero, body.LinearVelocity);
        Assert.Equal(Fixed64.Zero, body.AngularVelocity.X);
        Assert.Equal(Fixed64.Zero, body.AngularVelocity.Y);
    }

    [Fact]
    public void JoinedWalls_WarmedGenerationShouldRetainCapacityWithoutAllocating()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateWalls(false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, new Vector3d(Fixed64.FromFraction(3,10), Fixed64.Zero, Fixed64.FromFraction(3,10)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair)) throw new InvalidOperationException("Joined walls lost contact.");
        }, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 4);
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void FullLoop_GeneratedMeshConeGroups_ShouldReplayGroundSleepWakeAndNotifyOnce()
    {
        using PhysicsScenarioBuilder first = CreateGroundedCorner(out LSMeshCollider mesh, out SolidBody body);
        using PhysicsScenarioBuilder second = CreateGroundedCorner(out LSMeshCollider repeatedMesh, out SolidBody repeatedBody);
        int meshEnter = 0, coneEnter = 0, meshExit = 0, coneExit = 0;
        mesh.OnContactEnter += other => { Assert.Same(body, other); meshEnter++; };
        body.Collider.OnContactEnter += other => { Assert.Same(mesh.Body, other); coneEnter++; };
        mesh.OnContactExit += _ => meshExit++;
        body.Collider.OnContactExit += _ => coneExit++;
        for (int frame = 0; frame < 4; frame++)
        {
            if (frame == 2)
            {
                body.AddLinearImpulse(Vector3d.Forward * Fixed64.Quarter);
                repeatedBody.AddLinearImpulse(Vector3d.Forward * Fixed64.Quarter);
                Assert.False(body.IsSleeping);
                Assert.False(repeatedBody.IsSleeping);
            }
            first.Context.Simulate();
            first.Context.LateSimulate();
            second.Context.Simulate();
            second.Context.LateSimulate();
            Assert.True(body.IsGrounded);
            Assert.Equal(Vector3d.Up, body.GroundNormal);
            Assert.Equal(first.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches),
                second.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
            if (frame == 0)
            {
                CollisionPair pair = first.Context.Physics.GetCollisionPair(mesh.Id, body.Collider.Id)!;
                Assert.True(pair.Manifold.GroupCount >= 2);
                CollisionPair repeatedPair = second.Context.Physics.GetCollisionPair(repeatedMesh.Id, repeatedBody.Collider.Id)!;
                Assert.True(repeatedPair.Manifold.GroupCount >= 2);
            }
            if (frame == 1)
            {
                Assert.True(body.IsSleeping);
                Assert.True(repeatedBody.IsSleeping);
            }
        }
        body.Deactivate();
        repeatedBody.Deactivate();
        Assert.Equal(1, meshEnter);
        Assert.Equal(1, coneEnter);
        Assert.Equal(1, meshExit);
        Assert.Equal(1, coneExit);
        Assert.Equal(first.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches),
            second.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    private static PhysicsScenarioBuilder CreateGroundedCorner(out LSMeshCollider mesh, out SolidBody body)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        Vector3d[] vertices = { new(0,0,-2), new(4,0,-2), new(4,0,2), new(0,0,2),
            new(0,4,-2), new(0,4,2) };
        mesh = new(vertices, new[] { 0,3,1, 1,3,2, 0,4,3, 3,4,5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        // Put the host body origin at its feet, matching automatic grounding's
        // height contract, while the cone's base exactly touches the floor.
        var cone = new LSConeCollider { LocalOffset = Vector3d.Up * Fixed64.Half,
            Material = PhysicsMaterial.Frictionless };
        body = scenario.CreateBody(cone, Vector3d.Right * Fixed64.Half, FixedQuaternion.Identity,
            preventAngularForces: true).Body;
        body.SleepFrameThreshold = 2;
        return scenario;
    }

    private static LSMeshCollider CreateDisconnectedSupports()
    {
        Fixed64 near = Fixed64.Quarter, far = Fixed64.FromFraction(3,4);
        Vector3d[] vertices = { new(-far,Fixed64.Zero,-near), new(-near,Fixed64.Zero,-near),
            new(-near,Fixed64.Zero,near), new(-far,Fixed64.Zero,near), new(near,Fixed64.Zero,-near),
            new(far,Fixed64.Zero,-near), new(far,Fixed64.Zero,near), new(near,Fixed64.Zero,near) };
        return new(vertices, new[] { 0,1,2, 0,2,3, 4,5,6, 4,6,7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider CreateWalls(bool reversed, int wall = -1, bool remoteBoundsTriangle = false)
    {
        // Two finite walls meet at one shared crease, enclosing the positive
        // X/Z quadrant. This is ordinary connected concave surface geometry.
        Vector3d[] vertices = { new(0, -4, 0), new(0, 4, 0), new(0, 4, 4), new(0, -4, 4),
            new(4, -4, 0), new(4, 4, 0) };
        int[] triangles = { 0, 1, 2, 0, 2, 3, 0, 4, 5, 0, 5, 1 };
        if (remoteBoundsTriangle)
        {
            vertices = vertices.Concat(new[] { new Vector3d(12,12,12), new Vector3d(14,12,12), new Vector3d(12,12,14) }).ToArray();
            triangles = triangles.Concat(new[] { 6,7,8 }).ToArray();
        }
        if (wall >= 0)
        {
            Vector3d[] selected = new Vector3d[4];
            int[] indices = wall == 0 ? new[] { 0, 1, 2, 3 } : new[] { 0, 4, 5, 1 };
            for (int i = 0; i < selected.Length; i++)
                selected[i] = vertices[indices[i]];
            vertices = selected;
            triangles = new[] { 0, 1, 2, 0, 2, 3 };
        }
        if (reversed)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        return new(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
