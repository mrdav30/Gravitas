using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class MeshTrianglePairContactTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedHullAgainstSeparatedTrustedPatch_ShouldRejectOverlappingBounds(bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        var sheet = new LSMeshCollider(new[]
        {
            new Vector3d(Fixed64.Half, Fixed64.One, -Fixed64.One), new(Fixed64.One, Fixed64.Half, -Fixed64.One),
            new(Fixed64.One, Fixed64.Half, Fixed64.One), new(Fixed64.Half, Fixed64.One, Fixed64.One)
        }, new[] { 0, 1, 2, 0, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        InitializeStaticMesh(context, hull, Vector3d.One);
        InitializeStaticMesh(context, sheet, Vector3d.One);
        var manifold = new ContactManifold();

        Assert.True(hull.Bounds.Intersects(sheet.Bounds));
        Assert.False(sheet.Mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty);
        Assert.False(CollisionDetection.DoCollisionCheck(CreateWorkItem(context,
            reverse ? hull : sheet, reverse ? sheet : hull, manifold)));
        Assert.False(manifold.HasContact);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ClosedHullAgainstApexTouchingSheet_ShouldRetainTheFiniteZeroDepthContact(bool hullFirst, bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var hull = new LSMeshCollider(new[] { new Vector3d(-1, -1, -1), new(1, -1, -1), new(0, 1, -1), new(0, 0, 1) },
            new[] { 0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3 }, MeshColliderMode.Convex)
        { Material = PhysicsMaterial.Bouncy };
        var sheet = new LSMeshCollider(new[] { new Vector3d(0, 0, 1), new(-1, -1, 3), new(0, -2, 5), new(1, -1, 3) },
            new[] { 0, 1, 2, 0, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        sheet.Material = PhysicsMaterial.Frictionless;
        InitializeStaticMesh(context, hullFirst ? hull : sheet, Vector3d.One);
        InitializeStaticMesh(context, hullFirst ? sheet : hull, Vector3d.One);
        Assert.False(sheet.Mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty);
        var manifold = new ContactManifold();

        Assert.True(CollisionDetection.DoCollisionCheck(CreateWorkItem(context,
            reverse ? hull : sheet, reverse ? sheet : hull, manifold)));
        Assert.Equal(1, manifold.GroupCount);
        Assert.NotEmpty(manifold);
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Fixed64.Zero, row.Depth);
            Assert.Equal(new Vector3d(0, 0, 1), row.PointA);
            Assert.Equal(row.PointA, row.PointB);
            Assert.True(row.HasMaterialOverride);
            Assert.Equal(reverse ? hull.Material : sheet.Material, row.MaterialA);
            Assert.Equal(reverse ? sheet.Material : hull.Material, row.MaterialB);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ClosedConcaveBoundaryAgainstConvexHull_ShouldRetainFiniteSurfaceContacts(bool hullFirst, bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        LSMeshCollider boundary = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave);
        Vector3d boundaryCenter = Vector3d.Right * Fixed64.FromFraction(3, 4);
        if (hullFirst) InitializeStaticMesh(context, hull, Vector3d.One);
        InitializeStaticMesh(context, boundary, boundaryCenter, Vector3d.One);
        if (!hullFirst) InitializeStaticMesh(context, hull, Vector3d.One);
        Assert.True(hull.Mesh.IsClosedSurface);
        Assert.True(boundary.Mesh.IsClosedSurface);
        var manifold = new ContactManifold();

        Assert.True(CollisionDetection.DoCollisionCheck(CreateWorkItem(context,
            reverse ? hull : boundary, reverse ? boundary : hull, manifold)));
        Assert.NotEmpty(manifold);
        foreach (ManifoldContact row in manifold)
        {
            AssertOnCube(reverse ? row.PointA : row.PointB, Vector3d.Zero);
            AssertOnCube(reverse ? row.PointB : row.PointA, boundaryCenter);
            Assert.InRange(row.Depth.m_rawValue, 0, Fixed64.Quarter.m_rawValue);
            Assert.False(row.HasMaterialOverride);
        }

        static void AssertOnCube(Vector3d point, Vector3d center)
        {
            Vector3d local = point - center;
            Assert.InRange(local.X.m_rawValue, -Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue);
            Assert.InRange(local.Y.m_rawValue, -Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue);
            Assert.InRange(local.Z.m_rawValue, -Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue);
            Assert.True(local.X.Abs() == Fixed64.Half || local.Y.Abs() == Fixed64.Half || local.Z.Abs() == Fixed64.Half);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ClosedHullAgainstConcaveAndConvexPatches_ShouldRetainBothOwnersAndMaterials(bool hullFirst, bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var sheet = new LSMeshCollider(new[]
        {
            new Vector3d(-1, 0, -1), new(1, 0, -1), new(1, 0, 0), new(0, 0, 0), new(0, 0, 1), new(-1, 0, 1),
            new Vector3d(Fixed64.Half, -Fixed64.One, -Fixed64.One), new(Fixed64.Half, Fixed64.One, -Fixed64.One),
            new(Fixed64.Half, Fixed64.One, Fixed64.One), new(Fixed64.Half, -Fixed64.One, Fixed64.One)
        }, new[] { 0, 1, 3, 1, 2, 3, 0, 3, 5, 3, 4, 5, 6, 7, 8, 6, 8, 9 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        hull.Material = PhysicsMaterial.Bouncy;
        Vector3d center = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter);
        if (hullFirst) InitializeStaticMesh(context, hull, center, Vector3d.One);
        InitializeStaticMesh(context, sheet, Vector3d.One);
        if (!hullFirst) InitializeStaticMesh(context, hull, center, Vector3d.One);
        Assert.True(sheet.Mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty);
        Assert.False(sheet.Mesh.GetConvexCoplanarPatchCornerVertexIndices(4).IsEmpty);
        var manifold = new ContactManifold();

        Assert.True(CollisionDetection.DoCollisionCheck(CreateWorkItem(context,
            reverse ? hull : sheet, reverse ? sheet : hull, manifold)));
        Assert.Equal(2, manifold.GroupCount);
        int concaveOrdinal = sheet.Mesh.GetCanonicalSurfaceOrdinal(0);
        int convexOrdinal = sheet.Mesh.GetCanonicalSurfaceOrdinal(4);
        Assert.NotEqual(concaveOrdinal, convexOrdinal);
        for (int index = 0; index < manifold.GroupCount; index++)
        {
            ref ContactGroup group = ref manifold.GetGroup(index);
            int ordinal = reverse ? group.Key.SurfaceB : group.Key.SurfaceA;
            Assert.True(ordinal == concaveOrdinal || ordinal == convexOrdinal);
            Assert.True(group.Count > 0);
            for (int sample = 0; sample < group.Count; sample++)
            {
                ManifoldContact row = group[sample];
                Vector3d target = reverse ? row.PointB : row.PointA;
                Vector3d native = reverse ? row.PointA : row.PointB;
                if (ordinal == concaveOrdinal)
                {
                    Assert.Equal(Fixed64.Zero, target.Y);
                    Assert.InRange(target.X.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                    Assert.InRange(target.Z.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                    Assert.True(target.X <= Fixed64.Zero || target.Z <= Fixed64.Zero);
                    // Triangle fallback retains its finite crossing depth; the cube extends 1/4 below the sheet.
                    Assert.InRange(row.Depth.m_rawValue, 0, Fixed64.Quarter.m_rawValue);
                }
                else
                {
                    Assert.Equal(Fixed64.Half, target.X);
                    Assert.InRange(target.Y.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                    Assert.InRange(target.Z.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                    Assert.Equal(Fixed64.Quarter, row.Depth);
                }
                Assert.InRange(native.X.m_rawValue, -Fixed64.Quarter.m_rawValue, Fixed64.FromFraction(3, 4).m_rawValue);
                Assert.InRange(native.Y.m_rawValue, -Fixed64.Quarter.m_rawValue, Fixed64.FromFraction(3, 4).m_rawValue);
                Assert.InRange(native.Z.m_rawValue, -Fixed64.Quarter.m_rawValue, Fixed64.FromFraction(3, 4).m_rawValue);
                Assert.True(row.HasMaterialOverride);
                Assert.Equal(reverse ? hull.Material : sheet.Material, row.MaterialA);
                Assert.Equal(reverse ? sheet.Material : hull.Material, row.MaterialB);
            }
        }
    }

    [Theory]
    [InlineData(false, MeshColliderMode.Concave)]
    [InlineData(true, MeshColliderMode.Concave)]
    [InlineData(false, MeshColliderMode.Convex)]
    [InlineData(true, MeshColliderMode.Convex)]
    public void ClosedHullAgainstFiniteSheet_ShouldKeepTheSolidDepthAndMatchedPatch(bool reverse, MeshColliderMode mode)
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var wall = new LSMeshCollider(new[] { new Vector3d(-4, -2, 0), new(4, -2, 0),
            new(4, 2, 0), new(-4, 2, 0) }, new[] { 0, 1, 2, 0, 2, 3 },
            mode, MeshInertiaPolicy.SurfaceApproximation);
        scenario.CreateBody(wall, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scenario.CreateBody(hull, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        CollisionPair pair = reverse ? scenario.CreatePair(hull, wall) : scenario.CreatePair(wall, hull);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(1, pair.Manifold.GroupCount);
        Assert.Equal(4, pair.Manifold.Count);
        foreach (ManifoldContact row in pair.Manifold)
        {
            Assert.Equal(reverse ? Vector3d.Backward : Vector3d.Forward, row.Normal);
            Assert.Equal(Fixed64.Quarter, row.Depth);
            ContactAnchor native = reverse ? row.AnchorA : row.AnchorB;
            ContactAnchor target = reverse ? row.AnchorB : row.AnchorA;
            Assert.True(native.TryGetOffsetFrom(Vector3d.Zero, out Vector3d first));
            Assert.True(target.TryGetOffsetFrom(Vector3d.Zero, out Vector3d second));
            Assert.Equal(Vector3d.Forward * Fixed64.Quarter, second - first);
            Assert.Equal(Fixed64.Half, first.X.Abs());
            Assert.Equal(Fixed64.Half, first.Y.Abs());
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedHullAgainstFloorAndWall_ShouldRetainBothPhysicalPatchGroups(bool reverse)
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var corner = new LSMeshCollider(new[]
        {
            new Vector3d(-4, 0, 0), new(4, 0, 0), new(4, 4, 0), new(-4, 4, 0),
            new Vector3d(-4, 0, 0), new(-4, 0, 4), new(4, 0, 4), new(4, 0, 0)
        }, new[] { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.CreateBody(corner, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        LSMeshCollider hull = MeshTestFixtures.CreateConvexCube();
        scenario.CreateBody(hull, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Quarter), FixedQuaternion.Identity);
        CollisionPair pair = reverse ? scenario.CreatePair(hull, corner) : scenario.CreatePair(corner, hull);

        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(2, pair.Manifold.GroupCount);
        Assert.Equal(8, pair.Manifold.Count);
        Vector3d floorNormal = reverse ? Vector3d.Down : Vector3d.Up;
        Vector3d wallNormal = reverse ? Vector3d.Backward : Vector3d.Forward;
        Assert.Equal(4, pair.Manifold.Count(row => row.Normal == floorNormal));
        Assert.Equal(4, pair.Manifold.Count(row => row.Normal == wallNormal));
        foreach (ManifoldContact row in pair.Manifold)
        {
            Assert.Equal(Fixed64.Quarter, row.Depth);
            ContactAnchor native = reverse ? row.AnchorA : row.AnchorB;
            ContactAnchor target = reverse ? row.AnchorB : row.AnchorA;
            Assert.True(native.TryGetOffsetFrom(Vector3d.Zero, out Vector3d first));
            Assert.True(target.TryGetOffsetFrom(Vector3d.Zero, out Vector3d second));
            Assert.Equal((reverse ? -row.Normal : row.Normal) * Fixed64.Quarter, second - first);
        }
    }

    [Fact]
    public void ConvexMeshMesh_ShouldForwardCanonicalContactInBothDispatchDirections()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d secondOrigin =
            Fixed64.FromFraction(3, 4) * Vector3d.Right;
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            secondOrigin,
            FixedQuaternion.Identity);
        CollisionPair forward = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair reversed = scenario.CreatePair(second.Collider, first.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        Fixed64 half = Fixed64.Half;
        ManifoldContact contact = forward.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(Vector3d.Zero);
        contact.AnchorA.Rotation.Should().Be(FixedQuaternion.Identity);
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(half, -half, -half));
        contact.AnchorB.Origin.Should().Be(secondOrigin);
        contact.AnchorB.Rotation.Should().Be(FixedQuaternion.Identity);
        contact.AnchorB.LocalPoint.Should().Be(new Vector3d(-half, -half, -half));
        contact.Normal.Should().Be(Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.DepthIsClamped.Should().BeFalse();

        ManifoldContact reversedContact = reversed.Manifold.PrimaryContact;
        reversedContact.ContactId.Should().Be(contact.ContactId);
        reversedContact.AnchorA.Should().Be(contact.AnchorB);
        reversedContact.AnchorB.Should().Be(contact.AnchorA);
        reversedContact.Normal.Should().Be(-contact.Normal);
        reversedContact.Depth.Should().Be(contact.Depth);
        reversedContact.DepthIsClamped.Should().Be(contact.DepthIsClamped);
    }

    [Fact]
    public void MeshMesh_WithUnsupportedCanonicalExtent_ShouldRejectBeforePairCreation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.FromRaw(4611686011448066045L);
        LSMeshCollider mesh = CreateSeparatedTriangleMesh(extent);
        Action initialize = () => InitializeStaticMesh(scenario.Context, mesh,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), Vector3d.One);
        initialize.Should().Throw<ArgumentException>().WithMessage("*geometry reach*");
        mesh.Id.Should().Be(-1);
    }

    [Fact]
    public void MeshMesh_SupportedLargeEdgeCrossSeparatorShouldNotProduceContact()
    {
        Fixed64 scale = (Fixed64)512;
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d(1, -2, 2) * scale,
            new Vector3d(0, 3, 1) * scale,
            new Vector3d(-2, 2, 1) * scale);
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d(2, 2, -2) * scale,
            new Vector3d(-2, 0, -3) * scale,
            new Vector3d(-2, -1, 2) * scale);
        InitializeStaticMesh(context, first, Vector3d.One);
        InitializeStaticMesh(context, second, Vector3d.One);
        var manifold = new ContactManifold();
        var workItem = new CollisionWorkItem(
            context,
            first,
            second,
            CollisionType.Mesh_Mesh,
            manifold);

        first.Bounds.Intersects(second.Bounds).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(workItem).Should().BeFalse();
        manifold.Count.Should().Be(0);
    }

    [Fact]
    public void MeshMesh_ShouldPreserveExactCanonicalLocalAnchors()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d(0, 0, 0),
            new Vector3d(3, 0, 0),
            new Vector3d(0, 3, 0));
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d(0, 0, -3),
            new Vector3d(0, 0, 3),
            new Vector3d(0, 3, 0));
        ContactManifold manifold = Collide(context, first, second);

        ManifoldContact contact = manifold.PrimaryContact;
        // Exact lattice geometry retains the existing centered rigid witness
        // contract: barycentric weights round before their point products.
        Fixed64 roundedThird = Fixed64.Half + Fixed64.MinIncrement;
        contact.AnchorA.Origin.Should().Be(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2), Fixed64.Zero));
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(-Fixed64.FromFraction(3, 2), -roundedThird, Fixed64.Zero));
        contact.AnchorB.Origin.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.Zero));
        contact.AnchorB.LocalPoint.Should().Be(new Vector3d(
            Fixed64.Zero,
            -roundedThird,
            Fixed64.MinIncrement * (Fixed64)3));
        contact.PointA.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero));
        contact.PointB.Should().Be(new Vector3d(
            Fixed64.Zero,
            Fixed64.One - Fixed64.MinIncrement,
            Fixed64.MinIncrement * (Fixed64)3));
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void MeshMesh_ShouldRetainCanonicalAnchorsWhenWorldPointsAreUnrepresentable()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        LSMeshCollider first = CreateTriangleMesh(
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 2), Fixed64.Zero));
        LSMeshCollider second = CreateTriangleMesh(
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(-3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 2), Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        scenario.InitializeStaticCollider(first, origin);
        scenario.InitializeStaticCollider(second, origin);

        CollisionPair pair = scenario.CreatePair(first, second);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(origin);
        contact.AnchorB.Origin.Should().Be(origin);
        contact.AnchorA.LocalPoint.Should().Be(new Vector3d(
            Fixed64.One - Fixed64.MinIncrement * (Fixed64)4,
            -Fixed64.Half - Fixed64.MinIncrement,
            Fixed64.Zero));
        contact.AnchorB.LocalPoint.Should().Be(contact.AnchorA.LocalPoint);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out _).Should().BeFalse();
    }

    [Fact]
    public void MeshMesh_ReversedDispatchShouldPreserveStableContactOrdering()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider first = CreateQuadMesh(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0),
            new Vector3d(2, 2, 0));
        LSMeshCollider second = CreateQuadMesh(
            new Vector3d(0, -2, -2),
            new Vector3d(0, -2, 2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2));
        scenario.InitializeStaticCollider(first, Vector3d.Zero);
        scenario.InitializeStaticCollider(
            second,
            Fixed64.FromFraction(1, 4) * Vector3d.Right);
        CollisionPair forward = scenario.CreatePair(first, second);
        CollisionPair reversed = scenario.CreatePair(second, first);
        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        ManifoldContact[] expectedContacts = forward.Manifold.ToArray();
        ManifoldContact[] expectedReversedContacts = reversed.Manifold.ToArray();
        ulong[] expectedIds = expectedContacts.Select(contact => contact.ContactId).ToArray();
        ulong[] expectedReversedIds = expectedReversedContacts.Select(contact => contact.ContactId).ToArray();

        expectedContacts.Should().HaveCountGreaterThan(1);
        expectedIds.Should().BeInAscendingOrder();
        expectedReversedIds.Should().BeInAscendingOrder();
        expectedReversedIds.Should().Equal(expectedIds);
        for (int index = 0; index < expectedContacts.Length; index++)
        {
            ManifoldContact contact = expectedContacts[index];
            ManifoldContact reversedContact = expectedReversedContacts[index];
            reversedContact.AnchorA.Should().Be(contact.AnchorB);
            reversedContact.AnchorB.Should().Be(contact.AnchorA);
            reversedContact.Normal.Should().Be(-contact.Normal);
            reversedContact.Depth.Should().Be(contact.Depth);
            reversedContact.DepthIsClamped.Should().Be(contact.DepthIsClamped);
        }

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        forward.Manifold.Select(contact => contact.ContactId).Should().Equal(expectedIds);
        reversed.Manifold.Select(contact => contact.ContactId).Should().Equal(expectedReversedIds);
        forward.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        reversed.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
    }

    [Fact]
    public void MeshMesh_WithUnsupportedDepthGeometry_ShouldRejectBeforePairCreation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 half = Fixed64.MaxValue / Fixed64.Two - Fixed64.One;
        LSMeshCollider mesh = CreateTriangleMesh(new Vector3d(-half, half, -half),
            new Vector3d(-half, -half, half), new Vector3d(half, half, half), MeshInertiaPolicy.RequireClosedVolume);
        Action initialize = () => InitializeStaticMesh(scenario.Context, mesh, Vector3d.One * Fixed64.Two);
        initialize.Should().Throw<ArgumentException>().WithMessage("*geometry reach*");
        mesh.Id.Should().Be(-1);
    }

    [Fact]
    public void MeshMesh_WarmedChecksShouldNotAllocate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(CreateQuadMesh(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0),
            new Vector3d(2, 2, 0)), Vector3d.Zero, FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(CreateQuadMesh(
            new Vector3d(0, -2, -2),
            new Vector3d(0, -2, 2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2)), Vector3d.Zero, FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair))
                throw new InvalidOperationException("Expected the prepared mesh pair to collide.");
        });

        allocatedBytes.Should().Be(0);
    }

    private static LSMeshCollider CreateTriangleMesh(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.SurfaceApproximation) =>
        new(
            new[] { first, second, third },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            inertiaPolicy);

    private static LSMeshCollider CreateSeparatedTriangleMesh(Fixed64 extent) =>
        new(
            new[]
            {
                new Vector3d(-extent, -extent, -extent),
                new Vector3d(-extent, -extent, extent),
                new Vector3d(extent, -extent, -extent),
                new Vector3d(-extent, extent, -extent),
                new Vector3d(extent, extent, -extent),
                new Vector3d(-extent, extent, extent)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.RequireClosedVolume);

    private static LSMeshCollider CreateQuadMesh(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d fourth) =>
        new(
            new[] { first, second, third, fourth },
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

    private static ContactManifold Collide(
        GravitasWorldContext context,
        LSMeshCollider first,
        LSMeshCollider second)
    {
        var manifold = new ContactManifold();
        CollisionDetection.DoCollisionCheck(CreateWorkItem(context, first, second, manifold)).Should().BeTrue();
        return manifold;
    }

    private static CollisionWorkItem CreateWorkItem(
        GravitasWorldContext context,
        LSMeshCollider first,
        LSMeshCollider second,
        ContactManifold manifold) =>
        new(context, first, second, CollisionType.Mesh_Mesh, manifold);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d scale) =>
        InitializeStaticMesh(context, collider, Vector3d.Zero, scale);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        Vector3d scale) =>
        InitializeStaticMesh(
            context,
            collider,
            position,
            FixedQuaternion.Identity,
            scale);

    private static void InitializeStaticMesh(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d scale)
    {
        var transform = new FixedTransform(position, rotation, scale);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
    }

}
