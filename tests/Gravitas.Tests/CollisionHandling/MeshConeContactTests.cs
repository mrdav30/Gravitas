using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeContactTests
{
    [Theory]
    [InlineData(MeshColliderMode.Concave, false)]
    [InlineData(MeshColliderMode.Concave, true)]
    [InlineData(MeshColliderMode.Convex, false)]
    [InlineData(MeshColliderMode.Convex, true)]
    public void ConeBelowCoplanarQuad_ShouldUseSurfaceExitInsteadOfInternalDiagonal(
        MeshColliderMode mode, bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(mode);
        FixedQuaternion rotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        mesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone, Vector3d.Down * Fixed64.Quarter);
        CollisionPair pair = scenario.CreatePair(mesh, cone);

        // Base Y=-3/4 and apex Y=1/4. Any translation shorter than 1/4
        // retains an axial crossing inside the quad; downward 1/4 removes it.
        // Its triangulation diagonal cannot provide a shorter surface exit.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Normal.Should().Be(Vector3d.Down);
        contact.Depth.Should().Be(Fixed64.Quarter);
        contact.PointA.Should().Be(Vector3d.Zero);
        contact.PointB.Should().Be(Vector3d.Up * Fixed64.Quarter);
        Vector3d.Dot(Vector3d.Right, contact.Normal).Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnrepresentableRelativeCenter_WithInteriorIntersection_RetainsCanonicalContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateTriangle(new Vector3d(-3, -1, 0), new Vector3d(3, -1, 0), new Vector3d(0, 1, 0));
        scenario.InitializeStaticCollider(mesh, new Vector3d(3, 0, 0));
        var cone = new LSConeCollider { Radius = Fixed64.MaxValue, Size = Vector3d.One };
        scenario.InitializeStaticCollider(cone, new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero, Fixed64.Zero));

        mesh.Bounds.Intersects(cone.Bounds).Should().BeTrue();
        new FixedPointAnchor(cone.Center, FixedQuaternion.Identity, Vector3d.Zero)
            .TryGetLocalPointIn(mesh.Mesh.Origin, mesh.Mesh.Rotation, out _).Should().BeFalse();
        // At Y=-1/2+oneRaw the triangle includes X=1, strictly inside the cone.
        // Rejecting the unrepresentable center would discard that real overlap.
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.AnchorA.Origin.Should().Be(mesh.Mesh.Origin);
        pair.Manifold.PrimaryContact.AnchorB.Origin.Should().Be(cone.Center);
        pair.Manifold.PrimaryContact.Normal.IsNormalized().Should().BeTrue();
    }

    [Fact]
    public void SideCrossing_AwayFromNearestCenterPoint_IsNotRejectedByFailedSamples()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair pair = CreateSideCrossingPair(scenario);

        // (9/10,-19/20,0) lies inside the triangle's [4/5,137/100] slice
        // and inside the cone's radius-39/40 cross-section.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointA.Z.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CentralWall_SelectsDepthNormalAndAnchorsFromTheSameExitFeature()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        // The triangle spans the whole cone projection onto X=0. Either X
        // direction needs one unit of translation; moving toward an edge costs more.
        var mesh = CreateCentralWall();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, cone);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        var contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.One);
        FixedMath.Abs(contact.Normal.X).Should().Be(Fixed64.One);
        contact.Normal.Y.Should().Be(Fixed64.Zero);
        contact.Normal.Z.Should().Be(Fixed64.Zero);
        contact.PointA.Should().Be(new Vector3d(0, -1, 0));
        contact.PointB.Should().Be(new Vector3d(-contact.Normal.X, -Fixed64.One, Fixed64.Zero));
        (contact.PointA - contact.PointB).Should().Be(contact.Normal);
        contact.DepthIsClamped.Should().BeFalse();

        // Input order does not invert the canonical mesh-first dispatch.
        CollisionPair reversed = scenario.CreatePair(cone, mesh);
        reversed.ColliderA.Should().BeSameAs(mesh);
        reversed.ColliderB.Should().BeSameAs(cone);
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        AssertSameContact(contact, reversed.Manifold.PrimaryContact);
    }

    [Fact]
    public void CompoundConePart_MapsMeshFirstContactBackToCompoundOwnerOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateCentralWall();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cone(Fixed64.One, Fixed64.Two, Vector3d.Zero, PhysicsMaterial.Frictionless));
        scenario.InitializeStaticCollider(compound, Vector3d.Zero);
        LSCollider conePart = compound.GetPartCollider(0);
        CollisionPair direct = scenario.CreatePair(mesh, conePart);
        CollisionDetection.DoCollisionCheck(direct).Should().BeTrue();
        ManifoldContact meshFirst = direct.Manifold.PrimaryContact;

        CollisionPair pair = scenario.CreatePair(mesh, compound);
        pair.ColliderA.Should().BeSameAs(compound);
        pair.CollisionType.Should().Be(CollisionType.Compound);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.One);
        contact.DepthIsClamped.Should().Be(meshFirst.DepthIsClamped);
        contact.Normal.Should().Be(-meshFirst.Normal);
        contact.AnchorA.Should().Be(meshFirst.AnchorB);
        contact.AnchorB.Should().Be(meshFirst.AnchorA);
        (contact.PointA - contact.PointB).Should().Be(contact.Normal);
        contact.HasMaterialOverride.Should().BeTrue();
        contact.MaterialA.Should().Be(conePart.Material);
        contact.MaterialB.Should().Be(mesh.Material);
        contact.FeatureNamespaceA.Should().Be(1);
        contact.FeatureNamespaceB.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MultipleTriangles_SelectMinimumDepthRegardlessOfAuthoredOrder(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair pair = CreateHorizontalPair(scenario, Fixed64.FromFraction(7, 8), reverse);

        // The lower plane needs 1/4 upward translation; the upper plane
        // needs only 1/8 downward translation past the apex. Both triangles
        // contain the axial crossings for every smaller displacement.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 8));
        contact.DepthIsClamped.Should().BeFalse();
        contact.Normal.Should().Be(Vector3d.Down);
        contact.PointA.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(7, 8), Fixed64.Zero));
        contact.PointB.Should().Be(Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EqualDepthTriangles_RetainFirstBvhCandidateAndStableContact(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair pair = CreateHorizontalPair(scenario, Fixed64.FromFraction(3, 4), reverse);
        var mesh = (LSMeshCollider)pair.ColliderA;
        var candidates = new SwiftList<int>();
        mesh.GetTrianglesInBounds(new FixedBoundVolume(pair.ColliderB.BoundsMin, pair.ColliderB.BoundsMax), candidates);
        candidates.Count.Should().Be(2);
        mesh.Mesh.GetLocalTriangleVertices(candidates[0], out Vector3d first, out _, out _);
        mesh.Mesh.CreatePointAnchor(first).TryGetPoint(out Vector3d firstWorld).Should().BeTrue();
        Vector3d expectedNormal = firstWorld.Y < Fixed64.Zero ? Vector3d.Up : Vector3d.Down;

        // Both exits are exactly 1/4. The reducer promises first candidate,
        // not triangle-index ordering independent of the existing BVH.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal.Should().Be(expectedNormal);
        contact.PointA.Y.Should().Be(firstWorld.Y);
        contact.PointB.Y.Should().Be(-expectedNormal.Y);
        for (int iteration = 0; iteration < 3; iteration++)
        {
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            AssertSameContact(contact, pair.Manifold.PrimaryContact);
        }
    }

    [Fact]
    public void SideCrossing_RepeatedWarmedDispatchRetainsContactWithoutAllocating()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair pair = CreateSideCrossingPair(scenario);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact expected = pair.Manifold.PrimaryContact;

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair))
                throw new InvalidOperationException("The prepared cone/triangle intersection disappeared.");
            ManifoldContact actual = pair.Manifold.PrimaryContact;
            if (actual.ContactId != expected.ContactId || actual.Depth != expected.Depth
                || actual.Normal != expected.Normal || actual.DepthIsClamped != expected.DepthIsClamped)
                throw new InvalidOperationException("Repeated cone/triangle dispatch changed the selected contact.");
        }, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 4);

        allocated.Should().Be(0);
        AssertSameContact(expected, pair.Manifold.PrimaryContact);
    }

    private static void AssertSameContact(ManifoldContact expected, ManifoldContact actual)
    {
        actual.ContactId.Should().Be(expected.ContactId);
        actual.Depth.Should().Be(expected.Depth);
        actual.DepthIsClamped.Should().Be(expected.DepthIsClamped);
        actual.Normal.Should().Be(expected.Normal);
        actual.AnchorA.Should().Be(expected.AnchorA);
        actual.AnchorB.Should().Be(expected.AnchorB);
    }

    private static CollisionPair CreateSideCrossingPair(PhysicsScenarioBuilder scenario)
    {
        var mesh = CreateTriangle(
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(4, 5), (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(2, -2, 0));
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        return scenario.CreatePair(mesh, cone);
    }

    private static CollisionPair CreateHorizontalPair(PhysicsScenarioBuilder scenario, Fixed64 upperY, bool reverse)
    {
        Fixed64 lowerY = Fixed64.FromFraction(-3, 4);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-16), lowerY, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, lowerY, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, lowerY, (Fixed64)16),
            new Vector3d((Fixed64)(-16), upperY, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, upperY, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, upperY, (Fixed64)16)
        }, reverse ? new[] { 3, 4, 5, 0, 1, 2 } : new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) };
        scenario.InitializeStaticCollider(cone, Vector3d.Zero);
        return scenario.CreatePair(mesh, cone);
    }

    private static LSMeshCollider CreateCentralWall() =>
        CreateTriangle(new Vector3d(0, -16, -16), new Vector3d(0, 16, -16), new Vector3d(0, 0, 16));

    private static LSMeshCollider CreateTriangle(Vector3d first, Vector3d second, Vector3d third) =>
        new(new[] { first, second, third }, new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
}
