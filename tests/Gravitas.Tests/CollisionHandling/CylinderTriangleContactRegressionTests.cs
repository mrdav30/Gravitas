using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CylinderTriangleContactRegressionTests
{
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
    public void CylinderTriangle_UnrepresentableCenterOffset_ShouldRetainRealIntersection()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(-3, -1, 0), new Vector3d(3, -1, 0), new Vector3d(0, 1, 0)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(3, 0, 0), FixedQuaternion.Identity, Vector3d.One)));
        var cylinder = new LSCylinderCollider { Radius = Fixed64.MaxValue, Size = Vector3d.One };
        cylinder.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity, Vector3d.One)));
        var pair = new CollisionPair(mesh, cylinder);

        // World triangle (0,-1,0),(6,-1,0),(3,1,0) contains (3/2,0,0).
        // The cylinder extends to X=2-oneRaw, so this is a real intrusion
        // despite its center being unrepresentable in the triangle's frame.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.DepthIsClamped.Should().BeFalse();
        Fixed64 projected = Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal);
        projected.m_rawValue.Should().BeInRange(contact.Depth.m_rawValue - 8, contact.Depth.m_rawValue + 8);
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
    [InlineData(3L, 1L, 0L)]
    [InlineData(1L, -1L, 2L)]
    public void CylinderTriangle_EnrichedCap_ShouldRoundExactDepthOnlyOnce(
        long heightRawOffset, long triangleRawOffset, long expectedDepthRaw)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 y = (Fixed64)5 + Fixed64.FromRaw(triangleRawOffset);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-16), y, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, y, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, y, (Fixed64)16),
            new Vector3d((Fixed64)8, -y, (Fixed64)8),
            new Vector3d((Fixed64)12, -y, (Fixed64)8),
            new Vector3d((Fixed64)8, -y, (Fixed64)12)
        }, new[] { 0, 1, 2, 3, 4, 5 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new TestMatterAgent(context));
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
