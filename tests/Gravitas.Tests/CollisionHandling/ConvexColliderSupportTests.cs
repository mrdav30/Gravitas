using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexColliderSupportTests
{
    [Fact]
    public void IsSupported_ShouldRejectUnsupportedColliderTypes()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        ConvexColliderSupport.IsSupported(sphere.Collider).Should().BeTrue();
        ConvexColliderSupport.IsSupported(unsupported).Should().BeFalse();
        ConvexColliderSupport.Intersects(sphere.Collider, unsupported).Should().BeFalse();
        ConvexColliderSupport.Intersects(unsupported, sphere.Collider).Should().BeFalse();
    }

    [Fact]
    public void ProjectOntoAxis_WithZeroAxis_ShouldUseRightAxisFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        FixedRange range = ConvexColliderSupport.ProjectOntoAxis(sphere.Collider, Vector3d.Zero);

        range.Min.Should().Be(Fixed64.FromFraction(3, 2));
        range.Max.Should().Be(Fixed64.FromFraction(5, 2));
    }

    [Fact]
    public void ProjectOntoAxis_WithNonZeroAxis_ShouldUseRequestedAxis()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));

        FixedRange range = ConvexColliderSupport.ProjectOntoAxis(sphere.Collider, Vector3d.Up);

        range.Min.Should().Be(Fixed64.FromFraction(3, 2));
        range.Max.Should().Be(Fixed64.FromFraction(5, 2));
    }

    [Fact]
    public void Support_WithZeroDirection_ShouldUseRightAxisFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);

        ConvexColliderSupport.Support(sphere.Collider, Vector3d.Zero)
            .Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void Support_WithCylinderAxisDirection_ShouldUseStableRadialTie()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);

        ConvexColliderSupport.Support(cylinder.Collider, Vector3d.Up)
            .Should().Be(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void Support_WithExtremeTranslatedCuboid_ShouldCompareCenteredFeatures()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(offset, offset, Fixed64.Zero));
        Vector3d direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;

        Vector3d support = ConvexColliderSupport.Support(cuboid.Collider, direction);

        support.Should().Be(new Vector3d(offset + Fixed64.Half, offset + Fixed64.Half, -Fixed64.Half));
    }

    [Fact]
    public void Support_WithExtremeTranslatedConvexMesh_ShouldCompareLocalFeatures()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d(offset, offset, Fixed64.Zero),
            FixedQuaternion.Identity);
        Vector3d direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;

        Vector3d support = ConvexColliderSupport.Support(mesh.Collider, direction);

        support.X.Should().Be(offset + Fixed64.Half);
        support.Y.Should().Be(offset + Fixed64.Half);
    }

    [Fact]
    public void Support_WithExtremeTranslatedRotatedCapsule_ShouldSelectForwardSegmentEndpoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(offset, offset, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-45)));
        Vector3d direction = capsule.Collider.LineDirection;

        Vector3d support = ConvexColliderSupport.Support(capsule.Collider, direction);

        support.Should().Be(capsule.Collider.LineSegmentEnd + direction * capsule.Collider.ScaledRadius);
    }

    [Fact]
    public void Support_ShouldUseConvexMeshVerticesAndRejectUnsupportedColliders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        Vector3d support = ConvexColliderSupport.Support(mesh.Collider, Vector3d.Right);
        Action unsupportedSupport = () => ConvexColliderSupport.Support(unsupported, Vector3d.Right);

        support.X.Should().Be(Fixed64.FromFraction(5, 2));
        unsupportedSupport.Should().Throw<NotSupportedException>();
    }

    [Fact]
    public void Intersects_WithSameCenterSpheres_ShouldReturnTrueImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WhenIterationBudgetIsExhaustedWithoutSeparation_ShouldRemainConservative()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider, maxIterations: 0).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithSeparatedConvexShapes_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        ConvexColliderSupport.Intersects(sphere.Collider, cuboid.Collider).Should().BeFalse();
    }

    [Fact]
    public void Intersects_WithOverlappingRotatedConvexShapes_ShouldReturnTrue()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(35));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            PhysicsScenarioBuilder.Yaw(-20));

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithTouchingAxisAlignedSpheres_ShouldReturnTrue()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithVerticallyTouchingSpheres_ShouldUseStablePerpendicularFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Up);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithOffsetCapsuleAgainstCuboid_ShouldReduceTriangleSimplex()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            PhysicsScenarioBuilder.Yaw(90));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);

        ConvexColliderSupport.Intersects(capsule.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithSphereInsideCuboid_ShouldReduceTetrahedronSimplex()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)3, (Fixed64)3, (Fixed64)3)
            },
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(15));

        ConvexColliderSupport.Intersects(sphere.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithConeRimTouchingCuboidFace_ShouldAcceptEpsilonDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.One, Fixed64.FromFraction(1, 8)));
        Vector3d touchingPoint = new(-Fixed64.One, -Fixed64.One, Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(cone.Collider, cuboid.Collider);

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        cone.Collider.WorldBaseCenter.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
    }

    [Fact]
    public void Intersects_WithConeRimTouchingCuboidCorner_ShouldAcceptIterationBudget()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        ScenarioBody<LSCuboidCollider> separated = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(13, 8), -Fixed64.Half, -Fixed64.Half));
        Vector3d touchingPoint = new(-Fixed64.One, -Fixed64.One, Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(cone.Collider, cuboid.Collider);

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ConvexColliderSupport.Intersects(cone.Collider, separated.Collider).Should().BeFalse();
        cone.Collider.WorldBaseCenter.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
        cuboid.Collider.BoundsMin.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.Z.Should().Be(touchingPoint.Z);
    }

    [Fact]
    public void IntersectsConeVolume_ShouldDetectConvexHitsAndMisses()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> hit = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> miss = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero));
        Vector3d apex = Vector3d.Zero;
        Vector3d baseCenter = new(Fixed64.Zero, (Fixed64)2, Fixed64.Zero);
        Vector3d axis = Vector3d.Up;
        Fixed64 radius = Fixed64.One;

        ConvexColliderSupport.IntersectsConeVolume(hit.Collider, apex, baseCenter, axis, radius).Should().BeTrue();
        ConvexColliderSupport.IntersectsConeVolume(miss.Collider, apex, baseCenter, axis, radius).Should().BeFalse();
    }

    [Fact]
    public void IntersectsConeVolume_WithSphereTouchingApex_ShouldReturnTrueImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));

        ConvexColliderSupport.IntersectsConeVolume(
            sphere.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One).Should().BeTrue();
        sphere.Collider.BoundsMax.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void IntersectsConeVolume_WithCuboidTouchingBaseRim_ShouldAcceptEpsilonDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), (Fixed64)2, Fixed64.FromFraction(1, 4)));
        Vector3d touchingPoint = new(-Fixed64.One, (Fixed64)2, Fixed64.Zero);

        ConvexColliderSupport.IntersectsConeVolume(
            cuboid.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One).Should().BeTrue();
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
        cuboid.Collider.BoundsMin.Y.Should().BeLessThanOrEqualTo(touchingPoint.Y);
        cuboid.Collider.BoundsMax.Y.Should().BeGreaterThanOrEqualTo(touchingPoint.Y);
    }

    [Fact]
    public void IntersectsConeVolume_ShouldRejectUnsupportedColliderTypes()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        ConvexColliderSupport.IntersectsConeVolume(
            unsupported,
            Vector3d.Zero,
            Vector3d.Up,
            Vector3d.Up,
            Fixed64.Half).Should().BeFalse();
    }

    [Fact]
    public void IntersectsConeVolume_WithExhaustedIterationBudget_ShouldPreserveCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        ConvexColliderSupport.IntersectsConeVolume(
            sphere.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One,
            maxIterations: 0).Should().BeTrue();
    }
}
