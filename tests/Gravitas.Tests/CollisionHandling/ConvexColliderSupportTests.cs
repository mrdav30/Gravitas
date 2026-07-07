using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
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
    public void Intersects_WithSameCenterSpheres_ShouldReturnTrueImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
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
    public void IntersectsConeVolume_ShouldDetectConvexHitsAndMisses()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> hit = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> miss = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero));
        Vector3d apex = Vector3d.Zero;
        Vector3d axis = Vector3d.Up;
        Fixed64 length = (Fixed64)2;
        Fixed64 radius = Fixed64.One;

        ConvexColliderSupport.IntersectsConeVolume(hit.Collider, apex, axis, length, radius).Should().BeTrue();
        ConvexColliderSupport.IntersectsConeVolume(miss.Collider, apex, axis, length, radius).Should().BeFalse();
    }
}
