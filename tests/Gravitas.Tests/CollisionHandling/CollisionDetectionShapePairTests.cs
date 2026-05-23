using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionDetectionShapePairTests
{
    [Fact]
    public void SphereSphere_ShouldDetectOverlapTouchAndDegenerateCenter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> overlapping = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> touching = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> sameCenter = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, left.Collider, overlapping.Collider, CollisionType.Sphere_Sphere);
        AssertCollision(scenario, left.Collider, touching.Collider, CollisionType.Sphere_Sphere);
        AssertCollision(scenario, left.Collider, sameCenter.Collider, CollisionType.Sphere_Sphere);
        AssertNoCollision(scenario, left.Collider, separated.Collider, CollisionType.Sphere_Sphere);
    }

    [Fact]
    public void CapsuleSphere_ShouldDetectOverlapAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> overlapping = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(3, 0, 0));

        AssertCollision(scenario, capsule.Collider, overlapping.Collider, CollisionType.Capsule_Sphere);
        AssertNoCollision(scenario, capsule.Collider, separated.Collider, CollisionType.Capsule_Sphere);
    }

    [Fact]
    public void CapsuleCapsule_ShouldDetectOverlapAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> overlapping = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCapsuleCollider> separated = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(3, 0, 0));

        AssertCollision(scenario, capsule.Collider, overlapping.Collider, CollisionType.Capsule_Capsule);
        AssertNoCollision(scenario, capsule.Collider, separated.Collider, CollisionType.Capsule_Capsule);
    }

    [Fact]
    public void CuboidSphere_ShouldDetectAxisAlignedOverlapAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> overlapping = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(3, 0, 0));

        AssertCollision(scenario, cuboid.Collider, overlapping.Collider, CollisionType.Cuboid_Sphere);
        AssertNoCollision(scenario, cuboid.Collider, separated.Collider, CollisionType.Cuboid_Sphere);
    }

    [Fact]
    public void CuboidCapsule_ShouldDetectAxisAlignedAndRotatedOverlap()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> axisAligned = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> axisAlignedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> rotated = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            PhysicsScenarioBuilder.Yaw(45));
        ScenarioBody<LSCapsuleCollider> rotatedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(
            (Fixed64)4 + Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero));

        axisAligned.Collider.Shape.Should().Be(ColliderType.AABox);
        rotated.Collider.Shape.Should().Be(ColliderType.OBBox);
        AssertCollision(scenario, axisAligned.Collider, axisAlignedCapsule.Collider, CollisionType.AABox_Capsule);
        AssertCollision(scenario, rotated.Collider, rotatedCapsule.Collider, CollisionType.OBBox_Capsule);
    }

    [Fact]
    public void CuboidCuboid_ShouldDistinguishOverlapTouchAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> overlapping = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> edgeTouching = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSCuboidCollider> separated = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, first.Collider, overlapping.Collider, CollisionType.Cuboid_Cuboid);
        AssertNoCollision(scenario, first.Collider, edgeTouching.Collider, CollisionType.Cuboid_Cuboid);
        AssertNoCollision(scenario, first.Collider, separated.Collider, CollisionType.Cuboid_Cuboid);
    }

    [Fact]
    public void CuboidCuboid_ShouldDetectRotatedOverlap()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            PhysicsScenarioBuilder.Yaw(45));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        first.Collider.Shape.Should().Be(ColliderType.OBBox);
        second.Collider.Shape.Should().Be(ColliderType.AABox);
        AssertCollision(scenario, first.Collider, second.Collider, CollisionType.Cuboid_Cuboid);
    }

    private static CollisionPair AssertCollision(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB,
        CollisionType expectedType)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);

        pair.CollisionType.Should().Be(expectedType);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.ContactPoint.Depth.Should().BeGreaterThan(Fixed64.Zero);
        return pair;
    }

    private static void AssertNoCollision(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB,
        CollisionType expectedType)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);

        pair.CollisionType.Should().Be(expectedType);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.ContactPoint.Depth.Should().Be(Fixed64.Zero);
    }
}
