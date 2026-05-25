using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionDetectionShapePairTests
{
    [Fact]
    public void CollisionTypeMatrix_ShouldDeclareSupportedAndDeferredPairs()
    {
        var expected = new Dictionary<(ColliderType, ColliderType), CollisionType>
        {
            [(ColliderType.Sphere, ColliderType.Sphere)] = CollisionType.Sphere_Sphere,
            [(ColliderType.Sphere, ColliderType.Capsule)] = CollisionType.Capsule_Sphere,
            [(ColliderType.Sphere, ColliderType.AABox)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.Sphere, ColliderType.OBBox)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.Sphere, ColliderType.Cylinder)] = CollisionType.Cylinder_Sphere,
            [(ColliderType.Sphere, ColliderType.Mesh)] = CollisionType.Mesh_Sphere,
            [(ColliderType.Capsule, ColliderType.Sphere)] = CollisionType.Capsule_Sphere,
            [(ColliderType.Capsule, ColliderType.Capsule)] = CollisionType.Capsule_Capsule,
            [(ColliderType.Capsule, ColliderType.AABox)] = CollisionType.AABox_Capsule,
            [(ColliderType.Capsule, ColliderType.OBBox)] = CollisionType.OBBox_Capsule,
            [(ColliderType.Capsule, ColliderType.Cylinder)] = CollisionType.Cylinder_Capsule,
            [(ColliderType.Capsule, ColliderType.Mesh)] = CollisionType.Mesh_Capsule,
            [(ColliderType.AABox, ColliderType.Sphere)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.AABox, ColliderType.Capsule)] = CollisionType.AABox_Capsule,
            [(ColliderType.AABox, ColliderType.AABox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.AABox, ColliderType.OBBox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.AABox, ColliderType.Cylinder)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.AABox, ColliderType.Mesh)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.OBBox, ColliderType.Sphere)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.OBBox, ColliderType.Capsule)] = CollisionType.OBBox_Capsule,
            [(ColliderType.OBBox, ColliderType.AABox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.OBBox, ColliderType.OBBox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.OBBox, ColliderType.Cylinder)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.OBBox, ColliderType.Mesh)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.Cylinder, ColliderType.Sphere)] = CollisionType.Cylinder_Sphere,
            [(ColliderType.Cylinder, ColliderType.Capsule)] = CollisionType.Cylinder_Capsule,
            [(ColliderType.Cylinder, ColliderType.AABox)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.Cylinder, ColliderType.OBBox)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.Cylinder, ColliderType.Cylinder)] = CollisionType.Cylinder_Cylinder,
            [(ColliderType.Cylinder, ColliderType.Mesh)] = CollisionType.Mesh_Cylinder,
            [(ColliderType.Mesh, ColliderType.Sphere)] = CollisionType.Mesh_Sphere,
            [(ColliderType.Mesh, ColliderType.Capsule)] = CollisionType.Mesh_Capsule,
            [(ColliderType.Mesh, ColliderType.AABox)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.Mesh, ColliderType.OBBox)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.Mesh, ColliderType.Cylinder)] = CollisionType.Mesh_Cylinder,
            [(ColliderType.Mesh, ColliderType.Mesh)] = CollisionType.Mesh_Mesh
        };
        ColliderType[] activeTypes =
        {
            ColliderType.Sphere,
            ColliderType.Capsule,
            ColliderType.AABox,
            ColliderType.OBBox,
            ColliderType.Cylinder,
            ColliderType.Mesh
        };

        foreach (ColliderType first in activeTypes)
        {
            foreach (ColliderType second in activeTypes)
            {
                CollisionType expectedType = expected.TryGetValue((first, second), out CollisionType mapped)
                    ? mapped
                    : CollisionType.None;

                ColliderSettings.GetCollisionType(first, second).Should().Be(expectedType);
            }
        }
    }

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

    [Fact]
    public void CylinderSphere_ShouldDetectSideCapRotationAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sideOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> capOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.Fraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> rotatedCylinder = scenario.CreateCylinder(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        ScenarioBody<LSSphereCollider> rotatedCapOverlap = scenario.CreateSphere(new Vector3d((Fixed64)4 + Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, sideOverlap.Collider, cylinder.Collider, CollisionType.Cylinder_Sphere);
        AssertCollision(scenario, cylinder.Collider, capOverlap.Collider, CollisionType.Cylinder_Sphere);
        AssertCollision(scenario, rotatedCylinder.Collider, rotatedCapOverlap.Collider, CollisionType.Cylinder_Sphere);
        AssertNoCollision(scenario, cylinder.Collider, separated.Collider, CollisionType.Cylinder_Sphere);
    }

    [Fact]
    public void CylinderCapsule_ShouldDetectOverlapSeparationAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> overlapping = scenario.CreateCapsule(new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCapsuleCollider> separated = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(3, 0, 0));

        AssertCollision(scenario, cylinder.Collider, overlapping.Collider, CollisionType.Cylinder_Capsule);
        AssertCollision(scenario, overlapping.Collider, cylinder.Collider, CollisionType.Cylinder_Capsule);
        AssertNoCollision(scenario, cylinder.Collider, separated.Collider, CollisionType.Cylinder_Capsule);
    }

    [Fact]
    public void CylinderCylinder_ShouldRespectFlatCapsAndSideOverlap()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCylinderCollider> sideOverlap = scenario.CreateCylinder(new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> capSeparated = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.Fraction(5, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> separated = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, cylinder.Collider, sideOverlap.Collider, CollisionType.Cylinder_Cylinder);
        AssertNoCollision(scenario, cylinder.Collider, capSeparated.Collider, CollisionType.Cylinder_Cylinder);
        AssertNoCollision(scenario, cylinder.Collider, separated.Collider, CollisionType.Cylinder_Cylinder);
    }

    [Fact]
    public void CuboidCylinder_ShouldDetectAxisAlignedRotatedAndSeparated()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCylinderCollider> overlapping = scenario.CreateCylinder(new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> rotatedCuboid = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            PhysicsScenarioBuilder.Yaw(45));
        ScenarioBody<LSCylinderCollider> rotatedOverlap = scenario.CreateCylinder(new Vector3d((Fixed64)4 + Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> separated = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, cuboid.Collider, overlapping.Collider, CollisionType.Cuboid_Cylinder);
        AssertCollision(scenario, rotatedCuboid.Collider, rotatedOverlap.Collider, CollisionType.Cuboid_Cylinder);
        AssertNoCollision(scenario, cuboid.Collider, separated.Collider, CollisionType.Cuboid_Cylinder);
    }

    [Fact]
    public void MeshCylinder_ShouldDetectCapSideSeparationAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> capOverlap = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.Fraction(1, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> capSeparated = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        ScenarioBody<LSMeshCollider> wall = scenario.CreateBody(
            CreateVerticalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(6, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> sideOverlap = scenario.CreateCylinder(new Vector3d((Fixed64)6 + Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero));

        CollisionPair capPair = AssertCollision(scenario, floor.Collider, capOverlap.Collider, CollisionType.Mesh_Cylinder);
        CollisionPair sidePair = AssertCollision(scenario, sideOverlap.Collider, wall.Collider, CollisionType.Mesh_Cylinder);
        AssertNoCollision(scenario, floor.Collider, capSeparated.Collider, CollisionType.Mesh_Cylinder);

        capPair.ContactPoint.Normal.y.Should().BeGreaterThan(Fixed64.Zero);
        sidePair.ContactPoint.Normal.x.Should().BeGreaterThan(Fixed64.Zero);
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
        pair.ContactPoint.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        Vector3d centerDelta = pair.ColliderB.Center - pair.ColliderA.Center;
        if (centerDelta.SqrMagnitude > Fixed64.Epsilon)
            Vector3d.Dot(pair.ContactPoint.Normal, centerDelta).Should().BeGreaterThan(Fixed64.Zero);
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

    private static LSMeshCollider CreateHorizontalPlaneMesh() =>
        new(
            new[]
            {
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 });

    private static LSMeshCollider CreateVerticalPlaneMesh() =>
        new(
            new[]
            {
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)2),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 });
}
