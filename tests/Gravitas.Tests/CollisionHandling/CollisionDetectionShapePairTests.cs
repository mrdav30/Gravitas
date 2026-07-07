using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using System.Linq;
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
            [(ColliderType.Sphere, ColliderType.Cone)] = CollisionType.Cone_Sphere,
            [(ColliderType.Sphere, ColliderType.Mesh)] = CollisionType.Mesh_Sphere,
            [(ColliderType.Sphere, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.Capsule, ColliderType.Sphere)] = CollisionType.Capsule_Sphere,
            [(ColliderType.Capsule, ColliderType.Capsule)] = CollisionType.Capsule_Capsule,
            [(ColliderType.Capsule, ColliderType.AABox)] = CollisionType.AABox_Capsule,
            [(ColliderType.Capsule, ColliderType.OBBox)] = CollisionType.OBBox_Capsule,
            [(ColliderType.Capsule, ColliderType.Cylinder)] = CollisionType.Cylinder_Capsule,
            [(ColliderType.Capsule, ColliderType.Cone)] = CollisionType.Cone_Convex,
            [(ColliderType.Capsule, ColliderType.Mesh)] = CollisionType.Mesh_Capsule,
            [(ColliderType.Capsule, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.AABox, ColliderType.Sphere)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.AABox, ColliderType.Capsule)] = CollisionType.AABox_Capsule,
            [(ColliderType.AABox, ColliderType.AABox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.AABox, ColliderType.OBBox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.AABox, ColliderType.Cylinder)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.AABox, ColliderType.Cone)] = CollisionType.Cone_Convex,
            [(ColliderType.AABox, ColliderType.Mesh)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.AABox, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.OBBox, ColliderType.Sphere)] = CollisionType.Cuboid_Sphere,
            [(ColliderType.OBBox, ColliderType.Capsule)] = CollisionType.OBBox_Capsule,
            [(ColliderType.OBBox, ColliderType.AABox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.OBBox, ColliderType.OBBox)] = CollisionType.Cuboid_Cuboid,
            [(ColliderType.OBBox, ColliderType.Cylinder)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.OBBox, ColliderType.Cone)] = CollisionType.Cone_Convex,
            [(ColliderType.OBBox, ColliderType.Mesh)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.OBBox, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.Cylinder, ColliderType.Sphere)] = CollisionType.Cylinder_Sphere,
            [(ColliderType.Cylinder, ColliderType.Capsule)] = CollisionType.Cylinder_Capsule,
            [(ColliderType.Cylinder, ColliderType.AABox)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.Cylinder, ColliderType.OBBox)] = CollisionType.Cuboid_Cylinder,
            [(ColliderType.Cylinder, ColliderType.Cylinder)] = CollisionType.Cylinder_Cylinder,
            [(ColliderType.Cylinder, ColliderType.Cone)] = CollisionType.Cone_Convex,
            [(ColliderType.Cylinder, ColliderType.Mesh)] = CollisionType.Mesh_Cylinder,
            [(ColliderType.Cylinder, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.Cone, ColliderType.Sphere)] = CollisionType.Cone_Sphere,
            [(ColliderType.Cone, ColliderType.Capsule)] = CollisionType.Cone_Convex,
            [(ColliderType.Cone, ColliderType.AABox)] = CollisionType.Cone_Convex,
            [(ColliderType.Cone, ColliderType.OBBox)] = CollisionType.Cone_Convex,
            [(ColliderType.Cone, ColliderType.Cylinder)] = CollisionType.Cone_Convex,
            [(ColliderType.Cone, ColliderType.Cone)] = CollisionType.Cone_Convex,
            [(ColliderType.Cone, ColliderType.Mesh)] = CollisionType.Mesh_Cone,
            [(ColliderType.Cone, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.Mesh, ColliderType.Sphere)] = CollisionType.Mesh_Sphere,
            [(ColliderType.Mesh, ColliderType.Capsule)] = CollisionType.Mesh_Capsule,
            [(ColliderType.Mesh, ColliderType.AABox)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.Mesh, ColliderType.OBBox)] = CollisionType.Mesh_Cuboid,
            [(ColliderType.Mesh, ColliderType.Cylinder)] = CollisionType.Mesh_Cylinder,
            [(ColliderType.Mesh, ColliderType.Cone)] = CollisionType.Mesh_Cone,
            [(ColliderType.Mesh, ColliderType.Mesh)] = CollisionType.Mesh_Mesh,
            [(ColliderType.Mesh, ColliderType.Compound)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Sphere)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Capsule)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.AABox)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.OBBox)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Cylinder)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Cone)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Mesh)] = CollisionType.Compound,
            [(ColliderType.Compound, ColliderType.Compound)] = CollisionType.Compound
        };
        ColliderType[] activeTypes =
        {
            ColliderType.Sphere,
            ColliderType.Capsule,
            ColliderType.AABox,
            ColliderType.OBBox,
            ColliderType.Cylinder,
            ColliderType.Cone,
            ColliderType.Mesh,
            ColliderType.Compound
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
    public void CuboidSphere_ShouldDetectRotatedOverlap()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45));
        ScenarioBody<LSSphereCollider> overlapping = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));

        cuboid.Collider.Shape.Should().Be(ColliderType.OBBox);
        Vector3d.Distance(
            cuboid.Collider.ClosestPointOnSurface(overlapping.Collider.Center),
            overlapping.Collider.Center).Should().BeLessThanOrEqualTo(overlapping.Collider.ScaledRadius);
        AssertCollision(scenario, cuboid.Collider, overlapping.Collider, CollisionType.Cuboid_Sphere);
    }

    [Fact]
    public void CuboidCapsule_ShouldDetectAxisAlignedRotatedAndSeparatedCases()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> axisAligned = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> axisAlignedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCapsuleCollider> separatedAxisAlignedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(3, 0, 0));
        ScenarioBody<LSCuboidCollider> rotated = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            PhysicsScenarioBuilder.Yaw(45));
        ScenarioBody<LSCapsuleCollider> rotatedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(
            (Fixed64)4 + Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero));
        ScenarioBody<LSCapsuleCollider> separatedRotatedCapsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(7, 0, 0));

        axisAligned.Collider.Shape.Should().Be(ColliderType.AABox);
        rotated.Collider.Shape.Should().Be(ColliderType.OBBox);
        AssertCollision(scenario, axisAligned.Collider, axisAlignedCapsule.Collider, CollisionType.AABox_Capsule);
        AssertCollision(scenario, rotated.Collider, rotatedCapsule.Collider, CollisionType.OBBox_Capsule);
        AssertNoCollision(scenario, axisAligned.Collider, separatedAxisAlignedCapsule.Collider, CollisionType.AABox_Capsule);
        AssertNoCollision(scenario, rotated.Collider, separatedRotatedCapsule.Collider, CollisionType.OBBox_Capsule);
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
        AssertCollision(scenario, first.Collider, edgeTouching.Collider, CollisionType.Cuboid_Cuboid)
            .Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
        AssertNoCollision(scenario, first.Collider, separated.Collider, CollisionType.Cuboid_Cuboid);
    }

    [Fact]
    public void CuboidCuboidFaceOverlap_ShouldGenerateFourStableOrderedContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));

        CollisionPair forward = AssertCollision(scenario, first.Collider, second.Collider, CollisionType.Cuboid_Cuboid);
        CollisionPair reversed = AssertCollision(scenario, second.Collider, first.Collider, CollisionType.Cuboid_Cuboid);

        forward.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        reversed.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        forward.Manifold.Select(contact => contact.ContactId).Should().Equal(reversed.Manifold.Select(contact => contact.ContactId));
        forward.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();

        for (int i = 0; i < forward.Manifold.Count; i++)
        {
            ManifoldContact contact = forward.Manifold[i];
            ManifoldContact reversedContact = reversed.Manifold[i];

            contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
            contact.Normal.Should().Be(Vector3d.Right);
            contact.PointA.X.Should().Be(Fixed64.Half);
            contact.PointB.X.Should().Be(Fixed64.FromFraction(1, 4));

            reversedContact.PointA.Should().Be(contact.PointB);
            reversedContact.PointB.Should().Be(contact.PointA);
            reversedContact.Normal.Should().Be(-contact.Normal);
        }
    }

    [Fact]
    public void CuboidCuboidEdgeTouch_ShouldGenerateTwoZeroDepthContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(1, 1, 0));

        CollisionPair pair = AssertCollision(scenario, first.Collider, second.Collider, CollisionType.Cuboid_Cuboid);

        pair.Manifold.Count.Should().Be(2);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        for (int i = 0; i < pair.Manifold.Count; i++)
            pair.Manifold[i].Depth.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CuboidCuboidStackedFaceTouch_ShouldGenerateFourZeroDepthContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> bottom = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> top = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 1, 0));

        CollisionPair pair = AssertCollision(scenario, bottom.Collider, top.Collider, CollisionType.Cuboid_Cuboid);

        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
    }

    [Theory]
    [InlineData(3, 4, 1)]
    [InlineData(-3, 4, -1)]
    public void CuboidCuboidDepthOnZ_ShouldGenerateStableFaceContacts(int zNumerator, int zDenominator, int expectedNormalZ)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        Fixed64 offsetZ = Fixed64.FromFraction(zNumerator, zDenominator);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.Zero, offsetZ));

        CollisionPair pair = AssertCollision(scenario, first.Collider, second.Collider, CollisionType.Cuboid_Cuboid);

        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Depth.Should().Be(Fixed64.One - offsetZ.Abs());
            contact.Normal.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)expectedNormalZ));
            contact.PointA.Z.Should().Be(expectedNormalZ > 0 ? first.Collider.BoundsMax.Z : first.Collider.BoundsMin.Z);
            contact.PointB.Z.Should().Be(contact.PointA.Z - contact.Normal.Z * contact.Depth);
        }
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
    public void CuboidCuboid_WithRotatedCuboidSeparatedByEdgeAxis_ShouldNotUseReducedSatFalsePositive()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(
            new Vector3d(
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-3, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)10, (Fixed64)35, (Fixed64)15));

        AssertNoCollision(scenario, first.Collider, second.Collider, CollisionType.Cuboid_Cuboid);
        AssertNoCollision(scenario, second.Collider, first.Collider, CollisionType.Cuboid_Cuboid);
    }

    [Fact]
    public void AxisAlignedCuboidChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void RotatedCuboidSatChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            PhysicsScenarioBuilder.Yaw(35));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void CylinderSphere_ShouldDetectSideCapRotationAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sideOverlap = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> capOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> rotatedCylinder = scenario.CreateCylinder(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        ScenarioBody<LSSphereCollider> rotatedCapOverlap = scenario.CreateSphere(new Vector3d((Fixed64)4 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, sideOverlap.Collider, cylinder.Collider, CollisionType.Cylinder_Sphere);
        AssertCollision(scenario, cylinder.Collider, capOverlap.Collider, CollisionType.Cylinder_Sphere);
        AssertCollision(scenario, rotatedCylinder.Collider, rotatedCapOverlap.Collider, CollisionType.Cylinder_Sphere);
        AssertNoCollision(scenario, cylinder.Collider, separated.Collider, CollisionType.Cylinder_Sphere);
    }

    [Fact]
    public void CylinderSphere_RimOverlap_ShouldUseFiniteRimNormalAndDepth()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 rimAxisOffset = Fixed64.Half * FixedMath.Sqrt(Fixed64.Half);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(new Vector3d(
            Fixed64.Half + rimAxisOffset - Fixed64.FromFraction(1, 16),
            Fixed64.Half + rimAxisOffset - Fixed64.FromFraction(1, 16),
            Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, cylinder.Collider, sphere.Collider, CollisionType.Cylinder_Sphere);

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CylinderCapsule_ShouldDetectOverlapSeparationAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> overlapping = scenario.CreateCapsule(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
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
        ScenarioBody<LSCylinderCollider> sideOverlap = scenario.CreateCylinder(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> capSeparated = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(5, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> separated = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(2, 0, 0));

        AssertCollision(scenario, cylinder.Collider, sideOverlap.Collider, CollisionType.Cylinder_Cylinder);
        AssertNoCollision(scenario, cylinder.Collider, capSeparated.Collider, CollisionType.Cylinder_Cylinder);
        AssertNoCollision(scenario, cylinder.Collider, separated.Collider, CollisionType.Cylinder_Cylinder);
    }

    [Fact]
    public void CylinderCylinder_ParallelCapOverlap_ShouldGenerateFourStableContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> bottom = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCylinderCollider> top = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, bottom.Collider, top.Collider, CollisionType.Cylinder_Cylinder);

        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            pair.Manifold[i].Depth.Should().Be(Fixed64.FromFraction(1, 4));
            pair.Manifold[i].Normal.Should().Be(Vector3d.Up);
        }
    }

    [Fact]
    public void CuboidCylinder_ShouldDetectAxisAlignedRotatedAndSeparated()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCylinderCollider> overlapping = scenario.CreateCylinder(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
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
    public void CuboidCylinder_CapFaceOverlap_ShouldGenerateFourStableContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, floor.Collider, cylinder.Collider, CollisionType.Cuboid_Cylinder);

        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            pair.Manifold[i].Depth.Should().Be(Fixed64.FromFraction(1, 4));
            pair.Manifold[i].Normal.Should().Be(Vector3d.Up);
            pair.Manifold[i].PointA.Y.Should().Be(Fixed64.Half);
            pair.Manifold[i].PointB.Y.Should().Be(Fixed64.FromFraction(1, 4));
        }
    }

    [Fact]
    public void MeshCylinder_ShouldDetectCapSideSeparationAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> capOverlap = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> capSeparated = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        ScenarioBody<LSMeshCollider> wall = scenario.CreateBody(
            CreateVerticalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(6, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> sideOverlap = scenario.CreateCylinder(new Vector3d((Fixed64)6 + Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));

        CollisionPair capPair = AssertCollision(scenario, floor.Collider, capOverlap.Collider, CollisionType.Mesh_Cylinder);
        CollisionPair sidePair = AssertCollision(scenario, sideOverlap.Collider, wall.Collider, CollisionType.Mesh_Cylinder);
        AssertNoCollision(scenario, floor.Collider, capSeparated.Collider, CollisionType.Mesh_Cylinder);

        capPair.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
        sidePair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MeshCylinder_CapFaceOverlap_ShouldGenerateFourStableContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, floor.Collider, cylinder.Collider, CollisionType.Mesh_Cylinder);

        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            pair.Manifold[i].Depth.Should().Be(Fixed64.FromFraction(1, 4));
            pair.Manifold[i].Normal.Should().Be(Vector3d.Up);
            pair.Manifold[i].PointA.Y.Should().Be(Fixed64.Zero);
            pair.Manifold[i].PointB.Y.Should().Be(-Fixed64.FromFraction(1, 4));
        }
    }

    [Fact]
    public void MeshCylinderChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(floor.Collider, cylinder.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ConeSphere_ShouldDetectSideBaseApexAndSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, Vector3d.Zero);
        ScenarioBody<LSSphereCollider> sideOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> baseOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> apexOverlap = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> separated = scenario.CreateSphere(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        AssertCollision(scenario, cone.Collider, sideOverlap.Collider, CollisionType.Cone_Sphere)
            .Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        AssertCollision(scenario, cone.Collider, baseOverlap.Collider, CollisionType.Cone_Sphere)
            .Manifold.PrimaryContact.Normal.Y.Should().BeLessThan(Fixed64.Zero);
        AssertCollision(scenario, cone.Collider, apexOverlap.Collider, CollisionType.Cone_Sphere)
            .Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        AssertNoCollision(scenario, cone.Collider, separated.Collider, CollisionType.Cone_Sphere);
    }

    [Fact]
    public void ConeConvex_ShouldDetectPrimitiveMeshAndCompoundPairs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(5, 4), Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> separatedCuboid = scenario.CreateCuboid(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSConeCollider> otherCone = CreateCone(scenario, new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity);
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Cone(Fixed64.Half, (Fixed64)2, Vector3d.Zero)),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> compoundCone = CreateCone(scenario, new Vector3d((Fixed64)3, Fixed64.Half, Fixed64.Zero));

        AssertCollision(scenario, cone.Collider, cuboid.Collider, CollisionType.Cone_Convex);
        AssertCollision(scenario, cone.Collider, capsule.Collider, CollisionType.Cone_Convex);
        AssertCollision(scenario, cone.Collider, cylinder.Collider, CollisionType.Cone_Convex);
        AssertCollision(scenario, cone.Collider, otherCone.Collider, CollisionType.Cone_Convex);
        AssertCollision(scenario, cuboid.Collider, cone.Collider, CollisionType.Cone_Convex);
        AssertNoCollision(scenario, cone.Collider, separatedCuboid.Collider, CollisionType.Cone_Convex);
        AssertCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);
        AssertCollision(scenario, compound.Collider, compoundCone.Collider, CollisionType.Compound);
    }

    [Fact]
    public void MeshCone_WithConeSideCrossingTrianglePlane_ShouldUseConeSupportContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> wall = scenario.CreateBody(
            CreateVerticalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, wall.Collider, cone.Collider, CollisionType.Mesh_Cone);

        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointB.X.Should().BeLessThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointA.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MeshCone_WithConeContainedInClosedConvexMesh_ShouldUseConvexFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.FromFraction(1, 8),
                Size = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4))
            },
            new Vector3d(Fixed64.FromFraction(1, 8), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);

        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MeshSphere_WithSphereCenterOnConvexSurface_ShouldUseFaceNormalAndPositiveDepth()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);

        CollisionPair pair = AssertCollision(scenario, floor.Collider, sphere.Collider, CollisionType.Mesh_Sphere);

        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void MeshCapsule_WithConvexMesh_ShouldDetectFallbackContactAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            CreateTallCapsule(),
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> separated = scenario.CreateBody(
            CreateTallCapsule(),
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair forward = AssertCollision(scenario, floor.Collider, capsule.Collider, CollisionType.Mesh_Capsule);
        CollisionPair reversed = AssertCollision(scenario, capsule.Collider, floor.Collider, CollisionType.Mesh_Capsule);
        AssertNoCollision(scenario, floor.Collider, separated.Collider, CollisionType.Mesh_Capsule);

        forward.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        forward.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
        reversed.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        reversed.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void PrimitiveManifoldChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair[] pairs =
        {
            scenario.CreatePair(
                scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0)).Collider,
                scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(2, 0, 0)).Collider,
                scenario.CreateSphere(new Vector3d((Fixed64)2 + Fixed64.Half, Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(4, 0, 0)).Collider,
                scenario.CreateCapsule(new Vector3d((Fixed64)4 + Fixed64.Half, Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(6, 0, 0)).Collider,
                scenario.CreateSphere(new Vector3d((Fixed64)6 + Fixed64.Half, Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(8, 0, 0)).Collider,
                scenario.CreateSphere(new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(10, 0, 0)).Collider,
                scenario.CreateCapsule(new Vector3d((Fixed64)10 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider),
            scenario.CreatePair(
                scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(12, 0, 0)).Collider,
                scenario.CreateCylinder(new Vector3d((Fixed64)12 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider)
        };

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            for (int i = 0; i < pairs.Length; i++)
                EnsureCollision(pairs[i]);
        });

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MeshCuboid_ShouldPreserveTriangleContactAndReversedDispatch()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));

        CollisionPair forward = AssertCollision(scenario, floor.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
        CollisionPair reversed = AssertCollision(scenario, cuboid.Collider, floor.Collider, CollisionType.Mesh_Cuboid);

        forward.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
        reversed.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
        forward.Manifold.HasContact.Should().BeTrue();
        reversed.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void MeshCuboid_FaceOverlap_ShouldGenerateFourClippedContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));

        CollisionPair forward = AssertCollision(scenario, floor.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
        CollisionPair reversed = AssertCollision(scenario, cuboid.Collider, floor.Collider, CollisionType.Mesh_Cuboid);

        forward.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        reversed.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        forward.Manifold.Select(contact => contact.ContactId).Should().Equal(reversed.Manifold.Select(contact => contact.ContactId));
        forward.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();

        for (int i = 0; i < forward.Manifold.Count; i++)
        {
            ManifoldContact contact = forward.Manifold[i];
            contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
            contact.Normal.Should().Be(Vector3d.Up);
            contact.PointA.Y.Should().Be(Fixed64.Zero);
            contact.PointB.Y.Should().Be(-Fixed64.FromFraction(1, 4));
        }
    }

    [Fact]
    public void MeshCuboid_WithCuboidContainedInClosedConvexMesh_ShouldUseConvexFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        CollisionPair forward = AssertCollision(scenario, mesh.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
        CollisionPair reversed = AssertCollision(scenario, cuboid.Collider, mesh.Collider, CollisionType.Mesh_Cuboid);

        forward.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        reversed.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MeshCuboid_WithRotatedCuboidSeparatedByEdgeAxis_ShouldNotUseReducedFallbackFalsePositive()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-3, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)10, (Fixed64)35, (Fixed64)15));

        AssertNoCollision(scenario, mesh.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
        AssertNoCollision(scenario, cuboid.Collider, mesh.Collider, CollisionType.Mesh_Cuboid);
    }

    [Fact]
    public void MeshMesh_ShouldPreserveTriangleContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            CreateVerticalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, first.Collider, second.Collider, CollisionType.Mesh_Mesh);

        pair.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void MeshMesh_WithRotatedConvexMeshSeparatedByEdgeAxis_ShouldNotUseReducedSatFalsePositive()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            new Vector3d(
                Fixed64.FromFraction(-1, 4),
                Fixed64.FromFraction(-5, 8),
                Fixed64.FromFraction(5, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)32, (Fixed64)(-20), (Fixed64)12));

        AssertNoCollision(scenario, first.Collider, second.Collider, CollisionType.Mesh_Mesh);
        AssertNoCollision(scenario, second.Collider, first.Collider, CollisionType.Mesh_Mesh);
    }

    [Fact]
    public void MeshCuboidSatChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(floor.Collider, cuboid.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MeshCuboidConvexFallback_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(mesh.Collider, cuboid.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MeshMeshSatChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            CreateVerticalPlaneMesh(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
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
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        Vector3d centerDelta = pair.ColliderB.Center - pair.ColliderA.Center;
        if (centerDelta.MagnitudeSquared > Fixed64.Epsilon)
            Vector3d.Dot(pair.Manifold.PrimaryContact.Normal, centerDelta).Should().BeGreaterThan(Fixed64.Zero);
        return pair;
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSteadyState(action);

    private static void EnsureCollision(CollisionPair pair)
    {
        if (!CollisionDetection.DoCollisionCheck(pair))
            throw new InvalidOperationException("Expected the prepared collision pair to collide.");
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
        pair.Manifold.Count.Should().Be(0);
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
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider CreateVerticalPlaneMesh() =>
        new(
            new[]
            {
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)2),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

    private static ScenarioBody<LSConeCollider> CreateCone(PhysicsScenarioBuilder scenario, Vector3d position) =>
        scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            position,
            FixedQuaternion.Identity);

    private static LSCapsuleCollider CreateTallCapsule() =>
        new()
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };
}
