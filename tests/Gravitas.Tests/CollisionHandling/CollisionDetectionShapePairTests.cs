using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections;
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

    [Theory]
    [InlineData(ColliderType.None, ColliderType.Sphere)]
    [InlineData(ColliderType.Sphere, ColliderType.None)]
    [InlineData(ColliderType.None, ColliderType.Compound)]
    [InlineData(ColliderType.Compound, ColliderType.None)]
    [InlineData((ColliderType)250, ColliderType.Sphere)]
    [InlineData(ColliderType.Sphere, (ColliderType)250)]
    public void CollisionTypeMatrix_ShouldRejectNoneAndUnknownTypes(ColliderType first, ColliderType second)
    {
        ColliderSettings.GetCollisionType(first, second).Should().Be(CollisionType.None);
    }

    [Fact]
    public void CollisionDetection_WithUnsupportedWorkItemType_ShouldRejectWithoutContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CollisionPair pair = scenario.CreatePair(
            scenario.CreateSphere(Vector3d.Zero).Collider,
            scenario.CreateSphere(Vector3d.Right).Collider);
        var unsupported = new CollisionWorkItem(
            pair.Context,
            pair.ColliderA,
            pair.ColliderB,
            CollisionType.None,
            pair.Manifold);

        CollisionDetection.DoCollisionCheck(unsupported).Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void PriorityMatrix_ShouldRankKnownColliderTypesAndRejectUnknownTypes()
    {
        ColliderSettings.GetPriority(ColliderType.Sphere).Should().Be(0);
        ColliderSettings.GetPriority(ColliderType.Capsule).Should().Be(1);
        ColliderSettings.GetPriority(ColliderType.Cylinder).Should().Be(1);
        ColliderSettings.GetPriority(ColliderType.Cone).Should().Be(1);
        ColliderSettings.GetPriority(ColliderType.AABox).Should().Be(2);
        ColliderSettings.GetPriority(ColliderType.OBBox).Should().Be(2);
        ColliderSettings.GetPriority(ColliderType.Mesh).Should().Be(3);
        ColliderSettings.GetPriority(ColliderType.Compound).Should().Be(4);
        ColliderSettings.GetPriority(ColliderType.None).Should().Be(-1);
        ColliderSettings.GetPriority((ColliderType)250).Should().Be(-1);
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
    public void CuboidCapsule_WithRoundedEdgeBoundsOverlap_ShouldRejectExactMissInBothPairOrders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion cuboidRotation = PhysicsScenarioBuilder.Yaw(45);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, cuboidRotation);
        Vector3d capsuleLocalCenter = new(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10));
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            cuboidRotation * capsuleLocalCenter,
            cuboidRotation * FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90)));
        Vector3d closestOnCapsule = GetClosestAxisPoint(
            capsule.Collider,
            cuboid.Collider.Center);
        Vector3d closestOnCuboid = cuboid.Collider.ClosestPointOnSurface(closestOnCapsule);

        cuboid.Collider.Shape.Should().Be(ColliderType.OBBox);
        Vector3d.Distance(
            cuboidRotation.Inverse() * capsule.Collider.WorldAxis,
            Vector3d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        cuboid.Collider.Bounds.Intersects(capsule.Collider.Bounds).Should().BeTrue();
        Vector3d.Distance(closestOnCapsule, closestOnCuboid).Should().BeGreaterThan(capsule.Collider.ScaledRadius);
        AssertNoCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.OBBox_Capsule);
        AssertNoCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.OBBox_Capsule);
    }

    [Fact]
    public void CuboidCapsule_WithSaturatedSquaredDistances_ShouldRejectRepresentableSeparation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(Vector3d.Zero);
        Fixed64 radius = (Fixed64)48_000;
        capsule.Collider.Size = Vector3d.One * (radius * 2);
        capsule.Collider.Radius = radius;
        capsule.Body.ResetPosition(new Vector3d((Fixed64)50_001, Fixed64.Zero, Fixed64.Zero));
        capsule.Collider.RebuildRuntimeShapeOnly();

        Vector3d separation = capsule.Collider.Center - cuboid.Collider.BoundsMax;
        separation.MagnitudeSquared.Should().Be(Fixed64.MaxValue);
        capsule.Collider.ScaledRadiusSqr.Should().Be(Fixed64.MaxValue);
        Vector3d.TryGetMagnitude(separation, out Fixed64 distance).Should().BeTrue();
        distance.Should().BeGreaterThan(radius);

        AssertNoCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        AssertNoCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.AABox_Capsule);
    }

    [Fact]
    public void CuboidCapsule_WithUnrepresentableCoreDistance_ShouldRejectWithoutFabricatingContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(Vector3d.Zero);
        capsule.Collider.Radius = Fixed64.One;
        capsule.Collider.Size = Vector3d.One * Fixed64.Two;
        capsule.Body.ResetPosition(new Vector3d(
            (Fixed64)1_600_000_000,
            (Fixed64)1_600_000_000,
            Fixed64.Zero));
        capsule.Collider.RebuildRuntimeShapeOnly();
        Vector3d coreSeparation = capsule.Collider.Center - cuboid.Collider.BoundsMax;

        coreSeparation.MagnitudeSquared.Should().Be(Fixed64.MaxValue);
        Vector3d.TryGetMagnitude(coreSeparation, out _).Should().BeFalse();
        capsule.Collider.ScaledRadiusSqr.Should().Be(Fixed64.One);
        AssertNoCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
    }

    [Fact]
    public void CuboidCapsule_AxisAlignedRoundedEdgeBoundsOverlap_ShouldRejectExactMissInBothPairOrders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90)));
        Vector3d closestOnCapsule = GetClosestAxisPoint(
            capsule.Collider,
            cuboid.Collider.Center);
        Vector3d closestOnCuboid = cuboid.Collider.ClosestPointOnSurface(closestOnCapsule);

        cuboid.Collider.Shape.Should().Be(ColliderType.AABox);
        capsule.Collider.WorldAxis.Should().Be(Vector3d.Right);
        cuboid.Collider.Bounds.Intersects(capsule.Collider.Bounds).Should().BeTrue();
        Vector3d.Distance(closestOnCapsule, closestOnCuboid).Should().BeGreaterThan(capsule.Collider.ScaledRadius);
        AssertNoCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        AssertNoCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.AABox_Capsule);
    }

    [Fact]
    public void CuboidCapsule_AxisAlignedRoundedCornerOverlap_ShouldUseRadialManifoldInBothPairOrders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = Vector3d.One
            },
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero),
            FixedQuaternion.Identity);
        Vector3d separation = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10), Fixed64.Zero);
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d expectedNormal = new(diagonal, diagonal, Fixed64.Zero);
        Vector3d expectedPointA = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        // Exact radial depth rounds once after subtracting the irrational
        // distance; subtracting the already-rounded magnitude is one raw unit lower.
        Fixed64 expectedDepth = Fixed64.FromRaw(325_283_348);

        CollisionPair forward = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        CollisionPair reverse = AssertCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.AABox_Capsule);

        foreach (CollisionPair pair in new[] { forward, reverse })
        {
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            contact.PointA.Should().Be(expectedPointA);
            contact.AnchorA.TryGetOffsetFrom(
                contact.AnchorB,
                out Vector3d witnessSeparation).Should().BeTrue();
            Vector3d.Distance(
                witnessSeparation,
                contact.Normal * contact.Depth)
                .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            contact.Depth.Should().Be(expectedDepth);
            contact.Normal.Should().Be(expectedNormal);
        }
    }

    [Fact]
    public void CuboidCapsule_CoreSeparatedBeyondLinearTolerance_ShouldUseRadialCornerManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        Fixed64 gap = Fixed64.FromFraction(1, 10_000);
        Vector3d separation = new(gap, gap, Fixed64.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = Vector3d.One * (radius * 2)
            },
            new Vector3d(Fixed64.Half + gap, Fixed64.Half + gap, Fixed64.Zero),
            FixedQuaternion.Identity);
        Vector3d expectedNormal = separation.Normalized;

        CollisionPair pair = AssertCollision(
            scenario,
            cuboid.Collider,
            capsule.Collider,
            CollisionType.AABox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;

        Vector3d.Distance(contact.Normal, expectedNormal)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(
            contact.PointA,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero))
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - (radius - separation.Magnitude))
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void CuboidCapsule_AxisAlignedTallOverlap_ShouldBuildExactStableManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.PointA.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        pair.Manifold.PrimaryContact.PointB.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void CuboidCapsule_AxisAlignedExactTouch_ShouldBuildZeroDepthManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            Vector3d.Right,
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        CollisionPair reverse = AssertCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.AABox_Capsule);

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.PointA.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        pair.Manifold.PrimaryContact.PointB.Should().Be(pair.Manifold.PrimaryContact.PointA);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
        reverse.Manifold.PrimaryContact.Should().Be(pair.Manifold.PrimaryContact);
    }

    [Fact]
    public void CuboidCapsule_OffCenterContainedSphereLimit_ShouldUseNearestExitManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = Vector3d.One * (radius * Fixed64.Two)
            },
            new Vector3d(Fixed64.FromFraction(1, 5), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;

        contact.PointA.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        contact.PointB.Should().Be(capsule.Collider.Center - Vector3d.Right * capsule.Collider.ScaledRadius);
        contact.Depth.Should().Be(Fixed64.Half - capsule.Collider.Center.X + capsule.Collider.ScaledRadius);
        contact.Normal.Should().Be(Vector3d.Right);
        Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal).Should().Be(contact.Depth);
    }

    [Fact]
    public void CuboidCapsule_CoreEndpointOnCorner_ShouldUseCanonicalEdgeRepresentative()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        Vector3d touchingEndpoint = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsuleFromSegment(
            scenario,
            touchingEndpoint,
            new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(51, 100), Fixed64.Zero),
            Fixed64.FromFraction(1, 10));

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        CollisionPair reverse = AssertCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.AABox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;

        contact.PointA.Should().Be(touchingEndpoint);
        contact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        contact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
        Vector3d.Dot(contact.Normal, capsule.Collider.WorldAxis)
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d localNormal =
            (capsule.Collider.Rotation.Inverse() * -contact.Normal).Normalized;
        FixedPointAnchor expectedCapsuleAnchor =
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                cuboid.Collider.Center,
                capsule.Collider.Center,
                capsule.Collider.Rotation,
                Vector3d.Up,
                capsule.Collider.AxisLength,
                capsule.Collider.ScaledRadius,
                localNormal);
        expectedCapsuleAnchor.TryGetPoint(out Vector3d expectedPointB)
            .Should().BeTrue();
        Vector3d.Distance(contact.PointB, expectedPointB)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - capsule.Collider.ScaledRadius).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        reverse.Manifold.PrimaryContact.Should().Be(contact);
    }

    [Fact]
    public void CuboidCapsule_RotatedTallOverlap_ShouldBuildExactStableManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion cuboidRotation = PhysicsScenarioBuilder.Yaw(45);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, cuboidRotation);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            cuboidRotation * new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            cuboidRotation * FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90)));

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.OBBox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;

        Vector3d.Distance(
            contact.PointA,
            cuboidRotation * new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero)).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(
            contact.PointB,
            cuboidRotation * new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero)).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - Fixed64.FromFraction(1, 4)).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.Normal, cuboidRotation * Vector3d.Up).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void CuboidCapsule_RotatedExactTouch_ShouldBuildZeroDepthManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion cuboidRotation = PhysicsScenarioBuilder.Yaw(45);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, cuboidRotation);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            cuboidRotation * Vector3d.Right,
            cuboidRotation);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.OBBox_Capsule);
        CollisionPair reverse = AssertCollision(scenario, capsule.Collider, cuboid.Collider, CollisionType.OBBox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d expectedPoint = cuboidRotation * new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);

        Vector3d.Distance(contact.PointA, expectedPoint).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.PointB, expectedPoint).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        contact.Depth.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.Normal, cuboidRotation * Vector3d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        reverse.Manifold.PrimaryContact.Should().Be(contact);
    }

    [Fact]
    public void CuboidCapsule_PitchYawRollExactTouch_ShouldPreserveInclusiveFaceContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)23, (Fixed64)37, (Fixed64)11);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, rotation);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = Vector3d.One },
            rotation * Vector3d.Right,
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.OBBox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d expectedPoint = rotation * new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);

        Vector3d.Distance(contact.PointA, expectedPoint).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.PointB, expectedPoint).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        contact.Depth.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.Normal, rotation * Vector3d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void CuboidCapsule_PitchYawRollContainedSphereLimit_ShouldMatchFaceCenterWitness()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)23, (Fixed64)37, (Fixed64)11);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, rotation);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = Vector3d.One * (radius * Fixed64.Two)
            },
            rotation * new Vector3d(Fixed64.FromFraction(1, 5), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.OBBox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d expectedPointA = rotation * new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        Vector3d expectedPointB = rotation * new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.Zero);

        Vector3d.Distance(contact.PointA, expectedPointA).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.PointB, expectedPointB).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - Fixed64.FromFraction(2, 5)).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(contact.Normal, rotation * Vector3d.Right).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void CuboidCapsuleChecks_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(45);
        CollisionPair[] pairs =
        {
            scenario.CreatePair(
                scenario.CreateCuboid(Vector3d.Zero).Collider,
                scenario.CreateBody(
                    new LSCapsuleCollider
                    {
                        Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
                    },
                    new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.Identity).Collider),
            scenario.CreatePair(
                scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero), rotation).Collider,
                scenario.CreateBody(
                    new LSCapsuleCollider
                    {
                        Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
                    },
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)
                        + rotation * new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.FromFraction(2, 5)),
                    rotation * FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90))).Collider)
        };

        long axisAlignedAllocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pairs[0]));
        long rotatedAllocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pairs[1]));

        axisAlignedAllocatedBytes.Should().Be(0);
        rotatedAllocatedBytes.Should().Be(0);
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
    public void CylinderSphere_AcrossUnrepresentableCoordinateSpan_ShouldRejectCollision()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(
            new Vector3d(
                Fixed64.MinValue + (Fixed64)4,
                Fixed64.Zero,
                Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(
                Fixed64.MaxValue - (Fixed64)4,
                Fixed64.Zero,
                Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(
            cylinder.Collider,
            sphere.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CylinderSphere_WithFullDomainContainment_ShouldClampContactDepth()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder =
            scenario.CreateCylinder(Vector3d.Zero);
        var sphere = new LSSphereCollider
        {
            Radius = Fixed64.MaxValue,
        };
        scenario.InitializeStaticCollider(sphere, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(cylinder.Collider, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.MaxValue);
        pair.Manifold.PrimaryContact.DepthIsClamped.Should().BeTrue();
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
    public void CylinderCapsule_WithCylinderAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            FixedQuaternion.Identity);

        AssertNoCollision(scenario, cylinder.Collider, capsule.Collider, CollisionType.Cylinder_Capsule);
    }

    [Fact]
    public void CylinderCapsule_WithCapsuleAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        AssertNoCollision(scenario, cylinder.Collider, capsule.Collider, CollisionType.Cylinder_Capsule);
    }

    [Fact]
    public void CylinderCapsule_WithCrossAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(5, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        AssertNoCollision(scenario, cylinder.Collider, capsule.Collider, CollisionType.Cylinder_Capsule);
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
    public void CylinderCylinder_WithSecondCylinderAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinderA = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSCylinderCollider> cylinderB = scenario.CreateBody(
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d(-Fixed64.FromFraction(19, 16), -Fixed64.FromFraction(11, 8), Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-15)));

        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, cylinderA.Collider.WorldAxis).Should().BeTrue();
        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, cylinderB.Collider.WorldAxis).Should().BeFalse();
        CylinderProjectionsOverlap(
            cylinderA.Collider,
            cylinderB.Collider,
            Vector3d.Cross(cylinderA.Collider.WorldAxis, cylinderB.Collider.WorldAxis)).Should().BeTrue();
        FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            cylinderA.Collider.Center,
            cylinderA.Collider.WorldAxis,
            cylinderA.Collider.Height,
            cylinderB.Collider.Center,
            cylinderB.Collider.WorldAxis,
            cylinderB.Collider.Height,
            out Vector3d pointA,
            out Vector3d pointB).Should().BeTrue();
        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, pointB - pointA).Should().BeTrue();
        AssertNoCollision(scenario, cylinderA.Collider, cylinderB.Collider, CollisionType.Cylinder_Cylinder);
    }

    [Fact]
    public void CylinderCylinder_WithAxisCrossSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinderA = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSCylinderCollider> cylinderB = scenario.CreateBody(
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(5, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, cylinderA.Collider.WorldAxis).Should().BeTrue();
        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, cylinderB.Collider.WorldAxis).Should().BeTrue();
        Vector3d crossAxis = Vector3d.Cross(cylinderA.Collider.WorldAxis, cylinderB.Collider.WorldAxis);
        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, crossAxis).Should().BeFalse();
        FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            cylinderA.Collider.Center,
            cylinderA.Collider.WorldAxis,
            cylinderA.Collider.Height,
            cylinderB.Collider.Center,
            cylinderB.Collider.WorldAxis,
            cylinderB.Collider.Height,
            out Vector3d pointA,
            out Vector3d pointB).Should().BeTrue();
        CylinderProjectionsOverlap(cylinderA.Collider, cylinderB.Collider, pointB - pointA).Should().BeFalse();
        AssertNoCollision(scenario, cylinderA.Collider, cylinderB.Collider, CollisionType.Cylinder_Cylinder);
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
    public void CuboidCylinder_WithCylinderAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 8), Fixed64.Zero));

        AssertNoCollision(scenario, cuboid.Collider, cylinder.Collider, CollisionType.Cuboid_Cylinder);
    }

    [Fact]
    public void CuboidCylinder_WithEdgeCrossAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        // Face axes overlap; separation exists only on the first edge x cylinder-axis test.
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateBody(
            new LSCylinderCollider { Size = Vector3d.One },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(-7, 8), Fixed64.FromFraction(7, 8)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, Fixed64.Zero, Fixed64.Zero));

        AssertNoCollision(scenario, cuboid.Collider, cylinder.Collider, CollisionType.Cuboid_Cylinder);
    }

    [Fact]
    public void CuboidCylinder_WithClosestFeatureAxisSeparation_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);
        // Face and edge axes overlap; the diagonal closest-feature axis separates the pair.
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(
            new Vector3d(Fixed64.FromFraction(7, 8), Fixed64.Zero, Fixed64.FromFraction(7, 8)));

        AssertNoCollision(scenario, cuboid.Collider, cylinder.Collider, CollisionType.Cuboid_Cylinder);
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
    public void ConeSphere_AcrossUnrepresentableCoordinateSpan_ShouldRejectCollision()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d(
                Fixed64.MinValue + (Fixed64)4,
                Fixed64.Zero,
                Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(
                Fixed64.MaxValue - (Fixed64)4,
                Fixed64.Zero,
                Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(
            cone.Collider,
            sphere.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
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
    public void ConeConvex_WithCoincidentCenters_ShouldUseDeterministicFallbackNormal()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);

        CollisionPair pair = AssertCollision(scenario, cone.Collider, cuboid.Collider, CollisionType.Cone_Convex);

        pair.Manifold.PrimaryContact.Normal.Should().Be(-Vector3d.Right);
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Fact]
    public void ConeConvex_WithMinimumRepresentableGap_ShouldClampToleranceContactDepthToZero()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)(-90));
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, Vector3d.Zero, rotation);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(
            new Vector3d(Fixed64.FromFraction(3, 2) + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            rotation);

        CollisionPair pair = AssertCollision(scenario, cone.Collider, capsule.Collider, CollisionType.Cone_Convex);

        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
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
    public void MeshCone_WithBackFacingTriangleCrossingConeSide_ShouldUseWindingForContainment()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> wall = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d(-Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, wall.Collider, cone.Collider, CollisionType.Mesh_Cone);

        pair.Manifold.PrimaryContact.Normal.X.Should().BeLessThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointA.X.Should().Be(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointB.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MeshCone_WithOffsetMeshOrigin_ShouldOrientFromCandidateTriangle()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> wall = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
                    new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2),
                    new Vector3d((Fixed64)10, (Fixed64)(-2), (Fixed64)(-2)),
                    new Vector3d((Fixed64)10, (Fixed64)2, (Fixed64)(-2)),
                    new Vector3d((Fixed64)10, Fixed64.Zero, (Fixed64)2)
                },
                new[] { 0, 1, 2, 3, 4, 5 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.Zero));

        wall.Collider.Center.X.Should().Be((Fixed64)5);
        CollisionPair pair = scenario.CreatePair(wall.Collider, cone.Collider);

        pair.CollisionType.Should().Be(CollisionType.Mesh_Cone);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();

        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointA.X.Should().Be(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointB.X.Should().BeLessThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 10));
    }

    [Fact]
    public void MeshCone_WithOverlappingBoundsBeyondTrianglePlane_ShouldRejectSupportContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d(Fixed64.FromFraction(9, 20), -Fixed64.One, Fixed64.FromFraction(9, 20)),
                new Vector3d(Fixed64.FromFraction(9, 20), Fixed64.One, Fixed64.FromFraction(9, 20)),
                new Vector3d(Fixed64.FromFraction(9, 10), -Fixed64.One, Fixed64.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, Vector3d.Zero);

        mesh.Collider.BoundsMin.X.Should().BeLessThan(cone.Collider.BoundsMax.X);
        mesh.Collider.BoundsMin.Z.Should().BeLessThan(cone.Collider.BoundsMax.Z);

        AssertNoCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);
    }

    [Fact]
    public void MeshConeTriangleSupport_AtPositiveEpsilonGap_ShouldAdmitZeroDepthToleranceContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 planeXShift = Fixed64.FromFraction(1, 6)
            - Fixed64.Epsilon * Fixed64.FromFraction(5, 3);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d(planeXShift, Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-4) + planeXShift, (Fixed64)3, (Fixed64)2),
                new Vector3d((Fixed64)4 + planeXShift, (Fixed64)(-3), (Fixed64)2)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);

        Vector3d.Dot(
                pair.Manifold.PrimaryContact.PointB - pair.Manifold.PrimaryContact.PointA,
                pair.Manifold.PrimaryContact.Normal)
            .Should().Be(Fixed64.Epsilon);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
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
    public void MeshCone_WithSeparatedConcaveMesh_ShouldReturnFalseWithoutConvexFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateHorizontalPlaneMesh(MeshColliderMode.Concave),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        mesh.Collider.Mode.Should().Be(MeshColliderMode.Concave);

        AssertNoCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);
    }

    [Fact]
    public void MeshCone_WithSeparatedConvexMesh_ShouldRejectConvexFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(scenario, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        mesh.Collider.Mode.Should().Be(MeshColliderMode.Convex);

        AssertNoCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);
    }

    [Theory]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    public void FiniteAxisMeshContact_WithOverlappingClippedBoundsAndUnrepresentableFrameOffset_ShouldReject(
        ColliderType shape)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-3), -Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)3, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(
            mesh,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        LSCollider finiteAxis = shape == ColliderType.Cylinder
            ? new LSCylinderCollider
            {
                Radius = Fixed64.MaxValue,
                Size = Vector3d.One
            }
            : new LSConeCollider
            {
                Radius = Fixed64.MaxValue,
                Size = Vector3d.One
            };
        scenario.InitializeStaticCollider(
            finiteAxis,
            new Vector3d(
                Fixed64.MinValue + Fixed64.Two,
                Fixed64.Zero,
                Fixed64.Zero));

        mesh.Bounds.Intersects(finiteAxis.Bounds).Should().BeTrue();
        new FixedPointAnchor(
                finiteAxis.Center,
                FixedQuaternion.Identity,
                Vector3d.Zero)
            .TryGetLocalPointIn(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out _)
            .Should().BeFalse();
        if (finiteAxis is LSCylinderCollider cylinder)
        {
            CylinderContactGeometry.IsAxisAligned(
                    cylinder.Rotation,
                    Vector3d.Up,
                    mesh.Mesh.GetFaceNormalWorld(0))
                .Should().BeFalse();
        }
        AssertNoCollision(
            scenario,
            mesh,
            finiteAxis,
            shape == ColliderType.Cylinder
                ? CollisionType.Mesh_Cylinder
                : CollisionType.Mesh_Cone);
    }

    [Fact]
    public void MeshCone_WithMinimumRepresentableGap_ShouldClampToleranceContactDepthToZero()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)(-90));
        // The unique +X support vertex matches the rotated cone's -X support
        // so the one-raw-unit GJK tolerance is isolated from tie-breaking.
        LSMeshCollider convexMesh = new(
            new[]
            {
                new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
                new Vector3d(-Fixed64.Half, -Fixed64.Half, -Fixed64.Half),
                new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero),
                new Vector3d(-Fixed64.Half, -Fixed64.Half, Fixed64.Half)
            },
            new[] { 0, 2, 1, 0, 3, 2, 0, 1, 3, 1, 2, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            convexMesh,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSConeCollider> cone = CreateCone(
            scenario,
            new Vector3d(Fixed64.FromFraction(3, 2) + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            rotation);

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, cone.Collider, CollisionType.Mesh_Cone);

        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
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
    public void MeshSphere_WithCenterAboveConvexSurface_ShouldUseSurfaceDistance()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, floor.Collider, sphere.Collider, CollisionType.Mesh_Sphere);

        pair.Manifold.PrimaryContact.PointA.Should().Be(Vector3d.Zero);
        pair.Manifold.PrimaryContact.PointB.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(1, 4), Fixed64.Zero));
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void MeshSphere_WithCenterBeyondRadius_ShouldReturnSeparatedResult()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Up);

        AssertNoCollision(scenario, floor.Collider, sphere.Collider, CollisionType.Mesh_Sphere);
    }

    [Fact]
    public void MeshSphere_WithDisconnectedConcaveTriangles_ShouldNotUseBoundsAsSurface()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)4, (Fixed64)(-4), (Fixed64)(-4)),
                    new Vector3d((Fixed64)4, (Fixed64)(-3), (Fixed64)(-4)),
                    new Vector3d((Fixed64)4, (Fixed64)(-4), (Fixed64)(-3)),
                    new Vector3d((Fixed64)(-4), (Fixed64)4, (Fixed64)(-4)),
                    new Vector3d((Fixed64)(-3), (Fixed64)4, (Fixed64)(-4)),
                    new Vector3d((Fixed64)(-4), (Fixed64)4, (Fixed64)(-3)),
                    new Vector3d((Fixed64)(-4), (Fixed64)(-4), (Fixed64)4),
                    new Vector3d((Fixed64)(-3), (Fixed64)(-4), (Fixed64)4),
                    new Vector3d((Fixed64)(-4), (Fixed64)(-3), (Fixed64)4)
                },
                new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d boundsPoint = new((Fixed64)4, (Fixed64)4, (Fixed64)4);
        Vector3d sphereCenter = boundsPoint + Vector3d.Right * Fixed64.FromFraction(1, 4);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(sphereCenter);

        mesh.Collider.Bounds.ClosestPointOnSurface(sphereCenter).Should().Be(boundsPoint);
        Vector3d.Distance(boundsPoint, sphereCenter).Should().BeLessThan(sphere.Collider.ScaledRadius);
        AssertNoCollision(scenario, mesh.Collider, sphere.Collider, CollisionType.Mesh_Sphere);
    }

    [Fact]
    public void MeshSphere_WithSphereCenterOnConcaveTriangle_ShouldUseFaceNormalFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4)));

        CollisionPair pair = scenario.CreatePair(mesh.Collider, sphere.Collider);

        pair.CollisionType.Should().Be(CollisionType.Mesh_Sphere);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
        pair.Manifold.PrimaryContact.PointA.Should().Be(sphere.Collider.Center);
    }

    [Fact]
    public void MeshCapsule_WithSegmentCrossingTriangleProjection_ShouldUseEdgeClosestPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)(-1), Fixed64.Zero, -Fixed64.Half),
                new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.Half),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)5, Fixed64.One)
            },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)90, Fixed64.Zero, Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);

        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        pair.Manifold.PrimaryContact.PointA.Y.Should().Be(Fixed64.Zero);
        pair.Manifold.PrimaryContact.PointB.Y.Should().Be(-Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void MeshCuboid_WithOverlappingBoundsSeparatedOnTriangleNormal_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)2),
                new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));

        AssertNoCollision(scenario, mesh.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
    }

    [Fact]
    public void MeshCylinder_WithOverlappingBoundsAndSeparatedCapPlane_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)2),
                new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-45)));
        Vector3d triangleNormal = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;

        Vector3d.Dot(cylinder.Collider.WorldAxis, triangleNormal).Abs()
            .Should().BeGreaterThan(Fixed64.FromFraction(63, 64));
        AssertNoCollision(scenario, mesh.Collider, cylinder.Collider, CollisionType.Mesh_Cylinder);
    }

    [Fact]
    public void MeshMesh_WithOverlappingBoundsSeparatedOnEitherFaceNormal_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> horizontal = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)2, (Fixed64)(-1), Fixed64.Zero),
                new Vector3d((Fixed64)(-1), (Fixed64)2, Fixed64.Zero),
                new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> diagonal = scenario.CreateBody(
            CreateConcaveTriangle(
                new Vector3d((Fixed64)(-2), (Fixed64)2, (Fixed64)(-1)),
                new Vector3d((Fixed64)2, (Fixed64)(-2), (Fixed64)(-1)),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        AssertNoCollision(scenario, horizontal.Collider, diagonal.Collider, CollisionType.Mesh_Mesh);
        AssertNoCollision(scenario, diagonal.Collider, horizontal.Collider, CollisionType.Mesh_Mesh);
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
    public void MeshCapsule_WithConvexCubeRoundedEdgeBoundsOverlap_ShouldRejectExactMissInBothPairOrders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90)));
        Vector3d closestOnCapsule = GetClosestAxisPoint(
            capsule.Collider,
            mesh.Collider.Center);
        Vector3d closestOnCube = new(Fixed64.Zero, Fixed64.Half, Fixed64.Half);

        mesh.Collider.Bounds.Intersects(capsule.Collider.Bounds).Should().BeTrue();
        Vector3d.Distance(closestOnCapsule, closestOnCube).Should().BeGreaterThan(capsule.Collider.ScaledRadius);
        AssertNoCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);
        AssertNoCollision(scenario, capsule.Collider, mesh.Collider, CollisionType.Mesh_Capsule);
    }

    [Fact]
    public void MeshCapsule_WithConvexCubeTallOverlap_ShouldBuildExactStableManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.PointA.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        pair.Manifold.PrimaryContact.PointB.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void MeshCapsule_WithSphereLimitContainedByClosedConvexCube_ShouldUseNearestExitManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = Vector3d.One * (radius * Fixed64.Two)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;

        contact.PointA.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half));
        contact.PointB.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, capsule.Collider.ScaledRadius));
        contact.Depth.Should().Be(Fixed64.Half + capsule.Collider.ScaledRadius);
        contact.Normal.Should().Be(-Vector3d.Forward);
    }

    [Fact]
    public void MeshCapsule_WithAngledCoreContainedByClosedConvexCube_ShouldUseWholeCapsuleExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateElongatedConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsuleFromSegment(
            scenario,
            new Vector3d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(-2, 5), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(1, 5), Fixed64.Zero),
            Fixed64.FromFraction(1, 10));

        CollisionPair pair = AssertCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d expectedNormal = new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(-4, 5), Fixed64.Zero);
        Fixed64 expectedDepth = Fixed64.FromFraction(18, 25);

        Vector3d.Distance(contact.Normal, expectedNormal).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - expectedDepth).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        mesh.Collider.ClosestPointOnSurface(contact.PointA).Should().Be(contact.PointA);
    }

    [Fact]
    public void MeshCapsule_ContainedByScaledRotatedOffOriginReverseWoundCube_ShouldUseScaledInteriorPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        LSMeshCollider source = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = source.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] += Vector3d.Right;

        int[] triangles = source.Mesh.Triangles.ToArray();
        for (int i = 0; i < triangles.Length; i += 3)
            (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);

        var meshCollider = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
        Vector3d position = new((Fixed64)3, (Fixed64)2, (Fixed64)(-4));
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(35);
        Vector3d scale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        var transform = new FixedTransform(position, rotation, scale);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var meshBody = new SolidBody(agent, meshCollider) { Mass = Fixed64.One };
        meshBody.Initialize(position, rotation);
        Vector3d meshCenter = meshCollider.Bounds.Center;
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = Vector3d.One * (radius * Fixed64.Two)
            },
            meshCenter,
            FixedQuaternion.Identity);

        meshCollider.Bounds.Contains(meshCenter).Should().BeTrue();
        meshCollider.Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties massProperties, out _)
            .Should().BeTrue();
        Vector3d.Distance(
            massProperties.CenterOfMass,
            Vector3d.Zero).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(
            meshCollider.Mesh.ConvertScaledLocalToWorld(massProperties.CenterOfMass),
            meshCenter).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);

        CollisionPair pair = AssertCollision(scenario, meshCollider, capsule.Collider, CollisionType.Mesh_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d scaledXAxis = rotation * Vector3d.Right;

        (contact.Depth - (Fixed64.One + capsule.Collider.ScaledRadius))
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Dot(contact.Normal, scaledXAxis).Abs().Should().BeGreaterThan(Fixed64.One - Fixed64.Epsilon);
        (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        meshCollider.ClosestPointOnSurface(contact.PointA).Should().Be(contact.PointA);
    }

    [Fact]
    public void CuboidCapsule_WithAngledContainedCore_ShouldUseMatchedEdgeFeatureWitnesses()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.One, Fixed64.One, (Fixed64)4)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsuleFromSegment(
            scenario,
            new Vector3d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(-2, 5), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(1, 5), Fixed64.Zero),
            Fixed64.FromFraction(1, 10));

        CollisionPair pair = AssertCollision(scenario, cuboid.Collider, capsule.Collider, CollisionType.AABox_Capsule);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d expectedNormal = new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(-4, 5), Fixed64.Zero);
        Fixed64 expectedDepth = Fixed64.FromFraction(18, 25);

        Vector3d.Distance(contact.Normal, expectedNormal).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - expectedDepth).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(
            contact.PointA,
            new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.Zero)).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        cuboid.Collider.ClosestPointOnSurface(contact.PointA).Should().Be(contact.PointA);
    }

    [Fact]
    public void MeshCapsule_WithSeparatedCenterRepresentativeButSurfaceContact_ShouldUseTriangleFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = CreateOffRepresentativeMeshCapsule(scenario);
        Vector3d centerRepresentative = GetClosestAxisPoint(
            capsule.Collider,
            mesh.Collider.Center);
        Vector3d representativeSurfacePoint = mesh.Collider.ClosestPointOnSurface(centerRepresentative);

        Vector3d.Distance(centerRepresentative, representativeSurfacePoint)
            .Should().BeGreaterThan(capsule.Collider.ScaledRadius);
        ConvexColliderSupport.Intersects(mesh.Collider, capsule.Collider).Should().BeTrue();
        CollisionPair pair = AssertCollision(scenario, mesh.Collider, capsule.Collider, CollisionType.Mesh_Capsule);

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MeshCapsuleConvexFallback_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCapsuleCollider> capsule = CreateOffRepresentativeMeshCapsule(scenario);
        CollisionPair pair = scenario.CreatePair(mesh.Collider, capsule.Collider);
        Vector3d centerRepresentative = GetClosestAxisPoint(
            capsule.Collider,
            mesh.Collider.Center);

        Vector3d.Distance(centerRepresentative, mesh.Collider.ClosestPointOnSurface(centerRepresentative))
            .Should().BeGreaterThan(capsule.Collider.ScaledRadius);

        long allocatedBytes = MeasureAllocatedBytes(() => EnsureCollision(pair));

        allocatedBytes.Should().Be(0);
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
    public void MeshCuboid_WithNonUniformMeshScale_ShouldUseScaledCollisionGeometry()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            CreateHorizontalPlaneMesh(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        floor.Body.PositionTransform.LocalScale = new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)3);
        floor.Collider.Simulate();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d((Fixed64)3, Fixed64.FromFraction(1, 4), Fixed64.Zero));

        CollisionPair pair = AssertCollision(scenario, floor.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);

        pair.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
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
    public void MeshCuboid_WithOverlappingBoundsSeparatedByMeshFace_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion meshRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            meshRotation);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            (meshRotation * Vector3d.Right) * Fixed64.FromFraction(5, 4));

        mesh.Collider.Bounds.Intersects(cuboid.Collider.Bounds).Should().BeTrue();
        AssertNoCollision(scenario, mesh.Collider, cuboid.Collider, CollisionType.Mesh_Cuboid);
    }

    [Fact]
    public void MeshCuboid_WithOverlappingBoundsSeparatedByCuboidFace_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion middleFaceRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)45);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> middleFaceCuboid = scenario.CreateCuboid(
            (middleFaceRotation * Vector3d.Up) * Fixed64.FromFraction(5, 4),
            middleFaceRotation);
        FixedQuaternion firstFaceRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> firstFaceCuboid = scenario.CreateCuboid(
            (firstFaceRotation * Vector3d.Forward) * Fixed64.FromFraction(5, 4),
            firstFaceRotation);

        mesh.Collider.Bounds.Intersects(middleFaceCuboid.Collider.Bounds).Should().BeTrue();
        mesh.Collider.Bounds.Intersects(firstFaceCuboid.Collider.Bounds).Should().BeTrue();
        AssertNoCollision(scenario, mesh.Collider, middleFaceCuboid.Collider, CollisionType.Mesh_Cuboid);
        AssertNoCollision(scenario, mesh.Collider, firstFaceCuboid.Collider, CollisionType.Mesh_Cuboid);
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
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-9, 8),
                Fixed64.FromFraction(-3, 4)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)10, (Fixed64)35, (Fixed64)15));

        first.Collider.Bounds.Intersects(second.Collider.Bounds).Should().BeTrue();
        AssertNoCollision(scenario, first.Collider, second.Collider, CollisionType.Mesh_Mesh);
        AssertNoCollision(scenario, second.Collider, first.Collider, CollisionType.Mesh_Mesh);
    }

    [Fact]
    public void MeshMesh_WithTouchingFaces_ShouldNotTreatAdjacentProjectionsAsOverlap()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Right,
            FixedQuaternion.Identity);

        AssertNoCollision(scenario, first.Collider, second.Collider, CollisionType.Mesh_Mesh);
    }

    [Fact]
    public void MeshMesh_WithOverlappingBoundsSeparatedByEitherFaceSource_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion firstRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        ScenarioBody<LSMeshCollider> first = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero,
            firstRotation);
        ScenarioBody<LSMeshCollider> second = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(),
            (firstRotation * Vector3d.Right) * Fixed64.FromFraction(5, 4),
            FixedQuaternion.Identity);

        first.Collider.Bounds.Intersects(second.Collider.Bounds).Should().BeTrue();
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

    private static bool CylinderProjectionsOverlap(
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        Vector3d axis)
    {
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        return FixedSegment.DoCenteredFiniteCylindersOverlapOnAxis(
            axis.Normalized,
            cylinderA.Center,
            cylinderA.WorldAxis,
            cylinderA.Height,
            cylinderA.ScaledRadius,
            cylinderB.Center,
            cylinderB.WorldAxis,
            cylinderB.Height,
            cylinderB.ScaledRadius);
    }

    private static LSMeshCollider CreateHorizontalPlaneMesh(MeshColliderMode mode = MeshColliderMode.Convex) =>
        new(
            new[]
            {
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 },
            mode,
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

    private static LSMeshCollider CreateConcaveTriangle(Vector3d first, Vector3d second, Vector3d third) =>
        new(
            new[] { first, second, third },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

    private static ScenarioBody<LSConeCollider> CreateCone(
        PhysicsScenarioBuilder scenario,
        Vector3d position,
        FixedQuaternion? rotation = null) =>
        scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            position,
            rotation ?? FixedQuaternion.Identity);

    private static LSCapsuleCollider CreateTallCapsule() =>
        new()
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };

    private static Vector3d GetClosestAxisPoint(
        LSCapsuleCollider capsule,
        Vector3d point)
    {
        FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisLength,
            point,
            Vector3d.Right,
            Fixed64.Zero,
            out Vector3d closest,
            out _).Should().BeTrue();
        return closest;
    }

    private static LSMeshCollider CreateElongatedConvexCube()
    {
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i].Z *= 4;

        return new LSMeshCollider(
            vertices,
            cube.Mesh.Triangles.ToArray(),
            MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
    }

    private static ScenarioBody<LSCapsuleCollider> CreateOffRepresentativeMeshCapsule(
        PhysicsScenarioBuilder scenario)
    {
        Vector3d segmentStart = new(
            Fixed64.FromFraction(5, 4),
            Fixed64.FromFraction(3, 2),
            Fixed64.Half);
        Vector3d segmentEnd = new(
            Fixed64.Half,
            Fixed64.FromFraction(-5, 4),
            Fixed64.FromFraction(-3, 4));
        return CreateCapsuleFromSegment(
            scenario,
            segmentStart,
            segmentEnd,
            Fixed64.FromFraction(1, 4));
    }

    private static ScenarioBody<LSCapsuleCollider> CreateCapsuleFromSegment(
        PhysicsScenarioBuilder scenario,
        Vector3d segmentStart,
        Vector3d segmentEnd,
        Fixed64 radius)
    {
        Vector3d segment = segmentEnd - segmentStart;
        Fixed64 segmentLength = segment.Magnitude;
        Vector3d segmentDirection = segment / segmentLength;
        Vector3d rotationAxis = Vector3d.Cross(Vector3d.Up, segmentDirection);
        FixedQuaternion rotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One + Vector3d.Dot(Vector3d.Up, segmentDirection)).Normalized;
        return scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = new Vector3d(
                    radius * 2,
                    segmentLength + radius * 2,
                    radius * 2)
            },
            (segmentStart + segmentEnd) * Fixed64.Half,
            rotation);
    }
}
