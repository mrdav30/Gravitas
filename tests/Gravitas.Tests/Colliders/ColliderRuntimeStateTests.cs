using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderRuntimeStateTests
{
    [Fact]
    public void ConeShape_ShouldUseBoundingCenterOriginAndAsymmetricMassProperties()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> coneBody = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: (Fixed64)4);
        LSConeCollider cone = coneBody.Collider;

        cone.BaseCenter.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        cone.Apex.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        cone.Axis.Should().Be(Vector3d.Up);
        cone.Volume.Should().Be(Fixed64.Pi / (Fixed64)6);
        cone.CalculateLocalCenterOfMassOffset().Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        coneBody.Body.LocalCenterOfMassOffset.Should().Be(cone.CalculateLocalCenterOfMassOffset());

        Fixed3x3 inertia = cone.CalculateInertiaTensor((Fixed64)4, cone.CalculateLocalCenterOfMassOffset());

        AssertNear(inertia.M11, Fixed64.FromFraction(3, 4));
        AssertNear(inertia.M22, Fixed64.FromFraction(3, 10));
        AssertNear(inertia.M33, Fixed64.FromFraction(3, 4));
        inertia.M12.Should().Be(Fixed64.Zero);
        inertia.M13.Should().Be(Fixed64.Zero);
        inertia.M23.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ConeShape_WithCompoundRotation_ShouldRotateCenterOfMassIntoOwnerLocalSpace()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(
                ColliderShapeDefinition.Cone(Fixed64.Half, (Fixed64)2),
                Vector3d.Zero,
                FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90),
                Vector3d.One));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: Fixed64.One);

        AssertNear(body.Body.LocalCenterOfMassOffset.X, Fixed64.Half);
        AssertNear(body.Body.LocalCenterOfMassOffset.Y, Fixed64.Zero);
        AssertNear(body.Body.LocalCenterOfMassOffset.Z, Fixed64.Zero);
    }

    [Fact]
    public void ConeShape_WithArbitraryRotation_ShouldUseFiniteConeBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> coneBody = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.One, Fixed64.FromFraction(1, 4), -Fixed64.Half),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)25, (Fixed64)(-35), (Fixed64)50));
        LSConeCollider cone = coneBody.Collider;

        ConeGeometry.CreateFiniteConeBounds(
            cone.WorldApex,
            cone.WorldBaseCenter,
            cone.Axis,
            cone.ScaledRadius,
            out Vector3d expectedMin,
            out Vector3d expectedMax);

        AssertNear(cone.BoundsMin.X, expectedMin.X);
        AssertNear(cone.BoundsMin.Y, expectedMin.Y);
        AssertNear(cone.BoundsMin.Z, expectedMin.Z);
        AssertNear(cone.BoundsMax.X, expectedMax.X);
        AssertNear(cone.BoundsMax.Y, expectedMax.Y);
        AssertNear(cone.BoundsMax.Z, expectedMax.Z);
    }

    [Fact]
    public void CapsuleShapeMutations_ShouldRebuildDerivedStateOncePerSimulate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsuleBody = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(0, 0, 0));
        LSCapsuleCollider capsule = capsuleBody.Collider;

        uint initialVersion = capsule.RuntimeShapeVersion;

        capsule.LocalOffset = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        capsule.Radius = Fixed64.FromFraction(1, 4);
        capsule.Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One);
        capsuleBody.Body.PositionTransform.Scale = new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One);
        capsuleBody.Body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        capsule.Simulate();

        capsule.RuntimeShapeVersion.Should().Be(initialVersion + 1);
        capsule.CylinderHeight.Should().Be((Fixed64)2);
        capsule.Area.Should().BeGreaterThan(Fixed64.Zero);
        capsule.ScaledRadius.Should().Be(Fixed64.Half);
        capsule.LineSegmentStart.Should().NotBe(capsule.LineSegmentEnd);
        capsule.Bounds.Contains(capsule.LineSegmentStart).Should().BeTrue();
        capsule.Bounds.Contains(capsule.LineSegmentEnd).Should().BeTrue();

        uint rebuiltVersion = capsule.RuntimeShapeVersion;

        capsule.Simulate();

        capsule.RuntimeShapeVersion.Should().Be(rebuiltVersion);
    }

    [Fact]
    public void ShortCapsule_ShouldCollapseSegmentAndUseSphereInertiaFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsuleBody = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(0, 0, 0));
        LSCapsuleCollider capsule = capsuleBody.Collider;

        capsule.Size = new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.One);

        capsule.Simulate();

        capsule.CylinderHeight.Should().Be(Fixed64.Zero);
        capsule.LineSegmentStart.Should().Be(capsule.LineSegmentEnd);

        Fixed3x3 inertia = capsule.CalculateInertiaTensor(Fixed64.One);
        inertia.M11.Should().BeGreaterThan(Fixed64.Zero);
        inertia.M22.Should().Be(inertia.M11);
        inertia.M33.Should().Be(inertia.M11);
    }

    [Fact]
    public void Initialize_WithRotatedNonUniformCuboid_ShouldRotateInverseInertiaTensor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)4)
        };

        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45));

        body.Body.InverseInertiaTensor.M13.Should().NotBe(Fixed64.Zero);
        body.Body.InverseInertiaTensor.M31.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithRotatedNonUniformCuboid_ShouldNotCompoundWorldInverseInertiaOrientation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)4)
        };
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45));
        Fixed3x3 initial = body.Body.InverseInertiaTensor;

        scenario.Context.Simulate();
        scenario.Context.Simulate();

        body.Body.InverseInertiaTensor.Should().Be(initial);
    }

    [Fact]
    public void Initialize_WithOffAxisCompoundParts_ShouldInvertFullInertiaTensor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero)));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: (Fixed64)2);
        Fixed3x3 localTensor = collider.CalculateInertiaTensor(body.Body.Mass, body.Body.LocalCenterOfMassOffset);
        Fixed3x3 product = localTensor * body.Body.InverseInertiaTensor;

        localTensor.M12.Should().NotBe(Fixed64.Zero);
        body.Body.InverseInertiaTensor.M12.Should().NotBe(Fixed64.Zero);
        AssertNear(product.M11, Fixed64.One);
        AssertNear(product.M22, Fixed64.One);
        AssertNear(product.M33, Fixed64.One);
        AssertNear(product.M12, Fixed64.Zero);
        AssertNear(product.M21, Fixed64.Zero);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
}
