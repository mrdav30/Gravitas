using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderRuntimeStateTests
{
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
}
