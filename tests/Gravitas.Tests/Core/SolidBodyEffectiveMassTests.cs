using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyEffectiveMassTests
{
    [Fact]
    public void EffectiveMass_ForMovableBody_ShouldExposeRawInverseMassAndInertia()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            mass: (Fixed64)4);

        body.Body.CanTranslate.Should().BeTrue();
        body.Body.CanRotate.Should().BeTrue();
        body.Body.EffectiveInverseMass.Should().Be(Fixed64.FromFraction(1, 4));
        body.Body.EffectiveInverseInertiaTensor.Should().Be(body.Body.InverseInertiaTensor);
        body.Body.EffectiveInverseInertiaTensor.Should().NotBe(Fixed3x3.Zero);
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void EffectiveMass_ForKinematicOrPositionFrozenBody_ShouldBehaveAsInfiniteMass(
        bool positionFrozen,
        bool isKinematic)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            immovable: positionFrozen,
            isKinematic: isKinematic);

        body.Body.InverseMass.Should().Be(Fixed64.One);
        body.Body.CanTranslate.Should().BeFalse();
        body.Body.CanRotate.Should().BeFalse();
        body.Body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.Body.EffectiveInverseInertiaTensor.Should().Be(Fixed3x3.Zero);
    }

    [Fact]
    public void EffectiveMass_ForNonDynamicBody_ShouldBehaveAsInfiniteMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateBody(
            new LSSphereCollider(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: (Fixed64)4,
            isDynamic: false);

        body.Body.DynamicId.Should().Be(-1);
        body.Body.InverseMass.Should().Be(Fixed64.FromFraction(1, 4));
        body.Body.CanTranslate.Should().BeFalse();
        body.Body.CanRotate.Should().BeFalse();
        body.Body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.Body.EffectiveInverseInertiaTensor.Should().Be(Fixed3x3.Zero);
    }

    [Fact]
    public void EffectiveMass_WithAngularForcesDisabled_ShouldKeepLinearMassAndDisableRotation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(
            Vector3d.Zero,
            preventAngularForces: true);

        body.Body.CanTranslate.Should().BeTrue();
        body.Body.CanRotate.Should().BeFalse();
        body.Body.EffectiveInverseMass.Should().Be(body.Body.InverseMass);
        body.Body.EffectiveInverseInertiaTensor.Should().Be(Fixed3x3.Zero);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void EffectiveMass_WithNonPositiveMass_ShouldDisableSolverMotion(int mass)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            mass: (Fixed64)mass);

        body.Body.CanTranslate.Should().BeFalse();
        body.Body.CanRotate.Should().BeFalse();
        body.Body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.Body.EffectiveInverseInertiaTensor.Should().Be(Fixed3x3.Zero);
    }
}
