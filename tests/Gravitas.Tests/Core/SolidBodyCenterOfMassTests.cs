using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyCenterOfMassTests
{
    [Fact]
    public void Initialize_WithOffsetPrimitiveCollider_ShouldUseColliderCenterAsDefaultCenterOfMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSSphereCollider
        {
            LocalOffset = new Vector3d(Fixed64.Half, Fixed64.FromFraction(1, 4), -Fixed64.Half)
        };

        ScenarioBody<LSSphereCollider> body = scenario.CreateBody(
            collider,
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-3)),
            FixedQuaternion.Identity);

        body.Body.LocalCenterOfMassOffset.Should().Be(collider.ScaledOffset);
        body.Body.WorldCenterOfMass.Should().Be(body.Body.Position3d + collider.ScaledOffset);
    }

    [Fact]
    public void WorldCenterOfMass_ShouldRotateBodyLocalOffsetWithBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.LocalCenterOfMassOffset = Vector3d.Right;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.Body.SetRotation(rotation);

        body.Body.WorldCenterOfMass.Should().Be(body.Body.Position3d + (rotation * Vector3d.Right));
    }

    [Fact]
    public void WorldCenterOfMass_WhenAbsolutePointIsUnrepresentable_ShouldExposeTryContract()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        body.Body.LocalCenterOfMassOffset = Vector3d.Right;

        body.Body.TryGetWorldCenterOfMass(out Vector3d center).Should().BeFalse();
        center.Should().Be(Vector3d.Zero);

        Func<Vector3d> readWorldCenter = () => body.Body.WorldCenterOfMass;
        readWorldCenter.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void WorldCenterOfMass_WhenRotatedOffsetIsUnrepresentable_ShouldExposeTryContract()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.LocalCenterOfMassOffset = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.MaxValue);
        body.Body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero));

        body.Body.TryGetWorldCenterOfMass(out Vector3d center).Should().BeFalse();
        center.Should().Be(Vector3d.Zero);
        body.Body.TryGetOffsetFromCenterOfMass(
            new ContactAnchor(Vector3d.Zero, Vector3d.Zero),
            out Vector3d offset).Should().BeFalse();
        offset.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CenterOfMassOperations_WhenRotationOverflowsButFinalValuesCancel_ShouldSucceed()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)45,
                Fixed64.Zero);
        Vector3d localOffset = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.MaxValue);
        rotation.TryRotate(localOffset, out _).Should().BeFalse();

        ScenarioBody<LSSphereCollider> worldBody = scenario.CreateSphere(
            new Vector3d(
                -Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero));
        worldBody.Body.LocalCenterOfMassOffset = localOffset;
        worldBody.Body.SetRotation(rotation);
        worldBody.Body.TryGetWorldCenterOfMass(
            out Vector3d worldCenter).Should().BeTrue();
        worldCenter.X.Should().BeGreaterThan(Fixed64.Zero);
        worldCenter.X.Should().BeLessThan(Fixed64.MaxValue);

        ScenarioBody<LSSphereCollider> relativeBody =
            scenario.CreateSphere(Vector3d.Zero);
        relativeBody.Body.LocalCenterOfMassOffset = localOffset;
        relativeBody.Body.SetRotation(rotation);
        relativeBody.Body.TryGetOffsetFromCenterOfMass(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero)),
            out Vector3d relative).Should().BeTrue();
        relative.X.Should().BeGreaterThan(Fixed64.Zero);
        relative.X.Should().BeLessThan(Fixed64.MaxValue);
    }

    [Fact]
    public void TryGetOffsetFromCenterOfMass_WhenAbsolutePointsAreUnrepresentable_ShouldRetainExactLeverArm()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.LocalCenterOfMassOffset = Vector3d.Right;
        var anchor = new ContactAnchor(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);

        body.Body.TryGetOffsetFromCenterOfMass(anchor, out Vector3d offset).Should().BeTrue();
        offset.Should().Be(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void ResetCenterOfMassFromCollider_ShouldReturnExplicitOverrideToDerivedColliderCenter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSSphereCollider
        {
            LocalOffset = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)
        };
        ScenarioBody<LSSphereCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        body.Body.LocalCenterOfMassOffset = Vector3d.Up;

        body.Body.ResetCenterOfMassFromCollider();

        body.Body.LocalCenterOfMassOffset.Should().Be(collider.ScaledOffset);
    }

    [Fact]
    public void LocalCenterOfMassOffset_ShouldNoOpForSameExplicitValueAndAllowInactiveSetup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var inactive = new SolidBody(new TestMatterAgent(scenario.Context), new LSSphereCollider());

        inactive.LocalCenterOfMassOffset = Vector3d.Up;
        inactive.LocalCenterOfMassOffset.Should().Be(Vector3d.Up);

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.LocalCenterOfMassOffset = Vector3d.Right;
        body.Body.Sleep();

        body.Body.LocalCenterOfMassOffset = Vector3d.Right;

        body.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void Initialize_WithClosedMeshCollider_ShouldUseMeshCenterOfMassAsDefaultBodyCenterOfMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = CreateTetrahedronCollider();
        collider.Mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _).Should().BeTrue();

        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Vector3d expected = collider.ScaledOffset + properties.CenterOfMass;
        body.Body.LocalCenterOfMassOffset.Should().Be(expected);
        body.Body.WorldCenterOfMass.Should().Be(expected);
    }

    [Fact]
    public void CalculateImpulse_WithCenteredContactAwayFromCenterOfMass_ShouldApplyAngularImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> shifted = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        shifted.Body.LocalCenterOfMassOffset = Vector3d.Up * Fixed64.FromFraction(1, 4);
        sphere.Body.AddLinearImpulse(Vector3d.Left * Fixed64.FromFraction(15, 8) * sphere.Body.Mass);
        CollisionPair pair = scenario.CreatePair(shifted.Collider, sphere.Collider);
        pair.Manifold.SetContact(shifted.Collider.Center, sphere.Collider.Center, Fixed64.FromFraction(1, 10), Vector3d.Right);

        CollisionResponse.CalculateImpulse(pair);

        shifted.Body.AngularVelocity.Should().NotBe(Vector3d.Zero);
    }

    private static LSMeshCollider CreateTetrahedronCollider() =>
        new(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            });
}
