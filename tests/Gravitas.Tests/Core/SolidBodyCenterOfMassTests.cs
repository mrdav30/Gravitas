using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
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

        body.Body.LocalCenterOfMassOffset.Should().Be(properties.CenterOfMass);
        body.Body.WorldCenterOfMass.Should().Be(properties.CenterOfMass);
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
        sphere.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, Fixed64.Zero));
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
