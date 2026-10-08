using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupImpulseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FrozenImpact_WithNoMobility_ShouldProduceNoImpulse(bool restitution)
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var second = scenario.CreateCuboid(Vector3d.Right, immovable: true);
        var impact = new ContactResponseSnapshot(first.Body, second.Body,
            Vector3d.Zero, Vector3d.Zero, Vector3d.Left, Vector3d.Zero);
        Solve(null, Vector3d.Left, Vector3d.Zero, Vector3d.Zero,
            restitution ? Fixed64.One : Fixed64.Zero, Fixed64.Zero, impact, out var result)
            .Should().BeTrue();
        result.ImpulseScalar.Should().Be(Fixed64.Zero);
        result.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FrozenImpact_WithUnrepresentablePointSpeed_ShouldRequestExactFallback(bool incoming)
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var second = scenario.CreateCuboid(Vector3d.Right);
        var impact = new ContactResponseSnapshot(first.Body, second.Body,
            Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, incoming ? Vector3d.Forward * Fixed64.MaxValue : Vector3d.Zero);
        Solve(second.Body, Vector3d.Zero, incoming ? Vector3d.Zero : Vector3d.Forward * Fixed64.MaxValue, Vector3d.Up * Fixed64.Two,
            Fixed64.One, Fixed64.Zero, impact, out _).Should().BeFalse();
    }

    [Fact]
    public void FrozenImpact_WithUnrepresentableEffectiveMass_ShouldRequestExactFallback()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var second = scenario.CreateCuboid(Vector3d.Right);
        var impact = new ContactResponseSnapshot(first.Body, second.Body,
            Vector3d.Zero, Vector3d.Zero, Vector3d.Left, Vector3d.Zero);
        Solve(second.Body, Vector3d.Left, Vector3d.Zero, Vector3d.Up * Fixed64.MaxValue,
            Fixed64.One, Fixed64.Zero, impact, out _).Should().BeFalse();
    }

    [Fact]
    public void FrozenImpact_WithOverflowingPositiveAccumulator_ShouldRequestExactFallback()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var second = scenario.CreateCuboid(Vector3d.Right, preventAngularForces: true);
        var impact = new ContactResponseSnapshot(first.Body, second.Body,
            Vector3d.Zero, Vector3d.Zero, Vector3d.Left, Vector3d.Zero);
        Solve(second.Body, Vector3d.Left, Vector3d.Zero, Vector3d.Zero,
            Fixed64.One, Fixed64.MaxValue, impact, out _).Should().BeFalse();
    }

    [Fact]
    public void FrozenImpact_WithUnrepresentableNegativeDelta_ShouldUnwindOnlyCachedLoad()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var second = scenario.CreateCuboid(Vector3d.Right, mass: Fixed64.MaxValue,
            preventAngularForces: true);
        var impact = new ContactResponseSnapshot(first.Body, second.Body,
            Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Solve(second.Body, Vector3d.Right * Fixed64.MaxValue, Vector3d.Zero, Vector3d.Zero,
            Fixed64.One, Fixed64.One, impact, out var result).Should().BeTrue();
        result.ImpulseScalar.Should().Be(-Fixed64.One);
        result.LinearVelocityDeltaB.X.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void CompactFriction_WithFrozenTangentAxis_ShouldSkipUndefinedFriction()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var driver = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        var target = scenario.CreateSphere(Vector3d.Right);
        target.Body.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        driver.Body.Agent.Transform.LocalPosition = new Vector3d(Fixed64.FromFraction(1, 10),
            Fixed64.Zero, Fixed64.FromFraction(1, 20));
        scenario.Context.AdvanceLateSimulateToken();
        driver.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        CollisionPair pair = scenario.CreatePair(driver.Collider, target.Collider);
        pair.Manifold.SetContact(pair.ColliderA.Center, pair.ColliderB.Center, Fixed64.Zero,
            pair.ColliderA == driver.Collider ? Vector3d.Right : Vector3d.Left);

        CollisionResponse.CalculateImpulse(pair);

        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
    }

    private static bool Solve(SolidBody? body, Vector3d linear, Vector3d angular,
        Vector3d lever, Fixed64 restitution, Fixed64 cached, in ContactResponseSnapshot impact,
        out ContactNormalImpulseResult3D result) =>
        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaWithImpact(null, Vector3d.Zero,
            Vector3d.Zero, Vector3d.Zero, body, linear, angular, lever, Vector3d.Right,
            restitution, Fixed64.Zero, cached, impact, out result);
}
