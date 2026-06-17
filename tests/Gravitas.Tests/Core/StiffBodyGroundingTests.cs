using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class StiffBodyGroundingTests
{
    [Fact]
    public void Initialize_WithGroundBelow_ShouldProbeGroundImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void Initialize_WithoutGround_ShouldStartAirborne()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void LateSimulate_WithMovingPlatform_ShouldRefreshGroundPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        StaticCollider<LSCuboidCollider> ground = CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        ground.Transform.Position = new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(1, 4), Fixed64.Zero);
        ground.Collider.Simulate();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.FromFraction(1, 4));
        body.Body.HeightPos.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void SkipGrounding_ShouldKeepBodyAirborneDuringSkipWindow()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.SkipGrounding(Fixed64.One);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckGround_ShouldStoreSlopeNormal()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion slopeRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)15);
        CreateGround(scenario, new PhysicsLayer(1), rotation: slopeRotation);
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.GroundNormal.Should().NotBe(Vector3d.Zero);
        body.Body.GroundNormal.Should().NotBe(Vector3d.Up);
        body.Body.GroundNormal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CheckGround_ShouldHonorGroundCheckLayerMask()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeFalse();

        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(2);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeTrue();
    }

    private static StaticCollider<LSCuboidCollider> CreateGround(
        PhysicsScenarioBuilder scenario,
        PhysicsLayer layer,
        Vector3d? center = null,
        FixedQuaternion? rotation = null)
    {
        FixedTransform transform = new(
            center ?? new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            rotation ?? FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var collider = new LSCuboidCollider
        {
            Layer = layer,
            Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };

        collider.InitializeWithNoBody(agent);
        return new StaticCollider<LSCuboidCollider>(collider, transform);
    }

    private readonly record struct StaticCollider<TCollider>(TCollider Collider, FixedTransform Transform)
        where TCollider : LSCollider;
}
