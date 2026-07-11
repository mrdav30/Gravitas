using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyGroundProbeModeTests
{
    [Fact]
    public void CheckGround_ShouldUseSelectedProbeMode()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(
            scenario,
            new PhysicsLayer(1),
            center: new Vector3d((Fixed64)1.25f, -Fixed64.Half, Fixed64.Zero),
            size: new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.GroundProbeMode = GroundProbeMode.Ray;
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();

        body.Body.GroundProbeMode = GroundProbeMode.SweptSphere;
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
        body.Body.GroundNormal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CheckGround_AutoMode_ShouldUseSweptSphereForRoundBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(
            scenario,
            new PhysicsLayer(1),
            center: new Vector3d((Fixed64)1.25f, -Fixed64.Half, Fixed64.Zero),
            size: new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.GroundProbeMode.Should().Be(GroundProbeMode.Auto);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CheckGround_AutoMode_ShouldUseSweptSphereForCompoundBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(
            scenario,
            new PhysicsLayer(1),
            center: new Vector3d((Fixed64)1.25f, -Fixed64.Half, Fixed64.Zero),
            size: new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Body.GroundProbeMode.Should().Be(GroundProbeMode.Auto);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CheckGround_AutoMode_ShouldUseRayForSubThresholdCompoundRadius()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(
            scenario,
            new PhysicsLayer(1),
            center: new Vector3d((Fixed64)1.25f, -Fixed64.Half, Fixed64.Zero),
            size: new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.FromFraction(1, 16), Vector3d.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Body.GroundProbeMode.Should().Be(GroundProbeMode.Auto);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void CheckGround_ConeShouldUseAutoRayAndHonorExplicitSweptSphereRadius()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(
            scenario,
            new PhysicsLayer(1),
            center: new Vector3d((Fixed64)1.25f, -Fixed64.Half, Fixed64.Zero),
            size: new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSConeCollider> body = scenario.CreateCone(Vector3d.Zero);

        body.Body.GroundProbeMode.Should().Be(GroundProbeMode.Auto);
        body.Body.CheckGround();
        body.Body.IsGrounded.Should().BeFalse();

        body.Body.GroundProbeMode = GroundProbeMode.SweptSphere;
        body.Body.GroundProbeRadius = Fixed64.Half;
        body.Body.CheckGround();
        body.Body.IsGrounded.Should().BeTrue();

        body.Body.GroundProbeRadius = Fixed64.Zero;
        body.Body.CheckGround();
        body.Body.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void CheckGround_SweptSphereMode_ShouldHonorLayerMaskAndSelfExclusion()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(2));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GroundProbeMode = GroundProbeMode.SweptSphere;

        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();

        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(2);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CheckGround_SweptSphereMode_ShouldIgnoreMovableDynamicBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(0);
        scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GroundProbeMode = GroundProbeMode.SweptSphere;

        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();
    }

    private static StaticCollider<LSCuboidCollider> CreateGround(
        PhysicsScenarioBuilder scenario,
        PhysicsLayer layer,
        Vector3d? center = null,
        Vector3d? size = null)
    {
        FixedTransform transform = new(
            center ?? new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var collider = new LSCuboidCollider
        {
            Layer = layer,
            Size = size ?? new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };

        collider.InitializeWithNoBody(agent);
        return new StaticCollider<LSCuboidCollider>(collider, transform);
    }

    private readonly record struct StaticCollider<TCollider>(TCollider Collider, FixedTransform Transform)
        where TCollider : LSCollider;
}
