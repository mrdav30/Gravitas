using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Queries;
using SwiftCollections;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyGroundProbeModeTests
{
    [Theory]
    [InlineData(-1)]
    [InlineData(2)]
    public void GroundMinNormalDot_ShouldRejectOutOfRangeWithoutChangingPolicy(int value)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.GroundMinNormalDot.Should().Be(Fixed64.Half);
        System.Action set = () => body.GroundMinNormalDot = (Fixed64)value;
        set.Should().Throw<System.ArgumentException>();
        body.GroundMinNormalDot.Should().Be(Fixed64.Half);
        body.GroundMinNormalDot = Fixed64.One;
        body.GroundMinNormalDot.Should().Be(Fixed64.One);
    }

    [Theory]
    [InlineData(0, GroundProbeMode.Auto)]
    [InlineData(1, GroundProbeMode.Auto)]
    [InlineData(2, GroundProbeMode.Auto)]
    [InlineData(3, GroundProbeMode.Auto)]
    [InlineData(4, GroundProbeMode.Auto)]
    [InlineData(0, GroundProbeMode.SweptSphere)]
    [InlineData(1, GroundProbeMode.SweptSphere)]
    [InlineData(2, GroundProbeMode.SweptSphere)]
    [InlineData(3, GroundProbeMode.SweptSphere)]
    [InlineData(4, GroundProbeMode.SweptSphere)]
    [InlineData(5, GroundProbeMode.SweptSphere)]
    public void CheckGround_RoundAndExplicitSweptShapes_ShouldRejectTangentWalls(int shape, GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider collider = shape switch
        {
            0 => new LSSphereCollider(),
            1 => new LSCapsuleCollider(),
            2 => new LSCylinderCollider(),
            3 => new LSCuboidCollider(),
            4 => new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero)),
            _ => new LSConeCollider()
        };
        SolidBody body = scenario.CreateBody(collider, Vector3d.Zero, FixedQuaternion.Identity).Body;
        body.GroundProbeMode = mode;
        body.GroundProbeRadius = Fixed64.Half;
        body.GroundMinNormalDot = Fixed64.Zero;
        scenario.InitializeStaticCollider(new LSCuboidCollider { Size = new Vector3d(Fixed64.Eighth, (Fixed64)4, (Fixed64)4) },
            new Vector3d(Fixed64.FromFraction(9, 16), Fixed64.Zero, Fixed64.Zero));

        body.CheckGround();

        body.IsGrounded.Should().BeFalse();
        body.HasHitPoint.Should().BeFalse();
        body.Position3d.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(4)]
    public void FullStep_WhenContinuousSphereStopsAtWall_ShouldRemainAirborneAtOriginalHeight(int height)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(8);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = context.Environment.MaxFallSpeed = (Fixed64)64;
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d(-2, height, 4));
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d(Fixed64.Eighth, (Fixed64)4, (Fixed64)4) },
            new Vector3d(1, height, 4), FixedQuaternion.Identity, immovable: true);
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.Body.AddLinearImpulse(Vector3d.Right * (Fixed64)32);

        context.Simulate();
        context.LateSimulate();

        mover.Body.Position3d.X.Should().Be(Fixed64.FromFraction(7, 16));
        mover.Body.Position3d.Y.Should().Be((Fixed64)height);
        mover.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        mover.Body.IsGrounded.Should().BeFalse();
        mover.Body.HasHitPoint.Should().BeFalse();
        mover.Body.HitPlatform.Should().BeNull();
        mover.Body.GroundNormal.Should().Be(Vector3d.Zero);

        // Tangential wall hits remain valid public query geometry; only the
        // body's support policy must reject them.
        Vector3d start = mover.Body.Position3d + Vector3d.Up * Fixed64.Half;
        var hits = new SwiftList<Physics3DHit>();
        context.Query3D.SweepSphereAll(start, start + Vector3d.Down * Fixed64.Half,
            Fixed64.Half, PhysicsLayerMask.All, hits, mover.Collider).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(wall.Collider);
        hits[0].Distance.Should().Be(Fixed64.Zero);
        hits[0].Normal.Should().Be(-Vector3d.Right);
    }

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
