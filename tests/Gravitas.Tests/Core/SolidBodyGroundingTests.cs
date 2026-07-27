using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyGroundingTests
{
    [Fact]
    public void Initialize_WithGroundBelow_ShouldProbeGroundImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.WasGrounded.Should().BeFalse();
        body.Body.HasHitPoint.Should().BeTrue();
        body.Body.TryGetHitPoint(out Vector3d hitPoint).Should().BeTrue();
        hitPoint.Y.Should().Be(Fixed64.Zero);
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void Initialize_WithoutGround_ShouldStartAirborne()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeFalse();
        body.Body.HasHitPoint.Should().BeFalse();
        body.Body.TryGetHitPoint(out Vector3d hitPoint).Should().BeFalse();
        hitPoint.Should().Be(Vector3d.Zero);
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void LateSimulate_WhenAlreadyGrounded_ShouldExposeWasGrounded()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void CheckGround_WhenGroundNoLongerMatchesMask_ShouldPreservePreviousGroundedState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(2);
        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithMovingPlatform_ShouldRefreshGroundPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        StaticCollider<LSCuboidCollider> ground = CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        ground.Transform.LocalPosition = new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(1, 4), Fixed64.Zero);
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
        scenario.Context.SetFrameRate(1);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.SkipGrounding(Fixed64.One);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeFalse();
        scenario.Context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        for (int i = 0; i < 8; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.WasGrounded.Should().BeFalse();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UseManualGrounding_ShouldClearGroundStateAndSkipAutomaticProbe()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.UseManualGrounding();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void UseManualGrounding_ShouldDisableAutomaticProbeAndLeaveBodyAirborne()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.UseManualGrounding();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SetManualGrounding_ShouldPreserveHostGroundStateDuringSimulation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        Vector3d heightmapPoint = new(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.Zero);

        body.Body.SetManualGrounding(heightmapPoint, Vector3d.Up);

        body.Body.WasGrounded.Should().BeTrue();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.Body.IsGrounded.Should().BeTrue();
        body.Body.WasGrounded.Should().BeTrue();
        body.Body.HitPoint.Should().Be(heightmapPoint);
        body.Body.GroundNormal.Should().Be(Vector3d.Up);
        body.Body.HeightPos.Should().Be(heightmapPoint.Y);
    }

    [Fact]
    public void ClearManualGrounding_ShouldKeepAutomaticProbeDisabled()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.SetManualGrounding(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero), Vector3d.Up);
        body.Body.ClearManualGrounding();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
        body.Body.HitPoint.Should().Be(Vector3d.Zero);
        body.Body.GroundNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void UseAutomaticGrounding_ShouldResumeProbeOwnedGrounding()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateGround(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.SetManualGrounding(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero), Vector3d.Up);
        body.Body.UseAutomaticGrounding();

        body.Body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.Body.IsGrounded.Should().BeTrue();
        body.Body.HitPoint.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void GroundingModeTransitions_ShouldRespectNoClearAndNoImmediateRefreshOptions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        var platform = new FixedTransform(Vector3d.Down, FixedQuaternion.Identity, Vector3d.One);
        Vector3d manualPoint = new(Fixed64.Zero, Fixed64.Half, Fixed64.Zero);
        int groundedChanges = 0;
        body.Body.OnGrounded += _ => groundedChanges++;

        body.Body.SetManualGrounding(manualPoint, Vector3d.Up, platform);
        body.Body.UseManualGrounding(clearGrounding: false);
        body.Body.AddForce(Vector3d.Right);
        scenario.Context.LateSimulate();
        body.Body.UseAutomaticGrounding(checkGroundImmediately: false);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.Body.HitPlatform.Should().BeSameAs(platform);
        body.Body.HitPoint.Should().Be(manualPoint);
        body.Body.LastGroundedPosition.Should().Be(Vector3d.Zero);
        body.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        groundedChanges.Should().Be(1);
    }

    [Fact]
    public void UseAutomaticGrounding_WhenInactive_ShouldOnlyChangeOwnershipMode()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.UseManualGrounding();
        body.Body.Deactivate();

        body.Body.UseAutomaticGrounding();

        body.Body.Active.Should().BeFalse();
        body.Body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.Body.IsGrounded.Should().BeFalse();
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

    [Fact]
    public void AutomaticGrounding_WithUnmaterializableSurfaceWitness_ShouldPreserveGroundingWithoutChangingHeight()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 x = Fixed64.MaxValue - Fixed64.FromFraction(1, 16);
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)8,
                    (Fixed64)(-8),
                    (Fixed64)(-8)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)8,
                    (Fixed64)8)),
            out _).Should().BeTrue();
        context.Settings.GroundCheckLayerMask =
            PhysicsLayerMask.FromLayer(new PhysicsLayer(1));
        context.Environment.Gravity = Fixed64.Zero;
        var ground = new LSCuboidCollider
        {
            Layer = new PhysicsLayer(1),
            Size = new Vector3d(Fixed64.One, Fixed64.One, (Fixed64)4)
        };
        ground.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(x, -Fixed64.Half, Fixed64.Zero),
                FixedQuaternion.FromEulerAnglesInDegrees(
                    Fixed64.Zero,
                    Fixed64.Zero,
                    (Fixed64)15),
                Vector3d.One)));
        var body = new SolidBody(
            new TestMatterAgent(context),
            new LSSphereCollider())
        {
            Mass = Fixed64.One,
            GroundProbeMode = GroundProbeMode.SweptSphere,
            GroundProbeRadius = Fixed64.Half,
            GroundOriginOffset = Fixed64.Half,
            GroundedDistanceRay = Fixed64.One,
            GroundDownDistanceOnAir = Fixed64.One
        };

        body.Initialize(
            new Vector3d(x, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        body.IsGrounded.Should().BeTrue();
        body.HasHitPoint.Should().BeFalse();
        body.TryGetHitPoint(out Vector3d hitPoint).Should().BeFalse();
        hitPoint.Should().Be(Vector3d.Zero);
        Action readPoint = () => _ = body.HitPoint;
        readPoint.Should().Throw<InvalidOperationException>()
            .WithMessage("*TryGetHitPoint*");

        context.Simulate();
        context.LateSimulate();

        body.IsGrounded.Should().BeTrue();
        body.HeightPos.Should().Be(Fixed64.Zero);
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
