using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionDetectionTests
{
    private static readonly Fixed64 WallThickness = Fixed64.Fraction(1, 10);
    private static readonly Fixed64 ExpectedSphereImpactX = -(Fixed64.Half + WallThickness * Fixed64.Half);

    [Fact]
    public void InheritMode_WithDefaultDiscreteSettings_ShouldUseDiscreteIntegration()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        DisableGroundQueries(body);
        uint raycastVersionBeforeImpulse = scenario.Context.Raycasts.Version;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
        scenario.Context.Raycasts.Version.Should().Be(raycastVersionBeforeImpulse);
    }

    [Fact]
    public void InheritMode_WithContextContinuousSettings_ShouldSweepBeforeCommittingPosition()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.x.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DiscreteMode_ShouldOverrideContextContinuousSettings()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
    }

    [Fact]
    public void InheritMode_ShouldUseCachedTopParentBeforeContextDefault()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        ScenarioBody<LSSphereCollider> topParent = scenario.CreateSphere(new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> middleParent = scenario.CreateSphere(new Vector3d((Fixed64)(-7), Fixed64.Zero, Fixed64.Zero));
        (StiffBody body, LSCollider collider) = CreateMover(scenario, TestColliderShape.Sphere);
        topParent.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middleParent.Collider.SetParent(topParent.Collider);
        collider.SetParent(middleParent.Collider);

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.x.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void InheritMode_ShouldUseParentExplicitDiscreteBeforeContextContinuousDefault()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        CreateStaticWall(scenario, Fixed64.Zero);
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero));
        (StiffBody body, LSCollider collider) = CreateMover(scenario, TestColliderShape.Sphere);
        parent.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        collider.SetParent(parent.Collider);

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
    }

    [Theory]
    [InlineData(TestColliderShape.Sphere)]
    [InlineData(TestColliderShape.Capsule)]
    [InlineData(TestColliderShape.Cuboid)]
    [InlineData(TestColliderShape.Cylinder)]
    [InlineData(TestColliderShape.Compound)]
    public void ContinuousMode_ShouldPreventFastColliderTunnelingThroughThinStaticGeometry(TestColliderShape shape)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, shape);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.x.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AutoMode_ShouldSweepWhenFrameDisplacementExceedsColliderProxyRadius()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.x.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithMultipleTargets_ShouldUseEarliestTimeOfImpact()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        CreateStaticWall(scenario, Fixed64.One);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be(ExpectedSphereImpactX);
    }

    [Fact]
    public void ContinuousMode_ShouldPreserveTangentialVelocityAfterRemovingClosingVelocity()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        body.AddLinearImpulse(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.One));

        body.LinearVelocity.x.Should().Be(Fixed64.Zero);
        body.LinearVelocity.z.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ContinuousMode_ShouldNotRunDynamicVsDynamicCcdDuringAlpha()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(Vector3d.Zero);
        target.Collider.Size = new Vector3d(WallThickness, (Fixed64)8, (Fixed64)8);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_ShouldRespectContextCollisionMatrix()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var matrix = new[,]
        {
            { true, false },
            { false, true }
        };
        scenario.Context.ApplySettings(new PhysicsSettings(1, matrix, PhysicsLayerMask.None));
        LSCuboidCollider wall = CreateStaticWall(scenario, Fixed64.Zero);
        wall.Layer = new PhysicsLayer(1);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampAgainstTriggers()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSCuboidCollider trigger = CreateStaticWall(scenario, Fixed64.Zero);
        PhysicsScenarioBuilder.SetTrigger(trigger);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.x.Should().Be((Fixed64)2);
        body.LinearVelocity.x.Should().Be((Fixed64)4);
    }

    private static PhysicsScenarioBuilder CreateCcdScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(1);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)16;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)16;
        return scenario;
    }

    private static LSCuboidCollider CreateStaticWall(PhysicsScenarioBuilder scenario, Fixed64 x)
    {
        var wall = new LSCuboidCollider
        {
            Size = new Vector3d(WallThickness, (Fixed64)8, (Fixed64)8)
        };

        scenario.InitializeStaticCollider(wall, new Vector3d(x, Fixed64.Zero, Fixed64.Zero));
        return wall;
    }

    private static (StiffBody Body, LSCollider Collider) CreateMover(PhysicsScenarioBuilder scenario, TestColliderShape shape)
    {
        return shape switch
        {
            TestColliderShape.Sphere => ToTuple(scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Capsule => ToTuple(scenario.CreateCapsule(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Cuboid => ToTuple(scenario.CreateCuboid(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Cylinder => ToTuple(scenario.CreateCylinder(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Compound => ToTuple(scenario.CreateBody(
                new LSCompoundCollider(
                    new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero) }),
                    new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero) })),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity)),
            _ => throw new System.ArgumentOutOfRangeException(nameof(shape), shape, null)
        };
    }

    private static (StiffBody Body, LSCollider Collider) ToTuple<TCollider>(ScenarioBody<TCollider> scenarioBody)
        where TCollider : LSCollider =>
        (scenarioBody.Body, scenarioBody.Collider);

    private static void ApplyFastImpulse(StiffBody body) =>
        body.AddLinearImpulse(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

    private static void DisableGroundQueries(StiffBody body)
    {
        body.GroundedDistanceRay = Fixed64.Zero;
        body.GroundDownDistanceOnAir = Fixed64.Zero;
    }

    public enum TestColliderShape
    {
        Sphere,
        Capsule,
        Cuboid,
        Cylinder,
        Compound
    }
}
