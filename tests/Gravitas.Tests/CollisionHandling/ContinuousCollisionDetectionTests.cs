using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionDetectionTests
{
    private static readonly Fixed64 WallThickness = Fixed64.FromFraction(1, 10);
    private static readonly Fixed64 ExpectedSphereImpactX = -(Fixed64.Half + WallThickness * Fixed64.Half);

    [Fact]
    public void InheritMode_WithDefaultDiscreteSettings_ShouldUseDiscreteIntegration()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        DisableGroundQueries(body);
        uint raycastVersionBeforeImpulse = scenario.Context.Query3D.RaycastVersion;

        ApplyFastImpulse(body);

        body.Position3d.X.Should().Be((Fixed64)2);
        body.LinearVelocity.X.Should().Be((Fixed64)4);
        scenario.Context.Query3D.RaycastVersion.Should().Be(raycastVersionBeforeImpulse);
    }

    [Fact]
    public void InheritMode_WithContextContinuousSettings_ShouldSweepBeforeCommittingPosition()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);

        ApplyFastImpulse(body);

        body.Position3d.X.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
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

        body.Position3d.X.Should().Be((Fixed64)2);
        body.LinearVelocity.X.Should().Be((Fixed64)4);
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

        body.Position3d.X.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
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

        body.Position3d.X.Should().Be((Fixed64)2);
        body.LinearVelocity.X.Should().Be((Fixed64)4);
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

        if (shape == TestColliderShape.Sphere)
            body.Position3d.X.Should().Be(ExpectedSphereImpactX);
        else
            body.Position3d.X.Should().BeLessThanOrEqualTo(ExpectedSphereImpactX);

        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AutoMode_ShouldSweepWhenFrameDisplacementExceedsColliderProxyRadius()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        ApplyFastImpulse(body);

        body.Position3d.X.Should().Be(ExpectedSphereImpactX);
        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
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

        body.Position3d.X.Should().Be(ExpectedSphereImpactX);
    }

    [Fact]
    public void ContinuousMode_ShouldPreventFastSphereTunnelingThroughStaticMesh()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateBody(
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                MeshColliderMode.Convex,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero));
        StiffBody body = mover.Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldUseConservativeBoundsProxyForWideCuboid()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.One, Fixed64.One, (Fixed64)8)
            },
            new Vector3d((Fixed64)(-5), Fixed64.Zero, (Fixed64)3),
            FixedQuaternion.Identity);
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);

        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().BeLessThan(Fixed64.Zero);
        mover.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampThinCuboidWhenBoundsProxyHitsButSweptShapeMissesStaticSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);

        blade.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        blade.Body.Position3d.X.Should().Be((Fixed64)10);
        blade.Body.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShouldClampThinCuboidWhenSweptShapeHitsStaticSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);

        blade.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        blade.Body.Position3d.X.Should().BeLessThan((Fixed64)10);
        blade.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampCapsuleWhenBoundsProxyHitsButSweptShapeMissesStaticSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.FromFraction(5, 2)));
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.FromFraction(1, 10),
                Size = new Vector3d(Fixed64.FromFraction(1, 5), (Fixed64)6, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        capsule.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(capsule.Body);

        capsule.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        capsule.Body.Position3d.X.Should().Be((Fixed64)10);
        capsule.Body.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShapeExactTranslationalPath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);

        void SimulateShapeExactCcd()
        {
            blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            blade.Body.AddForce(Vector3d.Right * (Fixed64)10);
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateShapeExactCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldClampRotatingLongCuboidBeforeAngularTunnelingThroughStaticSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.Body.AddAngularImpulse(Vector3d.Up * (angularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));
        scenario.Context.LateSimulate();

        FixedQuaternion fullTurn = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        FixedQuaternion.Angle(blade.Body.Rotation, fullTurn).Should().BeGreaterThan(Fixed64.Zero);
        blade.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampRotatingLongCuboidForAngularNearMiss()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(5, 4)));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.Body.AddAngularImpulse(Vector3d.Up * (angularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));
        scenario.Context.LateSimulate();

        blade.Body.AngularVelocity.Y.Should().Be(angularVelocity);
    }

    [Fact]
    public void ContinuousMode_RotationalPath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);
        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        Vector3d angularImpulse = Vector3d.Up * (angularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22);

        void SimulateRotationalCcd()
        {
            blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            blade.Body.AddAngularImpulse(angularImpulse);
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateRotationalCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_ShouldPreserveTangentialVelocityAfterRemovingClosingVelocity()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        body.AddLinearImpulse(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.One));

        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        body.LinearVelocity.Z.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ContinuousMode_ShouldClampAgainstRestingDynamicBody()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(Vector3d.Zero);
        target.Collider.Size = new Vector3d(WallThickness, (Fixed64)8, (Fixed64)8);
        (StiffBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithOpposingDynamicBodies_ShouldClampBothAtSharedTimeOfImpact()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));
        left.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(left.Body);
        DisableGroundQueries(right.Body);

        left.Body.AddForce(Vector3d.Right * (Fixed64)5);
        right.Body.AddForce(-Vector3d.Right * (Fixed64)5);
        scenario.Context.LateSimulate();

        left.Body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Body.Position3d.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (right.Body.Position3d.X - left.Body.Position3d.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
        left.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        right.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldSkipMovableDynamicTargets()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));
        left.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.Body.AddForce(Vector3d.Right * (Fixed64)5);
        right.Body.AddForce(-Vector3d.Right * (Fixed64)5);
        scenario.Context.LateSimulate();

        left.Body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Body.Position3d.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldIncludeKinematicTargets()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateSphere(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero), isKinematic: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);

        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().Be((Fixed64)4);
        mover.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void PhysicsLateSimulate_DirectCalls_ShouldRefreshDynamicCcdFrame()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(new Vector3d((Fixed64)100, Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        scenario.Context.Physics.LateSimulate();
        target.Body.ResetPosition(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.Physics.LateSimulate();

        mover.Body.Position3d.X.Should().BeLessThanOrEqualTo((Fixed64)4);
        mover.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
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

        body.Position3d.X.Should().Be((Fixed64)2);
        body.LinearVelocity.X.Should().Be((Fixed64)4);
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

        body.Position3d.X.Should().Be((Fixed64)2);
        body.LinearVelocity.X.Should().Be((Fixed64)4);
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
                    CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                    CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero))),
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
