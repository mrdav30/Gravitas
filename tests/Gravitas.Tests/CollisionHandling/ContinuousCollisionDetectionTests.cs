using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);

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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        (SolidBody body, LSCollider collider) = CreateMover(scenario, TestColliderShape.Sphere);
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
        (SolidBody body, LSCollider collider) = CreateMover(scenario, TestColliderShape.Sphere);
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
    [InlineData(TestColliderShape.Cone)]
    [InlineData(TestColliderShape.ConvexMesh)]
    [InlineData(TestColliderShape.Compound)]
    public void ContinuousMode_ShouldPreventFastColliderTunnelingThroughThinStaticGeometry(TestColliderShape shape)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (SolidBody body, _) = CreateMover(scenario, shape);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        if (shape == TestColliderShape.Sphere)
            body.Position3d.X.Should().Be(ExpectedSphereImpactX);
        else
            body.Position3d.X.Should().BeLessThanOrEqualTo(ExpectedSphereImpactX);

        body.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldConsumeRemainingFrameTimeAfterSlidingIntoSecondStaticContact()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        LSCuboidCollider horizontalWall = new()
        {
            Size = new Vector3d((Fixed64)8, (Fixed64)8, WallThickness)
        };
        scenario.InitializeStaticCollider(horizontalWall, new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.Body.SleepEnabled = false;
        DisableGroundQueries(mover.Body);

        mover.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4));
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().BeLessThanOrEqualTo(ExpectedSphereImpactX);
        mover.Body.Position3d.Z.Should().BeGreaterThanOrEqualTo(Fixed64.FromFraction(49, 20));
        mover.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_WithToiIterationLimit_ShouldExposeDeterministicLimitState()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        CreateStaticWall(scenario, Fixed64.Zero);
        LSCuboidCollider horizontalWall = new()
        {
            Size = new Vector3d((Fixed64)8, (Fixed64)8, WallThickness)
        };
        scenario.InitializeStaticCollider(horizontalWall, new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);

        mover.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4));
        scenario.Context.LateSimulate();

        mover.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        mover.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        mover.Body.Position3d.Z.Should().BeLessThan(Fixed64.FromFraction(49, 20));
        mover.Body.LinearVelocity.Z.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_ToiIterationPath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        LSCuboidCollider horizontalWall = new()
        {
            Size = new Vector3d((Fixed64)8, (Fixed64)8, WallThickness)
        };
        scenario.InitializeStaticCollider(horizontalWall, new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);

        void SimulateToiIterationCcd()
        {
            mover.Body.ResetPosition(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            mover.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4));
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateToiIterationCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void AutoMode_ShouldSweepWhenFrameDisplacementExceedsColliderProxyRadius()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        CreateStaticWall(scenario, Fixed64.Zero);
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        SolidBody body = mover.Body;
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
        blade.Body.SleepEnabled = false;
        DisableGroundQueries(blade.Body);

        blade.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        blade.Body.Position3d.X.Should().Be((Fixed64)10);
        blade.Body.LinearVelocity.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampThinCuboidWhenBoundsProxyHitsButSweptShapeMissesStaticCuboid()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var target = new LSCuboidCollider { Size = Vector3d.One };
        scenario.InitializeStaticCollider(target, new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.SleepEnabled = false;
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
        blade.Body.SleepEnabled = false;
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
            scenario.Context.Simulate();
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
    public void ContinuousMode_ShouldRefineRotatingLongCuboidAngularToiBeyondPreviousSample()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(59, 20), Fixed64.Zero, Fixed64.FromFraction(-3, 4)));
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

        FixedQuaternion clampedAtZero = FixedQuaternion.Identity;
        FixedQuaternion.Angle(blade.Body.Rotation, clampedAtZero).Should().BeGreaterThan((Fixed64)1);
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
    public void AutoMode_ShouldSkipSmallRotationalArc()
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
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        DisableGroundQueries(blade.Body);

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)5);
        blade.Body.AddAngularImpulse(Vector3d.Up * (angularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));
        scenario.Context.LateSimulate();

        FixedQuaternion.Angle(blade.Body.Rotation, FixedQuaternion.Identity).Should().BeGreaterThan(Fixed64.Zero);
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
            scenario.Context.Simulate();
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        ApplyFastImpulse(body);

        body.Position3d.X.Should().BeLessThan(Fixed64.Half);
        body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.IsSleeping.Should().BeFalse();
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
        left.Body.LinearVelocity.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        right.Body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_ShouldIgnoreSiblingTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Collider.SetParent(source.Collider);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);

        source.Body.AddForce(Vector3d.Right * (Fixed64)4);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().Be(Fixed64.One);
        source.Body.LinearVelocity.X.Should().Be((Fixed64)4);
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_WithMatchingVelocity_ShouldNotClampSource()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);

        source.Body.AddForce(Vector3d.Right * (Fixed64)5);
        target.Body.AddForce(Vector3d.Right * (Fixed64)5);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)2);
        target.Body.Position3d.X.Should().Be((Fixed64)5);
        source.Body.LinearVelocity.X.Should().Be((Fixed64)5);
        target.Body.LinearVelocity.X.Should().Be((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_WithConfiguredRestitutionThreshold_ShouldSuppressDynamicBounce()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.RestitutionVelocityThreshold = (Fixed64)5;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);

        source.Body.AddForce(Vector3d.Right * (Fixed64)4);
        scenario.Context.LateSimulate();

        source.Body.LinearVelocity.X.Should().Be((Fixed64)2);
        target.Body.LinearVelocity.X.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ContinuousMode_WithZeroRestitutionThreshold_ShouldBounceLowSpeedDynamicContact()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);

        source.Body.AddForce(Vector3d.Right * (Fixed64)4);
        scenario.Context.LateSimulate();

        source.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        target.Body.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_WithChainedDynamicBodies_ShouldWakeAndContinueConnectedIsland()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> receiver = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        driver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(driver.Body);
        DisableGroundQueries(middle.Body);
        DisableGroundQueries(receiver.Body);
        middle.Body.Sleep();
        receiver.Body.Sleep();

        driver.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        middle.Body.IsSleeping.Should().BeFalse();
        receiver.Body.IsSleeping.Should().BeFalse();
        receiver.Body.Position3d.X.Should().BeGreaterThan((Fixed64)2);
        middle.Body.Position3d.X.Should().BeLessThanOrEqualTo(receiver.Body.Position3d.X - Fixed64.One);
        driver.Body.Position3d.X.Should().BeLessThanOrEqualTo(middle.Body.Position3d.X - Fixed64.One);
        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(2);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithChainedDynamicBodiesAndQueueLimit_ShouldExposeServiceLimitState()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> receiver = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        driver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(driver.Body);
        DisableGroundQueries(middle.Body);
        DisableGroundQueries(receiver.Body);
        middle.Body.Sleep();
        receiver.Body.Sleep();

        driver.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        scenario.Context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
    }

    [Fact]
    public void ContinuousHandoff_WithFrozenBody_ShouldNotMoveOrQueue()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.Position;

        body.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right,
            Vector3d.Right,
            Fixed64.Half);

        body.Body.Position3d.Should().Be(Vector3d.Zero);
        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.Body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WithNoRemainingMotion_ShouldApplyImmediateStateWithoutQueueing()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> noTime = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> noVelocity = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);

        noTime.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right,
            Fixed64.Zero);
        noVelocity.Body.ApplyContinuousCollisionHandoff(
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Half),
            Vector3d.Zero,
            Fixed64.Half);

        noTime.Body.Position3d.Should().Be(Vector3d.Right * Fixed64.Half);
        noTime.Body.LinearVelocity.Should().Be(Vector3d.Right);
        noTime.Body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
        noVelocity.Body.Position3d.Should().Be(new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Half));
        noVelocity.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        noVelocity.Body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeFalse();
    }

    [Fact]
    public void ContinuousHandoff_WithQueuedMotion_ShouldHonorDirectConsumeFlags()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        DisableGroundQueries(body.Body);
        int movedCount = 0;
        body.Body.OnMoved += () => movedCount++;

        body.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right,
            Fixed64.Half);

        body.Body.TryConsumeContinuousCollisionHandoff(
            updateSleepState: false,
            updateColliderState: false).Should().BeTrue();
        body.Body.Position3d.X.Should().Be(Fixed64.One);
        body.Body.LinearVelocity.Should().Be(Vector3d.Right);
        movedCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousHandoff_WithQueuedMotion_ShouldConsumeThroughDirectLateSimulate()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        DisableGroundQueries(body.Body);

        body.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right,
            Fixed64.Half);
        body.Body.LateSimulate();

        body.Body.Position3d.X.Should().Be(Fixed64.One);
        body.Body.LinearVelocity.Should().Be(Vector3d.Right);
        body.Collider.Center.X.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_ShouldNotClampThinCuboidWhenProxySpheresHitButShapesMiss()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        target.Body.Sleep();
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(target.Body);

        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().Be((Fixed64)10);
        mover.Body.LinearVelocity.X.Should().Be((Fixed64)10);
        target.Body.Position3d.Should().Be(new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
    }

    [Fact]
    public void ContinuousMode_DynamicRelativePath_ShouldClampThinCuboidAgainstDynamicCuboid()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = Vector3d.One
            },
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        target.Body.Sleep();
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(target.Body);

        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Collider.BoundsMax.X.Should().BeLessThanOrEqualTo(target.Collider.BoundsMin.X);
        mover.Body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        mover.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
        target.Body.IsSleeping.Should().BeFalse();
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.Position3d.X.Should().BeGreaterThan((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativeShapeExactPath_ShouldNotClampWhenBoundsProxyHitsButSweptShapeMissesSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(target.Body);

        target.Body.Sleep();
        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().Be((Fixed64)10);
        mover.Body.LinearVelocity.X.Should().Be((Fixed64)10);
        target.Body.IsSleeping.Should().BeTrue();
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativeShapeExactPath_ShouldClampThinCuboidAgainstDynamicSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(target.Body);

        target.Body.Sleep();
        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Collider.BoundsMax.X.Should().BeLessThanOrEqualTo(target.Collider.BoundsMin.X);
        mover.Body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        mover.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
        target.Body.IsSleeping.Should().BeFalse();
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(TestColliderShape.Cuboid)]
    [InlineData(TestColliderShape.Cylinder)]
    [InlineData(TestColliderShape.Cone)]
    [InlineData(TestColliderShape.ConvexMesh)]
    public void ContinuousMode_DynamicRelativeSphereSource_ShouldClampAgainstDynamicConvexTarget(TestColliderShape targetShape)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        (SolidBody targetBody, LSCollider targetCollider) = CreateDynamicTarget(
            scenario,
            targetShape,
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        targetBody.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(targetBody);

        targetBody.Sleep();
        mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
        scenario.Context.LateSimulate();

        mover.Body.Position3d.X.Should().BeLessThan(targetBody.Position3d.X);
        mover.Body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        mover.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
        targetBody.IsSleeping.Should().BeFalse();
        targetBody.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        targetCollider.BoundsMin.X.Should().BeGreaterThan(mover.Collider.BoundsMin.X);
    }

    [Fact]
    public void ContinuousMode_DynamicRelativeShapeExactPath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> mover = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(mover.Body);
        DisableGroundQueries(target.Body);

        void SimulateDynamicShapeExactCcd()
        {
            mover.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            target.Body.ResetPosition(new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero), FixedQuaternion.Identity);
            target.Body.Sleep();
            mover.Body.AddForce(Vector3d.Right * (Fixed64)10);
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateDynamicShapeExactCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
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
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldClampBeforeStaticTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.One);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldTransferVelocityToDynamicTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Body.Sleep();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)5);
        target.Body.Position3d.X.Should().BeGreaterThan(source.Body.Position3d.X);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslationAndShapeExactMiss_ShouldNotPushDynamicTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        target.Body.Sleep();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);

        source.Body.Agent.Transform.Position = Vector3d.Right * (Fixed64)10;
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)10);
        target.Body.Position3d.Should().Be(new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero));
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldNotTransferVelocityAcrossFrozenTargetAxis()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        target.Body.Sleep();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)5);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldRelayDynamicHandoffThroughChain()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> receiver = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Zero);
        receiver.Body.Sleep();
        middle.Body.Sleep();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        middle.Body.IsSleeping.Should().BeFalse();
        receiver.Body.IsSleeping.Should().BeFalse();
        middle.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.Body.Position3d.X.Should().BeGreaterThan((Fixed64)2);
        scenario.Context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(2);
        scenario.Context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldStillClampAtStaticAfterDynamicPush()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero));
        target.Body.Sleep();
        _ = scenario.CreateStaticSphere(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().BeLessThan((Fixed64)7);
        source.Body.Position3d.X.Should().BeLessThanOrEqualTo((Fixed64)2);
        target.Body.Position3d.X.Should().BeGreaterThan((Fixed64)(-1));
        target.Body.Position3d.X.Should().BeLessThanOrEqualTo((Fixed64)2);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_WithFastKinematicHostTranslation_ShouldNotPushDynamicTargetBehindEarlierStaticHit()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        target.Body.Sleep();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        scenario.Context.LateSimulate();

        Fixed64 expectedFirstHitX = -Fixed64.One;
        Fixed64 tolerance = Fixed64.FromFraction(1, 1024);
        source.Body.Position3d.X.Should().BeGreaterThanOrEqualTo(expectedFirstHitX - tolerance);
        source.Body.Position3d.X.Should().BeLessThanOrEqualTo(expectedFirstHitX + tolerance);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.Position3d.X.Should().Be((Fixed64)3);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_WithKinematicHostRotation_ShouldClampBeforeAngularTunnelingThroughStaticSphere()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        FixedQuaternion fullTurn = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        blade.Body.Agent.Transform.Rotation = fullTurn;
        scenario.Context.LateSimulate();

        FixedQuaternion.Angle(blade.Body.Rotation, fullTurn).Should().BeGreaterThan(Fixed64.Zero);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_WithKinematicHostTranslationAndRotation_ShouldSweepAngularCandidates()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        FixedQuaternion fullTurn = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);
        blade.Body.Agent.Transform.Position = Vector3d.Zero;
        blade.Body.Agent.Transform.Rotation = fullTurn;
        scenario.Context.LateSimulate();

        FixedQuaternion.Angle(blade.Body.Rotation, fullTurn).Should().BeGreaterThan(Fixed64.Zero);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_KinematicActiveSourcePath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        _ = scenario.CreateStaticSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        void SimulateKinematicCcd()
        {
            source.Body.ResetPosition(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateKinematicCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_KinematicActiveSourceBatch_ShouldNotAllocateAfterWarmup()
    {
        const int BodyCount = 256;
        const int Columns = 16;
        const int Spacing = 8;
        const int GridExtent = 96;
        Vector3d displacement = new((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        var sources = new SolidBody[BodyCount];
        var positions = new Vector3d[BodyCount];

        using GravitasWorldContext context = CreateOwnedCcdContext(GridExtent);
        for (int i = 0; i < BodyCount; i++)
        {
            int column = i % Columns;
            int row = i / Columns;
            var position = new Vector3d(
                (Fixed64)((column * Spacing) - ((Columns * Spacing) / 2)),
                Fixed64.Zero,
                (Fixed64)((row * Spacing) - ((Columns * Spacing) / 2)));
            positions[i] = position;
            sources[i] = CreateKinematicCcdSphere(context, position);
            _ = CreateStaticCcdSphere(context, position + new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        }

        void SimulateKinematicCcdBatch()
        {
            for (int i = 0; i < sources.Length; i++)
            {
                SolidBody source = sources[i];
                Vector3d position = positions[i];
                source.ResetPosition(position, FixedQuaternion.Identity);
                source.Agent.Transform.Position = position + displacement;
            }

            context.Simulate();
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateKinematicCcdBatch,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
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

        mover.Body.Position3d.X.Should().BeLessThan(target.Body.Position3d.X);
        mover.Body.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        mover.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)10);
        target.Body.Position3d.X.Should().BeGreaterThan((Fixed64)5);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
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
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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
        trigger.IsTrigger = true;
        (SolidBody body, _) = CreateMover(scenario, TestColliderShape.Sphere);
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

    private static GravitasWorldContext CreateOwnedCcdContext(int extent)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
        var configuration = new GridConfiguration(
            new Vector3d((Fixed64)(-extent), (Fixed64)(-extent), (Fixed64)(-extent)),
            new Vector3d((Fixed64)extent, (Fixed64)extent, (Fixed64)extent));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        return context;
    }

    private static SolidBody CreateKinematicCcdSphere(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(context, transform), new LSSphereCollider())
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            IsKinematic = true,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static SolidBody CreateStaticCcdSphere(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(
            new TestMatterAgent(context, transform),
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) })
        {
            FreezeAxes = BodyFreezeAxes3D.Position,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
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

    private static (SolidBody Body, LSCollider Collider) CreateDynamicTarget(
        PhysicsScenarioBuilder scenario,
        TestColliderShape shape,
        Vector3d position)
    {
        return shape switch
        {
            TestColliderShape.Cuboid => ToTuple(scenario.CreateCuboid(position)),
            TestColliderShape.Cylinder => ToTuple(scenario.CreateCylinder(position)),
            TestColliderShape.Cone => ToTuple(scenario.CreateCone(position)),
            TestColliderShape.ConvexMesh => ToTuple(scenario.CreateBody(
                MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
                position,
                FixedQuaternion.Identity)),
            _ => throw new System.ArgumentOutOfRangeException(nameof(shape), shape, null)
        };
    }

    private static (SolidBody Body, LSCollider Collider) CreateMover(PhysicsScenarioBuilder scenario, TestColliderShape shape)
    {
        return shape switch
        {
            TestColliderShape.Sphere => ToTuple(scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Capsule => ToTuple(scenario.CreateCapsule(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Cuboid => ToTuple(scenario.CreateCuboid(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Cylinder => ToTuple(scenario.CreateCylinder(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.Cone => ToTuple(scenario.CreateCone(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            TestColliderShape.ConvexMesh => ToTuple(scenario.CreateBody(
                MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity)),
            TestColliderShape.Compound => ToTuple(scenario.CreateBody(
                new LSCompoundCollider(
                    CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                    CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero))),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity)),
            _ => throw new System.ArgumentOutOfRangeException(nameof(shape), shape, null)
        };
    }

    private static (SolidBody Body, LSCollider Collider) ToTuple<TCollider>(ScenarioBody<TCollider> scenarioBody)
        where TCollider : LSCollider =>
        (scenarioBody.Body, scenarioBody.Collider);

    private static void ApplyFastImpulse(SolidBody body) =>
        body.AddLinearImpulse(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

    private static void DisableGroundQueries(SolidBody body)
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
        Cone,
        ConvexMesh,
        Compound
    }
}
