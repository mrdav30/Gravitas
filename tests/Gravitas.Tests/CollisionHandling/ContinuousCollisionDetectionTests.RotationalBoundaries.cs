using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalDynamicResponse_NonClosingPair_ShouldRejectWithoutMutation3D(
        bool tangent)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            -Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(
            tangent ? Vector3d.Up : Vector3d.Left);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        var contact = new ManifoldContact(
            contactId: 1,
            pointA: -Vector3d.Right,
            pointB: -Vector3d.Right,
            depth: Fixed64.Zero,
            normal: Vector3d.Right);

        source.Body.TryApplyRotationalContinuousCollisionResponse(
                target.Body,
                contact,
                Fixed64.Half,
                source.Body.Position3d,
                Vector3d.Zero,
                source.Body.Rotation,
                Fixed64.Zero,
                scenario.Context.DeltaTime,
                sourceIsKinematic: false)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void KinematicRotationalResponse_WithZeroMassFrozenRotationTarget_ShouldRejectZeroEffectiveMass3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Mass = Fixed64.Zero;
        target.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void DynamicRotationalResponse_WhenSourceTrajectoryIsFull_ShouldRejectPairAtomically3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        blade.Body.ApplyContinuousCollisionHandoff(
                blade.Body.Position3d,
                blade.Body.Rotation,
                blade.Body.LinearVelocity,
                blade.Body.AngularVelocity,
                scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector3d targetPosition = target.Body.Position3d;

        blade.Body.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();

        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void KinematicRotationalResponse_WhenTargetTrajectoryIsFull_ShouldLeaveTargetAtomic3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                target.Body.Position3d,
                target.Body.Rotation,
                Vector3d.Zero,
                Vector3d.Zero,
                scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector3d targetPosition = target.Body.Position3d;

        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void DynamicRotationalResponse_AtIterationLimit_ShouldStopAfterFirstMovingTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> first = CreateDynamicRotationalTarget3D(
            scenario,
            PhysicsScenarioBuilder.Yaw(30));
        ScenarioBody<LSSphereCollider> second = CreateDynamicRotationalTarget3D(
            scenario,
            PhysicsScenarioBuilder.Yaw(60));
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        second.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.Sleep();
        second.Body.Sleep();

        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.LateSimulate();

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        first.Body.IsSleeping.Should().BeFalse();
        second.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void KinematicRotationalResponse_PositionFrozenSphereTarget_ShouldRejectZeroMobilityImpulse()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void KinematicRotationalResponse_UnrepresentableWorldCenterOfMass_ShouldUseRelativeLeverArm()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.LocalCenterOfMassOffset = Vector3d.Up * Fixed64.MaxValue;
        blade.Body.ResetPosition(Vector3d.Up, FixedQuaternion.Identity);
        target.Body.ResetPosition(target.Body.Position3d + Vector3d.Up, FixedQuaternion.Identity);
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared
            + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void RotationalCandidateFallback_UnrepresentableRadius_ShouldFindOnlyRotatingBodies()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);

        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeFalse();

        _ = scenario.CreateStaticSphere(Vector3d.Right);
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeFalse();

        ScenarioBody<LSCuboidCollider> rotatingTarget = scenario.CreateCuboid(
            Vector3d.Right * (Fixed64)4);
        rotatingTarget.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up);

        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeTrue();
        source.Body.GatherRotationalContinuousCollisionCandidates(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .Be(2);
    }

    [Fact]
    public void SphereSeparationGap_ShouldSupportSphereAndCuboidPairsOnly()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider sphere = scenario.CreateStaticSphere(Vector3d.Right * (Fixed64)4);
        var cuboid = new LSCuboidCollider();
        scenario.InitializeStaticCollider(cuboid, Vector3d.Right * (Fixed64)4);
        var capsule = new LSCapsuleCollider();
        scenario.InitializeStaticCollider(capsule, Vector3d.Right * (Fixed64)4);

        SolidBody.TryGetSphereSeparationGap(source, sphere, out Fixed64 sphereGap)
            .Should()
            .BeTrue();
        sphereGap.Should().BeGreaterThan(Fixed64.Zero);
        SolidBody.TryGetSphereSeparationGap(source, cuboid, out Fixed64 cuboidGap)
            .Should()
            .BeTrue();
        cuboidGap.Should().BeGreaterThan(Fixed64.Zero);
        SolidBody.TryGetSphereSeparationGap(source, capsule, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_AtScalarFace_ShouldUseRelativeSurfaceAnchor()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)4,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        Vector3d center = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 8),
            Fixed64.Zero,
            Fixed64.Zero);
        var sphere = new LSSphereCollider();
        sphere.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                center,
                FixedQuaternion.Identity,
                Vector3d.One)));
        var cuboid = new LSCuboidCollider();
        cuboid.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                center,
                FixedQuaternion.Identity,
                Vector3d.One)));

        bool certified = true;
        Action query = () => certified =
            SolidBody.TryGetSphereSeparationGap(
                sphere,
                cuboid,
                out _);

        query.Should().NotThrow();
        certified.Should().BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCenterDelta_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_UnrepresentableAnchorDelta_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var target = new LSCuboidCollider();
        scenario.InitializeStaticCollider(
            target,
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableDistance_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCharacteristicScale_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_UnrepresentableProxyScale_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        var target = new LSCuboidCollider();
        scenario.InitializeStaticCollider(
            target,
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero));
        target.Size = new Vector3d(
            Fixed64.One,
            (Fixed64)2_000_000_000,
            (Fixed64)2_000_000_000);
        target.RebuildRuntimeShapeOnly(refreshMassProperties: false);

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCombinedRadius_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(Vector3d.Zero);
        source.Radius = Fixed64.MaxValue;
        target.Radius = Fixed64.MaxValue;

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }
}
