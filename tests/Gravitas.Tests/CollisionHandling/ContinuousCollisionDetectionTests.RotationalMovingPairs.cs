using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    private static readonly FixedQuaternion RotationalMovingPairQuarterTurn3D =
        PhysicsScenarioBuilder.Yaw(90);

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldPushDynamic3DTargetAndReachAuthoredRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMovingPairQuarterTurn3D);
        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void AutoMode_KinematicModestRotationWithLargeTranslation_ShouldUseUnifiedMovingPairPath()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        Vector3d authoredPosition = Vector3d.Right * (Fixed64)4;
        FixedQuaternion authoredRotation = PhysicsScenarioBuilder.Yaw(10);
        Vector3d rotatedTip = authoredPosition + authoredRotation * (Vector3d.Right * (Fixed64)3);
        ScenarioBody<LSSphereCollider> target = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            rotatedTip,
            FixedQuaternion.Identity);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalPosition = authoredPosition;
        blade.Body.Agent.Transform.LocalRotation = authoredRotation;
        scenario.Context.LateSimulate();

        blade.Body.Position3d.Should().Be(authoredPosition);
        blade.Body.Rotation.Should().Be(authoredRotation);
        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicPureRotation_ShouldTransferMomentumToDynamic3DTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
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
        target.Body.Sleep();

        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.LateSimulate();

        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousMode_DynamicRotationalMovingPair_ShouldReplayDeterministically()
    {
        var first = RunDynamicRotationalMovingPairReplay3D();
        var second = RunDynamicRotationalMovingPairReplay3D();

        first.TargetMotionMagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        first.Hash.Should().Be(second.Hash);
    }

    [Fact]
    public void ContinuousCollisionHandoff_WithPendingAngularContinuation_ShouldReplayAppendedTrajectoryDeterministically()
    {
        var first = RunPendingAngularHandoffReplay3D();
        var second = RunPendingAngularHandoffReplay3D();

        first.TrajectoryCount.Should().BeGreaterThan(1);
        first.TrajectoryAngularVelocity.MagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        first.Hash.Should().Be(second.Hash);
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldBeIndependentOf3DBodyRegistrationOrder()
    {
        var sourceFirst = RunKinematicRotationalMovingPair3D(targetFirst: false);
        var targetFirst = RunKinematicRotationalMovingPair3D(targetFirst: true);

        sourceFirst.TargetLinearVelocity.MagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        sourceFirst.Should().Be(targetFirst);
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_WithOffsetCenterOfMass_ShouldBeOrderIndependent()
    {
        var sourceFirst = RunKinematicRotationalMovingPair3D(
            targetFirst: false,
            useCenterOfMassOffset: true);
        var targetFirst = RunKinematicRotationalMovingPair3D(
            targetFirst: true,
            useCenterOfMassOffset: true);

        (sourceFirst.TargetLinearVelocity.MagnitudeSquared
            + sourceFirst.TargetAngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        sourceFirst.Should().Be(targetFirst);
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_WithUnrepresentableTargetContactArm_ShouldRejectResponseAtomically()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.LocalCenterOfMassOffset = Vector3d.Right * Fixed64.MaxValue;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        target.Body.IsSleeping.Should().BeTrue();
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_DynamicPureRotation_ShouldRespondToMovingKinematic3DTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            CreateDynamicRotationalTarget3DPosition(),
            isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        Vector3d angularVelocity = Vector3d.Up * FixedMath.DegToRad((Fixed64)90);
        blade.Body.ApplyCollisionAngularVelocityDelta(angularVelocity);
        scenario.Context.LateSimulate();

        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        blade.Body.AngularVelocity.MagnitudeSquared.Should().BeLessThan(angularVelocity.MagnitudeSquared);
        target.Body.Position3d.Should().Be(CreateDynamicRotationalTarget3DPosition());
    }

    [Fact]
    public void ContinuousMode_KinematicPureRotation_ShouldContinueThroughMultipleDynamic3DTargets()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
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

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMovingPairQuarterTurn3D);
        first.Body.IsSleeping.Should().BeFalse();
        second.Body.IsSleeping.Should().BeFalse();
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThanOrEqualTo(2);
    }

    [Fact]
    public void KinematicRotationalResponse_WhenIterationBudgetExhausts_ShouldReachAuthoredEndpoint()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMovingPairQuarterTurn3D);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        target.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ContinuousMode_RotationalMovingPairPath_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d targetPosition = target.Body.Position3d;

        void SimulateRotationalMovingPair()
        {
            blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            target.Body.ResetPosition(targetPosition, FixedQuaternion.Identity);
            target.Body.Sleep();
            blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateRotationalMovingPair,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_CombinedMotion_ShouldTransferContactPointMomentumToDynamic3DTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
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
        target.Body.Sleep();

        blade.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(1, 10));
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.LateSimulate();

        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousCollisionHandoff_WithAngularVelocity_ShouldContinueRemaining3DRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        body.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Up * FixedMath.DegToRad((Fixed64)90),
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeTrue();
        body.Body.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();

        FixedQuaternion.Angle(FixedQuaternion.Identity, body.Body.Rotation)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousCollisionHandoff_WithAnisotropicBody_ShouldRefreshWorldInertiaAtEveryAuthoritativeRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        body.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(body.Body);
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        Fixed3x3 localInverseInertia = body.Body.InverseInertiaTensor;
        FixedQuaternion impactRotation = RotationalMovingPairQuarterTurn3D;

        body.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                impactRotation,
                Vector3d.Zero,
                Vector3d.Up * Fixed64.Pi,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeTrue();

        body.Body.InverseInertiaTensor.Should().Be(
            impactRotation.ToMatrix3x3()
            * localInverseInertia
            * impactRotation.Conjugate().ToMatrix3x3());

        body.Body.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();
        body.Body.InverseInertiaTensor.Should().Be(
            body.Body.Rotation.ToMatrix3x3()
            * localInverseInertia
            * body.Body.Rotation.Conjugate().ToMatrix3x3());
    }

    [Fact]
    public void ContinuousMode_DynamicPureRotationAgainstMovingKinematic3DTarget_ShouldBeOrderIndependent()
    {
        var sourceFirst = RunDynamicRotationalMovingKinematicPair3D(targetFirst: false);
        var targetFirst = RunDynamicRotationalMovingKinematicPair3D(targetFirst: true);

        // Pure rotation starts with no linear motion; a non-zero linear result
        // proves the moving kinematic target actually participated in response.
        sourceFirst.BladeLinearVelocity.MagnitudeSquared
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        sourceFirst.Should().Be(targetFirst);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_NonRotatingSource_ShouldSampleIntermediateDiscreteKinematic3DTargetRotation(
        bool sourceTranslates)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        FixedQuaternion startRotation = PhysicsScenarioBuilder.Yaw(-45);
        FixedQuaternion targetRotation = PhysicsScenarioBuilder.Yaw(45);
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            startRotation,
            isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            new Vector3d(
                Fixed64.FromFraction(31, 10),
                Fixed64.Zero,
                Fixed64.FromFraction(-1, 10)),
            FixedQuaternion.Identity);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        Vector3d requestedVelocity = sourceTranslates
            ? Vector3d.Forward * Fixed64.FromFraction(1, 5)
            : Vector3d.Zero;

        blade.Body.Agent.Transform.LocalRotation = targetRotation;
        source.Body.AddLinearImpulse(requestedVelocity);
        scenario.Context.LateSimulate();

        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Body.LinearVelocity.Should().NotBe(requestedVelocity);
    }

    [Fact]
    public void RotationalHandoff_WhenTrajectoryBudgetIsFull_ShouldLeaveBodyStateAtomic()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        Fixed64 firstRemainingTime = scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4);
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Vector3d.Up,
                firstRemainingTime)
            .Should()
            .BeTrue();
        Vector3d positionBeforeRejectedHandoff = target.Body.Position3d;
        FixedQuaternion rotationBeforeRejectedHandoff = target.Body.Rotation;
        Vector3d linearVelocityBeforeRejectedHandoff = target.Body.LinearVelocity;
        Vector3d angularVelocityBeforeRejectedHandoff = target.Body.AngularVelocity;
        int trajectoryCountBeforeRejectedHandoff = target.Body.ContinuousCollisionTrajectoryCount;

        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Left,
                RotationalMovingPairQuarterTurn3D,
                Vector3d.Left,
                -Vector3d.Up,
                scenario.Context.DeltaTime * Fixed64.FromFraction(1, 2))
            .Should()
            .BeFalse();

        target.Body.Position3d.Should().Be(positionBeforeRejectedHandoff);
        target.Body.Rotation.Should().Be(rotationBeforeRejectedHandoff);
        target.Body.LinearVelocity.Should().Be(linearVelocityBeforeRejectedHandoff);
        target.Body.AngularVelocity.Should().Be(angularVelocityBeforeRejectedHandoff);
        target.Body.ContinuousCollisionTrajectoryCount.Should().Be(trajectoryCountBeforeRejectedHandoff);
    }

    [Fact]
    public void RotationalHandoff_DirtyCandidateBounds_ShouldRetainEarlierTrajectoryExcursion()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 2;
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        Fixed64 firstRemainingTime = scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4);
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Right * (Fixed64)8,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Up,
                firstRemainingTime)
            .Should()
            .BeTrue();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Left * (Fixed64)8,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Up,
                scenario.Context.DeltaTime * Fixed64.FromFraction(1, 4))
            .Should()
            .BeTrue();

        var queryBounds = new FixedBoundVolume(
            new Vector3d((Fixed64)7, (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d((Fixed64)9, Fixed64.One, Fixed64.One));
        SwiftList<int> candidateIds = scenario.Context.Physics.QueryContinuousCollisionCandidates(queryBounds);

        candidateIds.Should().Contain(target.Body.DynamicId);
    }

    private static (
        FixedQuaternion BladeRotation,
        Vector3d TargetPosition,
        Vector3d TargetLinearVelocity,
        Vector3d TargetAngularVelocity) RunKinematicRotationalMovingPair3D(
            bool targetFirst,
            bool useCenterOfMassOffset = false)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade;
        ScenarioBody<LSSphereCollider> target;
        if (targetFirst)
        {
            target = CreateDynamicRotationalTarget3D(scenario);
            blade = CreateKinematicRotationalCcdBlade(scenario);
        }
        else
        {
            blade = CreateKinematicRotationalCcdBlade(scenario);
            target = CreateDynamicRotationalTarget3D(scenario);
        }

        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        if (useCenterOfMassOffset)
        {
            target.Body.LocalCenterOfMassOffset = new Vector3d(
                Fixed64.FromFraction(1, 8),
                Fixed64.Zero,
                Fixed64.Zero);
        }
        target.Body.Sleep();
        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;

        scenario.Context.LateSimulate();

        return (
            blade.Body.Rotation,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            target.Body.AngularVelocity);
    }

    private static (
        Vector3d BladePosition,
        Vector3d BladeLinearVelocity,
        Vector3d BladeAngularVelocity,
        Vector3d TargetPosition) RunDynamicRotationalMovingKinematicPair3D(bool targetFirst)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade;
        ScenarioBody<LSSphereCollider> target;
        Vector3d targetStart = CreateDynamicRotationalTarget3DPosition();
        if (targetFirst)
        {
            target = scenario.CreateSphere(targetStart, isKinematic: true);
            blade = scenario.CreateBody(
                new LSCuboidCollider
                {
                    Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
                },
                Vector3d.Zero,
                FixedQuaternion.Identity);
        }
        else
        {
            blade = scenario.CreateBody(
                new LSCuboidCollider
                {
                    Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
                },
                Vector3d.Zero,
                FixedQuaternion.Identity);
            target = scenario.CreateSphere(targetStart, isKinematic: true);
        }

        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d targetEnd = targetStart - Vector3d.Right * Fixed64.FromFraction(1, 10);
        target.Body.Agent.Transform.LocalPosition = targetEnd;
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));

        scenario.Context.LateSimulate();

        target.Body.Position3d.Should().Be(targetEnd);
        return (
            blade.Body.Position3d,
            blade.Body.LinearVelocity,
            blade.Body.AngularVelocity,
            target.Body.Position3d);
    }

    private static ScenarioBody<LSSphereCollider> CreateDynamicRotationalTarget3D(
        PhysicsScenarioBuilder scenario)
        => CreateDynamicRotationalTarget3D(scenario, PhysicsScenarioBuilder.Yaw(45));

    private static (
        ChronicleHash Hash,
        Fixed64 TargetMotionMagnitudeSquared) RunDynamicRotationalMovingPairReplay3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
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
        target.Body.Sleep();
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));

        scenario.Context.LateSimulate();

        return (
            scenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches),
            target.Body.LinearVelocity.MagnitudeSquared
                + target.Body.AngularVelocity.MagnitudeSquared);
    }

    private static (
        ChronicleHash Hash,
        int TrajectoryCount,
        Vector3d TrajectoryAngularVelocity) RunPendingAngularHandoffReplay3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        body.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(body.Body);
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        body.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                RotationalMovingPairQuarterTurn3D,
                Vector3d.Zero,
                Vector3d.Up * Fixed64.Pi,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeTrue();

        return (
            scenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches),
            body.Body.ContinuousCollisionTrajectoryCount,
            body.Body.SampleContinuousCollisionAngularVelocity(Fixed64.One));
    }

    private static ScenarioBody<LSSphereCollider> CreateDynamicRotationalTarget3D(
        PhysicsScenarioBuilder scenario,
        FixedQuaternion contactRotation)
    {
        return scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            contactRotation
                * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
    }

    private static Vector3d CreateDynamicRotationalTarget3DPosition() =>
        PhysicsScenarioBuilder.Yaw(45)
        * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero);
}
