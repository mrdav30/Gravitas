using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    private static readonly Fixed64 RotationalMixedQuarterTurn =
        FixedMath.DegToRad((Fixed64)90);
    private static readonly FixedQuaternion RotationalMixedQuarterTurn3D =
        FixedQuaternion.FromAxisAngle(Vector3d.Up, RotationalMixedQuarterTurn);

    [Fact]
    public void MixedMode_Kinematic3DPureRotation_ShouldPushDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        target.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
        target.IsSleeping.Should().BeFalse();
        (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic2DPureRotation_ShouldPushDynamic3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        target.Body.Sleep();

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.LateSimulate();

        blade.Rotation.Should().Be(RotationalMixedQuarterTurn);
        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void BothMode_Kinematic3DPureRotation_ShouldNotPushDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        target.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
        target.IsSleeping.Should().BeTrue();
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void BothMode_Kinematic2DPureRotation_ShouldNotPushDynamic3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        target.Body.Sleep();

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.LateSimulate();

        blade.Rotation.Should().Be(RotationalMixedQuarterTurn);
        target.Body.IsSleeping.Should().BeTrue();
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void MixedMode_Dynamic3DPureRotation_ShouldRespondToBodyless2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> blade = CreateBody3D(
            context,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d targetPosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)45))
            * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero))
            .ToVector2d();
        _ = CreateBodylessCircle2D(context, targetPosition);
        Fixed64 requestedAngularVelocity = FixedMath.DegToRad((Fixed64)90);

        blade.Body.AddAngularImpulse(
            Vector3d.Up
            * (requestedAngularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));
        context.LateSimulate();

        blade.Body.AngularVelocity.Y.Should().NotBe(requestedAngularVelocity);
    }

    [Fact]
    public void MixedMode_Dynamic2DPureRotation_ShouldRespondToBodyless3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        blade.SetMotionType(BodyMotionType.Dynamic);
        Vector3d targetPosition = Vector2d.Rotate(
                new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)45))
            .ToVector3d(Fixed64.Zero);
        _ = CreateBodyless3D(context, new LSSphereCollider(), targetPosition);
        Fixed64 requestedAngularVelocity = FixedMath.DegToRad((Fixed64)90);

        blade.AddAngularImpulse(
            requestedAngularVelocity / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        blade.AngularVelocity.Should().NotBe(requestedAngularVelocity);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotationalMovingPair_ShouldBeRegistrationOrderIndependent()
    {
        RunKinematic3DRotationalMixedPair(targetFirst: false)
            .Should()
            .Be(RunKinematic3DRotationalMixedPair(targetFirst: true));
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_WithUnrepresentableSourceContactArm_ShouldRejectResponseAtomically()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        blade.Body.LocalCenterOfMassOffset = Vector3d.Right * Fixed64.MaxValue;
        target.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        target.IsSleeping.Should().BeTrue();
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_WithFullDomainSourceCenterOfMass_ShouldResolveRepresentableContactArm()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        blade.Body.LocalCenterOfMassOffset = Vector3d.Up * Fixed64.MaxValue;
        blade.Body.ResetPosition(Vector3d.Up, FixedQuaternion.Identity);
        target.Agent.Transform.LocalPosition = new Vector3d(
            target.Position.X,
            Fixed64.One,
            target.Position.Y);
        target.Collider.RebuildRuntimeShapeOnly();

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.LinearVelocity.Should().Be(new Vector2d(
            (Fixed64)0.45798352430574596,
            (Fixed64)(-2.9263750046957284)));
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_WithUnrepresentableFinalContactArm_ShouldPreserveResponse()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        blade.Body.LocalCenterOfMassOffset = new Vector3d(
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.Zero);
        blade.Body.ResetPosition(Vector3d.Up, FixedQuaternion.Identity);
        target.Agent.Transform.LocalPosition = new Vector3d(
            target.Position.X,
            Fixed64.One,
            target.Position.Y);
        target.Collider.RebuildRuntimeShapeOnly();

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_WithUnrepresentable2DCenterOfMass_ShouldPreserveResponse()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        target.LocalCenterOfMassOffset = Vector2d.Right * Fixed64.MaxValue;

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_WithZeroMassFrozen2DTarget_ShouldRejectZeroEffectiveMass()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        target.Mass = Fixed64.Zero;
        target.FreezeAxes = BodyFreezeAxes2D.Rotation;

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(RotationalMixedQuarterTurn3D);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotation_WithUnrepresentable3DCenterOfMass_ShouldPreserveResponse()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        target.Body.LocalCenterOfMassOffset = Vector3d.Right * Fixed64.MaxValue;

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.LateSimulate();

        blade.Rotation.Should().Be(RotationalMixedQuarterTurn);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotation_WithZeroMassFrozen3DTarget_ShouldRejectZeroEffectiveMass()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        target.Body.Mass = Fixed64.Zero;
        target.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(RotationalMixedQuarterTurn);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void MixedMode_Dynamic2DRotationalResponse_WhenSourceTrajectoryIsFull_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        blade.SetMotionType(BodyMotionType.Dynamic);
        Vector3d targetPosition = Vector2d.Rotate(
                new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)45))
            .ToVector3d(Fixed64.Zero);
        LSCollider target = CreateBodyless3D(context, new LSSphereCollider(), targetPosition);
        blade.ApplyCollisionAngularVelocityDelta(RotationalMixedQuarterTurn);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.ApplyContinuousCollisionHandoffState(
                blade.Position,
                blade.Rotation,
                blade.LinearVelocity,
                blade.AngularVelocity,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        blade.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();

        blade.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        target.Center.Should().Be(targetPosition);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotationalResponse_WhenTargetTrajectoryIsFull_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                target.Body.Position3d,
                target.Body.Rotation,
                Vector3d.Zero,
                Vector3d.Zero,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector3d targetPosition = target.Body.Position3d;
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;

        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Rotation.Should().BeLessThan(RotationalMixedQuarterTurn);
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotationalMovingPair_ShouldBeRegistrationOrderIndependent()
    {
        RunKinematic2DRotationalMixedPair(targetFirst: false)
            .Should()
            .Be(RunKinematic2DRotationalMixedPair(targetFirst: true));
    }

    [Fact]
    public void MixedDirty2DTrajectoryBounds_ShouldRetainEarlierExcursion()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D body = CreateRotationalMixedTarget2D(context);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        Vector2d excursion = Vector2d.Right * (Fixed64)6;

        body.ApplyContinuousCollisionHandoff(
                excursion,
                Vector2d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        body.ApplyContinuousCollisionHandoff(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        FixedBoundVolume query = DynamicCcdCandidateIndex.CreateBoundsBetween(
            new Vector3d(excursion.X, body.Collider.MixedSlabCenterY, excursion.Y),
            new Vector3d(excursion.X, body.Collider.MixedSlabCenterY, excursion.Y),
            Vector3d.One * Fixed64.FromFraction(1, 10));
        context.Physics2D.QueryMixedContinuousCollisionCandidates(query)
            .Should()
            .Contain(body.DynamicId);
    }

    [Fact]
    public void MixedMode_Unrepresentable3DPivotRadius_ShouldStillAdmitMoving2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        Vector3d extremeOffset = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        blade.Collider.LocalOffset = extremeOffset;
        blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        blade.Collider.RebuildRuntimeShapeOnly();
        target.Collider.LocalOffset = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                RotationalMixedQuarterTurn * Fixed64.Half)
            * extremeOffset).ToVector2d();
        target.ResetPosition(Vector2d.Zero);
        target.Sleep();
        blade.Body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.Rotation.Should().NotBe(RotationalMixedQuarterTurn3D);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MixedMode_Unrepresentable3DPivotRadius_ShouldStillClampAgainstBodyless2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        Vector3d extremeOffset = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        blade.Collider.LocalOffset = extremeOffset;
        blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        blade.Collider.RebuildRuntimeShapeOnly();
        Vector2d targetPosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                RotationalMixedQuarterTurn * Fixed64.Half)
            * extremeOffset).ToVector2d();
        _ = CreateBodylessCircle2D(context, targetPosition);
        blade.Body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);

        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.Rotation.Should().NotBe(RotationalMixedQuarterTurn3D);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MixedMode_Unrepresentable2DPivotRadius_ShouldStillAdmitMoving3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        Vector2d extremeOffset = new(Fixed64.MaxValue, Fixed64.Zero);
        blade.Collider.LocalOffset = extremeOffset;
        blade.ResetPosition(Vector2d.Zero);
        target.Collider.LocalOffset = Vector2d.Rotate(
                extremeOffset,
                RotationalMixedQuarterTurn * Fixed64.Half)
            .ToVector3d(Fixed64.Zero);
        target.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        target.Body.Sleep();
        blade.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);
        blade.ResolveMixedContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);

        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Rotation.Should().Be(RotationalMixedQuarterTurn);
        (target.Body.LinearVelocity.MagnitudeSquared
            + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        blade.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotationalMovingPair_ShouldReplayDeterministically()
    {
        RunKinematic3DRotationalMixedPairHash()
            .Should()
            .Be(RunKinematic3DRotationalMixedPairHash());
    }

    [Fact]
    public void MixedMode_Kinematic2DRotationalMovingPair_ShouldReplayDeterministically()
    {
        RunKinematic2DRotationalMixedPairHash()
            .Should()
            .Be(RunKinematic2DRotationalMixedPairHash());
    }

    [Fact]
    public void MixedMode_Kinematic3DRotationalMovingPair_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        Vector2d targetPosition = target.Position;

        void RunIteration()
        {
            blade.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            target.ResetPosition(targetPosition);
            target.Sleep();
            blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            RunIteration,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotationalMovingPair_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        Vector3d targetPosition = target.Body.Position3d;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                blade.Agent.Transform.LocalRotationXZRadians = Fixed64.Zero;
                blade.ResetPosition(Vector2d.Zero);
                target.Body.ResetPosition(targetPosition, FixedQuaternion.Identity);
                target.Body.Sleep();
                blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
                context.LateSimulate();
            },
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }
}
