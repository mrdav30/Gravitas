using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    public void ContinuousMode_CenteredCircleTurningOnSupport_ShouldAcceptNonClosingPose(
        int translationUnits,
        bool nonuniformScale)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(4, 2)),
            new Vector2d(Fixed64.Zero, -Fixed64.One), immovable: true);
        Fixed64 supportHeight = Fixed64.FromFraction(nonuniformScale ? 3 : 1, 8);
        var collider = new LSCircleCollider2D(Fixed64.FromFraction(1, 8));
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, supportHeight),
            FixedQuaternion.Identity,
            nonuniformScale ? new Vector3d(3, 1, 2) : Vector3d.One));
        var source = new SolidBody2D(agent, collider);
        source.Initialize(new Vector2d(Fixed64.Zero, supportHeight), motionType: BodyMotionType.Kinematic);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d target = source.Position
            + Vector2d.Right * Fixed64.FromFraction(translationUnits, 1024);
        Fixed64 rotation = Fixed64.Pi / (Fixed64)64;

        source.Agent.Transform.LocalPosition = new Vector3d(target.X, Fixed64.Zero, target.Y);
        source.Agent.Transform.LocalRotationXZRadians = rotation;
        // The host stores a quaternion; its canonical planar angle is the
        // requested pose seen by physics, including fixed-point conversion.
        Fixed64 requestedRotation = source.Agent.Transform.WorldRotationXZRadians;
        context.LateSimulate();

        source.Position.Should().Be(target);
        source.Rotation.Should().Be(requestedRotation);
        collider.ScaledRadius.Should().Be(supportHeight);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_CenteredCircleTurningDownIntoSupport_ShouldBlockClosingTranslation()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(4, 2)),
            new Vector2d(Fixed64.Zero, -Fixed64.One), immovable: true);
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8)),
            immovable: false, isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d start = source.Position;
        Vector2d target = start - Vector2d.Forward * Fixed64.FromFraction(1, 64);
        source.Agent.Transform.LocalPosition = new Vector3d(target.X, Fixed64.Zero, target.Y);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)10);

        context.LateSimulate();

        source.Position.Should().Be(start);
        source.Position.Should().NotBe(target);
    }

    [Fact]
    public void ContinuousMode_CenteredCircleTurningAcrossSupportIntoWall_ShouldStopAtWall()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(4, 2)),
            new Vector2d(Fixed64.Zero, -Fixed64.One), immovable: true);
        _ = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Quarter, (Fixed64)2)),
            new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8)),
            immovable: false, isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition = new Vector3d(Fixed64.One, Fixed64.Zero, source.Position.Y);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)10);

        context.LateSimulate();

        // The wall's left face is 3/8, leaving the circle's center at or before 1/4.
        source.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Position.X.Should().BeLessThanOrEqualTo(Fixed64.Quarter);
        source.Position.Y.Should().Be(Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void ContinuousMode_DynamicCenteredCircleTurningOnSupport_ShouldPreserveTangentialMotion()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        _ = CreateBody(context,
            new LSAABBoxCollider2D(new Vector2d(4, 2)) { Material = PhysicsMaterial.Frictionless },
            new Vector2d(Fixed64.Zero, -Fixed64.One), immovable: true);
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)) { Material = PhysicsMaterial.Frictionless },
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8)), immovable: false);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d velocity = Vector2d.Right * Fixed64.FromFraction(1, 1024);
        Vector2d target = source.Position + velocity;
        source.AddLinearImpulse(velocity);
        source.AddAngularImpulse(Fixed64.FromFraction(1, 64) / source.EffectiveInverseMomentOfInertia);
        Fixed64 angularVelocity = source.AngularVelocity;

        context.LateSimulate();

        source.Position.Should().Be(target);
        source.LinearVelocity.Should().Be(velocity);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.Rotation.Should().Be(angularVelocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_TurningCenteredCircle_ShouldStillBeHitByNearbyRotatingBlade(bool sourceFirst)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        var sourceCollider = new LSCircleCollider2D(Fixed64.Quarter);
        var sourceStart = new Vector2d(Fixed64.FromFraction(31, 10), Fixed64.FromFraction(-1, 10));
        SolidBody2D source;
        SolidBody2D blade;
        if (sourceFirst)
        {
            source = CreateBody(context, sourceCollider, sourceStart, immovable: false);
            blade = CreateRotationalBlade(context);
        }
        else
        {
            blade = CreateRotationalBlade(context);
            source = CreateBody(context, sourceCollider, sourceStart, immovable: false);
        }

        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        blade.ResetPosition(Vector2d.Zero, -FixedMath.DegToRad((Fixed64)45));
        source.AddAngularImpulse(Fixed64.FromFraction(1, 64) / source.EffectiveInverseMomentOfInertia);
        blade.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)45);

        context.LateSimulate();

        // Neither endpoint overlaps; the moving blade must transfer actual motion at an intermediate pose.
        source.LinearVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        source.Position.Should().NotBe(sourceStart);
    }

    [Fact]
    public void ContinuousMode_CenteredCircleTurningOnSupportNearSeparatedRotatingBlade_ShouldReachHostPose()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(4, 2)),
            new Vector2d(Fixed64.Zero, -Fixed64.One), immovable: true);
        SolidBody2D blade = CreateRotationalBlade(context);
        blade.ResetPosition(new Vector2d(Fixed64.Zero, (Fixed64)2), -FixedMath.DegToRad((Fixed64)5));
        // ResetPosition changes the body pose; the host must also submit its
        // intended kinematic endpoint (rotation only, not a return to origin).
        blade.Agent.Transform.LocalPosition = new Vector3d(0, 0, 2);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(1, 8)),
            immovable: false, isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d target = source.Position + Vector2d.Right * Fixed64.FromFraction(1, 1024);
        blade.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);
        source.Agent.Transform.LocalPosition = new Vector3d(target.X, Fixed64.Zero, target.Y);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);
        Fixed64 requestedRotation = source.Agent.Transform.WorldRotationXZRadians;

        context.LateSimulate();

        // Even at +/-5 degrees the blade stays above 1.6, far above the circle top at 1/4.
        source.Position.Should().Be(target);
        source.Rotation.Should().Be(requestedRotation);
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, true)]
    [InlineData(false, false)]
    public void ContinuousMode_SpinningCenteredCircle_ShouldRespectMixedModeForDiscreteBlade(
        bool sourceFirst,
        bool mixedContacts)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = mixedContacts ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.Both;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        var sourceStart = new Vector2d(Fixed64.FromFraction(31, 10), Fixed64.FromFraction(-1, 10));
        var sourceCollider = new LSCircleCollider2D(Fixed64.Quarter);
        var bladeCollider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
        };
        FixedQuaternion startRotation = PhysicsScenarioBuilder.Yaw(-45);
        FixedQuaternion endRotation = PhysicsScenarioBuilder.Yaw(45);
        SolidBody2D source;
        ScenarioBody<LSCuboidCollider> blade;
        if (sourceFirst)
        {
            source = CreateBody(context, sourceCollider, sourceStart, immovable: false);
            blade = scenario.CreateBody(bladeCollider, Vector3d.Zero, startRotation, isKinematic: true);
        }
        else
        {
            blade = scenario.CreateBody(bladeCollider, Vector3d.Zero, startRotation, isKinematic: true);
            source = CreateBody(context, sourceCollider, sourceStart, immovable: false);
        }

        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.UseManualGrounding();
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        CollisionDetectionMixed.TryCollide(bladeCollider, sourceCollider, out _).Should().BeFalse();
        blade.Body.SetRotation(FixedQuaternion.Identity);
        CollisionDetectionMixed.TryCollide(bladeCollider, sourceCollider, out _).Should().BeTrue();
        blade.Body.SetRotation(endRotation);
        CollisionDetectionMixed.TryCollide(bladeCollider, sourceCollider, out _).Should().BeFalse();
        blade.Body.ResetPosition(Vector3d.Zero, startRotation);
        blade.Body.Agent.Transform.LocalRotation = endRotation;
        source.AddAngularImpulse(Fixed64.FromFraction(1, 64) / source.EffectiveInverseMomentOfInertia);
        Fixed64 unconstrainedRotation = source.AngularVelocity;
        source.LinearVelocity.Should().Be(Vector2d.Zero);

        context.LateSimulate();

        if (mixedContacts)
        {
            source.LinearVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero,
                "the discrete mixed blade crosses the stationary source only between its endpoint poses");
            source.Position.Should().NotBe(sourceStart);
        }
        else
        {
            source.LinearVelocity.Should().Be(Vector2d.Zero);
            source.Position.Should().Be(sourceStart);
            source.Rotation.Should().Be(unconstrainedRotation);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_CenteredCircleTurningAlongNonRotatingMixedGeometry_ShouldReachHostPose(
        bool centeredTargetSpins)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateBody(context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)) { Material = PhysicsMaterial.Frictionless },
            Vector2d.Zero, immovable: false, isKinematic: true);
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        LSCollider targetCollider = centeredTargetSpins
            ? new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) }
            : new LSCuboidCollider { Size = new Vector3d(1, 1, 2) };
        targetCollider.Material = PhysicsMaterial.Frictionless;
        Vector3d targetStart = -Vector3d.Right
            * (centeredTargetSpins ? Fixed64.Quarter : Fixed64.FromFraction(5, 8));
        ScenarioBody<LSCollider> target = scenario.CreateBody(targetCollider, targetStart, FixedQuaternion.Identity);
        target.Body.UseManualGrounding();
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        if (centeredTargetSpins)
        {
            target.Body.AddAngularImpulse(Vector3d.Up
                * (Fixed64.FromFraction(1, 64) / target.Body.EffectiveInverseInertiaTensor.M22));
            target.Body.AngularVelocity.Y.Should().BeGreaterThan(Fixed64.Zero);
        }
        Vector3d targetAngularVelocity = target.Body.AngularVelocity;
        Vector2d requestedPosition = Vector2d.Forward * Fixed64.FromFraction(1, 1024);
        source.Agent.Transform.LocalPosition = requestedPosition.ToVector3d(Fixed64.Zero);
        source.Agent.Transform.LocalRotationXZRadians = FixedMath.DegToRad((Fixed64)5);
        Fixed64 requestedRotation = source.Agent.Transform.WorldRotationXZRadians;

        context.LateSimulate();

        source.Position.Should().Be(requestedPosition);
        source.Rotation.Should().Be(requestedRotation);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Body.Position3d.Should().Be(targetStart);
        target.Body.AngularVelocity.Should().Be(targetAngularVelocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalMixedCandidateFallback_WithConservativeRadius_ShouldAdmitOnlyRotating3DBodies(
        bool targetRotates)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        SolidBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero, immovable: false, isKinematic: true);
        source.HasNearbyRotationalContinuousCollisionTarget(
            Vector2d.Zero, Vector2d.Zero, Fixed64.MaxValue).Should().BeFalse();
        _ = scenario.CreateStaticSphere(Vector3d.Right * (Fixed64)2);
        source.HasNearbyRotationalContinuousCollisionTarget(
            Vector2d.Zero, Vector2d.Zero, Fixed64.MaxValue).Should().BeFalse();
        ScenarioBody<LSCuboidCollider> target = scenario.CreateBody(new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Half)
        }, Vector3d.Right * (Fixed64)4, FixedQuaternion.Identity);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        if (targetRotates)
            target.Body.AddAngularImpulse(Vector3d.Up / target.Body.EffectiveInverseInertiaTensor.M22);

        // The query bound is deliberately conservative; registered geometry remains ordinary and valid.
        source.HasNearbyRotationalContinuousCollisionTarget(
            Vector2d.Zero, Vector2d.Zero, Fixed64.MaxValue).Should().Be(targetRotates);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        source.HasNearbyRotationalContinuousCollisionTarget(
            Vector2d.Zero, Vector2d.Zero, Fixed64.MaxValue).Should().BeFalse();
    }
}
