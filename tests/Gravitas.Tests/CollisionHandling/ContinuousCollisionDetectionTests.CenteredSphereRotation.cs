using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    public void ContinuousMode_CenteredSphereTurningOnSupport_ShouldAcceptNonClosingPose(
        int translationUnits,
        bool nonuniformScale)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.InitializeStaticCollider(
            new LSCuboidCollider { Size = new Vector3d(4, 2, 4) },
            Vector3d.Down);
        Fixed64 supportHeight = Fixed64.FromFraction(nonuniformScale ? 3 : 1, 8);
        var collider = new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) };
        var agent = new TestMatterAgent(scenario.Context, new FixedTransform(
            Vector3d.Up * supportHeight,
            FixedQuaternion.Identity,
            nonuniformScale ? new Vector3d(3, 2, 1) : Vector3d.One));
        var source = new SolidBody(agent, collider);
        source.Initialize(Vector3d.Up * supportHeight, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source);
        Vector3d target = source.Position3d
            + Vector3d.Right * Fixed64.FromFraction(translationUnits, 1024);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi / (Fixed64)64);

        source.Agent.Transform.LocalPosition = target;
        source.Agent.Transform.LocalRotation = rotation;
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(target);
        source.Rotation.Should().Be(source.Agent.Transform.WorldRotation);
        source.Rotation.Should().Be(rotation);
        collider.ScaledRadius.Should().Be(supportHeight);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_CenteredSphereTurningDownIntoSupport_ShouldBlockClosingTranslation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.InitializeStaticCollider(
            new LSCuboidCollider { Size = new Vector3d(4, 2, 4) }, Vector3d.Down);
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) },
            Vector3d.Up * Fixed64.FromFraction(1, 8), FixedQuaternion.Identity, isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        Vector3d start = source.Body.Position3d;
        Vector3d target = start + Vector3d.Down * Fixed64.FromFraction(1, 64);
        source.Body.Agent.Transform.LocalPosition = target;
        source.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(10);

        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(start);
        source.Body.Position3d.Should().NotBe(target);
    }

    [Fact]
    public void ContinuousMode_CenteredSphereTurningAcrossSupportIntoWall_ShouldStopAtWall()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.InitializeStaticCollider(
            new LSCuboidCollider { Size = new Vector3d(4, 2, 4) }, Vector3d.Down);
        scenario.InitializeStaticCollider(
            new LSCuboidCollider { Size = new Vector3d(Fixed64.Quarter, (Fixed64)2, (Fixed64)2) },
            Vector3d.Right * Fixed64.Half);
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) },
            Vector3d.Up * Fixed64.FromFraction(1, 8), FixedQuaternion.Identity, isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        source.Body.Agent.Transform.LocalPosition = source.Body.Position3d + Vector3d.Right;
        source.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(10);

        scenario.Context.LateSimulate();

        // The wall's left face is 3/8, leaving the sphere's center at or before 1/4.
        source.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Body.Position3d.X.Should().BeLessThanOrEqualTo(Fixed64.Quarter);
        source.Body.Position3d.Y.Should().Be(Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void ContinuousMode_DynamicCenteredSphereTurningOnSupport_ShouldPreserveTangentialMotion()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.InitializeStaticCollider(new LSCuboidCollider
        {
            Size = new Vector3d(4, 2, 4), Material = PhysicsMaterial.Frictionless
        }, Vector3d.Down);
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(new LSSphereCollider
        {
            Radius = Fixed64.FromFraction(1, 8), Material = PhysicsMaterial.Frictionless
        }, Vector3d.Up * Fixed64.FromFraction(1, 8), FixedQuaternion.Identity);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        // Initialization already probed the floor; clear that cached support so
        // automatic body-origin snapping does not replace collider contact motion.
        source.Body.UseManualGrounding();
        Vector3d velocity = Vector3d.Right * Fixed64.FromFraction(1, 1024);
        Vector3d target = source.Body.Position3d + velocity;
        source.Body.AddLinearImpulse(velocity);
        source.Body.AddAngularImpulse(Vector3d.Up
            * (Fixed64.FromFraction(1, 64) / source.Body.EffectiveInverseInertiaTensor.M22));
        Vector3d angularVelocity = source.Body.AngularVelocity;

        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(target);
        source.Body.LinearVelocity.Should().Be(velocity);
        source.Body.AngularVelocity.Should().Be(angularVelocity);
        source.Body.Rotation.Should().NotBe(FixedQuaternion.Identity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_TurningCenteredSphere_ShouldStillBeHitByNearbyRotatingBlade(bool sourceFirst)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var sourceCollider = new LSSphereCollider { Radius = Fixed64.Quarter };
        var bladeCollider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
        };
        var sourceStart = new Vector3d(Fixed64.FromFraction(31, 10), Fixed64.Zero, Fixed64.FromFraction(-1, 10));
        ScenarioBody<LSSphereCollider> source;
        ScenarioBody<LSCuboidCollider> blade;
        if (sourceFirst)
        {
            source = scenario.CreateBody(sourceCollider, sourceStart, FixedQuaternion.Identity);
            blade = scenario.CreateBody(bladeCollider, Vector3d.Zero, PhysicsScenarioBuilder.Yaw(-45), isKinematic: true);
        }
        else
        {
            blade = scenario.CreateBody(bladeCollider, Vector3d.Zero, PhysicsScenarioBuilder.Yaw(-45), isKinematic: true);
            source = scenario.CreateBody(sourceCollider, sourceStart, FixedQuaternion.Identity);
        }

        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(blade.Body);
        source.Body.AddAngularImpulse(Vector3d.Up
            * (Fixed64.FromFraction(1, 64) / source.Body.EffectiveInverseInertiaTensor.M22));
        blade.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(45);

        scenario.Context.LateSimulate();

        // Neither endpoint overlaps; the moving blade must transfer actual motion at an intermediate pose.
        source.Body.LinearVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        source.Body.Position3d.Should().NotBe(sourceStart);
    }

    [Fact]
    public void ContinuousMode_CenteredSphereTurningOnSupportNearSeparatedRotatingBlade_ShouldReachHostPose()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.InitializeStaticCollider(
            new LSCuboidCollider { Size = new Vector3d(4, 2, 4) }, Vector3d.Down);
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5)) },
            Vector3d.Up, PhysicsScenarioBuilder.Yaw(-5), isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) },
            Vector3d.Up * Fixed64.FromFraction(1, 8), FixedQuaternion.Identity, isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(blade.Body);
        Vector3d target = source.Body.Position3d + Vector3d.Right * Fixed64.FromFraction(1, 1024);
        FixedQuaternion sourceRotation = PhysicsScenarioBuilder.Yaw(5);
        blade.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(5);
        source.Body.Agent.Transform.LocalPosition = target;
        source.Body.Agent.Transform.LocalRotation = sourceRotation;

        scenario.Context.LateSimulate();

        // The blade bottom stays at 1/2; the sphere top stays at 1/4 throughout the frame.
        source.Body.Position3d.Should().Be(target);
        source.Body.Rotation.Should().Be(sourceRotation);
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, true)]
    [InlineData(false, false)]
    public void ContinuousMode_SpinningCenteredSphere_ShouldRespectMixedModeForDiscreteBlade(
        bool sourceFirst,
        bool mixedContacts)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.RuntimeMode = mixedContacts ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.Both;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var sourceStart = new Vector3d(
            Fixed64.FromFraction(31, 10), Fixed64.Zero, Fixed64.FromFraction(-1, 10));
        var sourceCollider = new LSSphereCollider { Radius = Fixed64.Quarter };
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var blade = new SolidBody2D(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), bladeCollider);
        Fixed64 startRotation = -FixedMath.DegToRad((Fixed64)45);
        Fixed64 endRotation = -startRotation;
        ScenarioBody<LSSphereCollider> source;
        if (sourceFirst)
        {
            source = scenario.CreateBody(sourceCollider, sourceStart, FixedQuaternion.Identity);
            blade.Initialize(Vector2d.Zero, startRotation, BodyMotionType.Kinematic);
        }
        else
        {
            blade.Initialize(Vector2d.Zero, startRotation, BodyMotionType.Kinematic);
            source = scenario.CreateBody(sourceCollider, sourceStart, FixedQuaternion.Identity);
        }

        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        CollisionDetectionMixed.TryCollide(sourceCollider, bladeCollider, out _).Should().BeFalse();
        blade.SetRotation(Fixed64.Zero);
        CollisionDetectionMixed.TryCollide(sourceCollider, bladeCollider, out _).Should().BeTrue();
        blade.SetRotation(endRotation);
        CollisionDetectionMixed.TryCollide(sourceCollider, bladeCollider, out _).Should().BeFalse();
        blade.ResetPosition(Vector2d.Zero, startRotation);
        blade.Agent.Transform.LocalRotationXZRadians = endRotation;
        source.Body.AddAngularImpulse(Vector3d.Up
            * (Fixed64.FromFraction(1, 64) / source.Body.EffectiveInverseInertiaTensor.M22));
        FixedQuaternion unconstrainedRotation = new FixedQuaternion(
            Fixed64.Zero, source.Body.AngularVelocity.Y * Fixed64.Half, Fixed64.Zero, Fixed64.One).Normalized;
        source.Body.LinearVelocity.Should().Be(Vector3d.Zero);

        scenario.Context.LateSimulate();

        if (mixedContacts)
        {
            source.Body.LinearVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero,
                "the discrete mixed blade crosses the stationary source only between its endpoint poses");
            source.Body.Position3d.Should().NotBe(sourceStart);
        }
        else
        {
            source.Body.LinearVelocity.Should().Be(Vector3d.Zero);
            source.Body.Position3d.Should().Be(sourceStart);
            source.Body.Rotation.Should().Be(unconstrainedRotation);
            source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_CenteredSphereTurningAlongNonRotatingMixedGeometry_ShouldReachHostPose(
        bool centeredTargetSpins)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(new LSSphereCollider
        {
            Radius = Fixed64.FromFraction(1, 8), Material = PhysicsMaterial.Frictionless
        }, Vector3d.Zero, FixedQuaternion.Identity, isKinematic: true);
        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        LSCollider2D targetCollider = centeredTargetSpins
            ? new LSCircleCollider2D(Fixed64.FromFraction(1, 8))
            : new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.FromFraction(-1, 8), -Fixed64.One),
                new Vector2d(Fixed64.FromFraction(-1, 8), Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One));
        targetCollider.Material = PhysicsMaterial.Frictionless;
        Vector2d targetStart = centeredTargetSpins ? -Vector2d.Right * Fixed64.Quarter : Vector2d.Zero;
        var target = new SolidBody2D(new TestMatterAgent(scenario.Context,
            new FixedTransform(targetStart.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), targetCollider)
        {
            Mass = Fixed64.One
        };
        target.Initialize(targetStart);
        target.UseManualGrounding();
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        if (centeredTargetSpins)
        {
            target.AddAngularImpulse(Fixed64.FromFraction(1, 64) / target.EffectiveInverseMomentOfInertia);
            target.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
        }
        Fixed64 targetAngularVelocity = target.AngularVelocity;
        Vector3d requestedPosition = Vector3d.Forward * Fixed64.FromFraction(1, 1024);
        FixedQuaternion requestedRotation = PhysicsScenarioBuilder.Yaw(5);
        source.Body.Agent.Transform.LocalPosition = requestedPosition;
        source.Body.Agent.Transform.LocalRotation = requestedRotation;

        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(requestedPosition);
        source.Body.Rotation.Should().Be(requestedRotation);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(targetStart);
        target.AngularVelocity.Should().Be(targetAngularVelocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalMixedCandidateFallback_WithConservativeRadius_ShouldAdmitOnlyRotating2DBodies(
        bool targetRotates)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
            Vector3d.Zero, Vector3d.Zero, Fixed64.MaxValue).Should().BeFalse();
        var bodyless = new LSCircleCollider2D(Fixed64.Half);
        bodyless.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Right * (Fixed64)2, FixedQuaternion.Identity, Vector3d.One)));
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
            Vector3d.Zero, Vector3d.Zero, Fixed64.MaxValue).Should().BeFalse();
        var targetCollider = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.Quarter),
            new Vector2d(Fixed64.One, -Fixed64.Quarter),
            new Vector2d(Fixed64.One, Fixed64.Quarter),
            new Vector2d(-Fixed64.One, Fixed64.Quarter));
        var target = new SolidBody2D(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Right * (Fixed64)4, FixedQuaternion.Identity, Vector3d.One)), targetCollider)
        {
            Mass = Fixed64.One
        };
        target.Initialize(Vector2d.Right * (Fixed64)4);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        if (targetRotates)
            target.AddAngularImpulse(Fixed64.One / target.EffectiveInverseMomentOfInertia);

        // The query bound is deliberately conservative; registered geometry remains ordinary and valid.
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
            Vector3d.Zero, Vector3d.Zero, Fixed64.MaxValue).Should().Be(targetRotates);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
            Vector3d.Zero, Vector3d.Zero, Fixed64.MaxValue).Should().BeFalse();
    }
}
