using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Query;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    private static (
        FixedQuaternion SourceRotation,
        Vector2d TargetPosition,
        Vector2d TargetLinearVelocity,
        Fixed64 TargetAngularVelocity) RunKinematic3DRotationalMixedPair(bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade;
        SolidBody2D target;
        if (targetFirst)
        {
            target = CreateRotationalMixedTarget2D(context);
            blade = CreateRotationalMixedBlade3D(context);
        }
        else
        {
            blade = CreateRotationalMixedBlade3D(context);
            target = CreateRotationalMixedTarget2D(context);
        }

        target.AddForce(Vector2d.Right * Fixed64.FromFraction(1, 20));
        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.LateSimulate();
        return (
            blade.Body.Rotation,
            target.Position,
            target.LinearVelocity,
            target.AngularVelocity);
    }

    private static (
        Vector3d SourcePosition,
        Vector3d SourceLinearVelocity,
        int SourceToiIterations)
        RunTranslationOnly3DSourceAgainstRotating2DTarget(
            bool targetFirst,
            ContinuousCollisionMode targetMode = ContinuousCollisionMode.Discrete)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        Fixed64 startRotation = -FixedMath.DegToRad((Fixed64)45);
        Fixed64 targetRotation = FixedMath.DegToRad((Fixed64)45);
        Vector3d sourcePosition = new(
            Fixed64.FromFraction(31, 10),
            Fixed64.Zero,
            Fixed64.FromFraction(-1, 10));
        SolidBody2D target;
        ScenarioBody<LSSphereCollider> source;
        if (targetFirst)
        {
            target = CreateRotationalMixedBlade2D(context);
            source = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                sourcePosition);
        }
        else
        {
            source = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                sourcePosition);
            target = CreateRotationalMixedBlade2D(context);
        }

        target.ResetPosition(Vector2d.Zero, startRotation);
        target.ContinuousCollisionMode = targetMode;
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d requestedVelocity = Vector3d.Forward * Fixed64.FromFraction(1, 5);

        target.Agent.Transform.LocalRotationXZRadians = targetRotation;
        source.Body.AddLinearImpulse(requestedVelocity);
        context.LateSimulate();

        return (
            source.Body.Position3d,
            source.Body.LinearVelocity,
            source.Body.LastContinuousCollisionToiIterationCount);
    }

    private static (
        Vector2d SourcePosition,
        Vector2d SourceLinearVelocity,
        int SourceToiIterations)
        RunTranslationOnly2DSourceAgainstRotating3DTarget(
            bool targetFirst,
            ContinuousCollisionMode targetMode = ContinuousCollisionMode.Discrete)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        FixedQuaternion startRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad((Fixed64)(-45)));
        FixedQuaternion targetRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad((Fixed64)45));
        Vector2d sourcePosition = new(
            Fixed64.FromFraction(31, 10),
            Fixed64.FromFraction(-1, 10));
        ScenarioBody<LSCuboidCollider> target;
        SolidBody2D source;
        if (targetFirst)
        {
            target = CreateRotationalMixedBlade3D(context);
            source = CreateMixedTranslationSource2D(context, sourcePosition);
        }
        else
        {
            source = CreateMixedTranslationSource2D(context, sourcePosition);
            target = CreateRotationalMixedBlade3D(context);
        }

        target.Body.ResetPosition(Vector3d.Zero, startRotation);
        target.Body.ContinuousCollisionMode = targetMode;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d requestedVelocity = Vector2d.Forward * Fixed64.FromFraction(1, 5);

        target.Body.Agent.Transform.LocalRotation = targetRotation;
        source.AddLinearImpulse(requestedVelocity);
        context.LateSimulate();

        return (
            source.Position,
            source.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount);
    }

    private static ChronicleHash RunKinematic3DRotationalMixedPairHash()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        SolidBody2D target = CreateRotationalMixedTarget2D(context);
        target.AddForce(Vector2d.Right * Fixed64.FromFraction(1, 20));
        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;

        context.LateSimulate();

        return context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);
    }

    private static (
        Fixed64 SourceRotation,
        Vector3d TargetPosition,
        Vector3d TargetLinearVelocity,
        Vector3d TargetAngularVelocity) RunKinematic2DRotationalMixedPair(bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D blade;
        ScenarioBody<LSSphereCollider> target;
        if (targetFirst)
        {
            target = CreateRotationalMixedTarget3D(context);
            blade = CreateRotationalMixedBlade2D(context);
        }
        else
        {
            blade = CreateRotationalMixedBlade2D(context);
            target = CreateRotationalMixedTarget3D(context);
        }

        target.Body.AddForce(Vector3d.Right * Fixed64.FromFraction(1, 20));
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.LateSimulate();
        return (
            blade.Rotation,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            target.Body.AngularVelocity);
    }

    private static ChronicleHash RunKinematic2DRotationalMixedPairHash()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        ScenarioBody<LSSphereCollider> target = CreateRotationalMixedTarget3D(context);
        target.Body.AddForce(Vector3d.Right * Fixed64.FromFraction(1, 20));
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;

        context.LateSimulate();

        return context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);
    }

    private static (
        bool PreparedTranslation,
        bool PreparedRotation,
        int SourceToiIterations,
        Vector2d TargetPosition,
        Fixed64 TargetRotation,
        Vector2d TargetLinearVelocity,
        Fixed64 TargetAngularVelocity) RunCombined2DTargetTrajectory(bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        Vector2d targetPosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)45))
            * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero))
            .ToVector2d();
        ScenarioBody<LSCuboidCollider> blade;
        SolidBody2D target;
        if (targetFirst)
        {
            target = CreateRotationalRectangle2D(context, targetPosition);
            blade = CreateRotationalMixedBlade3D(context);
        }
        else
        {
            blade = CreateRotationalMixedBlade3D(context);
            target = CreateRotationalRectangle2D(context, targetPosition);
        }

        target.AddForce(Vector2d.Right * Fixed64.FromFraction(1, 20));
        target.AddAngularImpulse(
            FixedMath.DegToRad((Fixed64)10)
            / target.EffectiveInverseMomentOfInertia);
        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        bool preparedTranslation = target.ContinuousCollisionFrameStart
            != target.ContinuousCollisionFrameEnd;
        bool preparedRotation = target.ContinuousCollisionFrameRotation
            != target.ContinuousCollisionFrameTargetRotation;

        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            preparedTranslation,
            preparedRotation,
            blade.Body.LastContinuousCollisionToiIterationCount,
            target.Position,
            target.Rotation,
            target.LinearVelocity,
            target.AngularVelocity);
    }

    private static (
        bool PreparedTranslation,
        bool PreparedRotation,
        int SourceToiIterations,
        Vector3d TargetPosition,
        FixedQuaternion TargetRotation,
        Vector3d TargetLinearVelocity,
        Vector3d TargetAngularVelocity) RunCombined3DTargetTrajectory(bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        Vector2d targetPosition2D = Vector2d.Rotate(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        Vector3d targetPosition = targetPosition2D.ToVector3d(Fixed64.Zero);
        SolidBody2D blade;
        ScenarioBody<LSCuboidCollider> target;
        if (targetFirst)
        {
            target = CreateRotationalCuboid3D(context, targetPosition);
            blade = CreateRotationalMixedBlade2D(context);
        }
        else
        {
            blade = CreateRotationalMixedBlade2D(context);
            target = CreateRotationalCuboid3D(context, targetPosition);
        }

        target.Body.AddForce(Vector3d.Right * Fixed64.FromFraction(1, 20));
        target.Body.AddAngularImpulse(
            Vector3d.Up
            * (FixedMath.DegToRad((Fixed64)10)
                / target.Body.EffectiveInverseInertiaTensor.M22));
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        bool preparedTranslation = target.Body.ContinuousCollisionFrameStart
            != target.Body.ContinuousCollisionFrameEnd;
        bool preparedRotation = target.Body.ContinuousCollisionFrameRotation
            != target.Body.ContinuousCollisionFrameTargetRotation;

        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            preparedTranslation,
            preparedRotation,
            blade.LastContinuousCollisionToiIterationCount,
            target.Body.Position3d,
            target.Body.Rotation,
            target.Body.LinearVelocity,
            target.Body.AngularVelocity);
    }

    private static (int ToiIterations, FixedQuaternion Rotation, Vector3d AngularVelocity)
        RunAlternating3DRotationalContacts(bool registerMixedFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        ScenarioBody<LSCuboidCollider> blade = CreateBody3D(
            context,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Vector2d samePosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)15))
            * new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero))
            .ToVector2d();
        Vector2d mixedPosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)(-15)))
            * new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero))
            .ToVector2d();
        ScenarioBody<LSSphereCollider> sameTarget;
        SolidBody2D mixedTarget;
        if (registerMixedFirst)
        {
            mixedTarget = CreateCircle2D(context, mixedPosition, isKinematic: true);
            sameTarget = CreateSphere3D(context, samePosition.ToVector3d(Fixed64.Zero), isKinematic: true);
        }
        else
        {
            sameTarget = CreateSphere3D(context, samePosition.ToVector3d(Fixed64.Zero), isKinematic: true);
            mixedTarget = CreateCircle2D(context, mixedPosition, isKinematic: true);
        }

        sameTarget.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        mixedTarget.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Fixed64 requestedAngularVelocity = (Fixed64)4;
        blade.Body.AddAngularImpulse(
            Vector3d.Up
            * (requestedAngularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            blade.Body.LastContinuousCollisionToiIterationCount,
            blade.Body.Rotation,
            blade.Body.AngularVelocity);
    }

    private static (int ToiIterations, Fixed64 Rotation, Fixed64 AngularVelocity)
        RunAlternating2DRotationalContacts(bool registerMixedFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        blade.SetMotionType(BodyMotionType.Dynamic);
        blade.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Vector2d samePosition = Vector2d.Rotate(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)15));
        Vector2d mixedPosition = Vector2d.Rotate(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)(-15)));
        SolidBody2D sameTarget;
        ScenarioBody<LSSphereCollider> mixedTarget;
        if (registerMixedFirst)
        {
            mixedTarget = CreateSphere3D(
                context,
                mixedPosition.ToVector3d(Fixed64.Zero),
                isKinematic: true);
            sameTarget = CreateCircle2D(context, samePosition, isKinematic: true);
        }
        else
        {
            sameTarget = CreateCircle2D(context, samePosition, isKinematic: true);
            mixedTarget = CreateSphere3D(
                context,
                mixedPosition.ToVector3d(Fixed64.Zero),
                isKinematic: true);
        }

        sameTarget.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        mixedTarget.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Fixed64 requestedAngularVelocity = (Fixed64)2;
        blade.AddAngularImpulse(
            requestedAngularVelocity / blade.EffectiveInverseMomentOfInertia);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            blade.LastContinuousCollisionToiIterationCount,
            blade.Rotation,
            blade.AngularVelocity);
    }

    private static (
        Vector2d Target2DVelocity,
        Fixed64 Target2DAngularVelocity,
        Vector3d Target3DVelocity,
        Vector3d Target3DAngularVelocity,
        int SourceToiIterations)
        RunKinematic3DRotationalTie(bool registerMixedFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = CreateRotationalMixedBlade3D(context);
        Vector2d target2DPosition = (FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)45))
            * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero))
            .ToVector2d();
        Vector3d target3DPosition = FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)225))
            * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero);
        SolidBody2D target2D;
        ScenarioBody<LSSphereCollider> target3D;
        if (registerMixedFirst)
        {
            target2D = CreateRotationalMixedTarget2D(context, target2DPosition);
            target3D = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                target3DPosition);
        }
        else
        {
            target3D = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                target3DPosition);
            target2D = CreateRotationalMixedTarget2D(context, target2DPosition);
        }

        target2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target2D.Sleep();
        target3D.Body.Sleep();
        blade.Body.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            target2D.LinearVelocity,
            target2D.AngularVelocity,
            target3D.Body.LinearVelocity,
            target3D.Body.AngularVelocity,
            blade.Body.LastContinuousCollisionToiIterationCount);
    }

    private static (
        Vector2d Target2DVelocity,
        Fixed64 Target2DAngularVelocity,
        Vector3d Target3DVelocity,
        Vector3d Target3DAngularVelocity,
        int SourceToiIterations)
        RunKinematic2DRotationalTie(bool registerMixedFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        Vector2d target2DPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        Vector2d target3DPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)225));
        SolidBody2D target2D;
        ScenarioBody<LSSphereCollider> target3D;
        if (registerMixedFirst)
        {
            target3D = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                target3DPosition.ToVector3d(Fixed64.Zero));
            target2D = CreateRotationalMixedTarget2D(context, target2DPosition);
        }
        else
        {
            target2D = CreateRotationalMixedTarget2D(context, target2DPosition);
            target3D = CreateBody3D(
                context,
                new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
                target3DPosition.ToVector3d(Fixed64.Zero));
        }

        target2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target2D.Sleep();
        target3D.Body.Sleep();
        blade.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        blade.LateSimulate(updateSleepState: false, updateColliderState: false);

        return (
            target2D.LinearVelocity,
            target2D.AngularVelocity,
            target3D.Body.LinearVelocity,
            target3D.Body.AngularVelocity,
            blade.LastContinuousCollisionToiIterationCount);
    }

    private static ScenarioBody<LSCuboidCollider> CreateRotationalMixedBlade3D(
        GravitasWorldContext context)
    {
        ScenarioBody<LSCuboidCollider> blade = CreateBody3D(
            context,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            isKinematic: true);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return blade;
    }

    private static SolidBody2D CreateRotationalMixedBlade2D(GravitasWorldContext context)
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        var blade = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        blade.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return blade;
    }

    private static SolidBody2D CreateMixedTranslationSource2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.FromFraction(1, 4));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                position.ToVector3d(Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var source = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        source.Initialize(position);
        return source;
    }

    private static SolidBody2D CreateRotationalRectangle2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        Fixed64 halfWidth = Fixed64.FromFraction(1, 4);
        Fixed64 halfHeight = Fixed64.FromFraction(1, 8);
        var collider = new LSPolygonCollider2D(
            new Vector2d(-halfWidth, -halfHeight),
            new Vector2d(halfWidth, -halfHeight),
            new Vector2d(halfWidth, halfHeight),
            new Vector2d(-halfWidth, halfHeight));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                position.ToVector3d(Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var target = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        target.Initialize(position);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return target;
    }

    private static ScenarioBody<LSCuboidCollider> CreateRotationalCuboid3D(
        GravitasWorldContext context,
        Vector3d position)
    {
        ScenarioBody<LSCuboidCollider> target = CreateBody3D(
            context,
            new LSCuboidCollider
            {
                Size = new Vector3d(
                    Fixed64.Half,
                    Fixed64.Half,
                    Fixed64.FromFraction(1, 4))
            },
            position);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return target;
    }

    private static SolidBody2D CreateRotationalMixedTarget2D(
        GravitasWorldContext context,
        Vector2d? initialPosition = null)
    {
        Vector2d position = initialPosition
            ?? (FixedQuaternion.FromAxisAngle(
                    Vector3d.Up,
                    FixedMath.DegToRad((Fixed64)45))
                * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero))
                .ToVector2d();
        var collider = new LSCircleCollider2D(Fixed64.FromFraction(1, 4));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(position.X, Fixed64.Zero, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One));
        var target = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        target.Initialize(position);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return target;
    }

    private static ScenarioBody<LSSphereCollider> CreateRotationalMixedTarget3D(
        GravitasWorldContext context)
    {
        Vector3d position = Vector2d.Rotate(
                new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)45))
            .ToVector3d(Fixed64.Zero);
        ScenarioBody<LSSphereCollider> target = CreateBody3D(
            context,
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            position);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return target;
    }
}
