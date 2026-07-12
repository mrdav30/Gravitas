//=======================================================================
// Joint2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

namespace Gravitas.Constraints;

/// <summary>
/// Context-owned deterministic pure 2D joint runtime state.
/// </summary>
public sealed class Joint2D : IRecordable
{
    private readonly Fixed64[] _accumulatedImpulses = new Fixed64[JointSolver2D.MaxRowsPerJoint];
    private JointCollisionPolicy _collisionPolicy;
    private bool _isEnabled = true;

    internal Joint2D(
        GravitasConstraint2DService service,
        int id,
        in JointDefinition2D definition)
    {
        Service = service;
        Context = service.Context;
        Id = id;
        BodyA = definition.BodyA;
        BodyB = definition.BodyB;
        LocalFrameA = definition.LocalFrameA;
        LocalFrameB = definition.LocalFrameB;
        Type = definition.Type;
        Limits = ResolveRegistrationLimits(definition);
        Motor = definition.Motor;
        _collisionPolicy = definition.CollisionPolicy;
        IsActive = true;
    }

    internal GravitasConstraint2DService Service { get; }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context { get; }

    /// <summary>
    /// Gets this context-local joint ID.
    /// </summary>
    public int Id { get; }

    /// <summary>
    /// Gets the first body linked by the joint.
    /// </summary>
    public SolidBody2D BodyA { get; }

    /// <summary>
    /// Gets the second body linked by the joint.
    /// </summary>
    public SolidBody2D BodyB { get; }

    /// <summary>
    /// Gets the first local anchor frame.
    /// </summary>
    public JointFrame2D LocalFrameA { get; private set; }

    /// <summary>
    /// Gets the second local anchor frame.
    /// </summary>
    public JointFrame2D LocalFrameB { get; private set; }

    /// <summary>
    /// Gets the joint type.
    /// </summary>
    public JointType2D Type { get; private set; }

    /// <summary>
    /// Gets optional scalar limit data.
    /// </summary>
    public JointLimit2D Limits { get; private set; }

    /// <summary>
    /// Gets optional scalar motor data.
    /// </summary>
    public JointMotor2D Motor { get; private set; }

    /// <summary>
    /// Gets physical collision filtering behavior for the linked colliders.
    /// </summary>
    public JointCollisionPolicy CollisionPolicy => _collisionPolicy;

    /// <summary>
    /// Gets whether this joint is registered with its owning service.
    /// </summary>
    public bool IsActive { get; internal set; }

    /// <summary>
    /// Gets or sets whether this active joint emits solver rows.
    /// </summary>
    public bool IsEnabled
    {
        get => _isEnabled;
        set
        {
            if (_isEnabled == value)
                return;

            bool oldValue = _isEnabled;
            Service.UpdateJointEnabledState(this, oldValue, value);
            _isEnabled = value;
            ClearSolverCache();
            WakeBodies();
        }
    }

    /// <summary>
    /// Gets the number of rows emitted by the most recent solver pass.
    /// </summary>
    public int LastSolvedRowCount { get; internal set; }

    /// <summary>
    /// Gets the cumulative absolute incremental impulse emitted since the
    /// solver cache was last cleared.
    /// </summary>
    public Fixed64 AccumulatedImpulseMagnitude { get; internal set; }

    /// <summary>
    /// Gets deterministic metrics from the most recent solver pass.
    /// </summary>
    public JointSolveMetrics2D LastSolveMetrics { get; internal set; }

    /// <summary>
    /// Replaces the motor payload and wakes linked bodies.
    /// </summary>
    public void SetMotor(JointMotor2D motor)
    {
        motor.Validate();
        ValidatePayload(Type, Limits, motor);
        Motor = motor;
        ClearSolverCache();
        WakeBodies();
    }

    /// <summary>
    /// Disables the joint motor and wakes linked bodies.
    /// </summary>
    public void ClearMotor() => SetMotor(JointMotor2D.Disabled);

    internal Fixed64 GetCachedImpulse(int rowIndex) => _accumulatedImpulses[rowIndex];

    internal void SetCachedImpulse(int rowIndex, Fixed64 impulse) => _accumulatedImpulses[rowIndex] = impulse;

    internal void ClearSolverCache()
    {
        for (int i = 0; i < _accumulatedImpulses.Length; i++)
            _accumulatedImpulses[i] = Fixed64.Zero;

        LastSolvedRowCount = 0;
        AccumulatedImpulseMagnitude = Fixed64.Zero;
        LastSolveMetrics = default;
    }

    internal void WakeBodies()
    {
        BodyA.Wake();
        BodyB.Wake();
    }

    internal void SetCollisionPolicyFromRecord(JointCollisionPolicy collisionPolicy)
    {
        if (_collisionPolicy == collisionPolicy)
            return;

        Service.UpdateJointCollisionPolicy(this, _collisionPolicy, collisionPolicy);
        _collisionPolicy = collisionPolicy;
    }

    internal bool HasSolverParticipant() => IsSolverBody(BodyA) || IsSolverBody(BodyB);

    internal static bool IsSolverBody(SolidBody2D body) => body.Active && body.DynamicId >= 0 && body.CanTranslate;

    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("joint.2d", 1);
        writer.WriteInt32(Id);
        writer.WriteInt32(BodyA.DynamicId);
        writer.WriteInt32(BodyB.DynamicId);
        writer.WriteInt32(BodyA.Collider.Id);
        writer.WriteInt32(BodyB.Collider.Id);
        writer.WriteBool(IsActive);
        writer.WriteBool(IsEnabled);
        writer.WriteEnum(Type);
        writer.WriteEnum(Limits.Kind);
        writer.WriteFixed64(Limits.TargetDistance);
        writer.WriteFixed64(Limits.MinTranslation);
        writer.WriteFixed64(Limits.MaxTranslation);
        writer.WriteFixed64(Limits.MinAngle);
        writer.WriteFixed64(Limits.MaxAngle);
        writer.WriteEnum(Motor.Kind);
        writer.WriteFixed64(Motor.Target);
        writer.WriteFixed64(Motor.DriveStrength);
        writer.WriteFixed64(Motor.Damping);
        writer.WriteFixed64(Motor.MaximumMotorImpulse);
        writer.WriteEnum(CollisionPolicy);
        writer.WriteVector2d(LocalFrameA.Anchor);
        writer.WriteFixed64(LocalFrameA.Angle);
        writer.WriteVector2d(LocalFrameB.Anchor);
        writer.WriteFixed64(LocalFrameB.Angle);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("joint.2d.caches", 1);
        writer.WriteInt32(LastSolvedRowCount);
        writer.WriteFixed64(AccumulatedImpulseMagnitude);
        writer.WriteInt32(LastSolveMetrics.PreparedRowCount);
        writer.WriteFixed64(LastSolveMetrics.LinearAnchorErrorMagnitude);
        writer.WriteFixed64(LastSolveMetrics.AngularErrorMagnitude);
        writer.WriteFixed64(LastSolveMetrics.LimitErrorMagnitude);
        writer.WriteFixed64(LastSolveMetrics.AccumulatedImpulseMagnitude);
        writer.WriteFixed64(LastSolveMetrics.IncrementalImpulseMagnitude);
        writer.WriteFixed64(LastSolveMetrics.MotorImpulseMagnitude);
        writer.WriteFixed64(LastSolveMetrics.MotorErrorMagnitude);
        writer.WriteInt32(LastSolveMetrics.ClampedRowCount);
        for (int i = 0; i < _accumulatedImpulses.Length; i++)
            writer.WriteFixed64(_accumulatedImpulses[i]);
    }

    /// <summary>
    /// Records mutable joint state into a Chronicler pass.
    /// </summary>
    public void RecordData(IChronicler chronicler)
    {
        bool isEnabled = IsEnabled;
        JointType2D type = Type;
        JointLimitKind2D limitKind = Limits.Kind;
        Fixed64 targetDistance = Limits.TargetDistance;
        Fixed64 minTranslation = Limits.MinTranslation;
        Fixed64 maxTranslation = Limits.MaxTranslation;
        Fixed64 minAngle = Limits.MinAngle;
        Fixed64 maxAngle = Limits.MaxAngle;
        JointMotorKind2D motorKind = Motor.Kind;
        Fixed64 motorTarget = Motor.Target;
        Fixed64 motorStrength = Motor.DriveStrength;
        Fixed64 motorDamping = Motor.Damping;
        Fixed64 maxMotorImpulse = Motor.MaximumMotorImpulse;
        JointCollisionPolicy collisionPolicy = CollisionPolicy;
        Vector2d frameAAnchor = LocalFrameA.Anchor;
        Fixed64 frameAAngle = LocalFrameA.Angle;
        Vector2d frameBAnchor = LocalFrameB.Anchor;
        Fixed64 frameBAngle = LocalFrameB.Angle;

        RecordValues.Look(chronicler, ref isEnabled, "IsEnabled", true);
        RecordValues.Look(chronicler, ref type, "Type", JointType2D.Pin);
        RecordValues.Look(chronicler, ref limitKind, "LimitKind", JointLimitKind2D.Unrestricted);
        RecordValues.Look(chronicler, ref targetDistance, "TargetDistance");
        RecordValues.Look(chronicler, ref minTranslation, "MinTranslation");
        RecordValues.Look(chronicler, ref maxTranslation, "MaxTranslation");
        RecordValues.Look(chronicler, ref minAngle, "MinAngle");
        RecordValues.Look(chronicler, ref maxAngle, "MaxAngle");
        RecordValues.Look(chronicler, ref motorKind, "MotorKind", JointMotorKind2D.Disabled);
        RecordValues.Look(chronicler, ref motorTarget, "MotorTarget");
        RecordValues.Look(chronicler, ref motorStrength, "MotorStrength");
        RecordValues.Look(chronicler, ref motorDamping, "MotorDamping");
        RecordValues.Look(chronicler, ref maxMotorImpulse, "MaxMotorImpulse");
        RecordValues.Look(chronicler, ref collisionPolicy, "CollisionPolicy", JointCollisionPolicy.SuppressLinked);
        RecordValues.Look(chronicler, ref frameAAnchor, "LocalFrameAAnchor");
        RecordValues.Look(chronicler, ref frameAAngle, "LocalFrameAAngle");
        RecordValues.Look(chronicler, ref frameBAnchor, "LocalFrameBAnchor");
        RecordValues.Look(chronicler, ref frameBAngle, "LocalFrameBAngle");

        if (chronicler.Mode != SerializationMode.Loading)
            return;

        SwiftThrowHelper.ThrowIfArgument(!IsSupportedType(type), nameof(Type), "Unsupported 2D joint type.");
        SwiftThrowHelper.ThrowIfArgument(!IsSupportedLimitKind(limitKind), nameof(Limits), "Unsupported 2D joint limit kind.");
        SwiftThrowHelper.ThrowIfArgument(!IsSupportedMotorKind(motorKind), nameof(Motor), "Unsupported 2D joint motor kind.");
        SwiftThrowHelper.ThrowIfArgument(
            collisionPolicy != JointCollisionPolicy.SuppressLinked
                && collisionPolicy != JointCollisionPolicy.Collide,
            nameof(CollisionPolicy),
            "Unsupported joint collision policy.");

        JointLimit2D limits = limitKind switch
        {
            JointLimitKind2D.Distance => JointLimit2D.Distance(targetDistance),
            JointLimitKind2D.Slider => JointLimit2D.Slider(minTranslation, maxTranslation),
            JointLimitKind2D.Angular => JointLimit2D.Angular(minAngle, maxAngle),
            _ => JointLimit2D.Unrestricted
        };
        JointFrame2D localFrameA = new(frameAAnchor, frameAAngle);
        JointFrame2D localFrameB = new(frameBAnchor, frameBAngle);
        if (type == JointType2D.Distance && limits.Kind == JointLimitKind2D.Unrestricted)
            limits = JointLimit2D.Distance(ResolveCurrentAnchorDistance(BodyA, BodyB, localFrameA, localFrameB));

        JointMotor2D motor = motorKind switch
        {
            JointMotorKind2D.Angular => JointMotor2D.Angular(motorTarget, motorStrength, motorDamping, maxMotorImpulse),
            JointMotorKind2D.Linear => JointMotor2D.Linear(motorTarget, motorStrength, motorDamping, maxMotorImpulse),
            _ => JointMotor2D.Disabled
        };
        ValidatePayload(type, limits, motor);

        Type = type;
        Limits = limits;
        Motor = motor;
        SetCollisionPolicyFromRecord(collisionPolicy);
        LocalFrameA = localFrameA;
        LocalFrameB = localFrameB;
        if (_isEnabled == isEnabled)
            ClearSolverCache();
        else
            IsEnabled = isEnabled;
    }

    internal static bool IsSupportedType(JointType2D type) =>
        type == JointType2D.Distance
        || type == JointType2D.Pin
        || type == JointType2D.Weld
        || type == JointType2D.Prismatic;

    internal static bool IsSupportedLimitKind(JointLimitKind2D kind) =>
        kind == JointLimitKind2D.Unrestricted
        || kind == JointLimitKind2D.Distance
        || kind == JointLimitKind2D.Slider
        || kind == JointLimitKind2D.Angular;

    internal static bool IsSupportedMotorKind(JointMotorKind2D kind) =>
        kind == JointMotorKind2D.Disabled
        || kind == JointMotorKind2D.Angular
        || kind == JointMotorKind2D.Linear;

    internal static void ValidatePayload(
        JointType2D type,
        in JointLimit2D limits,
        in JointMotor2D motor)
    {
        SwiftThrowHelper.ThrowIfArgument(
            type == JointType2D.Distance
                && limits.Kind != JointLimitKind2D.Unrestricted
                && limits.Kind != JointLimitKind2D.Distance,
            nameof(limits),
            "Distance joints accept only distance limits.");
        SwiftThrowHelper.ThrowIfArgument(
            type == JointType2D.Prismatic
                && limits.Kind != JointLimitKind2D.Unrestricted
                && limits.Kind != JointLimitKind2D.Slider,
            nameof(limits),
            "Prismatic joints accept only slider limits.");
        SwiftThrowHelper.ThrowIfArgument(
            type != JointType2D.Prismatic
                && motor.Kind == JointMotorKind2D.Linear,
            nameof(motor),
            "Linear motors are supported only by prismatic joints.");
    }

    private static JointLimit2D ResolveRegistrationLimits(in JointDefinition2D definition)
    {
        if (definition.Type == JointType2D.Distance && definition.Limits.Kind == JointLimitKind2D.Unrestricted)
            return JointLimit2D.Distance(ResolveCurrentAnchorDistance(definition));

        return definition.Limits;
    }

    private static Fixed64 ResolveCurrentAnchorDistance(in JointDefinition2D definition)
    {
        return ResolveCurrentAnchorDistance(
            definition.BodyA,
            definition.BodyB,
            definition.LocalFrameA,
            definition.LocalFrameB);
    }

    private static Fixed64 ResolveCurrentAnchorDistance(
        SolidBody2D bodyA,
        SolidBody2D bodyB,
        in JointFrame2D localFrameA,
        in JointFrame2D localFrameB)
    {
        Vector2d anchorA = bodyA.Position + Vector2d.Rotate(localFrameA.Anchor, bodyA.Rotation);
        Vector2d anchorB = bodyB.Position + Vector2d.Rotate(localFrameB.Anchor, bodyB.Rotation);
        return (anchorB - anchorA).Magnitude;
    }
}
