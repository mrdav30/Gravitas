//=======================================================================
// Joint3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

namespace Gravitas.Constraints;

/// <summary>
/// Context-owned deterministic 3D joint runtime state.
/// </summary>
public sealed class Joint3D : IRecordable
{
    private readonly Fixed64[] _accumulatedImpulses = new Fixed64[JointSolver3D.MaxRowsPerJoint];
    private JointCollisionPolicy _collisionPolicy;
    private bool _isEnabled = true;

    internal Joint3D(
        GravitasConstraint3DService service,
        int id,
        JointDefinition3D definition,
        JointFrame3D localFrameA,
        JointFrame3D localFrameB)
    {
        Service = service;
        Context = service.Context;
        Id = id;
        BodyA = definition.BodyA;
        BodyB = definition.BodyB;
        LocalFrameA = localFrameA;
        LocalFrameB = localFrameB;
        Type = definition.Type;
        Limits = definition.Limits;
        Motor = definition.Motor;
        _collisionPolicy = definition.CollisionPolicy;
        IsActive = true;
    }

    internal GravitasConstraint3DService Service { get; }

    internal RagdollRuntime3D? OwningRagdoll { get; set; }

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
    public SolidBody BodyA { get; }

    /// <summary>
    /// Gets the second body linked by the joint.
    /// </summary>
    public SolidBody BodyB { get; }

    /// <summary>
    /// Gets the first local anchor frame.
    /// </summary>
    public JointFrame3D LocalFrameA { get; private set; }

    /// <summary>
    /// Gets the second local anchor frame.
    /// </summary>
    public JointFrame3D LocalFrameB { get; private set; }

    /// <summary>
    /// Gets the joint type.
    /// </summary>
    public JointType3D Type { get; private set; }

    /// <summary>
    /// Gets optional angular limit data.
    /// </summary>
    public JointLimit3D Limits { get; private set; }

    /// <summary>
    /// Gets optional motor data.
    /// </summary>
    public JointMotor3D Motor { get; private set; }

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
            SwiftThrowHelper.ThrowIfTrue(
                !IsActive,
                nameof(Joint3D),
                "Removed joints cannot mutate simulation state.");
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
    public JointSolveMetrics3D LastSolveMetrics { get; internal set; }

    /// <summary>
    /// Replaces the motor payload and wakes linked bodies.
    /// </summary>
    public void SetMotor(JointMotor3D motor)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !IsActive,
            nameof(Joint3D),
            "Removed joints cannot mutate simulation state.");
        motor.Validate();
        Motor = motor;
        ClearSolverCache();
        WakeBodies();
    }

    /// <summary>
    /// Disables the joint motor and wakes linked bodies.
    /// </summary>
    public void ClearMotor() => SetMotor(JointMotor3D.Disabled);

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

    internal static bool IsSolverBody(SolidBody body) =>
        body.Active && body.DynamicId >= 0 && body.HasSolverMobility;

    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("joint.3d", 1);
        writer.WriteInt32(Id);
        writer.WriteInt32(BodyA.DynamicId);
        writer.WriteInt32(BodyB.DynamicId);
        writer.WriteInt32(BodyA.Collider.Id);
        writer.WriteInt32(BodyB.Collider.Id);
        writer.WriteBool(IsActive);
        writer.WriteBool(IsEnabled);
        writer.WriteEnum(Type);
        writer.WriteEnum(Limits.Kind);
        writer.WriteFixed64(Limits.MaxHingeAngle);
        writer.WriteFixed64(Limits.MaxConeAngle);
        writer.WriteFixed64(Limits.MaxTwistAngle);
        writer.WriteQuaternion(Motor.TargetLocalRotation);
        writer.WriteFixed64(Motor.AngularDriveStrength);
        writer.WriteFixed64(Motor.AngularDriveDamping);
        writer.WriteFixed64(Motor.MaximumMotorImpulse);
        writer.WriteEnum(CollisionPolicy);
        writer.WriteVector3d(LocalFrameA.Position);
        writer.WriteQuaternion(LocalFrameA.Rotation);
        writer.WriteVector3d(LocalFrameB.Position);
        writer.WriteQuaternion(LocalFrameB.Rotation);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("joint.3d.caches", 1);
        writer.WriteInt32(LastSolvedRowCount);
        writer.WriteFixed64(AccumulatedImpulseMagnitude);
        writer.WriteInt32(LastSolveMetrics.PreparedRowCount);
        writer.WriteFixed64(LastSolveMetrics.LinearAnchorErrorMagnitude);
        writer.WriteFixed64(LastSolveMetrics.AngularLimitErrorMagnitude);
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
        SwiftThrowHelper.ThrowIfTrue(
            !IsActive,
            nameof(Joint3D),
            "Removed joints cannot participate in serialization.");
        bool isEnabled = IsEnabled;
        JointType3D type = Type;
        JointLimitKind3D limitKind = Limits.Kind;
        Fixed64 maxHingeAngle = Limits.MaxHingeAngle;
        Fixed64 maxConeAngle = Limits.MaxConeAngle;
        Fixed64 maxTwistAngle = Limits.MaxTwistAngle;
        FixedQuaternion motorTarget = Motor.TargetLocalRotation;
        Fixed64 motorStrength = Motor.AngularDriveStrength;
        Fixed64 motorDamping = Motor.AngularDriveDamping;
        Fixed64 maxMotorImpulse = Motor.MaximumMotorImpulse;
        JointCollisionPolicy collisionPolicy = CollisionPolicy;
        Vector3d frameAPosition = LocalFrameA.Position;
        FixedQuaternion frameARotation = LocalFrameA.Rotation;
        Vector3d frameBPosition = LocalFrameB.Position;
        FixedQuaternion frameBRotation = LocalFrameB.Rotation;

        RecordValues.Look(chronicler, ref isEnabled, "IsEnabled", true);
        RecordValues.Look(chronicler, ref type, "Type", JointType3D.BallSocket);
        RecordValues.Look(chronicler, ref limitKind, "LimitKind", JointLimitKind3D.Unrestricted);
        RecordValues.Look(chronicler, ref maxHingeAngle, "MaxHingeAngle");
        RecordValues.Look(chronicler, ref maxConeAngle, "MaxConeAngle");
        RecordValues.Look(chronicler, ref maxTwistAngle, "MaxTwistAngle");
        RecordValues.Look(chronicler, ref motorTarget, "MotorTarget");
        RecordValues.Look(chronicler, ref motorStrength, "MotorStrength");
        RecordValues.Look(chronicler, ref motorDamping, "MotorDamping");
        RecordValues.Look(chronicler, ref maxMotorImpulse, "MaxMotorImpulse");
        RecordValues.Look(chronicler, ref collisionPolicy, "CollisionPolicy", JointCollisionPolicy.SuppressLinked);
        RecordValues.Look(chronicler, ref frameAPosition, "LocalFrameAPosition");
        RecordValues.Look(chronicler, ref frameARotation, "LocalFrameARotation");
        RecordValues.Look(chronicler, ref frameBPosition, "LocalFrameBPosition");
        RecordValues.Look(chronicler, ref frameBRotation, "LocalFrameBRotation");

        if (chronicler.Mode != SerializationMode.Loading)
            return;

        SwiftThrowHelper.ThrowIfArgument(!IsSupportedType(type), nameof(Type), "Unsupported 3D joint type.");
        SwiftThrowHelper.ThrowIfArgument(!IsSupportedLimitKind(limitKind), nameof(Limits), "Unsupported joint limit kind.");
        SwiftThrowHelper.ThrowIfArgument(
            collisionPolicy != JointCollisionPolicy.SuppressLinked
                && collisionPolicy != JointCollisionPolicy.Collide,
            nameof(CollisionPolicy),
            "Unsupported joint collision policy.");

        JointLimit3D limits = limitKind switch
        {
            JointLimitKind3D.Hinge => JointLimit3D.Hinge(maxHingeAngle),
            JointLimitKind3D.ConeTwist => JointLimit3D.ConeTwist(maxConeAngle, maxTwistAngle),
            _ => JointLimit3D.Unrestricted
        };
        JointMotor3D motor = new(motorTarget, motorStrength, motorDamping, maxMotorImpulse);
        ValidatePayload(type, limits);

        Type = type;
        Limits = limits;
        Motor = motor;
        SetCollisionPolicyFromRecord(collisionPolicy);
        LocalFrameA = new JointFrame3D(frameAPosition, frameARotation);
        LocalFrameB = new JointFrame3D(frameBPosition, frameBRotation);
        if (_isEnabled == isEnabled)
            ClearSolverCache();
        else
            IsEnabled = isEnabled;
    }

    internal static bool IsSupportedType(JointType3D type) =>
        type == JointType3D.BallSocket
        || type == JointType3D.Hinge
        || type == JointType3D.ConeTwist
        || type == JointType3D.Fixed;

    internal static bool IsSupportedLimitKind(JointLimitKind3D kind) =>
        kind == JointLimitKind3D.Unrestricted
        || kind == JointLimitKind3D.Hinge
        || kind == JointLimitKind3D.ConeTwist;

    internal static void ValidatePayload(
        JointType3D type,
        in JointLimit3D limits)
    {
        SwiftThrowHelper.ThrowIfArgument(
            type == JointType3D.Hinge
                && limits.Kind != JointLimitKind3D.Unrestricted
                && limits.Kind != JointLimitKind3D.Hinge,
            nameof(limits),
            "Hinge joints accept only hinge limits.");
        SwiftThrowHelper.ThrowIfArgument(
            type == JointType3D.ConeTwist
                && limits.Kind != JointLimitKind3D.Unrestricted
                && limits.Kind != JointLimitKind3D.ConeTwist,
            nameof(limits),
            "Cone-twist joints accept only cone-twist limits.");
        SwiftThrowHelper.ThrowIfArgument(
            (type == JointType3D.BallSocket || type == JointType3D.Fixed)
                && limits.Kind != JointLimitKind3D.Unrestricted,
            nameof(limits),
            "Ball-socket and fixed joints do not accept angular limit payloads.");
    }
}
