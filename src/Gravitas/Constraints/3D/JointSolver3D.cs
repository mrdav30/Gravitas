//=======================================================================
// JointSolver3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;

namespace Gravitas.Constraints;

internal static class JointSolver3D
{
    internal const int MaxRowsPerJoint = 12;
    private static readonly Fixed64 BiasFactor = Fixed64.One / (Fixed64)5;
    private static readonly Fixed64 RowEpsilon = Fixed64.Epsilon;
    private static readonly Fixed64 QuaternionLogVectorEpsilon = Fixed64.FromRaw(0x00001000L);

    internal static void Solve(Joint3D joint, bool applyCachedImpulse)
    {
        Span<JointConstraintRow3D> rows = stackalloc JointConstraintRow3D[MaxRowsPerJoint];
        int rowCount = BuildRows(
            joint,
            rows,
            out Fixed64 linearAnchorErrorMagnitude,
            out Fixed64 angularLimitErrorMagnitude,
            out Fixed64 motorErrorMagnitude);

        Fixed64 incrementalImpulseMagnitude = Fixed64.Zero;
        Fixed64 motorImpulseMagnitude = Fixed64.Zero;
        int clampedRowCount = 0;
        for (int i = 0; i < rowCount; i++)
        {
            JointConstraintRow3D row = rows[i];
            if (applyCachedImpulse)
            {
                row.AccumulatedImpulse = joint.GetCachedImpulse(row.CacheIndex);
                ApplyImpulse(joint.BodyA, joint.BodyB, row, row.AccumulatedImpulse);
            }

            Fixed64 impulse = SolveRow(joint.BodyA, joint.BodyB, row, out bool clamped);
            row.AccumulatedImpulse += impulse;
            rows[i] = row;
            joint.SetCachedImpulse(row.CacheIndex, row.AccumulatedImpulse);
            Fixed64 rowImpulseMagnitude = impulse.Abs();
            incrementalImpulseMagnitude += rowImpulseMagnitude;
            if (row.Kind == JointConstraintRowKind3D.Motor)
                motorImpulseMagnitude += rowImpulseMagnitude;
            if (clamped)
                clampedRowCount++;
        }

        Fixed64 impulseMagnitude = Fixed64.Zero;
        for (int i = 0; i < rowCount; i++)
            impulseMagnitude += rows[i].AccumulatedImpulse.Abs();

        for (int i = rowCount; i < MaxRowsPerJoint; i++)
            joint.SetCachedImpulse(i, Fixed64.Zero);

        joint.LastSolvedRowCount = rowCount;
        joint.AccumulatedImpulseMagnitude += incrementalImpulseMagnitude;
        joint.LastSolveMetrics = new JointSolveMetrics3D(
            rowCount,
            linearAnchorErrorMagnitude,
            angularLimitErrorMagnitude,
            impulseMagnitude,
            incrementalImpulseMagnitude,
            motorImpulseMagnitude,
            motorErrorMagnitude,
            clampedRowCount);
        if (incrementalImpulseMagnitude > Fixed64.Zero)
            joint.Context.Diagnostics.EmitJointImpulse(joint, joint.LastSolveMetrics);
    }

    private static int BuildRows(
        Joint3D joint,
        Span<JointConstraintRow3D> rows,
        out Fixed64 linearAnchorErrorMagnitude,
        out Fixed64 angularLimitErrorMagnitude,
        out Fixed64 motorErrorMagnitude)
    {
        int count = 0;
        angularLimitErrorMagnitude = Fixed64.Zero;
        motorErrorMagnitude = Fixed64.Zero;
        SolidBody bodyA = joint.BodyA;
        SolidBody bodyB = joint.BodyB;

        FixedQuaternion worldRotationA = (bodyA.Rotation * joint.LocalFrameA.Rotation).Normalized;
        FixedQuaternion worldRotationB = (bodyB.Rotation * joint.LocalFrameB.Rotation).Normalized;
        Vector3d anchorA = bodyA.Position3d + bodyA.Rotation * joint.LocalFrameA.Position;
        Vector3d anchorB = bodyB.Position3d + bodyB.Rotation * joint.LocalFrameB.Position;
        Vector3d relativeAnchorA = anchorA - bodyA.WorldCenterOfMass;
        Vector3d relativeAnchorB = anchorB - bodyB.WorldCenterOfMass;
        Vector3d anchorError = anchorB - anchorA;
        linearAnchorErrorMagnitude = anchorError.Magnitude;

        AddLinearAnchorRow(rows, ref count, Vector3d.Right, anchorError, relativeAnchorA, relativeAnchorB);
        AddLinearAnchorRow(rows, ref count, Vector3d.Up, anchorError, relativeAnchorA, relativeAnchorB);
        AddLinearAnchorRow(rows, ref count, Vector3d.Forward, anchorError, relativeAnchorA, relativeAnchorB);

        AddAngularRows(joint, rows, ref count, worldRotationA, worldRotationB, ref angularLimitErrorMagnitude);
        AddMotorRows(joint, rows, ref count, worldRotationA, worldRotationB, out motorErrorMagnitude);
        return count;
    }

    private static void AddLinearAnchorRow(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d axis,
        Vector3d anchorError,
        Vector3d relativeAnchorA,
        Vector3d relativeAnchorB)
    {
        Fixed64 error = Vector3d.Dot(anchorError, axis);
        rows[count] = new JointConstraintRow3D(
            JointConstraintRowKind3D.Linear,
            axis,
            relativeAnchorA,
            relativeAnchorB,
            error * BiasFactor,
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            count);
        count++;
    }

    private static void AddAngularRows(
        Joint3D joint,
        Span<JointConstraintRow3D> rows,
        ref int count,
        FixedQuaternion worldRotationA,
        FixedQuaternion worldRotationB,
        ref Fixed64 angularLimitErrorMagnitude)
    {
        Vector3d error = GetAngularError(worldRotationA, worldRotationB);
        switch (joint.Type)
        {
            case JointType3D.Fixed:
                AddAngularErrorRows(rows, ref count, error, Fixed64.Zero, Fixed64.MaxValue);
                break;
            case JointType3D.Hinge:
                AddAxisAlignmentRows(rows, ref count, worldRotationA * Vector3d.Right, worldRotationB * Vector3d.Right);
                AddHingeLimitRow(joint, rows, ref count, error, worldRotationA * Vector3d.Right, ref angularLimitErrorMagnitude);
                break;
            case JointType3D.ConeTwist:
                AddConeTwistRows(joint, rows, ref count, worldRotationA, worldRotationB, error, ref angularLimitErrorMagnitude);
                break;
        }
    }

    private static void AddMotorRows(
        Joint3D joint,
        Span<JointConstraintRow3D> rows,
        ref int count,
        FixedQuaternion worldRotationA,
        FixedQuaternion worldRotationB,
        out Fixed64 motorErrorMagnitude)
    {
        motorErrorMagnitude = Fixed64.Zero;
        JointMotor3D motor = joint.Motor;
        if (!motor.IsEnabled)
            return;

        FixedQuaternion targetWorldB = (worldRotationA * motor.TargetLocalRotation).Normalized;
        Vector3d motorError = GetAngularError(targetWorldB, worldRotationB);
        motorErrorMagnitude = motorError.Magnitude;
        AddMotorErrorRows(rows, ref count, motorError, motor.AngularDriveStrength, motor.AngularDriveDamping, motor.MaximumMotorImpulse);
    }

    private static void AddAxisAlignmentRows(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d axisA,
        Vector3d axisB)
    {
        Vector3d cross = Vector3d.Cross(axisA.Normalized, axisB.Normalized);
        Fixed64 magnitude = cross.Magnitude;
        if (magnitude <= RowEpsilon)
            return;

        AddAngularRow(rows, ref count, cross / magnitude, magnitude, Fixed64.Zero, Fixed64.MaxValue);
    }

    private static void AddHingeLimitRow(
        Joint3D joint,
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d angularError,
        Vector3d hingeAxis,
        ref Fixed64 angularLimitErrorMagnitude)
    {
        if (joint.Limits.Kind != JointLimitKind3D.Hinge)
            return;

        Vector3d axis = hingeAxis.Normalized;
        Fixed64 twist = Vector3d.Dot(angularError, axis);
        Fixed64 max = joint.Limits.MaxHingeAngle;
        if (twist.Abs() <= max)
            return;

        Fixed64 limitedError = twist > Fixed64.Zero ? twist - max : twist + max;
        angularLimitErrorMagnitude += limitedError.Abs();
        AddAngularRow(rows, ref count, axis, limitedError, Fixed64.Zero, Fixed64.MaxValue);
        joint.Context.Diagnostics.EmitJointLimitReached(joint, limitedError);
    }

    private static void AddConeTwistRows(
        Joint3D joint,
        Span<JointConstraintRow3D> rows,
        ref int count,
        FixedQuaternion worldRotationA,
        FixedQuaternion worldRotationB,
        Vector3d angularError,
        ref Fixed64 angularLimitErrorMagnitude)
    {
        Vector3d forwardA = (worldRotationA * Vector3d.Forward).Normalized;
        Vector3d forwardB = (worldRotationB * Vector3d.Forward).Normalized;
        AddAxisAlignmentRows(rows, ref count, forwardA, forwardB);

        if (joint.Limits.Kind != JointLimitKind3D.ConeTwist)
            return;

        Fixed64 dot = FixedMath.Clamp(Vector3d.Dot(forwardA, forwardB), -Fixed64.One, Fixed64.One);
        Fixed64 swing = FixedMath.Acos(dot);
        if (swing > joint.Limits.MaxConeAngle)
        {
            Vector3d swingAxis = Vector3d.Cross(forwardA, forwardB);
            if (swingAxis.MagnitudeSquared > RowEpsilon)
            {
                Fixed64 limitedError = swing - joint.Limits.MaxConeAngle;
                angularLimitErrorMagnitude += limitedError.Abs();
                AddAngularRow(rows, ref count, swingAxis.Normalized, limitedError, Fixed64.Zero, Fixed64.MaxValue);
                joint.Context.Diagnostics.EmitJointLimitReached(joint, limitedError);
            }
        }

        Fixed64 twist = Vector3d.Dot(angularError, forwardA);
        Fixed64 maxTwist = joint.Limits.MaxTwistAngle;
        if (twist.Abs() > maxTwist)
        {
            Fixed64 limitedError = twist > Fixed64.Zero ? twist - maxTwist : twist + maxTwist;
            angularLimitErrorMagnitude += limitedError.Abs();
            AddAngularRow(rows, ref count, forwardA, limitedError, Fixed64.Zero, Fixed64.MaxValue);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, limitedError);
        }
    }

    private static void AddAngularErrorRows(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d error,
        Fixed64 damping,
        Fixed64 maxImpulse)
    {
        AddAngularRow(rows, ref count, Vector3d.Right, Vector3d.Dot(error, Vector3d.Right), damping, maxImpulse);
        AddAngularRow(rows, ref count, Vector3d.Up, Vector3d.Dot(error, Vector3d.Up), damping, maxImpulse);
        AddAngularRow(rows, ref count, Vector3d.Forward, Vector3d.Dot(error, Vector3d.Forward), damping, maxImpulse);
    }

    private static void AddMotorErrorRows(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d error,
        Fixed64 strength,
        Fixed64 damping,
        Fixed64 maxImpulse)
    {
        AddMotorRow(rows, ref count, Vector3d.Right, Vector3d.Dot(error, Vector3d.Right) * strength, damping, maxImpulse);
        AddMotorRow(rows, ref count, Vector3d.Up, Vector3d.Dot(error, Vector3d.Up) * strength, damping, maxImpulse);
        AddMotorRow(rows, ref count, Vector3d.Forward, Vector3d.Dot(error, Vector3d.Forward) * strength, damping, maxImpulse);
    }

    private static void AddAngularRow(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d axis,
        Fixed64 error,
        Fixed64 damping,
        Fixed64 maxImpulse)
    {
        if (error.Abs() <= RowEpsilon || axis.MagnitudeSquared <= RowEpsilon)
            return;

        rows[count] = new JointConstraintRow3D(
            JointConstraintRowKind3D.Angular,
            axis.Normalized,
            Vector3d.Zero,
            Vector3d.Zero,
            error * BiasFactor,
            damping,
            -maxImpulse,
            maxImpulse,
            count);
        count++;
    }

    private static void AddMotorRow(
        Span<JointConstraintRow3D> rows,
        ref int count,
        Vector3d axis,
        Fixed64 biasVelocity,
        Fixed64 damping,
        Fixed64 maxImpulse)
    {
        if (biasVelocity.Abs() <= RowEpsilon)
            return;

        rows[count] = new JointConstraintRow3D(
            JointConstraintRowKind3D.Motor,
            axis,
            Vector3d.Zero,
            Vector3d.Zero,
            biasVelocity,
            damping,
            -maxImpulse,
            maxImpulse,
            count);
        count++;
    }

    private static Vector3d GetAngularError(FixedQuaternion reference, FixedQuaternion current)
    {
        FixedQuaternion error = (current * reference.Inverse()).Normalized;
        return GetSafeQuaternionLog(error);
    }

    private static Vector3d GetSafeQuaternionLog(FixedQuaternion quaternion)
    {
        FixedQuaternion normalized = quaternion.Normalized;
        Vector3d vector = new(normalized.X, normalized.Y, normalized.Z);
        Fixed64 vectorLength = vector.Magnitude;
        if (vectorLength < QuaternionLogVectorEpsilon)
            return Vector3d.Zero;

        Fixed64 w = FixedMath.Clamp(normalized.W, -Fixed64.One, Fixed64.One);
        Fixed64 theta = Fixed64.Two * FixedMath.Acos(w);
        return (vector / vectorLength) * theta;
    }

    private static Fixed64 SolveRow(
        SolidBody bodyA,
        SolidBody bodyB,
        JointConstraintRow3D row,
        out bool clampedToBounds)
    {
        clampedToBounds = false;
        Fixed64 denominator = ComputeDenominator(bodyA, bodyB, row);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 velocity = ComputeRelativeVelocity(bodyA, bodyB, row);
        Fixed64 lambda = -(velocity + row.BiasVelocity + velocity * row.Damping) / denominator;
        Fixed64 unclamped = row.AccumulatedImpulse + lambda;
        Fixed64 clamped = FixedMath.Clamp(unclamped, row.LowerImpulse, row.UpperImpulse);
        clampedToBounds = clamped != unclamped;
        lambda = clamped - row.AccumulatedImpulse;
        if (lambda == Fixed64.Zero)
            return Fixed64.Zero;

        ApplyImpulse(bodyA, bodyB, row, lambda);
        return lambda;
    }

    private static Fixed64 ComputeRelativeVelocity(SolidBody bodyA, SolidBody bodyB, JointConstraintRow3D row)
    {
        if (row.Kind == JointConstraintRowKind3D.Linear)
        {
            Vector3d velocityA = bodyA.LinearVelocity + Vector3d.Cross(bodyA.AngularVelocity, row.RelativeAnchorA);
            Vector3d velocityB = bodyB.LinearVelocity + Vector3d.Cross(bodyB.AngularVelocity, row.RelativeAnchorB);
            return Vector3d.Dot(velocityB - velocityA, row.Axis);
        }

        return Vector3d.Dot(bodyB.AngularVelocity - bodyA.AngularVelocity, row.Axis);
    }

    private static Fixed64 ComputeDenominator(SolidBody bodyA, SolidBody bodyB, JointConstraintRow3D row)
    {
        if (row.Kind != JointConstraintRowKind3D.Linear)
        {
            Vector3d angularA = bodyA.ApplyConstrainedInverseInertia(row.Axis);
            Vector3d angularB = bodyB.ApplyConstrainedInverseInertia(row.Axis);
            return Vector3d.Dot(row.Axis, angularA + angularB);
        }

        Fixed64 inverseMass = bodyA.GetConstrainedInverseMass(row.Axis) + bodyB.GetConstrainedInverseMass(row.Axis);
        Vector3d torqueA = Vector3d.Cross(row.RelativeAnchorA, row.Axis);
        Vector3d torqueB = Vector3d.Cross(row.RelativeAnchorB, row.Axis);
        Vector3d linearAngularA = bodyA.ApplyConstrainedInverseInertia(torqueA);
        Vector3d linearAngularB = bodyB.ApplyConstrainedInverseInertia(torqueB);
        return inverseMass
            + Vector3d.Dot(torqueA, linearAngularA)
            + Vector3d.Dot(torqueB, linearAngularB);
    }

    private static void ApplyImpulse(SolidBody bodyA, SolidBody bodyB, JointConstraintRow3D row, Fixed64 lambda)
    {
        if (lambda == Fixed64.Zero)
            return;

        Vector3d impulse = row.Axis * lambda;
        if (row.Kind == JointConstraintRowKind3D.Linear)
        {
            bodyA.ApplyCollisionLinearVelocityDelta(-impulse * bodyA.GetConstrainedInverseMass(row.Axis));
            bodyB.ApplyCollisionLinearVelocityDelta(impulse * bodyB.GetConstrainedInverseMass(row.Axis));

            Vector3d angularImpulseA = bodyA.ApplyConstrainedInverseInertia(Vector3d.Cross(row.RelativeAnchorA, -impulse));
            Vector3d angularImpulseB = bodyB.ApplyConstrainedInverseInertia(Vector3d.Cross(row.RelativeAnchorB, impulse));
            bodyA.ApplyCollisionAngularVelocityDelta(angularImpulseA);
            bodyB.ApplyCollisionAngularVelocityDelta(angularImpulseB);
            return;
        }

        bodyA.ApplyCollisionAngularVelocityDelta(bodyA.ApplyConstrainedInverseInertia(-impulse));
        bodyB.ApplyCollisionAngularVelocityDelta(bodyB.ApplyConstrainedInverseInertia(impulse));
    }
}
