//=======================================================================
// JointSolver3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;

namespace Gravitas.Constraints;

/// <summary>
/// Builds and solves spatial joint rows, applying cached impulses and recording solve metrics.
/// </summary>
internal static class JointSolver3D
{
    internal const int MaxRowsPerJoint = 12;
    // Fraction of structural error targeted by the next fixed step.
    private static readonly Fixed64 BiasFactor = Fixed64.FromFraction(2, 5);
    private static readonly Fixed64 RowEpsilon = Fixed64.Epsilon;
    // 4,096 raw Q32.32 units, approximately 9.536743e-7 radians.
    private static readonly Fixed64 QuaternionTwistVectorEpsilon = Fixed64.FromRaw(4_096);

    internal static void Solve(Joint3D joint, bool applyCachedImpulse, Fixed64 linearCorrectionRate)
    {
        Span<JointConstraintRow3D> rows = stackalloc JointConstraintRow3D[MaxRowsPerJoint];
        int rowCount = BuildRows(
            joint,
            rows,
            out Fixed64 linearAnchorErrorMagnitude,
            out Fixed64 angularLimitErrorMagnitude,
            out Fixed64 motorErrorMagnitude);
        if (!TryPrepareBiasVelocities(joint, rows[..rowCount], linearCorrectionRate))
        {
            joint.LastSolvedRowCount = 0;
            joint.LastSolveMetrics = default;
            return;
        }

        Fixed64 incrementalImpulseMagnitude = Fixed64.Zero;
        Fixed64 motorImpulseMagnitude = Fixed64.Zero;
        int clampedRowCount = 0;
        for (int i = 0; i < rowCount; i++)
        {
            JointConstraintRow3D row = rows[i];
            if (applyCachedImpulse)
            {
                row.AccumulatedImpulse = ApplyImpulse(joint.BodyA, joint.BodyB, row,
                    joint.GetCachedImpulse(row.CacheIndex));
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

    private static bool TryPrepareBiasVelocities(Joint3D joint, Span<JointConstraintRow3D> rows,
        Fixed64 linearCorrectionRate)
    {
        foreach (ref JointConstraintRow3D row in rows)
        {
            // Structural error is corrected per fixed step. Motor gains already target velocity.
            if (row.Kind != JointConstraintRowKind3D.Motor && !Fixed64.TryMultiplyDivide(
                    row.BiasVelocity,
                    row.Kind == JointConstraintRowKind3D.Linear ? linearCorrectionRate : joint.Context.InvDeltaTime,
                    Fixed64.One, out row.BiasVelocity))
            {
                if (ComputeDenominator(joint.BodyA, joint.BodyB, row) <= RowEpsilon)
                {
                    row.BiasVelocity = Fixed64.Zero;
                    continue;
                }
                GravitasLogger.Channel.Write(DiagnosticLevel.Error,
                    "A 3D joint's correction velocity cannot be represented.", nameof(JointSolver3D));
                return false;
            }
        }
        return true;
    }

    internal static bool TryGetLinearCorrectionDistance(Joint3D joint, out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (!TryPrepareAnchors(joint, out Vector3d relativeA, out Vector3d relativeB, out Vector3d error))
            return false;
        var row = new JointConstraintRow3D
        {
            Kind = JointConstraintRowKind3D.Linear,
            RelativeAnchorA = relativeA,
            RelativeAnchorB = relativeB
        };
        for (int axis = 0; axis < 3; axis++)
        {
            row.Axis = axis == 0 ? Vector3d.Right : axis == 1 ? Vector3d.Up : Vector3d.Forward;
            // Frozen, unresponsive errors cannot consume the connected island's repair budget.
            if (ComputeDenominator(joint.BodyA, joint.BodyB, row) <= RowEpsilon)
                error[axis] = Fixed64.Zero;
        }
        return Vector3d.TryGetMagnitude(error, out distance);
    }

    private static bool TryPrepareAnchors(Joint3D joint, out Vector3d relativeAnchorA,
        out Vector3d relativeAnchorB, out Vector3d anchorError)
    {
        relativeAnchorA = relativeAnchorB = anchorError = default;
        SolidBody bodyA = joint.BodyA, bodyB = joint.BodyB;
        return bodyA.Rotation.TryRotate(joint.LocalFrameA.Position, out Vector3d anchorOffsetA)
            && bodyB.Rotation.TryRotate(joint.LocalFrameB.Position, out Vector3d anchorOffsetB)
            && bodyA.TryGetOffsetFromCenterOfMass(new ContactAnchor(bodyA.Position3d, anchorOffsetA), out relativeAnchorA)
            && bodyB.TryGetOffsetFromCenterOfMass(new ContactAnchor(bodyB.Position3d, anchorOffsetB), out relativeAnchorB)
            && Vector3d.TrySubtractSums(bodyB.Position3d, anchorOffsetB, bodyA.Position3d, anchorOffsetA, out anchorError);
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
        if (!TryPrepareAnchors(joint, out Vector3d relativeAnchorA,
                out Vector3d relativeAnchorB, out Vector3d anchorError))
        {
            linearAnchorErrorMagnitude = Fixed64.Zero;
            GravitasLogger.Channel.Write(
                DiagnosticLevel.Error,
                "A 3D joint's anchors cannot be represented in response space.",
                nameof(JointSolver3D));
            return 0;
        }

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
                AddConeTwistRows(joint, rows, ref count, worldRotationA, worldRotationB, ref angularLimitErrorMagnitude);
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
        ref Fixed64 angularLimitErrorMagnitude)
    {
        Vector3d forwardA = (worldRotationA * Vector3d.Forward).Normalized;
        Vector3d forwardB = (worldRotationB * Vector3d.Forward).Normalized;
        if (joint.Limits.Kind != JointLimitKind3D.ConeTwist)
            return;

        Fixed64 dot = FixedMath.Clamp(Vector3d.Dot(forwardA, forwardB), -Fixed64.One, Fixed64.One);
        Fixed64 swing = FixedMath.Acos(dot);
        if (swing > joint.Limits.MaxConeAngle)
        {
            Vector3d swingAxis = Vector3d.Cross(forwardA, forwardB);
            if (swingAxis.MagnitudeSquared <= RowEpsilon)
                swingAxis = Perpendicular(forwardA);

            Fixed64 limitedError = swing - joint.Limits.MaxConeAngle;
            angularLimitErrorMagnitude += limitedError.Abs();
            AddAngularRow(rows, ref count, swingAxis.Normalized, limitedError, Fixed64.Zero, Fixed64.MaxValue);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, limitedError);
        }

        Fixed64 twist = GetSignedTwistAngle(worldRotationA, worldRotationB, forwardA);
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
        return FixedQuaternion.QuaternionLog(error);
    }

    private static Fixed64 GetSignedTwistAngle(
        FixedQuaternion worldRotationA,
        FixedQuaternion worldRotationB,
        Vector3d twistAxis)
    {
        FixedQuaternion relative = (worldRotationB * worldRotationA.Inverse()).Normalized;
        Vector3d relativeVector = new(relative.X, relative.Y, relative.Z);
        Fixed64 projectedTwist = Vector3d.Dot(relativeVector, twistAxis);
        if (projectedTwist.Abs() < QuaternionTwistVectorEpsilon
            && relative.W.Abs() < QuaternionTwistVectorEpsilon)
        {
            return Fixed64.Zero;
        }

        return NormalizeAngle(Fixed64.Two * FixedMath.Atan2(projectedTwist, relative.W));
    }

    private static Fixed64 NormalizeAngle(Fixed64 angle)
    {
        angle %= Fixed64.TwoPi;
        if (angle < -Fixed64.Pi)
            angle += Fixed64.TwoPi;
        else if (angle >= Fixed64.Pi)
            angle -= Fixed64.TwoPi;
        return angle;
    }

    private static Vector3d Perpendicular(Vector3d axis)
    {
        Vector3d candidate = Vector3d.Cross(axis, Vector3d.Up);
        return candidate.MagnitudeSquared > RowEpsilon
            ? candidate
            : Vector3d.Cross(axis, Vector3d.Right);
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

        return ApplyImpulse(bodyA, bodyB, row, lambda);
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

    private static Fixed64 ApplyImpulse(SolidBody bodyA, SolidBody bodyB, JointConstraintRow3D row, Fixed64 lambda)
    {
        if (lambda == Fixed64.Zero)
            return Fixed64.Zero;
        bool negative = lambda < Fixed64.Zero;
        if (!Fixed64.TryMultiplyDivide(lambda, negative ? -Fixed64.One : Fixed64.One, Fixed64.One, out Fixed64 magnitude))
            return Fixed64.Zero;

        // Limit the shared impulse, then prepare again from that exact scalar before publishing either body.
        for (int attempt = 0; attempt < 2; attempt++)
        {
            bool prepared = ContactResponseArithmetic3D.TryScale(row.Axis, lambda, out Vector3d impulse)
                & Vector3d.TrySubtract(Vector3d.Zero, impulse, out Vector3d opposite);
            prepared &= TryPrepareImpulseBody(bodyA, row, row.RelativeAnchorA, opposite, out Vector3d linearA, out Vector3d angularA)
                & TryPrepareImpulseBody(bodyB, row, row.RelativeAnchorB, impulse, out Vector3d linearB, out Vector3d angularB);
            if (!prepared)
                return Fixed64.Zero;

            Fixed64 limit = bodyA.Context.Environment.MaxSpeed;
            Fixed64 allowed = FixedMath.Min(
                FixedMath.Min(GetAllowedImpulseMagnitude(bodyA.LinearVelocity, linearA, limit, magnitude),
                    GetAllowedImpulseMagnitude(bodyB.LinearVelocity, linearB, limit, magnitude)),
                FixedMath.Min(GetAllowedImpulseMagnitude(bodyA.AngularVelocity, angularA, limit, magnitude),
                    GetAllowedImpulseMagnitude(bodyB.AngularVelocity, angularB, limit, magnitude)));
            if (allowed == magnitude)
            {
                bodyA.ApplyCollisionVelocityState(linearA, angularA);
                bodyB.ApplyCollisionVelocityState(linearB, angularB);
                return lambda;
            }
            if (allowed == Fixed64.Zero)
                return Fixed64.Zero;
            magnitude = allowed;
            lambda = negative ? -allowed : allowed;
        }
        return Fixed64.Zero;
    }

    private static bool TryPrepareImpulseBody(SolidBody body, JointConstraintRow3D row, Vector3d relativeAnchor,
        Vector3d impulse, out Vector3d linear, out Vector3d angular)
    {
        Vector3d linearDelta = default, torque = impulse;
        bool prepared = true;
        if (row.Kind == JointConstraintRowKind3D.Linear)
            prepared = ContactResponseArithmetic3D.TryScale(impulse, body.GetConstrainedInverseMass(row.Axis), out linearDelta)
                & Vector3d.TryCross(relativeAnchor, impulse, out torque);
        prepared &= Fixed3x3.TryTransformDirection(body.GetConstrainedInverseInertiaTensor(), torque, out Vector3d angularDelta);
        return prepared & body.TryPrepareCollisionVelocityState(linearDelta, angularDelta, out linear, out angular);
    }

    private static Fixed64 GetAllowedImpulseMagnitude(Vector3d current, Vector3d prepared, Fixed64 limit, Fixed64 magnitude)
    {
        // Match the body's rounded speed admission so publication cannot apply a different radial cap.
        if (current == prepared || prepared.Magnitude <= limit)
            return magnitude;
        var bound = new FixedBoundSphere(Vector3d.Zero, limit);
        var segment = new FixedSegment(current, prepared);
        if (!segment.TryGetSphereIntersectionDistanceInterval(bound, magnitude, out _, out Fixed64 exit))
            return Fixed64.Zero;
        // The interval rounds half to even. Retreat one impulse LSB and recheck the materialized endpoint.
        return exit > Fixed64.Zero ? exit - Fixed64.FromRaw(1) : Fixed64.Zero;
    }
}
