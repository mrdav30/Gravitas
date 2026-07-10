//=======================================================================
// JointSolver2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Constraints;

internal static class JointSolver2D
{
    internal const int MaxRowsPerJoint = 8;

    private const int CacheLinearX = 0;
    private const int CacheLinearY = 1;
    private const int CacheAngular = 2;
    private const int CacheLimit = 3;
    private const int CacheMotor = 4;

    private static readonly Fixed64 BiasFactor = Fixed64.One / (Fixed64)5;
    private static readonly Fixed64 RowEpsilon = Fixed64.Epsilon;

    internal static void Solve(Joint2D joint, bool applyCachedImpulse)
    {
        Span<JointConstraintRow2D> rows = stackalloc JointConstraintRow2D[MaxRowsPerJoint];
        int rowCount = BuildRows(
            joint,
            rows,
            out Fixed64 linearAnchorErrorMagnitude,
            out Fixed64 angularErrorMagnitude,
            out Fixed64 limitErrorMagnitude,
            out Fixed64 motorErrorMagnitude);
        if (rowCount == 0)
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
            JointConstraintRow2D row = rows[i];
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
            if (row.Kind == JointConstraintRowKind2D.Motor)
                motorImpulseMagnitude += rowImpulseMagnitude;
            if (clamped)
                clampedRowCount++;
        }

        Fixed64 impulseMagnitude = Fixed64.Zero;
        for (int i = 0; i < rowCount; i++)
            impulseMagnitude += rows[i].AccumulatedImpulse.Abs();

        for (int i = 0; i < MaxRowsPerJoint; i++)
        {
            bool rowPrepared = false;
            for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
            {
                if (rows[rowIndex].CacheIndex == i)
                {
                    rowPrepared = true;
                    break;
                }
            }

            if (!rowPrepared)
                joint.SetCachedImpulse(i, Fixed64.Zero);
        }

        joint.LastSolvedRowCount = rowCount;
        joint.AccumulatedImpulseMagnitude += incrementalImpulseMagnitude;
        joint.LastSolveMetrics = new JointSolveMetrics2D(
            rowCount,
            linearAnchorErrorMagnitude,
            angularErrorMagnitude,
            limitErrorMagnitude,
            impulseMagnitude,
            incrementalImpulseMagnitude,
            motorImpulseMagnitude,
            motorErrorMagnitude,
            clampedRowCount);
        if (incrementalImpulseMagnitude > Fixed64.Zero)
            joint.Context.Diagnostics.EmitJointImpulse(joint, joint.LastSolveMetrics);
    }

    private static int BuildRows(
        Joint2D joint,
        Span<JointConstraintRow2D> rows,
        out Fixed64 linearAnchorErrorMagnitude,
        out Fixed64 angularErrorMagnitude,
        out Fixed64 limitErrorMagnitude,
        out Fixed64 motorErrorMagnitude)
    {
        int count = 0;
        limitErrorMagnitude = Fixed64.Zero;
        motorErrorMagnitude = Fixed64.Zero;

        SolidBody2D bodyA = joint.BodyA;
        SolidBody2D bodyB = joint.BodyB;
        Fixed64 frameAngleA = bodyA.Rotation + joint.LocalFrameA.Angle;
        Fixed64 frameAngleB = bodyB.Rotation + joint.LocalFrameB.Angle;
        Vector2d anchorA = bodyA.Position + Vector2d.Rotate(joint.LocalFrameA.Anchor, bodyA.Rotation);
        Vector2d anchorB = bodyB.Position + Vector2d.Rotate(joint.LocalFrameB.Anchor, bodyB.Rotation);
        Vector2d relativeAnchorA = anchorA - bodyA.WorldCenterOfMass;
        Vector2d relativeAnchorB = anchorB - bodyB.WorldCenterOfMass;
        Vector2d anchorError = anchorB - anchorA;
        linearAnchorErrorMagnitude = anchorError.Magnitude;
        angularErrorMagnitude = NormalizeAngle(frameAngleB - frameAngleA).Abs();

        switch (joint.Type)
        {
            case JointType2D.Distance:
                AddDistanceRow(joint, rows, ref count, anchorError, bodyB.Position - bodyA.Position, frameAngleA, relativeAnchorA, relativeAnchorB);
                break;
            case JointType2D.Pin:
                AddPlanarAnchorRows(rows, ref count, anchorError, relativeAnchorA, relativeAnchorB);
                AddAngularLimitRow(joint, rows, ref count, frameAngleA, frameAngleB, ref limitErrorMagnitude);
                break;
            case JointType2D.Weld:
                AddPlanarAnchorRows(rows, ref count, anchorError, relativeAnchorA, relativeAnchorB);
                AddAngularErrorRow(rows, ref count, NormalizeAngle(frameAngleB - frameAngleA), Fixed64.Zero, Fixed64.MaxValue, CacheAngular);
                AddAngularLimitRow(joint, rows, ref count, frameAngleA, frameAngleB, ref limitErrorMagnitude);
                break;
            case JointType2D.Prismatic:
                AddPrismaticRows(joint, rows, ref count, anchorError, relativeAnchorA, relativeAnchorB, frameAngleA, frameAngleB, ref limitErrorMagnitude);
                break;
        }

        AddMotorRow(joint, rows, ref count, anchorError, frameAngleA, frameAngleB, relativeAnchorA, relativeAnchorB, out motorErrorMagnitude);
        return count;
    }

    private static void AddDistanceRow(
        Joint2D joint,
        Span<JointConstraintRow2D> rows,
        ref int count,
        Vector2d anchorError,
        Vector2d bodyDelta,
        Fixed64 frameAngleA,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB)
    {
        Fixed64 magnitudeSquared = anchorError.MagnitudeSquared;
        Fixed64 targetDistance = joint.Limits.TargetDistance;
        if (magnitudeSquared <= RowEpsilon && targetDistance <= RowEpsilon)
            return;

        Fixed64 distance = magnitudeSquared > RowEpsilon ? FixedMath.Sqrt(magnitudeSquared) : Fixed64.Zero;
        Vector2d axis;
        if (distance > RowEpsilon)
        {
            axis = anchorError / distance;
        }
        else if (bodyDelta.MagnitudeSquared > RowEpsilon)
        {
            axis = bodyDelta.Normalized;
        }
        else
        {
            axis = Vector2d.Rotate(Vector2d.Right, frameAngleA).Normalized;
        }

        AddLinearRow(
            rows,
            ref count,
            axis,
            relativeAnchorA,
            relativeAnchorB,
            (distance - targetDistance) * BiasFactor,
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            CacheLinearX);
    }

    private static void AddPlanarAnchorRows(
        Span<JointConstraintRow2D> rows,
        ref int count,
        Vector2d anchorError,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB)
    {
        AddLinearRow(
            rows,
            ref count,
            Vector2d.Right,
            relativeAnchorA,
            relativeAnchorB,
            Vector2d.Dot(anchorError, Vector2d.Right) * BiasFactor,
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            CacheLinearX);
        AddLinearRow(
            rows,
            ref count,
            Vector2d.Forward,
            relativeAnchorA,
            relativeAnchorB,
            Vector2d.Dot(anchorError, Vector2d.Forward) * BiasFactor,
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            CacheLinearY);
    }

    private static void AddPrismaticRows(
        Joint2D joint,
        Span<JointConstraintRow2D> rows,
        ref int count,
        Vector2d anchorError,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB,
        Fixed64 frameAngleA,
        Fixed64 frameAngleB,
        ref Fixed64 limitErrorMagnitude)
    {
        Vector2d axis = Vector2d.Rotate(Vector2d.Right, frameAngleA).Normalized;
        Vector2d normal = Perpendicular(axis);
        AddLinearRow(
            rows,
            ref count,
            normal,
            relativeAnchorA,
            relativeAnchorB,
            Vector2d.Dot(anchorError, normal) * BiasFactor,
            Fixed64.Zero,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            CacheLinearX);

        AddAngularErrorRow(rows, ref count, NormalizeAngle(frameAngleB - frameAngleA), Fixed64.Zero, Fixed64.MaxValue, CacheAngular);

        if (joint.Limits.Kind != JointLimitKind2D.Slider)
            return;

        Fixed64 translation = Vector2d.Dot(anchorError, axis);
        if (translation < joint.Limits.MinTranslation)
        {
            Fixed64 error = translation - joint.Limits.MinTranslation;
            limitErrorMagnitude += error.Abs();
            AddLinearRow(
                rows,
                ref count,
                axis,
                relativeAnchorA,
                relativeAnchorB,
                error * BiasFactor,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                CacheLimit);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, error);
        }
        else if (translation > joint.Limits.MaxTranslation)
        {
            Fixed64 error = translation - joint.Limits.MaxTranslation;
            limitErrorMagnitude += error.Abs();
            AddLinearRow(
                rows,
                ref count,
                axis,
                relativeAnchorA,
                relativeAnchorB,
                error * BiasFactor,
                Fixed64.Zero,
                Fixed64.MinValue,
                Fixed64.Zero,
                CacheLimit);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, error);
        }
    }

    private static void AddAngularLimitRow(
        Joint2D joint,
        Span<JointConstraintRow2D> rows,
        ref int count,
        Fixed64 frameAngleA,
        Fixed64 frameAngleB,
        ref Fixed64 limitErrorMagnitude)
    {
        if (joint.Limits.Kind != JointLimitKind2D.Angular)
            return;

        Fixed64 angle = NormalizeAngle(frameAngleB - frameAngleA);
        if (angle < joint.Limits.MinAngle)
        {
            Fixed64 error = angle - joint.Limits.MinAngle;
            limitErrorMagnitude += error.Abs();
            AddAngularRow(
                rows,
                ref count,
                error * BiasFactor,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                CacheLimit);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, error);
        }
        else if (angle > joint.Limits.MaxAngle)
        {
            Fixed64 error = angle - joint.Limits.MaxAngle;
            limitErrorMagnitude += error.Abs();
            AddAngularRow(
                rows,
                ref count,
                error * BiasFactor,
                Fixed64.Zero,
                Fixed64.MinValue,
                Fixed64.Zero,
                CacheLimit);
            joint.Context.Diagnostics.EmitJointLimitReached(joint, error);
        }
    }

    private static void AddMotorRow(
        Joint2D joint,
        Span<JointConstraintRow2D> rows,
        ref int count,
        Vector2d anchorError,
        Fixed64 frameAngleA,
        Fixed64 frameAngleB,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB,
        out Fixed64 motorErrorMagnitude)
    {
        motorErrorMagnitude = Fixed64.Zero;
        JointMotor2D motor = joint.Motor;
        if (!motor.IsEnabled)
            return;

        if (motor.Kind == JointMotorKind2D.Angular)
        {
            Fixed64 error = NormalizeAngle(frameAngleB - frameAngleA - motor.Target);
            motorErrorMagnitude = error.Abs();
            AddAngularRow(
                rows,
                ref count,
                error * motor.DriveStrength,
                motor.Damping,
                -motor.MaximumMotorImpulse,
                motor.MaximumMotorImpulse,
                CacheMotor,
                JointConstraintRowKind2D.Motor);
            return;
        }

        if (joint.Type != JointType2D.Prismatic)
            return;

        Vector2d axis = Vector2d.Rotate(Vector2d.Right, frameAngleA).Normalized;
        Fixed64 errorTranslation = Vector2d.Dot(anchorError, axis) - motor.Target;
        motorErrorMagnitude = errorTranslation.Abs();
        AddLinearRow(
            rows,
            ref count,
            axis,
            relativeAnchorA,
            relativeAnchorB,
            errorTranslation * motor.DriveStrength,
            motor.Damping,
            -motor.MaximumMotorImpulse,
            motor.MaximumMotorImpulse,
            CacheMotor,
            JointConstraintRowKind2D.Motor);
    }

    private static void AddAngularErrorRow(
        Span<JointConstraintRow2D> rows,
        ref int count,
        Fixed64 error,
        Fixed64 damping,
        Fixed64 maxImpulse,
        int cacheIndex)
    {
        AddAngularRow(
            rows,
            ref count,
            error * BiasFactor,
            damping,
            -maxImpulse,
            maxImpulse,
            cacheIndex);
    }

    private static void AddLinearRow(
        Span<JointConstraintRow2D> rows,
        ref int count,
        Vector2d axis,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB,
        Fixed64 biasVelocity,
        Fixed64 damping,
        Fixed64 lowerImpulse,
        Fixed64 upperImpulse,
        int cacheIndex,
        JointConstraintRowKind2D kind = JointConstraintRowKind2D.Linear)
    {
        if (axis.MagnitudeSquared <= RowEpsilon)
            return;

        Vector2d normalizedAxis = axis.MagnitudeSquared == Fixed64.One
            ? axis
            : axis.Normalized;
        rows[count] = new JointConstraintRow2D(
            kind,
            normalizedAxis,
            relativeAnchorA,
            relativeAnchorB,
            biasVelocity,
            damping,
            lowerImpulse,
            upperImpulse,
            cacheIndex);
        count++;
    }

    private static void AddAngularRow(
        Span<JointConstraintRow2D> rows,
        ref int count,
        Fixed64 biasVelocity,
        Fixed64 damping,
        Fixed64 lowerImpulse,
        Fixed64 upperImpulse,
        int cacheIndex,
        JointConstraintRowKind2D kind = JointConstraintRowKind2D.Angular)
    {
        rows[count] = new JointConstraintRow2D(
            kind,
            Vector2d.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
            biasVelocity,
            damping,
            lowerImpulse,
            upperImpulse,
            cacheIndex);
        count++;
    }

    private static Fixed64 SolveRow(
        SolidBody2D bodyA,
        SolidBody2D bodyB,
        JointConstraintRow2D row,
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

    private static Fixed64 ComputeRelativeVelocity(SolidBody2D bodyA, SolidBody2D bodyB, JointConstraintRow2D row)
    {
        if (row.Kind == JointConstraintRowKind2D.Linear || row.Axis != Vector2d.Zero)
        {
            Vector2d velocityA = bodyA.LinearVelocity + AngularVelocityAtPoint(bodyA.AngularVelocity, row.RelativeAnchorA);
            Vector2d velocityB = bodyB.LinearVelocity + AngularVelocityAtPoint(bodyB.AngularVelocity, row.RelativeAnchorB);
            return Vector2d.Dot(velocityB - velocityA, row.Axis);
        }

        return bodyB.AngularVelocity - bodyA.AngularVelocity;
    }

    private static Fixed64 ComputeDenominator(SolidBody2D bodyA, SolidBody2D bodyB, JointConstraintRow2D row)
    {
        if (row.Kind != JointConstraintRowKind2D.Linear && row.Axis == Vector2d.Zero)
            return bodyA.EffectiveInverseMomentOfInertia + bodyB.EffectiveInverseMomentOfInertia;

        Fixed64 inverseMass = bodyA.GetConstrainedInverseMass(row.Axis) + bodyB.GetConstrainedInverseMass(row.Axis);
        Fixed64 torqueA = Cross(row.RelativeAnchorA, row.Axis);
        Fixed64 torqueB = Cross(row.RelativeAnchorB, row.Axis);
        return inverseMass
            + torqueA * torqueA * bodyA.EffectiveInverseMomentOfInertia
            + torqueB * torqueB * bodyB.EffectiveInverseMomentOfInertia;
    }

    private static void ApplyImpulse(SolidBody2D bodyA, SolidBody2D bodyB, JointConstraintRow2D row, Fixed64 lambda)
    {
        if (lambda == Fixed64.Zero)
            return;

        if (row.Kind != JointConstraintRowKind2D.Linear && row.Axis == Vector2d.Zero)
        {
            bodyA.ApplyCollisionAngularVelocityDelta(-lambda * bodyA.EffectiveInverseMomentOfInertia);
            bodyB.ApplyCollisionAngularVelocityDelta(lambda * bodyB.EffectiveInverseMomentOfInertia);
            return;
        }

        Vector2d impulse = row.Axis * lambda;
        bodyA.ApplyCollisionLinearVelocityDelta(-impulse * bodyA.GetConstrainedInverseMass(row.Axis));
        bodyB.ApplyCollisionLinearVelocityDelta(impulse * bodyB.GetConstrainedInverseMass(row.Axis));

        Fixed64 angularA = Cross(row.RelativeAnchorA, -impulse) * bodyA.EffectiveInverseMomentOfInertia;
        Fixed64 angularB = Cross(row.RelativeAnchorB, impulse) * bodyB.EffectiveInverseMomentOfInertia;
        bodyA.ApplyCollisionAngularVelocityDelta(angularA);
        bodyB.ApplyCollisionAngularVelocityDelta(angularB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Fixed64 angularVelocity, Vector2d relativeAnchor) =>
        new(-angularVelocity * relativeAnchor.Y, angularVelocity * relativeAnchor.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d Perpendicular(Vector2d vector) => new(-vector.Y, vector.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d left, Vector2d right) => left.X * right.Y - left.Y * right.X;

    private static Fixed64 NormalizeAngle(Fixed64 angle)
    {
        angle %= Fixed64.TwoPi;
        if (angle < -Fixed64.Pi)
            angle += Fixed64.TwoPi;
        else if (angle > Fixed64.Pi)
            angle -= Fixed64.TwoPi;
        return angle;
    }
}
