//=======================================================================
// ContactNormalImpulseMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free mixed 3D/2D contact-normal impulse result.
/// </summary>
internal readonly struct ContactNormalImpulseResultMixed
{
    public ContactNormalImpulseResultMixed(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Vector3d LinearVelocityDelta3D { get; }

    public Vector3d AngularVelocityDelta3D { get; }

    public Vector2d LinearVelocityDelta2D { get; }

    public Fixed64 AngularVelocityDelta2D { get; }
}

/// <summary>
/// Side-effect-free mixed velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResultMixed
{
    public ContactNormalVelocityDeltaResultMixed(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
    }

    public Fixed64 NormalVelocity { get; }

    public Vector3d LinearVelocityDelta3D { get; }

    public Vector3d AngularVelocityDelta3D { get; }

    public Vector2d LinearVelocityDelta2D { get; }

    public Fixed64 AngularVelocityDelta2D { get; }
}

/// <summary>
/// Calculates allocation-free mixed 3D/2D contact-point normal response
/// without mutating either participant.
/// </summary>
internal static class ContactNormalImpulseMixed
{
    internal static bool TryCalculateVelocityDeltas(
        SolidBody? body3D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector3d relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResultMixed result)
    {
        result = default;
        Fixed64 normalVelocity = ComputeNormalVelocity(
            linearVelocity3D,
            angularVelocity3D,
            relativeContactPoint3D,
            linearVelocity2D,
            angularVelocity2D,
            relativeContactPoint2D,
            normal);
        if (normalVelocity >= Fixed64.Zero)
        {
            result = ZeroVelocityDelta(normalVelocity);
            return true;
        }

        Fixed64 denominator = ComputeDenominator(
            body3D,
            relativeContactPoint3D,
            body2D,
            relativeContactPoint2D,
            normal);
        if (denominator <= Fixed64.Zero)
            return false;

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        Vector2d planarNormal = normal.ToVector2d();
        bool linear3DResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                body3D?.ProjectLinearMotion(-normal) ?? Vector3d.Zero,
                normalVelocity,
                responseFactor,
                body3D?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector3d linearVelocityDelta3D);
        bool angular3DResolved = TryResolveAngularVelocityDelta3D(
                body3D,
                relativeContactPoint3D,
                -normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Vector3d angularVelocityDelta3D);
        bool linear2DResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                body2D?.ProjectLinearMotion(planarNormal) ?? Vector2d.Zero,
                normalVelocity,
                responseFactor,
                body2D?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector2d linearVelocityDelta2D);
        bool angular2DResolved = TryResolveAngularVelocityDelta2D(
                body2D,
                relativeContactPoint2D,
                planarNormal,
                normalVelocity,
                responseFactor,
                denominator,
                out Fixed64 angularVelocityDelta2D);
        if (!(linear3DResolved
                & angular3DResolved
                & linear2DResolved
                & angular2DResolved))
        {
            return false;
        }

        result = new ContactNormalVelocityDeltaResultMixed(
            normalVelocity,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D);
        return true;
    }

    internal static ContactNormalImpulseResultMixed CalculateAccumulatedDelta(
        SolidBody? body3D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector3d relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale)
    {
        Fixed64 normalVelocity = ComputeNormalVelocity(
            linearVelocity3D,
            angularVelocity3D,
            relativeContactPoint3D,
            linearVelocity2D,
            angularVelocity2D,
            relativeContactPoint2D,
            normal);
        Fixed64 denominator = ComputeDenominator(
            body3D,
            relativeContactPoint3D,
            body2D,
            relativeContactPoint2D,
            normal);
        if (denominator <= Fixed64.Epsilon)
            return ZeroImpulse(normalVelocity);

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        Fixed64 impulseScale = normalVelocity < Fixed64.Zero
            ? positiveImpulseScale
            : negativeImpulseScale;
        Fixed64 scaledImpulse;
        if (!Fixed64.TryMultiplyDivide(
                normalVelocity,
                responseFactor,
                impulseScale,
                denominator,
                out scaledImpulse))
        {
            scaledImpulse = normalVelocity < Fixed64.Zero
                ? Fixed64.MaxValue
                : Fixed64.MinValue;
        }
        Fixed64 impulseScalar = FixedMath.Max(Fixed64.Zero, accumulatedImpulse + scaledImpulse)
            - accumulatedImpulse;
        if (impulseScalar == Fixed64.Zero)
            return ZeroImpulse(normalVelocity);

        Vector3d impulse3D = -normal * impulseScalar;
        Vector2d impulse2D = normal.ToVector2d() * impulseScalar;
        return new ContactNormalImpulseResultMixed(
            normalVelocity,
            impulseScalar,
            ComputeLinearVelocityDelta3D(body3D, impulse3D),
            ComputeAngularVelocityDelta3D(body3D, relativeContactPoint3D, impulse3D),
            ComputeLinearVelocityDelta2D(body2D, impulse2D),
            ComputeAngularVelocityDelta2D(body2D, relativeContactPoint2D, impulse2D));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeNormalVelocity(
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector3d relativeContactPoint3D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal)
    {
        Vector3d pointVelocity3D = linearVelocity3D
            + Vector3d.Cross(angularVelocity3D, relativeContactPoint3D);
        Vector2d pointVelocity2D = linearVelocity2D
            + AngularVelocityAtPoint(relativeContactPoint2D, angularVelocity2D);
        return Vector3d.Dot(
            pointVelocity2D.ToVector3d(Fixed64.Zero) - pointVelocity3D,
            normal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeDenominator(
        SolidBody? body3D,
        Vector3d relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal)
    {
        Vector2d planarNormal = normal.ToVector2d();
        return GetConstrainedInverseMass3D(body3D, normal)
            + GetConstrainedInverseMass2D(body2D, normal)
            + ComputeAngularDenominator3D(body3D, relativeContactPoint3D, normal)
            + ComputeAngularDenominator2D(body2D, relativeContactPoint2D, planarNormal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass3D(SolidBody? body, Vector3d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass2D(SolidBody2D? body, Vector3d axis)
    {
        if (body == null)
            return Fixed64.Zero;

        Vector2d planarAxis = axis.ToVector2d();
        return planarAxis == Vector2d.Zero
            ? Fixed64.Zero
            : body.GetConstrainedInverseMass(planarAxis) * planarAxis.MagnitudeSquared;
    }

    private static Fixed64 ComputeAngularDenominator3D(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d axis)
    {
        if (body?.CanRotate != true)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Fixed64 denominator = Vector3d.Dot(
            Vector3d.Cross(angularVelocityDelta, relativeContactPoint),
            axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeAngularDenominator2D(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d axis)
    {
        if (body?.CanRotate != true || axis == Vector2d.Zero)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativeContactPoint, axis);
        return cross * cross * body.EffectiveInverseMomentOfInertia;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeLinearVelocityDelta3D(SolidBody? body, Vector3d impulse) =>
        body == null
            ? Vector3d.Zero
            : body.ProjectLinearMotion(impulse * body.EffectiveInverseMass);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeAngularVelocityDelta3D(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d impulse) =>
        body?.ApplyConstrainedInverseInertia(Vector3d.Cross(relativeContactPoint, impulse))
            ?? Vector3d.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComputeLinearVelocityDelta2D(SolidBody2D? body, Vector2d impulse) =>
        body == null
            ? Vector2d.Zero
            : body.ProjectLinearMotion(impulse * body.EffectiveInverseMass);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeAngularVelocityDelta2D(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d impulse) =>
        body?.CanRotate == true
            ? Vector2d.CrossProduct(relativeContactPoint, impulse)
                * body.EffectiveInverseMomentOfInertia
            : Fixed64.Zero;

    private static bool TryResolveAngularVelocityDelta3D(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d signedNormal,
        Fixed64 normalVelocity,
        Fixed64 responseFactor,
        Fixed64 denominator,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        if (body?.CanRotate != true)
            return true;

        Vector3d response = body.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, signedNormal));
        bool xResolved = Fixed64.TryMultiplyDivide(
            response.X,
            normalVelocity,
            responseFactor,
            denominator,
            out Fixed64 x);
        bool yResolved = Fixed64.TryMultiplyDivide(
            response.Y,
            normalVelocity,
            responseFactor,
            denominator,
            out Fixed64 y);
        bool zResolved = Fixed64.TryMultiplyDivide(
            response.Z,
            normalVelocity,
            responseFactor,
            denominator,
            out Fixed64 z);
        velocityDelta = new Vector3d(x, y, z);
        return xResolved & yResolved & zResolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryResolveAngularVelocityDelta2D(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d signedNormal,
        Fixed64 normalVelocity,
        Fixed64 responseFactor,
        Fixed64 denominator,
        out Fixed64 velocityDelta)
    {
        velocityDelta = Fixed64.Zero;
        if (body?.CanRotate != true)
            return true;

        Fixed64 torqueScale = Vector2d.CrossProduct(relativeContactPoint, signedNormal);
        if (torqueScale == Fixed64.Zero)
            return true;

        bool angularScaleResolved = Fixed64.TryMultiplyDivide(
            normalVelocity,
            responseFactor,
            body.EffectiveInverseMomentOfInertia,
            denominator,
            out Fixed64 angularScale);
        bool velocityDeltaResolved = Fixed64.TryMultiplyDivide(
            torqueScale,
            angularScale,
            Fixed64.One,
            out velocityDelta);
        return angularScaleResolved
            & (angularScale != Fixed64.Zero | torqueScale.Abs() <= Fixed64.One)
            & velocityDeltaResolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalImpulseResultMixed ZeroImpulse(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Fixed64.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector2d.Zero,
            Fixed64.Zero);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalVelocityDeltaResultMixed ZeroVelocityDelta(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector2d.Zero,
            Fixed64.Zero);
}
