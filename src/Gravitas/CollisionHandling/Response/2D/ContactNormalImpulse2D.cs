//=======================================================================
// ContactNormalImpulse2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 2D contact-normal impulse result for two participants.
/// </summary>
internal readonly struct ContactNormalImpulseResult2D
{
    public ContactNormalImpulseResult2D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Vector2d LinearVelocityDeltaA { get; }

    public Fixed64 AngularVelocityDeltaA { get; }

    public Vector2d LinearVelocityDeltaB { get; }

    public Fixed64 AngularVelocityDeltaB { get; }
}

/// <summary>
/// Side-effect-free 2D velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResult2D
{
    public ContactNormalVelocityDeltaResult2D(
        Fixed64 normalVelocity,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
    }

    public Fixed64 NormalVelocity { get; }

    public Vector2d LinearVelocityDeltaA { get; }

    public Fixed64 AngularVelocityDeltaA { get; }

    public Vector2d LinearVelocityDeltaB { get; }

    public Fixed64 AngularVelocityDeltaB { get; }
}

/// <summary>
/// Calculates allocation-free 2D contact-point normal response without mutating either body.
/// </summary>
internal static class ContactNormalImpulse2D
{
    internal static bool TryCalculateVelocityDeltas(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Vector2d relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResult2D result)
    {
        result = default;
        Fixed64 normalVelocity = ComputeNormalVelocity(
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        if (normalVelocity >= Fixed64.Zero)
        {
            result = ZeroVelocityDelta(normalVelocity);
            return true;
        }

        Fixed64 denominator = ComputeDenominator(
            bodyA,
            relativeContactPointA,
            bodyB,
            relativeContactPointB,
            normal);
        if (denominator <= Fixed64.Zero)
            return false;

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        if (!ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                bodyA?.ProjectLinearMotion(-normal) ?? Vector2d.Zero,
                normalVelocity,
                responseFactor,
                bodyA?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector2d linearVelocityDeltaA)
            || !TryResolveAngularVelocityDelta(
                bodyA,
                relativeContactPointA,
                -normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Fixed64 angularVelocityDeltaA)
            || !ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                bodyB?.ProjectLinearMotion(normal) ?? Vector2d.Zero,
                normalVelocity,
                responseFactor,
                bodyB?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector2d linearVelocityDeltaB)
            || !TryResolveAngularVelocityDelta(
                bodyB,
                relativeContactPointB,
                normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Fixed64 angularVelocityDeltaB))
        {
            return false;
        }

        result = new ContactNormalVelocityDeltaResult2D(
            normalVelocity,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB);
        return true;
    }

    internal static ContactNormalImpulseResult2D CalculateAccumulatedDelta(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Vector2d relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale)
    {
        Fixed64 normalVelocity = ComputeNormalVelocity(
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        Fixed64 denominator = ComputeDenominator(
            bodyA,
            relativeContactPointA,
            bodyB,
            relativeContactPointB,
            normal);
        if (denominator <= Fixed64.Epsilon)
            return Zero(normalVelocity);

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
            return Zero(normalVelocity);

        Vector2d impulseB = normal * impulseScalar;
        Vector2d impulseA = -impulseB;
        return new ContactNormalImpulseResult2D(
            normalVelocity,
            impulseScalar,
            ComputeLinearVelocityDelta(bodyA, impulseA),
            ComputeAngularVelocityDelta(bodyA, relativeContactPointA, impulseA),
            ComputeLinearVelocityDelta(bodyB, impulseB),
            ComputeAngularVelocityDelta(bodyB, relativeContactPointB, impulseB));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody2D? body, Vector2d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeNormalVelocity(
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d relativeContactPointA,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Vector2d relativeContactPointB,
        Vector2d normal)
    {
        Vector2d pointVelocityA = linearVelocityA
            + AngularVelocityAtPoint(relativeContactPointA, angularVelocityA);
        Vector2d pointVelocityB = linearVelocityB
            + AngularVelocityAtPoint(relativeContactPointB, angularVelocityB);
        return Vector2d.Dot(pointVelocityB - pointVelocityA, normal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeDenominator(
        SolidBody2D? bodyA,
        Vector2d relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d relativeContactPointB,
        Vector2d normal) =>
        GetConstrainedInverseMass(bodyA, normal)
        + GetConstrainedInverseMass(bodyB, normal)
        + ComputeAngularDenominator(bodyA, relativeContactPointA, normal)
        + ComputeAngularDenominator(bodyB, relativeContactPointB, normal);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeAngularDenominator(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d axis)
    {
        if (body?.CanRotate != true)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativeContactPoint, axis);
        return cross * cross * body.EffectiveInverseMomentOfInertia;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComputeLinearVelocityDelta(SolidBody2D? body, Vector2d impulse) =>
        body == null
            ? Vector2d.Zero
            : body.ProjectLinearMotion(impulse * body.EffectiveInverseMass);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeAngularVelocityDelta(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d impulse) =>
        body?.CanRotate == true
            ? Vector2d.CrossProduct(relativeContactPoint, impulse)
                * body.EffectiveInverseMomentOfInertia
            : Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryResolveAngularVelocityDelta(
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

        if (!Fixed64.TryMultiplyDivide(
            normalVelocity,
            responseFactor,
            body.EffectiveInverseMomentOfInertia,
            denominator,
            out Fixed64 angularScale))
        {
            return false;
        }

        if (angularScale == Fixed64.Zero && torqueScale.Abs() > Fixed64.One)
            return false;

        return Fixed64.TryMultiplyDivide(
            torqueScale,
            angularScale,
            Fixed64.One,
            out velocityDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalImpulseResult2D Zero(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Fixed64.Zero,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Fixed64.Zero);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalVelocityDeltaResult2D ZeroVelocityDelta(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Fixed64.Zero);
}
