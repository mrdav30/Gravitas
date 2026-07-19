//=======================================================================
// ContactNormalImpulse3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 3D contact-normal impulse result for two participants.
/// </summary>
internal readonly struct ContactNormalImpulseResult3D
{
    public ContactNormalImpulseResult3D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB)
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

    public Vector3d LinearVelocityDeltaA { get; }

    public Vector3d AngularVelocityDeltaA { get; }

    public Vector3d LinearVelocityDeltaB { get; }

    public Vector3d AngularVelocityDeltaB { get; }
}

/// <summary>
/// Side-effect-free 3D velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResult3D
{
    public ContactNormalVelocityDeltaResult3D(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
    }

    public Fixed64 NormalVelocity { get; }

    public Vector3d LinearVelocityDeltaA { get; }

    public Vector3d AngularVelocityDeltaA { get; }

    public Vector3d LinearVelocityDeltaB { get; }

    public Vector3d AngularVelocityDeltaB { get; }
}

/// <summary>
/// Calculates allocation-free 3D contact-point normal response without mutating either body.
/// </summary>
internal static class ContactNormalImpulse3D
{
    internal static bool TryCalculateVelocityDeltas(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResult3D result)
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
                bodyA?.ProjectLinearMotion(-normal) ?? Vector3d.Zero,
                normalVelocity,
                responseFactor,
                bodyA?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector3d linearVelocityDeltaA)
            || !TryResolveAngularVelocityDelta(
                bodyA,
                relativeContactPointA,
                -normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Vector3d angularVelocityDeltaA)
            || !ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                bodyB?.ProjectLinearMotion(normal) ?? Vector3d.Zero,
                normalVelocity,
                responseFactor,
                bodyB?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector3d linearVelocityDeltaB)
            || !TryResolveAngularVelocityDelta(
                bodyB,
                relativeContactPointB,
                normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Vector3d angularVelocityDeltaB))
        {
            return false;
        }

        result = new ContactNormalVelocityDeltaResult3D(
            normalVelocity,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB);
        return true;
    }

    internal static ContactNormalImpulseResult3D CalculateAccumulatedDelta(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d relativeContactPointB,
        Vector3d normal,
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

        Vector3d impulseB = normal * impulseScalar;
        Vector3d impulseA = -impulseB;
        return new ContactNormalImpulseResult3D(
            normalVelocity,
            impulseScalar,
            ComputeLinearVelocityDelta(bodyA, impulseA),
            ComputeAngularVelocityDelta(bodyA, relativeContactPointA, impulseA),
            ComputeLinearVelocityDelta(bodyB, impulseB),
            ComputeAngularVelocityDelta(bodyB, relativeContactPointB, impulseB));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody? body, Vector3d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeNormalVelocity(
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d relativeContactPointA,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d relativeContactPointB,
        Vector3d normal)
    {
        Vector3d pointVelocityA = linearVelocityA
            + Vector3d.Cross(angularVelocityA, relativeContactPointA);
        Vector3d pointVelocityB = linearVelocityB
            + Vector3d.Cross(angularVelocityB, relativeContactPointB);
        return Vector3d.Dot(pointVelocityB - pointVelocityA, normal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeDenominator(
        SolidBody? bodyA,
        Vector3d relativeContactPointA,
        SolidBody? bodyB,
        Vector3d relativeContactPointB,
        Vector3d normal) =>
        GetConstrainedInverseMass(bodyA, normal)
        + GetConstrainedInverseMass(bodyB, normal)
        + ComputeAngularDenominator(bodyA, relativeContactPointA, normal)
        + ComputeAngularDenominator(bodyB, relativeContactPointB, normal);

    private static Fixed64 ComputeAngularDenominator(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d axis)
    {
        if (body == null)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Fixed64 denominator = Vector3d.Dot(
            Vector3d.Cross(angularVelocityDelta, relativeContactPoint),
            axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeLinearVelocityDelta(SolidBody? body, Vector3d impulse) =>
        body == null
            ? Vector3d.Zero
            : body.ProjectLinearMotion(impulse * body.EffectiveInverseMass);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeAngularVelocityDelta(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d impulse) =>
        body?.ApplyConstrainedInverseInertia(Vector3d.Cross(relativeContactPoint, impulse))
            ?? Vector3d.Zero;

    private static bool TryResolveAngularVelocityDelta(
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
        if (!Fixed64.TryMultiplyDivide(response.X, normalVelocity, responseFactor, denominator, out Fixed64 x)
            || !Fixed64.TryMultiplyDivide(response.Y, normalVelocity, responseFactor, denominator, out Fixed64 y)
            || !Fixed64.TryMultiplyDivide(response.Z, normalVelocity, responseFactor, denominator, out Fixed64 z))
        {
            return false;
        }

        velocityDelta = new Vector3d(x, y, z);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalImpulseResult3D Zero(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Fixed64.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ContactNormalVelocityDeltaResult3D ZeroVelocityDelta(Fixed64 normalVelocity) =>
        new(
            normalVelocity,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero);
}
