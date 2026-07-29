//=======================================================================
// ContactNormalImpulse3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        : this(
            normalVelocity,
            impulseScalar,
            impulseScalar,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB)
    {
    }

    public ContactNormalImpulseResult3D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        AppliedImpulseScalar = appliedImpulseScalar;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
        HasRepresentableAppliedImpulse = hasRepresentableAppliedImpulse;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

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
        : this(
            normalVelocity,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB,
            normalVelocity < Fixed64.Zero,
            hasRepresentableNormalVelocity: true)
    {
    }

    public ContactNormalVelocityDeltaResult3D(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB,
        bool isClosing,
        bool hasRepresentableNormalVelocity)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
        IsClosing = isClosing;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
    }

    public Fixed64 NormalVelocity { get; }

    public bool IsClosing { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public Vector3d LinearVelocityDeltaA { get; }

    public Vector3d AngularVelocityDeltaA { get; }

    public Vector3d LinearVelocityDeltaB { get; }

    public Vector3d AngularVelocityDeltaB { get; }
}

internal readonly struct ContactEffectiveMassTerms3D
{
    internal ContactEffectiveMassTerms3D(
        Fixed64 linearA,
        Fixed64 linearB,
        Fixed64 angularA,
        Fixed64 angularB)
    {
        LinearA = linearA;
        LinearB = linearB;
        AngularA = angularA;
        AngularB = angularB;
    }

    internal Fixed64 LinearA { get; }

    internal Fixed64 LinearB { get; }

    internal Fixed64 AngularA { get; }

    internal Fixed64 AngularB { get; }

    internal Fixed64 SaturatedSum =>
        LinearA + LinearB + AngularA + AngularB;

    internal bool TryGetValue(out Fixed64 value)
    {
        bool resolved =
            Fixed64.TryAdd(LinearA, LinearB, out Fixed64 linear);
        resolved &= Fixed64.TryAdd(
            linear,
            AngularA,
            out Fixed64 first);
        resolved &= Fixed64.TryAdd(first, AngularB, out value);
        if (!resolved)
            value = default;
        return resolved;
    }
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
        if (!TryComputeNormalVelocity(
                linearVelocityA,
                angularVelocityA,
                relativeContactPointA,
                linearVelocityB,
                angularVelocityB,
                relativeContactPointB,
                normal,
                out Fixed64 normalVelocity))
        {
            return false;
        }

        if (normalVelocity >= Fixed64.Zero)
        {
            result = ZeroVelocityDelta(normalVelocity);
            return true;
        }

        if (!TryComputeDenominator(
                bodyA,
                relativeContactPointA,
                bodyB,
                relativeContactPointB,
                normal,
                out ContactEffectiveMassTerms3D denominator)
            || denominator.SaturatedSum <= Fixed64.Zero)
        {
            return false;
        }

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        bool linearAResolved = TryResolveVelocityDelta(
            bodyA?.ProjectLinearMotion(-normal) ?? Vector3d.Zero,
            normalVelocity,
            responseFactor,
            bodyA?.EffectiveInverseMass ?? Fixed64.Zero,
            denominator,
            out Vector3d linearVelocityDeltaA);
        bool angularAResolved = TryResolveAngularVelocityDelta(
                bodyA,
                relativeContactPointA,
                -normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Vector3d angularVelocityDeltaA);
        bool linearBResolved = TryResolveVelocityDelta(
            bodyB?.ProjectLinearMotion(normal) ?? Vector3d.Zero,
            normalVelocity,
            responseFactor,
            bodyB?.EffectiveInverseMass ?? Fixed64.Zero,
            denominator,
            out Vector3d linearVelocityDeltaB);
        bool angularBResolved = TryResolveAngularVelocityDelta(
                bodyB,
                relativeContactPointB,
                normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Vector3d angularVelocityDeltaB);
        if (!(linearAResolved
                & angularAResolved
                & linearBResolved
                & angularBResolved))
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

    internal static bool TryCalculateVelocityDeltasExact(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResult3D result)
    {
        result = default;
        if (!ExactContactLever3D.TryGetNormalResponse(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                relativeContactPointA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                relativeContactPointB,
                normal,
                restitution,
                restitutionVelocityThreshold,
                out ExactNormalResponse3D response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        result = new ContactNormalVelocityDeltaResult3D(
            normalVelocity,
            response.FirstLinearVelocityDelta,
            response.FirstAngularVelocityDelta,
            response.SecondLinearVelocityDelta,
            response.SecondAngularVelocityDelta,
            response.IsClosing,
            hasNormalVelocity);
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
        _ = TryCalculateAccumulatedDelta(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal,
            restitution,
            restitutionVelocityThreshold,
            accumulatedImpulse,
            positiveImpulseScale,
            negativeImpulseScale,
            out ContactNormalImpulseResult3D result);
        return result;
    }

    internal static bool TryCalculateAccumulatedDelta(
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
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResult3D result)
    {
        result = default;
        if (!TryComputeNormalVelocity(
                linearVelocityA,
                angularVelocityA,
                relativeContactPointA,
                linearVelocityB,
                angularVelocityB,
                relativeContactPointB,
                normal,
                out Fixed64 normalVelocity)
            || !TryComputeDenominator(
                bodyA,
                relativeContactPointA,
                bodyB,
                relativeContactPointB,
                normal,
                out ContactEffectiveMassTerms3D denominator))
        {
            return false;
        }

        if (denominator.SaturatedSum <= Fixed64.Zero)
        {
            result = Zero(normalVelocity);
            return true;
        }

        if (!TryCalculateAccumulatedImpulseDelta(
                normalVelocity,
                denominator,
                restitution,
                restitutionVelocityThreshold,
                accumulatedImpulse,
                positiveImpulseScale,
                negativeImpulseScale,
                out Fixed64 impulseScalar))
        {
            return false;
        }

        if (impulseScalar == Fixed64.Zero)
        {
            result = Zero(normalVelocity);
            return true;
        }

        Vector3d impulseB = normal * impulseScalar;
        Vector3d impulseA = -impulseB;
        bool linearAResolved = TryComputeLinearVelocityDelta(
            bodyA,
            impulseA,
            out Vector3d linearVelocityDeltaA);
        bool angularAResolved = TryComputeAngularVelocityDelta(
            bodyA,
            relativeContactPointA,
            impulseA,
            out Vector3d angularVelocityDeltaA);
        bool linearBResolved = TryComputeLinearVelocityDelta(
            bodyB,
            impulseB,
            out Vector3d linearVelocityDeltaB);
        bool angularBResolved = TryComputeAngularVelocityDelta(
            bodyB,
            relativeContactPointB,
            impulseB,
            out Vector3d angularVelocityDeltaB);
        if (!(linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved))
        {
            return false;
        }

        result = new ContactNormalImpulseResult3D(
            normalVelocity,
            impulseScalar,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB);
        return true;
    }

    internal static bool TryCalculateAccumulatedDeltaExact(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResult3D result)
    {
        result = default;
        if (!ExactContactLever3D.TryGetAccumulatedNormalResponse(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                relativeContactPointA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                relativeContactPointB,
                normal,
                restitution,
                restitutionVelocityThreshold,
                accumulatedImpulse,
                positiveImpulseScale,
                negativeImpulseScale,
                out ExactNormalResponse3D response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        bool hasAppliedImpulse =
            response.TryGetAppliedImpulse(out Fixed64 appliedImpulse);
        Fixed64 impulseScalar;
        if (response.TryGetAccumulatedImpulse(
                out Fixed64 newAccumulatedImpulse))
        {
            // Both values are nonnegative, so their difference is always in
            // [-Fixed64.MaxValue, Fixed64.MaxValue].
            impulseScalar = newAccumulatedImpulse - accumulatedImpulse;
        }
        else
        {
            impulseScalar = -accumulatedImpulse;
        }

        result = new ContactNormalImpulseResult3D(
            normalVelocity,
            impulseScalar,
            appliedImpulse,
            response.FirstLinearVelocityDelta,
            response.FirstAngularVelocityDelta,
            response.SecondLinearVelocityDelta,
            response.SecondAngularVelocityDelta,
            hasNormalVelocity,
            hasAppliedImpulse);
        return true;
    }

    private static bool TryCalculateAccumulatedImpulseDelta(
        Fixed64 normalVelocity,
        in ContactEffectiveMassTerms3D denominator,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out Fixed64 impulseDelta)
    {
        Fixed64 appliedRestitution =
            normalVelocity < -restitutionVelocityThreshold
                ? restitution
                : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        Fixed64 impulseScale = normalVelocity < Fixed64.Zero
            ? positiveImpulseScale
            : negativeImpulseScale;
        if (!Fixed64.TryMultiplyDivideBySum(
                normalVelocity,
                responseFactor,
                impulseScale,
                Fixed64.One,
                denominator.LinearA,
                denominator.LinearB,
                denominator.AngularA,
                denominator.AngularB,
                out Fixed64 scaledImpulse))
        {
            impulseDelta = default;
            return normalVelocity >= Fixed64.Zero
                && responseFactor <= Fixed64.Zero
                && impulseScale >= Fixed64.Zero
                && accumulatedImpulse >= Fixed64.Zero
                && Fixed64.TrySubtract(
                    Fixed64.Zero,
                    accumulatedImpulse,
                    out impulseDelta);
        }

        if (!Fixed64.TryAdd(
                accumulatedImpulse,
                scaledImpulse,
                out Fixed64 accumulated))
        {
            impulseDelta = default;
            return false;
        }

        return Fixed64.TrySubtract(
            FixedMath.Max(Fixed64.Zero, accumulated),
            accumulatedImpulse,
            out impulseDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody? body, Vector3d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    internal static bool TryComputeNormalVelocity(
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d relativeContactPointA,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d relativeContactPointB,
        Vector3d normal,
        out Fixed64 normalVelocity)
    {
        if (ContactResponseArithmetic3D.CanUseFastPointVelocity(
                linearVelocityA,
                angularVelocityA,
                relativeContactPointA,
                linearVelocityB,
                angularVelocityB,
                relativeContactPointB,
                normal))
        {
            Vector3d fastPointVelocityA = linearVelocityA
                + Vector3d.Cross(
                    angularVelocityA,
                    relativeContactPointA);
            Vector3d fastPointVelocityB = linearVelocityB
                + Vector3d.Cross(
                    angularVelocityB,
                    relativeContactPointB);
            normalVelocity = Vector3d.Dot(
                fastPointVelocityB - fastPointVelocityA,
                normal);
            return true;
        }

        bool resolved = ContactResponseArithmetic3D.TryCross(
                angularVelocityA,
                relativeContactPointA,
                out Vector3d angularA)
            & ContactResponseArithmetic3D.TryCross(
                angularVelocityB,
                relativeContactPointB,
                out Vector3d angularB);
        if (!resolved
            || !Vector3d.TryAdd(
                linearVelocityA,
                angularA,
                out Vector3d pointVelocityA)
            || !Vector3d.TryAdd(
                linearVelocityB,
                angularB,
                out Vector3d pointVelocityB)
            || !Vector3d.TrySubtract(
                pointVelocityB,
                pointVelocityA,
                out Vector3d relativeVelocity))
        {
            normalVelocity = default;
            return false;
        }

        return ContactResponseArithmetic3D.TryDot(
            relativeVelocity,
            normal,
            out normalVelocity);
    }

    private static bool TryComputeDenominator(
        SolidBody? bodyA,
        Vector3d relativeContactPointA,
        SolidBody? bodyB,
        Vector3d relativeContactPointB,
        Vector3d normal,
        out ContactEffectiveMassTerms3D denominator)
    {
        denominator = default;
        bool angularAResolved = TryComputeAngularDenominator(
            bodyA,
            relativeContactPointA,
            normal,
            out Fixed64 angularA);
        bool angularBResolved = TryComputeAngularDenominator(
            bodyB,
            relativeContactPointB,
            normal,
            out Fixed64 angularB);
        if (!(angularAResolved & angularBResolved))
        {
            return false;
        }

        denominator = new ContactEffectiveMassTerms3D(
            GetConstrainedInverseMass(bodyA, normal),
            GetConstrainedInverseMass(bodyB, normal),
            angularA,
            angularB);
        return true;
    }

    internal static bool TryComputeAngularDenominator(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d axis,
        out Fixed64 denominator)
    {
        denominator = Fixed64.Zero;
        if (body?.CanRotate != true)
            return true;

        Fixed3x3 inverseInertia =
            body.GetConstrainedInverseInertiaTensor();
        if (ContactResponseArithmetic3D.CanUseFastAngularResponse(
                relativeContactPoint,
                axis,
                inverseInertia))
        {
            Vector3d fastTorqueAxis =
                Vector3d.Cross(relativeContactPoint, axis);
            Vector3d fastAngularVelocityDelta =
                Fixed3x3.TransformDirection(
                    inverseInertia,
                    fastTorqueAxis);
            Vector3d fastAngular = Vector3d.Cross(
                fastAngularVelocityDelta,
                relativeContactPoint);
            denominator = FixedMath.Max(
                Vector3d.Dot(fastAngular, axis),
                Fixed64.Zero);
            return true;
        }

        if (!ContactResponseArithmetic3D.TryCross(
                relativeContactPoint,
                axis,
                out Vector3d torqueAxis)
            || !ContactResponseArithmetic3D.TryTransformDirection(
                inverseInertia,
                torqueAxis,
                out Vector3d angularVelocityDelta)
            || !ContactResponseArithmetic3D.TryCross(
                angularVelocityDelta,
                relativeContactPoint,
                out Vector3d angular)
            || !ContactResponseArithmetic3D.TryDot(
                angular,
                axis,
                out denominator))
        {
            denominator = default;
            return false;
        }

        denominator = FixedMath.Max(denominator, Fixed64.Zero);
        return true;
    }

    internal static bool TryComputeLinearVelocityDelta(
        SolidBody? body,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        if (body != null)
            impulse = body.ProjectLinearMotion(impulse);

        if (!ContactResponseArithmetic3D.TryScale(
                impulse,
                body?.EffectiveInverseMass ?? Fixed64.Zero,
                out velocityDelta))
        {
            return false;
        }

        return true;
    }

    internal static bool TryComputeAngularVelocityDelta(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        if (body?.CanRotate != true)
            return true;

        Fixed3x3 inverseInertia =
            body.GetConstrainedInverseInertiaTensor();
        if (ContactResponseArithmetic3D.CanUseFastAngularResponse(
                relativeContactPoint,
                impulse,
                inverseInertia))
        {
            velocityDelta = Fixed3x3.TransformDirection(
                inverseInertia,
                Vector3d.Cross(relativeContactPoint, impulse));
            return true;
        }

        bool torqueResolved = ContactResponseArithmetic3D.TryCross(
                relativeContactPoint,
                impulse,
                out Vector3d torqueAxis);
        bool transformResolved =
            ContactResponseArithmetic3D.TryTransformDirection(
                inverseInertia,
                torqueAxis,
                out velocityDelta);
        return torqueResolved & transformResolved;
    }

    private static bool TryResolveAngularVelocityDelta(
        SolidBody? body,
        Vector3d relativeContactPoint,
        Vector3d signedNormal,
        Fixed64 normalVelocity,
        Fixed64 responseFactor,
        in ContactEffectiveMassTerms3D denominator,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        if (body?.CanRotate != true)
            return true;

        bool responseResolved = TryComputeAngularVelocityDelta(
            body,
            relativeContactPoint,
            signedNormal,
            out Vector3d response);
        bool deltaResolved = TryResolveVelocityDelta(
            response,
            normalVelocity,
            responseFactor,
            Fixed64.One,
            denominator,
            out velocityDelta);
        return responseResolved & deltaResolved;
    }

    private static bool TryResolveVelocityDelta(
        Vector3d response,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 thirdMultiplier,
        in ContactEffectiveMassTerms3D denominator,
        out Vector3d velocityDelta)
    {
        bool xResolved = Fixed64.TryMultiplyDivideBySum(
            response.X,
            firstMultiplier,
            secondMultiplier,
            thirdMultiplier,
            denominator.LinearA,
            denominator.LinearB,
            denominator.AngularA,
            denominator.AngularB,
            out Fixed64 x);
        bool yResolved = Fixed64.TryMultiplyDivideBySum(
            response.Y,
            firstMultiplier,
            secondMultiplier,
            thirdMultiplier,
            denominator.LinearA,
            denominator.LinearB,
            denominator.AngularA,
            denominator.AngularB,
            out Fixed64 y);
        bool zResolved = Fixed64.TryMultiplyDivideBySum(
            response.Z,
            firstMultiplier,
            secondMultiplier,
            thirdMultiplier,
            denominator.LinearA,
            denominator.LinearB,
            denominator.AngularA,
            denominator.AngularB,
            out Fixed64 z);
        velocityDelta = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
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
