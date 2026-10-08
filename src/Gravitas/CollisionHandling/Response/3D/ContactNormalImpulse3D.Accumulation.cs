//=======================================================================
// ContactNormalImpulse3D.Accumulation.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <content>Owns accumulated unilateral impulses and frozen incoming restitution targets.</content>
internal static partial class ContactNormalImpulse3D
{
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

        return TryCreateAccumulatedResult(bodyA, relativeContactPointA,
            bodyB, relativeContactPointB, normal, normalVelocity, impulseScalar, out result);
    }

    private static bool TryCreateAccumulatedResult(SolidBody? bodyA,
        Vector3d relativeContactPointA, SolidBody? bodyB, Vector3d relativeContactPointB,
        Vector3d normal, Fixed64 normalVelocity, Fixed64 impulseScalar,
        out ContactNormalImpulseResult3D result)
    {
        result = default;
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

    internal static bool TryCalculateAccumulatedDeltaWithImpact(
        SolidBody? bodyA, Vector3d linearVelocityA, Vector3d angularVelocityA,
        Vector3d relativeContactPointA, SolidBody? bodyB, Vector3d linearVelocityB,
        Vector3d angularVelocityB, Vector3d relativeContactPointB, Vector3d normal,
        Fixed64 restitution, Fixed64 restitutionVelocityThreshold, Fixed64 accumulatedImpulse,
        in ContactResponseSnapshot impact, out ContactNormalImpulseResult3D result)
    {
        if (restitution == Fixed64.Zero)
            return TryCalculateAccumulatedDelta(bodyA, linearVelocityA, angularVelocityA,
                relativeContactPointA, bodyB, linearVelocityB, angularVelocityB,
                relativeContactPointB, normal, restitution, restitutionVelocityThreshold,
                accumulatedImpulse, Fixed64.One, Fixed64.One, out result);
        result = default;
        if (!TryComputeNormalVelocity(linearVelocityA, angularVelocityA, relativeContactPointA,
                linearVelocityB, angularVelocityB, relativeContactPointB, normal, out Fixed64 velocity)
            || !TryComputeNormalVelocity(impact.LinearA, impact.AngularA, relativeContactPointA,
                impact.LinearB, impact.AngularB, relativeContactPointB, normal, out Fixed64 incoming)
            || !TryComputeDenominator(bodyA, relativeContactPointA, bodyB,
                relativeContactPointB, normal, out ContactEffectiveMassTerms3D denominator))
            return false;
        if (denominator.SaturatedSum <= Fixed64.Zero)
        {
            result = Zero(velocity);
            return true;
        }
        Fixed64 bounce = incoming < -restitutionVelocityThreshold ? restitution : Fixed64.Zero;
        // Fuse v_current + e*v_incoming before division. Materializing a rounded
        // bounce speed here would add another rounding to the completed impulse.
        Signed320 numerator = WideArithmetic.Negate(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(velocity), Signed192.One),
            WideArithmetic.MultiplySigned192(Signed192.Raw(incoming), Signed192.Raw(bounce))));
        Signed192 sum = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(Signed192.Raw(denominator.LinearA), Signed192.Raw(denominator.LinearB)),
            WideArithmetic.AddSigned192(Signed192.Raw(denominator.AngularA), Signed192.Raw(denominator.AngularB)));
        Fixed64 impulse;
        if (!Fixed64.TryGetSignedRawRatio(Signed832.ExtendValue(Signed576.ExtendValue(numerator)), Signed832.ExtendValue(sum), 0, out Fixed64 delta))
        {
            if (numerator.Sign >= 0) return false;
            impulse = -accumulatedImpulse;
        }
        else if (!Fixed64.TryAdd(accumulatedImpulse, delta, out Fixed64 accumulated)
            || !Fixed64.TrySubtract(FixedMath.Max(Fixed64.Zero, accumulated), accumulatedImpulse, out impulse))
            return false;
        return TryCreateAccumulatedResult(bodyA, relativeContactPointA, bodyB,
            relativeContactPointB, normal, velocity, impulse, out result);
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
        out ContactNormalImpulseResult3D result,
        in ContactResponseSnapshot impact = default)
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
                out ExactNormalResponse3D response, impact))
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

}
