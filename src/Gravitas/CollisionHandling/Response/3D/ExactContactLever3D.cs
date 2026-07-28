//=======================================================================
// ExactContactLever3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Evaluates solver products for a semantic lever that cannot be narrowed to
/// <see cref="Vector3d"/>.
/// </summary>
internal static class ExactContactLever3D
{
    internal static bool TryGetNormalResponse(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        in FixedLever relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in FixedLever relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out FixedLeverNormalResponse3d response)
    {
        FixedLeverResponseOperand3d first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -normal);
        FixedLeverResponseOperand3d second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        return FixedLever.TryGetNormalResponse(
            first,
            second,
            normal,
            restitution,
            restitutionVelocityThreshold,
            out response);
    }

    internal static bool TryGetAccumulatedNormalResponse(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        in FixedLever relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in FixedLever relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out FixedLeverNormalResponse3d response)
    {
        FixedLeverResponseOperand3d first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -normal);
        FixedLeverResponseOperand3d second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        return FixedLever.TryGetAccumulatedNormalResponse(
            first,
            second,
            normal,
            restitution,
            restitutionVelocityThreshold,
            accumulatedImpulse,
            positiveImpulseScale,
            negativeImpulseScale,
            out response);
    }

    internal static FixedLeverResponseOperand3d CreateResponseOperand(
        SolidBody? body,
        Vector3d linearVelocity,
        Vector3d angularVelocity,
        in FixedLever relativeContactPoint,
        Vector3d signedNormal) =>
        new(
            relativeContactPoint,
            linearVelocity,
            angularVelocity,
            body?.ProjectLinearMotion(signedNormal) ?? Vector3d.Zero,
            body?.EffectiveInverseMass ?? Fixed64.Zero,
            body?.GetConstrainedInverseInertiaTensor() ?? Fixed3x3.Zero);

    internal static bool TryComputeNormalVelocity(
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        in FixedLever relativeContactPointA,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in FixedLever relativeContactPointB,
        Vector3d normal,
        out Fixed64 normalVelocity) =>
        FixedLever.TryGetRelativePointVelocityProjection(
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal,
            out normalVelocity);

    internal static bool TryComputeDenominator(
        SolidBody? bodyA,
        in FixedLever relativeContactPointA,
        SolidBody? bodyB,
        in FixedLever relativeContactPointB,
        Vector3d axis,
        out Fixed64 denominator)
    {
        denominator = default;
        return TryComputeDenominatorTerms(
                bodyA,
                relativeContactPointA,
                bodyB,
                relativeContactPointB,
                axis,
                out ContactEffectiveMassTerms3D terms)
            && terms.TryGetValue(out denominator);
    }

    internal static bool TryComputeDenominatorTerms(
        SolidBody? bodyA,
        in FixedLever relativeContactPointA,
        SolidBody? bodyB,
        in FixedLever relativeContactPointB,
        Vector3d axis,
        out ContactEffectiveMassTerms3D denominator)
    {
        bool angularAResolved = TryGetAngularDenominator(
            bodyA,
            relativeContactPointA,
            axis,
            out Fixed64 angularA);
        bool angularBResolved = TryGetAngularDenominator(
            bodyB,
            relativeContactPointB,
            axis,
            out Fixed64 angularB);
        if (!(angularAResolved & angularBResolved))
        {
            denominator = default;
            return false;
        }

        denominator = new ContactEffectiveMassTerms3D(
            bodyA?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero,
            bodyB?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero,
            angularA,
            angularB);
        return true;
    }

    internal static bool TryGetAngularDenominator(
        SolidBody? body,
        in FixedLever relativeContactPoint,
        Vector3d axis,
        out Fixed64 denominator)
    {
        denominator = Fixed64.Zero;
        if (body?.CanRotate != true)
            return true;

        if (!relativeContactPoint.TryGetCrossProductQuadraticForm(
                axis,
                body.GetConstrainedInverseInertiaTensor(),
                out denominator))
        {
            return false;
        }

        denominator = FixedMath.Max(denominator, Fixed64.Zero);
        return true;
    }

    internal static bool TryGetAngularVelocityDelta(
        SolidBody? body,
        in FixedLever relativeContactPoint,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        return body?.CanRotate != true
            || relativeContactPoint.TryGetTransformedScaledCrossProduct(
                impulse,
                body.GetConstrainedInverseInertiaTensor(),
                Fixed64.One,
                Fixed64.One,
                out velocityDelta);
    }

    internal static bool TryGetImpulseCombinationVelocityDeltas(
        SolidBody? bodyA,
        in FixedLever relativeContactPointA,
        SolidBody? bodyB,
        in FixedLever relativeContactPointB,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        Vector3d thirdAxis,
        Fixed64 thirdScale,
        out Vector3d linearVelocityDeltaA,
        out Vector3d angularVelocityDeltaA,
        out Vector3d linearVelocityDeltaB,
        out Vector3d angularVelocityDeltaB)
    {
        bool linearAResolved = TryGetLinearCombinationVelocityDelta(
            bodyA,
            -firstAxis,
            firstScale,
            -secondAxis,
            secondScale,
            -thirdAxis,
            thirdScale,
            out linearVelocityDeltaA);
        bool angularAResolved = TryGetAngularCombinationVelocityDelta(
            bodyA,
            relativeContactPointA,
            -firstAxis,
            firstScale,
            -secondAxis,
            secondScale,
            -thirdAxis,
            thirdScale,
            out angularVelocityDeltaA);
        bool linearBResolved = TryGetLinearCombinationVelocityDelta(
            bodyB,
            firstAxis,
            firstScale,
            secondAxis,
            secondScale,
            thirdAxis,
            thirdScale,
            out linearVelocityDeltaB);
        bool angularBResolved = TryGetAngularCombinationVelocityDelta(
            bodyB,
            relativeContactPointB,
            firstAxis,
            firstScale,
            secondAxis,
            secondScale,
            thirdAxis,
            thirdScale,
            out angularVelocityDeltaB);
        return linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved;
    }

    private static bool TryGetLinearCombinationVelocityDelta(
        SolidBody? body,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        Vector3d thirdAxis,
        Fixed64 thirdScale,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        return body?.CanTranslate != true
            || Vector3d.TryScaledLinearCombination(
                body.ProjectLinearMotion(firstAxis),
                firstScale,
                body.ProjectLinearMotion(secondAxis),
                secondScale,
                body.ProjectLinearMotion(thirdAxis),
                thirdScale,
                body.EffectiveInverseMass,
                out velocityDelta);
    }

    private static bool TryGetAngularCombinationVelocityDelta(
        SolidBody? body,
        in FixedLever relativeContactPoint,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        Vector3d thirdAxis,
        Fixed64 thirdScale,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        return body?.CanRotate != true
            || relativeContactPoint
                .TryGetTransformedWeightedCrossProduct(
                    firstAxis,
                    firstScale,
                    secondAxis,
                    secondScale,
                    thirdAxis,
                    thirdScale,
                    body.GetConstrainedInverseInertiaTensor(),
                    out velocityDelta);
    }
}
