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
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ExactNormalResponse3D response)
    {
        ExactContactResponseOperand3D first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -normal);
        ExactContactResponseOperand3D second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        return ExactContactResponseKernel.TryGetNormalResponse(
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
        out ExactNormalResponse3D response)
    {
        ExactContactResponseOperand3D first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -normal);
        ExactContactResponseOperand3D second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            normal);
        return ExactContactResponseKernel.TryGetAccumulatedNormalResponse(
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

    internal static ExactContactResponseOperand3D CreateResponseOperand(
        SolidBody? body,
        Vector3d linearVelocity,
        Vector3d angularVelocity,
        in ExactLever3D relativeContactPoint,
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
        in ExactLever3D relativeContactPointA,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector3d normal,
        out Fixed64 normalVelocity) =>
        WideLever3d.TryGetRelativePointVelocityProjection(
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA.Value,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB.Value,
            normal,
            out normalVelocity);

    internal static bool TryComputeDenominator(
        SolidBody? bodyA,
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        in ExactLever3D relativeContactPointB,
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
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        in ExactLever3D relativeContactPointB,
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
        in ExactLever3D relativeContactPoint,
        Vector3d axis,
        out Fixed64 denominator)
    {
        denominator = Fixed64.Zero;
        if (body?.CanRotate != true)
            return true;

        if (!WideLever3d.TryGetCrossProductQuadraticForm(
                relativeContactPoint.Value,
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
        in ExactLever3D relativeContactPoint,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        return body?.CanRotate != true
            || WideLever3d.TryGetTransformedScaledCrossProduct(
                relativeContactPoint.Value,
                impulse,
                body.GetConstrainedInverseInertiaTensor(),
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out velocityDelta);
    }

    internal static bool TryGetImpulseCombinationVelocityDeltas(
        SolidBody? bodyA,
        in ExactLever3D relativeContactPointA,
        SolidBody? bodyB,
        in ExactLever3D relativeContactPointB,
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
        in ExactLever3D relativeContactPoint,
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
            || WideLever3d.TryGetTransformedWeightedCrossProduct(
                    relativeContactPoint.Value,
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
