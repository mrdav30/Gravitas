//=======================================================================
// CollisionResponse.Friction.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <content>Owns compact and exact two-axis Coulomb disk response.</content>
public static partial class CollisionResponse
{
    private static bool TrySolveFrictionImpulse(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse)
    {
        if (!contact.RelativeA.IsExact
            && !contact.RelativeB.IsExact
            && TryGetCompactFrictionResponse(
                contact,
                normalImpulseScalar,
                out tangentImpulse,
                out secondaryTangentImpulse,
                out Fixed64 tangentDelta,
                out Fixed64 secondaryTangentDelta)
            && ((tangentDelta == Fixed64.Zero
                    && secondaryTangentDelta == Fixed64.Zero)
                || TryApplyContactImpulseCombination(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    contact.Normal,
                    Fixed64.Zero,
                    contact.Tangent,
                    tangentDelta,
                    contact.SecondaryTangent,
                    secondaryTangentDelta)))
        {
            return true;
        }

        return TrySolveFrictionImpulseExact(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            normalImpulseScalar,
            out tangentImpulse,
            out secondaryTangentImpulse);
    }

    private static bool TryGetCompactFrictionResponse(
        SolverContact contact,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse,
        out Fixed64 tangentDelta,
        out Fixed64 secondaryTangentDelta)
    {
        tangentImpulse = default;
        secondaryTangentImpulse = default;
        tangentDelta = default;
        secondaryTangentDelta = default;
        bool limitsResolved = TryGetFrictionLimit(
            normalImpulseScalar,
            contact.StaticFriction,
            out Fixed64 staticFrictionLimit);
        limitsResolved &= TryGetFrictionLimit(
            normalImpulseScalar,
            contact.DynamicFriction,
            out Fixed64 dynamicFrictionLimit);
        if (!limitsResolved)
            return false;

        if (staticFrictionLimit == Fixed64.Zero
            && dynamicFrictionLimit == Fixed64.Zero)
        {
            return Fixed64.TrySubtract(
                    Fixed64.Zero,
                    contact.CachedTangentImpulse,
                    out tangentDelta)
                & Fixed64.TrySubtract(
                    Fixed64.Zero,
                    contact.CachedSecondaryTangentImpulse,
                    out secondaryTangentDelta);
        }

        Vector3d linearA = ResolveLinearVelocity(contact.A.Body);
        Vector3d angularA = ResolveAngularVelocity(contact.A.Body);
        Vector3d linearB = ResolveLinearVelocity(contact.B.Body);
        Vector3d angularB = ResolveAngularVelocity(contact.B.Body);
        if (!ContactResponseArithmetic3D.TryGetRelativePointVelocity(
                linearA,
                angularA,
                contact.RelativeA.Vector,
                linearB,
                angularB,
                contact.RelativeB.Vector,
                contact.Tangent,
                out Vector3d relativeVelocity))
        {
            return false;
        }

        bool deltasResolved = TryGetCompactTangentImpulseDelta(
            contact,
            relativeVelocity,
            contact.Tangent,
            out Fixed64 desiredTangentDelta);
        deltasResolved &= TryGetCompactTangentImpulseDelta(
            contact,
            relativeVelocity,
            contact.SecondaryTangent,
            out Fixed64 desiredSecondaryTangentDelta);
        bool diskResolved = Fixed64.TryAdd(
                contact.CachedTangentImpulse,
                desiredTangentDelta,
                out tangentImpulse)
            & Fixed64.TryAdd(
                contact.CachedSecondaryTangentImpulse,
                desiredSecondaryTangentDelta,
                out secondaryTangentImpulse)
            & TryGetMagnitudeSquared(
                tangentImpulse,
                secondaryTangentImpulse,
                out Fixed64 desiredMagnitudeSquared)
            & TryGetSquare(
                staticFrictionLimit,
                out Fixed64 staticLimitSquared);
        if (!(deltasResolved & diskResolved))
        {
            return false;
        }

        if (desiredMagnitudeSquared > staticLimitSquared)
        {
            Fixed64 magnitude = FixedMath.Sqrt(desiredMagnitudeSquared);
            Fixed64 scale = dynamicFrictionLimit / magnitude;
            if (dynamicFrictionLimit != Fixed64.Zero
                && scale == Fixed64.Zero)
            {
                return false;
            }

            Fixed64 desiredTangentImpulse = tangentImpulse;
            Fixed64 desiredSecondaryTangentImpulse =
                secondaryTangentImpulse;
            tangentImpulse *= scale;
            secondaryTangentImpulse *= scale;
            if (dynamicFrictionLimit != Fixed64.Zero
                && ((desiredTangentImpulse != Fixed64.Zero
                        && tangentImpulse == Fixed64.Zero)
                    || (desiredSecondaryTangentImpulse != Fixed64.Zero
                        && secondaryTangentImpulse == Fixed64.Zero)))
            {
                return false;
            }
        }

        return Fixed64.TrySubtract(
                tangentImpulse,
                contact.CachedTangentImpulse,
                out tangentDelta)
            & Fixed64.TrySubtract(
                secondaryTangentImpulse,
                contact.CachedSecondaryTangentImpulse,
                out secondaryTangentDelta);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TrySolveFrictionImpulseExact(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse)
    {
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        Vector3d linearA = ResolveLinearVelocity(contact.A.Body);
        Vector3d angularA = ResolveAngularVelocity(contact.A.Body);
        Vector3d linearB = ResolveLinearVelocity(contact.B.Body);
        Vector3d angularB = ResolveAngularVelocity(contact.B.Body);
        ExactContactResponseOperand3D primaryFirst =
            ExactContactLever3D.CreateResponseOperand(
                contact.A.Body,
                linearA,
                angularA,
                exactA,
                -contact.Tangent);
        ExactContactResponseOperand3D primarySecond =
            ExactContactLever3D.CreateResponseOperand(
                contact.B.Body,
                linearB,
                angularB,
                exactB,
                contact.Tangent);
        ExactContactResponseOperand3D secondaryFirst =
            ExactContactLever3D.CreateResponseOperand(
                contact.A.Body,
                linearA,
                angularA,
                exactA,
                -contact.SecondaryTangent);
        ExactContactResponseOperand3D secondarySecond =
            ExactContactLever3D.CreateResponseOperand(
                contact.B.Body,
                linearB,
                angularB,
                exactB,
                contact.SecondaryTangent);
        if (!ExactContactResponseKernel.TryGetCoulombDiskResponse(
                contact.Normal,
                normalImpulseScalar,
                primaryFirst,
                primarySecond,
                contact.Tangent,
                contact.CachedTangentImpulse,
                secondaryFirst,
                secondarySecond,
                contact.SecondaryTangent,
                contact.CachedSecondaryTangentImpulse,
                contact.StaticFriction,
                contact.DynamicFriction,
                out ExactCoulombResponse3D response))
        {
            tangentImpulse = default;
            secondaryTangentImpulse = default;
            return false;
        }

        _ = response.TryGetPrimaryAccumulatedImpulse(out tangentImpulse);
        _ = response.TryGetSecondaryAccumulatedImpulse(
            out secondaryTangentImpulse);
        return !response.HasAppliedImpulse
            || TryApplyVelocityDeltas(
                contact,
                response.FirstLinearVelocityDelta,
                response.FirstAngularVelocityDelta,
                response.SecondLinearVelocityDelta,
                response.SecondAngularVelocityDelta);
    }

    private static bool TryGetCompactTangentImpulseDelta(
        SolverContact contact,
        Vector3d relativeVelocity,
        Vector3d tangent,
        out Fixed64 impulseDelta)
    {
        impulseDelta = Fixed64.Zero;
        if (!ContactResponseArithmetic3D.TryDot(
                relativeVelocity,
                tangent,
                out Fixed64 tangentVelocity))
        {
            return false;
        }

        if (tangentVelocity >= -Fixed64.Epsilon
            && tangentVelocity <= Fixed64.Epsilon)
        {
            return true;
        }

        bool denominatorsResolved =
            ContactNormalImpulse3D.TryComputeAngularDenominator(
                contact.A.Body,
                contact.RelativeA.Vector,
                tangent,
                out Fixed64 angularA);
        denominatorsResolved &=
            ContactNormalImpulse3D.TryComputeAngularDenominator(
                contact.B.Body,
                contact.RelativeB.Vector,
                tangent,
                out Fixed64 angularB);
        denominatorsResolved &= TryGetCompactConstrainedInverseMass(
            contact.A,
            tangent,
            out Fixed64 linearA);
        denominatorsResolved &= TryGetCompactConstrainedInverseMass(
            contact.B,
            tangent,
            out Fixed64 linearB);
        var denominatorTerms = new ContactEffectiveMassTerms3D(
            linearA,
            linearB,
            angularA,
            angularB);
        bool denominatorResolved = denominatorsResolved
            & denominatorTerms.TryGetValue(out Fixed64 denominator);
        if (!denominatorResolved)
        {
            return false;
        }

        if (denominator <= Fixed64.Epsilon)
            return true;

        return Fixed64.TryMultiplyDivide(
                tangentVelocity,
                -Fixed64.One,
                denominator,
                out impulseDelta)
            && impulseDelta != Fixed64.Zero;
    }

    private static bool TryGetCompactConstrainedInverseMass(
        ResponseBody body,
        Vector3d axis,
        out Fixed64 inverseMass)
    {
        inverseMass = body.GetConstrainedInverseMass(axis);
        return inverseMass != Fixed64.Zero
            || body.InverseMass == Fixed64.Zero
            || body.Body.ProjectLinearMotion(axis) == Vector3d.Zero;
    }

    private static bool TryGetFrictionLimit(
        Fixed64 normalImpulse,
        Fixed64 friction,
        out Fixed64 limit)
    {
        if (normalImpulse <= Fixed64.Zero || friction <= Fixed64.Zero)
        {
            limit = Fixed64.Zero;
            return true;
        }

        return Fixed64.TryMultiplyDivide(
                normalImpulse,
                friction,
                Fixed64.One,
                out limit)
            && limit != Fixed64.Zero;
    }

    private static bool TryGetMagnitudeSquared(
        Fixed64 first,
        Fixed64 second,
        out Fixed64 result)
    {
        bool resolved = TryGetSquare(first, out Fixed64 firstSquared)
            & TryGetSquare(second, out Fixed64 secondSquared);
        if (!resolved)
        {
            result = default;
            return false;
        }

        return Fixed64.TryAdd(
            firstSquared,
            secondSquared,
            out result);
    }

    private static bool TryGetSquare(Fixed64 value, out Fixed64 square) =>
        Fixed64.TryMultiplyDivide(
            value,
            value,
            Fixed64.One,
            out square)
        && (value == Fixed64.Zero || square != Fixed64.Zero);

}
