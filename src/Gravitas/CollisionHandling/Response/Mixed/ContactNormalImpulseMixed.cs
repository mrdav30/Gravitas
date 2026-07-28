//=======================================================================
// ContactNormalImpulseMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        : this(
            normalVelocity,
            impulseScalar,
            impulseScalar,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D)
    {
    }

    public ContactNormalImpulseResultMixed(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        AppliedImpulseScalar = appliedImpulseScalar;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
        HasRepresentableAppliedImpulse = hasRepresentableAppliedImpulse;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

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
        : this(
            normalVelocity,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D,
            normalVelocity < Fixed64.Zero,
            hasRepresentableNormalVelocity: true)
    {
    }

    public ContactNormalVelocityDeltaResultMixed(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D,
        bool isClosing,
        bool hasRepresentableNormalVelocity)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
        IsClosing = isClosing;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
    }

    public Fixed64 NormalVelocity { get; }

    public bool IsClosing { get; }

    public bool HasRepresentableNormalVelocity { get; }

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
    internal static bool CanUseCompactResponse(
        SolidBody? body3D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector3d relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        Vector2d relativeContactPoint2D,
        Vector3d axis)
    {
        Vector3d planarLever =
            ExactContactLever2D.ToSpatial(relativeContactPoint2D);
        return ContactResponseArithmetic3D.CanUseFastPointVelocity(
                linearVelocity3D,
                angularVelocity3D,
                relativeContactPoint3D,
                ExactContactLever2D.ToSpatial(linearVelocity2D),
                new Vector3d(
                    Fixed64.Zero,
                    -angularVelocity2D,
                    Fixed64.Zero),
                planarLever,
                axis)
            && ContactResponseArithmetic3D.CanUseFastAngularResponse(
                relativeContactPoint3D,
                axis,
                body3D?.GetConstrainedInverseInertiaTensor()
                    ?? Fixed3x3.Zero)
            && ContactResponseArithmetic3D.CanUseFastAngularResponse(
                planarLever,
                axis,
                ExactContactLever2D.CreateInverseInertia(body2D));
    }

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
        if (!TryComputeNormalVelocity(
                linearVelocity3D,
                angularVelocity3D,
                relativeContactPoint3D,
                linearVelocity2D,
                angularVelocity2D,
                relativeContactPoint2D,
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
                body3D,
                relativeContactPoint3D,
                body2D,
                relativeContactPoint2D,
                normal,
                out Fixed64 denominator))
        {
            return false;
        }
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

    internal static bool TryCalculateAccumulatedDelta(
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
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResultMixed result)
    {
        result = default;
        bool inputsValid =
            accumulatedImpulse >= Fixed64.Zero
            & positiveImpulseScale >= Fixed64.Zero
            & negativeImpulseScale >= Fixed64.Zero;
        if (!inputsValid)
            return false;
        if (!TryComputeNormalVelocity(
                linearVelocity3D,
                angularVelocity3D,
                relativeContactPoint3D,
                linearVelocity2D,
                angularVelocity2D,
                relativeContactPoint2D,
                normal,
                out Fixed64 normalVelocity)
            || !TryComputeDenominator(
                body3D,
                relativeContactPoint3D,
                body2D,
                relativeContactPoint2D,
                normal,
                out Fixed64 denominator))
        {
            return false;
        }
        if (denominator <= Fixed64.Zero)
        {
            result = ZeroImpulse(normalVelocity);
            return true;
        }

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        Fixed64 impulseScale = normalVelocity < Fixed64.Zero
            ? positiveImpulseScale
            : negativeImpulseScale;
        Fixed64 impulseScalar;
        if (!Fixed64.TryMultiplyDivide(
                normalVelocity,
                responseFactor,
                impulseScale,
                denominator,
                out Fixed64 scaledImpulse))
        {
            if (normalVelocity < Fixed64.Zero)
                return false;
            impulseScalar = -accumulatedImpulse;
        }
        else if (!Fixed64.TryAdd(
                    accumulatedImpulse,
                    scaledImpulse,
                    out Fixed64 accumulated)
                || !Fixed64.TrySubtract(
                    FixedMath.Max(Fixed64.Zero, accumulated),
                    accumulatedImpulse,
                    out impulseScalar))
        {
            return false;
        }
        if (impulseScalar == Fixed64.Zero)
        {
            result = ZeroImpulse(normalVelocity);
            return true;
        }

        Vector2d planarNormal = normal.ToVector2d();
        bool impulse3DResolved = ContactResponseArithmetic3D.TryScale(
            -normal,
            impulseScalar,
            out Vector3d impulse3D);
        bool linear3DResolved =
            ContactNormalImpulse3D.TryComputeLinearVelocityDelta(
                body3D,
                impulse3D,
                out Vector3d linearVelocityDelta3D);
        bool angular3DResolved =
            ContactNormalImpulse3D.TryComputeAngularVelocityDelta(
                body3D,
                relativeContactPoint3D,
                impulse3D,
                out Vector3d angularVelocityDelta3D);
        bool linear2DResolved =
            ContactNormalImpulse2D.TryComputeLinearVelocityDelta(
                body2D,
                planarNormal,
                impulseScalar,
                out Vector2d linearVelocityDelta2D);
        bool angular2DResolved =
            ContactNormalImpulse2D.TryComputeAngularVelocityDelta(
                body2D,
                relativeContactPoint2D,
                planarNormal,
                impulseScalar,
                out Fixed64 angularVelocityDelta2D);
        if (!(impulse3DResolved
            & linear3DResolved
            & angular3DResolved
            & linear2DResolved
            & angular2DResolved))
        {
            return false;
        }

        result = new ContactNormalImpulseResultMixed(
            normalVelocity,
            impulseScalar,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D);
        return true;
    }

    internal static bool TryCalculateVelocityDeltasExact(
        SolidBody? body3D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        in FixedLever relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        in FixedLever relativeContactPoint2D,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResultMixed result)
    {
        result = default;
        FixedLeverResponseOperand3d first =
            ExactContactLever3D.CreateResponseOperand(
                body3D,
                linearVelocity3D,
                angularVelocity3D,
                relativeContactPoint3D,
                -normal);
        FixedLeverResponseOperand3d second =
            ExactContactLever2D.CreateResponseOperand(
                body2D,
                linearVelocity2D,
                angularVelocity2D,
                relativeContactPoint2D,
                normal);
        if (!FixedLever.TryGetNormalResponse(
                first,
                second,
                normal,
                restitution,
                restitutionVelocityThreshold,
                out FixedLeverNormalResponse3d response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        result = new ContactNormalVelocityDeltaResultMixed(
            normalVelocity,
            response.FirstLinearVelocityDelta,
            response.FirstAngularVelocityDelta,
            ExactContactLever2D.ToPlanar(response.SecondLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.SecondAngularVelocityDelta),
            response.IsClosing,
            hasNormalVelocity);
        return true;
    }

    internal static bool TryCalculateAccumulatedDeltaExact(
        SolidBody? body3D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        in FixedLever relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        in FixedLever relativeContactPoint2D,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResultMixed result)
    {
        result = default;
        FixedLeverResponseOperand3d first =
            ExactContactLever3D.CreateResponseOperand(
                body3D,
                linearVelocity3D,
                angularVelocity3D,
                relativeContactPoint3D,
                -normal);
        FixedLeverResponseOperand3d second =
            ExactContactLever2D.CreateResponseOperand(
                body2D,
                linearVelocity2D,
                angularVelocity2D,
                relativeContactPoint2D,
                normal);
        if (!FixedLever.TryGetAccumulatedNormalResponse(
                first,
                second,
                normal,
                restitution,
                restitutionVelocityThreshold,
                accumulatedImpulse,
                positiveImpulseScale,
                negativeImpulseScale,
                out FixedLeverNormalResponse3d response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        bool hasAppliedImpulse =
            response.TryGetAppliedImpulse(out Fixed64 appliedImpulse);
        Fixed64 impulseScalar = response.TryGetAccumulatedImpulse(
                out Fixed64 newAccumulatedImpulse)
            ? newAccumulatedImpulse - accumulatedImpulse
            : -accumulatedImpulse;
        result = new ContactNormalImpulseResultMixed(
            normalVelocity,
            impulseScalar,
            appliedImpulse,
            response.FirstLinearVelocityDelta,
            response.FirstAngularVelocityDelta,
            ExactContactLever2D.ToPlanar(response.SecondLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.SecondAngularVelocityDelta),
            hasNormalVelocity,
            hasAppliedImpulse);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryComputeNormalVelocity(
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector3d relativeContactPoint3D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal,
        out Fixed64 normalVelocity) =>
        ContactNormalImpulse3D.TryComputeNormalVelocity(
            linearVelocity3D,
            angularVelocity3D,
            relativeContactPoint3D,
            ExactContactLever2D.ToSpatial(linearVelocity2D),
            new Vector3d(Fixed64.Zero, -angularVelocity2D, Fixed64.Zero),
            ExactContactLever2D.ToSpatial(relativeContactPoint2D),
            normal,
            out normalVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryComputeDenominator(
        SolidBody? body3D,
        Vector3d relativeContactPoint3D,
        SolidBody2D? body2D,
        Vector2d relativeContactPoint2D,
        Vector3d normal,
        out Fixed64 denominator)
    {
        Vector2d planarNormal = normal.ToVector2d();
        bool angular3DResolved =
            ContactNormalImpulse3D.TryComputeAngularDenominator(
                body3D,
                relativeContactPoint3D,
                normal,
                out Fixed64 angular3D);
        bool angular2DResolved =
            ContactNormalImpulse2D.TryComputeAngularDenominator(
                body2D,
                relativeContactPoint2D,
                planarNormal,
                out Fixed64 angular2D);
        bool sumResolved = Fixed64.TryAdd(
                GetConstrainedInverseMass3D(body3D, normal),
                GetConstrainedInverseMass2D(body2D, normal),
                out Fixed64 linear)
            & Fixed64.TryAdd(linear, angular3D, out Fixed64 first)
            & Fixed64.TryAdd(first, angular2D, out denominator);
        if (!(angular3DResolved & angular2DResolved & sumResolved))
        {
            denominator = default;
            return false;
        }

        return true;
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
