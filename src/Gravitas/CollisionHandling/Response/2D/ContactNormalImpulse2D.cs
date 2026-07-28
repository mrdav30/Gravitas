//=======================================================================
// ContactNormalImpulse2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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

    public ContactNormalImpulseResult2D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true,
        bool hasRepresentableAccumulatedImpulse = true)
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
        HasRepresentableAccumulatedImpulse =
            hasRepresentableAccumulatedImpulse;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

    public bool HasRepresentableAccumulatedImpulse { get; }

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

    public ContactNormalVelocityDeltaResult2D(
        Fixed64 normalVelocity,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB,
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
                out Fixed64 denominator)
            || denominator <= Fixed64.Zero)
        {
            return false;
        }

        Fixed64 appliedRestitution = normalVelocity < -restitutionVelocityThreshold
            ? restitution
            : Fixed64.Zero;
        Fixed64 responseFactor = -(Fixed64.One + appliedRestitution);
        bool linearAResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                bodyA?.ProjectLinearMotion(-normal) ?? Vector2d.Zero,
                normalVelocity,
                responseFactor,
                bodyA?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector2d linearVelocityDeltaA);
        bool angularAResolved = TryResolveAngularVelocityDelta(
                bodyA,
                relativeContactPointA,
                -normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Fixed64 angularVelocityDeltaA);
        bool linearBResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                bodyB?.ProjectLinearMotion(normal) ?? Vector2d.Zero,
                normalVelocity,
                responseFactor,
                bodyB?.EffectiveInverseMass ?? Fixed64.Zero,
                denominator,
                out Vector2d linearVelocityDeltaB);
        bool angularBResolved = TryResolveAngularVelocityDelta(
                bodyB,
                relativeContactPointB,
                normal,
                normalVelocity,
                responseFactor,
                denominator,
                out Fixed64 angularVelocityDeltaB);
        if (!(linearAResolved
                & angularAResolved
                & linearBResolved
                & angularBResolved))
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

    internal static bool TryCalculateAccumulatedDelta(
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
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResult2D result)
    {
        result = default;
        bool inputsValid =
            accumulatedImpulse >= Fixed64.Zero
            & positiveImpulseScale >= Fixed64.Zero
            & negativeImpulseScale >= Fixed64.Zero;
        if (!inputsValid)
            return false;
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
                out Fixed64 denominator))
        {
            return false;
        }
        if (denominator <= Fixed64.Zero)
        {
            result = Zero(normalVelocity);
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
            result = Zero(normalVelocity);
            return true;
        }

        bool linearAResolved = TryComputeLinearVelocityDelta(
            bodyA,
            -normal,
            impulseScalar,
            out Vector2d linearA);
        bool angularAResolved = TryComputeAngularVelocityDelta(
            bodyA,
            relativeContactPointA,
            -normal,
            impulseScalar,
            out Fixed64 angularA);
        bool linearBResolved = TryComputeLinearVelocityDelta(
            bodyB,
            normal,
            impulseScalar,
            out Vector2d linearB);
        bool angularBResolved = TryComputeAngularVelocityDelta(
            bodyB,
            relativeContactPointB,
            normal,
            impulseScalar,
            out Fixed64 angularB);
        if (!(linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved))
        {
            return false;
        }
        result = new ContactNormalImpulseResult2D(
            normalVelocity,
            impulseScalar,
            linearA,
            angularA,
            linearB,
            angularB);
        return true;
    }

    internal static bool TryCalculateVelocityDeltasExact(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        in FixedLever relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        in FixedLever relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ContactNormalVelocityDeltaResult2D result)
    {
        result = default;
        if (!ExactContactLever2D.TryGetNormalResponse(
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
                out FixedLeverNormalResponse3d response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        result = new ContactNormalVelocityDeltaResult2D(
            normalVelocity,
            ExactContactLever2D.ToPlanar(response.FirstLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.FirstAngularVelocityDelta),
            ExactContactLever2D.ToPlanar(response.SecondLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.SecondAngularVelocityDelta),
            response.IsClosing,
            hasNormalVelocity);
        return true;
    }

    internal static bool TryCalculateAccumulatedDeltaExact(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        in FixedLever relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        in FixedLever relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out ContactNormalImpulseResult2D result)
    {
        result = default;
        if (!ExactContactLever2D.TryGetAccumulatedNormalResponse(
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
                out FixedLeverNormalResponse3d response))
        {
            return false;
        }

        bool hasNormalVelocity =
            response.TryGetNormalVelocity(out Fixed64 normalVelocity);
        bool hasAppliedImpulse =
            response.TryGetAppliedImpulse(out Fixed64 appliedImpulse);
        bool hasAccumulatedImpulse =
            response.TryGetAccumulatedImpulse(
                out Fixed64 newAccumulatedImpulse);
        Fixed64 impulseScalar = hasAccumulatedImpulse
            ? newAccumulatedImpulse - accumulatedImpulse
            : -accumulatedImpulse;
        result = new ContactNormalImpulseResult2D(
            normalVelocity,
            impulseScalar,
            appliedImpulse,
            ExactContactLever2D.ToPlanar(response.FirstLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.FirstAngularVelocityDelta),
            ExactContactLever2D.ToPlanar(response.SecondLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(response.SecondAngularVelocityDelta),
            hasNormalVelocity,
            hasAppliedImpulse,
            hasAccumulatedImpulse);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody2D? body, Vector2d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryComputeNormalVelocity(
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d relativeContactPointA,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Vector2d relativeContactPointB,
        Vector2d normal,
        out Fixed64 normalVelocity)
    {
        bool angularAResolved = ContactResponseArithmetic3D.TryCross(
            new Vector3d(
                Fixed64.Zero,
                -angularVelocityA,
                Fixed64.Zero),
            ExactContactLever2D.ToSpatial(relativeContactPointA),
            out Vector3d angularA);
        bool angularBResolved = ContactResponseArithmetic3D.TryCross(
            new Vector3d(
                Fixed64.Zero,
                -angularVelocityB,
                Fixed64.Zero),
            ExactContactLever2D.ToSpatial(relativeContactPointB),
            out Vector3d angularB);
        bool relativeResolved = Vector3d.TrySubtractSums(
            ExactContactLever2D.ToSpatial(linearVelocityB),
            angularB,
            ExactContactLever2D.ToSpatial(linearVelocityA),
            angularA,
            out Vector3d relative);
        bool projectionResolved = ContactResponseArithmetic3D.TryDot(
            relative,
            ExactContactLever2D.ToSpatial(normal),
            out normalVelocity);
        if (!(angularAResolved
            & angularBResolved
            & relativeResolved
            & projectionResolved))
        {
            normalVelocity = default;
            return false;
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryComputeDenominator(
        SolidBody2D? bodyA,
        Vector2d relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d relativeContactPointB,
        Vector2d normal,
        out Fixed64 denominator)
    {
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
        bool sumResolved = Fixed64.TryAdd(
                GetConstrainedInverseMass(bodyA, normal),
                GetConstrainedInverseMass(bodyB, normal),
                out Fixed64 linear)
            & Fixed64.TryAdd(
                linear,
                angularA,
                out Fixed64 first)
            & Fixed64.TryAdd(
                first,
                angularB,
                out denominator);
        if (!(angularAResolved & angularBResolved & sumResolved))
        {
            denominator = default;
            return false;
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryComputeAngularDenominator(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d axis,
        out Fixed64 denominator)
    {
        if (body?.CanRotate != true)
        {
            denominator = Fixed64.Zero;
            return true;
        }

        denominator = default;
        return ContactResponseArithmetic3D.TryCross(
                ExactContactLever2D.ToSpatial(relativeContactPoint),
                ExactContactLever2D.ToSpatial(axis),
                out Vector3d cross)
            && Fixed64.TryMultiplyDivide(
                cross.Y,
                cross.Y,
                body.EffectiveInverseMomentOfInertia,
                Fixed64.One,
                out denominator);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryComputeLinearVelocityDelta(
        SolidBody2D? body,
        Vector2d signedNormal,
        Fixed64 impulseScalar,
        out Vector2d velocityDelta) =>
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            body?.ProjectLinearMotion(signedNormal) ?? Vector2d.Zero,
            impulseScalar,
            body?.EffectiveInverseMass ?? Fixed64.Zero,
            Fixed64.One,
            out velocityDelta);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryComputeAngularVelocityDelta(
        SolidBody2D? body,
        Vector2d relativeContactPoint,
        Vector2d signedNormal,
        Fixed64 impulseScalar,
        out Fixed64 velocityDelta)
    {
        velocityDelta = Fixed64.Zero;
        return body?.CanRotate != true
            || (ContactResponseArithmetic3D.TryCross(
                    ExactContactLever2D.ToSpatial(relativeContactPoint),
                    ExactContactLever2D.ToSpatial(signedNormal),
                    out Vector3d cross)
                && Fixed64.TryMultiplyDivide(
                    -cross.Y,
                    impulseScalar,
                    body.EffectiveInverseMomentOfInertia,
                    Fixed64.One,
                    out velocityDelta));
    }

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
