//=======================================================================
// CollisionResponseMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solves deterministic mixed 3D/2D contact response with the 2D body constrained to its X/Z plane.
/// </summary>
public static class CollisionResponseMixed
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = (Fixed64)0.8f;

    internal static bool Resolve(CollisionPairMixed pair, MixedContact contact) =>
        Resolve(pair, contact, iteration: 0, iterationLimit: 1, applyPositionCorrection: true);

    internal static bool Resolve(
        CollisionPairMixed pair,
        MixedContact contact,
        int iteration,
        int iterationLimit,
        bool applyPositionCorrection)
    {
        if (!contact.HasContact || pair.Collider3D.IsTrigger || pair.Collider2D.IsTrigger)
            return false;

        SolidBody? body3D = pair.Collider3D.Body;
        SolidBody2D? body2D = pair.Collider2D.Body;
        Vector3d normal = ResolveNormal(pair, contact);
        bool hasPlanarResponseCoupling = normal.ToVector2d() != Vector2d.Zero;

        Fixed64 inverseMass3D = body3D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 inverseMass2D = body2D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 correctionInverseMass = GetConstrainedInverseMass(body3D, normal)
            + GetConstrainedPlanarInverseMass(body2D, normal);

        Vector2d relative2D = default;
        bool resolved3D = body3D != null
            ? body3D.TryGetOffsetFromCenterOfMass(contact.Anchor3D, out Vector3d relative3D)
            : contact.Anchor3D.TryGetOffsetFrom(pair.Collider3D.Center, out relative3D);
        if (!resolved3D
            || (body2D != null
                && !contact.TryGetPlanarOffset2DFrom(
                    body2D.Position,
                    body2D.Rotation,
                    body2D.LocalCenterOfMassOffset,
                    out relative2D)))
        {
            GravitasLogger.Channel.Error(
                $"Mixed contact for colliders {pair.Collider3DId}/{pair.Collider2DId} cannot be rebased onto its response centers.");
            return false;
        }
        if (body2D == null)
            relative2D = Vector2d.Zero;
        Fixed64 inverseMoment2D = body2D?.EffectiveInverseMomentOfInertia ?? Fixed64.Zero;

        if (applyPositionCorrection && correctionInverseMass > Fixed64.Zero)
            ApplyPositionCorrection(body3D, body2D, normal, contact.Depth, correctionInverseMass);

        Fixed64 normalImpulse = ApplyNormalImpulse(
            pair,
            contact,
            body3D,
            body2D,
            normal,
            relative3D,
            relative2D,
            iteration,
            iterationLimit);

        PhysicsMaterial material3D = contact.HasMaterialOverride
            ? contact.Material3D
            : pair.Collider3D.Material;
        PhysicsMaterial material2D = contact.HasMaterialOverride
            ? contact.Material2D
            : pair.Collider2D.Material;
        bool appliedImpulse = normalImpulse > Fixed64.Zero;
        appliedImpulse |= ApplyFrictionImpulse(
            body3D,
            body2D,
            normal,
            relative3D,
            relative2D,
            inverseMass3D,
            inverseMass2D,
            inverseMoment2D,
            material3D,
            material2D,
            normalImpulse,
            hasPlanarResponseCoupling);

        return appliedImpulse;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool HasPlanarResponseCoupling(CollisionPairMixed pair, MixedContact contact) =>
        ResolveNormal(pair, contact).ToVector2d() != Vector2d.Zero;

    private static void ApplyPositionCorrection(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Fixed64 depth,
        Fixed64 effectiveInverseMass)
    {
        Fixed64 correctionDepth = depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 correctionScalar = correctionDepth * PenetrationCorrectionPercent / effectiveInverseMass;
        Fixed64 constrainedInverseMass3D = GetConstrainedInverseMass(body3D, normal);
        if (constrainedInverseMass3D > Fixed64.Zero)
            body3D!.ApplyCollisionPositionCorrection(-normal * (correctionScalar * constrainedInverseMass3D));

        Fixed64 constrainedInverseMass2D = GetConstrainedPlanarInverseMass(body2D, normal);
        if (constrainedInverseMass2D <= Fixed64.Zero)
            return;

        Vector2d planarNormal = normal.ToVector2d();
        body2D!.ApplyCollisionPositionCorrection(planarNormal * (correctionScalar * constrainedInverseMass2D));
    }

    private static Fixed64 ApplyNormalImpulse(
        CollisionPairMixed pair,
        MixedContact contact,
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Vector2d relative2D,
        int iteration,
        int iterationLimit)
    {
        PhysicsMaterial material3D = contact.HasMaterialOverride
            ? contact.Material3D
            : pair.Collider3D.Material;
        PhysicsMaterial material2D = contact.HasMaterialOverride
            ? contact.Material2D
            : pair.Collider2D.Material;
        ContactNormalImpulseResultMixed result = ContactNormalImpulseMixed.CalculateAccumulatedDelta(
            body3D,
            body3D == null ? Vector3d.Zero : ResolveLinearVelocity(body3D),
            body3D?.AngularVelocity ?? Vector3d.Zero,
            relative3D,
            body2D,
            body2D == null ? Vector2d.Zero : ResolveLinearVelocity(body2D),
            body2D?.AngularVelocity ?? Fixed64.Zero,
            relative2D,
            normal,
            PhysicsMaterial.CombineRestitution(material3D, material2D),
            pair.Context.Settings.RestitutionVelocityThreshold,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        if (result.ImpulseScalar <= Fixed64.Zero)
            return Fixed64.Zero;

        pair.Context.Diagnostics.EmitMixedResponseImpulse(
            pair,
            contact,
            normal * result.ImpulseScalar,
            result.NormalVelocity,
            iteration,
            iterationLimit);
        ApplyNormalVelocityDeltas(body3D, body2D, result);
        return result.ImpulseScalar;
    }

    private static void ApplyNormalVelocityDeltas(
        SolidBody? body3D,
        SolidBody2D? body2D,
        ContactNormalImpulseResultMixed result)
    {
        if (body3D != null)
        {
            body3D.ApplyCollisionLinearVelocityDelta(result.LinearVelocityDelta3D);
            body3D.ApplyCollisionAngularVelocityDelta(result.AngularVelocityDelta3D);
        }

        if (body2D == null)
            return;

        body2D.ApplyCollisionLinearVelocityDelta(result.LinearVelocityDelta2D);
        body2D.ApplyCollisionAngularVelocityDelta(result.AngularVelocityDelta2D);
    }

    private static bool ApplyFrictionImpulse(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Vector2d relative2D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 inverseMoment2D,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D,
        Fixed64 normalImpulse,
        bool applyTo2D)
    {
        if (normalImpulse <= Fixed64.Zero)
            return false;

        PhysicsMaterial.CombineFriction(material3D, material2D, out Fixed64 staticFriction, out Fixed64 dynamicFriction);
        if (staticFriction <= Fixed64.Zero && dynamicFriction <= Fixed64.Zero)
            return false;

        Vector3d relativeVelocity = ComputeRelativeVelocity(body3D, body2D, relative3D, relative2D);
        Vector3d tangentVelocity = relativeVelocity - normal * Vector3d.Dot(relativeVelocity, normal);
        if (tangentVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Vector3d tangent = tangentVelocity.Normalized;
        Fixed64 denominator = GetConstrainedInverseMass(body3D, tangent)
            + ComputeAngularDenominator(body3D, relative3D, tangent);
        if (applyTo2D)
        {
            denominator += GetConstrainedPlanarInverseMass(body2D, tangent)
                + ComputePlanarAngularDenominator(relative2D, tangent.ToVector2d(), inverseMoment2D);
        }
        if (denominator <= Fixed64.Epsilon)
            return false;

        Fixed64 tangentVelocityMagnitude = Vector3d.Dot(relativeVelocity, tangent);
        Fixed64 impulseScalar = -tangentVelocityMagnitude / denominator;
        Fixed64 staticLimit = normalImpulse * staticFriction;
        if (impulseScalar.Abs() > staticLimit)
        {
            Fixed64 dynamicLimit = normalImpulse * dynamicFriction;
            impulseScalar = FixedMath.Clamp(impulseScalar, -dynamicLimit, dynamicLimit);
        }
        if (impulseScalar == Fixed64.Zero)
            return false;

        ApplyImpulse(
            body3D,
            body2D,
            tangent,
            relative3D,
            relative2D,
            inverseMass3D,
            inverseMass2D,
            inverseMoment2D,
            impulseScalar,
            applyTo2D);
        return true;
    }

    private static void ApplyImpulse(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d axis,
        Vector3d relative3D,
        Vector2d relative2D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 inverseMoment2D,
        Fixed64 impulseScalar,
        bool applyTo2D)
    {
        if (body3D != null)
        {
            Vector3d impulse3D = -axis * impulseScalar;
            body3D.ApplyCollisionLinearVelocityDelta(impulse3D * inverseMass3D);
            Vector3d angularVelocityDelta =
                body3D.ApplyConstrainedInverseInertia(Vector3d.Cross(relative3D, impulse3D));
            body3D.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
        }

        if (!applyTo2D || body2D == null)
            return;

        Vector2d planarAxis = axis.ToVector2d();
        Vector2d planarImpulse = planarAxis * impulseScalar;
        body2D.ApplyCollisionLinearVelocityDelta(planarImpulse * inverseMass2D);
        body2D.ApplyCollisionAngularVelocityDelta(
            Vector2d.CrossProduct(relative2D, planarImpulse) * inverseMoment2D);
    }

    private static Vector3d ComputeRelativeVelocity(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d relative3D,
        Vector2d relative2D)
    {
        Vector3d velocity3D = body3D == null
            ? Vector3d.Zero
            : ResolveLinearVelocity(body3D)
                + Vector3d.Cross(ResolveAngularVelocity(body3D), relative3D);
        Vector3d velocity2D = body2D == null
            ? Vector3d.Zero
            : (ResolveLinearVelocity(body2D)
                + AngularVelocityAtPoint(relative2D, ResolveAngularVelocity(body2D)))
                .ToVector3d(Fixed64.Zero);
        return velocity2D - velocity3D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveLinearVelocity(SolidBody body) =>
        body.ProjectLinearMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
                : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveLinearVelocity(SolidBody2D body) =>
        body.ProjectLinearMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
                : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveAngularVelocity(SolidBody body) =>
        body.ProjectAngularMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
                : body.AngularVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveAngularVelocity(SolidBody2D body) =>
        body.IsKinematic
            ? body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
            : body.AngularVelocity;

    private static Fixed64 ComputeAngularDenominator(SolidBody? body3D, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (body3D == null)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body3D!.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Vector3d angular = Vector3d.Cross(angularVelocityDelta, relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return FixedMath.Max(denominator, Fixed64.Zero);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputePlanarAngularDenominator(
        Vector2d relativePoint,
        Vector2d axis,
        Fixed64 inverseMoment)
    {
        if (inverseMoment <= Fixed64.Zero || axis == Vector2d.Zero)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativePoint, axis);
        return cross * cross * inverseMoment;
    }

    private static Vector3d ResolveNormal(CollisionPairMixed pair, MixedContact contact)
    {
        Vector3d fallback = ResolveFallbackDirection(pair, contact);
        if (contact.Normal3DTo2D.MagnitudeSquared > Fixed64.Epsilon)
        {
            Vector3d normal = contact.Normal3DTo2D.Normalized;
            return fallback.MagnitudeSquared > Fixed64.Epsilon && Vector3d.Dot(normal, fallback) < Fixed64.Zero
                ? -normal
                : normal;
        }

        return fallback == Vector3d.Zero ? Vector3d.Up : fallback;
    }

    private static Vector3d ResolveFallbackDirection(CollisionPairMixed pair, MixedContact contact)
    {
        LSCollider2D collider2D = pair.Collider2D;
        Vector3d embeddedCenter = new(
            collider2D.Center.X,
            collider2D.MixedSlabCenterY,
            collider2D.Center.Y);
        Vector3d centerDirection = embeddedCenter - pair.Collider3D.Center;
        if (centerDirection.MagnitudeSquared > Fixed64.Epsilon)
            return centerDirection.Normalized;

        if (contact.TryGetPoint2D(out Vector3d point2D)
            && contact.TryGetPoint3D(out Vector3d point3D))
        {
            Vector3d pointDirection = point2D - point3D;
            if (pointDirection.MagnitudeSquared > Fixed64.Epsilon)
                return pointDirection.Normalized;
        }

        return Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody? body, Vector3d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedPlanarInverseMass(SolidBody2D? body, Vector3d axis)
    {
        if (body == null)
            return Fixed64.Zero;

        Vector2d planarAxis = axis.ToVector2d();
        if (planarAxis == Vector2d.Zero)
            return Fixed64.Zero;

        return body.GetConstrainedInverseMass(planarAxis) * planarAxis.MagnitudeSquared;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

}
