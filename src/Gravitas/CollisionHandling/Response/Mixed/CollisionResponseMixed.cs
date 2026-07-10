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

        Fixed64 inverseMass3D = body3D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 inverseMass2D = body2D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 correctionInverseMass = GetConstrainedInverseMass(body3D, normal)
            + GetConstrainedPlanarInverseMass(body2D, normal);
        if (correctionInverseMass <= Fixed64.Zero)
            return false;

        Vector3d relative3D = contact.Point3D - (body3D?.WorldCenterOfMass ?? pair.Collider3D.Center);
        Vector2d relative2D = body2D == null
            ? Vector2d.Zero
            : contact.Point2D.ToVector2d() - body2D.WorldCenterOfMass;
        Fixed64 inverseMoment2D = body2D?.EffectiveInverseMomentOfInertia ?? Fixed64.Zero;

        if (applyPositionCorrection)
            ApplyPositionCorrection(body3D, body2D, normal, contact.Depth, correctionInverseMass);

        Fixed64 normalImpulse = ApplyNormalImpulse(
            pair,
            contact,
            body3D,
            body2D,
            normal,
            relative3D,
            relative2D,
            inverseMass3D,
            inverseMass2D,
            inverseMoment2D,
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
            normalImpulse);

        return appliedImpulse;
    }

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
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 inverseMoment2D,
        int iteration,
        int iterationLimit)
    {
        Vector3d relativeVelocity = ComputeRelativeVelocity(body3D, body2D, relative3D, relative2D);
        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 denominator = GetConstrainedInverseMass(body3D, normal)
            + GetConstrainedPlanarInverseMass(body2D, normal)
            + ComputeAngularDenominator(body3D, relative3D, normal)
            + ComputePlanarAngularDenominator(relative2D, normal.ToVector2d(), inverseMoment2D);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 restitution = ResolveRestitution(
            contact.HasMaterialOverride ? contact.Material3D : pair.Collider3D.Material,
            contact.HasMaterialOverride ? contact.Material2D : pair.Collider2D.Material,
            -normalVelocity,
            pair.Context.Settings.RestitutionVelocityThreshold);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / denominator;
        pair.Context.Diagnostics.EmitMixedResponseImpulse(
            pair,
            contact,
            normal * impulseScalar,
            normalVelocity,
            iteration,
            iterationLimit);
        ApplyImpulse(body3D, body2D, normal, relative3D, relative2D, inverseMass3D, inverseMass2D, inverseMoment2D, impulseScalar);
        return impulseScalar;
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
        Fixed64 normalImpulse)
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
            + GetConstrainedPlanarInverseMass(body2D, tangent)
            + ComputeAngularDenominator(body3D, relative3D, tangent)
            + ComputePlanarAngularDenominator(relative2D, tangent.ToVector2d(), inverseMoment2D);
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

        ApplyImpulse(body3D, body2D, tangent, relative3D, relative2D, inverseMass3D, inverseMass2D, inverseMoment2D, impulseScalar);
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
        Fixed64 impulseScalar)
    {
        if (inverseMass3D > Fixed64.Zero)
        {
            Vector3d impulse3D = -axis * impulseScalar;
            body3D!.ApplyCollisionLinearVelocityDelta(impulse3D * inverseMass3D);

            if (CanRotate(body3D))
            {
                Vector3d angularVelocityDelta =
                    body3D!.ApplyConstrainedInverseInertia(Vector3d.Cross(relative3D, impulse3D));
                body3D.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
            }
        }

        if (inverseMass2D <= Fixed64.Zero)
            return;

        Vector2d planarAxis = axis.ToVector2d();
        if (planarAxis == Vector2d.Zero)
            return;

        Vector2d planarImpulse = planarAxis * impulseScalar;
        body2D!.ApplyCollisionLinearVelocityDelta(planarImpulse * inverseMass2D);

        if (inverseMoment2D > Fixed64.Zero)
            body2D.ApplyCollisionAngularVelocityDelta(Vector2d.CrossProduct(relative2D, planarImpulse) * inverseMoment2D);
    }

    private static Vector3d ComputeRelativeVelocity(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d relative3D,
        Vector2d relative2D)
    {
        Vector3d velocity3D = body3D == null
            ? Vector3d.Zero
            : ResolveLinearVelocity(body3D) + Vector3d.Cross(body3D.AngularVelocity, relative3D);
        Vector3d velocity2D = body2D == null
            ? Vector3d.Zero
            : (ResolveLinearVelocity(body2D) + AngularVelocityAtPoint(relative2D, body2D.AngularVelocity)).ToVector3d(Fixed64.Zero);
        return velocity2D - velocity3D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveLinearVelocity(SolidBody body) =>
        body.ProjectLinearMotion(body.IsKinematic ? body.ResolveContinuousCollisionFrameVelocity() : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveLinearVelocity(SolidBody2D body) =>
        body.ProjectLinearMotion(body.IsKinematic ? body.ResolveContinuousCollisionFrameVelocity() : body.LinearVelocity);

    private static Fixed64 ComputeAngularDenominator(SolidBody? body3D, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (!CanRotate(body3D))
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body3D!.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Vector3d angular = Vector3d.Cross(angularVelocityDelta, relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
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

        Vector3d pointDirection = contact.Point2D - contact.Point3D;
        return pointDirection.MagnitudeSquared > Fixed64.Epsilon
            ? pointDirection.Normalized
            : Vector3d.Zero;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanRotate(SolidBody? body) =>
        body?.CanRotate == true;

    private static Fixed64 ResolveRestitution(
        PhysicsMaterial material3D,
        PhysicsMaterial material2D,
        Fixed64 closingSpeed,
        Fixed64 restitutionVelocityThreshold)
    {
        if (closingSpeed <= restitutionVelocityThreshold)
            return Fixed64.Zero;

        return PhysicsMaterial.CombineRestitution(material3D, material2D);
    }
}
