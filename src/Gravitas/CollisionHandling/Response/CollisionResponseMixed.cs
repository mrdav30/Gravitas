using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Solves deterministic mixed 3D/2D contact response with the 2D body constrained to its X/Z plane.
/// </summary>
public static class CollisionResponseMixed
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = (Fixed64)0.8f;

    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;

    internal static void Resolve(CollisionPairMixed pair, MixedContact contact)
    {
        if (!contact.HasContact || pair.Collider3D.IsTrigger || pair.Collider2D.IsTrigger)
            return;

        StiffBody? body3D = pair.Collider3D.Body;
        StiffBody2D? body2D = pair.Collider2D.Body;
        Fixed64 inverseMass3D = GetInverseMass(body3D);
        Fixed64 inverseMass2D = GetInverseMass(body2D);
        if (inverseMass3D + inverseMass2D <= Fixed64.Zero)
            return;

        Vector3d normal = ResolveNormal(pair, contact);
        if (normal == Vector3d.Zero)
            return;

        Fixed64 planarScaleSquared = GetPlanarScaleSquared(normal);
        Fixed64 effectiveInverseMass = inverseMass3D + inverseMass2D * planarScaleSquared;
        if (effectiveInverseMass <= Fixed64.Zero)
            return;

        Vector3d relative3D = contact.Point3D - pair.Collider3D.Center;
        ApplyPositionCorrection(body3D, body2D, normal, contact.Depth, inverseMass3D, inverseMass2D, effectiveInverseMass);
        Fixed64 normalImpulse = ApplyNormalImpulse(
            pair,
            contact,
            body3D,
            body2D,
            normal,
            relative3D,
            inverseMass3D,
            inverseMass2D,
            effectiveInverseMass);
        ApplyFrictionImpulse(body3D, body2D, normal, relative3D, inverseMass3D, inverseMass2D, normalImpulse);
    }

    private static void ApplyPositionCorrection(
        StiffBody? body3D,
        StiffBody2D? body2D,
        Vector3d normal,
        Fixed64 depth,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 effectiveInverseMass)
    {
        Fixed64 correctionDepth = depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 correctionScalar = correctionDepth * PenetrationCorrectionPercent / effectiveInverseMass;
        if (inverseMass3D > Fixed64.Zero)
            body3D?.ApplyCollisionPositionCorrection(-normal * (correctionScalar * inverseMass3D));

        if (inverseMass2D <= Fixed64.Zero)
            return;

        Vector2d planarNormal = ToPlanar(normal);
        if (planarNormal == Vector2d.Zero)
            return;

        body2D?.ApplyCollisionPositionCorrection(planarNormal * (correctionScalar * inverseMass2D));
    }

    private static Fixed64 ApplyNormalImpulse(
        CollisionPairMixed pair,
        MixedContact contact,
        StiffBody? body3D,
        StiffBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 effectiveInverseMass)
    {
        Vector3d relativeVelocity = ComputeRelativeVelocity(body3D, body2D, relative3D);
        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 denominator = effectiveInverseMass + ComputeAngularDenominator(body3D, relative3D, normal);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 restitution = ResolveRestitution(body3D, body2D, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / denominator;
        if (impulseScalar <= Fixed64.Zero)
            return Fixed64.Zero;

        pair.Context.Diagnostics.EmitMixedResponseImpulse(pair, contact, normal * impulseScalar, normalVelocity);
        ApplyImpulse(body3D, body2D, normal, relative3D, inverseMass3D, inverseMass2D, impulseScalar);
        return impulseScalar;
    }

    private static void ApplyFrictionImpulse(
        StiffBody? body3D,
        StiffBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 normalImpulse)
    {
        if (normalImpulse <= Fixed64.Zero)
            return;

        Fixed64 friction = ResolveFriction(body3D, body2D);
        if (friction <= Fixed64.Zero)
            return;

        Vector3d relativeVelocity = ComputeRelativeVelocity(body3D, body2D, relative3D);
        Vector3d tangentVelocity = relativeVelocity - normal * Vector3d.Dot(relativeVelocity, normal);
        if (tangentVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        Vector3d tangent = tangentVelocity.Normalized;
        Fixed64 denominator = inverseMass3D
            + inverseMass2D * GetPlanarScaleSquared(tangent)
            + ComputeAngularDenominator(body3D, relative3D, tangent);
        if (denominator <= Fixed64.Epsilon)
            return;

        Fixed64 tangentVelocityMagnitude = Vector3d.Dot(relativeVelocity, tangent);
        Fixed64 impulseScalar = -tangentVelocityMagnitude / denominator;
        Fixed64 maxFrictionImpulse = normalImpulse * friction;
        impulseScalar = FixedMath.Clamp(impulseScalar, -maxFrictionImpulse, maxFrictionImpulse);
        if (impulseScalar == Fixed64.Zero)
            return;

        ApplyImpulse(body3D, body2D, tangent, relative3D, inverseMass3D, inverseMass2D, impulseScalar);
    }

    private static void ApplyImpulse(
        StiffBody? body3D,
        StiffBody2D? body2D,
        Vector3d axis,
        Vector3d relative3D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 impulseScalar)
    {
        if (inverseMass3D > Fixed64.Zero)
        {
            Vector3d impulse3D = -axis * impulseScalar;
            body3D?.ApplyCollisionLinearVelocityDelta(impulse3D * inverseMass3D);

            if (CanRotate(body3D))
            {
                Vector3d angularVelocityDelta =
                    body3D!.EffectiveInverseInertiaTensor * Vector3d.Cross(relative3D, impulse3D);
                body3D.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
            }
        }

        if (inverseMass2D <= Fixed64.Zero)
            return;

        Vector2d planarAxis = ToPlanar(axis);
        if (planarAxis == Vector2d.Zero)
            return;

        body2D?.ApplyCollisionLinearVelocityDelta(planarAxis * (impulseScalar * inverseMass2D));
    }

    private static Vector3d ComputeRelativeVelocity(StiffBody? body3D, StiffBody2D? body2D, Vector3d relative3D)
    {
        Vector3d velocity3D = body3D == null
            ? Vector3d.Zero
            : body3D.LinearVelocity + Vector3d.Cross(body3D.AngularVelocity, relative3D);
        Vector3d velocity2D = body2D == null
            ? Vector3d.Zero
            : new Vector3d(body2D.LinearVelocity.X, Fixed64.Zero, body2D.LinearVelocity.Y);
        return velocity2D - velocity3D;
    }

    private static Fixed64 ComputeAngularDenominator(StiffBody? body3D, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (!CanRotate(body3D))
            return Fixed64.Zero;

        Vector3d angular = Vector3d.Cross(
            body3D!.EffectiveInverseInertiaTensor * Vector3d.Cross(relativeContactPoint, axis),
            relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
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
    private static Fixed64 GetInverseMass(StiffBody? body) =>
        body?.EffectiveInverseMass ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetInverseMass(StiffBody2D? body) =>
        body != null && body.CanMove ? body.InverseMass : Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetPlanarScaleSquared(Vector3d axis) => axis.X * axis.X + axis.Z * axis.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ToPlanar(Vector3d axis) => new(axis.X, axis.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanRotate(StiffBody? body) =>
        body?.CanRotate == true;

    private static Fixed64 ResolveRestitution(StiffBody? body3D, StiffBody2D? body2D, Fixed64 closingSpeed)
    {
        if (body3D == null || body2D == null || closingSpeed <= RestitutionVelocityThreshold)
            return Fixed64.Zero;

        Fixed64 restitution = FixedMath.Min(body3D.RestitutionCoefficient, body2D.RestitutionCoefficient);
        return FixedMath.Clamp(restitution, Fixed64.Zero, Fixed64.One);
    }

    private static Fixed64 ResolveFriction(StiffBody? body3D, StiffBody2D? body2D)
    {
        if (body3D == null && body2D == null)
            return Fixed64.Zero;
        if (body3D == null)
            return body2D!.FrictionCoefficient;
        if (body2D == null)
            return body3D.FrictionCoefficient;

        Fixed64 frictionProduct = body3D.FrictionCoefficient * body2D.FrictionCoefficient;
        return frictionProduct > Fixed64.Zero
            ? FixedMath.Sqrt(frictionProduct)
            : Fixed64.Zero;
    }
}
