using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Constants for the alpha pure 2D contact response path.
/// </summary>
public static class CollisionResponse2D
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = Fixed64.One;

    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;

    internal static void Resolve(CollisionPair2D pair, Contact2D contact)
    {
        if (!contact.HasContact)
            return;

        StiffBody2D? bodyA = pair.ColliderA.Body;
        StiffBody2D? bodyB = pair.ColliderB.Body;
        if (bodyA == null && bodyB == null)
            return;

        Fixed64 inverseMassA = bodyA?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 inverseMassB = bodyB?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 totalInverseMass = inverseMassA + inverseMassB;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector2d normal = ResolveNormal(pair.ColliderA, pair.ColliderB, contact.Normal);
        if (normal == Vector2d.Zero)
            return;

        Vector2d relativeA = bodyA == null
            ? Vector2d.Zero
            : contact.PointA - bodyA.WorldCenterOfMass;
        Vector2d relativeB = bodyB == null
            ? Vector2d.Zero
            : contact.PointB - bodyB.WorldCenterOfMass;
        Fixed64 inverseMomentA = bodyA?.EffectiveInverseMomentOfInertia ?? Fixed64.Zero;
        Fixed64 inverseMomentB = bodyB?.EffectiveInverseMomentOfInertia ?? Fixed64.Zero;

        ApplyPositionCorrection(bodyA, bodyB, normal, contact.Depth, inverseMassA, inverseMassB, totalInverseMass);
        Fixed64 normalImpulse = ApplyNormalImpulse(
            bodyA,
            bodyB,
            normal,
            relativeA,
            relativeB,
            inverseMassA,
            inverseMassB,
            inverseMomentA,
            inverseMomentB);
        ApplyFrictionImpulse(
            bodyA,
            bodyB,
            normal,
            relativeA,
            relativeB,
            inverseMassA,
            inverseMassB,
            inverseMomentA,
            inverseMomentB,
            normalImpulse);
    }

    private static void ApplyPositionCorrection(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d normal,
        Fixed64 depth,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 totalInverseMass)
    {
        Fixed64 correctionDepth = depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Vector2d correction = normal * (correctionDepth * PenetrationCorrectionPercent / totalInverseMass);
        bodyA?.ApplyCollisionPositionCorrection(-correction * inverseMassA);
        bodyB?.ApplyCollisionPositionCorrection(correction * inverseMassB);
    }

    private static Fixed64 ApplyNormalImpulse(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d normal,
        Vector2d relativeA,
        Vector2d relativeB,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 inverseMomentA,
        Fixed64 inverseMomentB)
    {
        Vector2d relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, relativeA, relativeB);
        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
        if (normalVelocity >= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 denominator =
            inverseMassA
            + inverseMassB
            + ComputeAngularDenominator(relativeA, normal, inverseMomentA)
            + ComputeAngularDenominator(relativeB, normal, inverseMomentB);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 restitution = ResolveRestitution(bodyA, bodyB, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / denominator;
        if (impulseScalar <= Fixed64.Zero)
            return Fixed64.Zero;

        ApplyImpulse(
            bodyA,
            bodyB,
            normal * impulseScalar,
            relativeA,
            relativeB,
            inverseMassA,
            inverseMassB,
            inverseMomentA,
            inverseMomentB);
        return impulseScalar;
    }

    private static void ApplyFrictionImpulse(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d normal,
        Vector2d relativeA,
        Vector2d relativeB,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 inverseMomentA,
        Fixed64 inverseMomentB,
        Fixed64 normalImpulse)
    {
        if (normalImpulse <= Fixed64.Zero)
            return;

        Fixed64 frictionCoefficient = ResolveFrictionCoefficient(bodyA, bodyB, inverseMassA, inverseMassB);
        if (frictionCoefficient <= Fixed64.Zero)
            return;

        Vector2d relativeVelocity = ComputeRelativeVelocity(bodyA, bodyB, relativeA, relativeB);
        Vector2d tangentVelocity = relativeVelocity - normal * Vector2d.Dot(relativeVelocity, normal);
        if (tangentVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        Vector2d tangent = tangentVelocity.Normalized;
        Fixed64 denominator =
            inverseMassA
            + inverseMassB
            + ComputeAngularDenominator(relativeA, tangent, inverseMomentA)
            + ComputeAngularDenominator(relativeB, tangent, inverseMomentB);
        if (denominator <= Fixed64.Epsilon)
            return;

        Fixed64 tangentVelocityMagnitude = Vector2d.Dot(relativeVelocity, tangent);
        Fixed64 impulseScalar = -tangentVelocityMagnitude / denominator;
        Fixed64 maxFrictionImpulse = normalImpulse * frictionCoefficient;
        impulseScalar = FixedMath.Clamp(impulseScalar, -maxFrictionImpulse, maxFrictionImpulse);
        if (impulseScalar == Fixed64.Zero)
            return;

        ApplyImpulse(
            bodyA,
            bodyB,
            tangent * impulseScalar,
            relativeA,
            relativeB,
            inverseMassA,
            inverseMassB,
            inverseMomentA,
            inverseMomentB);
    }

    private static void ApplyImpulse(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d impulse,
        Vector2d relativeA,
        Vector2d relativeB,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 inverseMomentA,
        Fixed64 inverseMomentB)
    {
        if (inverseMassA > Fixed64.Zero)
            bodyA?.ApplyCollisionLinearVelocityDelta(-impulse * inverseMassA);
        if (inverseMomentA > Fixed64.Zero)
            bodyA?.ApplyCollisionAngularVelocityDelta(-Vector2d.CrossProduct(relativeA, impulse) * inverseMomentA);

        if (inverseMassB > Fixed64.Zero)
            bodyB?.ApplyCollisionLinearVelocityDelta(impulse * inverseMassB);
        if (inverseMomentB > Fixed64.Zero)
            bodyB?.ApplyCollisionAngularVelocityDelta(Vector2d.CrossProduct(relativeB, impulse) * inverseMomentB);
    }

    private static Vector2d ComputeRelativeVelocity(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d relativeA,
        Vector2d relativeB)
    {
        Vector2d velocityA = bodyA == null
            ? Vector2d.Zero
            : GetVelocityAtContact(bodyA, relativeA);
        Vector2d velocityB = bodyB == null
            ? Vector2d.Zero
            : GetVelocityAtContact(bodyB, relativeB);
        return velocityB - velocityA;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d GetVelocityAtContact(StiffBody2D body, Vector2d relativePoint) =>
        body.LinearVelocity + AngularVelocityAtPoint(relativePoint, body.AngularVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeAngularDenominator(
        Vector2d relativePoint,
        Vector2d axis,
        Fixed64 inverseMoment)
    {
        if (inverseMoment <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativePoint, axis);
        return cross * cross * inverseMoment;
    }

    private static Fixed64 ResolveRestitution(StiffBody2D? bodyA, StiffBody2D? bodyB, Fixed64 closingSpeed)
    {
        if (bodyA == null || bodyB == null || closingSpeed <= RestitutionVelocityThreshold)
            return Fixed64.Zero;

        Fixed64 restitution = FixedMath.Min(bodyA.RestitutionCoefficient, bodyB.RestitutionCoefficient);
        return FixedMath.Clamp(restitution, Fixed64.Zero, Fixed64.One);
    }

    private static Fixed64 ResolveFrictionCoefficient(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB)
    {
        if (bodyA == null && bodyB == null)
            return Fixed64.Zero;
        if (bodyA == null || inverseMassA <= Fixed64.Zero)
            return bodyB?.FrictionCoefficient ?? Fixed64.Zero;
        if (bodyB == null || inverseMassB <= Fixed64.Zero)
            return bodyA.FrictionCoefficient;

        Fixed64 frictionProduct = bodyA.FrictionCoefficient * bodyB.FrictionCoefficient;
        return frictionProduct > Fixed64.Zero
            ? FixedMath.Sqrt(frictionProduct)
            : Fixed64.Zero;
    }

    private static Vector2d ResolveNormal(LSCollider2D colliderA, LSCollider2D colliderB, Vector2d normal)
    {
        Vector2d fallback = ResolveFallbackDirection(colliderA, colliderB);
        Vector2d resolved = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : fallback;

        if (resolved == Vector2d.Zero)
            return Vector2d.Right;

        return fallback.MagnitudeSquared > Fixed64.Epsilon && Vector2d.Dot(resolved, fallback) < Fixed64.Zero
            ? -resolved
            : resolved;
    }

    private static Vector2d ResolveFallbackDirection(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        Vector2d direction = colliderB.Center - colliderA.Center;
        return direction.MagnitudeSquared > Fixed64.Epsilon
            ? direction.Normalized
            : Vector2d.Zero;
    }
}
