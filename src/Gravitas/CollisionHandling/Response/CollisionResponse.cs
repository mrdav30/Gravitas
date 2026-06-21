//=======================================================================
// CollisionResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solves deterministic contact response for one collision pair manifold.
/// </summary>
public static class CollisionResponse
{
    /// <summary>
    /// Penetration depth below this value is treated as contact slop and does not
    /// produce positional correction.
    /// </summary>
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    /// <summary>
    /// Fraction of penetration above slop corrected per solver call.
    /// </summary>
    public static readonly Fixed64 PenetrationCorrectionPercent = (Fixed64)0.8f;

    /// <summary>
    /// Closing speed at or below this value is treated as resting contact and
    /// uses zero restitution to avoid small deterministic bounces.
    /// </summary>
    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;

    /// <summary>
    /// Applies positional correction, normal impulses, and Coulomb friction for
    /// the collision pair's current deterministic contact manifold.
    /// </summary>
    public static void CalculateImpulse(CollisionPair pair)
    {
        if (!TryCreateBodyPair(pair, out ResponseBody bodyA, out ResponseBody bodyB))
            return;

        SolverContactBuffer contacts = BuildContactBuffer(pair, bodyA, bodyB);
        if (contacts.Count == 0)
            return;

        Fixed64 contactShare = Fixed64.One / (Fixed64)contacts.Count;
        for (int i = 0; i < contacts.Count; i++)
            ApplyPositionCorrection(contacts.GetContact(i), contactShare);

        for (int i = 0; i < contacts.Count; i++)
        {
            Fixed64 impulseScalar = ComputeNormalImpulseScalar(
                contacts.GetContact(i),
                out Fixed64 normalVelocity);
            contacts.SetNormalImpulse(i, impulseScalar * contactShare, normalVelocity);
        }

        for (int i = 0; i < contacts.Count; i++)
            ApplyNormalImpulse(
                pair,
                contacts.GetContact(i),
                contacts.GetNormalImpulse(i),
                contacts.GetNormalVelocity(i));

        for (int i = 0; i < contacts.Count; i++)
            contacts.SetTangentImpulse(i, ApplyFrictionImpulse(contacts.GetContact(i), contacts.GetNormalImpulse(i)));

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            pair.StoreWarmStartImpulse(
                contact.ContactId,
                contacts.GetNormalImpulse(i),
                contacts.GetTangentImpulse(i));
        }
    }

    private static bool TryCreateBodyPair(CollisionPair pair, out ResponseBody bodyA, out ResponseBody bodyB)
    {
        bodyA = default;
        bodyB = default;

        if (pair.ColliderA.IsTrigger || pair.ColliderB.IsTrigger)
            return false;

        if (pair.ColliderA.Body == null || pair.ColliderB.Body == null)
            return false;

        if (!pair.Manifold.HasContact)
            return false;

        bodyA = ResponseBody.Create(pair.ColliderA);
        bodyB = ResponseBody.Create(pair.ColliderB);
        return bodyA.InverseMass + bodyB.InverseMass > Fixed64.Zero;
    }

    private static SolverContactBuffer BuildContactBuffer(CollisionPair pair, ResponseBody bodyA, ResponseBody bodyB)
    {
        SolverContactBuffer contacts = default;
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            if (TryCreateContact(pair, bodyA, bodyB, i, out SolverContact contact))
                contacts.Add(contact);
        }

        return contacts;
    }

    private static bool TryCreateContact(
        CollisionPair pair,
        ResponseBody bodyA,
        ResponseBody bodyB,
        int contactIndex,
        out SolverContact contact)
    {
        contact = default;
        ManifoldContact manifoldContact = pair.Manifold[contactIndex];
        Vector3d normal = ResolveContactNormal(manifoldContact.Normal, pair.ColliderB.Center - pair.ColliderA.Center);
        if (normal == Vector3d.Zero)
            return false;

        contact = new SolverContact(
            manifoldContact.ContactId,
            bodyA,
            bodyB,
            manifoldContact.PointA,
            manifoldContact.PointB,
            manifoldContact.PointA - bodyA.Body.WorldCenterOfMass,
            manifoldContact.PointB - bodyB.Body.WorldCenterOfMass,
            manifoldContact.Depth,
            normal);
        return true;
    }

    private static void ApplyPositionCorrection(SolverContact contact, Fixed64 contactShare)
    {
        Fixed64 correctionDepth = contact.Depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 totalInverseMass = contact.TotalInverseMass;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector3d correction = contact.Normal
            * (correctionDepth * PenetrationCorrectionPercent * contactShare / totalInverseMass);
        contact.A.Body.ApplyCollisionPositionCorrection(-correction * contact.A.InverseMass);
        contact.B.Body.ApplyCollisionPositionCorrection(correction * contact.B.InverseMass);
    }

    private static Fixed64 ComputeNormalImpulseScalar(SolverContact contact, out Fixed64 normalVelocity)
    {
        normalVelocity = Vector3d.Dot(ComputeRelativeVelocity(contact), contact.Normal);
        if (normalVelocity >= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 denominator = ComputeImpulseDenominator(contact, contact.Normal);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 restitution = ResolveRestitution(contact, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / denominator;
        return impulseScalar > Fixed64.Zero ? impulseScalar : Fixed64.Zero;
    }

    private static void ApplyNormalImpulse(
        CollisionPair pair,
        SolverContact contact,
        Fixed64 impulseScalar,
        Fixed64 normalVelocity)
    {
        if (impulseScalar <= Fixed64.Zero)
            return;

        Vector3d impulse = contact.Normal * impulseScalar;
        pair.Context.Diagnostics.EmitResponseImpulse(pair, impulse, normalVelocity);
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
    }

    private static Fixed64 ApplyFrictionImpulse(SolverContact contact, Fixed64 normalImpulseScalar)
    {
        if (normalImpulseScalar <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 frictionCoefficient = ResolveFrictionCoefficient(contact);
        if (frictionCoefficient <= Fixed64.Zero)
            return Fixed64.Zero;

        Vector3d relativeVelocity = ComputeRelativeVelocity(contact);
        Vector3d tangentVelocity = relativeVelocity - contact.Normal * Vector3d.Dot(relativeVelocity, contact.Normal);
        if (tangentVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Vector3d tangent = tangentVelocity.Normalized;
        Fixed64 denominator = ComputeImpulseDenominator(contact, tangent);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 tangentVelocityMagnitude = Vector3d.Dot(relativeVelocity, tangent);
        Fixed64 impulseScalar = -tangentVelocityMagnitude / denominator;
        Fixed64 maxFrictionImpulse = normalImpulseScalar * frictionCoefficient;
        impulseScalar = FixedMath.Clamp(impulseScalar, -maxFrictionImpulse, maxFrictionImpulse);
        if (impulseScalar == Fixed64.Zero)
            return Fixed64.Zero;

        Vector3d impulse = tangent * impulseScalar;
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
        return impulseScalar;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeRelativeVelocity(SolverContact contact)
    {
        Vector3d velocityA = contact.A.Body.LinearVelocity
            + Vector3d.Cross(contact.A.Body.AngularVelocity, contact.RelativeA);
        Vector3d velocityB = contact.B.Body.LinearVelocity
            + Vector3d.Cross(contact.B.Body.AngularVelocity, contact.RelativeB);
        return velocityB - velocityA;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeImpulseDenominator(SolverContact contact, Vector3d axis)
    {
        return contact.TotalInverseMass
            + ComputeAngularDenominator(contact.A, contact.RelativeA, axis)
            + ComputeAngularDenominator(contact.B, contact.RelativeB, axis);
    }

    private static Fixed64 ComputeAngularDenominator(ResponseBody body, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (!body.CanRotate)
            return Fixed64.Zero;

        Vector3d angular = Vector3d.Cross(
            body.InverseInertiaTensor * Vector3d.Cross(relativeContactPoint, axis),
            relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
    }

    private static void ApplyImpulse(ResponseBody body, Vector3d impulse, Vector3d relativeContactPoint)
    {
        if (!body.CanMove)
            return;

        body.Body.ApplyCollisionLinearVelocityDelta(impulse * body.InverseMass);

        if (!body.CanRotate)
            return;

        Vector3d angularVelocityDelta = body.InverseInertiaTensor * Vector3d.Cross(relativeContactPoint, impulse);
        body.Body.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static Fixed64 ResolveRestitution(SolverContact contact, Fixed64 closingSpeed)
    {
        if (closingSpeed <= RestitutionVelocityThreshold)
            return Fixed64.Zero;

        Fixed64 restitution = FixedMath.Min(
            contact.A.Body.RestitutionCoefficient,
            contact.B.Body.RestitutionCoefficient);
        return FixedMath.Clamp(restitution, Fixed64.Zero, Fixed64.One);
    }

    private static Fixed64 ResolveFrictionCoefficient(SolverContact contact)
    {
        Fixed64 frictionProduct = contact.A.Body.FrictionCoefficient * contact.B.Body.FrictionCoefficient;
        return frictionProduct > Fixed64.Zero
            ? FixedMath.Sqrt(frictionProduct)
            : Fixed64.Zero;
    }

    private static Vector3d ResolveContactNormal(Vector3d normal, Vector3d fallbackDirection)
    {
        Vector3d resolved = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : fallbackDirection.MagnitudeSquared > Fixed64.Epsilon
                ? fallbackDirection.Normalized
                : Vector3d.Zero;

        if (resolved == Vector3d.Zero)
            return resolved;

        return fallbackDirection.MagnitudeSquared > Fixed64.Epsilon
            && Vector3d.Dot(resolved, fallbackDirection) < Fixed64.Zero
                ? -resolved
                : resolved;
    }
}
