//=======================================================================
// CollisionResponse2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Deterministic pure 2D manifold contact response.
/// </summary>
public static class CollisionResponse2D
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = Fixed64.One;

    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;

    internal static void Resolve(CollisionPair2D pair) =>
        Resolve(pair, applyCachedImpulse: true, applyPositionCorrection: true);

    internal static void Resolve(
        CollisionPair2D pair,
        bool applyCachedImpulse,
        bool applyPositionCorrection)
    {
        if (!TryCreateBodyPair(pair, out ResponseBody2D bodyA, out ResponseBody2D bodyB))
            return;

        SolverContactBuffer2D contacts = BuildContactBuffer(pair, bodyA, bodyB);
        if (contacts.Count == 0)
            return;

        Fixed64 contactShare = Fixed64.One / (Fixed64)contacts.Count;
        if (applyPositionCorrection)
        {
            for (int i = 0; i < contacts.Count; i++)
                ApplyPositionCorrection(contacts.GetContact(i), contactShare);
        }

        if (applyCachedImpulse)
        {
            for (int i = 0; i < contacts.Count; i++)
                ApplyCachedImpulse(contacts.GetContact(i));
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact2D contact = contacts.GetContact(i);
            Fixed64 normalDelta = ComputeNormalImpulseDelta(
                contact,
                out Fixed64 normalVelocity);
            Fixed64 normalImpulse = FixedMath.Max(
                Fixed64.Zero,
                contact.CachedNormalImpulse + normalDelta * contactShare);
            contacts.SetNormalImpulse(
                i,
                normalImpulse,
                normalVelocity);
        }

        for (int i = 0; i < contacts.Count; i++)
            ApplyNormalImpulse(
                contacts.GetContact(i),
                contacts.GetNormalImpulse(i) - contacts.GetContact(i).CachedNormalImpulse);

        for (int i = 0; i < contacts.Count; i++)
        {
            Fixed64 tangentImpulse = SolveFrictionImpulse(
                contacts.GetContact(i),
                contacts.GetNormalImpulse(i));
            contacts.SetTangentImpulse(
                i,
                tangentImpulse);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact2D contact = contacts.GetContact(i);
            pair.StoreWarmStartImpulse(
                contact.ContactId,
                contacts.GetNormalImpulse(i),
                contacts.GetTangentImpulse(i));
        }
    }

    private static bool TryCreateBodyPair(
        CollisionPair2D pair,
        out ResponseBody2D bodyA,
        out ResponseBody2D bodyB)
    {
        bodyA = default;
        bodyB = default;

        if (pair.ColliderA.IsTrigger || pair.ColliderB.IsTrigger || !pair.Manifold.HasContact)
            return false;

        bodyA = ResponseBody2D.Create(pair.ColliderA);
        bodyB = ResponseBody2D.Create(pair.ColliderB);
        return bodyA.InverseMass + bodyB.InverseMass > Fixed64.Zero;
    }

    private static SolverContactBuffer2D BuildContactBuffer(
        CollisionPair2D pair,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB)
    {
        SolverContactBuffer2D contacts = default;
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            if (TryCreateContact(pair, bodyA, bodyB, i, out SolverContact2D contact))
                contacts.Add(contact);
        }

        return contacts;
    }

    private static bool TryCreateContact(
        CollisionPair2D pair,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB,
        int contactIndex,
        out SolverContact2D contact)
    {
        contact = default;
        ManifoldContact2D manifoldContact = pair.Manifold[contactIndex];
        Vector2d normal = ResolveContactNormal(
            manifoldContact.Normal,
            pair.ColliderB.Center - pair.ColliderA.Center);
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 cachedNormalImpulse = Fixed64.Zero;
        Fixed64 cachedTangentImpulse = Fixed64.Zero;
        if (pair.TryGetWarmStartImpulse(manifoldContact.ContactId, out ContactWarmStartImpulse cached))
        {
            cachedNormalImpulse = cached.NormalImpulse;
            cachedTangentImpulse = cached.TangentImpulse;
        }

        contact = new SolverContact2D(
            manifoldContact.ContactId,
            bodyA,
            bodyB,
            manifoldContact.PointA,
            manifoldContact.PointB,
            bodyA.Body == null ? Vector2d.Zero : manifoldContact.PointA - bodyA.Body.WorldCenterOfMass,
            bodyB.Body == null ? Vector2d.Zero : manifoldContact.PointB - bodyB.Body.WorldCenterOfMass,
            manifoldContact.Depth,
            normal,
            cachedNormalImpulse,
            cachedTangentImpulse);
        return true;
    }

    private static void ApplyPositionCorrection(SolverContact2D contact, Fixed64 contactShare)
    {
        Fixed64 correctionDepth = contact.Depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 totalInverseMass = contact.TotalInverseMass;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector2d correction = contact.Normal
            * (correctionDepth * PenetrationCorrectionPercent * contactShare / totalInverseMass);
        ApplyPositionCorrection(contact.A, -correction * contact.A.InverseMass);
        ApplyPositionCorrection(contact.B, correction * contact.B.InverseMass);
    }

    private static void ApplyCachedImpulse(SolverContact2D contact)
    {
        if (contact.CachedNormalImpulse == Fixed64.Zero && contact.CachedTangentImpulse == Fixed64.Zero)
            return;

        Vector2d impulse =
            contact.Normal * contact.CachedNormalImpulse
            + contact.Tangent * contact.CachedTangentImpulse;
        ApplyImpulse(contact, impulse);
    }

    private static Fixed64 ComputeNormalImpulseDelta(SolverContact2D contact, out Fixed64 normalVelocity)
    {
        normalVelocity = Vector2d.Dot(ComputeRelativeVelocity(contact), contact.Normal);
        Fixed64 denominator = ComputeImpulseDenominator(contact, contact.Normal);
        if (denominator <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 restitution = normalVelocity < Fixed64.Zero
            ? ResolveRestitution(contact, -normalVelocity)
            : Fixed64.Zero;
        return -(Fixed64.One + restitution) * normalVelocity / denominator;
    }

    private static void ApplyNormalImpulse(SolverContact2D contact, Fixed64 impulseScalar)
    {
        if (impulseScalar == Fixed64.Zero)
            return;

        ApplyImpulse(contact, contact.Normal * impulseScalar);
    }

    private static Fixed64 SolveFrictionImpulse(SolverContact2D contact, Fixed64 normalImpulseScalar)
    {
        Fixed64 frictionCoefficient = ResolveFrictionCoefficient(contact);
        Fixed64 maxFrictionImpulse = normalImpulseScalar > Fixed64.Zero && frictionCoefficient > Fixed64.Zero
            ? normalImpulseScalar * frictionCoefficient
            : Fixed64.Zero;
        Fixed64 impulseScalar = Fixed64.Zero;
        if (maxFrictionImpulse > Fixed64.Zero)
        {
            Fixed64 tangentVelocity = Vector2d.Dot(ComputeRelativeVelocity(contact), contact.Tangent);
            Fixed64 denominator = ComputeImpulseDenominator(contact, contact.Tangent);
            if (tangentVelocity.Abs() > Fixed64.Epsilon && denominator > Fixed64.Epsilon)
                impulseScalar = -tangentVelocity / denominator;
        }

        Fixed64 accumulated = FixedMath.Clamp(
            contact.CachedTangentImpulse + impulseScalar,
            -maxFrictionImpulse,
            maxFrictionImpulse);
        impulseScalar = accumulated - contact.CachedTangentImpulse;
        if (impulseScalar != Fixed64.Zero)
            ApplyImpulse(contact, contact.Tangent * impulseScalar);

        return accumulated;
    }

    private static void ApplyImpulse(SolverContact2D contact, Vector2d impulse)
    {
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
    }

    private static void ApplyPositionCorrection(ResponseBody2D body, Vector2d correction)
    {
        if (!body.CanTranslate || correction == Vector2d.Zero)
            return;

        body.Body!.ApplyCollisionPositionCorrection(correction);
    }

    private static void ApplyImpulse(ResponseBody2D body, Vector2d impulse, Vector2d relativeContactPoint)
    {
        if (!body.CanTranslate || impulse == Vector2d.Zero)
            return;

        body.Body!.ApplyCollisionLinearVelocityDelta(impulse * body.InverseMass);

        if (!body.CanRotate)
            return;

        Fixed64 angularVelocityDelta =
            Vector2d.CrossProduct(relativeContactPoint, impulse)
            * body.InverseMoment;
        body.Body.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static Vector2d ComputeRelativeVelocity(SolverContact2D contact)
    {
        Vector2d velocityA = contact.A.Body == null
            ? Vector2d.Zero
            : contact.A.Body.LinearVelocity + AngularVelocityAtPoint(contact.RelativeA, contact.A.Body.AngularVelocity);
        Vector2d velocityB = contact.B.Body == null
            ? Vector2d.Zero
            : contact.B.Body.LinearVelocity + AngularVelocityAtPoint(contact.RelativeB, contact.B.Body.AngularVelocity);
        return velocityB - velocityA;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeImpulseDenominator(SolverContact2D contact, Vector2d axis)
    {
        return contact.TotalInverseMass
            + ComputeAngularDenominator(contact.A, contact.RelativeA, axis)
            + ComputeAngularDenominator(contact.B, contact.RelativeB, axis);
    }

    private static Fixed64 ComputeAngularDenominator(
        ResponseBody2D body,
        Vector2d relativeContactPoint,
        Vector2d axis)
    {
        if (!body.CanRotate)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativeContactPoint, axis);
        return cross * cross * body.InverseMoment;
    }

    private static Fixed64 ResolveRestitution(SolverContact2D contact, Fixed64 closingSpeed)
    {
        if (contact.A.Body == null || contact.B.Body == null || closingSpeed <= RestitutionVelocityThreshold)
            return Fixed64.Zero;

        Fixed64 restitution = FixedMath.Min(
            contact.A.Body.RestitutionCoefficient,
            contact.B.Body.RestitutionCoefficient);
        return FixedMath.Clamp(restitution, Fixed64.Zero, Fixed64.One);
    }

    private static Fixed64 ResolveFrictionCoefficient(SolverContact2D contact)
    {
        if (contact.A.Body == null && contact.B.Body == null)
            return Fixed64.Zero;
        if (contact.A.Body == null || contact.A.InverseMass <= Fixed64.Zero)
            return contact.B.Body?.FrictionCoefficient ?? Fixed64.Zero;
        if (contact.B.Body == null || contact.B.InverseMass <= Fixed64.Zero)
            return contact.A.Body.FrictionCoefficient;

        Fixed64 frictionProduct = contact.A.Body.FrictionCoefficient * contact.B.Body.FrictionCoefficient;
        return frictionProduct > Fixed64.Zero
            ? FixedMath.Sqrt(frictionProduct)
            : Fixed64.Zero;
    }

    private static Vector2d ResolveContactNormal(Vector2d normal, Vector2d fallbackDirection)
    {
        Vector2d resolved = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : fallbackDirection.MagnitudeSquared > Fixed64.Epsilon
                ? fallbackDirection.Normalized
                : Vector2d.Zero;

        if (resolved == Vector2d.Zero)
            return resolved;

        return fallbackDirection.MagnitudeSquared > Fixed64.Epsilon
            && Vector2d.Dot(resolved, fallbackDirection) < Fixed64.Zero
                ? -resolved
                : resolved;
    }
}
