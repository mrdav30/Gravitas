//=======================================================================
// CollisionResponse2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Deterministic pure 2D manifold contact response.
/// </summary>
public static class CollisionResponse2D
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = Fixed64.One;

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

        Fixed64 restitutionVelocityThreshold = pair.ColliderA.Context.Settings.RestitutionVelocityThreshold;
        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact2D contact = contacts.GetContact(i);
            SolidBody2D? contactBodyA = contact.A.Body;
            SolidBody2D? contactBodyB = contact.B.Body;
            ContactNormalImpulseResult2D normalResult = ContactNormalImpulse2D.CalculateAccumulatedDelta(
                contactBodyA,
                ResolveLinearVelocity(contactBodyA),
                ResolveAngularVelocity(contactBodyA),
                contact.RelativeA,
                contactBodyB,
                ResolveLinearVelocity(contactBodyB),
                ResolveAngularVelocity(contactBodyB),
                contact.RelativeB,
                contact.Normal,
                contact.Restitution,
                restitutionVelocityThreshold,
                contact.CachedNormalImpulse,
                contactShare,
                contactShare);
            Fixed64 normalImpulse = contact.CachedNormalImpulse + normalResult.ImpulseScalar;
            contacts.SetNormalImpulse(
                i,
                normalImpulse,
                normalResult);
        }

        for (int i = 0; i < contacts.Count; i++)
            ApplyNormalImpulse(
                contacts.GetContact(i),
                contacts.GetNormalResult(i));

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
        return bodyA.HasSolverMobility || bodyB.HasSolverMobility;
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
        Vector2d relativeA = default;
        Vector2d relativeB = default;
        if ((bodyA.Body != null
                && !bodyA.Body.TryGetOffsetFromCenterOfMass(
                    manifoldContact.AnchorA,
                    out relativeA))
            || (bodyB.Body != null
                && !bodyB.Body.TryGetOffsetFromCenterOfMass(
                    manifoldContact.AnchorB,
                    out relativeB)))
        {
            GravitasLogger.Channel.Error(
                $"2D contact {manifoldContact.ContactId} cannot be rebased onto its response centers.");
            return false;
        }
        PhysicsMaterial materialA = manifoldContact.HasMaterialOverride
            ? manifoldContact.MaterialA
            : pair.ColliderA.Material;
        PhysicsMaterial materialB = manifoldContact.HasMaterialOverride
            ? manifoldContact.MaterialB
            : pair.ColliderB.Material;

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
            bodyA.Body == null ? Vector2d.Zero : relativeA,
            bodyB.Body == null ? Vector2d.Zero : relativeB,
            manifoldContact.Depth,
            normal,
            materialA,
            materialB,
            cachedNormalImpulse,
            cachedTangentImpulse);
        return true;
    }

    private static void ApplyPositionCorrection(SolverContact2D contact, Fixed64 contactShare)
    {
        Fixed64 correctionDepth = contact.Depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 inverseMassA = contact.A.GetConstrainedInverseMass(contact.Normal);
        Fixed64 inverseMassB = contact.B.GetConstrainedInverseMass(contact.Normal);
        Fixed64 totalInverseMass = inverseMassA + inverseMassB;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector2d correction = contact.Normal
            * (correctionDepth * PenetrationCorrectionPercent * contactShare / totalInverseMass);
        ApplyPositionCorrection(contact.A, -correction * inverseMassA);
        ApplyPositionCorrection(contact.B, correction * inverseMassB);
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

    private static void ApplyNormalImpulse(
        SolverContact2D contact,
        ContactNormalImpulseResult2D result)
    {
        if (result.ImpulseScalar == Fixed64.Zero)
            return;

        ApplyVelocityDelta(
            contact.A,
            result.LinearVelocityDeltaA,
            result.AngularVelocityDeltaA);
        ApplyVelocityDelta(
            contact.B,
            result.LinearVelocityDeltaB,
            result.AngularVelocityDeltaB);
    }

    private static void ApplyVelocityDelta(
        ResponseBody2D body,
        Vector2d linearVelocityDelta,
        Fixed64 angularVelocityDelta)
    {
        if (!body.HasSolverMobility)
            return;

        if (body.CanTranslate)
            body.Body!.ApplyCollisionLinearVelocityDelta(linearVelocityDelta);
        if (body.CanRotate)
            body.Body!.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static Fixed64 SolveFrictionImpulse(SolverContact2D contact, Fixed64 normalImpulseScalar)
    {
        Fixed64 staticFrictionLimit = normalImpulseScalar * contact.StaticFriction;
        Fixed64 dynamicFrictionLimit = normalImpulseScalar * contact.DynamicFriction;
        Fixed64 impulseScalar = Fixed64.Zero;
        if (staticFrictionLimit > Fixed64.Zero || dynamicFrictionLimit > Fixed64.Zero)
        {
            Fixed64 tangentVelocity = Vector2d.Dot(ComputeRelativeVelocity(contact), contact.Tangent);
            Fixed64 denominator = ComputeImpulseDenominator(contact, contact.Tangent);
            if (tangentVelocity.Abs() > Fixed64.Epsilon && denominator > Fixed64.Epsilon)
                impulseScalar = -tangentVelocity / denominator;
        }

        Fixed64 desiredAccumulated = contact.CachedTangentImpulse + impulseScalar;
        Fixed64 accumulated = desiredAccumulated.Abs() <= staticFrictionLimit
            ? desiredAccumulated
            : FixedMath.Clamp(
                desiredAccumulated,
                -dynamicFrictionLimit,
                dynamicFrictionLimit);
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
        if (!body.HasSolverMobility || impulse == Vector2d.Zero)
            return;

        if (body.CanTranslate)
            body.Body!.ApplyCollisionLinearVelocityDelta(impulse * body.InverseMass);

        if (!body.CanRotate)
            return;

        Fixed64 angularVelocityDelta =
            Vector2d.CrossProduct(relativeContactPoint, impulse)
            * body.InverseMoment;
        body.Body!.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static Vector2d ComputeRelativeVelocity(SolverContact2D contact)
    {
        Vector2d velocityA = ResolveLinearVelocity(contact.A.Body)
            + AngularVelocityAtPoint(contact.RelativeA, ResolveAngularVelocity(contact.A.Body));
        Vector2d velocityB = ResolveLinearVelocity(contact.B.Body)
            + AngularVelocityAtPoint(contact.RelativeB, ResolveAngularVelocity(contact.B.Body));
        return velocityB - velocityA;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveLinearVelocity(SolidBody2D? body) =>
        body == null
            ? Vector2d.Zero
            : body.ProjectLinearMotion(
                body.IsKinematic
                    ? body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
                    : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveAngularVelocity(SolidBody2D? body) =>
        body == null
            ? Fixed64.Zero
            : body.IsKinematic
                ? body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
                : body.AngularVelocity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeImpulseDenominator(SolverContact2D contact, Vector2d axis)
    {
        return contact.GetTotalInverseMass(axis)
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
