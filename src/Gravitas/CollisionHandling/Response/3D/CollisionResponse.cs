//=======================================================================
// CollisionResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
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

    private static readonly Fixed64 WarmStartNormalCompatibilityThreshold = Fixed64.FromFraction(63, 64);

    /// <summary>
    /// Applies positional correction, normal impulses, and Coulomb friction for
    /// the collision pair's current deterministic contact manifold.
    /// </summary>
    public static void CalculateImpulse(CollisionPair pair) =>
        CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: true);

    internal static void CalculateImpulse(
        CollisionPair pair,
        bool applyCachedImpulse,
        bool applyPositionCorrection)
    {
        if (!TryCreateBodyPair(pair, out ResponseBody bodyA, out ResponseBody bodyB))
            return;

        SolverContactBuffer contacts = BuildContactBuffer(pair, bodyA, bodyB);
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

        Fixed64 restitutionVelocityThreshold = pair.Context.Settings.RestitutionVelocityThreshold;
        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            ContactNormalImpulseResult3D normalResult = ContactNormalImpulse3D.CalculateAccumulatedDelta(
                contact.A.Body,
                ResolveLinearVelocity(contact.A.Body),
                ResolveAngularVelocity(contact.A.Body),
                contact.RelativeA,
                contact.B.Body,
                ResolveLinearVelocity(contact.B.Body),
                ResolveAngularVelocity(contact.B.Body),
                contact.RelativeB,
                contact.Normal,
                contact.Restitution,
                restitutionVelocityThreshold,
                contact.CachedNormalImpulse,
                contactShare,
                Fixed64.One);
            Fixed64 normalImpulse = contact.CachedNormalImpulse + normalResult.ImpulseScalar;
            contacts.SetNormalImpulse(i, normalImpulse, normalResult);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            ApplyNormalImpulse(
                pair,
                contact,
                contacts.GetNormalResult(i));
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            SolveFrictionImpulse(
                contact,
                FixedMath.Max(contacts.GetNormalImpulse(i), contact.CachedNormalImpulse),
                out Fixed64 tangentImpulse,
                out Fixed64 secondaryTangentImpulse);
            contacts.SetTangentImpulse(i, tangentImpulse, secondaryTangentImpulse);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            pair.StoreWarmStartImpulse(
                contact.ContactId,
                contact.Normal,
                contacts.GetNormalImpulse(i),
                contacts.GetTangentImpulse(i),
                contacts.GetSecondaryTangentImpulse(i));
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
        return bodyA.HasSolverMobility || bodyB.HasSolverMobility;
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
        PhysicsMaterial materialA = manifoldContact.HasMaterialOverride
            ? manifoldContact.MaterialA
            : pair.ColliderA.Material;
        PhysicsMaterial materialB = manifoldContact.HasMaterialOverride
            ? manifoldContact.MaterialB
            : pair.ColliderB.Material;

        Fixed64 cachedNormalImpulse = Fixed64.Zero;
        Fixed64 cachedTangentImpulse = Fixed64.Zero;
        Fixed64 cachedSecondaryTangentImpulse = Fixed64.Zero;
        if (pair.TryGetWarmStartImpulse(manifoldContact.ContactId, out ContactWarmStartImpulse cached))
        {
            if (IsWarmStartCompatible(cached.Normal, normal))
            {
                cachedNormalImpulse = cached.NormalImpulse;
                cachedTangentImpulse = cached.TangentImpulse;
                cachedSecondaryTangentImpulse = cached.SecondaryTangentImpulse;
            }
        }

        contact = new SolverContact(
            manifoldContact.ContactId,
            bodyA,
            bodyB,
            manifoldContact.PointA,
            manifoldContact.PointB,
            manifoldContact.PointA - bodyA.Body.WorldCenterOfMass,
            manifoldContact.PointB - bodyB.Body.WorldCenterOfMass,
            manifoldContact.Depth,
            normal,
            materialA,
            materialB,
            cachedNormalImpulse,
            cachedTangentImpulse,
            cachedSecondaryTangentImpulse);
        return true;
    }

    private static void ApplyPositionCorrection(SolverContact contact, Fixed64 contactShare)
    {
        Fixed64 correctionDepth = contact.Depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 inverseMassA = contact.A.GetConstrainedInverseMass(contact.Normal);
        Fixed64 inverseMassB = contact.B.GetConstrainedInverseMass(contact.Normal);
        Fixed64 totalInverseMass = inverseMassA + inverseMassB;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector3d correction = contact.Normal
            * (correctionDepth * PenetrationCorrectionPercent * contactShare / totalInverseMass);
        contact.A.Body.ApplyCollisionPositionCorrection(-correction * inverseMassA);
        contact.B.Body.ApplyCollisionPositionCorrection(correction * inverseMassB);
    }

    private static void ApplyCachedImpulse(SolverContact contact)
    {
        if (contact.CachedNormalImpulse == Fixed64.Zero
            && contact.CachedTangentImpulse == Fixed64.Zero
            && contact.CachedSecondaryTangentImpulse == Fixed64.Zero)
        {
            return;
        }

        Vector3d impulse =
            contact.Normal * contact.CachedNormalImpulse
            + contact.Tangent * contact.CachedTangentImpulse
            + contact.SecondaryTangent * contact.CachedSecondaryTangentImpulse;
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
    }

    private static void ApplyNormalImpulse(
        CollisionPair pair,
        SolverContact contact,
        ContactNormalImpulseResult3D result)
    {
        if (result.ImpulseScalar == Fixed64.Zero)
            return;

        Vector3d impulse = contact.Normal * result.ImpulseScalar;
        pair.Context.Diagnostics.EmitResponseImpulse(pair, impulse, result.NormalVelocity);
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
        ResponseBody body,
        Vector3d linearVelocityDelta,
        Vector3d angularVelocityDelta)
    {
        if (!body.HasSolverMobility)
            return;

        if (body.Body.CanTranslate)
            body.Body.ApplyCollisionLinearVelocityDelta(linearVelocityDelta);
        if (body.CanRotate)
            body.Body.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static void SolveFrictionImpulse(
        SolverContact contact,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse)
    {
        tangentImpulse = Fixed64.Zero;
        secondaryTangentImpulse = Fixed64.Zero;

        Fixed64 staticFrictionLimit = normalImpulseScalar > Fixed64.Zero && contact.StaticFriction > Fixed64.Zero
            ? normalImpulseScalar * contact.StaticFriction
            : Fixed64.Zero;
        Fixed64 dynamicFrictionLimit = normalImpulseScalar > Fixed64.Zero && contact.DynamicFriction > Fixed64.Zero
            ? normalImpulseScalar * contact.DynamicFriction
            : Fixed64.Zero;
        if (staticFrictionLimit <= Fixed64.Zero && dynamicFrictionLimit <= Fixed64.Zero)
        {
            ApplyFrictionDelta(contact, -contact.CachedTangentImpulse, -contact.CachedSecondaryTangentImpulse);
            return;
        }

        Vector3d relativeVelocity = ComputeRelativeVelocity(contact);
        Fixed64 tangentDelta = ComputeTangentImpulseDelta(contact, relativeVelocity, contact.Tangent);
        Fixed64 secondaryTangentDelta = ComputeTangentImpulseDelta(contact, relativeVelocity, contact.SecondaryTangent);
        tangentImpulse = contact.CachedTangentImpulse + tangentDelta;
        secondaryTangentImpulse = contact.CachedSecondaryTangentImpulse + secondaryTangentDelta;
        Fixed64 desiredMagnitudeSquared = tangentImpulse * tangentImpulse
            + secondaryTangentImpulse * secondaryTangentImpulse;
        Fixed64 staticLimitSquared = staticFrictionLimit * staticFrictionLimit;
        if (desiredMagnitudeSquared > staticLimitSquared)
        {
            ClampTangentImpulsePair(
                ref tangentImpulse,
                ref secondaryTangentImpulse,
                dynamicFrictionLimit);
        }

        ApplyFrictionDelta(
            contact,
            tangentImpulse - contact.CachedTangentImpulse,
            secondaryTangentImpulse - contact.CachedSecondaryTangentImpulse);
    }

    private static Fixed64 ComputeTangentImpulseDelta(
        SolverContact contact,
        Vector3d relativeVelocity,
        Vector3d tangent)
    {
        Fixed64 tangentVelocity = Vector3d.Dot(relativeVelocity, tangent);
        if (tangentVelocity.Abs() <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 denominator = ComputeImpulseDenominator(contact, tangent);
        return denominator > Fixed64.Epsilon
            ? -tangentVelocity / denominator
            : Fixed64.Zero;
    }

    private static void ApplyFrictionDelta(
        SolverContact contact,
        Fixed64 tangentDelta,
        Fixed64 secondaryTangentDelta)
    {
        if (tangentDelta == Fixed64.Zero && secondaryTangentDelta == Fixed64.Zero)
            return;

        Vector3d impulse =
            contact.Tangent * tangentDelta
            + contact.SecondaryTangent * secondaryTangentDelta;
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
    }

    private static void ClampTangentImpulsePair(
        ref Fixed64 tangentImpulse,
        ref Fixed64 secondaryTangentImpulse,
        Fixed64 maxMagnitude)
    {
        Fixed64 magnitudeSquared = tangentImpulse * tangentImpulse
            + secondaryTangentImpulse * secondaryTangentImpulse;
        Fixed64 magnitude = FixedMath.Sqrt(magnitudeSquared);
        Fixed64 scale = maxMagnitude / magnitude;
        tangentImpulse *= scale;
        secondaryTangentImpulse *= scale;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComputeRelativeVelocity(SolverContact contact)
    {
        Vector3d velocityA = ResolveLinearVelocity(contact.A.Body)
            + Vector3d.Cross(ResolveAngularVelocity(contact.A.Body), contact.RelativeA);
        Vector3d velocityB = ResolveLinearVelocity(contact.B.Body)
            + Vector3d.Cross(ResolveAngularVelocity(contact.B.Body), contact.RelativeB);
        return velocityB - velocityA;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveLinearVelocity(SolidBody body) =>
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
    private static Fixed64 ComputeImpulseDenominator(SolverContact contact, Vector3d axis)
    {
        return contact.GetTotalInverseMass(axis)
            + ComputeAngularDenominator(contact.A, contact.RelativeA, axis)
            + ComputeAngularDenominator(contact.B, contact.RelativeB, axis);
    }

    private static Fixed64 ComputeAngularDenominator(ResponseBody body, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (!body.CanRotate)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Vector3d angular = Vector3d.Cross(angularVelocityDelta, relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
    }

    private static void ApplyImpulse(ResponseBody body, Vector3d impulse, Vector3d relativeContactPoint)
    {
        if (!body.HasSolverMobility)
            return;

        if (body.Body.CanTranslate)
            body.Body.ApplyCollisionLinearVelocityDelta(impulse * body.InverseMass);

        if (!body.CanRotate)
            return;

        Vector3d angularVelocityDelta = body.ApplyConstrainedInverseInertia(Vector3d.Cross(relativeContactPoint, impulse));
        body.Body.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static bool IsWarmStartCompatible(Vector3d cachedNormal, Vector3d normal) =>
        Vector3d.Dot(cachedNormal, normal) >= WarmStartNormalCompatibilityThreshold;

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
