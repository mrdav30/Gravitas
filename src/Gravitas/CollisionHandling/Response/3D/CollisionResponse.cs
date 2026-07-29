//=======================================================================
// CollisionResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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

        ContactAnchor responseCenterA = bodyA.Body.GetCenterOfMassAnchor();
        ContactAnchor responseCenterB = bodyB.Body.GetCenterOfMassAnchor();
        SolverContactBuffer contacts = BuildContactBuffer(
            pair,
            bodyA,
            bodyB,
            responseCenterA,
            responseCenterB);
        if (contacts.Count == 0)
            return;

        Vector3d responsePositionA = bodyA.Body.Position3d;
        Vector3d responsePositionB = bodyB.Body.Position3d;
        byte failedResponseMask = 0;
        Fixed64 contactShare = Fixed64.One / (Fixed64)contacts.Count;
        if (applyPositionCorrection)
        {
            for (int i = 0; i < contacts.Count; i++)
                ApplyPositionCorrection(contacts.GetContact(i), contactShare);
        }

        if (applyCachedImpulse)
        {
            bool rebuildContacts = false;
            for (int i = 0; i < contacts.Count; i++)
            {
                SolverContact contact = contacts.GetContact(i);
                if (!TryApplyCachedImpulse(
                        pair,
                        contact,
                        responsePositionA,
                        responsePositionB))
                {
                    ClearWarmStartImpulse(pair, contact);
                    rebuildContacts = true;
                }
            }

            if (rebuildContacts)
            {
                contacts = BuildContactBuffer(
                    pair,
                    bodyA,
                    bodyB,
                    responseCenterA,
                    responseCenterB);
            }
        }

        Fixed64 restitutionVelocityThreshold = pair.Context.Settings.RestitutionVelocityThreshold;
        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact contact = contacts.GetContact(i);
            bool normalResolved;
            ContactNormalImpulseResult3D normalResult;
            if (contact.RelativeA.IsExact || contact.RelativeB.IsExact)
            {
                normalResolved = TryCalculateExactNormalResult(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    restitutionVelocityThreshold,
                    contactShare,
                    out normalResult);
            }
            else
            {
                normalResolved = ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                    contact.A.Body,
                    ResolveLinearVelocity(contact.A.Body),
                    ResolveAngularVelocity(contact.A.Body),
                    contact.RelativeA.Vector,
                    contact.B.Body,
                    ResolveLinearVelocity(contact.B.Body),
                    ResolveAngularVelocity(contact.B.Body),
                    contact.RelativeB.Vector,
                    contact.Normal,
                    contact.Restitution,
                    restitutionVelocityThreshold,
                    contact.CachedNormalImpulse,
                    contactShare,
                    Fixed64.One,
                    out normalResult);
                if (!normalResolved)
                {
                    normalResolved = TryCalculateExactNormalResult(
                        pair,
                        contact,
                        responsePositionA,
                        responsePositionB,
                        restitutionVelocityThreshold,
                        contactShare,
                        out normalResult);
                }
            }

            if (!normalResolved)
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
                continue;
            }

            // The kernel preflights this same cache-plus-delta sum.
            Fixed64 normalImpulse =
                contact.CachedNormalImpulse
                + normalResult.ImpulseScalar;
            contacts.SetNormalImpulse(i, normalImpulse, normalResult);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

            SolverContact contact = contacts.GetContact(i);
            if (!TryApplyNormalImpulse(
                    pair,
                    contact,
                    contacts.GetNormalResult(i)))
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
            }
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

            SolverContact contact = contacts.GetContact(i);
            if (!TrySolveFrictionImpulse(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    FixedMath.Max(contacts.GetNormalImpulse(i), contact.CachedNormalImpulse),
                    out Fixed64 tangentImpulse,
                    out Fixed64 secondaryTangentImpulse))
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
                continue;
            }

            contacts.SetTangentImpulse(i, tangentImpulse, secondaryTangentImpulse);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

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

    private static SolverContactBuffer BuildContactBuffer(
        CollisionPair pair,
        ResponseBody bodyA,
        ResponseBody bodyB,
        in ContactAnchor responseCenterA,
        in ContactAnchor responseCenterB)
    {
        SolverContactBuffer contacts = default;
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            if (TryCreateContact(
                    pair,
                    bodyA,
                    bodyB,
                    responseCenterA,
                    responseCenterB,
                    i,
                    out SolverContact contact))
            {
                contacts.Add(contact);
            }
        }

        return contacts;
    }

    private static bool TryCreateContact(
        CollisionPair pair,
        ResponseBody bodyA,
        ResponseBody bodyB,
        in ContactAnchor responseCenterA,
        in ContactAnchor responseCenterB,
        int contactIndex,
        out SolverContact contact)
    {
        contact = default;
        ManifoldContact manifoldContact = pair.Manifold[contactIndex];
        Vector3d normal = ResolveContactNormal(manifoldContact.Normal, pair.ColliderB.Center - pair.ColliderA.Center);
        if (normal == Vector3d.Zero)
            return false;
        ContactLever3D relativeA = ContactLever3D.Create(
            manifoldContact.AnchorA,
            responseCenterA);
        ContactLever3D relativeB = ContactLever3D.Create(
            manifoldContact.AnchorB,
            responseCenterB);
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
            contactIndex,
            manifoldContact.ContactId,
            bodyA,
            bodyB,
            relativeA,
            relativeB,
            manifoldContact.Depth,
            normal,
            materialA,
            materialB,
            cachedNormalImpulse,
            cachedTangentImpulse,
            cachedSecondaryTangentImpulse);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryCalculateExactNormalResult(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 contactShare,
        out ContactNormalImpulseResult3D result)
    {
        result = default;
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        return ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
            contact.A.Body,
            ResolveLinearVelocity(contact.A.Body),
            ResolveAngularVelocity(contact.A.Body),
            exactA,
            contact.B.Body,
            ResolveLinearVelocity(contact.B.Body),
            ResolveAngularVelocity(contact.B.Body),
            exactB,
            contact.Normal,
            contact.Restitution,
            restitutionVelocityThreshold,
            contact.CachedNormalImpulse,
            contactShare,
            Fixed64.One,
            out result);
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

    private static bool TryApplyCachedImpulse(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB)
    {
        if (contact.CachedNormalImpulse == Fixed64.Zero
            && contact.CachedTangentImpulse == Fixed64.Zero
            && contact.CachedSecondaryTangentImpulse == Fixed64.Zero)
        {
            return true;
        }

        return TryApplyContactImpulseCombination(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            contact.Normal,
            contact.CachedNormalImpulse,
            contact.Tangent,
            contact.CachedTangentImpulse,
            contact.SecondaryTangent,
            contact.CachedSecondaryTangentImpulse);
    }

    private static bool TryApplyContactImpulseCombination(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        Vector3d thirdAxis,
        Fixed64 thirdScale)
    {
        if (ContactResponseArithmetic3D.TryLinearCombination(
                firstAxis,
                firstScale,
                secondAxis,
                secondScale,
                thirdAxis,
                thirdScale,
                out Vector3d impulse)
            && CanNegate(impulse))
        {
            return TryApplyContactImpulse(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                impulse);
        }

        return TryApplyContactImpulseCombinationExact(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            firstAxis,
            firstScale,
            secondAxis,
            secondScale,
            thirdAxis,
            thirdScale);
    }

    private static bool TryApplyContactImpulse(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Vector3d impulse)
    {
        if (!contact.RelativeA.IsExact
            && !contact.RelativeB.IsExact
            && TryApplyCompactImpulse(contact, impulse))
        {
            return true;
        }

        return TryApplyContactImpulseExact(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            impulse);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryApplyContactImpulseExact(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Vector3d impulse)
    {
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        return TryApplyExactImpulse(
            contact,
            exactA,
            exactB,
            impulse);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryApplyContactImpulseCombinationExact(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        Vector3d thirdAxis,
        Fixed64 thirdScale)
    {
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        if (!ExactContactLever3D
                .TryGetImpulseCombinationVelocityDeltas(
                    contact.A.Body,
                    exactA,
                    contact.B.Body,
                    exactB,
                    firstAxis,
                    firstScale,
                    secondAxis,
                    secondScale,
                    thirdAxis,
                    thirdScale,
                    out Vector3d linearA,
                    out Vector3d angularA,
                    out Vector3d linearB,
                    out Vector3d angularB))
        {
            return false;
        }

        return TryApplyVelocityDeltas(
            contact,
            linearA,
            angularA,
            linearB,
            angularB);
    }

    private static bool TryApplyNormalImpulse(
        CollisionPair pair,
        SolverContact contact,
        ContactNormalImpulseResult3D result)
    {
        if (result.LinearVelocityDeltaA == Vector3d.Zero
            && result.AngularVelocityDeltaA == Vector3d.Zero
            && result.LinearVelocityDeltaB == Vector3d.Zero
            && result.AngularVelocityDeltaB == Vector3d.Zero)
        {
            return true;
        }

        if (!TryPrepareVelocityStates(
                contact,
                result.LinearVelocityDeltaA,
                result.AngularVelocityDeltaA,
                result.LinearVelocityDeltaB,
                result.AngularVelocityDeltaB,
                out Vector3d linearA,
                out Vector3d angularA,
                out Vector3d linearB,
                out Vector3d angularB))
        {
            return false;
        }

        if (result.HasRepresentableAppliedImpulse
            && result.HasRepresentableNormalVelocity
            && result.AppliedImpulseScalar != Fixed64.Zero)
        {
            Vector3d impulse =
                contact.Normal * result.AppliedImpulseScalar;
            pair.Context.Diagnostics.EmitResponseImpulse(
                pair,
                impulse,
                result.NormalVelocity);
        }
        ApplyVelocityStates(
            contact,
            linearA,
            angularA,
            linearB,
            angularB);
        return true;
    }

    private static bool TrySolveFrictionImpulse(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse)
    {
        if (contact.RelativeA.IsExact || contact.RelativeB.IsExact)
        {
            return TrySolveFrictionImpulseExact(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                normalImpulseScalar,
                out tangentImpulse,
                out secondaryTangentImpulse);
        }

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
            return TryApplyFrictionDelta(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                -contact.CachedTangentImpulse,
                -contact.CachedSecondaryTangentImpulse);
        }

        Vector3d relativeVelocity = ComputeRelativeVelocity(contact);
        Fixed64 tangentDelta = ComputeTangentImpulseDelta(
            contact,
            relativeVelocity,
            contact.Tangent);
        Fixed64 secondaryTangentDelta = ComputeTangentImpulseDelta(
            contact,
            relativeVelocity,
            contact.SecondaryTangent);

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

        return TryApplyFrictionDelta(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            tangentImpulse - contact.CachedTangentImpulse,
            secondaryTangentImpulse - contact.CachedSecondaryTangentImpulse);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TrySolveFrictionImpulseExact(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 normalImpulseScalar,
        out Fixed64 tangentImpulse,
        out Fixed64 secondaryTangentImpulse)
    {
        tangentImpulse = Fixed64.Zero;
        secondaryTangentImpulse = Fixed64.Zero;
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);

        Fixed64 staticFrictionLimit =
            normalImpulseScalar > Fixed64.Zero
                && contact.StaticFriction > Fixed64.Zero
                ? normalImpulseScalar * contact.StaticFriction
                : Fixed64.Zero;
        Fixed64 dynamicFrictionLimit =
            normalImpulseScalar > Fixed64.Zero
                && contact.DynamicFriction > Fixed64.Zero
                ? normalImpulseScalar * contact.DynamicFriction
                : Fixed64.Zero;
        if (staticFrictionLimit <= Fixed64.Zero
            && dynamicFrictionLimit <= Fixed64.Zero)
        {
            return TryApplyFrictionDelta(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                -contact.CachedTangentImpulse,
                -contact.CachedSecondaryTangentImpulse);
        }

        if (!TryComputeTangentImpulseDeltaExact(
                contact,
                exactA,
                exactB,
                contact.Tangent,
                out Fixed64 tangentDelta)
            || !TryComputeTangentImpulseDeltaExact(
                contact,
                exactA,
                exactB,
                contact.SecondaryTangent,
                out Fixed64 secondaryTangentDelta))
        {
            return false;
        }

        tangentImpulse = contact.CachedTangentImpulse + tangentDelta;
        secondaryTangentImpulse =
            contact.CachedSecondaryTangentImpulse
            + secondaryTangentDelta;
        Fixed64 desiredMagnitudeSquared =
            tangentImpulse * tangentImpulse
            + secondaryTangentImpulse * secondaryTangentImpulse;
        Fixed64 staticLimitSquared =
            staticFrictionLimit * staticFrictionLimit;
        if (desiredMagnitudeSquared > staticLimitSquared)
        {
            ClampTangentImpulsePair(
                ref tangentImpulse,
                ref secondaryTangentImpulse,
                dynamicFrictionLimit);
        }

        return TryApplyFrictionDelta(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            tangentImpulse - contact.CachedTangentImpulse,
            secondaryTangentImpulse
                - contact.CachedSecondaryTangentImpulse);
    }

    private static bool TryComputeTangentImpulseDeltaExact(
        SolverContact contact,
        in ExactLever3D exactA,
        in ExactLever3D exactB,
        Vector3d tangent,
        out Fixed64 impulseDelta)
    {
        impulseDelta = Fixed64.Zero;
        if (!TryComputeRelativeVelocityProjectionExact(
                contact,
                exactA,
                exactB,
                tangent,
                out Fixed64 tangentVelocity))
        {
            return false;
        }

        if (tangentVelocity.Abs() <= Fixed64.Epsilon)
            return true;

        if (!TryComputeImpulseDenominatorExact(
                contact,
                exactA,
                exactB,
                tangent,
                out Fixed64 denominator))
        {
            return false;
        }

        if (denominator > Fixed64.Epsilon)
            impulseDelta = -tangentVelocity / denominator;
        return true;
    }

    private static Fixed64 ComputeTangentImpulseDelta(
        SolverContact contact,
        Vector3d relativeVelocity,
        Vector3d tangent)
    {
        Fixed64 tangentVelocity = Vector3d.Dot(relativeVelocity, tangent);
        if (tangentVelocity.Abs() <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 denominator =
            ComputeImpulseDenominator(contact, tangent);
        return denominator > Fixed64.Epsilon
            ? -tangentVelocity / denominator
            : Fixed64.Zero;
    }

    private static bool TryApplyFrictionDelta(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        Fixed64 tangentDelta,
        Fixed64 secondaryTangentDelta)
    {
        return TryApplyContactImpulseCombination(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            contact.Normal,
            Fixed64.Zero,
            contact.Tangent,
            tangentDelta,
            contact.SecondaryTangent,
            secondaryTangentDelta);
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
            + Vector3d.Cross(
                ResolveAngularVelocity(contact.A.Body),
                contact.RelativeA.Vector);
        Vector3d velocityB = ResolveLinearVelocity(contact.B.Body)
            + Vector3d.Cross(
                ResolveAngularVelocity(contact.B.Body),
                contact.RelativeB.Vector);
        return velocityB - velocityA;
    }

    private static bool TryComputeRelativeVelocityProjectionExact(
        SolverContact contact,
        in ExactLever3D exactA,
        in ExactLever3D exactB,
        Vector3d axis,
        out Fixed64 projection) =>
        ExactContactLever3D.TryComputeNormalVelocity(
            ResolveLinearVelocity(contact.A.Body),
            ResolveAngularVelocity(contact.A.Body),
            exactA,
            ResolveLinearVelocity(contact.B.Body),
            ResolveAngularVelocity(contact.B.Body),
            exactB,
            axis,
            out projection);

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
    private static Fixed64 ComputeImpulseDenominator(
        SolverContact contact,
        Vector3d axis)
    {
        return contact.GetTotalInverseMass(axis)
            + ComputeAngularDenominator(
                contact.A,
                contact.RelativeA.Vector,
                axis)
            + ComputeAngularDenominator(
                contact.B,
                contact.RelativeB.Vector,
                axis);
    }

    private static Fixed64 ComputeAngularDenominator(
        ResponseBody body,
        Vector3d relativeContactPoint,
        Vector3d axis)
    {
        if (!body.CanRotate)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta =
            body.ApplyConstrainedInverseInertia(
                Vector3d.Cross(relativeContactPoint, axis));
        Vector3d angular = Vector3d.Cross(
            angularVelocityDelta,
            relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return denominator > Fixed64.Zero
            ? denominator
            : Fixed64.Zero;
    }

    private static bool TryComputeImpulseDenominatorExact(
        SolverContact contact,
        in ExactLever3D exactA,
        in ExactLever3D exactB,
        Vector3d axis,
        out Fixed64 denominator) =>
        ExactContactLever3D.TryComputeDenominator(
            contact.A.Body,
            exactA,
            contact.B.Body,
            exactB,
            axis,
            out denominator);

    private static bool TryApplyExactImpulse(
        SolverContact contact,
        in ExactLever3D exactA,
        in ExactLever3D exactB,
        Vector3d impulseB)
    {
        Vector3d impulseA = -impulseB;
        bool linearAResolved = TryGetLinearVelocityDelta(
            contact.A,
            impulseA,
            out Vector3d linearA);
        bool angularAResolved =
            ExactContactLever3D.TryGetAngularVelocityDelta(
            contact.A.Body,
            exactA,
            impulseA,
            out Vector3d angularA);
        bool linearBResolved = TryGetLinearVelocityDelta(
            contact.B,
            impulseB,
            out Vector3d linearB);
        bool angularBResolved =
            ExactContactLever3D.TryGetAngularVelocityDelta(
            contact.B.Body,
            exactB,
            impulseB,
            out Vector3d angularB);
        if (!(linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved))
        {
            return false;
        }

        return TryApplyVelocityDeltas(
            contact,
            linearA,
            angularA,
            linearB,
            angularB);
    }

    private static bool TryApplyCompactImpulse(
        SolverContact contact,
        Vector3d impulseB)
    {
        Vector3d impulseA = -impulseB;
        bool linearAResolved = TryGetLinearVelocityDelta(
            contact.A,
            impulseA,
            out Vector3d linearA);
        bool angularAResolved = TryGetAngularVelocityDelta(
            contact.A,
            contact.RelativeA.Vector,
            impulseA,
            out Vector3d angularA);
        bool linearBResolved = TryGetLinearVelocityDelta(
            contact.B,
            impulseB,
            out Vector3d linearB);
        bool angularBResolved = TryGetAngularVelocityDelta(
            contact.B,
            contact.RelativeB.Vector,
            impulseB,
            out Vector3d angularB);
        if (!(linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved))
        {
            return false;
        }

        return TryApplyVelocityDeltas(
            contact,
            linearA,
            angularA,
            linearB,
            angularB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanNegate(Vector3d value) =>
        value.X != Fixed64.MinValue
        && value.Y != Fixed64.MinValue
        && value.Z != Fixed64.MinValue;

    private static bool TryGetLinearVelocityDelta(
        ResponseBody body,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        if (!body.HasSolverMobility || !body.Body.CanTranslate)
        {
            velocityDelta = Vector3d.Zero;
            return true;
        }

        return ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            body.Body.ProjectLinearMotion(impulse),
            Fixed64.One,
            body.InverseMass,
            Fixed64.One,
            out velocityDelta);
    }

    private static bool TryGetAngularVelocityDelta(
        ResponseBody body,
        Vector3d relativeContactPoint,
        Vector3d impulse,
        out Vector3d velocityDelta)
    {
        velocityDelta = Vector3d.Zero;
        if (!body.HasSolverMobility || !body.CanRotate)
            return true;

        Fixed3x3 inverseInertia =
            body.Body.GetConstrainedInverseInertiaTensor();
        if (ContactResponseArithmetic3D.CanUseFastAngularResponse(
                relativeContactPoint,
                impulse,
                inverseInertia))
        {
            velocityDelta = Fixed3x3.TransformDirection(
                inverseInertia,
                Vector3d.Cross(relativeContactPoint, impulse));
            return true;
        }

        return ContactResponseArithmetic3D.TryCross(
                relativeContactPoint,
                impulse,
                out Vector3d torqueAxis)
            && ContactResponseArithmetic3D.TryTransformDirection(
                inverseInertia,
                torqueAxis,
                out velocityDelta);
    }

    private static bool TryPrepareVelocityStates(
        SolverContact contact,
        Vector3d linearA,
        Vector3d angularA,
        Vector3d linearB,
        Vector3d angularB,
        out Vector3d preparedLinearA,
        out Vector3d preparedAngularA,
        out Vector3d preparedLinearB,
        out Vector3d preparedAngularB)
    {
        bool firstPrepared =
            contact.A.Body.TryPrepareCollisionVelocityState(
                linearA,
                angularA,
                out preparedLinearA,
                out preparedAngularA);
        bool secondPrepared =
            contact.B.Body.TryPrepareCollisionVelocityState(
                linearB,
                angularB,
                out preparedLinearB,
                out preparedAngularB);
        return firstPrepared & secondPrepared;
    }

    private static bool TryApplyVelocityDeltas(
        SolverContact contact,
        Vector3d linearA,
        Vector3d angularA,
        Vector3d linearB,
        Vector3d angularB)
    {
        if (!TryPrepareVelocityStates(
                contact,
                linearA,
                angularA,
                linearB,
                angularB,
                out Vector3d preparedLinearA,
                out Vector3d preparedAngularA,
                out Vector3d preparedLinearB,
                out Vector3d preparedAngularB))
        {
            return false;
        }

        ApplyVelocityStates(
            contact,
            preparedLinearA,
            preparedAngularA,
            preparedLinearB,
            preparedAngularB);
        return true;
    }

    private static void ApplyVelocityStates(
        SolverContact contact,
        Vector3d linearA,
        Vector3d angularA,
        Vector3d linearB,
        Vector3d angularB)
    {
        contact.A.Body.ApplyCollisionVelocityState(linearA, angularA);
        contact.B.Body.ApplyCollisionVelocityState(linearB, angularB);
    }

    private static void GetExactLevers(
        CollisionPair pair,
        SolverContact contact,
        Vector3d responsePositionA,
        Vector3d responsePositionB,
        out ExactLever3D exactA,
        out ExactLever3D exactB)
    {
        ManifoldContact manifoldContact =
            pair.Manifold[contact.ManifoldIndex];
        var centerA = new ContactAnchor(
            responsePositionA,
            contact.A.Body.Rotation,
            contact.A.Body.LocalCenterOfMassOffset);
        var centerB = new ContactAnchor(
            responsePositionB,
            contact.B.Body.Rotation,
            contact.B.Body.LocalCenterOfMassOffset);
        exactA = manifoldContact.AnchorA.GetLeverFrom(centerA);
        exactB = manifoldContact.AnchorB.GetLeverFrom(centerB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasFailedResponse(byte failedResponseMask, int contactIndex) =>
        (failedResponseMask & (1 << contactIndex)) != 0;

    private static void RejectResponse(
        CollisionPair pair,
        SolverContact contact,
        int contactIndex,
        ref byte failedResponseMask)
    {
        failedResponseMask |= (byte)(1 << contactIndex);
        ClearWarmStartImpulse(pair, contact);
        GravitasLogger.Channel.Error(
            $"Contact response is outside the representable velocity domain.");
    }

    private static void ClearWarmStartImpulse(
        CollisionPair pair,
        SolverContact contact) =>
        pair.RemoveWarmStartImpulse(contact.ContactId);

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
