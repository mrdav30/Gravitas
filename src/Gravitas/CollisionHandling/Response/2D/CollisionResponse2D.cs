//=======================================================================
// CollisionResponse2D.cs
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

        ContactAnchor2D responseCenterA = bodyA.Body?.GetCenterOfMassAnchor() ?? default;
        ContactAnchor2D responseCenterB = bodyB.Body?.GetCenterOfMassAnchor() ?? default;
        SolverContactBuffer2D contacts = BuildContactBuffer(
            pair,
            bodyA,
            bodyB,
            responseCenterA,
            responseCenterB);
        if (contacts.Count == 0)
            return;

        Vector2d responsePositionA = bodyA.Body?.Position ?? Vector2d.Zero;
        Vector2d responsePositionB = bodyB.Body?.Position ?? Vector2d.Zero;
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
                SolverContact2D contact = contacts.GetContact(i);
                if (TryApplyCachedImpulse(
                        pair,
                        contact,
                        responsePositionA,
                        responsePositionB))
                {
                    continue;
                }

                pair.RemoveWarmStartImpulse(contact.ContactId);
                rebuildContacts = true;
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

        Vector2d normalLinearVelocityA =
            ResolveLinearVelocity(bodyA.Body);
        Fixed64 normalAngularVelocityA =
            ResolveAngularVelocity(bodyA.Body);
        Vector2d normalLinearVelocityB =
            ResolveLinearVelocity(bodyB.Body);
        Fixed64 normalAngularVelocityB =
            ResolveAngularVelocity(bodyB.Body);
        Fixed64 restitutionVelocityThreshold = pair.ColliderA.Context.Settings.RestitutionVelocityThreshold;
        for (int i = 0; i < contacts.Count; i++)
        {
            SolverContact2D contact = contacts.GetContact(i);
            SolidBody2D? contactBodyA = contact.A.Body;
            SolidBody2D? contactBodyB = contact.B.Body;
            bool normalResolved;
            ContactNormalImpulseResult2D normalResult = default;
            if (contact.RelativeA.IsExact || contact.RelativeB.IsExact)
            {
                normalResolved = TryCalculateExactNormalResult(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    normalLinearVelocityA,
                    normalAngularVelocityA,
                    normalLinearVelocityB,
                    normalAngularVelocityB,
                    restitutionVelocityThreshold,
                    contactShare,
                    out normalResult);
            }
            else
            {
                normalResolved = CanUseCompactAxisResponse(
                        contact,
                        contact.Normal)
                    && ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                        contactBodyA,
                        normalLinearVelocityA,
                        normalAngularVelocityA,
                        contact.RelativeA.Vector,
                        contactBodyB,
                        normalLinearVelocityB,
                        normalAngularVelocityB,
                        contact.RelativeB.Vector,
                        contact.Normal,
                        contact.Restitution,
                        restitutionVelocityThreshold,
                        contact.CachedNormalImpulse,
                        contactShare,
                        contactShare,
                        out normalResult);
                if (!normalResolved)
                {
                    normalResolved = TryCalculateExactNormalResult(
                        pair,
                        contact,
                        responsePositionA,
                        responsePositionB,
                        normalLinearVelocityA,
                        normalAngularVelocityA,
                        normalLinearVelocityB,
                        normalAngularVelocityB,
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
            Fixed64 normalImpulse = contact.CachedNormalImpulse + normalResult.ImpulseScalar;
            contacts.SetNormalImpulse(
                i,
                normalImpulse,
                normalResult);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

            if (!TryApplyNormalImpulse(
                    contacts.GetContact(i),
                    contacts.GetNormalResult(i)))
            {
                RejectResponse(
                    pair,
                    contacts.GetContact(i),
                    i,
                    ref failedResponseMask);
            }
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

            SolverContact2D contact = contacts.GetContact(i);
            if (!TrySolveFrictionImpulse(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    normalLinearVelocityA,
                    normalAngularVelocityA,
                    normalLinearVelocityB,
                    normalAngularVelocityB,
                    restitutionVelocityThreshold,
                    contactShare,
                    contacts.GetNormalResult(i),
                    contacts.GetNormalImpulse(i),
                    out Fixed64 tangentImpulse))
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
                continue;
            }
            contacts.SetTangentImpulse(
                i,
                tangentImpulse);
        }

        for (int i = 0; i < contacts.Count; i++)
        {
            if (HasFailedResponse(failedResponseMask, i))
                continue;

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
        ResponseBody2D bodyB,
        in ContactAnchor2D responseCenterA,
        in ContactAnchor2D responseCenterB)
    {
        SolverContactBuffer2D contacts = default;
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            if (TryCreateContact(
                    pair,
                    bodyA,
                    bodyB,
                    responseCenterA,
                    responseCenterB,
                    i,
                    out SolverContact2D contact))
                contacts.Add(contact);
        }

        return contacts;
    }

    private static bool TryCreateContact(
        CollisionPair2D pair,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB,
        in ContactAnchor2D responseCenterA,
        in ContactAnchor2D responseCenterB,
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
        ContactLever2D relativeA = bodyA.Body == null
            ? ContactLever2D.Zero
            : ContactLever2D.Create(
                manifoldContact.AnchorA,
                responseCenterA);
        ContactLever2D relativeB = bodyB.Body == null
            ? ContactLever2D.Zero
            : ContactLever2D.Create(
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
        if (pair.TryGetWarmStartImpulse(manifoldContact.ContactId, out ContactWarmStartImpulse cached))
        {
            cachedNormalImpulse = cached.NormalImpulse;
            cachedTangentImpulse = cached.TangentImpulse;
        }

        contact = new SolverContact2D(
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
            cachedTangentImpulse);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryCalculateExactNormalResult(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 contactShare,
        out ContactNormalImpulseResult2D result)
    {
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        return ContactNormalImpulse2D.TryCalculateAccumulatedDeltaExact(
            contact.A.Body,
            linearVelocityA,
            angularVelocityA,
            exactA,
            contact.B.Body,
            linearVelocityB,
            angularVelocityB,
            exactB,
            contact.Normal,
            contact.Restitution,
            restitutionVelocityThreshold,
            contact.CachedNormalImpulse,
            contactShare,
            contactShare,
            out result);
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

    private static bool TryApplyCachedImpulse(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB)
    {
        if (contact.CachedNormalImpulse == Fixed64.Zero && contact.CachedTangentImpulse == Fixed64.Zero)
            return true;

        return TryApplyContactImpulseCombination(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            contact.Normal,
            contact.CachedNormalImpulse,
            contact.Tangent,
            contact.CachedTangentImpulse);
    }

    private static bool TryApplyNormalImpulse(
        SolverContact2D contact,
        ContactNormalImpulseResult2D result)
    {
        if (result.LinearVelocityDeltaA == Vector2d.Zero
            && result.AngularVelocityDeltaA == Fixed64.Zero
            && result.LinearVelocityDeltaB == Vector2d.Zero
            && result.AngularVelocityDeltaB == Fixed64.Zero)
        {
            return true;
        }

        return TryApplyVelocityDeltas(
            contact,
            result.LinearVelocityDeltaA,
            result.AngularVelocityDeltaA,
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

    private static bool TrySolveFrictionImpulse(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        Vector2d normalLinearVelocityA,
        Fixed64 normalAngularVelocityA,
        Vector2d normalLinearVelocityB,
        Fixed64 normalAngularVelocityB,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 contactShare,
        ContactNormalImpulseResult2D normalResult,
        Fixed64 normalImpulseScalar,
        out Fixed64 accumulated)
    {
        accumulated = default;
        if (contact.RelativeA.IsExact
            || contact.RelativeB.IsExact
            || !normalResult.HasRepresentableAccumulatedImpulse
            || !CanUseCompactFriction(contact)
            || !Fixed64.TryMultiplyDivide(
                normalImpulseScalar,
                contact.StaticFriction,
                Fixed64.One,
                out Fixed64 staticFrictionLimit)
            || !Fixed64.TryMultiplyDivide(
                normalImpulseScalar,
                contact.DynamicFriction,
                Fixed64.One,
                out Fixed64 dynamicFrictionLimit))
        {
            return TrySolveFrictionImpulseExact(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                normalLinearVelocityA,
                normalAngularVelocityA,
                normalLinearVelocityB,
                normalAngularVelocityB,
                restitutionVelocityThreshold,
                contactShare,
                out accumulated);
        }

        Fixed64 impulseScalar = Fixed64.Zero;
        if (staticFrictionLimit > Fixed64.Zero || dynamicFrictionLimit > Fixed64.Zero)
        {
            Fixed64 tangentVelocity = Vector2d.Dot(
                ComputeRelativeVelocity(contact),
                contact.Tangent);
            Fixed64 denominator = ComputeImpulseDenominator(
                contact,
                contact.Tangent);
            if (tangentVelocity.Abs() > Fixed64.Epsilon
                && denominator > Fixed64.Zero
                && !Fixed64.TryMultiplyDivide(
                    -tangentVelocity,
                    Fixed64.One,
                    denominator,
                    out impulseScalar))
            {
                return TrySolveFrictionImpulseExact(
                    pair,
                    contact,
                    responsePositionA,
                    responsePositionB,
                    normalLinearVelocityA,
                    normalAngularVelocityA,
                    normalLinearVelocityB,
                    normalAngularVelocityB,
                    restitutionVelocityThreshold,
                    contactShare,
                    out accumulated);
            }
        }

        if (!Fixed64.TryAdd(
                contact.CachedTangentImpulse,
                impulseScalar,
                out Fixed64 desiredAccumulated))
        {
            return TrySolveFrictionImpulseExact(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                normalLinearVelocityA,
                normalAngularVelocityA,
                normalLinearVelocityB,
                normalAngularVelocityB,
                restitutionVelocityThreshold,
                contactShare,
                out accumulated);
        }
        accumulated = desiredAccumulated.Abs() <= staticFrictionLimit
            ? desiredAccumulated
            : FixedMath.Clamp(
                desiredAccumulated,
                -dynamicFrictionLimit,
                dynamicFrictionLimit);
        if (!Fixed64.TrySubtract(
                accumulated,
                contact.CachedTangentImpulse,
                out impulseScalar))
        {
            return TrySolveFrictionImpulseExact(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                normalLinearVelocityA,
                normalAngularVelocityA,
                normalLinearVelocityB,
                normalAngularVelocityB,
                restitutionVelocityThreshold,
                contactShare,
                out accumulated);
        }
        return impulseScalar == Fixed64.Zero
            || TryApplyContactImpulseCombination(
                pair,
                contact,
                responsePositionA,
                responsePositionB,
                contact.Normal,
                Fixed64.Zero,
                contact.Tangent,
                impulseScalar);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TrySolveFrictionImpulseExact(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        Vector2d normalLinearVelocityA,
        Fixed64 normalAngularVelocityA,
        Vector2d normalLinearVelocityB,
        Fixed64 normalAngularVelocityB,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 contactShare,
        out Fixed64 accumulated)
    {
        accumulated = default;
        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        Vector3d spatialNormal =
            ExactContactLever2D.ToSpatial(contact.Normal);
        var normalConstraint = new ExactNormalConstraint3D(
            ExactContactLever2D.CreateResponseOperand(
                contact.A.Body,
                normalLinearVelocityA,
                normalAngularVelocityA,
                exactA,
                -spatialNormal),
            ExactContactLever2D.CreateResponseOperand(
                contact.B.Body,
                normalLinearVelocityB,
                normalAngularVelocityB,
                exactB,
                spatialNormal),
            spatialNormal,
            contact.Restitution,
            restitutionVelocityThreshold,
            contact.CachedNormalImpulse,
            contactShare,
            contactShare);
        Vector3d spatialTangent =
            ExactContactLever2D.ToSpatial(contact.Tangent);
        if (!ExactContactResponseKernel.TryGetCoulombLineResponse(
                normalConstraint,
                ExactContactLever2D.CreateResponseOperand(
                    contact.A.Body,
                    ResolveLinearVelocity(contact.A.Body),
                    ResolveAngularVelocity(contact.A.Body),
                    exactA,
                    -spatialTangent),
                ExactContactLever2D.CreateResponseOperand(
                    contact.B.Body,
                    ResolveLinearVelocity(contact.B.Body),
                    ResolveAngularVelocity(contact.B.Body),
                    exactB,
                    spatialTangent),
                spatialTangent,
                contact.CachedTangentImpulse,
                contact.StaticFriction,
                contact.DynamicFriction,
                out ExactCoulombResponse3D response))
        {
            return false;
        }

        accumulated = response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 projectedAccumulated)
            ? projectedAccumulated
            : Fixed64.Zero;
        return !response.HasAppliedImpulse
            || TryApplyVelocityDeltas(
                contact,
                ExactContactLever2D.ToPlanar(
                    response.FirstLinearVelocityDelta),
                ExactContactLever2D.ToPlanarAngular(
                    response.FirstAngularVelocityDelta),
                ExactContactLever2D.ToPlanar(
                    response.SecondLinearVelocityDelta),
                ExactContactLever2D.ToPlanarAngular(
                    response.SecondAngularVelocityDelta));
    }

    private static bool CanUseCompactFriction(SolverContact2D contact) =>
        CanUseCompactAxisResponse(contact, contact.Tangent);

    private static bool CanUseCompactAxisResponse(
        SolverContact2D contact,
        Vector2d axis) =>
        ExactContactLever2D.CanUseCompactResponse(
            contact.A.Body,
            ResolveLinearVelocity(contact.A.Body),
            ResolveAngularVelocity(contact.A.Body),
            contact.RelativeA.Vector,
            contact.B.Body,
            ResolveLinearVelocity(contact.B.Body),
            ResolveAngularVelocity(contact.B.Body),
            contact.RelativeB.Vector,
            axis);

    private static void ApplyPositionCorrection(ResponseBody2D body, Vector2d correction)
    {
        if (!body.CanTranslate || correction == Vector2d.Zero)
            return;

        body.Body!.ApplyCollisionPositionCorrection(correction);
    }

    private static bool TryApplyContactImpulseCombination(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        Vector2d firstAxis,
        Fixed64 firstScale,
        Vector2d secondAxis,
        Fixed64 secondScale)
    {
        if (!contact.RelativeA.IsExact
            && !contact.RelativeB.IsExact
            && Vector3d.TryLinearCombination(
                ExactContactLever2D.ToSpatial(firstAxis),
                firstScale,
                ExactContactLever2D.ToSpatial(secondAxis),
                secondScale,
                Vector3d.Zero,
                Fixed64.Zero,
                out Vector3d spatialImpulse)
            && TryApplyCompactImpulse(
                contact,
                ExactContactLever2D.ToPlanar(spatialImpulse)))
        {
            return true;
        }

        GetExactLevers(
            pair,
            contact,
            responsePositionA,
            responsePositionB,
            out ExactLever3D exactA,
            out ExactLever3D exactB);
        return ExactContactLever2D.TryGetImpulseVelocityDeltas(
                contact.A.Body,
                exactA,
                contact.B.Body,
                exactB,
                firstAxis,
                firstScale,
                secondAxis,
                secondScale,
                out Vector2d linearA,
                out Fixed64 angularA,
                out Vector2d linearB,
                out Fixed64 angularB)
            && TryApplyVelocityDeltas(
                contact,
                linearA,
                angularA,
                linearB,
                angularB);
    }

    private static bool TryApplyCompactImpulse(
        SolverContact2D contact,
        Vector2d impulseB)
    {
        Vector2d impulseA = -impulseB;
        bool linearAResolved = TryGetLinearVelocityDelta(
            contact.A,
            impulseA,
            out Vector2d linearA);
        bool angularAResolved = TryGetAngularVelocityDelta(
            contact.A,
            contact.RelativeA.Vector,
            impulseA,
            out Fixed64 angularA);
        bool linearBResolved = TryGetLinearVelocityDelta(
            contact.B,
            impulseB,
            out Vector2d linearB);
        bool angularBResolved = TryGetAngularVelocityDelta(
            contact.B,
            contact.RelativeB.Vector,
            impulseB,
            out Fixed64 angularB);
        return linearAResolved
            & angularAResolved
            & linearBResolved
            & angularBResolved
            && TryApplyVelocityDeltas(
                contact,
                linearA,
                angularA,
                linearB,
                angularB);
    }

    private static bool TryGetLinearVelocityDelta(
        ResponseBody2D body,
        Vector2d impulse,
        out Vector2d velocityDelta)
    {
        if (!body.CanTranslate)
        {
            velocityDelta = Vector2d.Zero;
            return true;
        }

        return ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            body.Body!.ProjectLinearMotion(impulse),
            Fixed64.One,
            body.InverseMass,
            Fixed64.One,
            out velocityDelta);
    }

    private static bool TryGetAngularVelocityDelta(
        ResponseBody2D body,
        Vector2d relativeContactPoint,
        Vector2d impulse,
        out Fixed64 velocityDelta)
    {
        velocityDelta = Fixed64.Zero;
        if (!body.CanRotate)
            return true;

        return ContactResponseArithmetic3D.TryCross(
                ExactContactLever2D.ToSpatial(relativeContactPoint),
                ExactContactLever2D.ToSpatial(impulse),
                out Vector3d torque)
            && Fixed64.TryMultiplyDivide(
                -torque.Y,
                body.InverseMoment,
                Fixed64.One,
                out velocityDelta);
    }

    private static bool TryApplyVelocityDeltas(
        SolverContact2D contact,
        Vector2d linearA,
        Fixed64 angularA,
        Vector2d linearB,
        Fixed64 angularB)
    {
        bool firstFits = contact.A.Body?.CanApplyCollisionVelocityDeltas(
                linearA,
                angularA)
            ?? true;
        bool secondFits = contact.B.Body?.CanApplyCollisionVelocityDeltas(
                linearB,
                angularB)
            ?? true;
        if (!(firstFits & secondFits))
            return false;

        ApplyVelocityDelta(contact.A, linearA, angularA);
        ApplyVelocityDelta(contact.B, linearB, angularB);
        return true;
    }

    private static Vector2d ComputeRelativeVelocity(SolverContact2D contact)
    {
        Vector2d velocityA = ResolveLinearVelocity(contact.A.Body)
            + AngularVelocityAtPoint(contact.RelativeA.Vector, ResolveAngularVelocity(contact.A.Body));
        Vector2d velocityB = ResolveLinearVelocity(contact.B.Body)
            + AngularVelocityAtPoint(contact.RelativeB.Vector, ResolveAngularVelocity(contact.B.Body));
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
            + ComputeAngularDenominator(contact.A, contact.RelativeA.Vector, axis)
            + ComputeAngularDenominator(contact.B, contact.RelativeB.Vector, axis);
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

    private static void GetExactLevers(
        CollisionPair2D pair,
        SolverContact2D contact,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        out ExactLever3D exactA,
        out ExactLever3D exactB)
    {
        ManifoldContact2D manifoldContact =
            pair.Manifold[contact.ManifoldIndex];
        ContactAnchor2D zero =
            ContactAnchor2D.FromWorldPoint(Vector2d.Zero);
        exactA = contact.A.Body == null
            ? zero.GetXZLeverFrom(zero)
            : manifoldContact.AnchorA.GetXZLeverFrom(
                new ContactAnchor2D(
                    responsePositionA,
                    contact.A.Body.Rotation,
                    contact.A.Body.LocalCenterOfMassOffset));
        exactB = contact.B.Body == null
            ? zero.GetXZLeverFrom(zero)
            : manifoldContact.AnchorB.GetXZLeverFrom(
                new ContactAnchor2D(
                    responsePositionB,
                    contact.B.Body.Rotation,
                    contact.B.Body.LocalCenterOfMassOffset));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasFailedResponse(
        byte failedResponseMask,
        int contactIndex) =>
        (failedResponseMask & (1 << contactIndex)) != 0;

    private static void RejectResponse(
        CollisionPair2D pair,
        SolverContact2D contact,
        int contactIndex,
        ref byte failedResponseMask)
    {
        failedResponseMask |= (byte)(1 << contactIndex);
        pair.RemoveWarmStartImpulse(contact.ContactId);
        GravitasLogger.Channel.Error(
            $"2D contact response is outside the representable velocity domain.");
    }
}
