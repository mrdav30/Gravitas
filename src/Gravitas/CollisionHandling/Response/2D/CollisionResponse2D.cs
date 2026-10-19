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
    /// <summary>Penetration tolerated before positional correction is applied.</summary>
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    /// <summary>Fraction of excess penetration corrected by the solver.</summary>
    public static readonly Fixed64 PenetrationCorrectionPercent = Fixed64.One;

    // Ordinary pairs retain only a null reference. Grouped pairs reuse their
    // phase-local incoming motion and exact lever frames across island iterations.
    internal sealed class GroupedSolveState
    {
        internal ContactResponseSnapshot Impact;
        internal ContactAnchor2D CenterA, CenterB;
    }

    internal static void PrepareSolve(CollisionPair2D pair)
    {
        pair.ClearResponseSnapshot();
        if (pair.Manifold.GroupCount <= 1
            || !TryCreateBodyPair(pair, out ResponseBody2D bodyA, out ResponseBody2D bodyB)) return;
        pair.RetainWarmStart();
        GroupedSolveState state = pair.ResponseState ??= new GroupedSolveState();
        state.CenterA = bodyA.Body?.GetCenterOfMassAnchor() ?? default;
        state.CenterB = bodyB.Body?.GetCenterOfMassAnchor() ?? default;
        state.Impact = new ContactResponseSnapshot(
            ExactContactLever2D.ToSpatial(bodyA.Body?.Position ?? Vector2d.Zero),
            ExactContactLever2D.ToSpatial(bodyB.Body?.Position ?? Vector2d.Zero), default, default,
            ExactContactLever2D.ToSpatial(ResolveLinearVelocity(bodyA.Body)), Vector3d.Zero,
            ExactContactLever2D.ToSpatial(ResolveLinearVelocity(bodyB.Body)), Vector3d.Zero,
            planarAngularB: ResolveAngularVelocity(bodyB.Body),
            planarAngularA: ResolveAngularVelocity(bodyA.Body));
    }

    internal static void Resolve(CollisionPair2D pair)
    {
        bool grouped = pair.Manifold.GroupCount > 1;
        if (grouped) PrepareSolve(pair);
        Resolve(pair, applyCachedImpulse: true, applyPositionCorrection: true, iterateGroups: true);
        if (grouped) pair.ClearResponseSnapshot();
    }

    internal static void Resolve(CollisionPair2D pair, bool applyCachedImpulse, bool applyPositionCorrection) =>
        Resolve(pair, applyCachedImpulse, applyPositionCorrection, iterateGroups: false);

    private static void Resolve(CollisionPair2D pair, bool applyCachedImpulse,
        bool applyPositionCorrection, bool iterateGroups)
    {
        if (applyPositionCorrection) pair.ApplyRecovery();
        if (!TryCreateBodyPair(pair, out ResponseBody2D bodyA, out ResponseBody2D bodyB))
            return;

        if (pair.Manifold.GroupCount == 1)
        {
            pair.RetainWarmStart();
            ResolveGroup(pair, bodyA, bodyB, 0, applyCachedImpulse, applyPositionCorrection,
                bodyA.Body?.GetCenterOfMassAnchor() ?? default,
                bodyB.Body?.GetCenterOfMassAnchor() ?? default,
                bodyA.Body?.Position ?? Vector2d.Zero,
                bodyB.Body?.Position ?? Vector2d.Zero, default);
            return;
        }
        ResolveGroups(pair, bodyA, bodyB, applyCachedImpulse, applyPositionCorrection, iterateGroups);
    }

    private static void ResolveGroups(CollisionPair2D pair, ResponseBody2D bodyA, ResponseBody2D bodyB,
        bool applyCachedImpulse, bool applyPositionCorrection, bool iterateGroups)
    {
        // Standalone and island owners prepare the grouped incoming state
        // before any row or joint changes a participating body's velocity.
        GroupedSolveState state = pair.ResponseState!;
        ContactAnchor2D centerA = state.CenterA, centerB = state.CenterB;
        ref readonly ContactResponseSnapshot impact = ref state.Impact;
        Vector2d positionA = ExactContactLever2D.ToPlanar(impact.PositionA);
        Vector2d positionB = ExactContactLever2D.ToPlanar(impact.PositionB);
        if (applyCachedImpulse || applyPositionCorrection)
        {
            // Warm all groups before solving any row: later warm starts must
            // not overwrite the state against which an earlier group was solved.
            for (int g = 0; g < pair.Manifold.GroupCount; g++)
            {
                ref ContactGroup2D group = ref pair.Manifold.GetGroup(g);
                for (int point = 0; point < group.Count; point++)
                {
                    if (!TryCreateContact(pair, bodyA, bodyB, centerA, centerB,
                            g, point, out SolverContact2D contact)) continue;
                    if (applyPositionCorrection && IsCorrectionOwner(pair, contact))
                        ApplyPositionCorrection(contact, Fixed64.One);
                    if (applyCachedImpulse && !TryApplyCachedImpulse(pair, contact, positionA, positionB))
                        pair.RemoveWarmStartImpulse(contact.Group, contact.ContactId);
                }
            }
        }
        int iterations = iterateGroups
            ? pair.ColliderA.Context.Settings.DiscreteSolverIterations : 1;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
            {
                ResolveGroup(pair, bodyA, bodyB, group,
                    applyCachedImpulse: false, applyPositionCorrection: false,
                    centerA, centerB, positionA, positionB, impact);
            }
        }
    }

    private static void ResolveGroup(
        CollisionPair2D pair,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB,
        int groupIndex,
        bool applyCachedImpulse,
        bool applyPositionCorrection,
        in ContactAnchor2D responseCenterA,
        in ContactAnchor2D responseCenterB,
        Vector2d responsePositionA,
        Vector2d responsePositionB,
        in ContactResponseSnapshot impact)
    {
        SolverContactBuffer2D contacts = BuildContactBuffer(
            pair,
            bodyA,
            bodyB,
            responseCenterA,
            responseCenterB, groupIndex);
        if (contacts.Count == 0)
            return;

        byte failedResponseMask = 0;
        Fixed64 contactShare = impact.IsValid ? Fixed64.One : Fixed64.One / (Fixed64)contacts.Count;
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

                pair.RemoveWarmStartImpulse(contact.Group, contact.ContactId);
                rebuildContacts = true;
            }

            if (rebuildContacts)
            {
                contacts = BuildContactBuffer(
                    pair,
                    bodyA,
                    bodyB,
                    responseCenterA,
                    responseCenterB, groupIndex);
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
            if (impact.IsValid)
            {
                normalLinearVelocityA = ResolveLinearVelocity(bodyA.Body);
                normalAngularVelocityA = ResolveAngularVelocity(bodyA.Body);
                normalLinearVelocityB = ResolveLinearVelocity(bodyB.Body);
                normalAngularVelocityB = ResolveAngularVelocity(bodyB.Body);
            }
            SolidBody2D? contactBodyA = contact.A.Body;
            SolidBody2D? contactBodyB = contact.B.Body;
            bool normalResolved;
            ContactNormalImpulseResult2D normalResult = default;
            if (contact.RelativeA.IsExact || contact.RelativeB.IsExact
                || impact.IsValid && contact.Restitution != Fixed64.Zero)
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
                    out normalResult, impact);
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
                        out normalResult, impact);
                }
            }
            if (!normalResolved)
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
                continue;
            }
            if (impact.IsValid && !TryApplyNormalImpulse(contact, normalResult))
            {
                RejectResponse(pair, contact, i, ref failedResponseMask);
                continue;
            }
            Fixed64 normalImpulse = contact.CachedNormalImpulse + normalResult.ImpulseScalar;
            contacts.SetNormalImpulse(
                i,
                normalImpulse,
                normalResult);
            if (impact.IsValid)
            {
                if (!TrySolveFrictionImpulse(pair, contact, responsePositionA, responsePositionB,
                        normalLinearVelocityA, normalAngularVelocityA, normalLinearVelocityB,
                        normalAngularVelocityB, restitutionVelocityThreshold, Fixed64.One,
                        normalResult, normalImpulse, out Fixed64 tangentImpulse, impact))
                {
                    RejectResponse(pair, contact, i, ref failedResponseMask);
                    continue;
                }
                pair.StoreWarmStartImpulse(contact.Group, contact.ContactId, normalImpulse, tangentImpulse);
            }
        }
        if (impact.IsValid) return;

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
                contact.Group, contact.ContactId,
                contacts.GetNormalImpulse(i),
                contacts.GetTangentImpulse(i));
        }
    }

    private static bool IsCorrectionOwner(CollisionPair2D pair, SolverContact2D contact)
    {
        // Translation is shared by equal-normal samples; retain every velocity
        // row, but apply the deepest directional correction only once.
        for (int g = 0; g < pair.Manifold.GroupCount; g++)
        {
            ref ContactGroup2D group = ref pair.Manifold.GetGroup(g);
            for (int point = 0; point < group.Count; point++)
            {
                ManifoldContact2D other = group[point];
                if (ResolveContactNormal(pair, group.Key, other.Normal) != contact.Normal) continue;
                if (other.Depth > contact.Depth
                    || other.Depth == contact.Depth
                    && (g < contact.GroupIndex || g == contact.GroupIndex && point < contact.PointIndex)) return false;
            }
        }
        return true;
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
        in ContactAnchor2D responseCenterB,
        int groupIndex)
    {
        SolverContactBuffer2D contacts = default;
        for (int i = 0; i < pair.Manifold.GetGroup(groupIndex).Count; i++)
        {
            if (TryCreateContact(
                    pair,
                    bodyA,
                    bodyB,
                    responseCenterA,
                    responseCenterB,
                    groupIndex, i,
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
        int groupIndex,
        int pointIndex,
        out SolverContact2D contact)
    {
        contact = default;
        ref ContactGroup2D surface = ref pair.Manifold.GetGroup(groupIndex);
        ManifoldContact2D manifoldContact = surface[pointIndex];
        ContactGroupKey group = surface.Key;
        Vector2d normal = ResolveContactNormal(pair, group, manifoldContact.Normal);
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
        if (pair.TryGetWarmStartImpulse(group, manifoldContact.ContactId, out ContactWarmStartImpulse cached))
        {
            cachedNormalImpulse = cached.NormalImpulse;
            cachedTangentImpulse = cached.TangentImpulse;
        }

        contact = new SolverContact2D(
            groupIndex, pointIndex,
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
            cachedTangentImpulse, group);
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
        out ContactNormalImpulseResult2D result,
        in ContactResponseSnapshot impact = default)
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
            out result, impact);
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

        Fixed64 weightedDepth = correctionDepth * PenetrationCorrectionPercent * contactShare;
        Vector2d correctionA, correctionB;
        if (CollisionPositionCorrection.TryGetCompactScale(
                weightedDepth, inverseMassA, inverseMassB, out Fixed64 scale))
        {
            Vector2d correction = contact.Normal * scale;
            correctionA = -correction * inverseMassA;
            correctionB = correction * inverseMassB;
        }
        else
        {
            correctionA = -contact.Normal * CollisionPositionCorrection.GetWeightedDistance(
                weightedDepth, inverseMassA, inverseMassA, inverseMassB);
            correctionB = contact.Normal * CollisionPositionCorrection.GetWeightedDistance(
                weightedDepth, inverseMassB, inverseMassA, inverseMassB);
        }
        // Prepare both endpoints before publishing either participant's correction.
        Vector2d positionA = contact.A.Body?.PrepareCollisionPositionCorrection(correctionA) ?? default;
        Vector2d positionB = contact.B.Body?.PrepareCollisionPositionCorrection(correctionB) ?? default;
        contact.A.Body?.ApplyCollisionPositionState(positionA);
        contact.B.Body?.ApplyCollisionPositionState(positionB);
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
        out Fixed64 accumulated,
        in ContactResponseSnapshot impact = default)
    {
        accumulated = default;
        if (contact.RelativeA.IsExact
            || contact.RelativeB.IsExact
            || !normalResult.HasRepresentableAccumulatedImpulse
            || !CanUseCompactAxisResponse(contact, contact.Tangent)
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
                out accumulated, impact);
        }

        Fixed64 impulseScalar = Fixed64.Zero;
        if (staticFrictionLimit > Fixed64.Zero || dynamicFrictionLimit > Fixed64.Zero)
        {
            Vector3d spatialTangent =
                ExactContactLever2D.ToSpatial(contact.Tangent);
            Fixed64 tangentVelocity = default;
            bool tangentVelocityResolved =
                ContactResponseArithmetic3D.TryGetRelativePointVelocity(
                    ExactContactLever2D.ToSpatial(
                        ResolveLinearVelocity(contact.A.Body)),
                    new Vector3d(
                        Fixed64.Zero,
                        -ResolveAngularVelocity(contact.A.Body),
                        Fixed64.Zero),
                    ExactContactLever2D.ToSpatial(contact.RelativeA.Vector),
                    ExactContactLever2D.ToSpatial(
                        ResolveLinearVelocity(contact.B.Body)),
                    new Vector3d(
                        Fixed64.Zero,
                        -ResolveAngularVelocity(contact.B.Body),
                        Fixed64.Zero),
                    ExactContactLever2D.ToSpatial(contact.RelativeB.Vector),
                    spatialTangent,
                    out Vector3d relativeVelocity)
                && ContactResponseArithmetic3D.TryDot(
                    relativeVelocity,
                    spatialTangent,
                    out tangentVelocity);
            bool denominatorResolved =
                ContactNormalImpulse2D.TryComputeDenominator(
                    contact.A.Body,
                    contact.RelativeA.Vector,
                    contact.B.Body,
                    contact.RelativeB.Vector,
                    contact.Tangent,
                    out Fixed64 denominator);
            if (!(tangentVelocityResolved & denominatorResolved))
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
                    out accumulated, impact);
            }
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
                    out accumulated, impact);
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
                out accumulated, impact);
        }
        accumulated = desiredAccumulated >= -staticFrictionLimit
            && desiredAccumulated <= staticFrictionLimit
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
                out accumulated, impact);
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
        out Fixed64 accumulated,
        in ContactResponseSnapshot impact = default)
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
                out ExactCoulombResponse3D response, impact))
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

    private static Vector2d ResolveContactNormal(CollisionPair2D pair,
        in ContactGroupKey group, Vector2d normal) =>
        ResolveContactNormal(normal, group.SourceIdA != 0 || group.SourceIdB != 0
            ? Vector2d.Zero : pair.ColliderB.Center - pair.ColliderA.Center);

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
            pair.Manifold.GetGroup(contact.GroupIndex)[contact.PointIndex];
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
        pair.RemoveWarmStartImpulse(contact.Group, contact.ContactId);
        GravitasLogger.Channel.Error(
            $"2D contact response is outside the representable velocity domain.");
    }
}
