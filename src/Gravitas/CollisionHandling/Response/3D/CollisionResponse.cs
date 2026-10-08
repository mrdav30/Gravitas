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
/// <content>
/// Streams deterministic warm starts, independent normal rows and translation correction.
/// </content>
public static partial class CollisionResponse
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
    public static void CalculateImpulse(CollisionPair pair)
    {
        PrepareSolve(pair);
        int iterations = pair.Manifold.Count > 1 ? pair.Context.Settings.DiscreteSolverIterations : 1;
        for (int iteration = 0; iteration < iterations; iteration++)
            CalculateImpulse(pair, applyCachedImpulse: iteration == 0,
                applyPositionCorrection: iteration == 0);
        pair.ResponseSnapshot = default;
    }

    internal static void PrepareSolve(CollisionPair pair)
    {
        pair.ResponseSnapshot = default;
        // Also cover callers that author the public manifold without narrow phase.
        pair.RetainWarmStarts();
        if (TryCreateBodyPair(pair, out ResponseBody first, out ResponseBody second))
            pair.ResponseSnapshot = new ContactResponseSnapshot(first.Body, second.Body,
                ResolveLinearVelocity(first.Body), ResolveAngularVelocity(first.Body),
                ResolveLinearVelocity(second.Body), ResolveAngularVelocity(second.Body));
    }

    internal static void CalculateImpulse(
        CollisionPair pair,
        bool applyCachedImpulse,
        bool applyPositionCorrection)
    {
        if (!TryCreateBodyPair(pair, out ResponseBody bodyA, out ResponseBody bodyB))
            return;

        if (!pair.ResponseSnapshot.IsValid) PrepareSolve(pair);
        ref readonly ContactResponseSnapshot impact = ref pair.ResponseSnapshot;
        ContactAnchor responseCenterA = impact.CenterA;
        ContactAnchor responseCenterB = impact.CenterB;
        Vector3d responsePositionA = impact.PositionA;
        Vector3d responsePositionB = impact.PositionB;

        // Warm every admitted row before solving. Updating one cache must not
        // change which impulse another row uses for this warm-start pass.
        if (applyCachedImpulse || applyPositionCorrection)
        {
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
            {
                for (int point = 0; point < pair.Manifold.GetGroup(group).Count; point++)
                {
                    if (!TryCreateContact(pair, bodyA, bodyB, responseCenterA,
                            responseCenterB, group, point, out SolverContact contact))
                        continue;
                    if (applyPositionCorrection && IsCorrectionOwner(pair, contact))
                        ApplyPositionCorrection(contact, Fixed64.One);
                    if (applyCachedImpulse && !TryApplyCachedImpulse(pair, contact,
                            responsePositionA, responsePositionB))
                        ClearWarmStartImpulse(pair, contact);
                }
            }

        }

        Fixed64 threshold = pair.Context.Settings.RestitutionVelocityThreshold;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            for (int point = 0; point < pair.Manifold.GetGroup(group).Count; point++)
            {
                if (!TryCreateContact(pair, bodyA, bodyB, responseCenterA,
                        responseCenterB, group, point, out SolverContact contact))
                    continue;
                ContactNormalImpulseResult3D normalResult = default;
                bool resolved = !contact.RelativeA.IsExact && !contact.RelativeB.IsExact
                    && ContactNormalImpulse3D.TryCalculateAccumulatedDeltaWithImpact(
                        bodyA.Body, ResolveLinearVelocity(bodyA.Body), ResolveAngularVelocity(bodyA.Body),
                        contact.RelativeA.Vector, bodyB.Body, ResolveLinearVelocity(bodyB.Body),
                        ResolveAngularVelocity(bodyB.Body), contact.RelativeB.Vector,
                        contact.Normal, contact.Restitution, threshold, contact.CachedNormalImpulse,
                        impact, out normalResult);
                if (!resolved)
                    resolved = TryCalculateExactNormalResult(pair, contact, responsePositionA,
                        responsePositionB, threshold, Fixed64.One, out normalResult);
                if (!resolved || !TryApplyNormalImpulse(pair, contact, normalResult))
                {
                    RejectResponse(pair, contact);
                    continue;
                }
                Fixed64 normalImpulse = contact.CachedNormalImpulse + normalResult.ImpulseScalar;
                if (!TrySolveFrictionImpulse(pair, contact, responsePositionA, responsePositionB,
                        normalImpulse, out Fixed64 tangentImpulse, out Fixed64 secondaryTangentImpulse))
                {
                    RejectResponse(pair, contact);
                    continue;
                }
                pair.StoreWarmStartImpulse(pair.Manifold.GetGroup(group).Key, contact.ContactId,
                    contact.Normal, normalImpulse, tangentImpulse, secondaryTangentImpulse);
            }
        }
    }

    private static bool IsCorrectionOwner(CollisionPair pair, SolverContact contact)
    {
        // Position correction is translational: repeated samples of the same
        // direction must not multiply it. Velocity rows keep their own levers.
        // ponytail: quadratic scan over bounded group samples; index directions
        // only if measured large multi-surface workloads justify that scratch.
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup current = ref pair.Manifold.GetGroup(group);
            for (int point = 0; point < current.Count; point++)
            {
                ManifoldContact other = current[point];
                if (ResolveContactNormal(other.Normal, pair.ColliderB.Center - pair.ColliderA.Center) != contact.Normal) continue;
                if (other.Depth > contact.Depth) return false;
                if (other.Depth == contact.Depth && (group < contact.GroupIndex
                    || group == contact.GroupIndex && point < contact.PointIndex)) return false;
            }
        }
        return true;
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

    private static bool TryCreateContact(
        CollisionPair pair,
        ResponseBody bodyA,
        ResponseBody bodyB,
        in ContactAnchor responseCenterA,
        in ContactAnchor responseCenterB,
        int groupIndex,
        int pointIndex,
        out SolverContact contact)
    {
        contact = default;
        ref ContactGroup group = ref pair.Manifold.GetGroup(groupIndex);
        ManifoldContact manifoldContact = group[pointIndex];
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
        if (pair.TryGetWarmStartImpulse(group.Key, manifoldContact.ContactId, out ContactWarmStartImpulse cached))
        {
            if (IsWarmStartCompatible(cached.Normal, normal))
            {
                cachedNormalImpulse = cached.NormalImpulse;
                cachedTangentImpulse = cached.TangentImpulse;
                cachedSecondaryTangentImpulse = cached.SecondaryTangentImpulse;
            }
        }

        contact = new SolverContact(
            groupIndex,
            pointIndex,
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
            out result, pair.ResponseSnapshot);
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
            Vector3d fastTorqueAxis =
                Vector3d.Cross(relativeContactPoint, impulse);
            velocityDelta = Fixed3x3.TransformDirection(
                inverseInertia,
                fastTorqueAxis);
            return ContactResponseArithmetic3D
                    .PreservesNonzeroCrossProduct(
                        relativeContactPoint,
                        impulse,
                        fastTorqueAxis)
                && ContactResponseArithmetic3D
                    .PreservesNonzeroTransformDirection(
                        inverseInertia,
                        fastTorqueAxis,
                        velocityDelta);
        }

        if (!ContactResponseArithmetic3D.TryCross(
                relativeContactPoint,
                impulse,
                out Vector3d torqueAxis))
        {
            return false;
        }

        return ContactResponseArithmetic3D.TryTransformDirection(
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
            pair.Manifold.GetGroup(contact.GroupIndex)[contact.PointIndex];
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

    private static void RejectResponse(CollisionPair pair, SolverContact contact)
    {
        ClearWarmStartImpulse(pair, contact);
        GravitasLogger.Channel.Error($"Contact response is outside the representable velocity domain.");
    }

    private static void ClearWarmStartImpulse(
        CollisionPair pair,
        SolverContact contact) =>
        pair.RemoveWarmStartImpulse(pair.Manifold.GetGroup(contact.GroupIndex).Key, contact.ContactId);

    private static bool IsWarmStartCompatible(Vector3d cachedNormal, Vector3d normal) =>
        Vector3d.Dot(cachedNormal, normal) >= WarmStartNormalCompatibilityThreshold;

    private static Vector3d ResolveContactNormal(Vector3d normal, Vector3d fallbackDirection) =>
        normal.MagnitudeSquared > Fixed64.Epsilon ? normal
            : fallbackDirection.MagnitudeSquared > Fixed64.Epsilon ? fallbackDirection.Normalized
            : Vector3d.Zero;
}
