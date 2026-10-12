//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;

namespace Gravitas.CollisionHandling;

/// <content>Solves retained mixed groups with independent accumulated impulse rows.</content>
public static partial class CollisionResponseMixed
{
    internal static void ResolveManifold(CollisionPairMixed pair)
    {
        if (pair.Manifold.Count == 1)
        {
            Resolve(pair, pair.Contact, 0, 1, true, authoritativeNormal: true);
            return;
        }
        PrepareSolve(pair);
        int limit = pair.Manifold.Count > 1 ? pair.Context.Settings.DiscreteSolverIterations : 1;
        for (int iteration = 0; iteration < limit; iteration++)
            ResolveManifold(pair, iteration, limit, iteration == 0);
        pair.ResponseSnapshot = default;
    }

    internal static void PrepareSolve(CollisionPairMixed pair)
    {
        pair.SolveImpulses.Clear();
        ClearExactImpulses(pair.ExactSolveImpulses);
        SolidBody? body3D = pair.Collider3D.Body;
        SolidBody2D? body2D = pair.Collider2D.Body;
        Vector3d position3D = body3D?.Position3d ?? pair.Collider3D.Center;
        Vector3d position2D = body2D?.Agent.Transform.WorldPosition ?? MixedEmbedded2DGeometry.GetCenter3D(pair.Collider2D);
        ContactAnchor center2D = body2D == null
            ? ContactAnchor.FromWorldPoint(position2D)
            : new ContactAnchor(new Vector3d(body2D.Position.X, position2D.Y, body2D.Position.Y),
                FixedQuaternion.FromAxisAngle(Vector3d.Up, -body2D.Rotation),
                ExactContactLever2D.ToSpatial(body2D.LocalCenterOfMassOffset));
        // Every row keeps the incoming motion and lever frame, even after another row corrects position.
        pair.ResponseSnapshot = new ContactResponseSnapshot(position3D, position2D,
            body3D?.GetCenterOfMassAnchor() ?? ContactAnchor.FromWorldPoint(position3D), center2D,
            body3D == null ? Vector3d.Zero : ResolveLinearVelocity(body3D),
            body3D == null ? Vector3d.Zero : ResolveAngularVelocity(body3D),
            body2D == null ? Vector3d.Zero : ExactContactLever2D.ToSpatial(ResolveLinearVelocity(body2D)),
            new Vector3d(Fixed64.Zero, body2D == null ? Fixed64.Zero : -ResolveAngularVelocity(body2D), Fixed64.Zero),
            body2D == null ? Fixed64.Zero : ResolveAngularVelocity(body2D));
    }

    internal static bool HasPlanarResponseCoupling(CollisionPairMixed pair)
    {
        foreach (ManifoldContact row in pair.Manifold)
            if (row.Normal.ToVector2d() != Vector2d.Zero) return true;
        return false;
    }

    internal static void ResolveManifold(CollisionPairMixed pair, int iteration, int limit, bool correctPosition)
    {
        if (!pair.Manifold.HasContact || pair.Collider3D.IsTrigger || pair.Collider2D.IsTrigger) return;
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ref ContactGroup group = ref pair.Manifold.GetGroup(groupIndex);
            for (int point = 0; point < group.Count; point++)
                ResolveRow(pair, group.Key, group[point], iteration, limit,
                    correctPosition && IsCorrectionOwner(pair.Manifold, groupIndex, point),
                    groupIndex * ContactManifold.MaxContactsPerGroup + point);
        }
    }

    private static bool IsCorrectionOwner(ContactManifold manifold, int groupIndex, int pointIndex)
    {
        ManifoldContact row = manifold.GetGroup(groupIndex)[pointIndex];
        // Translation is shared by equal directions; contact levers remain independent velocity rows.
        for (int group = 0; group < manifold.GroupCount; group++)
        {
            ref ContactGroup otherGroup = ref manifold.GetGroup(group);
            for (int point = 0; point < otherGroup.Count; point++)
            {
                ManifoldContact other = otherGroup[point];
                if (other.Normal != row.Normal) continue;
                if (other.Depth > row.Depth || other.Depth == row.Depth
                    && (group < groupIndex || group == groupIndex && point < pointIndex)) return false;
            }
        }
        return true;
    }

    private static void ResolveRow(CollisionPairMixed pair, in ContactGroupKey key,
        in ManifoldContact row, int iteration, int limit, bool correctPosition, int stateIndex)
    {
        MixedContact contact = MixedContact.FromManifoldContact(row);
        SolidBody? first = pair.Collider3D.Body;
        SolidBody2D? second = pair.Collider2D.Body;
        Vector3d normal = row.Normal.Normalized;
        ContactResponseSnapshot impact = pair.ResponseSnapshot;
        ExactContactImpulseState? exact = GetExactImpulse(pair.ExactSolveImpulses, stateIndex);
        pair.SolveImpulses.TryGet(key, row.ContactId, out ContactWarmStartImpulse cached);
        ContactResponseSnapshot current = new(impact.PositionA, impact.PositionB, impact.CenterA, impact.CenterB,
                first == null ? Vector3d.Zero : ResolveLinearVelocity(first),
                first == null ? Vector3d.Zero : ResolveAngularVelocity(first),
                second == null ? Vector3d.Zero : ExactContactLever2D.ToSpatial(ResolveLinearVelocity(second)),
                new Vector3d(Fixed64.Zero, second == null ? Fixed64.Zero : -ResolveAngularVelocity(second), Fixed64.Zero),
                second == null ? Fixed64.Zero : ResolveAngularVelocity(second));
        if (!TrySolveNormalRow(pair.Collider3D, pair.Collider2D, row, impact, current, cached,
                ref exact, out ContactNormalImpulseResultMixed result, out ContactResponseSnapshot afterNormal)
            || !CanApplyNormalVelocityDeltas(first, second, result))
        {
            RejectManifoldResponse();
            return;
        }
        Fixed64 normalImpulse = cached.NormalImpulse + result.ImpulseScalar;
        if (correctPosition)
        {
            Fixed64 mass = GetConstrainedInverseMass(first, normal) + GetConstrainedPlanarInverseMass(second, normal);
            if (mass > Fixed64.Zero) ApplyPositionCorrection(first, second, normal, row.Depth, mass);
        }
        ApplyNormalVelocityDeltas(first, second, result);
        if (exact != null) SetExactImpulse(ref pair.ExactSolveImpulses, stateIndex, exact);
        // Publish a successful normal solve before friction, which can independently fail to materialize.
        pair.SolveImpulses.Set(key, row.ContactId, normal, normalImpulse,
            cached.TangentImpulse, cached.SecondaryTangentImpulse);
        if (result.HasRepresentableAppliedImpulse && result.HasRepresentableNormalVelocity && HasVelocityDelta(result))
            pair.Context.Diagnostics.EmitMixedResponseImpulse(pair, contact,
                normal * result.AppliedImpulseScalar, result.NormalVelocity, iteration, limit);
        if (!TrySolveManifoldFriction(pair.Collider3D, pair.Collider2D, contact, impact,
                afterNormal, normal, normalImpulse, cached, ref exact,
                out Fixed64 tangentImpulse, out Fixed64 secondaryImpulse,
                out ExactCoulombResponse3D friction))
        {
            RejectManifoldResponse();
            return;
        }
        ApplyVelocityDeltas(first, normal.ToVector2d() == Vector2d.Zero ? null : second,
            friction.FirstLinearVelocityDelta, friction.FirstAngularVelocityDelta,
            ExactContactLever2D.ToPlanar(friction.SecondLinearVelocityDelta),
            ExactContactLever2D.ToPlanarAngular(friction.SecondAngularVelocityDelta));
        if (exact != null) SetExactImpulse(ref pair.ExactSolveImpulses, stateIndex, exact);
        pair.SolveImpulses.Set(key, row.ContactId, normal, normalImpulse, tangentImpulse, secondaryImpulse);
    }

    private static bool TrySolveManifoldFriction(LSCollider collider3D, LSCollider2D collider2D,
        MixedContact contact, in ContactResponseSnapshot impact, in ContactResponseSnapshot current,
        Vector3d normal, Fixed64 normalImpulse, in ContactWarmStartImpulse cached,
        ref ExactContactImpulseState? exact, out Fixed64 tangentImpulse, out Fixed64 secondaryImpulse,
        out ExactCoulombResponse3D response)
    {
        tangentImpulse = secondaryImpulse = Fixed64.Zero;
        response = default;
        PhysicsMaterial.CombineFriction(contact.HasMaterialOverride ? contact.Material3D : collider3D.Material,
            contact.HasMaterialOverride ? contact.Material2D : collider2D.Material,
            out Fixed64 staticFriction, out Fixed64 dynamicFriction);
        if (staticFriction <= Fixed64.Zero && dynamicFriction <= Fixed64.Zero) return true;
        SolidBody? first = collider3D.Body;
        // Preserve the mixed embedding contract: vertical-only contact does not couple planar friction.
        SolidBody2D? second = normal.ToVector2d() == Vector2d.Zero ? null : collider2D.Body;
        Vector3d tangent = SolverContact.CreateTangent(normal);
        Vector3d secondary = Vector3d.Cross(normal, tangent).Normalized;
        ExactLever3D leverA = contact.Anchor3D.GetLeverFrom(impact.CenterA);
        ExactLever3D leverB = contact.Anchor2D.GetLeverFrom(impact.CenterB);
        Vector3d linearA = current.LinearA;
        Vector3d angularA = current.AngularA;
        Vector2d linearB = current.LinearB.ToVector2d();
        Fixed64 angularB = current.PlanarAngularB;
        if (!ExactContactResponseKernel.TryGetCoulombDiskResponse(normal, normalImpulse,
                ExactContactLever3D.CreateResponseOperand(first, linearA, angularA, leverA, -tangent),
                ExactContactLever2D.CreateResponseOperand(second, linearB, angularB, leverB, tangent),
                tangent, cached.TangentImpulse,
                ExactContactLever3D.CreateResponseOperand(first, linearA, angularA, leverA, -secondary),
                ExactContactLever2D.CreateResponseOperand(second, linearB, angularB, leverB, secondary),
                secondary, cached.SecondaryTangentImpulse, staticFriction, dynamicFriction,
                out response, ref exact)) return false;
        Vector2d deltaB = ExactContactLever2D.ToPlanar(response.SecondLinearVelocityDelta);
        Fixed64 angularDeltaB = ExactContactLever2D.ToPlanarAngular(response.SecondAngularVelocityDelta);
        if (!TryAddVelocityDeltas(current, response.FirstLinearVelocityDelta,
                response.FirstAngularVelocityDelta, ExactContactLever2D.ToSpatial(deltaB),
                angularDeltaB, out _)) return false;
        response.TryGetPrimaryAccumulatedImpulse(out tangentImpulse);
        response.TryGetSecondaryAccumulatedImpulse(out secondaryImpulse);
        return true;
    }

    // CCD retains its normal-only response policy and preflights the complete handoff before mutation.
    internal static bool TrySolveManifoldVelocity(LSCollider collider3D, LSCollider2D collider2D,
        ContactManifold manifold, in ContactResponseSnapshot impact, ref ContactWarmStartCache cache,
        ref ExactContactImpulseState?[]? exactImpulses, out ContactResponseSnapshot solved)
    {
        solved = impact;
        cache.Clear();
        ClearExactImpulses(exactImpulses);
        if (!manifold.HasContact) return false;
        bool changedVelocity = false;
        int iterations = manifold.Count == 1 ? 1 : collider3D.Context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int groupIndex = 0; groupIndex < manifold.GroupCount; groupIndex++)
            {
                ref ContactGroup group = ref manifold.GetGroup(groupIndex);
                for (int point = 0; point < group.Count; point++)
                {
                    ManifoldContact row = group[point];
                    cache.TryGet(group.Key, row.ContactId, out ContactWarmStartImpulse cached);
                    int stateIndex = groupIndex * ContactManifold.MaxContactsPerGroup + point;
                    ExactContactImpulseState? exact = GetExactImpulse(exactImpulses, stateIndex);
                    if (!TrySolveNormalRow(collider3D, collider2D, row, impact, solved, cached, ref exact,
                            out ContactNormalImpulseResultMixed response, out ContactResponseSnapshot next))
                        return false;
                    solved = next;
                    changedVelocity |= HasVelocityDelta(response);
                    if (exact != null) SetExactImpulse(ref exactImpulses, stateIndex, exact);
                    cache.Set(group.Key, row.ContactId, row.Normal, cached.NormalImpulse + response.ImpulseScalar,
                        Fixed64.Zero, Fixed64.Zero);
                }
            }
        }
        // A kinematic row with no movable effective response cannot admit a CCD handoff.
        return changedVelocity;
    }

    private static bool TrySolveNormalRow(LSCollider collider3D, LSCollider2D collider2D,
        in ManifoldContact row, in ContactResponseSnapshot impact, in ContactResponseSnapshot current,
        in ContactWarmStartImpulse cached, ref ExactContactImpulseState? exact,
        out ContactNormalImpulseResultMixed result, out ContactResponseSnapshot solved)
    {
        solved = default;
        bool compactA = row.AnchorA.TryGetOffsetFrom(impact.CenterA, out Vector3d relativeA);
        bool compactB = row.AnchorB.TryGetOffsetFrom(impact.CenterB, out Vector3d relativeB);
        return TryGetNormalImpulse(collider3D, collider2D, MixedContact.FromManifoldContact(row),
                collider3D.Body, collider2D.Body, row.Normal.Normalized, relativeA, relativeB.ToVector2d(),
                compactA, compactB, current.LinearA, current.AngularA, current.LinearB.ToVector2d(),
                current.PlanarAngularB, ref exact, out result, cached.NormalImpulse, impact,
                row.FeatureNamespaceB, detected: true)
            && TryAddVelocityDeltas(current, result.LinearVelocityDelta3D, result.AngularVelocityDelta3D,
                ExactContactLever2D.ToSpatial(result.LinearVelocityDelta2D),
                result.AngularVelocityDelta2D, out solved);
    }

    private static bool TryAddVelocityDeltas(in ContactResponseSnapshot current, Vector3d linearA,
        Vector3d angularA, Vector3d linearB, Fixed64 angularB, out ContactResponseSnapshot solved)
    {
        solved = default;
        if (!Vector3d.TryAdd(current.LinearA, linearA, out Vector3d nextLinearA)
            || !Vector3d.TryAdd(current.AngularA, angularA, out Vector3d nextAngularA)
            || !Vector3d.TryAdd(current.LinearB, linearB, out Vector3d nextLinearB)
            || !Fixed64.TryAdd(current.PlanarAngularB, angularB, out Fixed64 nextAngularB)) return false;
        solved = new ContactResponseSnapshot(current.PositionA, current.PositionB, current.CenterA,
            current.CenterB, nextLinearA, nextAngularA, nextLinearB,
            new Vector3d(Fixed64.Zero, -nextAngularB, Fixed64.Zero), nextAngularB);
        return true;
    }

    private static ExactContactImpulseState? GetExactImpulse(ExactContactImpulseState?[]? states, int index) =>
        states != null && index < states.Length ? states[index] : null;

    private static void SetExactImpulse(ref ExactContactImpulseState?[]? states, int index, ExactContactImpulseState state)
    {
        if (states == null || index >= states.Length)
            Array.Resize(ref states, Math.Max(index + 1, states?.Length * 2 ?? ContactManifold.MaxContactsPerGroup));
        states![index] = state;
    }

    private static void ClearExactImpulses(ExactContactImpulseState?[]? states)
    {
        if (states == null) return;
        foreach (ExactContactImpulseState? state in states) state?.Clear();
    }

    private static void RejectManifoldResponse() => GravitasLogger.Channel.Write(DiagnosticLevel.Error,
        "Mixed manifold response is outside the representable velocity domain.", nameof(CollisionResponseMixed));
}
