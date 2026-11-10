//=======================================================================
// CollisionResponse.SupportedIsland.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <content>Seeds the normal load of a two-body cuboid support island.</content>
public static partial class CollisionResponse
{
    internal static bool TrySeedSupportedIslandNormals(CollisionPair first, CollisionPair second, out bool correctedPositions)
    {
        correctedPositions = false;
        if (!TryGetSupportedBodies(first, second, out SolidBody lower, out SolidBody upper)
            && !TryGetSupportedBodies(second, first, out lower, out upper))
            return false;

        Span<IslandNormal> rows = stackalloc IslandNormal[SmallContactNormalSolver.MaxRows];
        int firstCount = first.Manifold.Count;
        int count = firstCount + second.Manifold.Count;
        System.Diagnostics.Debug.Assert(count <= rows.Length);
        if (!(TryReadIslandNormals(first, lower, upper, rows[..firstCount])
            & TryReadIslandNormals(second, lower, upper, rows[firstCount..count])))
            return false;

        Span<Fixed64> matrix = stackalloc Fixed64[SmallContactNormalSolver.MaxRows * SmallContactNormalSolver.MaxRows];
        Span<Fixed64> speeds = stackalloc Fixed64[SmallContactNormalSolver.MaxRows];
        Span<Fixed64> impulses = stackalloc Fixed64[SmallContactNormalSolver.MaxRows];
        var lowerMobility = new IslandMobility(lower);
        var upperMobility = new IslandMobility(upper);
        bool preparedSystem = TryPrepareIslandFriction(rows[..count], lowerMobility, upperMobility,
            out IslandMotion lowerFriction, out IslandMotion upperFriction);
        for (int i = 0; i < count; i++)
        {
            ref IslandNormal row = ref rows[i];
            bool prepared = row.Lower.TryDot(lowerFriction, out Fixed64 lowerSpeed)
                & row.Upper.TryDot(upperFriction, out Fixed64 upperSpeed)
                & lowerMobility.TryTransform(row.Lower, out row.LowerResponse)
                & upperMobility.TryTransform(row.Upper, out row.UpperResponse);
            prepared &= Fixed64.TryAdd(lowerSpeed, upperSpeed, out Fixed64 warmSpeed);
            preparedSystem &= prepared & Fixed64.TryAdd(row.Speed, warmSpeed, out speeds[i]);
            for (int j = 0; j <= i; j++)
            {
                bool terms = rows[j].Lower.TryDot(row.LowerResponse, out Fixed64 low)
                    & rows[j].Upper.TryDot(row.UpperResponse, out Fixed64 high);
                preparedSystem &= terms & Fixed64.TryAdd(low, high, out Fixed64 coefficient);
                matrix[i * count + j] = matrix[j * count + i] = coefficient;
            }
        }

        if (!preparedSystem)
            return false;
        if (!SmallContactNormalSolver.TrySolve(count, matrix[..(count * count)], speeds[..count], impulses[..count]))
            return false;
        bool preparedVelocities = TryPrepareIslandVelocity(lower, rows[..count], impulses[..count], lowerFriction,
                isLower: true, out IslandMotion lowerState)
            & TryPrepareIslandVelocity(upper, rows[..count], impulses[..count], upperFriction,
                isLower: false, out IslandMotion upperState);
        if (!preparedVelocities || !ArePreparedIslandNormalsFeasible(rows[..count], impulses[..count], lowerState, upperState))
            return false;

        correctedPositions = TryPrepareIslandPositions(rows[..firstCount], rows[firstCount..count], lower, upper,
            out Vector3d lowerPosition, out Vector3d upperPosition);

        // Commit only after both endpoints and every normal impulse were prepared.
        // The subsequent row sweep refines friction against the new normal loads.
        if (correctedPositions)
        {
            lower.ApplyCollisionPositionState(lowerPosition);
            upper.ApplyCollisionPositionState(upperPosition);
        }
        StoreIslandNormals(first, rows[..firstCount], speeds[..firstCount], impulses[..firstCount]);
        StoreIslandNormals(second, rows[firstCount..count], speeds[firstCount..count], impulses[firstCount..count]);
        lower.ApplyCollisionVelocityState(lowerState.Linear, lowerState.Angular);
        upper.ApplyCollisionVelocityState(upperState.Linear, upperState.Angular);
        return true;
    }

    private static bool TryGetSupportedBodies(CollisionPair support, CollisionPair load,
        out SolidBody lower, out SolidBody upper)
    {
        lower = null!;
        upper = null!;
        if (support.CollisionType != CollisionType.Cuboid_Cuboid | load.CollisionType != CollisionType.Cuboid_Cuboid
            | support.Manifold.GroupCount != 1 | load.Manifold.GroupCount != 1)
            return false;

        SolidBody? a = support.ColliderA.Body;
        SolidBody? b = support.ColliderB.Body;
        SolidBody? supported = IsFixedIslandSupport(a) ? b : IsFixedIslandSupport(b) ? a : null;
        if (supported == null || !HasUnconstrainedIslandMobility(supported))
            return false;
        // Two contacts in this connected island must share the only moving support endpoint.
        System.Diagnostics.Debug.Assert(load.ColliderA.Body == supported || load.ColliderB.Body == supported);
        SolidBody? loaded = load.ColliderA.Body == supported ? load.ColliderB.Body : load.ColliderA.Body;
        if (loaded == null || loaded.Mass <= supported.Mass || !HasUnconstrainedIslandMobility(loaded))
            return false;
        lower = supported;
        upper = loaded;
        return true;
    }

    private static bool IsFixedIslandSupport(SolidBody? body) => body == null || body.IsStatic && body.IsAtRest;

    private static bool HasUnconstrainedIslandMobility(SolidBody body) => body.IsDynamic
        && body.FreezeAxes == BodyFreezeAxes3D.None && body.CanTranslate && body.CanRotate;

    private static bool TryReadIslandNormals(CollisionPair pair, SolidBody lower, SolidBody upper, Span<IslandNormal> rows)
    {
        if (!pair.ResponseSnapshot.IsValid)
            return false;
        ResponseBody3D a = ResponseBody3D.Create(pair.ColliderA);
        ResponseBody3D b = ResponseBody3D.Create(pair.ColliderB);
        ref readonly ContactResponseSnapshot snapshot = ref pair.ResponseSnapshot;
        bool lowerIsA = a.Body == lower, upperIsA = a.Body == upper, upperIsB = b.Body == upper;
        for (int i = 0; i < rows.Length; i++)
        {
            bool contactPrepared = TryCreateContact(pair, a, b, snapshot.CenterA, snapshot.CenterB, 0, i,
                out SolverContact3D contact, requireMatchingTangentBasis: true);
            if (!(contactPrepared & !contact.RelativeA.IsExact & !contact.RelativeB.IsExact & (contact.Restitution == Fixed64.Zero)))
                return false;
            bool prepared = ContactNormalImpulse3D.TryComputeNormalVelocity(snapshot.LinearA, snapshot.AngularA,
                    contact.RelativeA.Vector, snapshot.LinearB, snapshot.AngularB,
                    contact.RelativeB.Vector, contact.Normal, out Fixed64 speed)
                & ContactResponseArithmetic3D.TryCross(contact.RelativeA.Vector, -contact.Normal, out Vector3d torqueA)
                & ContactResponseArithmetic3D.TryCross(contact.RelativeB.Vector, contact.Normal, out Vector3d torqueB);
            var first = new IslandMotion(-contact.Normal, torqueA);
            var second = new IslandMotion(contact.Normal, torqueB);
            if (!(prepared & TryReadIslandFriction(contact, out IslandMotion firstFriction, out IslandMotion secondFriction)))
                return false;
            rows[i] = new IslandNormal
            {
                Lower = lowerIsA ? first : second,
                Upper = upperIsA ? first : upperIsB ? second : default,
                LowerFriction = lowerIsA ? firstFriction : secondFriction,
                UpperFriction = upperIsA ? firstFriction : upperIsB ? secondFriction : default,
                Tangent = contact.CachedTangentImpulse,
                SecondaryTangent = contact.CachedSecondaryTangentImpulse,
                Speed = speed,
                Normal = contact.Normal,
                Depth = contact.Depth
            };
        }
        return true;
    }

    private static bool TryReadIslandFriction(SolverContact3D contact, out IslandMotion first, out IslandMotion second)
    {
        first = second = default;
        if (contact.CachedTangentImpulse == Fixed64.Zero && contact.CachedSecondaryTangentImpulse == Fixed64.Zero)
            return true;
        bool prepared = ContactResponseArithmetic3D.TryLinearCombination(contact.Tangent, contact.CachedTangentImpulse,
            contact.SecondaryTangent, contact.CachedSecondaryTangentImpulse, Vector3d.Zero, Fixed64.Zero, out Vector3d force);
        prepared &= Vector3d.TrySubtract(Vector3d.Zero, force, out Vector3d opposite);
        prepared &= ContactResponseArithmetic3D.TryCross(contact.RelativeA.Vector, opposite, out Vector3d firstTorque)
            & ContactResponseArithmetic3D.TryCross(contact.RelativeB.Vector, force, out Vector3d secondTorque);
        first = new IslandMotion(opposite, firstTorque);
        second = new IslandMotion(force, secondTorque);
        return prepared;
    }

    private static bool TryPrepareIslandFriction(ReadOnlySpan<IslandNormal> rows,
        IslandMobility lowerMobility, IslandMobility upperMobility, out IslandMotion lower, out IslandMotion upper)
    {
        lower = upper = default;
        IslandMotion lowerImpulse = default, upperImpulse = default;
        foreach (IslandNormal row in rows)
        {
            if (!(lowerImpulse.TryAdd(row.LowerFriction, out lowerImpulse)
                & upperImpulse.TryAdd(row.UpperFriction, out upperImpulse)))
                return false;
        }
        return lowerMobility.TryTransform(lowerImpulse, out lower) & upperMobility.TryTransform(upperImpulse, out upper);
    }

    private static bool TryPrepareIslandVelocity(SolidBody body, ReadOnlySpan<IslandNormal> rows,
        ReadOnlySpan<Fixed64> impulses, IslandMotion friction, bool isLower, out IslandMotion state)
    {
        state = default;
        Vector3d linear = friction.Linear, angular = friction.Angular;
        bool prepared = true;
        for (int i = 0; i < rows.Length; i++)
        {
            IslandMotion response = isLower ? rows[i].LowerResponse : rows[i].UpperResponse;
            bool scaled = ContactResponseArithmetic3D.TryScale(response.Linear, impulses[i], out Vector3d linearDelta)
                & ContactResponseArithmetic3D.TryScale(response.Angular, impulses[i], out Vector3d angularDelta);
            prepared &= scaled & Vector3d.TryAdd(linear, linearDelta, out linear)
                & Vector3d.TryAdd(angular, angularDelta, out angular);
        }
        prepared &= body.TryPrepareCollisionVelocityState(linear, angular, out linear, out angular);
        PhysicsEnvironment environment = body.Context.Environment;
        Fixed64 linearSpeed = linear.Magnitude, angularSpeed = angular.Magnitude;
        state = new IslandMotion(linearSpeed <= environment.MinSpeed ? Vector3d.Zero : linear,
            angularSpeed <= environment.MinSpeed ? Vector3d.Zero : angular);
        return prepared & (linearSpeed <= environment.MaxSpeed) & (angularSpeed <= environment.MaxSpeed);
    }

    private static bool ArePreparedIslandNormalsFeasible(ReadOnlySpan<IslandNormal> rows, ReadOnlySpan<Fixed64> impulses,
        IslandMotion lowerState, IslandMotion upperState)
    {
        // Materializing the body deltas adds rounding beyond the matrix solve.
        // Use the ordinary motion deadband, independent of a host's configured limit.
        Fixed64 tolerance = PhysicsEnvironment.DefaultMinSpeed;
        for (int i = 0; i < rows.Length; i++)
        {
            bool prepared = rows[i].Lower.TryGetPreparedSpeed(lowerState, out Fixed64 lowerSpeed)
                & rows[i].Upper.TryGetPreparedSpeed(upperState, out Fixed64 upperSpeed);
            prepared &= Fixed64.TryAdd(lowerSpeed, upperSpeed, out Fixed64 speed);
            if (!(prepared & (speed >= -tolerance) & ((impulses[i] == Fixed64.Zero) | (speed <= tolerance))))
                return false;
        }
        return true;
    }

    private static bool TryPrepareIslandPositions(ReadOnlySpan<IslandNormal> first, ReadOnlySpan<IslandNormal> second,
        SolidBody lower, SolidBody upper, out Vector3d lowerPosition, out Vector3d upperPosition)
    {
        lowerPosition = upperPosition = default;
        if (!(TryGetTranslationDepth(first, out Fixed64 firstDepth) & TryGetTranslationDepth(second, out Fixed64 secondDepth)))
            return false;

        // Project one depth per interface, without using angular inertia for a translation-only correction.
        Span<Vector3d> lowerAxes = stackalloc Vector3d[] { first[0].Lower.Linear, second[0].Lower.Linear };
        Span<Vector3d> upperAxes = stackalloc Vector3d[] { first[0].Upper.Linear, second[0].Upper.Linear };
        Span<Fixed64> depths = stackalloc Fixed64[] { PenetrationSlop - firstDepth, PenetrationSlop - secondDepth };
        Span<Fixed64> impulses = stackalloc Fixed64[2];
        impulses.Clear();
        bool prepared = SmallContactNormalSolver.TrySolveTranslations(lowerAxes, upperAxes,
            lower.EffectiveInverseMass, upper.EffectiveInverseMass, depths, impulses);
        return prepared & TryPrepareIslandTranslation(lower, lowerAxes, impulses, out lowerPosition)
            & TryPrepareIslandTranslation(upper, upperAxes, impulses, out upperPosition);
    }

    private static bool TryGetTranslationDepth(ReadOnlySpan<IslandNormal> rows, out Fixed64 depth)
    {
        depth = Fixed64.Zero;
        foreach (IslandNormal row in rows)
        {
            if (row.Normal != rows[0].Normal)
                return false;
            depth = FixedMath.Max(depth, row.Depth);
        }
        return true;
    }

    private static bool TryPrepareIslandTranslation(SolidBody body, ReadOnlySpan<Vector3d> axes,
        ReadOnlySpan<Fixed64> impulses, out Vector3d position)
    {
        position = default;
        Fixed64 inverseMass = PenetrationCorrectionPercent * body.EffectiveInverseMass;
        bool prepared = ContactResponseArithmetic3D.TryLinearCombination(axes[0], impulses[0], axes[1], impulses[1],
            Vector3d.Zero, Fixed64.Zero, out Vector3d translation);
        prepared &= ContactResponseArithmetic3D.TryScale(translation, inverseMass, out translation);
        return prepared & (inverseMass > Fixed64.Zero) & Vector3d.TryAdd(body.Position3d, translation, out position);
    }

    private static void StoreIslandNormals(CollisionPair pair, ReadOnlySpan<IslandNormal> rows,
        ReadOnlySpan<Fixed64> speeds, ReadOnlySpan<Fixed64> impulses)
    {
        ref ContactGroup group = ref pair.Manifold.GetGroup(0);
        var diagnostics = pair.Context.Diagnostics;
        for (int i = 0; i < rows.Length; i++)
        {
            pair.StoreWarmStartImpulse(group.Key, group[i].ContactId, rows[i].Normal, impulses[i],
                rows[i].Tangent, rows[i].SecondaryTangent);
            if (diagnostics.Enabled && impulses[i] != Fixed64.Zero)
                diagnostics.EmitResponseImpulse(pair, rows[i].Normal * impulses[i], speeds[i], group[i]);
        }
    }

    private struct IslandNormal
    {
        internal IslandMotion Lower, Upper, LowerResponse, UpperResponse, LowerFriction, UpperFriction;
        internal Vector3d Normal;
        internal Fixed64 Speed, Depth, Tangent, SecondaryTangent;
    }

    private readonly struct IslandMotion
    {
        internal IslandMotion(Vector3d linear, Vector3d angular)
        {
            Linear = linear;
            Angular = angular;
        }

        internal readonly Vector3d Linear, Angular;

        internal bool TryAdd(IslandMotion other, out IslandMotion result)
        {
            if (!(Vector3d.TryAdd(Linear, other.Linear, out Vector3d linear)
                & Vector3d.TryAdd(Angular, other.Angular, out Vector3d angular)))
            {
                result = default;
                return false;
            }
            result = new IslandMotion(linear, angular);
            return true;
        }

        internal bool TryDot(IslandMotion other, out Fixed64 result)
        {
            result = default;
            return ContactResponseArithmetic3D.TryDot(Linear, other.Linear, out Fixed64 linear)
                & ContactResponseArithmetic3D.TryDot(Angular, other.Angular, out Fixed64 angular)
                & Fixed64.TryAdd(linear, angular, out result);
        }

        internal bool TryGetPreparedSpeed(IslandMotion other, out Fixed64 result)
        {
            result = default;
            // These are already-materialized velocities. Rounding a sub-LSB term
            // to zero here cannot amplify it through a subsequent impulse scale.
            return Vector3d.TryDot(Linear, other.Linear, out Fixed64 linear)
                & Vector3d.TryDot(Angular, other.Angular, out Fixed64 angular)
                & Fixed64.TryAdd(linear, angular, out result);
        }
    }

    private readonly struct IslandMobility
    {
        private readonly Fixed64 _inverseMass;
        private readonly Fixed3x3 _inverseInertia;

        internal IslandMobility(SolidBody body)
        {
            _inverseMass = body.EffectiveInverseMass;
            _inverseInertia = body.GetConstrainedInverseInertiaTensor();
        }

        internal bool TryTransform(IslandMotion jacobian, out IslandMotion response)
        {
            bool prepared = ContactResponseArithmetic3D.TryScale(jacobian.Linear, _inverseMass, out Vector3d linear)
                & ContactResponseArithmetic3D.TryTransformDirection(_inverseInertia, jacobian.Angular, out Vector3d angular);
            response = new IslandMotion(linear, angular);
            return prepared;
        }
    }
}
