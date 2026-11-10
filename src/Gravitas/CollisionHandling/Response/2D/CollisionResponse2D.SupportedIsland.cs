//=======================================================================
// CollisionResponse2D.SupportedIsland.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <content>Seeds coupled normal loads in a two-interface planar support island.</content>
public static partial class CollisionResponse2D
{
    private const int MaxSupportRows = 2 * ContactManifold2D.MaxContactsPerGroup;

    internal static void CorrectSeededSupportPositions(CollisionPair2D first, CollisionPair2D second)
    {
        if (TryCorrectSeededSupportPositions(first, second))
            return;
        CorrectSeededSupportPosition(first);
        CorrectSeededSupportPosition(second);
    }

    internal static bool TryCorrectSeededSupportPositions(CollisionPair2D first, CollisionPair2D second)
    {
        if (!TryGetSupportedBodies(first, second, out SolidBody2D lower, out SolidBody2D upper)
            && !TryGetSupportedBodies(second, first, out lower, out upper))
            return false;

        Span<Vector3d> lowerAxes = stackalloc Vector3d[2];
        Span<Vector3d> upperAxes = stackalloc Vector3d[2];
        if (!(TryReadTranslationInterface(first, lower, upper, out lowerAxes[0], out upperAxes[0], out Fixed64 firstDepth)
            & TryReadTranslationInterface(second, lower, upper, out lowerAxes[1], out upperAxes[1], out Fixed64 secondDepth)))
            return false;
        if (firstDepth <= PenetrationSlop & secondDepth <= PenetrationSlop)
            return true;

        // Keep the signed slack of each interface. Correcting one can deepen the other.
        Span<Fixed64> depths = stackalloc Fixed64[] { PenetrationSlop - firstDepth, PenetrationSlop - secondDepth };
        Span<Fixed64> impulses = stackalloc Fixed64[2];
        if (!SmallContactNormalSolver.TrySolveTranslations(lowerAxes, upperAxes,
                lower.EffectiveInverseMass, upper.EffectiveInverseMass, depths, impulses)
            || !(TryPrepareSupportTranslation(lower, lowerAxes, impulses, out Vector2d lowerPosition)
                & TryPrepareSupportTranslation(upper, upperAxes, impulses, out Vector2d upperPosition)))
            return false;

        lower.ApplyCollisionPositionState(lowerPosition);
        upper.ApplyCollisionPositionState(upperPosition);
        return true;
    }

    private static bool TryReadTranslationInterface(CollisionPair2D pair, SolidBody2D lower, SolidBody2D upper,
        out Vector3d lowerAxis, out Vector3d upperAxis, out Fixed64 depth)
    {
        ref ContactGroup2D group = ref pair.Manifold.GetGroup(0);
        Vector2d normal = ResolveContactNormal(pair, group.Key, group[0].Normal);
        bool lowerIsA = pair.ColliderA.Body == lower;
        var axis = new Vector3d(normal.X, normal.Y, Fixed64.Zero);
        lowerAxis = lowerIsA ? -axis : axis;
        upperAxis = (lowerIsA ? pair.ColliderB.Body : pair.ColliderA.Body) == upper ? -lowerAxis : default;
        depth = group[0].Depth;
        bool compatible = !group[0].DepthIsClamped;
        for (int i = 1; i < group.Count; i++)
        {
            ManifoldContact2D row = group[i];
            compatible &= !row.DepthIsClamped & (ResolveContactNormal(pair, group.Key, row.Normal) == normal);
            depth = FixedMath.Max(depth, row.Depth);
        }
        return compatible;
    }

    private static bool TryPrepareSupportTranslation(SolidBody2D body, ReadOnlySpan<Vector3d> axes,
        ReadOnlySpan<Fixed64> impulses, out Vector2d position)
    {
        bool prepared = Vector3d.TryScaledLinearCombination(axes[0], impulses[0], axes[1], impulses[1],
            default, Fixed64.Zero, body.EffectiveInverseMass * PenetrationCorrectionPercent, out Vector3d delta);
        prepared &= Vector2d.TryAdd(body.Position, new Vector2d(delta.X, delta.Y), out position);
        return prepared && (position == body.Position || body.Collider.TryPrepareBodyPose(position, body.Rotation));
    }

    private static void CorrectSeededSupportPosition(CollisionPair2D pair)
    {
        ResponseBody2D a = ResponseBody2D.Create(pair.ColliderA), b = ResponseBody2D.Create(pair.ColliderB);
        SolverContactBuffer2D contacts = BuildContactBuffer(pair, a, b,
            a.Body?.GetCenterOfMassAnchor() ?? default, b.Body?.GetCenterOfMassAnchor() ?? default, 0);
        ApplyPositionCorrections(ref contacts, Fixed64.One / (Fixed64)Math.Max(1, contacts.Count));
    }

    internal static bool TrySeedSupportedIslandNormals(CollisionPair2D first, CollisionPair2D second)
    {
        if (!IsSupportPair(first) | !IsSupportPair(second))
            return false;
        if (!TryGetSupportedBodies(first, second, out SolidBody2D lower, out SolidBody2D upper)
            && !TryGetSupportedBodies(second, first, out lower, out upper))
            return false;

        int firstCount = first.Manifold.Count;
        int count = firstCount + second.Manifold.Count;
        Span<SupportNormal> rows = stackalloc SupportNormal[count];
        if (!(TryReadSupportRows(first, lower, rows[..firstCount])
            & TryReadSupportRows(second, lower, rows[firstCount..])))
            return false;

        Span<Fixed64> matrix = stackalloc Fixed64[MaxSupportRows * MaxSupportRows];
        Span<Fixed64> speeds = stackalloc Fixed64[MaxSupportRows];
        Span<Fixed64> impulses = stackalloc Fixed64[MaxSupportRows];
        Vector3d lowerMotion = ReadMotion(lower), upperMotion = ReadMotion(upper);
        for (int i = 0; i < count; i++)
        {
            ref SupportNormal row = ref rows[i];
            bool prepared = TryProjectSupportSpeed(row, lowerMotion, upperMotion, out speeds[i]);
            for (int j = 0; j <= i; j++)
            {
                bool terms = Vector3d.TryDot(rows[j].Lower, row.LowerResponse, out Fixed64 low)
                    & Vector3d.TryDot(rows[j].Upper, row.UpperResponse, out Fixed64 high);
                prepared &= terms & Fixed64.TryAdd(low, high, out Fixed64 coefficient);
                matrix[i * count + j] = matrix[j * count + i] = coefficient;
            }
            if (!prepared)
                return false;
        }

        if (!SmallContactNormalSolver.TrySolve(count, matrix, speeds, impulses)
            || !TryPrepareSupportVelocities(rows, impulses, lowerMotion, upperMotion,
                out Vector3d lowerDelta, out Vector3d upperDelta))
            return false;

        StoreSupportNormals(first, impulses[..firstCount]);
        StoreSupportNormals(second, impulses[firstCount..count]);
        ApplySupportDelta(lower, lowerDelta);
        ApplySupportDelta(upper, upperDelta);
        return true;
    }

    private static bool IsSupportPair(CollisionPair2D pair) => pair.CollisionType == CollisionType2D.Convex_Convex
        & pair.Manifold.GroupCount == 1 & !pair.HasRecovery;

    private static bool TryGetSupportedBodies(CollisionPair2D support, CollisionPair2D load,
        out SolidBody2D lower, out SolidBody2D upper)
    {
        lower = upper = null!;
        SolidBody2D? a = support.ColliderA.Body, b = support.ColliderB.Body;
        SolidBody2D? supported = IsFixedSupport(a) ? b : IsFixedSupport(b) ? a : null;
        if (!HasSupportMobility(supported))
            return false;

        // Both contacts belong to the same two-pair island and share the movable lower body.
        System.Diagnostics.Debug.Assert(load.ColliderA.Body == supported || load.ColliderB.Body == supported);
        SolidBody2D? loaded = load.ColliderA.Body == supported ? load.ColliderB.Body : load.ColliderA.Body;
        if (!HasSupportMobility(loaded) || loaded!.Mass <= supported!.Mass)
            return false;
        lower = supported;
        upper = loaded;
        return true;
    }

    private static bool IsFixedSupport(SolidBody2D? body) => body == null
        || (body.IsStatic & (body.LinearVelocity == Vector2d.Zero) & (body.AngularVelocity == Fixed64.Zero));

    private static bool HasSupportMobility(SolidBody2D? body) => body != null
        && (body.IsDynamic & body.CanTranslate & body.CanRotate & body.FreezeAxes == BodyFreezeAxes2D.None);

    private static bool TryReadSupportRows(CollisionPair2D pair, SolidBody2D lower, Span<SupportNormal> rows)
    {
        ResponseBody2D a = ResponseBody2D.Create(pair.ColliderA), b = ResponseBody2D.Create(pair.ColliderB);
        ContactAnchor2D centerA = a.Body?.GetCenterOfMassAnchor() ?? default;
        ContactAnchor2D centerB = b.Body?.GetCenterOfMassAnchor() ?? default;
        bool lowerIsA = a.Body == lower;
        for (int i = 0; i < rows.Length; i++)
        {
            bool read = TryCreateContact(pair, a, b, centerA, centerB, 0, i, out SolverContact2D contact);
            if (!(read & !contact.RelativeA.IsExact & !contact.RelativeB.IsExact & (contact.Restitution == Fixed64.Zero)))
                return false;

            bool prepared = TryReadSupportJacobian(contact.RelativeA.Vector, -contact.Normal, a, out Vector3d rowA, out Vector3d responseA)
                & TryReadSupportJacobian(contact.RelativeB.Vector, contact.Normal, b, out Vector3d rowB, out Vector3d responseB);
            if (!prepared)
                return false;
            rows[i] = lowerIsA ? new SupportNormal(rowA, rowB, responseA, responseB)
                : new SupportNormal(rowB, rowA, responseB, responseA);
        }
        return true;
    }

    private static bool TryReadSupportJacobian(Vector2d lever, Vector2d direction, ResponseBody2D body,
        out Vector3d row, out Vector3d response)
    {
        if (body.InverseMass == Fixed64.Zero)
        {
            row = response = default;
            return true;
        }

        bool valid = Fixed64.TrySubtractProducts(lever.X, direction.Y, lever.Y, direction.X, out Fixed64 torque);
        row = new Vector3d(direction.X, direction.Y, torque);
        valid &= Fixed64.TryMultiplyDivide(direction.X, body.InverseMass, Fixed64.One, out Fixed64 x)
            & Fixed64.TryMultiplyDivide(direction.Y, body.InverseMass, Fixed64.One, out Fixed64 y)
            & Fixed64.TryMultiplyDivide(torque, body.InverseMoment, Fixed64.One, out Fixed64 angular);
        response = new Vector3d(x, y, angular);
        return valid;
    }

    private static bool TryPrepareSupportVelocities(ReadOnlySpan<SupportNormal> rows, ReadOnlySpan<Fixed64> impulses,
        Vector3d lowerMotion, Vector3d upperMotion, out Vector3d lowerDelta, out Vector3d upperDelta)
    {
        lowerDelta = upperDelta = default;
        bool valid = true;
        for (int i = 0; i < rows.Length; i++)
        {
            valid &= TryAccumulateSupportDelta(lowerDelta, rows[i].LowerResponse, impulses[i], out lowerDelta)
                & TryAccumulateSupportDelta(upperDelta, rows[i].UpperResponse, impulses[i], out upperDelta);
        }

        valid &= TryPreparePublishedSupportMotion(lowerMotion, lowerDelta, out Vector3d lowerState)
            & TryPreparePublishedSupportMotion(upperMotion, upperDelta, out Vector3d upperState);
        Fixed64 tolerance = PhysicsEnvironment.DefaultMinSpeed;
        for (int i = 0; i < rows.Length; i++)
        {
            valid &= TryProjectSupportSpeed(rows[i], lowerState, upperState, out Fixed64 speed)
                & (speed >= -tolerance) & ((impulses[i] == Fixed64.Zero) | (speed <= tolerance));
        }
        return valid;
    }

    private static bool TryPreparePublishedSupportMotion(Vector3d motion, Vector3d delta, out Vector3d state)
    {
        bool fits = Vector3d.TryAdd(motion, delta, out state);
        // Applying a nonzero delta refreshes the corresponding body's motion deadband.
        var linear = new Vector2d(state.X, state.Y);
        if ((delta.X != Fixed64.Zero | delta.Y != Fixed64.Zero) && linear.Magnitude <= Fixed64.Epsilon)
            linear = Vector2d.Zero;
        Fixed64 angular = delta.Z != Fixed64.Zero && state.Z.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : state.Z;
        state = new Vector3d(linear.X, linear.Y, angular);
        return fits;
    }

    private static bool TryProjectSupportSpeed(in SupportNormal row, Vector3d lower, Vector3d upper, out Fixed64 speed)
    {
        bool valid = Vector3d.TryDot(row.Lower, lower, out Fixed64 first)
            & Vector3d.TryDot(row.Upper, upper, out Fixed64 second);
        return valid & Fixed64.TryAdd(first, second, out speed);
    }

    private static bool TryAccumulateSupportDelta(Vector3d sum, Vector3d response, Fixed64 impulse, out Vector3d delta) =>
        Vector3d.TryScaledLinearCombination(sum, Fixed64.One, response, impulse, default, Fixed64.Zero, Fixed64.One, out delta);

    private static Vector3d ReadMotion(SolidBody2D body) => new(body.LinearVelocity.X, body.LinearVelocity.Y, body.AngularVelocity);

    private static void ApplySupportDelta(SolidBody2D body, Vector3d delta)
    {
        body.ApplyCollisionLinearVelocityDelta(new Vector2d(delta.X, delta.Y));
        body.ApplyCollisionAngularVelocityDelta(delta.Z);
    }

    private static void StoreSupportNormals(CollisionPair2D pair, ReadOnlySpan<Fixed64> impulses)
    {
        pair.RetainWarmStart();
        ref ContactGroup2D group = ref pair.Manifold.GetGroup(0);
        // Friction starts cold on the seeded path. The existing row sweeps solve it against these normal loads.
        for (int i = 0; i < impulses.Length; i++)
            pair.StoreWarmStartImpulse(group.Key, group[i].ContactId, impulses[i], Fixed64.Zero);
    }

    private readonly struct SupportNormal
    {
        internal SupportNormal(Vector3d lower, Vector3d upper, Vector3d lowerResponse, Vector3d upperResponse)
        {
            Lower = lower;
            Upper = upper;
            LowerResponse = lowerResponse;
            UpperResponse = upperResponse;
        }

        internal readonly Vector3d Lower, Upper, LowerResponse, UpperResponse;
    }
}
