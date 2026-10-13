//=======================================================================
// GjkSimplexPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class GjkSimplexPolicy
{
    // Each vertex component <= 256 in magnitude bounds cubic crosses by 536,870,912; side dots stay exact.
    private const long ProductSafeVertexRaw = 256L << FixedMath.SHIFT_AMOUNT_I;

    public static void AddPoint(Span<Vector3d> simplex, ref int count, Vector3d point)
    {
        for (int i = Math.Min(count, 3); i > 0; i--)
            simplex[i] = simplex[i - 1];

        simplex[0] = point;
        if (count < 4)
            count++;
    }

    public static bool Update(Span<Vector3d> simplex, ref int count, ref Vector3d direction)
    {
        return count switch
        {
            2 => UpdateLine(simplex, ref count, ref direction),
            3 => UpdateTriangle(simplex, ref count, ref direction),
            _ => UpdateTetrahedron(simplex, ref count, ref direction)
        };
    }

    public static bool UpdateLine(Span<Vector3d> simplex, ref int count, ref Vector3d direction)
    {
        Vector3d a = simplex[0];
        Vector3d b = simplex[1];
        Vector3d ab = b - a;
        Vector3d ao = -a;
        bool exact = RequiresExactProducts(simplex, 2);

        if (SameDirection(ab, ao))
        {
            direction = exact
                ? Vector3d.GetNormalizedProjectionOnPlane(ao, ab)
                : TripleCross(ab, ao, ab);
            if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                direction = Perpendicular(ab);
            return false;
        }

        simplex[0] = a;
        count = 1;
        direction = ao;
        return false;
    }

    public static bool UpdateTriangle(Span<Vector3d> simplex, ref int count, ref Vector3d direction)
    {
        Vector3d a = simplex[0];
        Vector3d b = simplex[1];
        Vector3d c = simplex[2];
        Vector3d ab = b - a;
        Vector3d ac = c - a;
        Vector3d ao = -a;
        bool exact = RequiresExactProducts(simplex, 3);
        Vector3d abc = exact ? default : Vector3d.Cross(ab, ac);

        if (exact ? IsOutsideEdge(ab, ac, ao) : SameDirection(Vector3d.Cross(abc, ac), ao))
        {
            if (SameDirection(ac, ao))
            {
                simplex[1] = c;
                count = 2;
                direction = exact
                    ? Vector3d.GetNormalizedProjectionOnPlane(ao, ac)
                    : TripleCross(ac, ao, ac);
                if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                    direction = Perpendicular(ac);
                return false;
            }

            simplex[1] = b;
            count = 2;
            return UpdateLine(simplex, ref count, ref direction);
        }

        if (exact ? IsOutsideEdge(ac, ab, ao) : SameDirection(Vector3d.Cross(ab, abc), ao))
        {
            simplex[1] = b;
            count = 2;
            return UpdateLine(simplex, ref count, ref direction);
        }

        if (exact)
            abc = GetFaceDirection(ab, ac);
        if (exact ? WideGeometry.GetTripleProductSign(
                ao.X, ao.Y, ao.Z, ab.X, ab.Y, ab.Z, ac.X, ac.Y, ac.Z) > 0
            : SameDirection(abc, ao))
        {
            direction = abc;
            return false;
        }

        simplex[1] = c;
        simplex[2] = b;
        direction = -abc;
        return false;
    }

    public static bool UpdateTetrahedron(Span<Vector3d> simplex, ref int count, ref Vector3d direction)
    {
        Vector3d a = simplex[0];
        Vector3d b = simplex[1];
        Vector3d c = simplex[2];
        Vector3d d = simplex[3];
        bool exact = RequiresExactProducts(simplex, 4);

        if (IsOutsideFace(a, b, c, d, exact, out Vector3d abc))
        {
            simplex[0] = a;
            simplex[1] = b;
            simplex[2] = c;
            count = 3;
            direction = abc;
            return false;
        }

        if (IsOutsideFace(a, c, d, b, exact, out Vector3d acd))
        {
            simplex[0] = a;
            simplex[1] = c;
            simplex[2] = d;
            count = 3;
            direction = acd;
            return false;
        }

        if (IsOutsideFace(a, d, b, c, exact, out Vector3d adb))
        {
            simplex[0] = a;
            simplex[1] = d;
            simplex[2] = b;
            count = 3;
            direction = adb;
            return false;
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SameDirection(Vector3d first, Vector3d second) =>
        WideGeometry.GetDifferenceDotProduct3D(
            first.X, Fixed64.Zero, first.Y, Fixed64.Zero, first.Z, Fixed64.Zero,
            second.X, Fixed64.Zero, second.Y, Fixed64.Zero, second.Z, Fixed64.Zero).Sign > 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d TripleCross(Vector3d first, Vector3d second, Vector3d third) =>
        Vector3d.Cross(Vector3d.Cross(first, second), third);

    private static bool IsOutsideFace(
        Vector3d a, Vector3d b, Vector3d c, Vector3d opposite,
        bool exact, out Vector3d normal)
    {
        Vector3d ab = b - a;
        Vector3d ac = c - a;
        if (!exact)
        {
            normal = Vector3d.Cross(ab, ac);
            if (SameDirection(normal, opposite - a))
                normal = -normal;
            return SameDirection(normal, -a);
        }

        Vector3d oppositeOffset = opposite - a;
        Vector3d ao = -a;
        bool reversed = WideGeometry.GetTripleProductSign(
            oppositeOffset.X, oppositeOffset.Y, oppositeOffset.Z,
            ab.X, ab.Y, ab.Z, ac.X, ac.Y, ac.Z) > 0;
        int side = WideGeometry.GetTripleProductSign(
            ao.X, ao.Y, ao.Z, ab.X, ab.Y, ab.Z, ac.X, ac.Y, ac.Z);
        if ((reversed ? -side : side) <= 0)
        {
            normal = default;
            return false;
        }
        normal = GetFaceDirection(ab, ac);
        if (reversed)
            normal = -normal;
        return true;
    }

    public static bool HasSeparatingSupport(Vector3d point, Vector3d direction)
    {
        // This gate precedes simplex reduction: a new support can exceed its old bound.
        // 3 * 64 * 2^23 < 2^31 bounds the complete scalar dot, including partial sums.
        if (HasBoundedComponents(point, 64L << FixedMath.SHIFT_AMOUNT_I)
            && HasBoundedComponents(direction, (1L << 23) << FixedMath.SHIFT_AMOUNT_I))
        {
            return Vector3d.Dot(point, direction) < -Fixed64.Epsilon;
        }
        Signed192 dot = WideGeometry.GetDifferenceDotProduct3D(
            point.X, Fixed64.Zero, point.Y, Fixed64.Zero, point.Z, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero, direction.Z, Fixed64.Zero);
        return WideArithmetic.AddSigned192(
            dot, Fixed64.GetExactRawProduct(Fixed64.Epsilon, Fixed64.One)).Sign < 0;
    }

    private static bool RequiresExactProducts(Span<Vector3d> simplex, int count)
    {
        for (int i = 0; i < count; i++)
            if (!HasBoundedComponents(simplex[i], ProductSafeVertexRaw))
                return true;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasBoundedComponents(Vector3d value, long limit) =>
        value.X.m_rawValue >= -limit && value.X.m_rawValue <= limit
        && value.Y.m_rawValue >= -limit && value.Y.m_rawValue <= limit
        && value.Z.m_rawValue >= -limit && value.Z.m_rawValue <= limit;

    private static bool IsOutsideEdge(Vector3d firstEdge, Vector3d secondEdge, Vector3d ao)
    {
        // Both outward edge signs negate the plane-projection numerator. Test its dot directly:
        // |second|^2*(first.ao) - (first.second)*(second.ao); raw quartics fit Signed320.
        Signed192 q = WideGeometry.GetDifferenceDotProduct3D(
            secondEdge.X, Fixed64.Zero, secondEdge.Y, Fixed64.Zero, secondEdge.Z, Fixed64.Zero,
            secondEdge.X, Fixed64.Zero, secondEdge.Y, Fixed64.Zero, secondEdge.Z, Fixed64.Zero);
        Signed192 firstTowardOrigin = WideGeometry.GetDifferenceDotProduct3D(
            firstEdge.X, Fixed64.Zero, firstEdge.Y, Fixed64.Zero, firstEdge.Z, Fixed64.Zero,
            ao.X, Fixed64.Zero, ao.Y, Fixed64.Zero, ao.Z, Fixed64.Zero);
        Signed192 projection = WideGeometry.GetDifferenceDotProduct3D(
            firstEdge.X, Fixed64.Zero, firstEdge.Y, Fixed64.Zero, firstEdge.Z, Fixed64.Zero,
            secondEdge.X, Fixed64.Zero, secondEdge.Y, Fixed64.Zero, secondEdge.Z, Fixed64.Zero);
        Signed192 secondTowardOrigin = WideGeometry.GetDifferenceDotProduct3D(
            secondEdge.X, Fixed64.Zero, secondEdge.Y, Fixed64.Zero, secondEdge.Z, Fixed64.Zero,
            ao.X, Fixed64.Zero, ao.Y, Fixed64.Zero, ao.Z, Fixed64.Zero);
        return WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(q, firstTowardOrigin),
            WideArithmetic.MultiplySigned192(projection, secondTowardOrigin)).Sign < 0;
    }

    private static Vector3d GetFaceDirection(Vector3d ab, Vector3d ac)
    {
        WideGeometry.GetDifferenceCrossProduct3D(
            ab.X, Fixed64.Zero, ab.Y, Fixed64.Zero, ab.Z, Fixed64.Zero,
            ac.X, Fixed64.Zero, ac.Y, Fixed64.Zero, ac.Z, Fixed64.Zero,
            out Signed192 x, out Signed192 y, out Signed192 z);
        return WideNormalization.GetNormalized(x, y, z);
    }

    private static Vector3d Perpendicular(Vector3d vector)
    {
        Vector3d candidate = Vector3d.Cross(vector, Vector3d.Up);
        if (candidate.MagnitudeSquared > Fixed64.Epsilon)
            return candidate;

        candidate = Vector3d.Cross(vector, Vector3d.Right);
        return candidate.MagnitudeSquared > Fixed64.Epsilon ? candidate : Vector3d.Forward;
    }
}
