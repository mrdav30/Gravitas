//=======================================================================
// GjkSimplexPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class GjkSimplexPolicy
{
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

        if (SameDirection(ab, ao))
        {
            direction = TripleCross(ab, ao, ab);
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
        Vector3d abc = Vector3d.Cross(ab, ac);

        Vector3d acPerp = Vector3d.Cross(abc, ac);
        if (SameDirection(acPerp, ao))
        {
            if (SameDirection(ac, ao))
            {
                simplex[1] = c;
                count = 2;
                direction = TripleCross(ac, ao, ac);
                if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                    direction = Perpendicular(ac);
                return false;
            }

            simplex[1] = b;
            count = 2;
            return UpdateLine(simplex, ref count, ref direction);
        }

        Vector3d abPerp = Vector3d.Cross(ab, abc);
        if (SameDirection(abPerp, ao))
        {
            simplex[1] = b;
            count = 2;
            return UpdateLine(simplex, ref count, ref direction);
        }

        if (SameDirection(abc, ao))
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
        Vector3d ao = -a;

        Vector3d abc = OrientFaceNormal(a, b, c, d);
        if (SameDirection(abc, ao))
        {
            simplex[0] = a;
            simplex[1] = b;
            simplex[2] = c;
            count = 3;
            direction = abc;
            return false;
        }

        Vector3d acd = OrientFaceNormal(a, c, d, b);
        if (SameDirection(acd, ao))
        {
            simplex[0] = a;
            simplex[1] = c;
            simplex[2] = d;
            count = 3;
            direction = acd;
            return false;
        }

        Vector3d adb = OrientFaceNormal(a, d, b, c);
        if (SameDirection(adb, ao))
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
        Vector3d.Dot(first, second) > Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d TripleCross(Vector3d first, Vector3d second, Vector3d third) =>
        Vector3d.Cross(Vector3d.Cross(first, second), third);

    private static Vector3d OrientFaceNormal(Vector3d a, Vector3d b, Vector3d c, Vector3d opposite)
    {
        Vector3d normal = Vector3d.Cross(b - a, c - a);
        return Vector3d.Dot(normal, opposite - a) > Fixed64.Zero ? -normal : normal;
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
