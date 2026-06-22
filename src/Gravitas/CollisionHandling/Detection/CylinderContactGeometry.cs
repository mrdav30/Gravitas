//=======================================================================
// CylinderContactGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class CylinderContactGeometry
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsAxisAligned(Vector3d axis, Vector3d direction) =>
        Vector3d.Dot(axis, direction).Abs() >= Fixed64.FromFraction(63, 64);

    public static void GetCapBasis(LSCylinderCollider cylinder, out Vector3d tangentA, out Vector3d tangentB)
    {
        Vector3d axis = cylinder.LineDirection;
        Vector3d reference = Vector3d.Dot(axis, Vector3d.Up).Abs() > Fixed64.FromFraction(63, 64)
            ? Vector3d.Right
            : Vector3d.Up;

        tangentA = Vector3d.Cross(axis, reference);
        tangentA = tangentA.MagnitudeSquared <= Fixed64.Epsilon
            ? Vector3d.Forward
            : tangentA.Normalized;
        tangentB = Vector3d.Cross(axis, tangentA).Normalized;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d GetCapCenter(LSCylinderCollider cylinder, Vector3d direction)
    {
        Fixed64 sign = Vector3d.Dot(cylinder.LineDirection, direction) >= Fixed64.Zero
            ? Fixed64.One
            : -Fixed64.One;
        return cylinder.Center + cylinder.LineDirection * (cylinder.HalfHeight * sign);
    }
}
