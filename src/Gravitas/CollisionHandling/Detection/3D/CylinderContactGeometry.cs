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
    public static bool IsAxisAligned(
        FixedQuaternion rotation,
        Vector3d localAxis,
        Vector3d direction) =>
        Vector3d.Dot(
            localAxis,
            rotation.Inverse().Rotate(direction)).Abs()
        >= Fixed64.FromFraction(63, 64);

    public static void GetCapBasis(LSCylinderCollider cylinder, out Vector3d tangentA, out Vector3d tangentB)
    {
        tangentA = (cylinder.Rotation * Vector3d.Right).Normalized;
        tangentB = (cylinder.Rotation * Vector3d.Forward).Normalized;
    }
}
