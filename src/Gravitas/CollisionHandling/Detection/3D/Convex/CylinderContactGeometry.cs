//=======================================================================
// CylinderContactGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>Derives the world-space tangent basis for a cylinder cap.</summary>
internal static class CylinderContactGeometry
{
    public static void GetCapBasis(LSCylinderCollider cylinder, out Vector3d tangentA, out Vector3d tangentB)
    {
        tangentA = (cylinder.Rotation * Vector3d.Right).Normalized;
        tangentB = (cylinder.Rotation * Vector3d.Forward).Normalized;
    }
}
